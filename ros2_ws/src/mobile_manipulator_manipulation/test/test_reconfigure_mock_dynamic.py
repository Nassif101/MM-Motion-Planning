"""ReconfigurePanel in footprint_mode:=dynamic (B4) on ros2_control mock hardware (no Unity).

Launches the mock stack at an open-space pose, move_group with the known scene, the server
in dynamic mode and dynamic_footprint_node, all on wall time. The test publishes a zero /odom
so the base reads as stopped, and a latched /global_costmap/costmap (after test_a) for the
server's hull check. Tests run in name order.
"""
import math
import threading
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PolygonStamped
from launch.launch_description_sources import PythonLaunchDescriptionSource
from mobile_manipulator_interfaces.action import ReconfigurePanel
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
BASE = (12.0, 0.0, math.pi)
# 6 x 6 m costmap at 0.05 m centred on the base.
RESOLUTION, SIZE, ORIGIN = 0.05, 120, (9.0, -3.0)
Result = ReconfigurePanel.Result


@pytest.mark.launch_test
def generate_test_description():
    moveit = get_package_share_directory("mobile_manipulator_moveit_config")
    manipulation = get_package_share_directory("mobile_manipulator_manipulation")
    navigation = get_package_share_directory("mobile_manipulator_navigation")
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{moveit}/launch/mock_stack.launch.py"),
            launch_arguments={"base_pose": ",".join(str(v) for v in BASE)}.items()),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{manipulation}/launch/manipulation.launch.py"),
            launch_arguments={"use_sim_time": "false", "footprint_mode": "dynamic"}.items()),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{navigation}/launch/dynamic_footprint.launch.py"),
            launch_arguments={"use_sim_time": "false", "stats_file": ""}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestReconfigureMockDynamic(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("reconfigure_mock_dynamic_test")
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.node)
        threading.Thread(target=cls.executor.spin, daemon=True).start()
        cls.client = ActionClient(cls.node, ReconfigurePanel, "/reconfigure_panel")
        cls.joints = {}
        cls.footprint = None
        odom = cls.node.create_publisher(Odometry, "/odom", 10)

        def publish_odom():
            while rclpy.ok():
                message = Odometry()
                message.header.stamp = cls.node.get_clock().now().to_msg()
                odom.publish(message)
                time.sleep(0.02)
        threading.Thread(target=publish_odom, daemon=True).start()
        cls.node.create_subscription(JointState, "/joint_states",
                                     lambda m: cls.joints.update(zip(m.name, m.position)), 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.node.create_subscription(PolygonStamped, "/dynamic_footprint/footprint",
                                     lambda m: setattr(cls, "footprint", m), latched)
        cls.costmap = cls.node.create_publisher(OccupancyGrid, "/global_costmap/costmap", latched)
        assert cls.client.wait_for_server(timeout_sec=120), "/reconfigure_panel did not start"
        scene = cls.node.create_client(GetPlanningScene, "/get_planning_scene")
        deadline = time.time() + 120
        while time.time() < deadline:
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            if scene.wait_for_service(timeout_sec=1.0):
                future = scene.call_async(request)
                while not future.done() and time.time() < deadline:
                    time.sleep(0.02)
                if future.done() and future.result().scene.robot_state.attached_collision_objects:
                    break
            time.sleep(1.0)
        else:
            raise AssertionError("the panel was never attached")
        time.sleep(1.0)  # the server needs 0.5 s of stopped /odom before it accepts a goal

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        rclpy.shutdown()

    def publish_costmap(self, lethal=()):
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.header.stamp = self.node.get_clock().now().to_msg()
        grid.info.resolution = RESOLUTION
        grid.info.width = grid.info.height = SIZE
        grid.info.origin.position.x, grid.info.origin.position.y = ORIGIN
        grid.info.origin.orientation.w = 1.0
        data = [0] * (SIZE * SIZE)
        for x, y in lethal:
            col = int(math.floor((x - ORIGIN[0]) / RESOLUTION))
            row = int(math.floor((y - ORIGIN[1]) / RESOLUTION))
            data[row * SIZE + col] = 100
        grid.data = data
        self.costmap.publish(grid)
        time.sleep(1.0)

    def arm(self):
        return [self.joints.get(name, math.nan) for name in ARM]

    def send(self, named):
        goal = ReconfigurePanel.Goal()
        goal.target_type = ReconfigurePanel.Goal.NAMED_STATE
        goal.named_state = named
        goal.footprint_profile = "ignored_in_dynamic_mode"
        goal.planning_time_s = 5.0
        sent = self.client.send_goal_async(goal)
        deadline = time.time() + 10.0
        while not sent.done() and time.time() < deadline:
            time.sleep(0.02)
        handle = sent.result()
        self.assertTrue(handle is not None and handle.accepted)
        result_future = handle.get_result_async()
        deadline = time.time() + 60.0
        while not result_future.done() and time.time() < deadline:
            time.sleep(0.02)
        self.assertTrue(result_future.done(), "no result within 60 s")
        return result_future.result().result

    def assertUnchanged(self, before):
        for actual, expected in zip(self.arm(), before):
            self.assertAlmostEqual(actual, expected, delta=1e-3)

    def footprint_half_width(self, timeout=5.0):
        deadline = time.time() + timeout
        while self.footprint is None and time.time() < deadline:
            time.sleep(0.05)
        self.assertIsNotNone(self.footprint, "no /dynamic_footprint/footprint")
        return max(abs(p.y) for p in self.footprint.polygon.points)

    def test_a_no_costmap_fails_closed(self):
        before = self.arm()
        result = self.send("vertical_carry")
        self.assertEqual(result.error_code, Result.HULL_IN_COLLISION, result.message)
        self.assertIn("costmap", result.message)
        self.assertUnchanged(before)

    def test_b_reconfiguration_follows_hull(self):
        self.publish_costmap()
        result = self.send("vertical_carry")
        self.assertEqual(result.error_code, Result.SUCCESS, result.message)
        self.assertEqual(result.applied_footprint_profile, "dynamic")
        self.assertFalse(result.profile_violated)
        for clearance in (result.planned_hull_clearance_m, result.measured_hull_clearance_m):
            self.assertTrue(math.isfinite(clearance) and clearance > 0.0, clearance)
        # vertical_carry hull: +/-0.365 m, +0.02 m padding.
        deadline = time.time() + 5.0
        while self.footprint_half_width() > 0.386 and time.time() < deadline:
            time.sleep(0.1)
        self.assertLessEqual(self.footprint_half_width(), 0.386)
        publishers = [info.node_name for info in self.node.get_publishers_info_by_topic("/global_costmap/footprint")]
        self.assertEqual(publishers, ["dynamic_footprint_node"])

    def test_c_hull_in_collision_does_not_move(self):
        # Inside the home hull, outside the vertical_carry hull (base at yaw pi: base y = -map y).
        block = [(12.1 + dx, -0.5 + dy) for dx in (0.0, 0.05) for dy in (0.0, 0.05)]
        self.publish_costmap(block)
        before = self.arm()
        result = self.send("home")
        self.assertEqual(result.error_code, Result.HULL_IN_COLLISION, result.message)
        self.assertEqual(result.planned_hull_clearance_m, 0.0)
        self.assertUnchanged(before)

    def test_d_restore_home(self):
        self.publish_costmap()
        result = self.send("home")
        self.assertEqual(result.error_code, Result.SUCCESS, result.message)
