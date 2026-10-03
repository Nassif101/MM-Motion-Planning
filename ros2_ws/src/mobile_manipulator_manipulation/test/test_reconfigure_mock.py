"""ReconfigurePanel end to end on ros2_control mock hardware (no Unity).

Launches the mock stack at an open-space pose, move_group with the known scene, and the
server on wall time. The test publishes a zero /odom so the base reads as stopped.
"""
import math
import subprocess
import threading
import time
import unittest
import json

import launch
import launch_ros
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from geometry_msgs.msg import Polygon, PolygonStamped
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from mobile_manipulator_interfaces.action import ReconfigurePanel
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
POSES = {"home": [0.0] * 6, "vertical_carry": [math.pi / 2, 0, 0, 0, math.pi / 2, 0]}
TOLERANCE_M = 0.01
TOLERANCE_RAD = 0.01
Result = ReconfigurePanel.Result
Feedback = ReconfigurePanel.Feedback


@pytest.mark.launch_test
def generate_test_description():
    moveit = get_package_share_directory("mobile_manipulator_moveit_config")
    manipulation = get_package_share_directory("mobile_manipulator_manipulation")
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{moveit}/launch/mock_stack.launch.py"),
            launch_arguments={"base_pose": "12.0,0.0,3.141593"}.items()),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{manipulation}/launch/manipulation.launch.py"),
            launch_arguments={"use_sim_time": "false"}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestReconfigureMock(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("reconfigure_mock_test")
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.node)
        threading.Thread(target=cls.executor.spin, daemon=True).start()
        cls.client = ActionClient(cls.node, ReconfigurePanel, "/reconfigure_panel")
        cls.speed = 0.0
        cls.joints = {}
        cls.footprints = []
        odom = cls.node.create_publisher(Odometry, "/odom", 10)

        # A plain thread rather than an rclpy timer: the timer starved while the test
        # thread polled action futures, and the server then saw a moving base.
        def publish_odom():
            while rclpy.ok():
                message = Odometry()
                message.header.stamp = cls.node.get_clock().now().to_msg()
                message.twist.twist.linear.x = cls.speed
                odom.publish(message)
                time.sleep(0.02)  # 50 Hz, as Unity publishes /odom

        threading.Thread(target=publish_odom, daemon=True).start()
        cls.node.create_subscription(JointState, "/joint_states",
                                     lambda m: cls.joints.update(zip(m.name, m.position)), 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic in ("/global_costmap/footprint", "/local_costmap/footprint"):
            cls.node.create_subscription(Polygon, topic,
                                         lambda m, t=topic: cls.footprints.append((t, m)), latched)
        cls.zones = {}
        for topic in ("/collision_monitor/stop_zone_in", "/collision_monitor/slowdown_zone_in"):
            cls.node.create_subscription(PolygonStamped, topic,
                                         lambda m, t=topic: cls.zones.update({t: m}), latched)
        assert cls.client.wait_for_server(timeout_sec=120), "/reconfigure_panel did not start"
        scene = cls.node.create_client(GetPlanningScene, "/get_planning_scene")
        deadline = time.time() + 120
        while time.time() < deadline:
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            if scene.wait_for_service(timeout_sec=1.0):
                response = cls.call(scene.call_async(request), 5.0)
                if response and response.scene.robot_state.attached_collision_objects:
                    break
            time.sleep(1.0)
        else:
            raise AssertionError("the panel was never attached")

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        rclpy.shutdown()

    @staticmethod
    def call(future, timeout):
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def arm(self):
        return [self.joints.get(name, math.nan) for name in ARM]

    def send(self, profile, named=None, panel=None, planning_time=5.0, cancel_at=None):
        goal = ReconfigurePanel.Goal()
        goal.footprint_profile = profile
        goal.planning_time_s = planning_time
        if named is not None:
            goal.target_type = ReconfigurePanel.Goal.NAMED_STATE
            goal.named_state = named
        else:
            goal.target_type = ReconfigurePanel.Goal.PANEL_POSE
            goal.panel_pose = panel["pose"]
            # vertical_carry leaves 0.02 m per side: a 1.2 m panel tilted 0.01 rad moves its
            # edge 0.006 m, so these tolerances keep every goal inside the profile.
            goal.position_tolerance.x = goal.position_tolerance.y = goal.position_tolerance.z = TOLERANCE_M
            goal.orientation_tolerance.x = goal.orientation_tolerance.y = goal.orientation_tolerance.z = TOLERANCE_RAD
        phases = []
        sent = self.client.send_goal_async(goal, feedback_callback=lambda m: phases.append(m.feedback.phase))
        handle = self.call(sent, 10.0)
        self.assertIsNotNone(handle)
        self.assertTrue(handle.accepted)
        # Cancel from this thread once the phase is seen: feedback can arrive before the
        # goal handle, and an exception in a callback would stop the executor.
        result_future = handle.get_result_async()
        deadline = time.time() + 60.0
        canceled = False
        while not result_future.done() and time.time() < deadline:
            if cancel_at is not None and not canceled and cancel_at in phases:
                handle.cancel_goal_async()
                canceled = True
            time.sleep(0.005)
        result = result_future.result() if result_future.done() else None
        self.assertIsNotNone(result, "no result within 60 s")
        return result.result

    def go(self, state):
        result = self.send(state, named=state)
        self.assertEqual(result.error_code, Result.SUCCESS, result.message)
        for actual, expected in zip(self.arm(), POSES[state]):
            self.assertAlmostEqual(actual, expected, delta=0.04)

    def assertUnchanged(self, before):
        for actual, expected in zip(self.arm(), before):
            self.assertAlmostEqual(actual, expected, delta=1e-3)

    def zone(self, topic):
        message = self.zones[topic]
        self.assertEqual(message.header.frame_id, "base_footprint")
        xs = sorted({round(p.x, 3) for p in message.polygon.points})
        ys = sorted({round(p.y, 3) for p in message.polygon.points})
        return xs, ys

    def test_monitor_zones_follow_the_profile(self):
        self.go("home")
        # Stop zone = profile + 0.05 m, slowdown zone = profile + 0.30 m (nav2_navigation.yaml).
        self.assertEqual(self.zone("/collision_monitor/stop_zone_in"), ([-0.75, 0.59], [-0.67, 0.67]))
        self.assertEqual(self.zone("/collision_monitor/slowdown_zone_in"), ([-1.0, 0.84], [-0.92, 0.92]))
        result = self.send("vertical_carry", named="vertical_carry")
        self.assertEqual(result.error_code, Result.SUCCESS, result.message)
        self.assertEqual(self.zone("/collision_monitor/stop_zone_in"), ([-0.75, 0.59], [-0.435, 0.435]))
        self.assertEqual(self.zone("/collision_monitor/slowdown_zone_in"), ([-1.0, 0.84], [-0.685, 0.685]))
        self.go("home")

    # Follow-up 2c: a client that sends its next goal as soon as a result arrives (as the
    # mission task does) must not be rejected as busy.
    def test_back_to_back_goals_are_accepted(self):
        for state in ("home", "vertical_carry", "home", "vertical_carry", "home"):
            result = self.send(state, named=state)
            self.assertEqual(result.error_code, Result.SUCCESS, result.message)

    def test_named_vertical_carry_succeeds(self):
        self.go("home")
        self.footprints.clear()
        result = self.send("vertical_carry", named="vertical_carry")
        self.assertEqual(result.error_code, Result.SUCCESS, result.message)
        topics = {topic for topic, _ in self.footprints}
        self.assertEqual(topics, {"/global_costmap/footprint", "/local_costmap/footprint"})
        xs = sorted({round(p.x, 3) for _, m in self.footprints for p in m.points})
        ys = sorted({round(p.y, 3) for _, m in self.footprints for p in m.points})
        self.assertEqual((xs, ys), ([-0.7, 0.54], [-0.385, 0.385]))
        self.assertGreaterEqual(result.measured_containment_margin_m, 0.0)
        self.assertGreater(result.trajectory_duration_s, 0.0)
        self.assertGreater(result.joint_path_length_rad, 0.0)
        # Open space: the nearest known obstacle is metres away (the raised floor, which
        # only bounds panel ground clearance, is not counted).
        self.assertGreater(result.min_planned_clearance_m, 1.0)
        self.assertEqual(result.applied_footprint_profile, "vertical_carry")
        self.go("home")

    def test_panel_pose_goal_succeeds(self):
        self.go("home")
        printed = json.loads(subprocess.run(
            ["ros2", "run", "mobile_manipulator_navigation", "panel_pose", "--pose", "vertical_carry"],
            check=True, capture_output=True, text=True).stdout)
        from geometry_msgs.msg import PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = "base_footprint"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = printed["xyz"]
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = printed["quaternion_xyzw"]
        result = self.send("vertical_carry", panel={"pose": pose})
        self.assertEqual(result.error_code, Result.SUCCESS, result.message)
        reached = result.reached_panel_pose
        for actual, target in zip((reached.position.x, reached.position.y, reached.position.z), printed["xyz"]):
            self.assertLessEqual(abs(actual - target), TOLERANCE_M + 0.01)
        q = printed["quaternion_xyzw"]
        dot = abs(reached.orientation.x * q[0] + reached.orientation.y * q[1] +
                  reached.orientation.z * q[2] + reached.orientation.w * q[3])
        self.assertLessEqual(2 * math.acos(min(1.0, dot)), TOLERANCE_RAD * math.sqrt(3) + 0.02)
        self.go("home")

    def test_profile_too_small_does_not_move(self):
        self.go("home")
        before = self.arm()
        self.footprints.clear()
        result = self.send("vertical_carry", named="level_extension")
        self.assertEqual(result.error_code, Result.PROFILE_TOO_SMALL, result.message)
        self.assertLess(result.planned_containment_margin_m, 0.0)
        self.assertUnchanged(before)
        self.assertEqual(self.footprints, [])

    def test_unknown_profile(self):
        before = self.arm()
        result = self.send("no_such_profile", named="home")
        self.assertEqual(result.error_code, Result.UNKNOWN_PROFILE, result.message)
        self.assertUnchanged(before)

    def test_base_not_stopped(self):
        before = self.arm()
        type(self).speed = 0.1
        time.sleep(1.0)
        try:
            result = self.send("vertical_carry", named="vertical_carry")
        finally:
            type(self).speed = 0.0
            time.sleep(1.0)
        self.assertEqual(result.error_code, Result.BASE_NOT_STOPPED, result.message)
        self.assertUnchanged(before)

    def test_arm_not_active(self):
        before = self.arm()
        subprocess.run(["ros2", "control", "set_controller_state", "arm_controller", "inactive"], check=True)
        try:
            result = self.send("vertical_carry", named="vertical_carry")
        finally:
            subprocess.run(["ros2", "control", "set_controller_state", "arm_controller", "active"], check=True)
        self.assertEqual(result.error_code, Result.ARM_NOT_ACTIVE, result.message)
        self.assertUnchanged(before)

    def test_cancel_during_planning(self):
        self.go("home")
        before = self.arm()
        self.footprints.clear()
        result = self.send("vertical_carry", named="vertical_carry", cancel_at=Feedback.PLANNING)
        self.assertEqual(result.error_code, Result.CANCELED, result.message)
        self.assertUnchanged(before)
        self.assertEqual(self.footprints, [])

    def test_cancel_during_execution(self):
        self.go("home")
        self.footprints.clear()
        result = self.send("vertical_carry", named="vertical_carry", cancel_at=Feedback.EXECUTING)
        self.assertEqual(result.error_code, Result.CANCELED, result.message)
        self.assertEqual(result.applied_footprint_profile, "home")
        self.assertEqual(self.footprints, [])
        # Stopped between home and vertical carry, which the home profile still contains:
        # the cancel must halt the arm, not let the trajectory run to its end.
        self.assertEqual(len(result.reached_joint_positions), 6)
        self.assertLess(result.reached_joint_positions[0], math.pi / 2 - 0.1)
        self.assertFalse(result.profile_violated)
        self.go("home")

    # Review Focus 3 (final review I3): move_group still discovered but not answering.
    def test_zy_planning_deadline_with_stalled_move_group(self):
        self.go("home")
        subprocess.run(["pkill", "-STOP", "-f", "moveit_ros_move_group/move_group"], check=False)
        try:
            start = time.time()
            result = self.send("vertical_carry", named="vertical_carry", planning_time=1.0)
            elapsed = time.time() - start
        finally:
            subprocess.run(["pkill", "-CONT", "-f", "moveit_ros_move_group/move_group"], check=False)
        self.assertEqual(result.error_code, Result.PLANNING_FAILED, result.message)
        self.assertLess(elapsed, 1.0 + 5.0 + 2.0)  # planning_time_s + 5 s deadline + slack

    # Review Focus 3; runs last (alphabetical) because it stops move_group.
    def test_zz_planning_deadline_without_move_group(self):
        subprocess.run(["pkill", "-f", "moveit_ros_move_group/move_group"], check=False)
        time.sleep(2.0)
        start = time.time()
        result = self.send("vertical_carry", named="vertical_carry", planning_time=1.0)
        self.assertEqual(result.error_code, Result.PLANNING_FAILED, result.message)
        self.assertLess(time.time() - start, 7.0)
