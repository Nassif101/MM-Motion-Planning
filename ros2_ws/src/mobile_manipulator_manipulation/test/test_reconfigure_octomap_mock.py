"""ReconfigurePanel with the lidar Octomap scene source, on mock hardware (no Unity).

The test publishes a synthetic filtered-lidar wall (base_footprint frame) through the space
the vertical-carry panel occupies but clear of the home pose. While the wall is seen,
MoveIt must refuse vertical carry; once it is gone, the server's clear-and-refill at the
next goal must let the same goal succeed.
"""
import math
import threading
import time
import unittest

import launch
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from mobile_manipulator_interfaces.action import ReconfigurePanel
from moveit_msgs.srv import GetStateValidity
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import PointCloud2, PointField

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
POSES = {"home": [0.0] * 6, "vertical_carry": [math.pi / 2, 0, 0, 0, math.pi / 2, 0]}
Result = ReconfigurePanel.Result


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
            launch_arguments={"use_sim_time": "false", "scene_source": "octomap"}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


def wall_cloud(stamp):
    """Points on the vertical-carry panel plane (y = 0.225 m), below the home panel (z 1.545 m)."""
    xs, zs = np.arange(-0.6, 0.5, 0.03), np.arange(0.8, 1.45, 0.03)
    points = np.array([(x, 0.225, z) for x in xs for z in zs], dtype=np.float32)
    cloud = PointCloud2()
    cloud.header.frame_id = "base_footprint"
    cloud.header.stamp = stamp
    cloud.height, cloud.width = 1, len(points)
    cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate("xyz")]
    cloud.point_step, cloud.row_step = 12, 12 * len(points)
    cloud.is_dense = True
    cloud.data = points.tobytes()
    return cloud


class TestReconfigureOctomap(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("reconfigure_octomap_test")
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.node)
        threading.Thread(target=cls.executor.spin, daemon=True).start()
        cls.client = ActionClient(cls.node, ReconfigurePanel, "/reconfigure_panel")
        cls.validity = cls.node.create_client(GetStateValidity, "/check_state_validity")
        cls.wall = False
        odom = cls.node.create_publisher(Odometry, "/odom", 10)
        lidar = cls.node.create_publisher(PointCloud2, "/livox/points_filtered", 10)

        def publish():
            tick = 0
            while rclpy.ok():
                message = Odometry()
                message.header.stamp = cls.node.get_clock().now().to_msg()
                odom.publish(message)
                if cls.wall and tick % 5 == 0:  # 10 Hz cloud
                    lidar.publish(wall_cloud(message.header.stamp))
                tick += 1
                time.sleep(0.02)  # 50 Hz /odom, as Unity publishes it

        threading.Thread(target=publish, daemon=True).start()
        assert cls.client.wait_for_server(timeout_sec=120), "/reconfigure_panel did not start"
        assert cls.validity.wait_for_service(timeout_sec=60)
        time.sleep(5.0)  # the scene loader attaches the panel

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        rclpy.shutdown()

    @staticmethod
    def wait(future, timeout):
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def valid(self, pose):
        request = GetStateValidity.Request()
        request.group_name = "arm"
        request.robot_state.is_diff = True
        request.robot_state.joint_state.name = ARM
        request.robot_state.joint_state.position = POSES[pose]
        response = self.wait(self.validity.call_async(request), 10.0)
        self.assertIsNotNone(response)
        return response.valid

    def send(self, state):
        goal = ReconfigurePanel.Goal()
        goal.target_type = ReconfigurePanel.Goal.NAMED_STATE
        goal.named_state = state
        goal.footprint_profile = state
        goal.planning_time_s = 3.0
        handle = self.wait(self.client.send_goal_async(goal), 10.0)
        self.assertTrue(handle is not None and handle.accepted)
        result = self.wait(handle.get_result_async(), 90.0)
        self.assertIsNotNone(result)
        return result.result

    def test_octomap_wall_blocks_goal(self):
        self.assertEqual(self.send("home").error_code, Result.SUCCESS)
        type(self).wall = True
        time.sleep(3.0)
        self.assertTrue(self.valid("home"))
        self.assertFalse(self.valid("vertical_carry"))
        blocked = self.send("vertical_carry")
        self.assertIn(blocked.error_code, (Result.PLANNING_FAILED, Result.NO_IK), blocked.message)
        type(self).wall = False
        # Let the last wall cloud land first: one captured before the obstacle vanished but
        # inserted after the server's clear would stay (misses never clear voxels).
        time.sleep(1.0)
        result = self.send("vertical_carry")
        self.assertEqual(result.error_code, Result.SUCCESS, result.message)
        self.assertEqual(self.send("home").error_code, Result.SUCCESS)
