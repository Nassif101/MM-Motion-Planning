"""medium_horizon_node against a mock robot (no Unity, wall time; plan Task 11).

The test publishes what the node reads while B5 tracks a goal: TF map -> base_footprint, /joint_states (the six arm
joints at carry), /odom and B5's /wholebody_mpc/cycle at 20 Hz, and B5's route on /wholebody_mpc/route (latched). It
runs in its own ROS domain (domain_coordinator, as ament's isolated tests): the mock robot must never reach a live stack.
"""
import json
import os
import threading
import time
import unittest

import domain_coordinator
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped, TransformStamped
from mobile_manipulator_interfaces.msg import MediumCycle, MediumPlan, MpcCycle, WholeBodyRoute
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
DRAGGABLES = "/test/draggables"
_DOMAIN = domain_coordinator.domain_id()
os.environ["ROS_DOMAIN_ID"] = str(_DOMAIN.__enter__())
assert os.environ["ROS_DOMAIN_ID"] != "0", "the test must not share a live stack's domain"


def _carry():
    from ament_index_python.packages import get_package_share_directory
    with open(os.path.join(get_package_share_directory("mobile_manipulator_control"), "config",
                           "qualified_payload.json")) as stream:
        return json.load(stream)["poses_rad"]["carry"]


CARRY = _carry()


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package="mobile_manipulator_medium", executable="medium_horizon_node", name="medium_horizon",
        output="screen", parameters=[{"draggables_topic": DRAGGABLES, "rate_hz": 5.0}])
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class MockRobot:
    """The robot at rest at the origin facing +x, at carry, with B5 tracking (its cycle messages)."""

    def __init__(self, node):
        self.node = node
        self.tf = TransformBroadcaster(node)
        self.joints = node.create_publisher(JointState, "/joint_states", 10)
        self.odom = node.create_publisher(Odometry, "/odom", 10)
        self.b5 = node.create_publisher(MpcCycle, "/wholebody_mpc/cycle", 10)
        self.timer = node.create_timer(0.05, self.publish)

    def publish(self):
        now = self.node.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "map"
        t.child_frame_id = "base_footprint"
        t.transform.rotation.w = 1.0
        self.tf.sendTransform(t)
        j = JointState()
        j.header.stamp = now
        j.name = ARM
        j.position = list(CARRY)
        j.velocity = [0.0] * 6
        self.joints.publish(j)
        o = Odometry()
        o.header.stamp = now
        self.odom.publish(o)
        c = MpcCycle()
        c.header.stamp = now
        self.b5.publish(c)


class TestMediumHorizonNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("medium_test")
        cls.robot = MockRobot(cls.node)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.route_pub = cls.node.create_publisher(WholeBodyRoute, "/wholebody_mpc/route", latched)
        cls.drag_pub = cls.node.create_publisher(MarkerArray, DRAGGABLES, 10)
        cls.plans, cls.cycles = [], []
        cls.lock = threading.Lock()
        cls.node.create_subscription(MediumPlan, "/medium/plan", lambda m: cls._keep(cls.plans, m), 10)
        cls.node.create_subscription(MediumCycle, "/medium/cycle", lambda m: cls._keep(cls.cycles, m), 10)
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spin = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _keep(cls, store, msg):
        with cls.lock:
            store.append((time.monotonic(), msg))

    def announce(self, route_id, goal_x):
        m = WholeBodyRoute()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.route_id = route_id
        m.path.header = m.header
        for x in (0.0, goal_x):
            p = PoseStamped()
            p.header = m.header
            p.pose.position.x = x
            p.pose.orientation.w = 1.0
            m.path.poses.append(p)
        m.goal = Pose2D(x=goal_x, y=0.0, theta=0.0)
        self.route_pub.publish(m)
        return time.monotonic()

    def wait_for(self, store, predicate, since, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.lock:
                hits = [m for t, m in store if t >= since and predicate(m)]
            if hits:
                return hits[0]
            time.sleep(0.02)
        return None

    def test_1_plans_on_announced_route(self):
        time.sleep(1.0)  # the node is up and has state
        since = self.announce(1, 5.0)
        plan = self.wait_for(self.plans, lambda m: m.route_id == 1, since, 1.5)
        self.assertIsNotNone(plan, "no plan for route 1")
        self.assertGreaterEqual(len(plan.samples), 2)
        self.assertEqual(plan.header.frame_id, "map")
        self.assertAlmostEqual(plan.samples[0].x, 0.0, delta=0.05)

    def test_2_new_route_id_triggers_full_multistart(self):
        # A new goal (route 2): a full multi-start; a same-goal update (route 3) keeps the incumbent and the seeds go
        # on in turn (the Task 9 ruling).
        self.announce(1, 5.0)
        time.sleep(1.0)
        since = self.announce(2, 4.0)
        cycle = self.wait_for(self.cycles, lambda m: m.full_multistart, since, 1.5)
        self.assertIsNotNone(cycle, "no full multi-start after a new goal")
        self.assertIsNotNone(self.wait_for(self.plans, lambda m: m.route_id == 2, since, 1.5))
        time.sleep(0.5)
        since = self.announce(3, 4.0)
        plan = self.wait_for(self.plans, lambda m: m.route_id == 3, since, 1.5)
        self.assertIsNotNone(plan)
        with self.lock:
            full = [m for t, m in self.cycles if t >= since and m.plan_id == plan.plan_id and m.full_multistart]
        self.assertEqual(full, [], "a same-goal route update is not a full multi-start")

    def test_3_draggable_move_triggers_full_multistart(self):
        self.drag(3.0)
        time.sleep(1.0)
        with self.lock:
            last = self.cycles[-1][1].plan_id if self.cycles else 0
        since = time.monotonic()
        self.drag(3.3)  # 0.3 m: more than 0.05 m
        cycle = self.wait_for(self.cycles, lambda m: m.full_multistart and m.plan_id > last, since, 1.5)
        self.assertIsNotNone(cycle, "no full multi-start after the draggable moved")

    def drag(self, x):
        marker = Marker()
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.header.frame_id = "map"
        marker.ns = "box"
        marker.id = 1
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = 2.0
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.5
        self.drag_pub.publish(MarkerArray(markers=[marker]))
