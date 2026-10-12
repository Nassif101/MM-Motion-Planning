"""wholebody_mpc_node as the FollowPath server against a mock plant (no Unity, wall time).

The test is the plant: it integrates cmd_vel_wholebody as a unicycle, moves the arm along the
streamed /arm_controller/joint_trajectory (linear between points, as a JTC would track it),
and publishes TF map -> base_footprint, /joint_states (the six arm joints plus a non-finite
wheel joint, which the node must ignore) and /odom at 50 Hz. The plant starts in open space
at (12, 0, pi), the open_space scenario's start. A second node instance in namespace
`scripted` loads the narrow_gate_mission references; it never gets a goal. Each test resets
the plant and leaves no goal active. The test runs in its own ROS domain (from
domain_coordinator, as ament's isolated tests): the mock plant publishes /joint_states, TF and
arm trajectories, which must never reach a live stack in the same container.
"""
import json
import math
import os
import tempfile
import threading
import time
import unittest

import domain_coordinator
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from mobile_manipulator_interfaces.msg import MpcCycle, WholeBodyPlan
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from trajectory_msgs.msg import JointTrajectory

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
HOME = [0.0] * 6
START = (12.0, 0.0, math.pi)
HORIZON, DT = 20, 0.1  # wholebody_mpc.yaml
# Held for the module's lifetime; the launched nodes inherit the environment.
_DOMAIN = domain_coordinator.domain_id()
os.environ["ROS_DOMAIN_ID"] = str(_DOMAIN.__enter__())
STATS_FILE = os.path.join(tempfile.mkdtemp(prefix="wholebody_mpc_test_"), "mpc_stats.json")
Result = FollowPath.Result


@pytest.mark.launch_test
def generate_test_description():
    mpc = launch_ros.actions.Node(
        package="mobile_manipulator_mpc", executable="wholebody_mpc_node", name="wholebody_mpc",
        output="screen", parameters=[{"stats_file": STATS_FILE}])
    scripted = launch_ros.actions.Node(
        package="mobile_manipulator_mpc", executable="wholebody_mpc_node", name="wholebody_mpc",
        namespace="scripted", output="screen",
        parameters=[{"scenario": "narrow_gate_mission", "references": "scripted"}],
        remappings=[("/arm_controller/joint_trajectory", "/scripted/arm_controller/joint_trajectory")])
    return launch.LaunchDescription([mpc, scripted, launch_testing.actions.ReadyToTest()]), \
        {"mpc": mpc, "scripted": scripted}


class Plant:
    """Base and arm of the robot, driven by the node's outputs."""

    RATE_HZ = 50.0
    CMD_TIMEOUT_S = 0.5

    def __init__(self, node):
        self.node = node
        self.lock = threading.Lock()
        self.pose = list(START)
        self.joints = list(HOME)
        self.velocity = (0.0, 0.0)
        self.cmd = (0.0, 0.0, -1.0)  # v, omega, receive time
        self.trajectory = None  # receive time, joints at receipt, [(t, positions)]
        self.joint_names = list(ARM)  # published arm joints (a test may drop some)
        self.publish_joints = True
        self.stuck = False  # the base ignores its command (wedged)
        self.cmd_log, self.trajectories, self.plans, self.cycles = [], [], [], []
        self.joint_pub = node.create_publisher(JointState, "/joint_states", 50)
        self.odom_pub = node.create_publisher(Odometry, "/odom", 20)
        self.tf = TransformBroadcaster(node)
        node.create_subscription(Twist, "/cmd_vel_wholebody", self.on_cmd, 50)
        node.create_subscription(JointTrajectory, "/arm_controller/joint_trajectory", self.on_trajectory, 50)
        node.create_subscription(WholeBodyPlan, "/wholebody_mpc/plan", lambda m: self.log(self.plans, m), 50)
        node.create_subscription(MpcCycle, "/wholebody_mpc/cycle", lambda m: self.log(self.cycles, m), 50)
        threading.Thread(target=self.run, daemon=True).start()

    def log(self, store, message):
        with self.lock:
            store.append((time.monotonic(), message))

    def on_cmd(self, m):
        with self.lock:
            self.cmd = (m.linear.x, m.angular.z, time.monotonic())
            self.cmd_log.append((time.monotonic(), m.linear.x, m.angular.z))

    def on_trajectory(self, m):
        with self.lock:
            now = time.monotonic()
            self.trajectories.append((now, m))
            order = [m.joint_names.index(name) for name in ARM]
            points = [(p.time_from_start.sec + 1e-9 * p.time_from_start.nanosec,
                       [p.positions[i] for i in order]) for p in m.points]
            self.trajectory = (now, list(self.joints), points)

    def reset(self, pose=START, joints=HOME):
        with self.lock:
            self.pose, self.joints = list(pose), list(joints)
            self.velocity, self.cmd, self.trajectory = (0.0, 0.0), (0.0, 0.0, -1.0), None
            self.joint_names, self.publish_joints, self.stuck = list(ARM), True, False

    def clear_logs(self):
        with self.lock:
            self.cmd_log, self.trajectories, self.plans, self.cycles = [], [], [], []

    def state(self):
        with self.lock:
            return list(self.pose), list(self.joints), self.velocity

    def arm_at(self, now):
        start_time, start, points = self.trajectory
        t = now - start_time
        before_t, before = 0.0, start
        for point_t, positions in points:
            if t <= point_t:
                a = (t - before_t) / (point_t - before_t) if point_t > before_t else 1.0
                return [b + a * (p - b) for b, p in zip(before, positions)]
            before_t, before = point_t, positions
        return list(before)

    def run(self):
        dt = 1.0 / self.RATE_HZ
        while rclpy.ok():
            with self.lock:
                now = time.monotonic()
                v, w, received = self.cmd
                if now - received > self.CMD_TIMEOUT_S or self.stuck:
                    v, w = 0.0, 0.0
                x, y, yaw = self.pose
                self.pose = [x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, yaw + w * dt]
                self.velocity = (v, w)
                if self.trajectory is not None:
                    self.joints = self.arm_at(now)
                pose, joints = list(self.pose), list(self.joints)
                names, publish_joints = list(self.joint_names), self.publish_joints
            stamp = self.node.get_clock().now().to_msg()
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = "map"
            transform.child_frame_id = "base_footprint"
            transform.transform.translation.x, transform.transform.translation.y = pose[0], pose[1]
            transform.transform.rotation.z = math.sin(pose[2] / 2)
            transform.transform.rotation.w = math.cos(pose[2] / 2)
            self.tf.sendTransform(transform)
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"
            odom.twist.twist.linear.x, odom.twist.twist.angular.z = v, w
            self.odom_pub.publish(odom)
            if publish_joints:
                state = JointState()
                state.header.stamp = stamp
                state.name = names + ["front_left_wheel_joint"]
                state.position = [joints[ARM.index(n)] for n in names] + [math.nan]
                self.joint_pub.publish(state)
            time.sleep(dt)


class TestWholeBodyMpcNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("wholebody_mpc_test")
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.node)
        threading.Thread(target=cls.executor.spin, daemon=True).start()
        cls.plant = Plant(cls.node)
        cls.client = ActionClient(cls.node, FollowPath, "/wholebody_mpc/follow_path")
        assert cls.client.wait_for_server(timeout_sec=120), "/wholebody_mpc/follow_path did not start"
        time.sleep(1.0)  # TF, joint states and odometry reach the node

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        rclpy.shutdown()

    # ---- helpers ----

    def setUp(self):
        self.plant.reset()
        time.sleep(0.5)
        self.plant.clear_logs()

    def path(self, start, goal, step=0.1):
        """A straight path from `start` to `goal` (x, y) in map, every pose facing along it."""
        length = math.dist(start, goal)
        yaw = math.atan2(goal[1] - start[1], goal[0] - start[0])
        path = Path()
        path.header.frame_id = "map"
        n = max(1, int(math.ceil(length / step)))
        for i in range(n + 1):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = start[0] + (goal[0] - start[0]) * i / n
            pose.pose.position.y = start[1] + (goal[1] - start[1]) * i / n
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            path.poses.append(pose)
        return path

    def wait_for(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertTrue(future.done(), f"no answer within {timeout} s")
        return future.result()

    def send(self, path):
        goal = FollowPath.Goal()
        goal.path = path
        handle = self.wait_for(self.client.send_goal_async(goal), 10.0)
        self.assertTrue(handle.accepted)
        return handle, handle.get_result_async()

    def wait_until(self, condition, timeout, message):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if condition():
                return
            time.sleep(0.02)
        self.fail(message)

    def cmds_since(self, t):
        with self.plant.lock:
            return [(s, v, w) for s, v, w in self.plant.cmd_log if s >= t]

    def trajectories_since(self, t):
        with self.plant.lock:
            return [m for s, m in self.plant.trajectories if s >= t]

    # ---- tests ----

    def test_follow_path_succeeds_on_straight_route(self):
        _, result = self.send(self.path(START[:2], (8.0, 0.0)))
        outcome = self.wait_for(result, 90.0)
        self.assertEqual(outcome.status, GoalStatus.STATUS_SUCCEEDED, outcome.result.error_msg)
        self.assertEqual(outcome.result.error_code, Result.NONE)
        pose, _, _ = self.plant.state()
        self.assertLess(math.dist(pose[:2], (8.0, 0.0)), 0.15, pose)
        self.assertLess(abs(math.remainder(pose[2] - math.pi, 2 * math.pi)), 0.15, pose)
        cmds = self.cmds_since(0.0)
        self.assertGreater(max(v for _, v, _ in cmds), 0.25, "never cruised")
        self.assertEqual(cmds[-1][1:], (0.0, 0.0), "the last base command is not zero")
        self.assertTrue(all(v >= 0.0 for _, v, _ in cmds), "reversed")

    def test_publishes_plan_cycle_and_trajectory(self):
        handle, result = self.send(self.path(START[:2], (9.0, 0.0)))
        time.sleep(3.0)
        pose, _, _ = self.plant.state()
        with self.plant.lock:
            plans = [m for _, m in self.plant.plans]
            cycles = [m for _, m in self.plant.cycles]
            trajectories = [m for _, m in self.plant.trajectories]
        self.wait_for(handle.cancel_goal_async(), 5.0)
        self.assertEqual(self.wait_for(result, 5.0).status, GoalStatus.STATUS_CANCELED)

        # 20 Hz: plan, cycle and trajectory every cycle.
        for name, messages in (("plans", plans), ("cycles", cycles), ("trajectories", trajectories)):
            self.assertGreaterEqual(len(messages), 40, name)
        plan = plans[-1]
        self.assertEqual(plan.header.frame_id, "map")
        self.assertEqual(list(plan.joint_names), ARM)
        self.assertEqual(len(plan.time_from_start), HORIZON + 1)
        self.assertEqual(len(plan.base), HORIZON + 1)
        self.assertEqual(len(plan.positions), (HORIZON + 1) * len(ARM))
        for k, t in enumerate(plan.time_from_start):
            self.assertAlmostEqual(t, k * DT, places=9)
        self.assertLess(math.hypot(plan.base[0].x - pose[0], plan.base[0].y - pose[1]), 0.1)
        self.assertLess(plan.base[-1].x, plan.base[0].x, "the plan does not drive along -x")

        statuses = [c.status for c in cycles]
        self.assertGreater(statuses.count("ok"), 0.9 * len(statuses), statuses)
        for c in cycles:
            for field in ("linearize_s", "collision_s", "setup_s", "solve_s", "total_s", "state_to_command_s"):
                value = getattr(c, field)
                self.assertTrue(math.isfinite(value) and value >= 0.0, (field, value))
            self.assertLessEqual(c.solve_s, c.total_s)
            self.assertLessEqual(c.state_to_command_s, c.total_s)
            self.assertTrue(0.0 <= c.tightness <= 1.0)
            self.assertGreaterEqual(c.manipulability, 0.0)
            self.assertEqual(c.deadline_missed, c.total_s > 1.0 / 20.0)
            if c.status == "ok":
                self.assertGreater(c.iterations, 0)

        for trajectory in trajectories:
            self.assertEqual(list(trajectory.joint_names), ARM)
            self.assertEqual((trajectory.header.stamp.sec, trajectory.header.stamp.nanosec), (0, 0))
        streamed = trajectories[len(trajectories) // 2]
        self.assertEqual(len(streamed.points), HORIZON + 1, "horizon stages 1..N and the braking tail")
        times = [p.time_from_start.sec + 1e-9 * p.time_from_start.nanosec for p in streamed.points]
        self.assertGreater(times[0], 0.0)
        self.assertTrue(all(b > a for a, b in zip(times, times[1:])), times)
        for p in streamed.points:
            self.assertEqual(len(p.positions), len(ARM))
            self.assertEqual(len(p.velocities), len(ARM))
        # It ends at rest after a tail of at least 0.5 s that decelerates within the arm limit.
        tail, last = streamed.points[-2], streamed.points[-1]
        self.assertEqual(list(last.velocities), [0.0] * len(ARM))
        span = times[-1] - times[-2]
        self.assertGreaterEqual(span, 0.5 - 1e-9)
        for j in range(len(ARM)):
            self.assertLessEqual(abs(tail.velocities[j]) / span, 0.61 + 1e-9)
            self.assertAlmostEqual(last.positions[j], tail.positions[j] + 0.5 * tail.velocities[j] * span, places=6)

    def test_cancel_holds_arm_in_place(self):
        # Mid-reorientation: the arm starts away from home and the posture preference turns it back.
        start_joints = [0.8, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.plant.reset(joints=start_joints)
        time.sleep(0.5)
        handle, result = self.send(self.path(START[:2], (8.0, 0.0)))
        self.wait_until(lambda: self.plant.state()[1][0] < 0.7, 10.0, "the arm never moved towards home")
        _, joints_at_cancel, _ = self.plant.state()
        cancel_time = time.monotonic()
        self.wait_for(handle.cancel_goal_async(), 5.0)
        self.assertEqual(self.wait_for(result, 5.0).status, GoalStatus.STATUS_CANCELED)
        time.sleep(1.5)

        trajectories = self.trajectories_since(cancel_time)
        self.assertTrue(trajectories, "no hold point after the cancel")
        hold = trajectories[-1]
        self.assertEqual(len(hold.points), 1, "the last trajectory is not a single hold point")
        point = hold.points[0]
        self.assertEqual(list(point.velocities), [0.0] * len(ARM))
        for held, at_cancel in zip(point.positions, joints_at_cancel):
            self.assertAlmostEqual(held, at_cancel, delta=0.06)  # where the arm was, not home
        self.assertGreater(point.positions[0], 0.3, "snapped back towards home")
        _, joints_after, _ = self.plant.state()
        for now, held in zip(joints_after, point.positions):
            self.assertAlmostEqual(now, held, delta=1e-6)
        cmds = self.cmds_since(cancel_time)
        self.assertTrue(cmds, "no base command after the cancel")
        self.assertEqual(cmds[-1][1:], (0.0, 0.0))
        time.sleep(0.5)
        self.assertEqual(len(self.trajectories_since(cancel_time)), len(trajectories), "streamed after the cancel")

    def test_new_goal_preempts(self):
        first, first_result = self.send(self.path(START[:2], (6.0, 0.0)))
        self.wait_until(lambda: self.plant.state()[2][0] > 0.2, 10.0, "never drove")
        pose, _, _ = self.plant.state()
        goal = (pose[0] - 1.0, 0.0)
        preempt_time = time.monotonic()
        second, second_result = self.send(self.path(pose[:2], goal))
        self.assertEqual(self.wait_for(first_result, 2.0).status, GoalStatus.STATUS_ABORTED, "not preempted")
        # Updated in place: the base keeps driving through the switch.
        time.sleep(1.0)
        switch = [v for t, v, _ in self.cmds_since(preempt_time) if t < preempt_time + 1.0]
        self.assertTrue(switch)
        self.assertGreater(min(switch), 0.1, "the base stopped at the preemption")
        outcome = self.wait_for(second_result, 60.0)
        self.assertEqual(outcome.status, GoalStatus.STATUS_SUCCEEDED, outcome.result.error_msg)
        pose, _, _ = self.plant.state()
        self.assertLess(math.dist(pose[:2], goal), 0.15, pose)

    def test_stale_joint_states_brake_and_fail(self):
        handle, result = self.send(self.path(START[:2], (6.0, 0.0)))
        self.wait_until(lambda: self.plant.state()[2][0] > 0.25, 10.0, "never cruised")
        stop_time = time.monotonic()
        with self.plant.lock:
            self.plant.publish_joints = False
        outcome = self.wait_for(result, 5.0)
        self.assertEqual(outcome.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(outcome.result.error_code, Result.NO_VALID_CONTROL, outcome.result.error_msg)
        self.assertIn("joint", outcome.result.error_msg)
        time.sleep(0.3)
        # Joint states turn stale 0.5 s after the last one: from then on the base brakes at the
        # envelope's limit (0.5 m/s^2 over at most one stage per cycle), not a step to zero, to rest.
        speeds = [v for _, v, _ in self.cmds_since(stop_time + 0.6)]
        self.assertEqual(speeds[-1], 0.0)
        self.assertTrue(all(b <= a + 1e-9 for a, b in zip(speeds, speeds[1:])), speeds)
        self.assertTrue(all(a - b <= 0.5 * DT + 1e-6 for a, b in zip(speeds, speeds[1:])), speeds)
        self.assertGreaterEqual(sum(1 for v in speeds if 0.0 < v < 0.25), 3, speeds)
        # The stream stops with one hold point where the arm is; nothing is commanded from missing data.
        _, joints, _ = self.plant.state()
        after = self.trajectories_since(stop_time)
        holds = [m for m in after if len(m.points) == 1]
        self.assertEqual(len(holds), 1, [len(m.points) for m in after])
        self.assertIs(after[-1], holds[0])
        self.assertEqual(list(holds[0].points[0].velocities), [0.0] * len(ARM))
        for held, now in zip(holds[0].points[0].positions, joints):
            self.assertAlmostEqual(held, now, delta=1e-6)

        # A partial joint state (wrist_3 missing): a new goal fails at once without moving anything.
        with self.plant.lock:
            self.plant.joint_names = ARM[:5]
            self.plant.publish_joints = True
        time.sleep(1.0)
        sent = time.monotonic()
        _, result = self.send(self.path(self.plant.state()[0][:2], (4.0, 0.0)))
        outcome = self.wait_for(result, 5.0)
        self.assertEqual(outcome.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(outcome.result.error_code, Result.NO_VALID_CONTROL, outcome.result.error_msg)
        self.assertTrue(all(v == 0.0 and w == 0.0 for _, v, w in self.cmds_since(sent)))
        self.assertEqual(self.trajectories_since(sent), [])

    def test_no_progress_fails(self):
        # A wedged base (Nav2's progress checker equivalent: 0.3 m or 0.5 rad within 10 s).
        with self.plant.lock:
            self.plant.stuck = True
        sent = time.monotonic()
        _, result = self.send(self.path(START[:2], (8.0, 0.0)))
        outcome = self.wait_for(result, 20.0)
        elapsed = time.monotonic() - sent
        self.assertEqual(outcome.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(outcome.result.error_code, Result.FAILED_TO_MAKE_PROGRESS, outcome.result.error_msg)
        self.assertGreater(elapsed, 9.5)
        self.assertEqual(self.cmds_since(sent)[-1][1:], (0.0, 0.0))

    def test_scenario_references_loaded(self, proc_output, scripted):
        proc_output.assertWaitFor("Loaded 2 panel references", process=scripted, timeout=60)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_stats_file_written_on_shutdown(self, proc_info, mpc):
        launch_testing.asserts.assertExitCodes(proc_info, process=mpc)
        with open(STATS_FILE) as stream:
            stats = json.load(stream)
        self.assertGreater(stats["cycles"], 100)
        self.assertEqual(stats["rate_hz"], 20.0)
        for field in ("linearize_s", "collision_s", "setup_s", "solve_s", "total_s", "state_to_command_s"):
            summary = stats[field]
            for key in ("mean", "median", "p90", "p95", "p99", "max", "misses"):
                self.assertIn(key, summary, field)
            self.assertLessEqual(summary["median"], summary["p90"])
            self.assertLessEqual(summary["p90"], summary["p95"])
            self.assertLessEqual(summary["p95"], summary["p99"])
            self.assertLessEqual(summary["p99"], summary["max"])
        self.assertEqual(stats["deadline_misses"], stats["total_s"]["misses"])
        self.assertGreater(stats["statuses"]["ok"], 100)
