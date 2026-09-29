#!/usr/bin/env python3
"""Navigate-to-pose scenario task: preflight, one NavigateToPose goal, and run metrics.

Used by tools/run_nav_scenario.py with navigation.launch.py active. Preflight refuses to
send a goal unless the placed pose matches the scenario start, the base is stationary on
/odom, the local costmap is publishing, and the collision monitor is the only /cmd_vel
publisher. All timing is simulation time unless named wall_*.

Metrics: success and error code, time to goal, executed path length, final position and
heading error, cross-track error to the latest /plan, minimum clearance between the
posed footprint polygon and occupied static-map cells (and, separately, the scenario's
unmapped obstacle boxes), recoveries, collision-monitor
activations, /cmd_vel acceleration and jerk, filtered-lidar gaps over 0.5 s, local
costmap publish interval, and CPU/memory of the navigation processes.
"""
import argparse
import json
import math
import os
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

from mobile_manipulator_navigation.telemetry import cross_track as polyline_distance

NAV_PROCESSES = ("controller_server", "planner_server", "bt_navigator", "velocity_smoother",
                 "collision_monitor", "behavior_server", "map_server", "livox_robot_filter")
ACTIONS = {0: "none", 1: "stop", 2: "slowdown", 3: "approach", 4: "limit"}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def proc_cpu_mem():
    """(cpu seconds, rss MB) per navigation process name, from /proc."""
    tick = os.sysconf("SC_CLK_TCK")
    usage = {}
    for pid in filter(str.isdigit, os.listdir("/proc")):
        try:
            cmdline = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode()
            name = next((n for n in NAV_PROCESSES if n in cmdline and "ros2 launch" not in cmdline), None)
            if name is None or "__node:=" not in cmdline and name != "livox_robot_filter":
                continue
            fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
            rss = int(Path(f"/proc/{pid}/statm").read_text().split()[1]) * os.sysconf("SC_PAGE_SIZE")
            cpu = (int(fields[11]) + int(fields[12])) / tick
            previous = usage.get(name, (0.0, 0.0))
            usage[name] = (previous[0] + cpu, max(previous[1], rss / 1e6))
        except (OSError, ValueError, IndexError):
            continue
    return usage


def rectangle_clearance(points_robot, bounds):
    """Distance from each robot-frame point to an axis-aligned rectangle (0 if inside)."""
    x0, x1, y0, y1 = bounds
    dx = np.maximum(np.maximum(x0 - points_robot[:, 0], points_robot[:, 0] - x1), 0.0)
    dy = np.maximum(np.maximum(y0 - points_robot[:, 1], points_robot[:, 1] - y1), 0.0)
    return np.hypot(dx, dy)


def box_outline(obstacle, spacing=0.02):
    """Points along the outline of an axis-aligned obstacle box in the map frame."""
    x0, x1 = obstacle["x"] - obstacle["size_x"] / 2, obstacle["x"] + obstacle["size_x"] / 2
    y0, y1 = obstacle["y"] - obstacle["size_y"] / 2, obstacle["y"] + obstacle["size_y"] / 2
    xs = np.linspace(x0, x1, max(2, int(round((x1 - x0) / spacing)) + 1))
    ys = np.linspace(y0, y1, max(2, int(round((y1 - y0) / spacing)) + 1))
    return np.vstack([np.column_stack((xs, np.full_like(xs, y0))), np.column_stack((xs, np.full_like(xs, y1))),
                      np.column_stack((np.full_like(ys, x0), ys)), np.column_stack((np.full_like(ys, x1), ys))])


def stats(values):
    if not values:
        return None
    ordered = sorted(values)
    return {"mean": round(float(np.mean(ordered)), 4),
            "p95": round(ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))], 4),
            "max": round(ordered[-1], 4)}


class Recorder(Node):
    def __init__(self):
        super().__init__("navigate_scenario_task",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map = None
        self.plan = None
        self.odom = None
        self.cmd = []
        self.lidar = []
        self.costmap_stamps = []
        self.monitor = []
        self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        self.create_subscription(PathMsg, "/plan", self.on_plan, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 50)
        self.create_subscription(PointCloud2, "/livox/points_filtered", self.on_lidar,
                                 qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.on_costmap, 10)
        self.create_subscription(CollisionMonitorState, "/collision_monitor_state",
                                 self.on_monitor, 10)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_map(self, message):
        self.map = message

    def on_plan(self, message):
        self.plan = np.array([(p.pose.position.x, p.pose.position.y) for p in message.poses])

    def on_odom(self, message):
        self.odom = message

    def on_cmd(self, message):
        self.cmd.append((self.now(), message.linear.x, message.angular.z))

    def on_lidar(self, message):
        self.lidar.append(message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)

    def on_costmap(self, message):
        self.costmap_stamps.append(message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)

    def on_monitor(self, message):
        self.monitor.append((self.now(), ACTIONS.get(message.action_type, str(message.action_type)),
                             message.polygon_name))

    def pose(self):
        if not self.buffer.can_transform("map", "base_footprint", Time()):
            return None
        t = self.buffer.lookup_transform("map", "base_footprint", Time()).transform
        return t.translation.x, t.translation.y, yaw_of(t.rotation)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start", type=float, nargs=3, required=True, metavar=("X", "Y", "YAW"))
    parser.add_argument("--goal", type=float, nargs=3, required=True, metavar=("X", "Y", "YAW"))
    parser.add_argument("--footprint-profile", required=True)
    parser.add_argument("--timeout", type=float, required=True, help="simulated seconds")
    parser.add_argument("--start-tolerance", type=float, default=0.10)
    parser.add_argument("--obstacles", default="[]", help="JSON list of scenario obstacle boxes")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    share = Path(get_package_share_directory("mobile_manipulator_navigation"))
    polygon = yaml.safe_load((share / "config" / "footprint_profiles.yaml").read_text())[
        "profiles"][args.footprint_profile]["polygon"]
    bounds = (min(p[0] for p in polygon), max(p[0] for p in polygon),
              min(p[1] for p in polygon), max(p[1] for p in polygon))

    rclpy.init()
    node = Recorder()
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")

    # ---- Preflight (no goal is sent unless every check passes) ----
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline and not (
            node.map and node.odom and node.pose() and len(node.costmap_stamps) >= 2):
        rclpy.spin_once(node, timeout_sec=0.1)
    problems = []
    placed = node.pose()
    if node.map is None or node.odom is None or placed is None or len(node.costmap_stamps) < 2:
        problems.append("map, odom, TF, or local costmap not available")
    else:
        offset = math.dist(placed[:2], args.start[:2])
        if offset > args.start_tolerance:
            problems.append(f"robot is {offset:.3f} m from the scenario start")
        twist = node.odom.twist.twist
        if abs(twist.linear.x) > 0.02 or abs(twist.angular.z) > 0.02:
            problems.append("base is not stationary")
    # A fresh node discovers the ~20-node graph gradually: wait until the /cmd_vel
    # publisher list is non-empty and unchanged for 2 s (at most 10 s).
    publishers, stable_since, deadline = None, time.monotonic(), time.monotonic() + 10.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        current = sorted(p.node_name for p in node.get_publishers_info_by_topic("/cmd_vel"))
        if current != publishers:
            publishers, stable_since = current, time.monotonic()
        elif publishers and time.monotonic() - stable_since >= 2.0:
            break
    if publishers != ["collision_monitor"]:
        problems.append(f"/cmd_vel publishers are {publishers}, expected collision_monitor")
    if not client.wait_for_server(timeout_sec=10.0):
        problems.append("navigate_to_pose is not available")
    if problems:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({"preflight_failed": problems}, indent=2) + "\n")
        raise SystemExit("Preflight failed: " + "; ".join(problems))

    occupied = []
    grid = node.map
    data = np.asarray(grid.data, dtype=np.int16).reshape(grid.info.height, grid.info.width)
    rows, cols = np.nonzero(data >= 65)
    occupied = np.column_stack((grid.info.origin.position.x + (cols + 0.5) * grid.info.resolution,
                                grid.info.origin.position.y + (rows + 0.5) * grid.info.resolution))

    obstacle_points = (np.vstack([box_outline(o) for o in json.loads(args.obstacles)])
                       if json.loads(args.obstacles) else np.zeros((0, 2)))

    # ---- Goal ----
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x, goal.pose.pose.position.y = args.goal[0], args.goal[1]
    goal.pose.pose.orientation.z = math.sin(args.goal[2] / 2)
    goal.pose.pose.orientation.w = math.cos(args.goal[2] / 2)
    feedback = {"recoveries": 0}

    def on_feedback(message):
        feedback["recoveries"] = message.feedback.number_of_recoveries

    cpu_before, wall_before = proc_cpu_mem(), time.monotonic()
    started = node.now()
    node.cmd.clear(), node.lidar.clear(), node.costmap_stamps.clear(), node.monitor.clear()
    handle_future = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, handle_future)
    handle = handle_future.result()
    if not handle.accepted:
        raise SystemExit("NavigateToPose goal rejected")
    result_future = handle.get_result_async()

    trajectory, cross_track, clearance, obstacle_clearance = [], [], [], []
    next_sample = started
    while not result_future.done():
        rclpy.spin_once(node, timeout_sec=0.02)
        now = node.now()
        if now - started > args.timeout:
            handle.cancel_goal_async()
            break
        if now < next_sample:
            continue
        next_sample = now + 0.05
        pose = node.pose()
        if pose is None:
            continue
        trajectory.append((now, *pose))
        if node.plan is not None and len(node.plan):
            # Distance to the path polyline, not to its nearest discrete pose (which
            # overstated the error by up to half the pose spacing before 2026-09-28).
            cross_track.append(polyline_distance(pose[:2], [tuple(p) for p in node.plan]))
        c, s = math.cos(pose[2]), math.sin(pose[2])
        near = occupied[np.hypot(*(occupied - pose[:2]).T) < 8.0]
        if len(near):
            local = (near - pose[:2]) @ np.array([[c, -s], [s, c]])
            clearance.append(float(rectangle_clearance(local, bounds).min()))
        if len(obstacle_points):
            local = (obstacle_points - pose[:2]) @ np.array([[c, -s], [s, c]])
            obstacle_clearance.append(float(rectangle_clearance(local, bounds).min()))
    deadline = time.monotonic() + 10.0
    while not result_future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    finished = node.now()
    cpu_after, wall_elapsed = proc_cpu_mem(), time.monotonic() - wall_before

    status = result_future.result().status if result_future.done() else None
    error_code = result_future.result().result.error_code if result_future.done() else None
    final = node.pose()
    points = np.array([t[1:3] for t in trajectory]) if trajectory else np.zeros((0, 2))
    length = float(np.sum(np.hypot(*np.diff(points, axis=0).T))) if len(points) > 1 else 0.0

    commands = node.cmd
    accel, jerk = [], []
    for (t0, v0, w0), (t1, v1, w1) in zip(commands, commands[1:]):
        if t1 > t0:
            accel.append((t1, (v1 - v0) / (t1 - t0), (w1 - w0) / (t1 - t0)))
    for (t0, a0, b0), (t1, a1, b1) in zip(accel, accel[1:]):
        if t1 > t0:
            jerk.append(((a1 - a0) / (t1 - t0), (b1 - b0) / (t1 - t0)))
    transitions, previous = [], None
    for stamp, action, polygon_name in node.monitor:
        if (action, polygon_name) != previous and action != "none":
            transitions.append({"t": round(stamp - started, 2), "action": action, "polygon": polygon_name})
        previous = (action, polygon_name)
    gaps = [b - a for a, b in zip(node.lidar, node.lidar[1:])]
    costmap_gaps = [b - a for a, b in zip(node.costmap_stamps, node.costmap_stamps[1:])]

    report = {
        "status": {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_ABORTED: "aborted",
                   GoalStatus.STATUS_CANCELED: "canceled"}.get(status, str(status)),
        "error_code": error_code,
        "timed_out": finished - started > args.timeout,
        "time_s": round(finished - started, 2),
        "wall_s": round(wall_elapsed, 2),
        "path_length_m": round(length, 3),
        "straight_line_m": round(math.dist(args.start[:2], args.goal[:2]), 3),
        "final_position_error_m": round(math.dist(final[:2], args.goal[:2]), 3) if final else None,
        "final_yaw_error_rad": round(abs(math.remainder(final[2] - args.goal[2], math.tau)), 3) if final else None,
        "cross_track_m": stats(cross_track),
        "min_footprint_clearance_to_static_map_m": round(min(clearance), 3) if clearance else None,
        "min_footprint_clearance_to_obstacles_m": round(min(obstacle_clearance), 3) if obstacle_clearance else None,
        "recoveries": feedback["recoveries"],
        "collision_monitor_activations": transitions,
        "cmd_vel": {"messages": len(commands),
                    "abs_linear_accel": stats([abs(a[1]) for a in accel]),
                    "abs_angular_accel": stats([abs(a[2]) for a in accel]),
                    "abs_linear_jerk": stats([abs(j[0]) for j in jerk]),
                    "abs_angular_jerk": stats([abs(j[1]) for j in jerk])},
        "lidar_gaps_over_0p5s": sum(g > 0.5 for g in gaps),
        "max_lidar_gap_s": round(max(gaps), 3) if gaps else None,
        "local_costmap_publish_interval_s": stats(costmap_gaps),
        "cpu_percent_of_core": {name: round(100 * (cpu_after[name][0] - cpu_before.get(name, (0, 0))[0])
                                            / wall_elapsed, 1)
                                for name in cpu_after},
        "max_rss_mb": {name: round(value[1], 1) for name, value in cpu_after.items()},
        "trajectory": [[round(v, 3) for v in sample] for sample in trajectory[::4]],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    summary = {k: v for k, v in report.items() if k not in ("trajectory", "cpu_percent_of_core", "max_rss_mb")}
    print(json.dumps(summary))
    rclpy.shutdown()
    raise SystemExit(0 if report["status"] == "succeeded" else 3)


if __name__ == "__main__":
    main()
