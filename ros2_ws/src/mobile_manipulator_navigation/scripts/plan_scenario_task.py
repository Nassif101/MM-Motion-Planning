#!/usr/bin/env python3
"""Planner-only scenario task: verify the placed start pose, then query each global planner.

Read-only (ComputePathToPose; no motion). Used by tools/run_nav_scenario.py. Reports
success, planner-reported and wall-clock latency, path length, and minimum clearance
between path poses and occupied /map cells (the path reference point, not the footprint).
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def pose(xyyaw, stamp):
    message = PoseStamped()
    message.header.frame_id = "map"
    message.header.stamp = stamp
    message.pose.position.x, message.pose.position.y = xyyaw[0], xyyaw[1]
    message.pose.orientation.z = math.sin(xyyaw[2] / 2)
    message.pose.orientation.w = math.cos(xyyaw[2] / 2)
    return message


def occupied_points(grid):
    data = np.asarray(grid.data, dtype=np.int16).reshape(grid.info.height, grid.info.width)
    rows, cols = np.nonzero(data >= 65)
    resolution = grid.info.resolution
    origin = grid.info.origin.position
    return np.column_stack((origin.x + (cols + 0.5) * resolution,
                            origin.y + (rows + 0.5) * resolution))


def min_clearance(path, obstacles):
    if not len(path) or not len(obstacles):
        return None
    path = np.asarray(path)
    lo, hi = path.min(axis=0) - 3.0, path.max(axis=0) + 3.0
    near = obstacles[np.all((obstacles >= lo) & (obstacles <= hi), axis=1)]
    if not len(near):
        return 3.0
    best = min(float(np.min(np.hypot(*(near - p).T))) for p in path)
    return round(best, 3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start", type=float, nargs=3, required=True, metavar=("X", "Y", "YAW"))
    parser.add_argument("--goal", type=float, nargs=3, required=True, metavar=("X", "Y", "YAW"))
    parser.add_argument("--planners", nargs="+", required=True)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--start-tolerance", type=float, default=0.10,
                        help="max distance between placed TF pose and scenario start (m)")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    rclpy.init()
    node = Node("plan_scenario_task", parameter_overrides=[Parameter("use_sim_time", value=True)])
    buffer = Buffer()
    TransformListener(buffer, node)
    maps = []
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, "/map", maps.append, latched)
    client = ActionClient(node, ComputePathToPose, "compute_path_to_pose")

    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline and not (
            maps and buffer.can_transform("map", "base_footprint", Time())):
        rclpy.spin_once(node, timeout_sec=0.1)
    if not maps:
        raise SystemExit("No /map received")
    placed = buffer.lookup_transform("map", "base_footprint", Time()).transform
    placed_xy = (placed.translation.x, placed.translation.y)
    offset = math.dist(placed_xy, args.start[:2])
    if offset > args.start_tolerance:
        raise SystemExit(f"Robot is {offset:.3f} m from the scenario start; placement failed")
    if not client.wait_for_server(timeout_sec=10.0):
        raise SystemExit("compute_path_to_pose is not available")
    obstacles = occupied_points(maps[-1])

    report = {"start": args.start, "goal": args.goal,
              "placed_start_offset_m": round(offset, 3), "planners": {}}
    for planner in args.planners:
        runs = []
        for _ in range(args.repeats):
            request = ComputePathToPose.Goal()
            request.start = pose(args.start, node.get_clock().now().to_msg())
            request.goal = pose(args.goal, node.get_clock().now().to_msg())
            request.planner_id = planner
            request.use_start = True
            began = time.monotonic()
            handle = client.send_goal_async(request)
            rclpy.spin_until_future_complete(node, handle)
            result_future = handle.result().get_result_async()
            rclpy.spin_until_future_complete(node, result_future)
            wall = time.monotonic() - began
            result = result_future.result().result
            path = [(p.pose.position.x, p.pose.position.y) for p in result.path.poses]
            runs.append({
                "error_code": result.error_code,
                "planner_time_s": result.planning_time.sec + result.planning_time.nanosec * 1e-9,
                "wall_latency_s": round(wall, 4),
                "length_m": round(sum(math.dist(a, b) for a, b in zip(path, path[1:])), 3),
                "poses": len(path),
                "min_static_clearance_m": min_clearance(path, obstacles),
            })
        report["planners"][planner] = {
            "success_rate": sum(r["error_code"] == 0 for r in runs) / len(runs),
            "runs": runs,
        }
        print(planner, json.dumps(report["planners"][planner]), flush=True)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
