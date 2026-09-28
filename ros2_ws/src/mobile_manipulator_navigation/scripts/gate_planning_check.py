#!/usr/bin/env python3
"""Query each global planner across the ConstructionSiteV1 gates and report the route.

Read-only: sends ComputePathToPose goals with explicit start poses and never commands
motion. Requires global_planning.launch.py active (complete map -> base_footprint TF).
Gate geometry comes from ConstructionSiteTools (Unity) converted with ROS x = Unity z,
ROS y = -Unity x.
"""
import argparse
import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node

# name: (start xy, goal xy, heading, gate line axis, line value, opening min, max)
CASES = {
    # ManipulationRequiredGate_1p05m: posts at Unity z=-7.225, opening Unity x 7.20..8.25.
    "gate_1p05": ((-5.3, -7.725), (-9.0, -7.725), math.pi, "x", -7.225, -8.25, -7.20),
    # ControlledGate_1p35m: posts at Unity x=0, opening Unity z -8.00..-6.65.
    "gate_1p35": ((-7.325, 2.5), (-7.325, -2.5), -math.pi / 2, "y", 0.0, -8.0, -6.65),
}


def pose(node, xy, heading):
    message = PoseStamped()
    message.header.frame_id = "map"
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.position.x, message.pose.position.y = xy
    message.pose.orientation.z = math.sin(heading / 2)
    message.pose.orientation.w = math.cos(heading / 2)
    return message


def crossing(points, axis, line, low, high):
    """Coordinate where the path crosses the gate line, if it does."""
    index = 0 if axis == "x" else 1
    for a, b in zip(points, points[1:]):
        if (a[index] - line) * (b[index] - line) <= 0 and a[index] != b[index]:
            s = (line - a[index]) / (b[index] - a[index])
            other = a[1 - index] + s * (b[1 - index] - a[1 - index])
            return other, low <= other <= high
    return None, False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planners", nargs="+", default=["GridBased", "Lattice"])
    parser.add_argument("--label", required=True, help="e.g. the active footprint profile")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    rclpy.init()
    node = Node("gate_planning_check")
    client = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
    if not client.wait_for_server(timeout_sec=10.0):
        raise SystemExit("compute_path_to_pose is not available")

    results = {"label": args.label, "cases": {}}
    for name, (start, goal, heading, axis, line, low, high) in CASES.items():
        for planner in args.planners:
            request = ComputePathToPose.Goal()
            request.start = pose(node, start, heading)
            request.goal = pose(node, goal, heading)
            request.planner_id = planner
            request.use_start = True
            handle_future = client.send_goal_async(request)
            rclpy.spin_until_future_complete(node, handle_future)
            result_future = handle_future.result().get_result_async()
            rclpy.spin_until_future_complete(node, result_future)
            result = result_future.result().result
            points = [(p.pose.position.x, p.pose.position.y) for p in result.path.poses]
            length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
            where, through = crossing(points, axis, line, low, high) if points else (None, False)
            entry = {
                "error_code": result.error_code,
                "planning_time_s": result.planning_time.sec + result.planning_time.nanosec * 1e-9,
                "poses": len(points),
                "length_m": round(length, 3),
                "straight_line_m": round(math.dist(start, goal), 3),
                "through_gate": through,
                "gate_line_crossing": None if where is None else round(where, 3),
            }
            results["cases"][f"{name}/{planner}"] = entry
            print(f"{args.label:15s} {name:10s} {planner:10s} {json.dumps(entry)}", flush=True)

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(results, indent=2) + "\n")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
