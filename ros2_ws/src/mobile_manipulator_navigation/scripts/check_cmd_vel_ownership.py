#!/usr/bin/env python3
"""Fail unless /cmd_vel has at most one publisher, of the expected node and type (ADR 0006).

Read-only graph check to run before experiments. Unity's in-editor keyboard teleop is
not visible to ROS and must be disabled separately.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node

TOPIC = "/cmd_vel"
TYPE = "geometry_msgs/msg/Twist"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expect", help="Node name that must be the only publisher "
                        "(e.g. collision_monitor); omit to require no publisher")
    parser.add_argument("--discovery-seconds", type=float, default=2.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node("check_cmd_vel_ownership")
    deadline = time.monotonic() + args.discovery_seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    publishers = node.get_publishers_info_by_topic(TOPIC)
    subscribers = node.get_subscriptions_info_by_topic(TOPIC)
    rclpy.shutdown()

    names = [p.node_name for p in publishers]
    problems = []
    for info in publishers + subscribers:
        if info.topic_type != TYPE:
            problems.append(f"{info.node_name} uses {info.topic_type}, expected {TYPE}")
    if args.expect is None and names:
        problems.append(f"expected no publisher, found {names}")
    if args.expect is not None and names != [args.expect]:
        problems.append(f"expected only {args.expect}, found {names or 'none'}")

    print(f"{TOPIC}: publishers={names} subscribers={[s.node_name for s in subscribers]}")
    for problem in problems:
        print(f"FAIL: {problem}")
    sys.exit(1 if problems else 0)


if __name__ == "__main__":
    main()
