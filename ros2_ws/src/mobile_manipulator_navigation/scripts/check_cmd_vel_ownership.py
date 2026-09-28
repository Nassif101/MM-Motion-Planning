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
    parser.add_argument("--discovery-seconds", type=float, default=8.0,
                        help="upper bound; returns early once the graph is stable for 2 s")
    args = parser.parse_args()

    rclpy.init()
    node = Node("check_cmd_vel_ownership")
    # A fresh node discovers large graphs gradually; wait until the endpoint lists of the
    # topic stop changing (and at least one endpoint is known) rather than a fixed delay.
    deadline = time.monotonic() + args.discovery_seconds
    snapshot, stable_since = None, time.monotonic()
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        current = (sorted(p.node_name for p in node.get_publishers_info_by_topic(TOPIC)),
                   sorted(s.node_name for s in node.get_subscriptions_info_by_topic(TOPIC)))
        if current != snapshot:
            snapshot, stable_since = current, time.monotonic()
        elif current != ([], []) and time.monotonic() - stable_since >= 2.0:
            break
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
