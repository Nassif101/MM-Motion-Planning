#!/usr/bin/env python3
"""Fail if a configured controller parameter is not declared by the running controller server.

ROS 2 silently ignores parameter overrides that a plugin never declares, so a misspelled
key in config/nav2_controllers.yaml would leave the stock default in place. Run with
navigation.launch.py active and the same `controller` block it was launched with.
"""
import argparse
import json
import sys
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import ListParameters


def flatten(block, prefix):
    for key, value in block.items():
        name = f"{prefix}.{key}"
        if isinstance(value, dict):
            yield from flatten(value, name)
        else:
            yield name


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("controller", help="block name in config/nav2_controllers.yaml")
    parser.add_argument("--node", default="/controller_server")
    args = parser.parse_args()

    share = Path(get_package_share_directory("mobile_manipulator_navigation"))
    block = yaml.safe_load((share / "config" / "nav2_controllers.yaml").read_text())[
        "controllers"][args.controller]
    configured = set(flatten(block, "FollowPath"))

    rclpy.init()
    node = rclpy.create_node("check_controller_params")
    client = node.create_client(ListParameters, f"{args.node}/list_parameters")
    if not client.wait_for_service(timeout_sec=10.0):
        sys.exit(f"{args.node}/list_parameters is not available")
    future = client.call_async(ListParameters.Request(prefixes=["FollowPath"], depth=0))
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if future.result() is None:
        sys.exit(f"{args.node} did not list its parameters")
    declared = set(future.result().result.names)
    undeclared = sorted(configured - declared)
    print(json.dumps({"controller": args.controller, "configured": len(configured),
                      "undeclared": undeclared}))
    rclpy.shutdown()
    sys.exit(1 if undeclared else 0)


if __name__ == "__main__":
    main()
