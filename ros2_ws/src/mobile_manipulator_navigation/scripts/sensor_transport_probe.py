#!/usr/bin/env python3
"""Measure Unity -> ROS transport load and small-message jitter with the lidar active.

Read-only. For a wall-clock window it records /livox/lidar size and rate, wall-clock
inter-arrival statistics of /clock, /tf, and /arm/state, and the ROS-TCP endpoint's CPU
use (from /proc). Receiver-side observations, not one-way latency.
"""
import argparse
import json
import os
import statistics
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, PointCloud2
from tf2_msgs.msg import TFMessage


def cpu_seconds(pid):
    fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")


def interval_stats(arrivals):
    gaps = [b - a for a, b in zip(arrivals, arrivals[1:])]
    ordered = sorted(gaps)
    pick = lambda q: round(ordered[min(len(ordered) - 1, int(q * len(ordered)))] * 1000, 2)
    return {"count": len(arrivals),
            "rate_hz": round((len(arrivals) - 1) / (arrivals[-1] - arrivals[0]), 2),
            "gap_ms": {"p50": pick(0.5), "p95": pick(0.95), "p99": pick(0.99),
                       "max": round(ordered[-1] * 1000, 2),
                       "stdev": round(statistics.pstdev(gaps) * 1000, 2)}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=30.0, help="wall-clock window")
    parser.add_argument("--label", required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    endpoint = subprocess.run(
        ["pgrep", "-f", "lib/mobile_manipulator_control/unity_control_endpoint.py"],
        text=True, capture_output=True).stdout.split()
    endpoint_pid = int(endpoint[0]) if endpoint else None

    rclpy.init()
    node = Node("sensor_transport_probe")
    arrivals = {"/clock": [], "/tf": [], "/arm/state": [], "/livox/lidar": []}
    cloud = {"bytes": [], "points": []}

    def arrived(topic):
        return lambda _msg: arrivals[topic].append(time.monotonic())

    def on_cloud(message):
        arrivals["/livox/lidar"].append(time.monotonic())
        cloud["bytes"].append(len(message.data))
        cloud["points"].append(message.width * message.height)

    node.create_subscription(Clock, "/clock", arrived("/clock"), 1000)
    node.create_subscription(TFMessage, "/tf", arrived("/tf"), 1000)
    node.create_subscription(JointState, "/arm/state", arrived("/arm/state"), 1000)
    node.create_subscription(PointCloud2, "/livox/lidar", on_cloud, qos_profile_sensor_data)

    start_cpu = cpu_seconds(endpoint_pid) if endpoint_pid else None
    start = time.monotonic()
    while time.monotonic() - start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    elapsed = time.monotonic() - start
    end_cpu = cpu_seconds(endpoint_pid) if endpoint_pid else None
    rclpy.shutdown()

    report = {
        "label": args.label,
        "window_s": round(elapsed, 2),
        "lidar": {
            "mean_points": round(statistics.mean(cloud["points"])),
            "mean_bytes": round(statistics.mean(cloud["bytes"])),
            "bandwidth_mb_s": round(sum(cloud["bytes"]) / elapsed / 1e6, 3),
        } if cloud["bytes"] else None,
        "endpoint_cpu_percent": (round(100 * (end_cpu - start_cpu) / elapsed, 1)
                                 if endpoint_pid else None),
        "topics": {topic: interval_stats(times) for topic, times in arrivals.items() if len(times) > 2},
    }
    print(json.dumps(report, indent=2))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
