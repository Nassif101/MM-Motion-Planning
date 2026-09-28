#!/usr/bin/env python3
"""Check Unity /odom against the ground-truth odom -> base_footprint TF.

Read-only. Records both streams for a fixed simulated duration (drive the base
meanwhile, e.g. with base_step_test.py) and reports: rate, frames, stamps on physics
ticks, pose equality for identical stamps, and twist agreement with centred TF
differences expressed in base_footprint.
"""
import argparse
import json
import math
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage

TICK_NS = 20_000_000


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=30.0, help="simulated duration")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    rclpy.init()
    node = Node("odom_consistency_check",
                parameter_overrides=[Parameter("use_sim_time", value=True)])
    odom, tf = {}, {}
    node.create_subscription(Odometry, "/odom", lambda m: odom.setdefault(stamp_ns(m.header.stamp), m), 100)

    def on_tf(message):
        for t in message.transforms:
            if t.child_frame_id == "base_footprint":
                tf.setdefault(stamp_ns(t.header.stamp), t)
    node.create_subscription(TFMessage, "/tf", on_tf, 100)

    while node.get_clock().now().nanoseconds == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    end = node.get_clock().now().nanoseconds + int(args.seconds * 1e9)
    while node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    rclpy.shutdown()

    stamps = sorted(set(odom) & set(tf))
    pose_error = max(math.dist(
        (odom[s].pose.pose.position.x, odom[s].pose.pose.position.y, odom[s].pose.pose.position.z),
        (tf[s].transform.translation.x, tf[s].transform.translation.y, tf[s].transform.translation.z))
        for s in stamps)
    yaw_error = max(abs(math.remainder(yaw(odom[s].pose.pose.orientation) - yaw(tf[s].transform.rotation), math.tau))
                    for s in stamps)
    linear_errors, angular_errors, steady_linear, steady_angular = [], [], [], []
    peak_v, peak_w = 0.0, 0.0
    for before, current, after in zip(stamps, stamps[2:], stamps[4:]):
        a, b = tf[before].transform, tf[after].transform
        dt = (after - before) * 1e-9
        heading = yaw(tf[current].transform.rotation)
        vx = (b.translation.x - a.translation.x) / dt
        vy = (b.translation.y - a.translation.y) / dt
        v_body = (math.cos(heading) * vx + math.sin(heading) * vy,
                  -math.sin(heading) * vx + math.cos(heading) * vy)
        w = math.remainder(yaw(b.rotation) - yaw(a.rotation), math.tau) / dt
        twist = odom[current].twist.twist
        linear_errors.append(math.dist(v_body, (twist.linear.x, twist.linear.y)))
        angular_errors.append(abs(w - twist.angular.z))
        # Steady: the 80 ms window's endpoint twists agree, so the centred difference is unbiased.
        ends = (odom[before].twist.twist, odom[after].twist.twist)
        if (abs(ends[0].linear.x - ends[1].linear.x) < 0.01
                and abs(ends[0].angular.z - ends[1].angular.z) < 0.01):
            steady_linear.append(linear_errors[-1])
            steady_angular.append(angular_errors[-1])
        peak_v, peak_w = max(peak_v, abs(twist.linear.x)), max(peak_w, abs(twist.angular.z))

    # PhysX integrates semi-implicitly (x[k] = x[k-1] + v[k] dt), so a one-tick TF
    # difference must reproduce the reported twist at the later tick if /odom is exact.
    tick_linear, tick_angular = [], []
    for previous, current in zip(stamps, stamps[1:]):
        if current - previous != TICK_NS:
            continue
        a, b = tf[previous].transform, tf[current].transform
        dt = TICK_NS * 1e-9
        heading = yaw(b.rotation)
        vx = (b.translation.x - a.translation.x) / dt
        vy = (b.translation.y - a.translation.y) / dt
        v_body = (math.cos(heading) * vx + math.sin(heading) * vy,
                  -math.sin(heading) * vx + math.cos(heading) * vy)
        w = math.remainder(yaw(b.rotation) - yaw(a.rotation), math.tau) / dt
        twist = odom[current].twist.twist
        tick_linear.append(math.dist(v_body, (twist.linear.x, twist.linear.y)))
        tick_angular.append(abs(w - twist.angular.z))

    def quantiles(values):
        ordered = sorted(values)
        pick = lambda q: round(ordered[min(len(ordered) - 1, int(q * len(ordered)))], 4)
        return {"n": len(ordered), "p50": pick(0.5), "p95": pick(0.95),
                "p99": pick(0.99), "max": round(ordered[-1], 4)}

    sample = odom[stamps[-1]]
    report = {
        "odom_messages": len(odom), "tf_samples": len(tf), "paired": len(stamps),
        "odom_rate_hz": round(len(odom) / args.seconds, 2),
        "frames": [sample.header.frame_id, sample.child_frame_id],
        "all_stamps_on_physics_ticks": all(s % TICK_NS == 0 for s in odom),
        "max_pose_error_m": pose_error, "max_yaw_error_rad": yaw_error,
        "linear_twist_error_mps": quantiles(linear_errors),
        "angular_twist_error_radps": quantiles(angular_errors),
        "steady_linear_twist_error_mps": quantiles(steady_linear),
        "steady_angular_twist_error_radps": quantiles(steady_angular),
        "one_tick_linear_twist_error_mps": quantiles(tick_linear),
        "one_tick_angular_twist_error_radps": quantiles(tick_angular),
        "peak_linear_mps": round(peak_v, 3), "peak_angular_radps": round(peak_w, 3),
    }
    print(json.dumps(report, indent=2))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
