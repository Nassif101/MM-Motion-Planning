#!/usr/bin/env python3
"""Classify one /livox/lidar scan relative to the robot for the local-costmap contract.

Read-only. Separates misses (encoded by UnitySensors as zero points), transforms the
returns into base_footprint with TF, and reports robot self-returns (inside the active
footprint polygon and above the ground), ground returns, and the nearest ground /
obstacle ranges. Run with the robot stationary in an open area.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


def quaternion_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default="home")
    parser.add_argument("--ground-band", type=float, default=0.05,
                        help="points below this height in base_footprint count as ground")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    share = Path(get_package_share_directory("mobile_manipulator_navigation"))
    polygon = yaml.safe_load((share / "config" / "footprint_profiles.yaml").read_text())[
        "profiles"][args.profile]["polygon"]
    x_min, x_max = min(p[0] for p in polygon), max(p[0] for p in polygon)
    y_min, y_max = min(p[1] for p in polygon), max(p[1] for p in polygon)

    rclpy.init()
    node = Node("lidar_self_return_probe")
    buffer = Buffer()
    TransformListener(buffer, node)
    clouds = []
    node.create_subscription(PointCloud2, "/livox/lidar", clouds.append, qos_profile_sensor_data)
    while len(clouds) < 3:
        rclpy.spin_once(node, timeout_sec=0.1)
    cloud = clouds[-1]
    while not buffer.can_transform("base_footprint", cloud.header.frame_id, Time()):
        rclpy.spin_once(node, timeout_sec=0.1)
    transform = buffer.lookup_transform("base_footprint", cloud.header.frame_id, Time()).transform
    rclpy.shutdown()

    raw = point_cloud2.read_points_numpy(cloud, field_names=("x", "y", "z"), skip_nans=True)
    # UnitySensors encodes misses and out-of-range returns as (0, 0, 0) in the sensor frame.
    miss = np.linalg.norm(raw, axis=1) == 0.0
    raw = raw[~miss]
    points = raw @ quaternion_matrix(transform.rotation).T + np.array(
        [transform.translation.x, transform.translation.y, transform.translation.z])
    planar = np.hypot(points[:, 0], points[:, 1])
    ground = points[:, 2] < args.ground_band
    inside = ((points[:, 0] >= x_min) & (points[:, 0] <= x_max) &
              (points[:, 1] >= y_min) & (points[:, 1] <= y_max))
    self_returns = inside & ~ground
    others = ~inside & ~ground

    report = {
        "profile": args.profile,
        "frame_id": cloud.header.frame_id,
        "points": int(len(points) + miss.sum()),
        "misses_encoded_at_sensor_origin": int(miss.sum()),
        "self_returns_inside_footprint": int(self_returns.sum()),
        "self_return_height_range_m": ([round(float(points[self_returns, 2].min()), 3),
                                        round(float(points[self_returns, 2].max()), 3)]
                                       if self_returns.any() else None),
        "ground_returns": int(ground.sum()),
        "nearest_ground_return_m": round(float(planar[ground].min()), 3) if ground.any() else None,
        "nearest_non_robot_return_m": round(float(planar[others].min()), 3) if others.any() else None,
        "sensor_height_m": round(transform.translation.z, 3),
        "predicted_ground_blind_radius_m": round(
            transform.translation.z / math.tan(math.radians(7.212303)) + transform.translation.x, 3),
    }
    print(json.dumps(report, indent=2))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
