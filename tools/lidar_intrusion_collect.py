#!/usr/bin/env python3
"""Container side of measure_lidar_intrusion.py: per-scan k-th deepest lidar intrusion into one face.

Reads /livox/points_filtered for --seconds, transforms each cloud into base_footprint (tf2), keeps points in
the StopZone's height band (0.05-0.60 m) whose lateral coordinate lies within +-lateral (the StopZone polygon's
extent along the face), and prints one JSON line {"per_scan": [...]} with, per scan, the k-th deepest intrusion
in front of the face at --face along --axis (x: in front of the robot, y: to its left).
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parent))
from measure_lidar_intrusion import kth_deepest_intrusion  # noqa: E402

BAND = (0.05, 0.60)


def quaternion_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Collector(Node):
    def __init__(self, args):
        super().__init__("lidar_intrusion_collect")
        self.args = args
        self.values = []
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(PointCloud2, "/livox/points_filtered", self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, cloud):
        try:
            t = self.buffer.lookup_transform("base_footprint", cloud.header.frame_id, rclpy.time.Time(),
                                             timeout=Duration(seconds=0.1)).transform
        except Exception:  # noqa: BLE001 (TF not ready yet)
            return
        xyz = point_cloud2.read_points_numpy(cloud, field_names=("x", "y", "z"), skip_nans=True)
        if xyz.size == 0:
            self.values.append(0.0)
            return
        rotation = quaternion_matrix(t.rotation)
        points = xyz @ rotation.T + np.array([t.translation.x, t.translation.y, t.translation.z])
        a = self.args
        normal, lateral = (0, 1) if a.axis == "x" else (1, 0)
        keep = (points[:, 2] >= BAND[0]) & (points[:, 2] <= BAND[1]) & (np.abs(points[:, lateral]) <= a.lateral)
        selected = points[keep]
        self.values.append(kth_deepest_intrusion([(p[normal], 0.0) for p in selected], face_x=a.face, k=a.k))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--axis", choices=("x", "y"), required=True)
    parser.add_argument("--face", type=float, required=True)
    parser.add_argument("--lateral", type=float, required=True)
    parser.add_argument("--k", type=int, default=4)
    args = parser.parse_args()
    rclpy.init()
    node = Collector(args)
    end = node.get_clock().now() + Duration(seconds=args.seconds)
    while rclpy.ok() and node.get_clock().now() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(json.dumps({"per_scan": node.values}))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
