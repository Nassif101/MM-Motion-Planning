#!/usr/bin/env python3
"""Container side of measure_lidar_intrusion.py: per scan, the k-th nearest lidar point to the padded base hull.

Reads /livox/points_filtered for --seconds, transforms each cloud into base_footprint (tf2), keeps points in the
StopZone's height band (0.05-0.60 m) and prints one JSON line {"pose": [x, y, yaw], "per_scan": [...]}: the
robot's map -> base_footprint pose at the start and, per scan, the k-th smallest distance of the band points
from the hull |x| <= half_x, |y| <= half_y (infinity, as JSON Infinity, with fewer than k points).
"""
import argparse
import json
import math
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
from measure_lidar_intrusion import kth_nearest_to_hull  # noqa: E402

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
        self.pose = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(PointCloud2, "/livox/points_filtered", self.on_cloud, qos_profile_sensor_data)

    def read_pose(self):
        try:
            t = self.buffer.lookup_transform("map", "base_footprint", rclpy.time.Time()).transform
        except Exception:  # noqa: BLE001 (TF not ready yet)
            return
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = [t.translation.x, t.translation.y, yaw]

    def on_cloud(self, cloud):
        if self.pose is None:
            self.read_pose()
        try:
            t = self.buffer.lookup_transform("base_footprint", cloud.header.frame_id, rclpy.time.Time(),
                                             timeout=Duration(seconds=0.1)).transform
        except Exception:  # noqa: BLE001 (TF not ready yet)
            return
        xyz = point_cloud2.read_points_numpy(cloud, field_names=("x", "y", "z"), skip_nans=True)
        if xyz.size == 0:
            self.values.append(math.inf)
            return
        points = xyz @ quaternion_matrix(t.rotation).T + np.array([t.translation.x, t.translation.y, t.translation.z])
        band = points[(points[:, 2] >= BAND[0]) & (points[:, 2] <= BAND[1])]
        a = self.args
        self.values.append(kth_nearest_to_hull(band[:, :2].tolist(), a.half_x, a.half_y, a.k))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--half-x", type=float, required=True)
    parser.add_argument("--half-y", type=float, required=True)
    parser.add_argument("--k", type=int, default=4)
    args = parser.parse_args()
    rclpy.init()
    node = Collector(args)
    end = node.get_clock().now() + Duration(seconds=args.seconds)
    while rclpy.ok() and node.get_clock().now() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(json.dumps({"pose": node.pose, "per_scan": node.values}))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
