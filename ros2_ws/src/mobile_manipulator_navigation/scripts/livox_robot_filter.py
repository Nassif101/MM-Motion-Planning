#!/usr/bin/env python3
"""Filter /livox/lidar for navigation consumers (local-costmap perception contract).

Drops UnitySensors zero-point misses and robot/panel self-returns inside the active
footprint profile, then republishes the remaining points unchanged in livox_frame on
/livox/points_filtered so costmaps keep the sensor origin for ray tracing. Ground and
overhead returns are kept: consumers apply their own marking heights, and ground rays
are what clear low obstacles near the robot. The sensor mount is fixed, so the livox_frame ->
base_footprint transform is read once from TF.
"""
import statistics
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

from mobile_manipulator_navigation.lidar_robot_filter import footprint_bounds, keep_mask


def quaternion_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class LivoxRobotFilter(Node):
    def __init__(self):
        super().__init__("livox_robot_filter")
        profile = self.declare_parameter("footprint_profile", "home").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        share = Path(get_package_share_directory("mobile_manipulator_navigation"))
        profiles = yaml.safe_load((share / "config" / "footprint_profiles.yaml").read_text())
        if profile not in profiles["profiles"]:
            raise ValueError(f"Unknown footprint_profile '{profile}'")
        self.bounds = footprint_bounds(profiles["profiles"][profile]["polygon"])

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.transform = None
        self.publisher = self.create_publisher(PointCloud2, "/livox/points_filtered",
                                               qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/livox/lidar", self.on_cloud,
                                 qos_profile_sensor_data)
        self.durations = []
        self.kept = []
        self.get_logger().info(f"Filtering misses and self-returns for footprint profile "
                               f"'{profile}' {self.bounds}")

    def sensor_transform(self, frame):
        if self.transform is None:
            if not self.buffer.can_transform(self.base_frame, frame, Time()):
                return None
            t = self.buffer.lookup_transform(self.base_frame, frame, Time()).transform
            self.transform = (quaternion_matrix(t.rotation),
                              np.array([t.translation.x, t.translation.y, t.translation.z]))
        return self.transform

    def on_cloud(self, cloud):
        started = time.perf_counter()
        transform = self.sensor_transform(cloud.header.frame_id)
        if transform is None:
            return
        offsets = {f.name: f.offset for f in cloud.fields}
        raw = np.frombuffer(cloud.data, dtype=np.uint8).reshape(-1, cloud.point_step)
        xyz = np.stack([raw[:, offsets[a]:offsets[a] + 4].copy().view(np.float32)[:, 0]
                        for a in ("x", "y", "z")], axis=1)
        keep = keep_mask(xyz, transform[0], transform[1], self.bounds)

        out = PointCloud2()
        out.header = cloud.header
        out.height = 1
        out.width = int(keep.sum())
        out.fields = cloud.fields
        out.is_bigendian = cloud.is_bigendian
        out.point_step = cloud.point_step
        out.row_step = out.width * out.point_step
        out.is_dense = True
        out.data = raw[keep].tobytes()
        self.publisher.publish(out)

        self.durations.append(time.perf_counter() - started)
        self.kept.append(out.width / max(1, len(raw)))
        if len(self.durations) == 100:
            ordered = sorted(self.durations)
            self.get_logger().info(
                f"100 scans: processing p50 {ordered[49] * 1e3:.2f} ms, "
                f"p99 {ordered[98] * 1e3:.2f} ms, max {ordered[-1] * 1e3:.2f} ms; "
                f"kept {statistics.mean(self.kept) * 100:.1f} % of points")
            self.durations.clear()
            self.kept.clear()


def main():
    rclpy.init()
    node = LivoxRobotFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
