#!/usr/bin/env python3
"""Filter /livox/lidar for navigation consumers (local-costmap perception contract).

Drops UnitySensors zero-point misses and robot self-returns, then republishes the
remaining points unchanged in livox_frame on /livox/points_filtered so costmaps keep the
sensor origin for ray tracing. Self-returns are points inside the URDF collision
primitives or the attached reference panel, posed from the current TF: either inside a
primitive enlarged by `margin`, or on a ray that first hits the robot and no more than
`noise_band` (4 sigma of the lidar range noise) in front of that surface. Obstacles
between the sensor and the robot or off its rays stay visible, even inside the
footprint rectangle.
Ground and overhead returns are kept for the consumers' own height handling.
"""
import json
import statistics
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from mobile_manipulator_navigation.lidar_robot_filter import keep_mask, load_primitives


def matrix_of(transform):
    x, y, z, w = (transform.rotation.x, transform.rotation.y,
                  transform.rotation.z, transform.rotation.w)
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    matrix[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
    return matrix


class LivoxRobotFilter(Node):
    def __init__(self):
        super().__init__("livox_robot_filter")
        self.margin = self.declare_parameter("margin", 0.03).value
        # UnitySensors Mid-360 range noise is Gaussian with sigma 0.02 m; 4 sigma.
        self.noise_band = self.declare_parameter("noise_band", 0.08).value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        description = Path(get_package_share_directory("mobile_manipulator_description"))
        control = Path(get_package_share_directory("mobile_manipulator_control"))
        payload = json.loads((control / "config" / "qualified_payload.json").read_text())["payload"]
        self.primitives = load_primitives(
            (description / "urdf" / "mobile_manipulator.urdf").read_text(),
            {"link": "tool0", "size": payload["dimensions_tool_ros_m"],
             "center": payload["com_tool_ros_m"]})
        self.links = sorted({link for link, *_ in self.primitives})

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(PointCloud2, "/livox/points_filtered",
                                               qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/livox/lidar", self.on_cloud,
                                 qos_profile_sensor_data)
        self.durations, self.kept, self.skipped = [], [], 0
        self.get_logger().info(f"Self-model: {len(self.primitives)} primitives on "
                               f"{len(self.links)} links incl. payload panel, margin {self.margin} m, "
                               f"noise band {self.noise_band} m")

    def posed_primitives(self, sensor_frame):
        """Latest link poses in the base frame; None if any transform is missing."""
        try:
            sensor = matrix_of(self.buffer.lookup_transform(self.base_frame, sensor_frame, Time()).transform)
            links = {link: matrix_of(self.buffer.lookup_transform(self.base_frame, link, Time()).transform)
                     for link in self.links}
        except TransformException:
            return None, None
        return sensor, [(links[link] @ pose, kind, dims) for link, pose, kind, dims in self.primitives]

    def on_cloud(self, cloud):
        started = time.perf_counter()
        sensor, posed = self.posed_primitives(cloud.header.frame_id)
        if sensor is None:
            self.skipped += 1
            return
        offsets = {f.name: f.offset for f in cloud.fields}
        raw = np.frombuffer(cloud.data, dtype=np.uint8).reshape(-1, cloud.point_step)
        xyz = np.stack([raw[:, offsets[a]:offsets[a] + 4].copy().view(np.float32)[:, 0]
                        for a in ("x", "y", "z")], axis=1)
        keep = keep_mask(xyz, sensor, posed, self.margin, self.noise_band)

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
                f"kept {statistics.mean(self.kept) * 100:.1f} % of points; "
                f"skipped {self.skipped} scans without TF")
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
