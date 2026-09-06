#!/usr/bin/env python3
"""Publish the ros2_control-augmented model without introducing a second TF publisher."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = Node('arm_control_description')
    description = node.declare_parameter('robot_description', '').value
    if not description:
        raise RuntimeError('robot_description is required')
    publisher = node.create_publisher(String, '/arm/robot_description',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publisher.publish(String(data=description))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
