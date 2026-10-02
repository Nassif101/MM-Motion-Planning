"""Robot state, TF and a mock arm on wall time, for MoveIt work without Unity.

Stands in for Unity (ADR 0001): static map -> odom -> base_footprint at `base_pose`
("x,y,yaw" in map), robot_state_publisher, and /joint_states merging the mock arm's
broadcaster with zero wheel joints. Never run alongside Unity.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def static_tf(name, parent, child, x=0.0, y=0.0, yaw=0.0):
    return Node(package='tf2_ros', executable='static_transform_publisher', name=name,
                arguments=['--x', str(x), '--y', str(y), '--yaw', str(yaw),
                           '--frame-id', parent, '--child-frame-id', child])


def launch_setup(context):
    x, y, yaw = (float(v) for v in LaunchConfiguration('base_pose').perform(context).split(','))
    description = Path(get_package_share_directory('mobile_manipulator_description'))
    control = Path(get_package_share_directory('mobile_manipulator_control'))
    return [
        static_tf('map_to_odom', 'map', 'odom'),
        static_tf('odom_to_base_footprint', 'odom', 'base_footprint', x, y, yaw),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='mobile_manipulator_state_publisher',
             parameters=[{'robot_description':
                          (description / 'urdf/mobile_manipulator.urdf').read_text(encoding='utf-8')}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'source_list': ['/arm_joint_state_broadcaster/joint_states'],
                          'rate': 50,
                          'robot_description':
                          (description / 'urdf/mobile_manipulator.urdf').read_text(encoding='utf-8')}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(control / 'launch/arm_control.launch.py')),
            launch_arguments={'use_mock_hardware': 'true'}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('base_pose', default_value='0,0,0',
                              description='base_footprint pose in map as "x,y,yaw"'),
        OpaqueFunction(function=launch_setup),
    ])
