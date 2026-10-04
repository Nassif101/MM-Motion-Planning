"""dynamic_footprint_node for footprint_mode:=dynamic (baseline B4, ADR 0010).

The node is the only publisher of both costmap footprints and the collision monitor's zone
inputs; its geometry and update settings are config/dynamic_footprint.yaml. footprint_model
overrides the configured model (mesh or disc).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("footprint_model", default_value="mesh", description="mesh or disc"),
        DeclareLaunchArgument("stats_file", default_value="/tmp/mm_dynamic_footprint_stats.json",
                              description="timing and publish statistics written on shutdown"),
        Node(package="mobile_manipulator_navigation", executable="dynamic_footprint_node",
             name="dynamic_footprint_node", output="screen",
             parameters=[{"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                          "model": LaunchConfiguration("footprint_model"),
                          "stats_file": LaunchConfiguration("stats_file")}]),
    ])
