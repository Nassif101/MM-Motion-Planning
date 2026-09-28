"""Standalone local-costmap qualification harness: Livox filter plus rolling costmap.

The controller server loads the same nav2_local_costmap.yaml block in the full
navigation launch; do not run both at once.
"""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _nodes(context):
    share = Path(get_package_share_directory("mobile_manipulator_navigation"))
    profiles = yaml.safe_load(
        (share / "config" / "footprint_profiles.yaml").read_text(encoding="utf-8")
    )["profiles"]
    profile = LaunchConfiguration("footprint_profile").perform(context)
    if profile not in profiles:
        raise RuntimeError(
            f"Unknown footprint_profile '{profile}'; expected one of {sorted(profiles)}"
        )
    parameters = RewrittenYaml(
        source_file=str(share / "config" / "nav2_local_costmap.yaml"),
        param_rewrites={"footprint": str(profiles[profile]["polygon"])},
        convert_types=True,
    )
    return [
        Node(
            package="mobile_manipulator_navigation",
            executable="livox_robot_filter.py",
            name="livox_robot_filter",
            output="screen",
            parameters=[{"footprint_profile": profile, "use_sim_time": True}],
        ),
        Node(
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d",
            name="local_costmap",
            namespace="local_costmap",
            output="screen",
            parameters=[parameters],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_local_costmap",
            output="screen",
            parameters=[parameters],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "footprint_profile",
                default_value="home",
                description="Arm-pose footprint from config/footprint_profiles.yaml",
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
