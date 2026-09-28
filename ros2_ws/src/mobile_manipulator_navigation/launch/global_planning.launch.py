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
    map_yaml = str(share / "maps" / "construction_site.yaml")
    profiles = yaml.safe_load(
        (share / "config" / "footprint_profiles.yaml").read_text(encoding="utf-8")
    )["profiles"]
    profile = LaunchConfiguration("footprint_profile").perform(context)
    if profile not in profiles:
        raise RuntimeError(
            f"Unknown footprint_profile '{profile}'; expected one of {sorted(profiles)}"
        )
    parameters = RewrittenYaml(
        source_file=str(share / "config" / "nav2_global_planning.yaml"),
        param_rewrites={
            "footprint": str(profiles[profile]["polygon"]),
            "lattice_filepath": str(
                Path(get_package_share_directory("nav2_smac_planner"))
                / "sample_primitives" / "5cm_resolution" / "0.5m_turning_radius"
                / "diff" / "output.json"
            ),
        },
        convert_types=True,
    )

    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[parameters, {"yaml_filename": map_yaml}],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[parameters],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_global_planning",
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
                description="Arm-pose footprint from config/footprint_profiles.yaml "
                "(home or vertical_carry)",
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
