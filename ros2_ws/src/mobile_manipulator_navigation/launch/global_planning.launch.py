import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

MANAGER_DELAY_S = 3.0


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
    planner_parameters = []
    mode = LaunchConfiguration("global_obstacles").perform(context)
    obstacle_layers = {"static": None, "false": None, "persistent": "persistent_obstacle_layer",
                       "live": "obstacle_layer", "true": "obstacle_layer"}
    if mode not in obstacle_layers:
        raise RuntimeError(f"Unknown global_obstacles '{mode}'; expected persistent, live, or static")
    if obstacle_layers[mode]:
        # A separate file keeps the string-array type (RewrittenYaml would write a string).
        layers = {"global_costmap": {"global_costmap": {"ros__parameters": {
            "plugins": ["static_layer", obstacle_layers[mode], "inflation_layer"]}}}}
        override = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
        yaml.safe_dump(layers, override)  # needs /livox/points_filtered (navigation.launch.py)
        override.close()
        planner_parameters.append(override.name)
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
            parameters=[parameters, *planner_parameters],
        ),
        # Start managing only after the nodes have sim time and the static map -> odom
        # transform: a costmap whose transform wait starts before /clock arrives times out at
        # once, and the planner then plans on an inactive, empty costmap (2026-09-28).
        TimerAction(period=MANAGER_DELAY_S, actions=[Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_global_planning",
            output="screen",
            parameters=[parameters],
        )]),
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
            DeclareLaunchArgument(
                "global_obstacles",
                default_value="static",
                description="static: static map only; persistent: lidar obstacles that stay "
                "for 2 s; live (or true): every lidar obstacle, 10 s decay. persistent and live "
                "need the Livox robot filter from navigation.launch.py",
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
