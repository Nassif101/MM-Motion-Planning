import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

MANAGER_DELAY_S = 3.0
FOOTPRINT_MODES = ("static", "profiles", "dynamic", "wholebody")
OBSTACLE_BANDS = ("full", "base_band")


def band_overrides(band):
    """Parameter file narrowing both lidar obstacle layers to the base band (B5).

    band is base_band of nav2_global_planning.yaml: the whole-body MPC avoids overhead obstacles
    itself, so the global route only reacts to what the base can hit.
    """
    heights = {"min_obstacle_height": band["min_obstacle_height"],
               "max_obstacle_height": band["max_obstacle_height"]}
    return {"global_costmap": {"global_costmap": {"ros__parameters": {
        "persistent_obstacle_layer": dict(heights),
        "obstacle_layer": {"livox_mark": dict(heights)}}}}}



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
    footprint_mode = LaunchConfiguration("footprint_mode").perform(context)
    if footprint_mode not in FOOTPRINT_MODES:
        raise RuntimeError(f"Unknown footprint_mode '{footprint_mode}'; expected one of {FOOTPRINT_MODES}")
    planner_parameters = []
    band = LaunchConfiguration("obstacle_band").perform(context)
    if band not in OBSTACLE_BANDS:
        raise RuntimeError(f"Unknown obstacle_band '{band}'; expected one of {OBSTACLE_BANDS}")
    if footprint_mode == "wholebody" and band != "base_band":
        raise RuntimeError("footprint_mode wholebody (B5) needs obstacle_band:=base_band")
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
    if band == "base_band":
        planning = yaml.safe_load((share / "config" / "nav2_global_planning.yaml").read_text(encoding="utf-8"))
        band_heights = planning["global_costmap"]["global_costmap"]["ros__parameters"]["base_band"]
        override = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
        yaml.safe_dump(band_overrides(band_heights), override)
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

    # footprint_mode:=dynamic or wholebody: the footprint node owns the footprint topics; the launch
    # profile is only what the costmaps start with until its first footprint arrives.
    dynamic = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / "dynamic_footprint.launch.py")),
        launch_arguments={"footprint_model": LaunchConfiguration("footprint_model"),
                          "footprint_mode": "wholebody" if footprint_mode == "wholebody" else "dynamic"}.items(),
    )] if footprint_mode in ("dynamic", "wholebody") else []

    return dynamic + [
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
            DeclareLaunchArgument(
                "footprint_mode",
                default_value="static",
                description="static (launch profile fixed), profiles (ReconfigurePanel switches "
                "named profiles, B3), dynamic (dynamic_footprint_node, B4), or wholebody "
                "(dynamic_footprint_node with the base-only footprint, B5; needs obstacle_band:=base_band)",
            ),
            DeclareLaunchArgument(
                "obstacle_band",
                default_value="full",
                description="full: lidar obstacles from 0.05 to 2.0 m (B1-B4); base_band: only the "
                "band of base_band in nav2_global_planning.yaml (0.05 to 0.60 m, B5)",
            ),
            DeclareLaunchArgument("footprint_model", default_value="mesh",
                                  description="dynamic footprint model: mesh or disc"),
            OpaqueFunction(function=_nodes),
        ]
    )
