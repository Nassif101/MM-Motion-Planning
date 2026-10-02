"""Phase 1 Nav2 navigation: global planning, Livox filter, and the ADR 0006 command chain.

controller_server / behavior_server -> cmd_vel_nav -> velocity_smoother ->
cmd_vel_smoothed -> collision_monitor -> /cmd_vel (the only /cmd_vel publisher).
controller:=rpp|dwb|mppi selects the local controller; everything else is shared.
Lidar obstacles that persist for 2 s are also in the global costmap (global_obstacles:=persistent,
the default), so a crossing worker stays a local obstacle while a blockage changes the route;
global_obstacles:=live adds every lidar obstacle and global_obstacles:=static keeps the Phase 1
static-map baseline.
dynamic_monitor_zones:=true (B3 missions) makes the collision monitor's stop and slowdown zones
follow each zone's dynamic_polygon_topic, which ReconfigurePanel publishes whenever it switches
the footprint profile; otherwise they are fixed to footprint_profile + margin.
Do not run local_costmap.launch.py at the same time.
"""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

MANAGER_DELAY_S = 3.0  # see global_planning.launch.py


def padded(polygon, margin):
    """Axis-aligned polygon grown by margin on every side (profiles are rectangles)."""
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    x0, x1 = round(min(xs) - margin, 3), round(max(xs) + margin, 3)
    y0, y1 = round(min(ys) - margin, 3), round(max(ys) + margin, 3)
    return [[x1, y1], [x1, y0], [x0, y0], [x0, y1]]


def _nodes(context):
    share = Path(get_package_share_directory("mobile_manipulator_navigation"))
    config = share / "config"
    profiles = yaml.safe_load((config / "footprint_profiles.yaml").read_text())["profiles"]
    profile = LaunchConfiguration("footprint_profile").perform(context)
    if profile not in profiles:
        raise RuntimeError(
            f"Unknown footprint_profile '{profile}'; expected one of {sorted(profiles)}"
        )
    polygon = profiles[profile]["polygon"]
    navigation = yaml.safe_load((config / "nav2_navigation.yaml").read_text())
    monitor = navigation["collision_monitor"]["ros__parameters"]

    local = RewrittenYaml(source_file=str(config / "nav2_local_costmap.yaml"),
                          param_rewrites={"footprint": str(polygon)}, convert_types=True)
    params = str(config / "nav2_navigation.yaml")
    sized = [zone for zone in monitor["polygons"] if "margin_m" in monitor[zone]]
    if LaunchConfiguration("dynamic_monitor_zones").perform(context).lower() == "true":
        # Unparsable points make the zone take its polygon from the topic (latched).
        zones = {key: value for zone in sized for key, value in (
            (f"{zone}.points", ""),
            (f"{zone}.polygon_sub_topic", monitor[zone]["dynamic_polygon_topic"]),
            (f"{zone}.polygon_subscribe_transient_local", True))}
    else:
        zones = {f"{zone}.points": str(padded(polygon, monitor[zone]["margin_m"])) for zone in sized}
    trees = {"replan_if_invalid": "navigate_to_pose_replan_if_invalid_wait_clear.xml",
             "replan_1hz": "navigate_to_pose_wait_clear_recovery.xml"}
    choice = LaunchConfiguration("behavior_tree").perform(context)
    if choice not in trees:
        raise RuntimeError(f"Unknown behavior_tree '{choice}'; expected one of {sorted(trees)}")
    tree = str(share / "behavior_trees" / trees[choice])
    # One controller is loaded, always under the id the behaviour tree requests.
    controllers = yaml.safe_load((config / "nav2_controllers.yaml").read_text())["controllers"]
    controller = LaunchConfiguration("controller").perform(context)
    if controller not in controllers:
        raise RuntimeError(f"Unknown controller '{controller}'; expected one of {sorted(controllers)}")
    chain = [("cmd_vel", "cmd_vel_nav")]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "global_planning.launch.py")),
            launch_arguments={"footprint_profile": profile,
                              "global_obstacles": LaunchConfiguration("global_obstacles")}.items(),
        ),
        Node(package="mobile_manipulator_navigation", executable="livox_robot_filter",
             name="livox_robot_filter", output="screen",
             parameters=[{"use_sim_time": True}]),
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen",
             parameters=[params, local, {"FollowPath": controllers[controller]}],
             remappings=chain),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen",
             parameters=[params], remappings=chain),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother",
             name="velocity_smoother", output="screen",
             parameters=[params], remappings=chain),
        Node(package="nav2_collision_monitor", executable="collision_monitor",
             name="collision_monitor", output="screen",
             parameters=[params, zones]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen",
             parameters=[params, {"default_nav_to_pose_bt_xml": tree}]),
        # Same start delay as global_planning.launch.py, for the local costmap.
        TimerAction(period=MANAGER_DELAY_S, actions=[Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation", output="screen", parameters=[params])]),
        # Small aggregated state for the Unity telemetry window (about 2-3 KB/s).
        Node(package="mobile_manipulator_navigation", executable="nav_telemetry",
             name="nav_telemetry", output="screen",
             parameters=[{"footprint_profile": profile, "behavior_tree": choice,
                          "controller": controller.upper()}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("footprint_profile", default_value="home",
                              description="Arm-pose footprint from config/footprint_profiles.yaml"),
        DeclareLaunchArgument("behavior_tree", default_value="replan_if_invalid",
                              description="replan_if_invalid (default) or replan_1hz (first baseline)"),
        DeclareLaunchArgument("global_obstacles", default_value="persistent",
                              description="persistent (default), live, or static global-costmap "
                                          "obstacles; see global_planning.launch.py"),
        DeclareLaunchArgument("controller", default_value="rpp",
                              description="rpp (bring-up), dwb (B1), or mppi (B2) from "
                                          "config/nav2_controllers.yaml"),
        DeclareLaunchArgument("dynamic_monitor_zones", default_value="false",
                              description="true: stop/slowdown zones follow ReconfigurePanel's "
                                          "profile switches (B3 missions)"),
        OpaqueFunction(function=_nodes),
    ])
