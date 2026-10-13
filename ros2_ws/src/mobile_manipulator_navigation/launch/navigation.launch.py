"""Phase 1 Nav2 navigation: global planning, Livox filter, and the ADR 0006 command chain.

controller_server / behavior_server -> cmd_vel_nav -> velocity_smoother ->
cmd_vel_smoothed -> collision_monitor -> /cmd_vel (the only /cmd_vel publisher).
controller:=rpp|dwb|mppi selects the local controller; everything else is shared.
Lidar obstacles that persist for 2 s are also in the global costmap (global_obstacles:=persistent,
the default), so a crossing worker stays a local obstacle while a blockage changes the route;
global_obstacles:=live adds every lidar obstacle and global_obstacles:=static keeps the Phase 1
static-map baseline.
footprint_mode selects the one owner of the costmap footprints and the collision monitor's
stop and slowdown zones (ADR 0010): static keeps footprint_profile (+ margin) for the run;
profiles (B3 missions) and dynamic (B4) make the zones follow each zone's dynamic_polygon_topic,
published by ReconfigurePanel at profile switches or by dynamic_footprint_node from the joint
state (footprint_model:=mesh|disc).
controller:=wholebody (B5) replaces controller_server and velocity_smoother with
wholebody_mpc_node, a FollowPath action server in its own process (no local costmap) that
publishes the base command on cmd_vel_smoothed, the collision monitor's input, and the arm
trajectory itself; the behaviour tree is navigate_to_pose_wholebody.xml, footprint_mode is forced
to wholebody (base-only costmap footprint, swept-hull slowdown zone) and the global costmap and
the collision monitor see obstacles only in the 0.05-0.60 m base band (base_band in
nav2_global_planning.yaml): the MPC avoids overhead obstacles itself. posture_mode, comfort_pose and
clearance_weight override wholebody_mpc.yaml's posture preference and clearance cost ("" = the file's
value, the recorded B5).
slowdown_zone:=false (any controller) disables the collision monitor's slowdown zone
(SlowdownZone.enabled false): it stays declared and its input topic subscribed, so footprint ownership
and the zone checks are unchanged, but it never scales a command; the stop zone and FootprintApproach
stay. The default true is every recorded baseline.
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
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

MANAGER_DELAY_S = 3.0  # see global_planning.launch.py
WHOLEBODY = "wholebody"
# B5: the lifecycle manager manages these. wholebody_mpc is a plain rclcpp node, not a lifecycle node.
WHOLEBODY_MANAGED = ["behavior_server", "collision_monitor", "bt_navigator"]


def padded(polygon, margin):
    """Axis-aligned polygon grown by margin on every side (profiles are rectangles)."""
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    x0, x1 = round(min(xs) - margin, 3), round(max(xs) + margin, 3)
    y0, y1 = round(min(ys) - margin, 3), round(max(ys) + margin, 3)
    return [[x1, y1], [x1, y0], [x0, y0], [x0, y1]]


def wholebody_monitor_parameters(config):
    """B5 collision-monitor overrides: the base band, and the global costmap's footprint.

    There is no local costmap, so FootprintApproach follows the global costmap's footprint (the
    base-only footprint the footprint node publishes to both costmaps).
    """
    planning = yaml.safe_load((config / "nav2_global_planning.yaml").read_text())
    band = planning["global_costmap"]["global_costmap"]["ros__parameters"]["base_band"]
    return {"livox.min_height": band["min_obstacle_height"],
            "livox.max_height": band["max_obstacle_height"],
            "FootprintApproach.footprint_topic": "/global_costmap/published_footprint"}


def slowdown_parameters(value):
    """slowdown_zone:=true|false: false disables the collision monitor's slowdown zone only.

    The zone stays declared (its points or input topic as before), so the footprint owner keeps
    publishing its input and the ownership and zone checks see the same graph; the monitor just
    skips a disabled polygon. The stop zone and FootprintApproach are untouched.
    """
    choice = value.strip().lower()
    if choice not in ("true", "false"):
        raise RuntimeError(f"slowdown_zone must be true or false, not '{value}'")
    return {} if choice == "true" else {"SlowdownZone.enabled": False}


# controller:=wholebody options that override wholebody_mpc.yaml when given ("" keeps the file's value).
# value_type str: a name such as none must stay text, never parse as YAML.
MPC_OVERRIDES = {"posture_mode": str, "comfort_pose": str, "clearance_weight": float}


def mpc_overrides(context):
    """The wholebody_mpc parameters given as launch arguments; empty ones keep wholebody_mpc.yaml's."""
    return {name: ParameterValue(LaunchConfiguration(name), value_type=kind) for name, kind in MPC_OVERRIDES.items()
            if LaunchConfiguration(name).perform(context).strip()}


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
    controller = LaunchConfiguration("controller").perform(context)
    wholebody = controller == WHOLEBODY
    footprint_mode = LaunchConfiguration("footprint_mode").perform(context)
    if wholebody:
        footprint_mode = "wholebody"
    elif footprint_mode == "wholebody":
        raise RuntimeError("footprint_mode wholebody is only valid with controller:=wholebody")
    if footprint_mode in ("profiles", "dynamic", "wholebody"):
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
    if wholebody:
        # One tree: the MPC is a FollowPath server of its own and there is no local costmap to clear.
        choice = "wholebody"
        trees[choice] = "navigate_to_pose_wholebody.xml"
    if choice not in trees:
        raise RuntimeError(f"Unknown behavior_tree '{choice}'; expected one of {sorted(trees)}")
    tree = str(share / "behavior_trees" / trees[choice])
    # One controller is loaded, always under the id the behaviour tree requests.
    controllers = yaml.safe_load((config / "nav2_controllers.yaml").read_text())["controllers"]
    if controller not in controllers and not wholebody:
        raise RuntimeError(
            f"Unknown controller '{controller}'; expected one of {sorted([*controllers, WHOLEBODY])}")
    chain = [("cmd_vel", "cmd_vel_nav")]
    if wholebody:
        zones.update(wholebody_monitor_parameters(config))
    zones.update(slowdown_parameters(LaunchConfiguration("slowdown_zone").perform(context)))

    behavior_parameters = [params]
    if wholebody:
        # No local costmap: Wait does not use the collision checker, but the server still
        # subscribes to its costmap and footprint, so point both at the global costmap.
        behavior_parameters.append({"local_costmap_topic": "global_costmap/costmap_raw",
                                    "local_footprint_topic": "global_costmap/published_footprint"})
        command_nodes = [
            Node(package="nav2_behaviors", executable="behavior_server",
                 name="behavior_server", output="screen", parameters=behavior_parameters,
                 # Wait's stop commands go nowhere: wholebody_mpc is the only command source.
                 remappings=[("cmd_vel", "cmd_vel_nav")]),
            # FollowPath server and the only command publisher upstream of the collision monitor.
            Node(package="mobile_manipulator_mpc", executable="wholebody_mpc_node",
                 name="wholebody_mpc", output="screen",
                 # value_type str: an empty scenario must stay "", not parse as YAML null.
                 parameters=[{"use_sim_time": True,
                              "scenario": ParameterValue(LaunchConfiguration("scenario"), value_type=str),
                              "references": ParameterValue(LaunchConfiguration("references"), value_type=str),
                              "stats_file": ParameterValue(LaunchConfiguration("mpc_stats_file"), value_type=str),
                              "rate_hz": ParameterValue(LaunchConfiguration("mpc_rate_hz"), value_type=float),
                              **mpc_overrides(context)}],
                 remappings=[("cmd_vel_wholebody", "cmd_vel_smoothed")]),
        ]
    else:
        command_nodes = [
            Node(package="nav2_controller", executable="controller_server",
                 name="controller_server", output="screen",
                 parameters=[params, local, {"FollowPath": controllers[controller]}],
                 remappings=chain),
            Node(package="nav2_behaviors", executable="behavior_server",
                 name="behavior_server", output="screen",
                 parameters=behavior_parameters, remappings=chain),
            Node(package="nav2_velocity_smoother", executable="velocity_smoother",
                 name="velocity_smoother", output="screen",
                 parameters=[params], remappings=chain),
        ]
    manager_parameters = [params] + ([{"node_names": WHOLEBODY_MANAGED}] if wholebody else [])

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "global_planning.launch.py")),
            launch_arguments={"footprint_profile": profile,
                              "global_obstacles": LaunchConfiguration("global_obstacles"),
                              "footprint_mode": footprint_mode,
                              "obstacle_band": "base_band" if wholebody else "full",
                              "footprint_model": LaunchConfiguration("footprint_model")}.items(),
        ),
        Node(package="mobile_manipulator_navigation", executable="livox_robot_filter",
             name="livox_robot_filter", output="screen",
             parameters=[{"use_sim_time": True}]),
        *command_nodes,
        Node(package="nav2_collision_monitor", executable="collision_monitor",
             name="collision_monitor", output="screen",
             parameters=[params, zones]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen",
             parameters=[params, {"default_nav_to_pose_bt_xml": tree}]),
        # Same start delay as global_planning.launch.py, for the local costmap.
        TimerAction(period=MANAGER_DELAY_S, actions=[Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation", output="screen", parameters=manager_parameters)]),
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
                                          "config/nav2_controllers.yaml, or wholebody (B5, "
                                          "wholebody_mpc_node)"),
        DeclareLaunchArgument("footprint_mode", default_value="static",
                              description="static, profiles (ReconfigurePanel, B3 missions) or "
                                          "dynamic (dynamic_footprint_node, B4); wholebody is "
                                          "forced by controller:=wholebody and invalid otherwise"),
        DeclareLaunchArgument("scenario", default_value="",
                              description="controller:=wholebody: scenario whose known obstacles "
                                          "the MPC loads (\"\" = none, free drive)"),
        DeclareLaunchArgument("references", default_value="none",
                              description="controller:=wholebody: none or scripted panel references"),
        DeclareLaunchArgument("mpc_stats_file", default_value="",
                              description="controller:=wholebody: file the MPC writes its timing "
                                          "summary (JSON) to on shutdown (\"\" = none)"),
        DeclareLaunchArgument("mpc_rate_hz", default_value="20.0",
                              description="controller:=wholebody: MPC loop rate in Hz (20 bring-up, "
                                          "50 target)"),
        DeclareLaunchArgument("posture_mode", default_value="",
                              description="controller:=wholebody: blend, comfort_only or none (\"\" = "
                                          "wholebody_mpc.yaml's, blend)"),
        DeclareLaunchArgument("comfort_pose", default_value="",
                              description="controller:=wholebody: home or carry (\"\" = "
                                          "wholebody_mpc.yaml's, carry; home is the recorded B5); the "
                                          "pose must be in "
                                          "qualified_payload.json poses_rad"),
        DeclareLaunchArgument("clearance_weight", default_value="",
                              description="controller:=wholebody: weight of the optional clearance cost "
                                          "(\"\" = wholebody_mpc.yaml's, 0 = off)"),
        DeclareLaunchArgument("footprint_model", default_value="mesh",
                              description="dynamic footprint model: mesh or disc"),
        DeclareLaunchArgument("slowdown_zone", default_value="true",
                              description="true (default, every recorded baseline) or false: disable the "
                                          "collision monitor's slowdown zone (stop zone and "
                                          "FootprintApproach stay), any controller"),
        OpaqueFunction(function=_nodes),
    ])
