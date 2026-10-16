import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
NAV = yaml.safe_load((ROOT / "config" / "nav2_navigation.yaml").read_text())
CONTROLLERS = yaml.safe_load((ROOT / "config" / "nav2_controllers.yaml").read_text())["controllers"]
ENVELOPE = yaml.safe_load((ROOT / "config" / "nav_operating_envelope.yaml").read_text())
PROFILES = yaml.safe_load((ROOT / "config" / "footprint_profiles.yaml").read_text())["profiles"]
TREES = {name: ET.parse(ROOT / "behavior_trees" / f"navigate_to_pose_{name}.xml")
         for name in ("replan_if_invalid_wait_clear", "wait_clear_recovery")}


def params(node):
    return NAV[node]["ros__parameters"]


def strictest(key):
    return min(profile[key] for profile in ENVELOPE["profiles"].values())


def test_command_generation_stays_inside_the_operating_envelope():
    smoother = params("velocity_smoother")
    assert smoother["max_velocity"][0] <= strictest("max_forward_mps")
    assert -smoother["min_velocity"][0] <= strictest("max_reverse_mps")
    assert smoother["max_velocity"][2] <= strictest("max_yaw_radps")
    assert -smoother["min_velocity"][2] <= strictest("max_yaw_radps")
    assert smoother["max_velocity"][1] == smoother["min_velocity"][1] == 0.0
    acceleration = ENVELOPE["acceleration"]
    assert smoother["max_accel"][0] <= acceleration["linear_accel_mps2"]
    assert -smoother["max_decel"][0] <= acceleration["linear_decel_mps2"]
    assert smoother["max_accel"][2] <= acceleration["yaw_accel_radps2"]
    assert -smoother["max_decel"][2] <= acceleration["yaw_decel_radps2"]


def test_every_controller_stays_inside_the_operating_envelope():
    acceleration = ENVELOPE["acceleration"]
    forward, yaw = strictest("max_forward_mps"), strictest("max_yaw_radps")
    assert set(CONTROLLERS) == {"rpp", "dwb", "mppi"}

    rpp = CONTROLLERS["rpp"]
    assert rpp["desired_linear_vel"] <= forward
    assert rpp["rotate_to_heading_angular_vel"] <= yaw
    assert rpp["max_angular_accel"] <= acceleration["yaw_accel_radps2"]
    assert rpp["allow_reversing"] is False

    dwb = CONTROLLERS["dwb"]
    assert max(dwb["max_vel_x"], dwb["max_speed_xy"]) <= forward
    assert dwb["min_vel_x"] == 0.0  # forward only, like the bring-up controller
    assert dwb["min_vel_y"] == dwb["max_vel_y"] == 0.0
    assert dwb["max_vel_theta"] <= yaw
    assert dwb["acc_lim_x"] <= acceleration["linear_accel_mps2"]
    assert -dwb["decel_lim_x"] <= acceleration["linear_decel_mps2"]
    assert dwb["acc_lim_theta"] <= acceleration["yaw_accel_radps2"]
    assert -dwb["decel_lim_theta"] <= acceleration["yaw_decel_radps2"]

    mppi = CONTROLLERS["mppi"]
    assert mppi["motion_model"] == "DiffDrive"
    assert mppi["vx_max"] <= forward
    assert mppi["vx_min"] == 0.0 and mppi["vy_max"] == 0.0
    assert mppi["wz_max"] <= yaw
    # MPPI's forward and yaw acceleration models are stock (see nav2_controllers.yaml); the
    # velocity smoother enforces the envelope on its commands. Braking stays in the envelope.
    assert -mppi["ax_min"] <= acceleration["linear_decel_mps2"]


def test_slowest_rpp_commands_clear_the_measured_breakaway():
    # From rest the base ignores commands below the breakaway speed (base-controller README).
    breakaway = ENVELOPE["tracking"]["breakaway_linear_mps"]
    rpp = CONTROLLERS["rpp"]
    assert rpp["min_approach_linear_velocity"] >= breakaway
    assert rpp["regulated_linear_scaling_min_speed"] >= breakaway
    assert rpp["rotate_to_heading_angular_vel"] >= ENVELOPE["tracking"]["breakaway_yaw_radps"]


def test_every_controller_checks_the_full_footprint():
    # The profiles are 1.24 m rectangles; a circle or the base-origin cell is not enough.
    assert CONTROLLERS["rpp"]["use_collision_detection"] is True
    critics = CONTROLLERS["dwb"]["critics"]
    assert "ObstacleFootprint" in critics and "BaseObstacle" not in critics
    mppi = CONTROLLERS["mppi"]
    assert "CostCritic" in mppi["critics"] and mppi["CostCritic"]["consider_footprint"] is True


def test_controllers_agree_with_the_controller_server():
    server = params("controller_server")
    period = 1.0 / server["controller_frequency"]
    dwb = CONTROLLERS["dwb"]
    assert dwb["xy_goal_tolerance"] == server["general_goal_checker"]["xy_goal_tolerance"]
    assert dwb["trajectory_generator_name"] == "dwb_plugins::StandardTrajectoryGenerator"
    mppi = CONTROLLERS["mppi"]
    assert mppi["model_dt"] == pytest.approx(period)
    assert mppi["prune_distance"] >= mppi["time_steps"] * mppi["model_dt"] * mppi["vx_max"]


def test_launch_loads_one_controller_under_the_tree_controller_id():
    server = params("controller_server")
    assert server["controller_plugins"] == ["FollowPath"] and "FollowPath" not in server
    for tree in TREES.values():
        assert next(tree.iter("ControllerSelector")).get("default_controller") == "FollowPath"
    launch = (ROOT / "launch" / "navigation.launch.py").read_text()
    assert 'DeclareLaunchArgument("controller", default_value="rpp"' in launch
    assert '{"FollowPath": controllers[controller]}' in launch


def test_command_chain_matches_adr_0006():
    for node in ("controller_server", "velocity_smoother", "collision_monitor", "behavior_server"):
        assert params(node)["enable_stamped_cmd_vel"] is False
    monitor = params("collision_monitor")
    assert (monitor["cmd_vel_in_topic"], monitor["cmd_vel_out_topic"]) == ("cmd_vel_smoothed", "cmd_vel")
    launch = (ROOT / "launch" / "navigation.launch.py").read_text()
    assert 'chain = [("cmd_vel", "cmd_vel_nav")]' in launch
    assert launch.count("remappings=chain") == 3  # controller, behaviors, smoother input


def test_collision_zones_cover_every_profile_with_stop_distance_margin():
    spec = importlib.util.spec_from_file_location("navigation_launch", ROOT / "launch" / "navigation.launch.py")
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    monitor = params("collision_monitor")
    stop, slow = monitor["StopZone"], monitor["SlowdownZone"]
    assert stop["action_type"] == "stop" and slow["action_type"] == "slowdown"
    # Explicit-zero stops from 0.3 m/s travelled at most 0.07 m (base measurements);
    # the slowdown zone halves speed well before the stop zone is reached.
    assert slow["margin_m"] >= 0.23 and stop["margin_m"] > 0.0 and slow["margin_m"] > stop["margin_m"]
    assert monitor["livox"]["topic"] == "/livox/points_filtered"
    approach = monitor["FootprintApproach"]
    assert approach["action_type"] == "approach"
    assert approach["footprint_topic"] == "/local_costmap/published_footprint"
    # A footprint corner sweeping at ~0.3 m/s must be caught more than a scan ahead.
    assert approach["time_before_collision"] >= 1.0 and approach["simulation_time_step"] <= 0.1
    for profile in PROFILES.values():
        zone = launch.padded(profile["polygon"], stop["margin_m"])
        for (x, y), (zx, zy) in zip(profile["polygon"], zone):
            assert abs(zx) > abs(x) and abs(zy) > abs(y)


def test_behavior_trees_and_server_exclude_unqualified_recoveries():
    for tree in TREES.values():
        tags = {element.tag for element in tree.iter()}
        assert not tags & {"Spin", "BackUp", "DriveOnHeading", "AssistedTeleop"}
        assert "Wait" in tags and "ClearEntireCostmap" in tags
        assert next(tree.iter("PlannerSelector")).get("default_planner") == "Lattice"
    # The default tree keeps its path until the goal changes or the path becomes invalid.
    default = {element.tag for element in TREES["replan_if_invalid_wait_clear"].iter()}
    assert {"IsPathValid", "GlobalUpdatedGoal"} <= default
    launch = (ROOT / "launch" / "navigation.launch.py").read_text()
    assert 'DeclareLaunchArgument("behavior_tree", default_value="replan_if_invalid"' in launch
    assert params("behavior_server")["behavior_plugins"] == ["wait"]
    assert params("bt_navigator")["navigators"] == ["navigate_to_pose"]
    lifecycle = params("lifecycle_manager_navigation")["node_names"]
    assert set(lifecycle) == {"controller_server", "behavior_server", "velocity_smoother",
                              "collision_monitor", "bt_navigator"}


# --- Baseline B5 (controller:=wholebody) -------------------------------------------------

def load_launch(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "launch" / f"{name}.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda package: str(ROOT)
    return module


def evaluate(value, context):
    """A launch parameter or remapping with its substitutions performed."""
    if hasattr(value, "perform"):
        return value.perform(context)
    if isinstance(value, (tuple, list)):
        parts = [evaluate(item, context) for item in value]
        return "".join(parts) if value and all(hasattr(item, "perform") for item in value) else parts
    return value


class Launched:
    """navigation.launch.py's Node actions for the given arguments, by executable."""

    def __init__(self, controller, **arguments):
        from launch import LaunchContext
        from launch.actions import TimerAction
        self.context = LaunchContext()
        self.context.launch_configurations.update({
            "footprint_profile": "home", "behavior_tree": "replan_if_invalid",
            "global_obstacles": "persistent", "controller": controller, "footprint_mode": "static",
            "footprint_model": "mesh", "scenario": "", "references": "none", "mpc_stats_file": "", "mpc_rate_hz": "20.0",
            "slowdown_zone": "true", "posture_mode": "", "comfort_pose": "", "clearance_weight": "",
            "slack_gate": "", "site": "", **arguments})
        actions = []
        for action in load_launch("navigation")._nodes(self.context):
            actions += [inner for inner in action.actions] if isinstance(action, TimerAction) else [action]
        self.nodes = {evaluate(action._Node__node_executable, self.context): action
                      for action in actions if hasattr(action, "_Node__node_executable")}

    def value(self, value):
        if hasattr(value, "evaluate"):  # ParameterValue: kept as typed text, never parsed as YAML
            return value.evaluate(self.context)
        value = evaluate(value, self.context)
        if isinstance(value, list):
            return [yaml.safe_load(item) if isinstance(item, str) else item for item in value]
        return yaml.safe_load(value) if isinstance(value, str) else value

    def parameters(self, executable):
        merged = {}
        for entry in self.nodes[executable]._Node__parameters:
            if isinstance(entry, dict):
                # launch_ros dumps every parameter value as YAML text.
                merged.update({evaluate(key, self.context): self.value(value) for key, value in entry.items()})
        return merged

    def remappings(self, executable):
        return [tuple(evaluate(part, self.context) for part in pair)
                for pair in self.nodes[executable]._Node__remappings]


def test_wholebody_launch_has_no_controller_server_or_smoother():
    wholebody = Launched("wholebody")
    assert not set(wholebody.nodes) & {"controller_server", "velocity_smoother"}
    assert {"wholebody_mpc_node", "behavior_server", "collision_monitor", "bt_navigator",
            "lifecycle_manager"} <= set(wholebody.nodes)
    # The MPC node is the one command publisher upstream of the collision monitor (ADR 0006); the
    # behaviour server's stop commands go to the dead-end cmd_vel_nav.
    assert ("cmd_vel_wholebody", "cmd_vel_smoothed") in wholebody.remappings("wholebody_mpc_node")
    assert params("collision_monitor")["cmd_vel_in_topic"] == "cmd_vel_smoothed"
    assert ("cmd_vel", "cmd_vel_nav") in wholebody.remappings("behavior_server")
    mpc = wholebody.parameters("wholebody_mpc_node")
    assert mpc["scenario"] == "" and mpc["references"] == "none" and mpc["stats_file"] == ""
    # wholebody_mpc is a plain rclcpp node: the lifecycle manager must not manage it.
    assert wholebody.parameters("lifecycle_manager")["node_names"] == [
        "behavior_server", "collision_monitor", "bt_navigator"]
    assert "wholebody_mpc" not in wholebody.parameters("lifecycle_manager")["node_names"]
    # The pass-through of scenario and references.
    scripted = Launched("wholebody", scenario="overhead_course_nav", references="scripted").parameters("wholebody_mpc_node")
    assert scripted["scenario"] == "overhead_course_nav" and scripted["references"] == "scripted"
    # The runner collects the timing summary the MPC writes on shutdown.
    stats = Launched("wholebody", mpc_stats_file="/tmp/mm_mpc_stats.json").parameters("wholebody_mpc_node")
    assert stats["stats_file"] == "/tmp/mm_mpc_stats.json"
    # The loop rate (20 Hz bring-up, 50 Hz timing runs) reaches the node as a double.
    assert mpc["rate_hz"] == 20.0
    assert Launched("wholebody", mpc_rate_hz="50").parameters("wholebody_mpc_node")["rate_hz"] == 50.0
    # B1-B4 keep the controller server and the smoother, and the lifecycle manager's own list.
    for controller in ("rpp", "dwb", "mppi"):
        classic = Launched(controller)
        assert {"controller_server", "velocity_smoother"} <= set(classic.nodes)
        assert "wholebody_mpc_node" not in classic.nodes
        assert "node_names" not in classic.parameters("lifecycle_manager")
        assert "livox.max_height" not in classic.parameters("collision_monitor")


def test_wholebody_forces_the_wholebody_footprint_mode_and_nothing_else_may_use_it():
    with pytest.raises(RuntimeError, match="wholebody"):
        Launched("rpp", footprint_mode="wholebody")
    # Whatever footprint_mode says, controller:=wholebody makes the collision zones follow the footprint
    # node's topics (the same wiring as profiles and dynamic).
    for mode in ("static", "dynamic"):
        zones = Launched("wholebody", footprint_mode=mode).parameters("collision_monitor")
        assert zones["StopZone.polygon_sub_topic"] == "/collision_monitor/stop_zone_in"
        assert zones["SlowdownZone.polygon_sub_topic"] == "/collision_monitor/slowdown_zone_in"
        assert zones["FootprintApproach.footprint_topic"] == "/global_costmap/published_footprint"


def test_wholebody_tree_has_no_local_costmap_clear_and_points_follow_path_at_mpc():
    tree = ET.parse(ROOT / "behavior_trees" / "navigate_to_pose_wholebody.xml")
    services = [element.get("service_name") for element in tree.iter("ClearEntireCostmap")]
    assert services and all("local_costmap" not in service for service in services)
    follow = list(tree.iter("FollowPath"))
    assert len(follow) == 1 and follow[0].get("server_name") == "wholebody_mpc/follow_path"
    tags = {element.tag for element in tree.iter()}
    assert not tags & {"Spin", "BackUp", "DriveOnHeading", "AssistedTeleop"}
    assert "Wait" in tags and {"IsPathValid", "GlobalUpdatedGoal"} <= tags
    assert next(tree.iter("PlannerSelector")).get("default_planner") == "Lattice"
    launch = (ROOT / "launch" / "navigation.launch.py").read_text()
    assert "navigate_to_pose_wholebody.xml" in launch


def test_base_band_heights():
    planning = yaml.safe_load((ROOT / "config" / "nav2_global_planning.yaml").read_text())
    layers = planning["global_costmap"]["global_costmap"]["ros__parameters"]
    band = layers["base_band"]
    assert (band["min_obstacle_height"], band["max_obstacle_height"]) == (0.05, 0.60)
    # B1-B4 keep the full 0.05-2.0 m band.
    assert layers["persistent_obstacle_layer"]["max_obstacle_height"] == 2.0
    assert layers["obstacle_layer"]["livox_mark"]["max_obstacle_height"] == 2.0
    assert params("collision_monitor")["livox"]["max_height"] == 2.0
    # The wholebody launch overrides the global costmap layers and the monitor's livox source.
    overrides = load_launch("global_planning").band_overrides(band)
    ros = overrides["global_costmap"]["global_costmap"]["ros__parameters"]
    assert ros["persistent_obstacle_layer"] == {"min_obstacle_height": 0.05, "max_obstacle_height": 0.60}
    assert ros["obstacle_layer"]["livox_mark"] == {"min_obstacle_height": 0.05, "max_obstacle_height": 0.60}
    monitor = Launched("wholebody").parameters("collision_monitor")
    assert monitor["livox.max_height"] == 0.60 and monitor["livox.min_height"] == 0.05


def test_slowdown_zone_switch_disables_only_the_slowdown_polygon():
    launch = (ROOT / "launch" / "navigation.launch.py").read_text()
    assert 'DeclareLaunchArgument("slowdown_zone", default_value="true"' in launch
    monitor = params("collision_monitor")
    assert monitor["polygons"] == ["StopZone", "SlowdownZone", "FootprintApproach"]
    assert all(monitor[zone]["enabled"] is True for zone in monitor["polygons"])
    for controller in ("rpp", "dwb", "mppi", "wholebody"):
        for mode in (("static", "dynamic") if controller != "wholebody" else ("static",)):
            default = Launched(controller, footprint_mode=mode).parameters("collision_monitor")
            off = Launched(controller, footprint_mode=mode, slowdown_zone="false").parameters("collision_monitor")
            # Default (every recorded B1-B5 run): nothing about the zones' enabled state is overridden.
            assert not any(key.endswith(".enabled") for key in default), (controller, mode)
            # Off: the slowdown polygon is disabled and nothing else changes; it stays declared with its
            # points or input topic, so ownership and the zone-topic checks see the same graph.
            assert off.pop("SlowdownZone.enabled") is False, (controller, mode)
            assert off == default, (controller, mode)
            assert "polygons" not in off
    for value in ("TRUE", "True"):
        assert Launched("rpp", slowdown_zone=value).parameters("collision_monitor") == Launched(
            "rpp").parameters("collision_monitor")
    with pytest.raises(RuntimeError, match="slowdown_zone"):
        Launched("rpp", slowdown_zone="off")


def test_mpc_posture_options_reach_the_node_only_when_given():
    # Not given (every recorded B5 run): the node keeps wholebody_mpc.yaml's posture preference.
    default = Launched("wholebody").parameters("wholebody_mpc_node")
    assert not {"posture_mode", "comfort_pose", "clearance_weight", "slack_gate"} & set(default)
    given = Launched("wholebody", posture_mode="comfort_only", comfort_pose="carry").parameters("wholebody_mpc_node")
    assert given["posture_mode"] == "comfort_only" and given["comfort_pose"] == "carry"
    assert Launched("wholebody", posture_mode="none").parameters("wholebody_mpc_node")["posture_mode"] == "none"
    # The clearance weight reaches the node as a double, also when written as an integer.
    assert Launched("wholebody", clearance_weight="20").parameters("wholebody_mpc_node")["clearance_weight"] == 20.0
    assert {key: value for key, value in given.items() if key not in ("posture_mode", "comfort_pose")} == default
    # The slack gate reaches the node as a boolean (the recorded B5 also needs comfort_pose:=home).
    assert Launched("wholebody", slack_gate="false").parameters("wholebody_mpc_node")["slack_gate"] is False
    assert Launched("wholebody", slack_gate="true").parameters("wholebody_mpc_node")["slack_gate"] is True


# --- Site selection (construction site V2) ------------------------------------------------

def load_site_selection():
    spec = importlib.util.spec_from_file_location("site_selection", ROOT / "launch" / "site_selection.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def v2_share(tmp_path):
    (tmp_path / "config").mkdir()
    (tmp_path / "maps").mkdir()
    (tmp_path / "config" / "scenarios.yaml").write_text("scenarios:\n  v1_case: {}\n")
    (tmp_path / "config" / "scenarios_site_v2.yaml").write_text(
        "site: construction_site_v2\nscenarios:\n  v2_case: {}\n")
    for site in ("construction_site", "construction_site_v2"):
        (tmp_path / "maps" / f"{site}.yaml").write_text("image: x.pgm\n")
    return tmp_path


def test_site_map_yaml_resolves_v1():
    assert load_site_selection().site_map_yaml(ROOT, "construction_site") == str(
        ROOT / "maps" / "construction_site.yaml")


def test_site_map_yaml_rejects_unknown_site(v2_share):
    with pytest.raises(RuntimeError, match=r"Unknown site 'nowhere'.*construction_site.*construction_site_v2"):
        load_site_selection().site_map_yaml(v2_share, "nowhere")


def test_launch_site_defaults_to_v1():
    selection = load_site_selection()
    assert selection.launch_site(ROOT, "", "") == "construction_site"
    assert selection.launch_site(ROOT, "", next(iter(
        yaml.safe_load((ROOT / "config" / "scenarios.yaml").read_text())["scenarios"]))) == "construction_site"
    assert selection.launch_site(ROOT, "construction_site_v2", "") == "construction_site_v2"


def test_launch_site_follows_the_scenario(v2_share):
    selection = load_site_selection()
    assert selection.scenario_site(v2_share, "v2_case") == "construction_site_v2"
    assert selection.scenario_site(v2_share, "v1_case") == "construction_site"
    assert selection.launch_site(v2_share, "", "v2_case") == "construction_site_v2"
    assert selection.launch_site(v2_share, "construction_site_v2", "v2_case") == "construction_site_v2"
    with pytest.raises(RuntimeError, match="nope"):
        selection.scenario_site(v2_share, "nope")


def test_launch_site_mismatch_raises(v2_share):
    with pytest.raises(RuntimeError, match="site construction_site differs from scenario v2_case's site construction_site_v2"):
        load_site_selection().launch_site(v2_share, "construction_site", "v2_case")


def test_launch_files_declare_site():
    from launch.actions import DeclareLaunchArgument
    manipulation = Path(__file__).resolve().parents[2] / "mobile_manipulator_manipulation" / "launch" / "manipulation.launch.py"
    for path in (ROOT / "launch" / "global_planning.launch.py", ROOT / "launch" / "navigation.launch.py", manipulation):
        spec = importlib.util.spec_from_file_location(path.stem, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        declared = {action.name: action.default_value for action in module.generate_launch_description().entities
                    if isinstance(action, DeclareLaunchArgument)}
        assert "site" in declared, path.name
        assert "".join(part.text for part in declared["site"]) == "", path.name
