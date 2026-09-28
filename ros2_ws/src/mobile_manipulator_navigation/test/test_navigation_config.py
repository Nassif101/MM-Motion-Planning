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
    # MPPI's forward-acceleration model is stock (see nav2_controllers.yaml); the velocity
    # smoother enforces the envelope on its commands. Braking and yaw stay in the envelope.
    assert -mppi["ax_min"] <= acceleration["linear_decel_mps2"]
    assert mppi["az_max"] <= acceleration["yaw_accel_radps2"]


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
