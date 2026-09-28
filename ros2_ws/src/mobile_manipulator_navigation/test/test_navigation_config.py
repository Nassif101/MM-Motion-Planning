import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
NAV = yaml.safe_load((ROOT / "config" / "nav2_navigation.yaml").read_text())
ENVELOPE = yaml.safe_load((ROOT / "config" / "nav_operating_envelope.yaml").read_text())
PROFILES = yaml.safe_load((ROOT / "config" / "footprint_profiles.yaml").read_text())["profiles"]
TREE = ET.parse(ROOT / "behavior_trees" / "navigate_to_pose_wait_clear_recovery.xml")


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

    rpp = params("controller_server")["FollowPath"]
    assert rpp["desired_linear_vel"] <= strictest("max_forward_mps")
    assert rpp["rotate_to_heading_angular_vel"] <= strictest("max_yaw_radps")
    assert rpp["max_angular_accel"] <= acceleration["yaw_accel_radps2"]
    assert rpp["allow_reversing"] is False


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


def test_behavior_tree_and_server_exclude_unqualified_recoveries():
    tags = {element.tag for element in TREE.iter()}
    assert not tags & {"Spin", "BackUp", "DriveOnHeading", "AssistedTeleop"}
    assert "Wait" in tags and "ClearEntireCostmap" in tags
    selector = next(TREE.iter("PlannerSelector"))
    assert selector.get("default_planner") == "Lattice"
    assert params("behavior_server")["behavior_plugins"] == ["wait"]
    assert params("bt_navigator")["navigators"] == ["navigate_to_pose"]
    lifecycle = params("lifecycle_manager_navigation")["node_names"]
    assert set(lifecycle) == {"controller_server", "behavior_server", "velocity_smoother",
                              "collision_monitor", "bt_navigator"}
