"""Consistency of the MoveIt configuration with the robot, payload and controller contracts."""
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE_ROOT / "config"
SRDF = ET.parse(CONFIG / "mobile_manipulator.srdf").getroot()
DESCRIPTION = Path(get_package_share_directory("mobile_manipulator_description"))
CONTROL = Path(get_package_share_directory("mobile_manipulator_control"))
URDF = ET.parse(DESCRIPTION / "urdf" / "mobile_manipulator.urdf").getroot()
PAYLOAD = json.loads((CONTROL / "config" / "qualified_payload.json").read_text(encoding="utf-8"))
CONTROLLERS = yaml.safe_load((CONTROL / "config" / "controllers.yaml").read_text(encoding="utf-8"))
LEDGER = (DESCRIPTION / "docs" / "design-ledger.md").read_text(encoding="utf-8")
ARM_JOINTS = CONTROLLERS["arm_controller"]["ros__parameters"]["joints"]
ARM_LINKS = {"shoulder_pan_link", "upper_arm_link", "forearm_link",
             "wrist_1_link", "wrist_2_link", "wrist_3_link"}


def load_yaml(name):
    return yaml.safe_load((CONFIG / name).read_text(encoding="utf-8"))


def ledger_limits():
    """Joint -> (max acceleration, max jerk) from the design ledger's motion-limit table."""
    section = LEDGER.split("### Arm motion-limit contract", 1)[1].split("\n### ", 1)[0]
    limits = {}
    for row in re.findall(r"^\| `(\w+)` \|(.+)\|$", section, flags=re.MULTILINE):
        cells = [cell.strip() for cell in row[1].split("|")]
        limits[row[0]] = (float(cells[2]), float(cells[3]))
    return limits


def test_named_states_match_qualified_payload():
    states = {state.get("name"): state for state in SRDF.findall("group_state")
              if state.get("group") == "arm"}
    for name in ("home", "vertical_carry", "level_extension"):
        values = {joint.get("name"): float(joint.get("value"))
                  for joint in states[name].findall("joint")}
        expected = dict(zip(PAYLOAD["joint_order"], PAYLOAD["poses_rad"][name]))
        assert values.keys() == expected.keys(), name
        for joint, value in expected.items():
            assert abs(values[joint] - value) < 1e-9, (name, joint)


def test_group_joints_match_controller_order():
    group = next(g for g in SRDF.findall("group") if g.get("name") == "arm")
    chain = group.find("chain")
    assert (chain.get("base_link"), chain.get("tip_link")) == ("arm_mount_link", "tool0")
    parent = {j.find("child").get("link"): j for j in URDF.findall("joint")}
    joints, link = [], "tool0"
    while link != "arm_mount_link":
        joint = parent[link]
        if joint.get("type") != "fixed":
            joints.insert(0, joint.get("name"))
        link = joint.find("parent").get("link")
    assert joints == ARM_JOINTS == PAYLOAD["joint_order"]


def test_virtual_joint_is_planar_map_to_base_footprint():
    (virtual,) = SRDF.findall("virtual_joint")
    assert (virtual.get("type"), virtual.get("parent_frame"), virtual.get("child_link")) == (
        "planar", "map", "base_footprint")


def test_joint_limits_match_design_ledger():
    config = load_yaml("joint_limits.yaml")
    assert config["default_velocity_scaling_factor"] == 0.5
    assert config["default_acceleration_scaling_factor"] == 0.5
    expected = ledger_limits()
    assert set(expected) == set(ARM_JOINTS)
    for joint, (acceleration, jerk) in expected.items():
        limits = config["joint_limits"][joint]
        assert limits["has_acceleration_limits"] and limits["has_jerk_limits"], joint
        assert limits["max_acceleration"] == acceleration, joint
        assert limits["max_jerk"] == jerk, joint


def test_disabled_collisions_only_name_real_links():
    with_geometry = {link.get("name") for link in URDF.findall("link")
                     if link.find("collision") is not None}
    pairs = SRDF.findall("disable_collisions")
    assert pairs
    for pair in pairs:
        links = {pair.get("link1"), pair.get("link2")}
        assert links <= with_geometry, links
        assert pair.get("reason") in ("Adjacent", "Never"), links
        if "base_link" in links and links & (ARM_LINKS - {"shoulder_pan_link"}):
            assert pair.get("reason") == "Never", links


def test_pipeline():
    ompl = load_yaml("ompl_planning.yaml")
    assert ompl["planning_plugins"] == ["ompl_interface/OMPLPlanner"]
    assert ompl["arm"]["default_planner_config"] == "RRTConnectkConfigDefault"
    assert "default_planning_response_adapters/AddRuckigTrajectorySmoothing" in ompl["response_adapters"]
    controllers = load_yaml("moveit_controllers.yaml")
    manager = controllers["moveit_simple_controller_manager"]
    assert manager["controller_names"] == ["arm_controller"]
    arm = manager["arm_controller"]
    assert (arm["type"], arm["action_ns"]) == ("FollowJointTrajectory", "follow_joint_trajectory")
    assert arm["joints"] == ARM_JOINTS


def test_octomap_sensor():
    # Loaded only with scene_source:=octomap (a config/sensors_3d.yaml would always load).
    assert not (CONFIG / "sensors_3d.yaml").exists()
    sensors = load_yaml("sensors_3d_octomap.yaml")
    (name,) = sensors["sensors"]
    livox = sensors[name]
    assert livox["sensor_plugin"] == "occupancy_map_monitor/PointCloudOctomapUpdater"
    assert livox["point_cloud_topic"] == "/livox/points_filtered"
    assert livox["max_range"] == 5.0
    assert sensors["octomap_frame"] == "map"
    assert sensors["octomap_resolution"] == 0.05
