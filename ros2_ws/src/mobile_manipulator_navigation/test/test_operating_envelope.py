import json
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ENVELOPE = yaml.safe_load(
    (PACKAGE_ROOT / "config" / "nav_operating_envelope.yaml").read_text(encoding="utf-8")
)
FOOTPRINTS = yaml.safe_load(
    (PACKAGE_ROOT / "config" / "footprint_profiles.yaml").read_text(encoding="utf-8")
)
PAYLOAD = json.loads(
    (Path(get_package_share_directory("mobile_manipulator_control"))
     / "config" / "qualified_payload.json").read_text(encoding="utf-8")
)

URDF = ET.parse(
    Path(get_package_share_directory("mobile_manipulator_description")) / "urdf" / "mobile_manipulator.urdf"
).getroot()

# Unity SkidSteerBaseController actuator limits (design ledger base motion table).
ACTUATOR = {"forward": 0.8, "reverse": 0.5, "yaw": 0.8,
            "linear_accel": 0.5, "linear_decel": 0.8, "yaw_accel": 0.8, "yaw_decel": 1.2}


def test_every_footprint_profile_has_a_qualified_envelope():
    # base_only is a footprint strategy, not an arm pose: the arm then still holds a qualified
    # pose whose envelope applies.
    arm_poses = {name for name, profile in FOOTPRINTS["profiles"].items() if not profile.get("base_only")}
    assert set(ENVELOPE["profiles"]) == arm_poses
    for name, profile in ENVELOPE["profiles"].items():
        assert profile["footprint_profile"] == name


def test_speeds_do_not_exceed_tested_payload_commands():
    tested = PAYLOAD["base_commands_tested"]
    for name, profile in ENVELOPE["profiles"].items():
        assert name in tested, f"{name} has no recorded payload qualification"
        assert profile["max_forward_mps"] <= tested[name]["forward_mps"]
        assert profile["max_reverse_mps"] <= tested[name]["forward_mps"]
        assert profile["max_yaw_radps"] <= tested[name]["yaw_radps"]


def test_envelope_stays_inside_unity_actuator_limits():
    for profile in ENVELOPE["profiles"].values():
        assert profile["max_forward_mps"] <= ACTUATOR["forward"]
        assert profile["max_reverse_mps"] <= ACTUATOR["reverse"]
        assert profile["max_yaw_radps"] <= ACTUATOR["yaw"]
    acceleration = ENVELOPE["acceleration"]
    assert acceleration["linear_accel_mps2"] <= ACTUATOR["linear_accel"]
    assert acceleration["linear_decel_mps2"] <= ACTUATOR["linear_decel"]
    assert acceleration["yaw_accel_radps2"] <= ACTUATOR["yaw_accel"]
    assert acceleration["yaw_decel_radps2"] <= ACTUATOR["yaw_decel"]
    assert ENVELOPE["safety"]["worst_case_linear_decel_mps2"] <= acceleration["linear_decel_mps2"]


def urdf_limits():
    joints = PAYLOAD["joint_order"]
    limits = {joint.get("name"): joint.find("limit") for joint in URDF.iter("joint") if joint.get("name") in joints}
    assert set(limits) == set(joints)
    return limits


def test_wholebody_arm_limits_within_urdf():
    arm = ENVELOPE["wholebody_arm"]
    joints = PAYLOAD["joint_order"]
    velocity_limit = {name: float(limit.get("velocity")) for name, limit in urdf_limits().items()}
    assert len(arm["max_joint_velocity_radps"]) == len(joints)
    assert len(arm["max_joint_accel_radps2"]) == len(joints)
    for name, velocity, acceleration in zip(
            joints, arm["max_joint_velocity_radps"], arm["max_joint_accel_radps2"]):
        assert 0 < velocity <= velocity_limit[name], name
        assert acceleration > 0, name
    assert arm["evidence"]


def test_wholebody_arm_joint_bounds_are_ordered_and_inside_urdf_limits():
    bounds = ENVELOPE["wholebody_arm"].get("joint_bounds")
    if bounds is None:  # every extra held pose held; the URDF limits then apply unchanged
        return
    limits = urdf_limits()
    joints = PAYLOAD["joint_order"]
    assert len(bounds) == len(joints)
    for name, (low, high) in zip(joints, bounds):
        assert float(limits[name].get("lower")) <= low < high <= float(limits[name].get("upper")), name
