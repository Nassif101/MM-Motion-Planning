import json
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

# Unity SkidSteerBaseController actuator limits (design ledger base motion table).
ACTUATOR = {"forward": 0.8, "reverse": 0.5, "yaw": 0.8,
            "linear_accel": 0.5, "linear_decel": 0.8, "yaw_accel": 0.8, "yaw_decel": 1.2}


def test_every_footprint_profile_has_a_qualified_envelope():
    assert set(ENVELOPE["profiles"]) == set(FOOTPRINTS["profiles"])
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
