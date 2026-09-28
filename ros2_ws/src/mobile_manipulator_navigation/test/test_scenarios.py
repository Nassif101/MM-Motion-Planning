import importlib.util
import math
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "scenario_spec.py"
spec = importlib.util.spec_from_file_location("scenario_spec", SCRIPT)
scenario_spec = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scenario_spec)

HOME = scenario_spec.load("footprint_profiles.yaml")["profiles"]["home"]["polygon"]


@pytest.mark.parametrize("name", sorted(scenario_spec.load("scenarios.yaml")["scenarios"]))
def test_every_scenario_is_consistent_and_starts_in_free_space(name):
    resolved = scenario_spec.resolve(name)
    scenario = resolved["scenario"]
    assert resolved["start_free"], resolved["blocked_cells"]
    assert scenario["task"] in ("compute_path", "navigate_to_pose")
    if scenario["task"] == "compute_path":
        assert set(scenario["planners"]) <= {"GridBased", "Lattice"}
    else:
        assert scenario["timeout_s"] > 0
    assert len(scenario["start"]) == 3 and len(scenario["goal"]) == 3


def test_free_space_check_rejects_a_start_inside_the_gate_post():
    # ManipulationRequiredGate_1p05m left post centre: Unity (7.0, -7.225) -> ROS (-7.225, -7.0).
    free, blocked = scenario_spec.start_is_free(HOME, (-7.225, -7.0, math.pi))
    assert not free and blocked


def test_free_space_check_uses_the_rotated_footprint():
    # East lane: 2.4 m between fences, running along ROS x, centred on y = -10.5.
    # The 1.24 x 1.24 m home footprint fits at either heading.
    assert scenario_spec.start_is_free(HOME, (13.0, -10.5, 0.0))[0]
    assert scenario_spec.start_is_free(HOME, (13.0, -10.5, math.pi / 2))[0]
    # A footprint 2.6 m wide in body y fits only when body y points along the lane.
    wide = [[0.54, 1.3], [0.54, -1.3], [-0.70, -1.3], [-0.70, 1.3]]
    assert scenario_spec.start_is_free(wide, (13.0, -10.5, math.pi / 2))[0]
    assert not scenario_spec.start_is_free(wide, (13.0, -10.5, 0.0))[0]
