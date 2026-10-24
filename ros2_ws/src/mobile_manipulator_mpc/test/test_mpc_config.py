"""wholebody_mpc.yaml against the navigation configuration it is coupled to (Phase 4B-A spec Section 2)."""
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
NAVIGATION = ROOT.parent / "mobile_manipulator_navigation"
MPC = yaml.safe_load((ROOT / "config" / "wholebody_mpc.yaml").read_text())
MONITOR = yaml.safe_load((NAVIGATION / "config" / "nav2_navigation.yaml").read_text())[
    "collision_monitor"]["ros__parameters"]
FOOTPRINT = yaml.safe_load((NAVIGATION / "config" / "dynamic_footprint.yaml").read_text())[
    "dynamic_footprint_node"]["ros__parameters"]


def test_base_margin_is_padding_plus_stop_margin_plus_intrusion():
    # The MPC's base rows measure from the URDF base primitives; the wholebody StopZone is their hull grown by
    # padding_m and then by its margin.
    collision = MPC["collision"]
    expected = FOOTPRINT["padding_m"] + MONITOR["StopZone"]["wholebody_margin_m"] + collision["lidar_intrusion_m"]
    assert abs(collision["base_margin_m"] - expected) < 1e-9


def test_blocked_detection_values():
    # Phase 4B-A spec Section 2 (amended: commanding and executed thresholds are the base breakaway).
    assert MPC["blocked"] == {"detect_s": 1.0, "fail_s": 3.0, "clear_s": 0.5, "progress_m": 0.05}


def test_lidar_intrusion_is_the_measured_value():
    # docs/experiments/wholebody-mpc "Lidar intrusion at the StopZone" (2026-10-18): q = 0.08 m.
    assert MPC["collision"]["lidar_intrusion_m"] == 0.08
    assert MPC["collision"]["lidar_intrusion_m"] <= 0.08  # the spec's stop rule


def test_new_keys_are_the_chosen_configuration():
    # Phase 4B-B Task 11: the campaign-1 configuration is the default (HPIPM, arm rows, the arm stream's lead); every
    # other switch keeps the frozen B5's value (tag b5-frozen-2026-10-22).
    solver = MPC["solver"]
    assert solver["backend"] == "hpipm" and solver["warm_start"] is True and solver["deadline_cap"] is False
    assert MPC["arm_stream_lead_s"] == 0.0  # ADR 0014: the lead was not adopted (Phase 4B-C)
    # Phase 4B-C's switches are absent (their frozen defaults).
    assert "stream_dense_s" not in MPC and "slack_gate_arm_tolerance_m" not in MPC
    assert "slack_entry" not in MPC["weights"] and "midstage_rows" not in MPC["collision"]
    assert solver["publish_margin_s"] == 0.002
    assert solver["hpipm"] == {"mode": "balance", "max_iter": 50, "partial_condensing_blocks": 0, "warm_start": False}
    assert MPC["qp_iterations"] == 1 and MPC["iter2_budget"] == 0.5 and MPC["iter2_cost_tolerance"] == 0.01
    assert MPC["rti_split"] is False
    assert MPC["rti_max_deviation"] == {"base_m": 0.05, "yaw_rad": 0.05, "joint_rad": 0.05}
    assert MPC["collision"]["self_rows"] is True and MPC["collision"]["self_pairs_per_stage"] == 2
