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


def test_lidar_intrusion_is_the_measured_value():
    # docs/experiments/wholebody-mpc "Lidar intrusion at the StopZone" (2026-10-18): q = 0.08 m.
    assert MPC["collision"]["lidar_intrusion_m"] == 0.08
    assert MPC["collision"]["lidar_intrusion_m"] <= 0.08  # the spec's stop rule
