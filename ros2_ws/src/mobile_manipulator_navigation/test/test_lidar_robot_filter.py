import math
from pathlib import Path
import sys

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mobile_manipulator_navigation.lidar_robot_filter import footprint_bounds, keep_mask  # noqa: E402

PROFILES = yaml.safe_load((Path(__file__).resolve().parents[1] / "config" /
                           "footprint_profiles.yaml").read_text())["profiles"]
# livox_frame in base_footprint: xyz (0.24, 0, 0.387), no rotation (design ledger).
ROTATION = np.eye(3)
TRANSLATION = np.array([0.24, 0.0, 0.387])
BAND = (0.05, 2.0)


def keep(points, profile="home", band=None):
    bounds = footprint_bounds(PROFILES[profile]["polygon"])
    return keep_mask(np.array(points, dtype=float), ROTATION, TRANSLATION, bounds, band).tolist()


def test_drops_zero_point_misses_but_keeps_real_returns():
    assert keep([[0, 0, 0], [3.0, 0, 0]]) == [False, True]


def test_drops_self_returns_inside_the_active_footprint_column():
    # Panel point 1.2 m above the deck, 0.2 m behind the sensor: inside both profiles.
    assert keep([[-0.2, 0.3, 1.2]], "home") == [False]
    # 0.5 m to the side is inside home (+/-0.62) but outside vertical_carry (+/-0.385).
    assert keep([[0.0, 0.5, 0.0]], "home") == [False]
    assert keep([[0.0, 0.5, 0.0]], "vertical_carry") == [True]


GROUND = [3.0, 0.0, -0.387 + 0.01]      # 0.01 m above the ground
LOW = [3.0, 0.0, -0.387 + 0.10]         # 0.10 m obstacle
OVERHEAD = [3.0, 0.0, -0.387 + 2.5]     # above the robot


def test_keeps_ground_and_overhead_returns_for_costmap_clearing_by_default():
    assert keep([GROUND, LOW, OVERHEAD]) == [True, True, True]


def test_optional_height_band_is_applied_in_base_footprint():
    assert keep([GROUND, LOW, OVERHEAD], band=BAND) == [False, True, False]


def test_uses_the_full_sensor_transform():
    # A sensor yawed by 90 deg: a point 1 m along sensor +x lies 1 m to the robot's left.
    yaw = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    bounds = footprint_bounds(PROFILES["vertical_carry"]["polygon"])
    point = np.array([[1.0, 0.0, 0.0]])
    assert keep_mask(point, yaw, TRANSLATION, bounds).tolist() == [True]
    assert math.isclose((point @ yaw.T + TRANSLATION)[0, 1], 1.0)
