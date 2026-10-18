import pytest

from measure_lidar_intrusion import kth_deepest_intrusion, q_from


def test_kth_deepest_intrusion_per_scan():
    # face at x = 1.0, robot on the -x side: intrusion = 1.0 - x for points with x < 1.0
    scan = [(0.97, 0.0), (0.99, 0.1), (0.95, -0.1), (1.02, 0.0), (0.98, 0.2)]
    assert kth_deepest_intrusion(scan, face_x=1.0, k=4) == pytest.approx(0.01)


def test_scan_with_fewer_than_k_points_counts_zero():
    assert kth_deepest_intrusion([(0.9, 0.0)], face_x=1.0, k=4) == 0.0


def test_points_behind_the_face_do_not_count():
    assert kth_deepest_intrusion([(1.01, 0.0)] * 6, face_x=1.0, k=4) == 0.0


def test_q_is_the_999th_percentile_rounded_up_to_a_centimetre():
    assert q_from(per_scan=[0.01] * 998 + [0.043] * 2) == pytest.approx(0.05)


def test_q_of_no_intrusion_is_zero():
    assert q_from(per_scan=[0.0] * 100) == 0.0


def test_points_far_in_front_of_the_face_are_other_obstacles():
    # A wall 9 m away on the robot's other side is not intrusion into this face.
    scan = [(-8.0, 0.0)] * 6 + [(0.99, 0.0)] * 3
    assert kth_deepest_intrusion(scan, face_x=1.0, k=4) == 0.0


def test_a_tall_fixture_inside_the_carry_panel_footprint_is_invalid():
    from measure_lidar_intrusion import overlaps_carry_panel
    # Carry panel: x -0.68..0.52, y +-0.60 m, bottom 1.27 m.
    assert overlaps_carry_panel("front_wall", 0.05, height=2.0)   # face at 0.51 m
    assert not overlaps_carry_panel("front_wall", 0.08, height=2.0)
    assert overlaps_carry_panel("side_wall", 0.20, height=2.0)    # face at 0.585 m
    assert not overlaps_carry_panel("side_wall", 0.22, height=2.0)
    assert not overlaps_carry_panel("front_wall", 0.05, height=1.0)  # below the panel


def test_q_is_the_largest_valid_row_percentile():
    from measure_lidar_intrusion import q_from_rows
    rows = [dict(fixture="front_wall", hull_distance_m=0.05, height=2.0, p999=0.07),
            dict(fixture="front_wall", hull_distance_m=0.08, height=2.0, p999=0.04),
            dict(fixture="side_wall", hull_distance_m=0.08, height=1.0, p999=0.03)]
    assert q_from_rows(rows) == pytest.approx(0.04)

