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


def test_kth_nearest_to_the_hull_counts_every_side():
    from measure_lidar_intrusion import kth_nearest_to_hull
    # Hull half sizes 0.46 x 0.385: two points ahead, two to the left, two to the right, one far away.
    points = [(0.50, 0.0), (0.52, 0.1), (0.0, 0.42), (0.1, 0.45), (0.0, -0.40), (0.0, -0.43), (3.0, 0.0)]
    # distances: 0.04, 0.06, 0.035, 0.065, 0.015, 0.045, 2.54 -> sorted 0.015, 0.035, 0.04, 0.045
    assert kth_nearest_to_hull(points, 0.46, 0.385, k=4) == pytest.approx(0.045)


def test_kth_nearest_to_the_hull_measures_corners_euclidean():
    from measure_lidar_intrusion import kth_nearest_to_hull
    assert kth_nearest_to_hull([(0.49, 0.425)] * 4, 0.46, 0.385, k=4) == pytest.approx(0.05)


def test_kth_nearest_without_enough_points_is_infinite():
    import math
    from measure_lidar_intrusion import kth_nearest_to_hull
    assert math.isinf(kth_nearest_to_hull([(0.5, 0.0)], 0.46, 0.385, k=4))


def test_drift_measures_distance_and_heading_from_the_start():
    import math
    from measure_lidar_intrusion import START, drift
    off, turned = drift((START[0] + 0.003, START[1] - 0.004, START[2] + math.radians(0.1)))
    assert off == pytest.approx(0.005)
    assert math.degrees(turned) == pytest.approx(0.1)
