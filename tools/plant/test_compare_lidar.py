"""Tests of compare_lidar.py on synthetic point sets (MuJoCo plant plan Task 10)."""
import math

import numpy as np
import pytest

import compare_lidar as cl

EYE = np.eye(3).tolist()


def box(label, pos, half, yaw=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    return dict(kind="box", label=label, pos=list(pos), rot=[[c, -s, 0], [s, c, 0], [0, 0, 1]], size=list(half))


def cylinder(label, pos, radius, half_height):
    return dict(kind="cylinder", label=label, pos=list(pos), rot=EYE, size=[radius, half_height, 0.0])


def test_class_fractions():
    fractions = cl.class_fractions(np.array(["ground", "ground", "ground", "site"]))
    assert fractions["ground"] == pytest.approx(0.75) and fractions["site"] == pytest.approx(0.25)
    assert fractions["robot"] == 0.0 and fractions["panel"] == 0.0
    assert sum(fractions.values()) == pytest.approx(1.0)


def test_label_unity_points():
    wall = box("site", (5.0, 0.0, 1.0), (0.1, 2.0, 1.0), yaw=0.3)
    wheel = cylinder("robot", (0.3, 0.3, 0.15), 0.15, 0.05)
    panel = box("panel", (0.0, 0.0, 1.2), (0.4, 0.3, 0.01))
    face = np.array([5.0, 0.0, 1.0]) + 0.11 * np.array([math.cos(0.3), math.sin(0.3), 0.0])  # 1 cm off the face
    points = np.array([face,
                       [9.0, 9.0, 0.01],     # the ground, far from any box
                       [0.3, 0.46, 0.15],    # 1 cm off the wheel's rim
                       [0.1, 0.1, 1.22],     # 1 cm above the panel
                       [3.0, 3.0, 2.0],      # nothing within the tolerance
                       [5.0, 0.0, 2.02]])    # 2 cm above the wall's top: the wall, nearer than nothing else
    labels = cl.label_unity_points(points, [wall], [wheel, panel], tol=0.03)
    assert labels.tolist() == ["site", "ground", "robot", "panel", "other", "site"]


def test_label_unity_points_takes_the_nearest():
    # a point 1 cm above the ground and 2.5 cm from a box's face: the ground is nearer
    low = box("site", (0.0, 0.0, 0.5), (0.5, 0.5, 0.5))
    assert cl.label_unity_points(np.array([[0.525, 0.0, 0.01]]), [low], [], tol=0.03).tolist() == ["ground"]


def pose_set(lidar, labels, base_offset=(0.24, 0.0, 0.5)):
    lidar = np.asarray(lidar, dtype=float)
    return dict(lidar=lidar, base=lidar + np.asarray(base_offset), labels=np.asarray(labels), scans=1)


def test_compare_flags_fraction_gap():
    ground = [[5.0, 0.1 * i, -0.5] for i in range(50)]  # -5.7 deg: inside the field of view
    walls = [[6.0, 0.1 * i, 0.5] for i in range(50)]
    self_hits = [[-1.0, 0.05 * i, 0.3] for i in range(10)]
    unity = {"open": pose_set(ground + walls + self_hits, ["ground"] * 50 + ["site"] * 50 + ["robot"] * 10)}
    same = cl.compare(unity, {"open": pose_set(ground + walls + self_hits, ["ground"] * 50 + ["site"] * 50 + ["robot"] * 10)})
    row = same["open"]
    assert row["fractions_pass"] and row["self_hit_jaccard"] == pytest.approx(1.0) and row["pass"]
    assert row["min_range_violations"] == {"unity": 0, "plant": 0} and row["blind_zone_violations"] == {"unity": 0, "plant": 0}

    tilted = cl.compare(unity, {"open": pose_set(ground[:20] + walls + walls[:30] + self_hits,
                                                 ["ground"] * 20 + ["site"] * 80 + ["robot"] * 10)})["open"]
    assert not tilted["fractions_pass"] and not tilted["pass"]
    assert tilted["fraction_gap_pp"]["ground"] == pytest.approx(-100 * 30 / 110)


def test_compare_counts_range_and_blind_zone_violations():
    ok = [[3.0, 0.0, 0.0]] * 4
    near = [[0.05, 0.0, 0.0]]                                                    # inside the 0.1 m minimum range
    steep = [[math.cos(math.radians(70)), 0.0, math.sin(math.radians(70))]]      # above the 52 deg field of view
    unity = {"p": pose_set(ok + near + steep, ["site"] * 6)}
    plant = {"p": pose_set(ok + [[0.0, 0.0, 0.0]], ["site"] * 5)}                # a zero point is "no return"
    row = cl.compare(unity, plant)["p"]
    assert row["min_range_violations"] == {"unity": 1, "plant": 0}
    assert row["blind_zone_violations"] == {"unity": 1, "plant": 0}
    assert not row["pass"]


def test_load_unity_moves_the_robot_to_unitys_pose(tmp_path):
    # the plant stands at x 0, Unity 5 mm further and turned 0.01 rad: a point on Unity's robot is still the robot
    def pose(x, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([[c, -s, 0, x], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], float)
    base_p, base_u = pose(0.0, 0.0), pose(0.005, 0.01)
    lidar_in_base = pose(0.24, 0.0)
    lidar_in_base[2, 3] = 0.5
    header = {"base": base_p.tolist(), "prims": [box("panel", (-0.5, 0.0, 1.2), (0.4, 0.3, 0.01)),
                                                 box("site", (6.0, 0.0, 1.0), (0.1, 2.0, 1.0))]}
    # Unity's panel top at (-0.5, 0, 1.21) in its base frame; the wall face at x 5.9 in the world; a miss
    panel_base = np.array([-0.5, 0.0, 1.21, 1.0])
    to_lidar = np.linalg.inv(base_u @ lidar_in_base)
    points = [(np.linalg.inv(lidar_in_base) @ panel_base)[:3], (to_lidar @ np.array([5.9, 0.3, 1.0, 1.0]))[:3], [0.0, 0.0, 0.0]]
    np.savez(tmp_path / "unity.npz", points=np.array(points, np.float32), counts=np.array([3]), base=base_u,
             lidar_in_base=lidar_in_base, q=np.zeros(6), frame="livox_frame")
    pose_set, _ = cl.load_unity(tmp_path / "unity.npz", header)
    assert pose_set["labels"].tolist() == ["panel", "site", "other"]
    assert np.allclose(pose_set["base"][0], panel_base[:3], atol=1e-6)


def test_both_simulators_are_labelled_alike():
    # range noise (sigma 2 cm) puts points beyond 3 cm of their surface: the plant's true labels against Unity's
    # proximity labels would bias the fractions, so both get proximity labels with a 4-sigma tolerance
    wall = box("site", (5.0, 0.0, 1.0), (0.1, 2.0, 1.0))
    noisy = np.array([[4.84, 0.0, 1.0], [4.95, 0.5, 1.2]])  # 6 cm and 5 cm off the face (3 sigma, 2.5 sigma)
    assert cl.TOL_M == pytest.approx(4 * 0.02)
    assert cl.label_unity_points(noisy, [wall], [], tol=cl.TOL_M).tolist() == ["site", "site"]
    header = {"base": np.eye(4).tolist(), "lidar_in_base": np.eye(4).tolist(), "prims": [wall]}
    labelled = cl.relabel(dict(lidar=noisy, base=noisy, labels=np.array(["site", "site"]), scans=1), header, np.eye(4))
    assert labelled["labels"].tolist() == ["site", "site"] and labelled["true_labels"].tolist() == ["site", "site"]


def test_load_unity_takes_the_urdf_mount_without_static_tf(tmp_path):
    # the runner starts no robot_state_publisher: no /tf_static in the bag, so the export stores NaN and the URDF's mount
    # (the plant's header, generated from the same URDF) places the lidar
    mount = np.eye(4)
    mount[:3, 3] = [0.24, 0.0, 0.5]
    header = {"base": np.eye(4).tolist(), "lidar_in_base": mount.tolist(), "prims": [box("site", (6.0, 0.0, 1.0), (0.1, 2.0, 1.0))]}
    np.savez(tmp_path / "unity.npz", points=np.array([[5.9 - 0.24, 0.0, 0.5]], np.float32), counts=np.array([1]), base=np.eye(4),
             lidar_in_base=np.full((4, 4), np.nan), q=np.zeros(6), frame="livox_frame")
    pose_set, _ = cl.load_unity(tmp_path / "unity.npz", header)
    assert pose_set["labels"].tolist() == ["site"] and np.allclose(pose_set["base"][0], [5.9, 0.0, 1.0])
