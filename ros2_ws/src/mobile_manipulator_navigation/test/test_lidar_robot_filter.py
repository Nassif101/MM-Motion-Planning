import json
import math
from pathlib import Path
import sys

import numpy as np
from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mobile_manipulator_navigation.lidar_robot_filter import (  # noqa: E402
    keep_mask, load_primitives, rpy_matrix, self_mask, transform)

URDF = (Path(get_package_share_directory("mobile_manipulator_description")) / "urdf" /
        "mobile_manipulator.urdf").read_text()
PAYLOAD = json.loads((Path(get_package_share_directory("mobile_manipulator_control")) /
                      "config" / "qualified_payload.json").read_text())["payload"]
PANEL = {"link": "tool0", "size": PAYLOAD["dimensions_tool_ros_m"], "center": PAYLOAD["com_tool_ros_m"]}
PRIMITIVES = load_primitives(URDF, PANEL)
MARGIN = 0.03


def posed_at_origin(link_poses):
    return [(link_poses.get(link, np.eye(4)) @ pose, kind, dims)
            for link, pose, kind, dims in PRIMITIVES]


def test_loads_every_collision_primitive_plus_the_panel():
    kinds = [kind for _, _, kind, _ in PRIMITIVES]
    assert len(PRIMITIVES) == 13 and kinds.count("box") == 4 and kinds.count("cylinder") == 9
    link, pose, kind, dims = PRIMITIVES[-1]
    assert (link, kind) == ("tool0", "box")
    assert np.allclose(dims, (0.6, 0.6, 0.02)) and np.allclose(pose[:3, 3], (0, 0, 0.035))


def test_box_and_cylinder_membership_with_margin():
    box = [(transform(np.eye(3), [1.0, 0.0, 0.0]), "box", (0.5, 0.2, 0.1))]
    points = [[1.52, 0.0, 0.0], [1.56, 0.0, 0.0], [1.0, 0.22, 0.12]]
    assert self_mask(points, box, 0.03).tolist() == [True, False, True]
    cylinder = [(transform(rpy_matrix(math.pi / 2, 0, 0), [0.0, 0.0, 0.0]), "cylinder", (0.14, 0.045))]
    # Axis along base y after the roll: radius in x/z, half length in y.
    assert self_mask([[0.16, 0.0, 0.0], [0.0, 0.07, 0.0], [0.0, 0.08, 0.0]], cylinder, 0.03).tolist() == \
        [True, True, False]


def test_obstacle_inside_the_footprint_rectangle_but_off_the_robot_is_kept():
    # base_link box spans x +/-0.425, y +/-0.295 at z 0.21 +/- 0.11 (links at the origin
    # here, so place base_link at its joint offset). A post at (0.5, 0.5) is inside the
    # home footprint rectangle (x <= 0.54, |y| <= 0.62) but outside every primitive.
    base_link = transform(np.eye(3), [0.0, 0.0, 0.21])
    posed = posed_at_origin({"base_link": base_link})
    post = np.array([[0.5, 0.5, 0.5], [0.5, 0.5, 1.0]])
    deck = np.array([[0.2, 0.1, 0.32]])
    assert not self_mask(post, posed, MARGIN).any()
    assert self_mask(deck, posed, MARGIN).all()


def test_keep_mask_drops_misses_and_self_returns_only():
    sensor = transform(np.eye(3), [0.24, 0.0, 0.387])
    base_link = transform(np.eye(3), [0.0, 0.0, 0.21])
    posed = posed_at_origin({"base_link": base_link})
    points = np.array([[0.0, 0.0, 0.0],          # miss
                       [-0.1, 0.0, -0.08],      # deck, 0.307 m above ground
                       [3.0, 0.0, -0.38],       # ground
                       [0.26, 0.5, 0.1]])       # obstacle beside the robot
    assert keep_mask(points, sensor, posed, MARGIN).tolist() == [False, False, True, True]


def test_ray_rule_removes_noisy_self_returns_but_keeps_obstacles_in_front():
    from mobile_manipulator_navigation.lidar_robot_filter import first_self_hit
    sensor = transform(np.eye(3), [0.24, 0.0, 0.387])
    # Arm pedestal: cylinder r 0.112 around (-0.08, 0), z 0.32..0.50 in base_footprint.
    pedestal = [(transform(np.eye(3), [-0.08, 0.0, 0.41]), "cylinder", (0.112, 0.09))]
    direction = np.array([[-1.0, 0.0, 0.0]])
    surface = first_self_hit(sensor[:3, 3], direction, pedestal)[0]
    assert math.isclose(surface, 0.24 - (-0.08 + 0.112), abs_tol=1e-9)   # 0.208 m
    # Sensor-frame points straight back: a return 0.07 m short of the surface (3.5 sigma
    # noise), the true surface, and an obstacle 0.15 m in front of the pedestal.
    points = np.array([[-(surface - 0.07), 0, 0], [-surface, 0, 0], [-(surface - 0.15), 0, 0]])
    keep = keep_mask(points, sensor, pedestal, margin=0.03, noise_band=0.08).tolist()
    assert keep == [False, False, True]


def test_ray_rule_hits_boxes_and_ignores_rays_that_miss_the_robot():
    from mobile_manipulator_navigation.lidar_robot_filter import first_self_hit
    box = [(transform(np.eye(3), [1.0, 0.0, 0.0]), "box", (0.1, 0.1, 0.1))]
    rays = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    hits = first_self_hit(np.zeros(3), rays, box)
    assert math.isclose(hits[0], 0.9) and math.isinf(hits[1])
