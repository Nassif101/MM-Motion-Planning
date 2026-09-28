import itertools
import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROFILES = yaml.safe_load(
    (PACKAGE_ROOT / "config" / "footprint_profiles.yaml").read_text(encoding="utf-8")
)
COSTMAP = yaml.safe_load(
    (PACKAGE_ROOT / "config" / "nav2_global_planning.yaml").read_text(encoding="utf-8")
)["global_costmap"]["global_costmap"]["ros__parameters"]
URDF = ET.parse(
    Path(get_package_share_directory("mobile_manipulator_description"))
    / "urdf" / "mobile_manipulator.urdf"
).getroot()
PAYLOAD = json.loads(
    (Path(get_package_share_directory("mobile_manipulator_control"))
     / "config" / "qualified_payload.json").read_text(encoding="utf-8")
)
PARENT_JOINT = {joint.find("child").get("link"): joint for joint in URDF.findall("joint")}


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_matrix(axis, angle):
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    skew = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * skew @ skew


def origin_transform(element):
    transform = np.eye(4)
    if element is not None:
        xyz = [float(v) for v in element.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in element.get("rpy", "0 0 0").split()]
        transform[:3, :3] = rpy_matrix(*rpy)
        transform[:3, 3] = xyz
    return transform


def link_transform(link, positions):
    """base_footprint -> link for named joint positions (unlisted joints at zero)."""
    if link == "base_footprint":
        return np.eye(4)
    joint = PARENT_JOINT[link]
    transform = origin_transform(joint.find("origin"))
    if joint.get("type") in ("revolute", "continuous"):
        motion = np.eye(4)
        axis = [float(v) for v in joint.find("axis").get("xyz").split()]
        motion[:3, :3] = axis_matrix(axis, positions.get(joint.get("name"), 0.0))
        transform = transform @ motion
    return link_transform(joint.find("parent").get("link"), positions) @ transform


def primitive_points(geometry):
    """Points whose convex hull contains the primitive, in the collision frame."""
    if geometry.tag == "box":
        half = [float(v) / 2.0 for v in geometry.get("size").split()]
        return [np.array([sx * half[0], sy * half[1], sz * half[2]])
                for sx, sy, sz in itertools.product((-1, 1), repeat=3)]
    if geometry.tag == "cylinder":
        radius = float(geometry.get("radius"))
        half_length = float(geometry.get("length")) / 2.0
        # A circumscribed 64-gon keeps the sampled hull outside the true circle.
        outer = radius / math.cos(math.pi / 64)
        return [np.array([outer * math.cos(a), outer * math.sin(a), z])
                for a in np.linspace(0.0, 2.0 * math.pi, 64, endpoint=False)
                for z in (-half_length, half_length)]
    raise AssertionError(f"Unsupported collision geometry {geometry.tag}")


def projected_bounds(pose_name):
    positions = dict(zip(PAYLOAD["joint_order"], PAYLOAD["poses_rad"][pose_name]))
    points = []
    for link in URDF.findall("link"):
        for collision in link.findall("collision"):
            frame = link_transform(link.get("name"), positions) @ origin_transform(
                collision.find("origin"))
            points += [(frame @ np.append(p, 1.0))[:3]
                       for p in primitive_points(collision.find("geometry")[0])]

    tool = link_transform("tool0", positions)
    size = PAYLOAD["payload"]["dimensions_tool_ros_m"]
    centre = PAYLOAD["payload"]["com_tool_ros_m"]
    points += [(tool @ np.array([centre[0] + sx * size[0] / 2,
                                 centre[1] + sy * size[1] / 2,
                                 centre[2] + sz * size[2] / 2, 1.0]))[:3]
               for sx, sy, sz in itertools.product((-1, 1), repeat=3)]
    points = np.array(points)
    return points[:, 0].min(), points[:, 0].max(), points[:, 1].min(), points[:, 1].max()


def rectangle(polygon):
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    assert len(polygon) == 4 and len(set(xs)) == 2 and len(set(ys)) == 2, polygon
    return min(xs), max(xs), min(ys), max(ys)


def test_profiles_contain_and_tightly_bound_robot_and_panel():
    allowance = PROFILES["perimeter_allowance_m"]
    for name, profile in PROFILES["profiles"].items():
        x_min, x_max, y_min, y_max = projected_bounds(profile["arm_pose"])
        px_min, px_max, py_min, py_max = rectangle(profile["polygon"])
        expected = (x_min - allowance, x_max + allowance,
                    y_min - allowance, y_max + allowance)
        actual = (px_min, px_max, py_min, py_max)
        # Contains the robot with the declared allowance (1 mm rounding tolerance) ...
        assert px_min <= expected[0] + 1e-3 and px_max >= expected[1] - 1e-3, (name, expected)
        assert py_min <= expected[2] + 1e-3 and py_max >= expected[3] - 1e-3, (name, expected)
        # ... and has not silently grown beyond it.
        assert np.allclose(actual, expected, atol=0.006), (name, actual, expected)


def test_global_costmap_uses_default_profile_and_sufficient_inflation():
    default = PROFILES["profiles"][PROFILES["default_profile"]]["polygon"]
    configured = [[float(v) for v in pair]
                  for pair in re.findall(r"\[\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\]",
                                         COSTMAP["footprint"])]
    assert configured == default

    padding = COSTMAP["footprint_padding"]
    inflation = COSTMAP["inflation_layer"]["inflation_radius"]
    for profile in PROFILES["profiles"].values():
        circumscribed = max(math.hypot(x, y) for x, y in profile["polygon"]) + padding
        assert inflation >= circumscribed, (inflation, circumscribed)
