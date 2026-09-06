#!/usr/bin/env python3
"""Project policy checks for the generated mobile manipulator URDF."""

from __future__ import annotations

import math
import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = PACKAGE_ROOT / "urdf" / "mobile_manipulator.urdf"
GENERATOR_PATH = PACKAGE_ROOT / "urdf" / "mobile_manipulator.py"
PACKAGE_PREFIX = "package://mobile_manipulator_description/"

EXPECTED_LIMITS = {
    "front_left_wheel_joint": (None, None, 18.0, 85.0),
    "front_right_wheel_joint": (None, None, 18.0, 85.0),
    "rear_left_wheel_joint": (None, None, 18.0, 85.0),
    "rear_right_wheel_joint": (None, None, 18.0, 85.0),
    "shoulder_pan_joint": (-2.96705972839, 2.96705972839, 1.8, 120.0),
    "shoulder_lift_joint": (-1.74532925199, 1.74532925199, 1.6, 160.0),
    "elbow_joint": (-2.35619449019, 2.35619449019, 1.8, 90.0),
    "wrist_1_joint": (-math.pi, math.pi, 2.5, 40.0),
    "wrist_2_joint": (-2.09439510239, 2.09439510239, 2.5, 30.0),
    "wrist_3_joint": (-2.0 * math.pi, 2.0 * math.pi, 3.2, 20.0),
}

EXPECTED_DYNAMICS = {
    "front_left_wheel_joint": (0.35, 0.08),
    "front_right_wheel_joint": (0.35, 0.08),
    "rear_left_wheel_joint": (0.35, 0.08),
    "rear_right_wheel_joint": (0.35, 0.08),
    "shoulder_pan_joint": (2.0, 0.3),
    "shoulder_lift_joint": (2.2, 0.25),
    "elbow_joint": (1.8, 0.2),
    "wrist_1_joint": (0.9, 0.12),
    "wrist_2_joint": (0.8, 0.1),
    "wrist_3_joint": (0.6, 0.08),
}


def _numbers(text, expected):
    values = tuple(float(value) for value in text.split())
    assert len(values) == expected
    assert all(math.isfinite(value) for value in values)
    return values


def _matmul(left, right):
    return [
        [sum(left[row][k] * right[k][column] for k in range(4)) for column in range(4)]
        for row in range(4)
    ]


def _identity():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _translation(x=0.0, y=0.0, z=0.0):
    result = _identity()
    result[0][3], result[1][3], result[2][3] = x, y, z
    return result


def _rotation_x(angle):
    cosine, sine = math.cos(angle), math.sin(angle)
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, cosine, -sine, 0.0],
        [0.0, sine, cosine, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotation_y(angle):
    cosine, sine = math.cos(angle), math.sin(angle)
    return [
        [cosine, 0.0, sine, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-sine, 0.0, cosine, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotation_z(angle):
    cosine, sine = math.cos(angle), math.sin(angle)
    return [
        [cosine, -sine, 0.0, 0.0],
        [sine, cosine, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _axis_rotation(axis, angle):
    x, y, z = axis
    cosine, sine = math.cos(angle), math.sin(angle)
    one_minus_cosine = 1.0 - cosine
    return [
        [cosine + x * x * one_minus_cosine, x * y * one_minus_cosine - z * sine, x * z * one_minus_cosine + y * sine, 0.0],
        [y * x * one_minus_cosine + z * sine, cosine + y * y * one_minus_cosine, y * z * one_minus_cosine - x * sine, 0.0],
        [z * x * one_minus_cosine - y * sine, z * y * one_minus_cosine + x * sine, cosine + z * z * one_minus_cosine, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _origin_transform(origin):
    xyz = _numbers(origin.attrib["xyz"], 3)
    roll, pitch, yaw = _numbers(origin.attrib["rpy"], 3)
    rotation = _matmul(_matmul(_rotation_z(yaw), _rotation_y(pitch)), _rotation_x(roll))
    return _matmul(_translation(*xyz), rotation)


def _urdf_base_to_tool(joints, positions):
    children_by_parent = {}
    for joint in joints.values():
        parent = joint.find("parent").attrib["link"]
        children_by_parent.setdefault(parent, []).append(joint)

    path = []

    def find_path(parent):
        if parent == "tool0":
            return True
        for joint in children_by_parent.get(parent, []):
            path.append(joint)
            if find_path(joint.find("child").attrib["link"]):
                return True
            path.pop()
        return False

    assert find_path("base_link")
    transform = _identity()
    for joint in path:
        transform = _matmul(transform, _origin_transform(joint.find("origin")))
        if joint.attrib["type"] != "fixed":
            axis = _numbers(joint.find("axis").attrib["xyz"], 3)
            transform = _matmul(
                transform,
                _axis_rotation(axis, positions[joint.attrib["name"]]),
            )
    return transform


def _standard_dh_base_to_tool(rows, positions):
    transform = _translation(-0.08, 0.0, 0.29)
    for joint_name, (theta_offset, d, a, alpha) in zip(
        (
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ),
        rows,
    ):
        link_transform = _matmul(
            _matmul(
                _matmul(_rotation_z(positions[joint_name] + theta_offset), _translation(z=d)),
                _translation(x=a),
            ),
            _rotation_x(alpha),
        )
        transform = _matmul(transform, link_transform)
    return transform


def _assert_matrices_close(actual, expected, tolerance=1e-10):
    for row in range(4):
        for column in range(4):
            assert math.isclose(
                actual[row][column],
                expected[row][column],
                rel_tol=0.0,
                abs_tol=tolerance,
            ), (row, column, actual[row][column], expected[row][column])


def main():
    root = ET.parse(URDF_PATH).getroot()
    assert root.tag == "robot"
    assert root.attrib["name"] == "mobile_manipulator"

    links = {link.attrib["name"]: link for link in root.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    assert len(links) == 18, len(links)
    assert len(joints) == 17, len(joints)
    assert len(joints) == len(links) - 1

    children = {}
    for joint in joints.values():
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        assert parent in links
        assert child in links
        assert child not in children
        children[child] = parent

        origin = joint.find("origin")
        _numbers(origin.attrib["xyz"], 3)
        _numbers(origin.attrib["rpy"], 3)

        if joint.attrib["type"] != "fixed":
            axis = _numbers(joint.find("axis").attrib["xyz"], 3)
            magnitude = math.sqrt(sum(value * value for value in axis))
            assert math.isclose(magnitude, 1.0, abs_tol=1e-12)
            limit = joint.find("limit")
            assert float(limit.attrib["effort"]) > 0.0
            assert float(limit.attrib["velocity"]) > 0.0

    roots = set(links) - set(children)
    assert roots == {"base_footprint"}, roots

    movable = [joint for joint in joints.values() if joint.attrib["type"] != "fixed"]
    continuous = [joint for joint in movable if joint.attrib["type"] == "continuous"]
    revolute = [joint for joint in movable if joint.attrib["type"] == "revolute"]
    assert len(movable) == 10
    assert len(continuous) == 4
    assert len(revolute) == 6

    assert set(EXPECTED_LIMITS) == {joint.attrib["name"] for joint in movable}
    for name, (lower, upper, velocity, effort) in EXPECTED_LIMITS.items():
        joint = joints[name]
        limit = joint.find("limit")
        assert math.isclose(float(limit.attrib["velocity"]), velocity, abs_tol=1e-12)
        assert math.isclose(float(limit.attrib["effort"]), effort, abs_tol=1e-12)
        if joint.attrib["type"] == "continuous":
            assert "lower" not in limit.attrib and "upper" not in limit.attrib
        else:
            assert math.isclose(float(limit.attrib["lower"]), lower, abs_tol=1e-12)
            assert math.isclose(float(limit.attrib["upper"]), upper, abs_tol=1e-12)

        dynamics = joint.find("dynamics")
        expected_damping, expected_friction = EXPECTED_DYNAMICS[name]
        assert math.isclose(float(dynamics.attrib["damping"]), expected_damping, abs_tol=1e-12)
        assert math.isclose(float(dynamics.attrib["friction"]), expected_friction, abs_tol=1e-12)

    physical_links = [link for link in links.values() if link.find("inertial") is not None]
    assert len(physical_links) == 12
    total_mass = sum(float(link.find("inertial/mass").attrib["value"]) for link in physical_links)
    assert math.isclose(total_mass, 107.0, abs_tol=1e-12)
    for link in physical_links:
        inertial = link.find("inertial")
        mass = float(inertial.find("mass").attrib["value"])
        assert mass > 0.0
        tensor = inertial.find("inertia").attrib
        ixx = float(tensor["ixx"])
        iyy = float(tensor["iyy"])
        izz = float(tensor["izz"])
        assert ixx > 0.0 and iyy > 0.0 and izz > 0.0
        assert ixx + iyy >= izz
        assert ixx + izz >= iyy
        assert iyy + izz >= ixx
        assert link.find("visual") is not None
        assert link.find("collision") is not None

    mesh_count = 0
    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib["filename"]
        assert filename.startswith(PACKAGE_PREFIX), filename
        relative = filename.removeprefix(PACKAGE_PREFIX)
        mesh_path = PACKAGE_ROOT / relative
        assert mesh_path.is_file(), mesh_path
        assert mesh_path.stat().st_size > 84, mesh_path
        assert _numbers(mesh.attrib["scale"], 3) == (0.001, 0.001, 0.001)
        mesh_count += 1
    assert mesh_count == 12

    expected_frame_links = {
        "base_footprint",
        "tool0",
        "top_sensor_mount_link",
        "livox_frame",
        "front_sensor_mount_link",
        "tool_sensor_mount_link",
    }
    for name in expected_frame_links:
        link = links[name]
        assert link.find("inertial") is None
        assert link.find("visual") is None
        assert link.find("collision") is None

    livox_joint = joints["livox_joint"]
    assert livox_joint.attrib["type"] == "fixed"
    assert livox_joint.find("parent").attrib["link"] == "top_sensor_mount_link"
    assert livox_joint.find("child").attrib["link"] == "livox_frame"
    assert _numbers(livox_joint.find("origin").attrib["xyz"], 3) == (
        0.0,
        0.0,
        0.047,
    )

    base_collision = links["base_link"].find("collision/geometry/box")
    assert _numbers(base_collision.attrib["size"], 3) == (0.85, 0.59, 0.22)
    for wheel_name in (
        "front_left_wheel_link",
        "front_right_wheel_link",
        "rear_left_wheel_link",
        "rear_right_wheel_link",
    ):
        cylinder = links[wheel_name].find("collision/geometry/cylinder")
        assert math.isclose(float(cylinder.attrib["radius"]), 0.14, abs_tol=1e-12)
        assert math.isclose(float(cylinder.attrib["length"]), 0.09, abs_tol=1e-12)

    expected_fixed_origins = {
        "top_sensor_mount_joint": (0.24, 0.0, 0.13),
        "front_sensor_mount_joint": (0.44, 0.0, 0.02),
        "tool_sensor_mount_joint": (0.0, 0.0, 0.0),
    }
    for name, expected_xyz in expected_fixed_origins.items():
        assert _numbers(joints[name].find("origin").attrib["xyz"], 3) == expected_xyz

    generator_spec = importlib.util.spec_from_file_location(
        "mobile_manipulator_generator",
        GENERATOR_PATH,
    )
    generator = importlib.util.module_from_spec(generator_spec)
    generator_spec.loader.exec_module(generator)
    assert len(generator.ARM_STANDARD_DH) == 6

    configurations = (
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        (0.4, -0.7, 1.1, -0.3, 0.8, -1.2),
        (-1.2, 0.9, -0.6, 1.4, -0.5, 2.0),
        (2.2, -1.0, 1.8, -2.1, 1.2, -3.0),
    )
    arm_names = tuple(generator.ARM_JOINT_LIMITS)
    for configuration in configurations:
        positions = dict(zip(arm_names, configuration))
        urdf_transform = _urdf_base_to_tool(joints, positions)
        dh_transform = _standard_dh_base_to_tool(generator.ARM_STANDARD_DH, positions)
        _assert_matrices_close(urdf_transform, dh_transform)

    neutral = _standard_dh_base_to_tool(
        generator.ARM_STANDARD_DH,
        dict.fromkeys(arm_names, 0.0),
    )
    _assert_matrices_close(neutral, _translation(-0.08, 0.0, 1.30))

    print(
        "Validated mobile_manipulator:",
        f"{len(links)} links, {len(joints)} joints,",
        f"{len(movable)} movable joints, {mesh_count} mesh references.",
    )


def test_mobile_manipulator_description():
    main()


if __name__ == "__main__":
    main()
