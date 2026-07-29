#!/usr/bin/env python3
"""Project policy checks for the generated mobile manipulator URDF."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = PACKAGE_ROOT / "urdf" / "mobile_manipulator.urdf"
PACKAGE_PREFIX = "package://mobile_manipulator_description/"


def _numbers(text, expected):
    values = tuple(float(value) for value in text.split())
    assert len(values) == expected
    assert all(math.isfinite(value) for value in values)
    return values


def main():
    root = ET.parse(URDF_PATH).getroot()
    assert root.tag == "robot"
    assert root.attrib["name"] == "mobile_manipulator"

    links = {link.attrib["name"]: link for link in root.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    assert len(links) == 17, len(links)
    assert len(joints) == 16, len(joints)
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

    physical_links = [link for link in links.values() if link.find("inertial") is not None]
    assert len(physical_links) == 12
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
        "front_sensor_mount_link",
        "tool_sensor_mount_link",
    }
    for name in expected_frame_links:
        link = links[name]
        assert link.find("inertial") is None
        assert link.find("visual") is None
        assert link.find("collision") is None

    print(
        "Validated mobile_manipulator:",
        f"{len(links)} links, {len(joints)} joints,",
        f"{len(movable)} movable joints, {mesh_count} mesh references.",
    )


def test_mobile_manipulator_description():
    main()


if __name__ == "__main__":
    main()
