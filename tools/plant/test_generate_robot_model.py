"""Tests of the MuJoCo robot model generator (MuJoCo plant plan Task 3; spec Section 3.2)."""
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

import generate_robot_model as grm

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / "ros2_ws/src/mobile_manipulator_plant/robots/mobile_manipulator/profile.yaml"
URDF = ROOT / "ros2_ws/src/mobile_manipulator_description/urdf/mobile_manipulator.urdf"
PAYLOAD = ROOT / "ros2_ws/src/mobile_manipulator_control/config/qualified_payload.json"
COMMITTED = PROFILE.parent / "robot.xml"


@pytest.fixture(scope="module")
def model():
    return ET.fromstring(grm.generate_robot(PROFILE))


@pytest.fixture(scope="module")
def urdf():
    return ET.parse(URDF).getroot()


def floats(text):
    return [float(v) for v in text.split()]


def body(model, name):
    return next(b for b in model.iter("body") if b.get("name") == name)


def test_planar_base_joints(model):
    base = body(model, "base_footprint")
    joints = [(j.get("name"), j.get("type"), j.get("axis")) for j in base.findall("joint")]
    assert joints == [("base_x", "slide", "1 0 0"), ("base_y", "slide", "0 1 0"), ("base_yaw", "hinge", "0 0 1")]


def test_arm_matches_urdf(model, urdf):
    profile = yaml.safe_load(PROFILE.read_text())
    for name in profile["arm_joints"]:
        uj = next(j for j in urdf.findall("joint") if j.get("name") == name)
        mj = next(j for j in model.iter("joint") if j.get("name") == name)
        child = body(model, uj.find("child").get("link"))
        assert mj in list(child.findall("joint"))
        assert floats(mj.get("axis")) == floats(uj.find("axis").get("xyz"))
        lim = uj.find("limit")
        assert floats(mj.get("range")) == [float(lim.get("lower")), float(lim.get("upper"))]
        assert floats(child.get("pos")) == floats(uj.find("origin").get("xyz"))
        dyn = uj.find("dynamics")
        assert float(mj.get("damping")) == float(dyn.get("damping"))
        assert float(mj.get("frictionloss")) == float(dyn.get("friction"))
    for link in urdf.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mi = body(model, link.get("name")).find("inertial")
        assert float(mi.get("mass")) == float(inertial.find("mass").get("value"))
        i = inertial.find("inertia").attrib
        assert floats(mi.get("fullinertia")) == [float(i[k]) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")]
        assert floats(mi.get("pos")) == floats(inertial.find("origin").get("xyz"))


def test_collision_geoms_are_urdf_primitives(model, urdf):
    geoms = {g.get("name"): g for g in model.iter("geom")}
    for link in urdf.findall("link"):
        for col in link.findall("collision"):
            g = geoms["robot/" + col.get("name")]
            shape = col.find("geometry")[0]
            if shape.tag == "box":
                assert g.get("type") == "box"
                np.testing.assert_allclose(floats(g.get("size")), [v / 2 for v in floats(shape.get("size"))])
            else:
                assert g.get("type") == "cylinder"
                np.testing.assert_allclose(floats(g.get("size")),
                                           [float(shape.get("radius")), float(shape.get("length")) / 2])
            assert floats(g.get("pos")) == floats(col.find("origin").get("xyz"))
            assert g in list(body(model, link.get("name")).findall("geom"))


def test_panel_from_payload(model):
    payload = json.loads(PAYLOAD.read_text())["payload"]
    panel = body(model, "panel")
    assert panel in list(body(model, "tool0"))
    assert floats(panel.get("pos")) == payload["com_tool_ros_m"]
    g = panel.find("geom")
    assert g.get("name") == "robot/panel" and g.get("type") == "box" and g.get("group") == "2"
    np.testing.assert_allclose(floats(g.get("size")), [v / 2 for v in payload["dimensions_tool_ros_m"]])
    inertial = panel.find("inertial")
    assert float(inertial.get("mass")) == payload["mass_kg"] == 3.0
    x, y, z = payload["dimensions_tool_ros_m"]
    m = payload["mass_kg"]
    np.testing.assert_allclose(floats(inertial.get("diaginertia")),
                               [m / 12 * (y * y + z * z), m / 12 * (x * x + z * z), m / 12 * (x * x + y * y)])


def test_wheels_have_no_contact(model):
    for g in model.iter("geom"):
        if "wheel" in g.get("name", ""):
            assert (g.get("contype"), g.get("conaffinity")) == ("0", "0")


def test_robot_geoms_collide_with_the_site_only(model):
    robot = next(d for d in model.iter("default") if d.get("class") == "robot").find("geom")
    assert (robot.get("contype"), robot.get("conaffinity")) == ("2", "1")
    # self contact: only the panel against the chassis, by an explicit pair
    pairs = [(p.get("geom1"), p.get("geom2")) for p in model.iter("pair")]
    assert pairs == [("robot/panel", "robot/base_chassis_collision")]


def test_servo_actuators_from_profile(model, urdf):
    profile = yaml.safe_load(PROFILE.read_text())
    for name, kp, kd in zip(profile["arm_joints"], profile["arm"]["kp"], profile["arm"]["kd"]):
        a = next(a for a in model.iter("general") if a.get("joint") == name)
        assert a.get("name") == f"servo/{name}"
        assert floats(a.get("gainprm")) == [kp]
        assert a.get("biastype") == "affine" and floats(a.get("biasprm")) == [0.0, -kp, -kd]
        effort = float(next(j for j in urdf.findall("joint") if j.get("name") == name).find("limit").get("effort"))
        assert floats(a.get("forcerange")) == [-effort, effort] and a.get("forcelimited") == "true"
    for joint, kv in zip(("base_x", "base_y", "base_yaw"), profile["base"]["servo_kv"]):
        a = next(a for a in model.iter("velocity") if a.get("joint") == joint)
        assert a.get("name") == f"drive/{joint}" and float(a.get("kv")) == kv


def test_lidar_site(model, urdf):
    site = next(s for s in model.iter("site") if s.get("name") == "livox_frame")
    assert site in list(body(model, "livox_frame"))


def test_compiler_and_option(model):
    assert model.find("option").get("timestep") == "0.002"
    assert model.find("option").get("integrator") == "implicitfast"


def test_committed_robot_is_current():
    assert COMMITTED.read_text() == grm.generate_robot(PROFILE)
