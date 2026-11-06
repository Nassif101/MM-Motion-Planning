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
        k = profile["arm_joints"].index(name)
        assert float(mj.get("damping")) == profile["arm"]["damping"][k]
        assert float(mj.get("frictionloss")) == profile["arm"]["frictionloss"][k]
        assert float(mj.get("armature")) == profile["arm"]["armature"][k]
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


def test_wheels_meet_obstacles_not_the_ground(model):
    wheels = [g for g in model.iter("geom") if "wheel" in g.get("name", "")]
    assert len(wheels) == 4
    for g in wheels:
        assert g.get("contype") == "2" and g.get("conaffinity") is None  # the class's conaffinity 1


def test_robot_geoms_collide_with_the_world_only(model):
    robot = next(d for d in model.iter("default") if d.get("class") == "robot").find("geom")
    assert (robot.get("contype"), robot.get("conaffinity")) == ("6", "1")
    # self contact: the panel against every base geom (B5's panel-against-base row), by explicit pairs
    pairs = sorted(p.get("geom2") for p in model.iter("pair") if p.get("geom1") == "robot/panel")
    assert pairs == sorted(["robot/base_chassis_collision", "robot/arm_mount_collision", "robot/front_left_wheel_collision",
                            "robot/front_right_wheel_collision", "robot/rear_left_wheel_collision",
                            "robot/rear_right_wheel_collision"])


def test_servo_actuators_from_profile(model, urdf):
    profile = yaml.safe_load(PROFILE.read_text())
    # a profile with a gain rule leaves the gains to the plant (resolve_arm_gains): the model carries unit placeholders
    kps = profile["arm"].get("kp", [1.0] * 6) if "gain_rule" not in profile["arm"] else [1.0] * 6
    kds = profile["arm"].get("kd", [1.0] * 6) if "gain_rule" not in profile["arm"] else [1.0] * 6
    for name, kp, kd in zip(profile["arm_joints"], kps, kds):
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


def test_custom_texts_do_not_collide_with_the_site(model):
    names = {t.get("name") for t in model.iter("text")}
    assert names == {"robot_generator", "robot_urdf_sha256"}


def test_no_physics_options(model):
    # the plant sets the physics options on the composed model (an attached child's options would conflict)
    assert model.find("option") is None


def test_committed_robot_is_current():
    assert COMMITTED.read_text() == grm.generate_robot(PROFILE)


COMMITTED_WHEELS = PROFILE.parent / "robot_wheels.xml"


@pytest.fixture(scope="module")
def wheels():
    return ET.fromstring(grm.generate_robot(PROFILE, wheels=True))


def urdf_wheels(urdf):
    return [j for j in urdf.findall("joint") if j.get("type") == "continuous"]


def test_wheels_model_has_free_base_and_driven_wheels(wheels, urdf):
    # amendment 2026-11-06: the realistic base stands on its wheels; gravity presses the whole robot on their contacts
    base = body(wheels, "base_footprint")
    assert [(j.tag, j.get("name")) for j in base if j.tag in ("joint", "freejoint")] == [("freejoint", "base_free")]
    profile = yaml.safe_load(PROFILE.read_text())
    for joint in urdf_wheels(urdf):
        wheel = body(wheels, joint.find("child").get("link"))
        hinge = wheel.find("joint")
        assert hinge.get("name") == joint.get("name") and hinge.get("type") == "hinge"
        assert floats(hinge.get("axis")) == floats(joint.find("axis").get("xyz"))
        a = next(a for a in wheels.iter("velocity") if a.get("joint") == joint.get("name"))
        assert a.get("name") == f"wheel/{joint.get('name')}" and float(a.get("kv")) == profile["base"]["wheel_kv"]
        effort = float(joint.find("limit").get("effort"))
        assert floats(a.get("forcerange")) == [-effort, effort] and a.get("forcelimited") == "true"
    assert not [a for a in wheels.iter("velocity") if a.get("name", "").startswith("drive/")]


def test_wheels_model_uses_urdf_wheel_dynamics(wheels, urdf):
    for joint in urdf_wheels(urdf):
        dyn = joint.find("dynamics")
        hinge = body(wheels, joint.find("child").get("link")).find("joint")
        assert float(hinge.get("damping")) == float(dyn.get("damping"))
        assert float(hinge.get("frictionloss")) == float(dyn.get("friction"))
        geom = body(wheels, joint.find("child").get("link")).find("geom")
        assert geom.get("contype") is None  # the robot class: the wheels touch the ground


def test_default_profile_arm_terms_are_the_urdf_s(urdf):
    # the URDF-based realistic mode switches the URDF's joint damping and friction on (profile_unity.yaml keeps zeros)
    profile = yaml.safe_load(PROFILE.read_text())
    for k, name in enumerate(profile["arm_joints"]):
        dyn = next(j for j in urdf.findall("joint") if j.get("name") == name).find("dynamics")
        assert profile["arm"]["damping"][k] == float(dyn.get("damping"))
        assert profile["arm"]["frictionloss"][k] == float(dyn.get("friction"))


def test_planar_model_unchanged_by_the_wheels(model):
    assert body(model, "base_footprint").find("freejoint") is None
    assert not [a for a in model.iter("velocity") if a.get("name", "").startswith("wheel/")]


def test_committed_wheels_robot_is_current():
    assert COMMITTED_WHEELS.read_text() == grm.generate_robot(PROFILE, wheels=True)
