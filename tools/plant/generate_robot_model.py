"""Generate the MuJoCo robot model from the URDF and the robot profile (MuJoCo plant spec Section 3.2, plan Task 3).

- base_footprint carries planar joints base_x, base_y (slides) and base_yaw (hinge); the URDF's links hang below it
  with their masses and inertias. The wheels are fixed bodies (no joint): they collide with obstacles but not with the
  ground (the planar base carries the robot), so a wheel scraping a door jamb is a contact.
- The six arm joints keep the URDF's axes and limits; their passive damping, frictionloss and armature come from the
  profile (Unity's drives have no URDF joint damping; its joint friction is a PhysX coefficient). Each is driven by a
  position servo (`general`, affine bias: force = kp (ctrl - q) - kd qdot, clamped to the URDF effort) whose damping
  MuJoCo integrates implicitly; the plant shifts ctrl to add the reference velocity and the feedforward torque, as
  Unity shifts its drive target, so the force limit caps the total.
- Collision geoms are the URDF's collision primitives (class `robot`: contype 6, conaffinity 1, group 1: obstacles and
  the ground; wheels contype 2: obstacles only); the panel is the qualified payload's box (group 2) on a body under
  tool0; the panel against every base geom (chassis, arm mount, wheels) are the self contacts (pairs), as B5's
  panel-against-base row.
- The base has a velocity actuator per planar joint (kv from the profile), used by the realistic mode.
No meshes: the plant is headless; the viewer adds visuals.

    python3 tools/plant/generate_robot_model.py [--check]
"""
import argparse
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "ros2_ws/src"
PROFILE = SRC / "mobile_manipulator_plant/robots/mobile_manipulator/profile.yaml"
GENERATOR = "generate_robot_model 1"


def num(value):
    return repr(float(value))


def vec(values):
    return " ".join(num(v) for v in values)


def floats(text):
    return [float(v) for v in text.split()]


def rpy_quat(rpy):
    """URDF roll-pitch-yaw (fixed axes x, y, z) as a quaternion (w, x, y, z)."""
    r, p, y = (v / 2 for v in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def origin_attrs(element):
    """pos (and quat when rotated) of a URDF <origin> child."""
    origin = element.find("origin")
    xyz = floats(origin.get("xyz", "0 0 0")) if origin is not None else [0.0, 0.0, 0.0]
    rpy = floats(origin.get("rpy", "0 0 0")) if origin is not None else [0.0, 0.0, 0.0]
    attrs = f'pos="{vec(xyz)}"'
    if any(abs(v) > 0 for v in rpy):
        attrs += f' quat="{vec(rpy_quat(rpy))}"'
    return attrs


class Generator:
    def __init__(self, profile_path):
        self.profile_path = Path(profile_path)
        self.profile = yaml.safe_load(self.profile_path.read_text())
        self.urdf_path = SRC / self.profile["urdf"]  # package-relative: the package directory is its source directory
        self.urdf = ET.parse(self.urdf_path).getroot()
        self.payload = json.loads((SRC / self.profile["payload_json"]).read_text())["payload"]
        self.links = {link.get("name"): link for link in self.urdf.findall("link")}
        self.children = {}
        for joint in self.urdf.findall("joint"):
            self.children.setdefault(joint.find("parent").get("link"), []).append(joint)
        self.arm = self.profile["arm_joints"]
        # the base's continuous joints: the wheels (fixed in the planar base model)
        self.wheel_joints = {j.get("name") for j in self.urdf.findall("joint") if j.get("type") == "continuous"}
        self.lines = []

    def emit(self, depth, text):
        self.lines.append("  " * depth + text)

    def link_body(self, link_name, joint, depth):
        link = self.links[link_name]
        self.emit(depth, f'<body name="{link_name}" {origin_attrs(joint)}>')
        if joint.get("type") in ("revolute", "continuous") and joint.get("name") in self.arm:
            lim, dyn = joint.find("limit"), joint.find("dynamics")
            k = self.arm.index(joint.get("name"))
            arm = self.profile["arm"]
            self.emit(depth + 1, f'<joint name="{joint.get("name")}" type="hinge" axis="{vec(floats(joint.find("axis").get("xyz")))}" '
                                 f'range="{num(lim.get("lower"))} {num(lim.get("upper"))}" limited="true" '
                                 f'damping="{num(arm["damping"][k])}" frictionloss="{num(arm["frictionloss"][k])}" '
                                 f'armature="{num(arm["armature"][k])}"/>')
        elif joint.get("type") not in ("fixed",) and joint.get("name") not in self.wheel_joints:
            raise ValueError(f"{joint.get('name')}: a moving joint that is neither an arm nor a wheel joint")
        self.inertial(link, depth + 1)
        self.collisions(link, depth + 1, wheel=joint.get("name") in self.wheel_joints)
        if link_name == self.profile["lidar"]["site"]:
            self.emit(depth + 1, f'<site name="{link_name}" size="0.03"/>')
        if link_name == "tool0":
            self.panel(depth + 1)
        for child in self.children.get(link_name, []):
            self.link_body(child.find("child").get("link"), child, depth + 1)
        self.emit(depth, "</body>")

    def inertial(self, link, depth):
        inertial = link.find("inertial")
        if inertial is None:
            return
        i = inertial.find("inertia").attrib
        full = vec(float(i[k]) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"))
        self.emit(depth, f'<inertial {origin_attrs(inertial)} mass="{num(inertial.find("mass").get("value"))}" '
                         f'fullinertia="{full}"/>')

    def collisions(self, link, depth, wheel):
        for col in link.findall("collision"):
            shape = col.find("geometry")[0]
            if shape.tag == "box":
                size, kind = vec(v / 2 for v in floats(shape.get("size"))), "box"
            elif shape.tag == "cylinder":
                size, kind = vec((float(shape.get("radius")), float(shape.get("length")) / 2)), "cylinder"
            else:
                raise ValueError(f"{col.get('name')}: unsupported collision geometry {shape.tag}")
            extra = ' contype="2"' if wheel else ""
            self.emit(depth, f'<geom name="robot/{col.get("name")}" class="robot" type="{kind}" size="{size}" '
                             f'{origin_attrs(col)}{extra}/>')

    def base_geoms(self):
        """The collision geoms of the base: every link not moved by an arm joint (B5's base primitives)."""
        moved, frontier = set(), [j.find("child").get("link") for j in self.urdf.findall("joint") if j.get("name") in self.arm]
        while frontier:
            link = frontier.pop()
            moved.add(link)
            frontier += [j.find("child").get("link") for j in self.children.get(link, [])]
        return [c.get("name") for name, link in self.links.items() if name not in moved for c in link.findall("collision")]

    def panel(self, depth):
        x, y, z = self.payload["dimensions_tool_ros_m"]
        m = self.payload["mass_kg"]
        inertia = vec((m / 12 * (y * y + z * z), m / 12 * (x * x + z * z), m / 12 * (x * x + y * y)))
        self.emit(depth, f'<body name="panel" pos="{vec(self.payload["com_tool_ros_m"])}">')
        self.emit(depth + 1, f'<inertial pos="0.0 0.0 0.0" mass="{num(m)}" diaginertia="{inertia}"/>')
        self.emit(depth + 1, f'<geom name="robot/panel" class="robot" type="box" size="{vec((x / 2, y / 2, z / 2))}" '
                             f'group="2" rgba="0.85 0.6 0.2 1"/>')
        self.emit(depth, "</body>")

    def generate(self):
        root_joint = self.children["base_footprint"]
        urdf_sha = hashlib.sha256(self.urdf_path.read_bytes()).hexdigest()
        self.lines = [
            f'<mujoco model="{self.profile["robot"]}">',
            f"  <!-- Generated by tools/plant/generate_robot_model.py from {self.urdf_path.name} and "
            f"{self.profile_path.name}; do not edit. -->",
            '  <compiler angle="radian" autolimits="true"/>',
            "  <custom>",
            f'    <text name="robot_generator" data="{GENERATOR}"/>',  # robot_: unique beside the site's texts
            f'    <text name="robot_urdf_sha256" data="{urdf_sha}"/>',
            "  </custom>",
            "  <default>",
            '    <default class="robot">',
            '      <geom contype="6" conaffinity="1" group="1" rgba="0.3 0.35 0.4 1"/>',
            "    </default>",
            "  </default>",
            "  <worldbody>",
            '    <body name="base_footprint" pos="0.0 0.0 0.0">',
            '      <joint name="base_x" type="slide" axis="1 0 0"/>',
            '      <joint name="base_y" type="slide" axis="0 1 0"/>',
            '      <joint name="base_yaw" type="hinge" axis="0 0 1"/>',
        ]
        for joint in root_joint:
            self.link_body(joint.find("child").get("link"), joint, 3)
        self.lines += ["    </body>", "  </worldbody>", "  <contact>"]
        for geom in self.base_geoms():
            self.emit(2, f'<pair geom1="robot/panel" geom2="robot/{geom}"/>')
        self.lines += ["  </contact>", "  <actuator>"]
        efforts = {j.get("name"): float(j.find("limit").get("effort")) for j in self.urdf.findall("joint")
                   if j.find("limit") is not None}
        for name, kp, kd in zip(self.arm, self.profile["arm"]["kp"], self.profile["arm"]["kd"]):
            e = efforts[name]
            self.emit(2, f'<general name="servo/{name}" joint="{name}" gainprm="{num(kp)}" biastype="affine" '
                         f'biasprm="0.0 {num(-kp)} {num(-kd)}" forcerange="{num(-e)} {num(e)}" forcelimited="true"/>')
        for joint, kv in zip(("base_x", "base_y", "base_yaw"), self.profile["base"]["servo_kv"]):
            self.emit(2, f'<velocity name="drive/{joint}" joint="{joint}" kv="{num(kv)}"/>')
        self.lines += ["  </actuator>", "</mujoco>", ""]
        return "\n".join(self.lines)


def generate_robot(profile_path):
    return Generator(profile_path).generate()


def output_path(profile_path):
    profile = yaml.safe_load(Path(profile_path).read_text())
    return Path(profile_path).parent / profile["model"]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--profile", default=str(PROFILE))
    parser.add_argument("--check", action="store_true", help="exit 1 if the committed model is not current")
    args = parser.parse_args()
    text, out = generate_robot(args.profile), output_path(args.profile)
    if args.check:
        current = out.exists() and out.read_text() == text
        print("robot model current" if current else f"{out} is not current")
        return 0 if current else 1
    out.write_text(text)
    print(f"wrote {out.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
