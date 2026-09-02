"""Generated URDF source of truth for the project mobile manipulator."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET


ROBOT_NAME = "mobile_manipulator"
PACKAGE_NAME = "mobile_manipulator_description"
MESH_SCALE_FROM_MM = (0.001, 0.001, 0.001)
TOP_SENSOR_MOUNT_TO_LIVOX_M = (0.0, 0.0, 0.047)

# Hard actuator-description limits. Position, velocity, and effort belong in
# URDF; acceleration and jerk remain planning/controller requirements until
# the corresponding MoveIt and ros2_control configurations exist.
WHEEL_JOINT_LIMITS = {"effort": 85.0, "velocity": 18.0}
ARM_JOINT_LIMITS = {
    "shoulder_pan_joint": {
        "lower": -2.96705972839,
        "upper": 2.96705972839,
        "effort": 120.0,
        "velocity": 1.8,
    },
    "shoulder_lift_joint": {
        "lower": -1.74532925199,
        "upper": 1.74532925199,
        "effort": 120.0,
        "velocity": 1.6,
    },
    "elbow_joint": {
        "lower": -2.35619449019,
        "upper": 2.35619449019,
        "effort": 90.0,
        "velocity": 1.8,
    },
    "wrist_1_joint": {
        "lower": -math.pi,
        "upper": math.pi,
        "effort": 40.0,
        "velocity": 2.5,
    },
    "wrist_2_joint": {
        "lower": -2.09439510239,
        "upper": 2.09439510239,
        "effort": 30.0,
        "velocity": 2.5,
    },
    "wrist_3_joint": {
        "lower": -2.0 * math.pi,
        "upper": 2.0 * math.pi,
        "effort": 20.0,
        "velocity": 3.2,
    },
}

# Standard DH representation of the same arm kinematics. Each tuple is
# (theta_offset, d, a, alpha), in radians/metres, using
# Rz(theta) Tz(d) Tx(a) Rx(alpha). The DH base is J1 at
# base_link xyz=(-0.08, 0, 0.29), with axes aligned to base_link at q=0.
ARM_STANDARD_DH = (
    (math.pi, 0.12, 0.0, math.pi / 2.0),
    (math.pi / 2.0, 0.0, 0.32, 0.0),
    (math.pi / 2.0, 0.0, 0.0, math.pi / 2.0),
    (math.pi, 0.38, 0.0, math.pi / 2.0),
    (math.pi, 0.0, 0.0, math.pi / 2.0),
    (0.0, 0.19, 0.0, 0.0),
)


def _fmt(values):
    return " ".join(f"{value:.12g}" for value in values)


def _origin(parent, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    ET.SubElement(parent, "origin", {"xyz": _fmt(xyz), "rpy": _fmt(rpy)})


def _box_inertia(mass, size):
    x, y, z = size
    return (
        mass * (y * y + z * z) / 12.0,
        mass * (x * x + z * z) / 12.0,
        mass * (x * x + y * y) / 12.0,
    )


def _cylinder_z_inertia(mass, radius, length):
    transverse = mass * (3.0 * radius * radius + length * length) / 12.0
    axial = 0.5 * mass * radius * radius
    return transverse, transverse, axial


def _cylinder_y_inertia(mass, radius, length):
    transverse = mass * (3.0 * radius * radius + length * length) / 12.0
    axial = 0.5 * mass * radius * radius
    return transverse, axial, transverse


def _add_materials(robot):
    colors = {
        "chassis_blue": (0.055, 0.22, 0.42, 1.0),
        "tire_black": (0.025, 0.025, 0.03, 1.0),
        "arm_orange": (0.9, 0.28, 0.04, 1.0),
        "joint_dark": (0.12, 0.14, 0.17, 1.0),
        "flange_silver": (0.55, 0.58, 0.62, 1.0),
    }
    for name, rgba in colors.items():
        material = ET.SubElement(robot, "material", {"name": name})
        ET.SubElement(material, "color", {"rgba": _fmt(rgba)})


def _add_inertial(link, mass, com, inertia):
    inertial = ET.SubElement(link, "inertial")
    _origin(inertial, com)
    ET.SubElement(inertial, "mass", {"value": f"{mass:.12g}"})
    ixx, iyy, izz = inertia
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": f"{ixx:.12g}",
            "ixy": "0",
            "ixz": "0",
            "iyy": f"{iyy:.12g}",
            "iyz": "0",
            "izz": f"{izz:.12g}",
        },
    )


def _add_mesh_visual(link, mesh_name, material_name):
    visual = ET.SubElement(link, "visual", {"name": f"{mesh_name}_visual"})
    _origin(visual)
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(
        geometry,
        "mesh",
        {
            "filename": f"package://{PACKAGE_NAME}/meshes/visual/{mesh_name}.stl",
            "scale": _fmt(MESH_SCALE_FROM_MM),
        },
    )
    ET.SubElement(visual, "material", {"name": material_name})


def _add_box_collision(link, name, size, xyz=(0.0, 0.0, 0.0)):
    collision = ET.SubElement(link, "collision", {"name": name})
    _origin(collision, xyz)
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry, "box", {"size": _fmt(size)})


def _add_cylinder_collision(
    link,
    name,
    radius,
    length,
    xyz=(0.0, 0.0, 0.0),
    rpy=(0.0, 0.0, 0.0),
):
    collision = ET.SubElement(link, "collision", {"name": name})
    _origin(collision, xyz, rpy)
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(
        geometry,
        "cylinder",
        {"radius": f"{radius:.12g}", "length": f"{length:.12g}"},
    )


def _add_joint(
    robot,
    name,
    joint_type,
    parent,
    child,
    xyz=(0.0, 0.0, 0.0),
    rpy=(0.0, 0.0, 0.0),
    axis=None,
    limits=None,
    damping=None,
    friction=None,
):
    joint = ET.SubElement(robot, "joint", {"name": name, "type": joint_type})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    _origin(joint, xyz, rpy)
    if axis is not None:
        ET.SubElement(joint, "axis", {"xyz": _fmt(axis)})
    if limits is not None:
        attributes = {
            "effort": f"{limits['effort']:.12g}",
            "velocity": f"{limits['velocity']:.12g}",
        }
        if joint_type != "continuous":
            attributes["lower"] = f"{limits['lower']:.12g}"
            attributes["upper"] = f"{limits['upper']:.12g}"
        ET.SubElement(joint, "limit", attributes)
    if damping is not None or friction is not None:
        ET.SubElement(
            joint,
            "dynamics",
            {
                "damping": f"{(damping or 0.0):.12g}",
                "friction": f"{(friction or 0.0):.12g}",
            },
        )


def _add_wheel(robot, position_name, xyz):
    link_name = f"{position_name}_wheel_link"
    link = ET.SubElement(robot, "link", {"name": link_name})
    mass = 4.0
    _add_inertial(link, mass, (0.0, 0.0, 0.0), _cylinder_y_inertia(mass, 0.14, 0.09))
    _add_mesh_visual(link, "wheel_link", "tire_black")
    _add_cylinder_collision(
        link,
        f"{position_name}_wheel_collision",
        0.14,
        0.09,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    _add_joint(
        robot,
        f"{position_name}_wheel_joint",
        "continuous",
        "base_link",
        link_name,
        xyz=xyz,
        axis=(0.0, 1.0, 0.0),
        limits=WHEEL_JOINT_LIMITS,
        damping=0.35,
        friction=0.08,
    )


def gen_urdf():
    robot = ET.Element("robot", {"name": ROBOT_NAME})
    _add_materials(robot)

    ET.SubElement(robot, "link", {"name": "base_footprint"})

    base = ET.SubElement(robot, "link", {"name": "base_link"})
    base_mass = 55.0
    _add_inertial(base, base_mass, (0.0, 0.0, 0.0), _box_inertia(base_mass, (0.85, 0.55, 0.22)))
    _add_mesh_visual(base, "base_link", "chassis_blue")
    _add_box_collision(base, "base_chassis_collision", (0.85, 0.59, 0.22))
    _add_joint(
        robot,
        "base_footprint_joint",
        "fixed",
        "base_footprint",
        "base_link",
        xyz=(0.0, 0.0, 0.21),
    )

    _add_wheel(robot, "front_left", (0.30, 0.32, -0.07))
    _add_wheel(robot, "front_right", (0.30, -0.32, -0.07))
    _add_wheel(robot, "rear_left", (-0.30, 0.32, -0.07))
    _add_wheel(robot, "rear_right", (-0.30, -0.32, -0.07))

    mount = ET.SubElement(robot, "link", {"name": "arm_mount_link"})
    mount_mass = 10.0
    _add_inertial(mount, mount_mass, (0.0, 0.0, 0.09), _cylinder_z_inertia(mount_mass, 0.10, 0.18))
    _add_mesh_visual(mount, "arm_mount_link", "joint_dark")
    _add_cylinder_collision(mount, "arm_mount_collision", 0.112, 0.18, xyz=(0.0, 0.0, 0.09))
    _add_joint(
        robot,
        "arm_mount_joint",
        "fixed",
        "base_link",
        "arm_mount_link",
        xyz=(-0.08, 0.0, 0.11),
    )

    shoulder = ET.SubElement(robot, "link", {"name": "shoulder_pan_link"})
    shoulder_mass = 7.0
    _add_inertial(
        shoulder,
        shoulder_mass,
        (0.0, 0.0, 0.06),
        _cylinder_z_inertia(shoulder_mass, 0.10, 0.12),
    )
    _add_mesh_visual(shoulder, "shoulder_pan_link", "joint_dark")
    _add_cylinder_collision(shoulder, "shoulder_pan_collision", 0.105, 0.12, xyz=(0.0, 0.0, 0.06))
    _add_joint(
        robot,
        "shoulder_pan_joint",
        "revolute",
        "arm_mount_link",
        "shoulder_pan_link",
        xyz=(0.0, 0.0, 0.18),
        axis=(0.0, 0.0, 1.0),
        limits=ARM_JOINT_LIMITS["shoulder_pan_joint"],
        damping=2.0,
        friction=0.3,
    )

    upper = ET.SubElement(robot, "link", {"name": "upper_arm_link"})
    upper_mass = 8.0
    _add_inertial(upper, upper_mass, (0.0, 0.0, 0.16), _box_inertia(upper_mass, (0.112, 0.10, 0.32)))
    _add_mesh_visual(upper, "upper_arm_link", "arm_orange")
    _add_box_collision(upper, "upper_arm_collision", (0.12, 0.11, 0.32), xyz=(0.0, 0.0, 0.16))
    _add_joint(
        robot,
        "shoulder_lift_joint",
        "revolute",
        "shoulder_pan_link",
        "upper_arm_link",
        xyz=(0.0, 0.0, 0.12),
        axis=(0.0, 1.0, 0.0),
        limits=ARM_JOINT_LIMITS["shoulder_lift_joint"],
        damping=2.2,
        friction=0.25,
    )

    forearm = ET.SubElement(robot, "link", {"name": "forearm_link"})
    forearm_mass = 5.0
    _add_inertial(
        forearm,
        forearm_mass,
        (0.0, 0.0, 0.14),
        _box_inertia(forearm_mass, (0.092, 0.084, 0.28)),
    )
    _add_mesh_visual(forearm, "forearm_link", "arm_orange")
    _add_box_collision(forearm, "forearm_collision", (0.10, 0.092, 0.28), xyz=(0.0, 0.0, 0.14))
    _add_joint(
        robot,
        "elbow_joint",
        "revolute",
        "upper_arm_link",
        "forearm_link",
        xyz=(0.0, 0.0, 0.32),
        axis=(0.0, 1.0, 0.0),
        limits=ARM_JOINT_LIMITS["elbow_joint"],
        damping=1.8,
        friction=0.2,
    )

    wrist_1 = ET.SubElement(robot, "link", {"name": "wrist_1_link"})
    wrist_1_mass = 2.5
    _add_inertial(
        wrist_1,
        wrist_1_mass,
        (0.0, 0.0, 0.05),
        _cylinder_z_inertia(wrist_1_mass, 0.058, 0.10),
    )
    _add_mesh_visual(wrist_1, "wrist_1_link", "joint_dark")
    _add_cylinder_collision(wrist_1, "wrist_1_collision", 0.06, 0.10, xyz=(0.0, 0.0, 0.05))
    _add_joint(
        robot,
        "wrist_1_joint",
        "revolute",
        "forearm_link",
        "wrist_1_link",
        xyz=(0.0, 0.0, 0.28),
        axis=(0.0, 0.0, 1.0),
        limits=ARM_JOINT_LIMITS["wrist_1_joint"],
        damping=0.9,
        friction=0.12,
    )

    wrist_2 = ET.SubElement(robot, "link", {"name": "wrist_2_link"})
    wrist_2_mass = 2.0
    _add_inertial(wrist_2, wrist_2_mass, (0.0, 0.0, 0.05), _box_inertia(wrist_2_mass, (0.092, 0.082, 0.10)))
    _add_mesh_visual(wrist_2, "wrist_2_link", "arm_orange")
    _add_cylinder_collision(wrist_2, "wrist_2_collision", 0.052, 0.10, xyz=(0.0, 0.0, 0.05))
    _add_joint(
        robot,
        "wrist_2_joint",
        "revolute",
        "wrist_1_link",
        "wrist_2_link",
        xyz=(0.0, 0.0, 0.10),
        axis=(0.0, 1.0, 0.0),
        limits=ARM_JOINT_LIMITS["wrist_2_joint"],
        damping=0.8,
        friction=0.1,
    )

    wrist_3 = ET.SubElement(robot, "link", {"name": "wrist_3_link"})
    wrist_3_mass = 1.5
    _add_inertial(
        wrist_3,
        wrist_3_mass,
        (0.0, 0.0, 0.045),
        _cylinder_z_inertia(wrist_3_mass, 0.05, 0.09),
    )
    _add_mesh_visual(wrist_3, "wrist_3_link", "flange_silver")
    _add_cylinder_collision(wrist_3, "wrist_3_collision", 0.055, 0.09, xyz=(0.0, 0.0, 0.045))
    _add_joint(
        robot,
        "wrist_3_joint",
        "revolute",
        "wrist_2_link",
        "wrist_3_link",
        xyz=(0.0, 0.0, 0.10),
        axis=(0.0, 0.0, 1.0),
        limits=ARM_JOINT_LIMITS["wrist_3_joint"],
        damping=0.6,
        friction=0.08,
    )

    ET.SubElement(robot, "link", {"name": "tool0"})
    _add_joint(
        robot,
        "tool0_joint",
        "fixed",
        "wrist_3_link",
        "tool0",
        xyz=(0.0, 0.0, 0.09),
    )

    ET.SubElement(robot, "link", {"name": "top_sensor_mount_link"})
    _add_joint(
        robot,
        "top_sensor_mount_joint",
        "fixed",
        "base_link",
        "top_sensor_mount_link",
        xyz=(0.24, 0.0, 0.13),
    )

    # Measured from the UnitySensorsROS Livox Mid-360 prefab: the
    # raycast/point-cloud origin is 47 mm above the mechanical mount.
    ET.SubElement(robot, "link", {"name": "livox_frame"})
    _add_joint(
        robot,
        "livox_joint",
        "fixed",
        "top_sensor_mount_link",
        "livox_frame",
        xyz=TOP_SENSOR_MOUNT_TO_LIVOX_M,
    )

    ET.SubElement(robot, "link", {"name": "front_sensor_mount_link"})
    _add_joint(
        robot,
        "front_sensor_mount_joint",
        "fixed",
        "base_link",
        "front_sensor_mount_link",
        xyz=(0.44, 0.0, 0.02),
    )

    ET.SubElement(robot, "link", {"name": "tool_sensor_mount_link"})
    _add_joint(
        robot,
        "tool_sensor_mount_joint",
        "fixed",
        "tool0",
        "tool_sensor_mount_link",
    )

    return robot
