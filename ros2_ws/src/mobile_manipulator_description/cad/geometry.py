"""Parametric CAD geometry for the project mobile manipulator.

CAD and STL geometry use millimetres. Every link-local shape is authored at the
corresponding URDF link frame. The neutral assembly uses the same transforms as
the URDF generator.
"""

from __future__ import annotations

from build123d import Align, Axis, Box, Compound, Cylinder, Location


CHASSIS_LENGTH = 850.0
CHASSIS_WIDTH = 550.0
CHASSIS_HEIGHT = 220.0
WHEEL_RADIUS = 140.0
WHEEL_WIDTH = 90.0
WHEEL_X = 300.0
WHEEL_Y = 320.0
WHEEL_Z = -70.0
ARM_X = -80.0


def _box(x: float, y: float, z: float, *, center_z: float = 0.0):
    shape = Box(x, y, z, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    return Location((0.0, 0.0, center_z)) * shape


def _cylinder_z(radius: float, height: float, *, center_z: float = 0.0):
    shape = Cylinder(radius, height, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    return Location((0.0, 0.0, center_z)) * shape


def _compound(label: str, *children):
    for index, child in enumerate(children):
        child.label = f"{label}_body_{index + 1}"
    return Compound(children=list(children), label=label)


def make_base_link():
    chassis = _box(CHASSIS_LENGTH, CHASSIS_WIDTH, CHASSIS_HEIGHT)
    lower_bumper = Location((340.0, 0.0, -70.0)) * _box(120.0, 590.0, 55.0)
    top_deck = Location((-60.0, 0.0, 120.0)) * _box(520.0, 460.0, 20.0)
    return _compound("base_link", chassis, lower_bumper, top_deck)


def make_wheel_link():
    tire = Cylinder(
        WHEEL_RADIUS,
        WHEEL_WIDTH,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).rotate(Axis.X, 90.0)
    hub = Cylinder(
        62.0,
        WHEEL_WIDTH + 8.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).rotate(Axis.X, 90.0)
    return _compound("wheel_link", tire, hub)


def make_arm_mount_link():
    plate = _box(240.0, 240.0, 24.0, center_z=12.0)
    pedestal = _cylinder_z(92.0, 150.0, center_z=87.0)
    top = _cylinder_z(112.0, 30.0, center_z=165.0)
    return _compound("arm_mount_link", plate, pedestal, top)


def make_shoulder_pan_link():
    rotary = _cylinder_z(102.0, 92.0, center_z=46.0)
    shoulder = _box(190.0, 150.0, 72.0, center_z=96.0)
    return _compound("shoulder_pan_link", rotary, shoulder)


def make_upper_arm_link():
    beam = _box(112.0, 100.0, 290.0, center_z=160.0)
    lower_housing = _cylinder_z(82.0, 82.0, center_z=35.0)
    upper_housing = _cylinder_z(72.0, 82.0, center_z=300.0)
    return _compound("upper_arm_link", lower_housing, beam, upper_housing)


def make_forearm_link():
    beam = _box(92.0, 84.0, 250.0, center_z=140.0)
    lower_housing = _cylinder_z(70.0, 76.0, center_z=32.0)
    upper_housing = _cylinder_z(58.0, 72.0, center_z=258.0)
    return _compound("forearm_link", lower_housing, beam, upper_housing)


def make_wrist_1_link():
    lower = _cylinder_z(58.0, 68.0, center_z=34.0)
    upper = _cylinder_z(48.0, 46.0, center_z=77.0)
    return _compound("wrist_1_link", lower, upper)


def make_wrist_2_link():
    housing = _box(92.0, 82.0, 72.0, center_z=36.0)
    collar = _cylinder_z(45.0, 42.0, center_z=79.0)
    return _compound("wrist_2_link", housing, collar)


def make_wrist_3_link():
    barrel = _cylinder_z(44.0, 74.0, center_z=37.0)
    flange = _cylinder_z(68.0, 18.0, center_z=81.0)
    return _compound("wrist_3_link", barrel, flange)


LINK_BUILDERS = {
    "base_link": make_base_link,
    "wheel_link": make_wheel_link,
    "arm_mount_link": make_arm_mount_link,
    "shoulder_pan_link": make_shoulder_pan_link,
    "upper_arm_link": make_upper_arm_link,
    "forearm_link": make_forearm_link,
    "wrist_1_link": make_wrist_1_link,
    "wrist_2_link": make_wrist_2_link,
    "wrist_3_link": make_wrist_3_link,
}


def make_link(link_name: str):
    try:
        return LINK_BUILDERS[link_name]()
    except KeyError as exc:
        raise ValueError(f"Unknown link geometry: {link_name}") from exc


def make_neutral_assembly():
    """Return a labeled neutral-pose assembly in the base_link frame."""

    parts = []

    base = make_base_link()
    base.label = "base_link"
    parts.append(base)

    for name, xyz in (
        ("front_left_wheel_link", (WHEEL_X, WHEEL_Y, WHEEL_Z)),
        ("front_right_wheel_link", (WHEEL_X, -WHEEL_Y, WHEEL_Z)),
        ("rear_left_wheel_link", (-WHEEL_X, WHEEL_Y, WHEEL_Z)),
        ("rear_right_wheel_link", (-WHEEL_X, -WHEEL_Y, WHEEL_Z)),
    ):
        wheel = Location(xyz) * make_wheel_link()
        wheel.label = name
        parts.append(wheel)

    mount = Location((ARM_X, 0.0, 110.0)) * make_arm_mount_link()
    mount.label = "arm_mount_link"
    parts.append(mount)

    shoulder = Location((ARM_X, 0.0, 290.0)) * make_shoulder_pan_link()
    shoulder.label = "shoulder_pan_link"
    parts.append(shoulder)

    upper = Location((ARM_X, 0.0, 410.0)) * make_upper_arm_link()
    upper.label = "upper_arm_link"
    parts.append(upper)

    forearm = Location((ARM_X, 0.0, 730.0)) * make_forearm_link()
    forearm.label = "forearm_link"
    parts.append(forearm)

    wrist_1 = Location((ARM_X, 0.0, 1010.0)) * make_wrist_1_link()
    wrist_1.label = "wrist_1_link"
    parts.append(wrist_1)

    wrist_2 = Location((ARM_X, 0.0, 1110.0)) * make_wrist_2_link()
    wrist_2.label = "wrist_2_link"
    parts.append(wrist_2)

    wrist_3 = Location((ARM_X, 0.0, 1210.0)) * make_wrist_3_link()
    wrist_3.label = "wrist_3_link"
    parts.append(wrist_3)

    return Compound(children=parts, label="mobile_manipulator")
