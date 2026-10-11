#!/usr/bin/env python3
"""Generate the whole-body kinematic model of the mobile manipulator as C code (CasADi).

Build-time only: CMake runs this script from the installed description and control shares and
compiles the generated C into the MPC library, so the runtime stays C++.

State x = (x, y, yaw, q1..q6) with the arm joints in qualified_payload.json joint_order;
control u = (v, omega, qdot1..qdot6). The forward kinematics follow
mobile_manipulator_geometry::FootprintProjector::link_pose: walk from the link to
base_footprint, each joint contributing origin * rotation(axis, angle), with every joint
outside the arm (the wheels) at zero. The map -> base_footprint transform is the planar
pose (x, y, yaw).

Writes into --out:
  wholebody_model.c/.h      CasADi functions
    wholebody_step(x, u, dt) -> (x_next, A, B)          RK4 step and its Jacobians
    wholebody_link_pose(link, x) -> T (4x4)             map -> link
    wholebody_point(link, x, p_link) -> (p, J)          map point and d p / d x
    wholebody_panel_error(x, p_ref, R_ref) -> (e, J)    panel error in base_footprint
    wholebody_manipulability(x) -> m                    sqrt(det(Jt Jt^T)), tool0, arm joints
  wholebody_model_links.h   link names in link-index order
All outputs are dense and column-major.
"""
import argparse
import json
import math
import os
import sys

import casadi as ca
from urdf_parser_py import xml_reflection
from urdf_parser_py.urdf import URDF

BASE_FRAME = 'base_footprint'


def _report_parse_issue(message):
    # urdf_parser_py does not know the (valid) name attribute of visual and collision
    # elements; every other parse issue is still reported.
    if not (message.startswith('Unknown attribute "name"') and
            ('/visual[' in message or '/collision[' in message)):
        print(message, file=sys.stderr)


xml_reflection.core.on_error = _report_parse_issue
PANEL = 'panel'
NX, NU = 9, 8


def rotation_rpy(rpy):
    """URDF fixed-axis roll, pitch, yaw: Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return ca.SX(ca.DM([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]))


def rotation_axis(axis, angle):
    """Rodrigues rotation about a unit axis by a symbolic angle."""
    k = ca.SX(ca.DM(axis))
    skew = ca.skew(k)
    return ca.cos(angle) * ca.SX.eye(3) + ca.sin(angle) * skew + (1 - ca.cos(angle)) * (k @ k.T)


def transform(rotation, translation):
    """Homogeneous transform from a rotation and a translation."""
    t = ca.SX.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = translation
    return t


def origin_transform(origin):
    """Transform of a URDF origin element (absent = identity)."""
    xyz = list(origin.xyz) if origin is not None and origin.xyz is not None else [0.0, 0.0, 0.0]
    rpy = list(origin.rpy) if origin is not None and origin.rpy is not None else [0.0, 0.0, 0.0]
    return transform(rotation_rpy(rpy), ca.SX(ca.DM(xyz)))


def link_in_base(robot, link, arm_angles):
    """base_footprint -> link, joints outside the arm at zero."""
    parent_joint = {joint.child: joint for joint in robot.joints}
    pose = ca.SX.eye(4)
    current = link
    while current != BASE_FRAME:
        if current not in parent_joint:
            raise ValueError(f'Link {link} is not below {BASE_FRAME}')
        joint = parent_joint[current]
        step = origin_transform(joint.origin)
        if joint.name in arm_angles:
            if joint.type not in ('revolute', 'continuous'):
                raise ValueError(f'Arm joint {joint.name} is {joint.type}, expected revolute')
            axis = [float(a) for a in (joint.axis if joint.axis is not None else [1.0, 0.0, 0.0])]
            norm = math.sqrt(sum(a * a for a in axis))
            step = step @ transform(rotation_axis([a / norm for a in axis], arm_angles[joint.name]),
                                    ca.SX.zeros(3))
        pose = step @ pose
        current = joint.parent
    return pose


def base_in_map(x):
    """map -> base_footprint: the planar base pose (x, y, yaw)."""
    c, s = ca.cos(x[2]), ca.sin(x[2])
    t = ca.SX.eye(4)
    t[0, 0], t[0, 1], t[1, 0], t[1, 1] = c, -s, s, c
    t[0, 3], t[1, 3] = x[0], x[1]
    return t


def dynamics(x, u):
    """Unicycle base (no lateral motion) and joint velocity integrators."""
    return ca.vertcat(u[0] * ca.cos(x[2]), u[0] * ca.sin(x[2]), u[1], u[2:])


def model_links(robot, payload_link):
    """Links carrying a collision primitive (URDF order), the payload's link, then the panel."""
    links = [link.name for link in robot.links if link.collisions]
    if payload_link not in links:
        links.append(payload_link)
    if PANEL in links:
        raise ValueError(f'The URDF already has a link named {PANEL}')
    return links + [PANEL]


def build_functions(robot, qualified_payload):
    """CasADi functions of the model and the link names in link-index order."""
    joint_order = qualified_payload['joint_order']
    if len(joint_order) != 6:
        raise ValueError('joint_order must list the six arm joints')
    payload = qualified_payload['payload']
    payload_link = 'tool0'  # as mobile_manipulator_geometry::payload_from_json
    panel_center = ca.SX(ca.DM([float(v) for v in payload['com_tool_ros_m']]))

    x = ca.SX.sym('x', NX)
    u = ca.SX.sym('u', NU)
    dt = ca.SX.sym('dt')
    arm = {name: x[3 + i] for i, name in enumerate(joint_order)}

    # RK4 step.
    k1 = dynamics(x, u)
    k2 = dynamics(x + dt / 2 * k1, u)
    k3 = dynamics(x + dt / 2 * k2, u)
    k4 = dynamics(x + dt * k3, u)
    x_next = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    step = ca.Function('wholebody_step', [x, u, dt],
                       [x_next, ca.densify(ca.jacobian(x_next, x)), ca.densify(ca.jacobian(x_next, u))],
                       ['x', 'u', 'dt'], ['x_next', 'A', 'B'])

    links = model_links(robot, payload_link)
    payload_in_base = link_in_base(robot, payload_link, arm)
    panel_in_base = payload_in_base @ transform(ca.SX.eye(3), panel_center)
    in_base = {link: (panel_in_base if link == PANEL else link_in_base(robot, link, arm)) for link in links}

    p_link = ca.SX.sym('p_link', 3)
    pose_functions, point_functions = [], []
    for i, link in enumerate(links):
        pose = base_in_map(x) @ in_base[link]
        pose_functions.append(ca.Function(f'wholebody_link_pose_{i}', [x], [ca.densify(pose)]))
        p = pose[:3, :3] @ p_link + pose[:3, 3]
        point_functions.append(ca.Function(f'wholebody_point_{i}', [x, p_link],
                                           [ca.densify(p), ca.densify(ca.jacobian(p, x))]))
    link_pose = ca.Function.conditional('wholebody_link_pose', pose_functions, pose_functions[0])
    point = ca.Function.conditional('wholebody_point', point_functions, point_functions[0])

    # Panel error in base_footprint; the base pose does not enter it.
    p_ref = ca.SX.sym('p_ref', 3)
    r_ref = ca.SX.sym('R_ref', 3, 3)
    rotation, position = panel_in_base[:3, :3], panel_in_base[:3, 3]
    e = ca.vertcat(position - p_ref, 0.5 * (3 - ca.trace(r_ref.T @ rotation)))
    panel_error = ca.Function('wholebody_panel_error', [x, p_ref, r_ref],
                              [ca.densify(e), ca.densify(ca.jacobian(e, x))],
                              ['x', 'p_ref', 'R_ref'], ['e', 'J'])

    # Translational manipulability of tool0 over the arm joints (rotation-invariant, so the
    # base frame serves).
    jt = ca.jacobian(payload_in_base[:3, 3], x[3:])
    m = ca.sqrt(ca.fmax(ca.det(jt @ jt.T), 0))
    manipulability = ca.Function('wholebody_manipulability', [x], [m], ['x'], ['m'])

    return [step, link_pose, point, panel_error, manipulability], links


def links_header(links, urdf_path):
    """C header with the link names in link-index order."""
    names = ',\n  '.join(f'"{link}"' for link in links)
    return (
        '/* Generated by mobile_manipulator_mpc/scripts/generate_model.py from '
        f'{os.path.basename(urdf_path)}; do not edit. */\n'
        '#ifndef WHOLEBODY_MODEL_LINKS_H\n'
        '#define WHOLEBODY_MODEL_LINKS_H\n\n'
        '/* Link index -> link name: links with collision primitives, the payload link, panel. */\n'
        f'#define WHOLEBODY_MODEL_LINK_COUNT {len(links)}\n'
        'static const char * const wholebody_model_links[WHOLEBODY_MODEL_LINK_COUNT] = {\n'
        f'  {names}\n'
        '};\n\n'
        '#endif\n')


def main(argv=None):
    """Parse the arguments and write the generated sources."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--urdf', required=True)
    parser.add_argument('--payload', required=True, help='qualified_payload.json')
    parser.add_argument('--out', required=True, help='output directory')
    args = parser.parse_args(argv)

    with open(args.urdf, encoding='utf-8') as stream:
        robot = URDF.from_xml_string(stream.read())
    with open(args.payload, encoding='utf-8') as stream:
        qualified_payload = json.load(stream)

    functions, links = build_functions(robot, qualified_payload)
    os.makedirs(args.out, exist_ok=True)
    generator = ca.CodeGenerator('wholebody_model.c', {'with_header': True})
    for function in functions:
        generator.add(function)
    generator.generate(args.out + os.sep)
    with open(os.path.join(args.out, 'wholebody_model_links.h'), 'w', encoding='utf-8') as stream:
        stream.write(links_header(links, args.urdf))
    return 0


if __name__ == '__main__':
    sys.exit(main())
