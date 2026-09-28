"""Pure point selection for the Livox robot filter (no ROS dependencies).

Self-returns are removed with a geometric self-model: the URDF collision primitives and
the rigidly attached payload panel, each posed from TF. Because the simulated lidar's
noise is along the ray (Gaussian range noise), a point is a self-return when its ray
from the sensor first hits the robot and the point lies no more than `noise_band` in
front of that surface. Points inside a primitive enlarged by `margin` are also removed.
Obstacles between the sensor and the robot, or off the robot's rays, are kept even
inside the footprint rectangle, so safety consumers still see them.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def transform(rotation, translation):
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def load_primitives(urdf_text, payload=None):
    """Collision primitives as (link, T_link_primitive, kind, dims).

    kind "box": dims = half extents (x, y, z); kind "cylinder": dims = (radius, half length)
    along the primitive's local z. payload: dict with "link", "size" (full x, y, z) and
    "center" (x, y, z) in that link's frame, added as a box.
    """
    primitives = []
    root = ET.fromstring(urdf_text)
    for link in root.findall("link"):
        for collision in link.findall("collision"):
            origin = collision.find("origin")
            xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
            rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
            geometry = collision.find("geometry")[0]
            pose = transform(rpy_matrix(*rpy), xyz)
            if geometry.tag == "box":
                dims = tuple(float(v) / 2.0 for v in geometry.get("size").split())
                primitives.append((link.get("name"), pose, "box", dims))
            elif geometry.tag == "cylinder":
                dims = (float(geometry.get("radius")), float(geometry.get("length")) / 2.0)
                primitives.append((link.get("name"), pose, "cylinder", dims))
            else:
                raise ValueError(f"Unsupported collision geometry {geometry.tag}")
    if payload is not None:
        half = tuple(v / 2.0 for v in payload["size"])
        primitives.append((payload["link"], transform(np.eye(3), payload["center"]), "box", half))
    return primitives


def self_mask(points_base, posed_primitives, margin):
    """True for points inside any posed primitive enlarged by margin.

    posed_primitives: iterable of (T_base_primitive, kind, dims).
    """
    points = np.asarray(points_base, dtype=np.float64)
    inside = np.zeros(len(points), dtype=bool)
    homogeneous = np.column_stack((points, np.ones(len(points))))
    for pose, kind, dims in posed_primitives:
        local = (homogeneous @ np.linalg.inv(pose).T)[:, :3]
        if kind == "box":
            hit = np.all(np.abs(local) <= np.asarray(dims) + margin, axis=1)
        else:
            radius, half_length = dims
            hit = ((np.hypot(local[:, 0], local[:, 1]) <= radius + margin) &
                   (np.abs(local[:, 2]) <= half_length + margin))
        inside |= hit
    return inside


def first_self_hit(origin, directions, posed_primitives):
    """Distance along each unit ray to the nearest posed primitive (inf if none)."""
    nearest = np.full(len(directions), np.inf)
    for pose, kind, dims in posed_primitives:
        # Cheap cull: only rays through the primitive's bounding sphere are intersected.
        radius_bound = float(np.linalg.norm(dims)) if kind == "box" else math.hypot(*dims)
        offset = pose[:3, 3] - origin
        along = directions @ offset
        rays = np.nonzero((along + radius_bound > 0) &
                          (offset @ offset - along * along <= radius_bound ** 2))[0]
        if not len(rays):
            continue
        inverse = np.linalg.inv(pose)
        o = inverse[:3, :3] @ origin + inverse[:3, 3]
        d = directions[rays] @ inverse[:3, :3].T
        with np.errstate(divide="ignore", invalid="ignore"):
            if kind == "box":
                half = np.asarray(dims)
                t1, t2 = (-half - o) / d, (half - o) / d
                t_near = np.nanmax(np.minimum(t1, t2), axis=1)
                t_far = np.nanmin(np.maximum(t1, t2), axis=1)
                hit = (t_near <= t_far) & (t_far > 0)
                t = np.where(t_near > 0, t_near, 0.0)
            else:
                radius, half_length = dims
                a = d[:, 0] ** 2 + d[:, 1] ** 2
                b = 2 * (o[0] * d[:, 0] + o[1] * d[:, 1])
                c = o[0] ** 2 + o[1] ** 2 - radius ** 2
                disc = b * b - 4 * a * c
                root = np.sqrt(np.maximum(disc, 0.0))
                candidates = []
                for side in ((-b - root) / (2 * a), (-b + root) / (2 * a)):
                    z = o[2] + side * d[:, 2]
                    candidates.append(np.where((disc >= 0) & (a > 0) & (np.abs(z) <= half_length)
                                               & (side > 0), side, np.inf))
                for cap in (-half_length, half_length):
                    side = (cap - o[2]) / d[:, 2]
                    x, y = o[0] + side * d[:, 0], o[1] + side * d[:, 1]
                    candidates.append(np.where((x * x + y * y <= radius ** 2) & (side > 0), side, np.inf))
                t = np.min(np.stack(candidates), axis=0)
                hit = np.isfinite(t)
        nearest[rays] = np.where(hit, np.minimum(nearest[rays], t), nearest[rays])
    return nearest


def keep_mask(points_sensor, sensor_to_base, posed_primitives, margin, noise_band=0.0):
    """Mask of points to keep: not a UnitySensors miss and not a robot self-return.

    points_sensor: (N, 3) in the sensor frame; sensor_to_base: 4x4 transform.
    noise_band: range noise allowance in front of a self surface (about 4 sigma).
    Ground and overhead returns are kept: consumers apply their own marking heights,
    and ground rays are what clear low obstacles near the robot.
    """
    points = np.asarray(points_sensor, dtype=np.float64)
    miss = ~np.any(points != 0.0, axis=1)
    base = points @ sensor_to_base[:3, :3].T + sensor_to_base[:3, 3]
    self_return = self_mask(base, posed_primitives, margin)
    if noise_band > 0.0:
        ranges = np.linalg.norm(points, axis=1)
        directions = (base - sensor_to_base[:3, 3]) / np.where(ranges > 0, ranges, 1.0)[:, None]
        surface = first_self_hit(sensor_to_base[:3, 3], directions, posed_primitives)
        self_return |= ranges >= surface - noise_band
    return ~miss & ~self_return
