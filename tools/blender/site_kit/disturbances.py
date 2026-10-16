"""Disturbances family (Tier 2): low chamfered pieces below the 0.05 m lidar band.

Each piece's box proxies are steps that follow its chamfer (the wheels meet a sequence of
small steps rather than one full-height edge). Cable protector: a 0.9 m segment, 0.4 m ramp
to ramp, 0.035 m high with a steel lid on top. Road plate: a 2.0 x 1.25 m steel plate,
0.025 m thick with 0.05 m ramped edges on all sides. Rumble strip: a 2.5 m bar, 0.15 m wide,
0.02 m high.
"""
from common import Mesh


def _ramp_profile(width, top_width, height):
    return [(-width / 2, 0.0), (width / 2, 0.0), (top_width / 2, height), (-top_width / 2, height)]


def cable_protector(piece, kit):
    lx, ly, h = piece["size"]
    lid_t, lid_w, ramp_run = 0.005, 0.15, (ly - 0.15) / 2
    body_h = h - lid_t
    top_width = ly - 2 * ramp_run * body_h / h  # same ramp slope as the full-height profile
    m = Mesh()
    m.prism(_ramp_profile(ly, top_width, body_h), "x", -lx / 2, lx / 2, "plastic")
    m.box((-lx / 2 + 0.01, -lid_w / 2, body_h), (lx / 2 - 0.01, lid_w / 2, h), "painted_steel")
    return m


def road_plate(piece, kit):
    lx, ly, h = piece["size"]
    ramp = 0.05
    m = Mesh()
    m.hexa([(x * (lx / 2 - (ramp if z else 0.0)), y * (ly / 2 - (ramp if z else 0.0)), z)
            for z in (0.0, h) for y in (-1, 1) for x in (-1, 1)], "painted_steel")
    return m


def rumble_strip(piece, kit):
    lx, ly, h = piece["size"]
    m = Mesh()
    m.prism(_ramp_profile(ly, 0.07, h), "x", -lx / 2, lx / 2, "plastic")
    return m


BUILDERS = {
    "cable_protector_0p035": cable_protector,
    "road_plate_0p025": road_plate,
    "rumble_strip_0p02": rumble_strip,
}
