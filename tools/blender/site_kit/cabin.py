"""Cabin family: 20 ft steel site cabins (office, welfare, drying room).

A painted body inside a steel frame (corner posts, bottom and top rails); vertical ribs
suggest the corrugated walls. Windows have frames, glazing and security grilles; doors are
steel. Everything stays inside the 6.058 x 2.438 x 2.591 m container envelope, with the ribs,
frames and doors in the outer 0.02 m of each wall. The front is the -y side.
"""
from common import Mesh

RIB = 0.02
POST = 0.15
BOTTOM_RAIL = 0.16
TOP_RAIL = 0.15
RIB_PITCH = 0.2
RIB_WIDTH = 0.05

# Openings per wall: (kind, centre along the wall, width, bottom, top). Walls run left to right
# as seen from outside: front (-y) and back (+y) along x, left (-x) and right (+x) along y.
LAYOUTS = {
    "cabin_office": {
        "front": [("window", -1.7, 1.2, 0.95, 2.05), ("window", 0.1, 1.2, 0.95, 2.05), ("door", 1.95, 0.9, None, 2.2)],
        "back": [("window", -1.5, 1.2, 0.95, 2.05), ("window", 1.5, 1.2, 0.95, 2.05)],
        "left": [("window", 0.0, 1.0, 0.95, 2.05)],
        "right": [],
    },
    "cabin_welfare": {
        "front": [("door", -2.0, 0.9, None, 2.2), ("window", -0.6, 0.6, 1.55, 2.1), ("window", 0.6, 0.6, 1.55, 2.1),
                  ("window", 1.8, 0.6, 1.55, 2.1)],
        "back": [("vent", -1.5, 0.4, 1.9, 2.2), ("vent", 1.5, 0.4, 1.9, 2.2)],
        "left": [],
        "right": [("vent", 0.0, 0.4, 1.9, 2.2)],
    },
    "cabin_drying": {
        "front": [("window", -1.0, 0.9, 1.1, 2.0), ("door", 1.9, 0.9, None, 2.2), ("vent", -2.3, 0.35, 0.4, 0.7),
                  ("vent", 0.4, 0.35, 0.4, 0.7)],
        "back": [("vent", -1.5, 0.35, 1.9, 2.2), ("vent", 1.5, 0.35, 1.9, 2.2)],
        "left": [("vent", 0.0, 0.35, 1.9, 2.2)],
        "right": [],
    },
}


class _Walls:
    """Boxes on a wall given along-wall u, height z and outward depth d (0 at the body face)."""

    def __init__(self, mesh, size):
        self.m = mesh
        self.lx, self.ly, self.h = size

    def span(self, wall):
        return (self.lx if wall in ("front", "back") else self.ly) / 2 - POST

    def box(self, wall, u0, u1, z0, z1, d0, d1, material):
        hx, hy = self.lx / 2 - RIB, self.ly / 2 - RIB
        if wall == "front":
            lo, hi = (u0, -hy - d1, z0), (u1, -hy - d0, z1)
        elif wall == "back":
            lo, hi = (-u1, hy + d0, z0), (-u0, hy + d1, z1)
        elif wall == "left":
            lo, hi = (-hx - d1, -u1, z0), (-hx - d0, -u0, z1)
        else:
            lo, hi = (hx + d0, u0, z0), (hx + d1, u1, z1)
        self.m.box(lo, hi, material)


def _window(w, wall, u, width, z0, z1):
    u0, u1, f = u - width / 2, u + width / 2, 0.05
    w.box(wall, u0 + f, u1 - f, z0 + f, z1 - f, 0.0, 0.004, "plastic")
    for a, b, c, d in ((u0, u1, z0, z0 + f), (u0, u1, z1 - f, z1), (u0, u0 + f, z0 + f, z1 - f),
                       (u1 - f, u1, z0 + f, z1 - f), (u - 0.02, u + 0.02, z0 + f, z1 - f)):
        w.box(wall, a, b, c, d, 0.0, 0.012, "galvanized_steel")
    bars = max(2, int(round(width / 0.12)))
    for k in range(1, bars):
        x = u0 + width * k / bars
        w.box(wall, x - 0.008, x + 0.008, z0 - 0.04, z1 + 0.04, 0.012, RIB, "painted_steel")
    for z in (z0 - 0.04, z1 + 0.01):
        w.box(wall, u0 - 0.03, u1 + 0.03, z, z + 0.03, 0.012, RIB, "painted_steel")


def _door(w, wall, u, width, top):
    u0, u1 = u - width / 2, u + width / 2
    w.box(wall, u0 - 0.05, u1 + 0.05, top, top + 0.05, 0.0, 0.012, "painted_steel")
    for a, b in ((u0 - 0.05, u0), (u1, u1 + 0.05)):
        w.box(wall, a, b, BOTTOM_RAIL, top, 0.0, 0.012, "painted_steel")
    w.box(wall, u0, u1, BOTTOM_RAIL, top, 0.0, 0.01, "painted_steel")
    w.box(wall, u1 - 0.16, u1 - 0.06, 1.0, 1.06, 0.01, RIB, "galvanized_steel")
    w.box(wall, u1 - 0.12, u1 - 0.08, 1.12, 1.2, 0.01, 0.016, "galvanized_steel")


def _vent(w, wall, u, width, z0, z1):
    u0, u1 = u - width / 2, u + width / 2
    w.box(wall, u0, u1, z0, z1, 0.0, 0.006, "painted_steel")
    slats = max(2, int(round((z1 - z0) / 0.05)))
    for k in range(slats):
        z = z0 + (z1 - z0) * (k + 0.5) / slats
        w.box(wall, u0 + 0.02, u1 - 0.02, z - 0.012, z + 0.012, 0.006, 0.016, "galvanized_steel")


def _ribs(w, wall, openings):
    span = w.span(wall)
    blocked = [(u - width / 2 - 0.05, u + width / 2 + 0.05) for _, u, width, _, _ in openings]
    u = -span + RIB_PITCH / 2
    while u < span - RIB_WIDTH:
        if not any(a < u + RIB_WIDTH and u - RIB_WIDTH < b for a, b in blocked):
            w.box(wall, u - RIB_WIDTH / 2, u + RIB_WIDTH / 2, BOTTOM_RAIL, w.h - TOP_RAIL, 0.0, RIB, "cabin_paint")
        u += RIB_PITCH


def cabin(piece, kit):
    lx, ly, h = piece["size"]
    m = Mesh()
    hx, hy = lx / 2, ly / 2
    m.box((-hx + RIB, -hy + RIB, BOTTOM_RAIL), (hx - RIB, hy - RIB, h - 0.02), "cabin_paint")
    for sx in (-1, 1):
        for sy in (-1, 1):
            x0, x1 = sorted((sx * hx, sx * (hx - POST)))
            y0, y1 = sorted((sy * hy, sy * (hy - POST)))
            m.box((x0, y0, 0.0), (x1, y1, h), "painted_steel")
    for z0, z1 in ((0.0, BOTTOM_RAIL), (h - TOP_RAIL, h)):
        for sy in (-1, 1):
            y0, y1 = sorted((sy * hy, sy * (hy - POST)))
            m.box((-hx + POST, y0, z0), (hx - POST, y1, z1), "painted_steel")
        for sx in (-1, 1):
            x0, x1 = sorted((sx * hx, sx * (hx - POST)))
            m.box((x0, -hy + POST, z0), (x1, hy - POST, z1), "painted_steel")
    w = _Walls(m, (lx, ly, h))
    for wall, openings in LAYOUTS[piece["id"]].items():
        _ribs(w, wall, openings)
        for kind, u, width, z0, z1 in openings:
            if kind == "window":
                _window(w, wall, u, width, z0, z1)
            elif kind == "door":
                _door(w, wall, u, width, z1)
            else:
                _vent(w, wall, u, width, z0, z1)
    return m


BUILDERS = {"cabin_office": cabin, "cabin_welfare": cabin, "cabin_drying": cabin}
