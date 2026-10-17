"""Props family: builder's skip, IBC tank and the spoil heap.

Skip: a 4-yard steel skip with sloped ends, runners, side stiffeners and lifting lugs. IBC:
a 1000 l tank in a galvanized tube cage on a pallet base. Spoil heap: a noisy mound built from
rounded contour rings that stay inside its three stepped box proxies (each tier box holds the
contours of its height band, including the band's lower ring, so the faces stay inside too).
"""
import math
import random

from common import Mesh


def skip(piece, kit):
    lx, ly, h = piece["size"]
    top_x, floor_x, body_y, wall, base = lx / 2, 0.75, ly / 2 - 0.07, 0.04, 0.06
    m = Mesh()
    for y in (-0.4, 0.4):
        m.box((-floor_x + 0.05, y - 0.05, 0.0), (floor_x - 0.05, y + 0.05, base), "painted_steel")
    m.box((-floor_x, -body_y, base), (floor_x, body_y, base + wall), "painted_steel")
    profile = [(-floor_x, base), (floor_x, base), (top_x, h), (-top_x, h)]
    for side in (-1, 1):
        y0, y1 = sorted((side * body_y, side * (body_y - wall)))
        m.prism(profile, "y", y0, y1, "painted_steel")
    for side in (-1, 1):
        bottom, top = side * floor_x, side * top_x
        inset = -side * wall
        m.hexa([(x, y, z) for z, xs in ((base, (bottom, bottom + inset)), (h, (top, top + inset)))
                for y in (-body_y + wall, body_y - wall) for x in sorted(xs)], "painted_steel")
    for side in (-1, 1):
        y0, y1 = sorted((side * body_y, side * (body_y + 0.03)))
        for x in (-0.45, 0.0, 0.45):
            m.box((x - 0.03, y0, base), (x + 0.03, y1, h - 0.06), "painted_steel")
        m.box((-top_x + 0.05, y0, h - 0.06), (top_x - 0.05, y1, h), "painted_steel")
        for x in (-0.72, 0.72):
            m.cylinder((x, side * body_y, 0.72), (x, side * ly / 2, 0.72), 0.045, "galvanized_steel", sides=10)
    return m


def ibc_tank(piece, kit):
    lx, ly, h = piece["size"]
    hx, hy = lx / 2, ly / 2
    base_top, tank_top, r = 0.15, h - 0.07, 0.01
    m = Mesh()
    for x in (-hx + 0.05, 0.0, hx - 0.05):
        for y in (-hy + 0.05, 0.0, hy - 0.05):
            m.box((x - 0.05, y - 0.05, 0.0), (x + 0.05, y + 0.05, 0.1), "galvanized_steel")
    m.box((-hx, -hy, 0.1), (hx, hy, base_top), "galvanized_steel")
    m.chamfer_box((-hx + 0.04, -hy + 0.04, base_top), (hx - 0.04, hy - 0.04, tank_top), 0.06, "plastic")
    m.cylinder((0, 0, tank_top - 0.01), (0, 0, h), 0.08, "plastic", sides=12)
    m.cylinder((0, -hy + 0.04, 0.27), (0, -hy + r, 0.27), 0.04, "plastic", sides=10)
    cx, cy, top = hx - r, hy - r, tank_top + 0.03
    for x in [-cx + 2 * cx * k / 6 for k in range(7)]:
        for y in (-cy, cy):
            m.cylinder((x, y, base_top), (x, y, top), r, "galvanized_steel", sides=6, caps=False)
    for y in [-cy + 2 * cy * k / 5 for k in range(1, 5)]:
        for x in (-cx, cx):
            m.cylinder((x, y, base_top), (x, y, top), r, "galvanized_steel", sides=6, caps=False)
    for z in (0.45, 0.75, top - r):
        for y in (-cy, cy):
            m.cylinder((-cx, y, z), (cx, y, z), r, "galvanized_steel", sides=6, caps=False)
        for x in (-cx, cx):
            m.cylinder((x, -cy, z), (x, cy, z), r, "galvanized_steel", sides=6, caps=False)
    for x in (-0.3, 0.3):
        m.cylinder((x, -cy, top - r), (x, cy, top - r), r, "galvanized_steel", sides=6, caps=False)
    return m


def spoil_heap(piece, kit):
    tiers = sorted(piece["proxies"], key=lambda p: p["centre"][2])
    rng = random.Random(20261016)
    n = 28
    # (height, half-extent scale, z jitter): boundary rings take the extents of the tier above.
    bands = []
    for i, tier in enumerate(tiers):
        bottom = tier["centre"][2] - tier["size"][2] / 2
        top = tier["centre"][2] + tier["size"][2] / 2
        a, b = tier["size"][0] / 2, tier["size"][1] / 2
        if i == 0:
            bands.append((bottom, a, b, 0.0))
        nxt = tiers[i + 1]["size"] if i + 1 < len(tiers) else (0.25 * tier["size"][0], 0.2 * tier["size"][1])
        mid = (bottom + top) / 2
        bands.append((mid, (a + nxt[0] / 2) / 2, (b + nxt[1] / 2) / 2, 0.04))
        bands.append((top, nxt[0] / 2, nxt[1] / 2, 0.0))
    rings = []
    for index, (z, a, b, jitter) in enumerate(bands):
        ring = []
        for k in range(n):
            t = 2 * math.pi * k / n
            c, s = math.cos(t), math.sin(t)
            shrink = 1.0 - 0.08 * rng.random()
            if index == 0 and k % (n // 4) == 0:
                shrink = 1.0  # the footprint reaches the piece bounds on both axes
            x = a * shrink * math.copysign(abs(c) ** 0.7, c)
            y = b * shrink * math.copysign(abs(s) ** 0.7, s)
            ring.append((x, y, z + (rng.uniform(-jitter, jitter) if jitter else 0.0)))
        rings.append(ring)
    verts = [v for ring in rings for v in ring]
    faces = []
    for r in range(len(rings) - 1):
        for k in range(n):
            a, b = r * n + k, r * n + (k + 1) % n
            c, d = (r + 1) * n + (k + 1) % n, (r + 1) * n + k
            faces += [(a, b, c), (a, c, d)]
    m = Mesh()
    m.add(verts, faces, "compacted_ground", smooth=True, inside=(0.0, 0.0, 0.3))
    for cap in (rings[0], rings[-1]):
        m.add(cap, [tuple(range(n))], "compacted_ground", inside=(0.0, 0.0, 0.3))
    return m


BUILDERS = {"skip": skip, "ibc_tank": ibc_tank, "spoil_heap": spoil_heap}
