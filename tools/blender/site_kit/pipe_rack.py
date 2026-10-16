"""Pipe rack family: T-supports, the pipe run they carry and the H3 pipe bridge.

T-support: base plate, an H-section post and an H-section arm (along x) whose top carries the
pipe run at 1.2 m. Pipe run: three pipes along x (0.2, 0.15 and 0.1 m), bottoms on z = 0,
with flanges at the ends and the middle; parametric along x. Pipe bridge: two H-section posts
on base plates carrying a truss deck whose bottom is the H3 height from the kit spec, with
the same three pipes on top; the clear span between the posts is 3.6 m.
"""
from common import Mesh

PIPES = ((-0.175, 0.1, "painted_steel"), (0.05, 0.075, "galvanized_steel"), (0.225, 0.05, "plastic"))
FLANGE = 0.03
PLATE_T = 0.015


def h_column(m, x0, x1, y0, y1, z0, z1, flange=0.012, web=0.008):
    """H-section post along z, flanges normal to y."""
    xc = (x0 + x1) / 2
    m.box((x0, y0, z0), (x1, y0 + flange, z1), "painted_steel")
    m.box((x0, y1 - flange, z0), (x1, y1, z1), "painted_steel")
    m.box((xc - web / 2, y0 + flange, z0), (xc + web / 2, y1 - flange, z1), "painted_steel")


def h_beam_x(m, x0, x1, y0, y1, z0, z1, flange=0.012, web=0.008):
    """H-section beam along x, flanges top and bottom."""
    yc = (y0 + y1) / 2
    m.box((x0, y0, z0), (x1, y1, z0 + flange), "painted_steel")
    m.box((x0, y0, z1 - flange), (x1, y1, z1), "painted_steel")
    m.box((x0, yc - web / 2, z0 + flange), (x1, yc + web / 2, z1 - flange), "painted_steel")


def pipes(m, x0, x1, z, flange_xs):
    for yc, r, material in PIPES:
        body = 0.92 * r
        m.cylinder((x0, yc, z + r), (x1, yc, z + r), body, material, sides=16 if r > 0.06 else 12)
        for xf in flange_xs:
            a, b = max(x0, xf - FLANGE / 2), min(x1, xf + FLANGE / 2)
            m.cylinder((a, yc, z + r), (b, yc, z + r), r, material, sides=16 if r > 0.06 else 12)


def support(piece, kit):
    lx, ly, h = piece["size"]
    post, arm_h = 0.15, 0.15
    m = Mesh()
    m.box((-ly / 2, -ly / 2, 0.0), (ly / 2, ly / 2, PLATE_T + 0.005), "galvanized_steel")
    h_column(m, -post / 2, post / 2, -post / 2, post / 2, PLATE_T + 0.005, h - arm_h)
    h_beam_x(m, -lx / 2, lx / 2, -post / 2, post / 2, h - arm_h, h)
    return m


def pipe_run(piece, kit):
    lx, _, _ = piece["size"]
    m = Mesh()
    pipes(m, -lx / 2, lx / 2, 0.0, (-lx / 2 + FLANGE / 2, 0.0, lx / 2 - FLANGE / 2))
    return m


def bridge(piece, kit):
    lx, ly, h = piece["size"]
    deck_bottom = kit["overhead"]["h3_bottom_m"]
    deck_top = deck_bottom + 0.2
    hx, post, plate = lx / 2, 0.2, 0.3
    m = Mesh()
    for side in (-1, 1):
        xc = side * (hx - plate / 2)
        m.box((xc - plate / 2, -plate / 2, 0.0), (xc + plate / 2, plate / 2, PLATE_T), "galvanized_steel")
        h_column(m, xc - post / 2, xc + post / 2, -post / 2, post / 2, PLATE_T, deck_bottom, flange=0.015, web=0.01)
    chord = 0.05
    for side in (-1, 1):
        y0, y1 = sorted((side * ly / 2, side * (ly / 2 - chord)))
        m.box((-hx, y0, deck_bottom), (hx, y1, deck_bottom + chord), "painted_steel")
        m.box((-hx, y0, deck_top - chord), (hx, y1, deck_top), "painted_steel")
        panels = 8
        for k in range(panels + 1):
            x = -hx + 0.03 + (lx - 0.06) * k / panels
            m.box((x - 0.02, y0 + 0.01, deck_bottom + chord), (x + 0.02, y1 - 0.01, deck_top - chord), "painted_steel")
            if k < panels:
                xn = -hx + 0.03 + (lx - 0.06) * (k + 1) / panels
                a, b = (x, xn) if k % 2 == 0 else (xn, x)
                lo_z, hi_z = deck_bottom + chord, deck_top - chord
                m.hexa([(x_, y, z) for z, (p, q) in ((lo_z, (a - 0.015, a + 0.015)), (hi_z, (b - 0.015, b + 0.015)))
                        for y in (y0 + 0.015, y1 - 0.015) for x_ in (p, q)], "painted_steel")
    for k in range(9):
        x = -hx + 0.03 + (lx - 0.06) * k / 8
        m.box((x - 0.03, -ly / 2 + chord, deck_top - 0.04), (x + 0.03, ly / 2 - chord, deck_top), "painted_steel")
    pipes(m, -hx, hx, deck_top, (-hx + FLANGE / 2, hx - FLANGE / 2))
    return m


BUILDERS = {"pipe_rack_support": support, "pipe_run": pipe_run, "pipe_bridge": bridge}
