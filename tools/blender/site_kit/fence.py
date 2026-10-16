"""Fence family: mesh fence panels on concrete feet and painted plywood hoarding.

Fence panel: a 3.5 x 2.0 m welded-mesh panel (40 mm frame tube) whose end posts stand in two
concrete feet inside the panel length, so panels placed end to end at 3.5 m pitch stand on
paired feet without overlapping. The panel frame starts on the feet tops.

Hoarding: two portrait plywood sheets (1.2 x 2.4 m, painted in the cabin colour) on three
timber rails and two posts; the painted face is the -y side.
"""
from common import Mesh

FOOT_LENGTH = 0.65
FOOT_HEIGHT = 0.13
FRAME_D = 0.04
WIRE = 0.005
CLAMP = 0.05


def fence_panel(piece, kit):
    length, depth, height = piece["size"]
    half = length / 2
    r = FRAME_D / 2
    m = Mesh()
    for side in (-1, 1):
        x0, x1 = sorted((side * (half - FOOT_LENGTH), side * half))
        m.chamfer_box((x0, -depth / 2, 0.0), (x1, depth / 2, FOOT_HEIGHT), 0.02, "concrete")
    post_x = half - r
    for side in (-1, 1):
        m.cylinder((side * post_x, 0, FOOT_HEIGHT), (side * post_x, 0, height), r, "galvanized_steel", sides=8)
        for z in (0.55, 1.75):  # couplers clamping the neighbouring panel
            x0, x1 = sorted((side * (half - CLAMP), side * half))
            m.box((x0, -CLAMP / 2, z), (x1, CLAMP / 2, z + 0.07), "galvanized_steel")
    for z in (FOOT_HEIGHT + r, height - r):
        m.cylinder((-post_x + r, 0, z), (post_x - r, 0, z), r, "galvanized_steel", sides=8)
    inner = post_x - r
    bottom, top = FOOT_HEIGHT + FRAME_D, height - FRAME_D
    count = 34
    for k in range(count + 1):
        x = -inner + 2 * inner * k / count
        if abs(x) > inner - 0.03:
            continue
        m.box((x - WIRE / 2, -WIRE / 2, bottom), (x + WIRE / 2, WIRE / 2, top), "galvanized_steel")
    rows = 7
    for k in range(1, rows):
        z = bottom + (top - bottom) * k / rows
        m.box((-inner, -WIRE / 2 - WIRE, z - WIRE / 2), (inner, -WIRE / 2, z + WIRE / 2), "galvanized_steel")
    return m


def hoarding_panel(piece, kit):
    length, depth, height = piece["size"]
    half = length / 2
    y_face, ply, rail = -depth / 2, 0.018, 0.05
    m = Mesh()
    joint = 0.0015
    for x0, x1 in ((-half, -joint), (joint, half)):
        m.box((x0, y_face, 0.03), (x1, y_face + ply, height), "cabin_paint")
    for z in (0.25, 1.2, 2.15):
        m.box((-half, y_face + ply, z - 0.05), (half, y_face + ply + rail, z + 0.05), "timber")
    for side in (-1, 1):
        x0, x1 = sorted((side * (half - 0.1), side * half))
        m.box((x0, y_face + ply + rail, 0.0), (x1, depth / 2, height), "timber")
    return m


BUILDERS = {"fence_panel_3p5": fence_panel, "hoarding_panel_2p4": hoarding_panel}
