"""Barrier family: precast concrete block barriers and the plastic trench barrier.

The block barrier is boxy (not a tapered Jersey profile) so one box proxy fits: a chamfered
body on two feet with a drainage/forklift slot between them, lifting anchors on top and a
reflective strip on both faces. The trench barrier is a pedestrian barrier panel (top rail,
kick board, end posts) on two splayed feet.
"""
from common import Mesh

STRIP = 0.003


def barrier_block(piece, kit):
    lx, ly, h = piece["size"]
    hx, hy = lx / 2, ly / 2 - STRIP
    foot_h, anchor_h, foot_l = 0.1, 0.015, 0.7
    m = Mesh()
    m.chamfer_box((-hx, -hy, foot_h), (hx, hy, h - anchor_h), 0.03, "concrete")
    for side in (-1, 1):
        x0, x1 = sorted((side * hx, side * (hx - foot_l)))
        m.chamfer_box((x0, -hy, 0.0), (x1, hy, foot_h), 0.015, "concrete")
    for x in (-0.8, 0.8):
        m.box((x - 0.04, -0.04, h - anchor_h), (x + 0.04, 0.04, h), "galvanized_steel")
    for side in (-1, 1):
        y0, y1 = sorted((side * hy, side * (hy + STRIP)))
        m.box((-0.8, y0, 0.56), (0.8, y1, 0.64), "plastic")
    return m


def trench_barrier(piece, kit):
    lx, ly, h = piece["size"]
    hx = lx / 2
    foot_h, foot_l, post, board = 0.05, 0.1, 0.05, 0.04
    m = Mesh()
    for side in (-1, 1):
        x0, x1 = sorted((side * hx, side * (hx - foot_l)))
        m.prism([(-ly / 2, 0.0), (ly / 2, 0.0), (0.06, foot_h), (-0.06, foot_h)], "x", x0, x1, "plastic")
        px0, px1 = sorted((side * (hx - 0.025), side * (hx - 0.025 - post)))
        m.box((px0, -post / 2, foot_h), (px1, post / 2, h), "plastic")
    inner = hx - 0.025 - post
    m.box((-inner, -board / 2, h - 0.25), (inner, board / 2, h - 0.02), "plastic")
    m.box((-inner, -board / 2, 0.2), (inner, board / 2, 0.4), "plastic")
    for x in (-0.5, 0.0, 0.5):
        m.box((x - 0.015, -0.01, 0.4), (x + 0.015, 0.01, h - 0.25), "plastic")
    m.box((-hx, -post / 2, h - 0.02), (hx, post / 2, h), "plastic")
    return m


BUILDERS = {"barrier_block_0p8": barrier_block, "trench_barrier": trench_barrier}
