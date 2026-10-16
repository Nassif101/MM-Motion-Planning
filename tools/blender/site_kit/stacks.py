"""Stacks family: Euro pallets and the material stacks of the laydown yard.

Euro pallet (EPAL, 1.2 x 0.8 x 0.144 m): three bottom boards, nine blocks, three stringer
boards and five top boards. Block stacks are concrete blocks (0.4 x 0.2 x 0.2 m) in banded
packs on pallets, two packs high; the low stack is cement bags on a pallet; the panel stack is
plywood sheet bundles on timber bearers; timber and rebar lie on three bearers.
"""
from common import Mesh

PALLET_H = 0.144
BLOCK = (0.4, 0.2, 0.2)
BAND = 0.003


def euro_pallet(m, z=0.0):
    """EPAL pallet with its bottom at z, 1.2 m along x and 0.8 m along y."""
    lx, ly = 1.2, 0.8
    t, block_h = 0.022, 0.078
    rows = ((-ly / 2, -ly / 2 + 0.1), (-0.0725, 0.0725), (ly / 2 - 0.1, ly / 2))
    cols = ((-lx / 2, -lx / 2 + 0.145), (-0.0725, 0.0725), (lx / 2 - 0.145, lx / 2))
    for y0, y1 in rows:
        m.box((-lx / 2, y0, z), (lx / 2, y1, z + t), "timber")
        for x0, x1 in cols:
            m.box((x0, y0, z + t), (x1, y1, z + t + block_h), "timber")
    for x0, x1 in cols:
        m.box((x0, -ly / 2, z + t + block_h), (x1, ly / 2, z + 2 * t + block_h), "timber")
    widths = (0.145, 0.1, 0.145, 0.1, 0.145)
    gap = (ly - sum(widths)) / (len(widths) - 1)
    y = -ly / 2
    for width in widths:
        m.box((-lx / 2, y, z + 2 * t + block_h), (lx / 2, y + width, z + PALLET_H), "timber")
        y += width + gap


def pallet(piece, kit):
    m = Mesh()
    euro_pallet(m)
    return m


def _block_pack(m, z, layers):
    """Concrete blocks 3 x 4 per layer on a 1.2 x 0.8 pallet, with two plastic bands."""
    bx, by, bz = BLOCK
    gap = 0.01
    inner_y = 0.4 - BAND
    for layer in range(layers):
        for ix in range(3):
            for iy in range(4):
                hidden = ix == 1 and iy in (1, 2) and layer < layers - 1
                if hidden:
                    continue
                x0 = -0.6 + ix * bx
                y0 = -inner_y + iy * (2 * inner_y / 4)
                m.box((x0 + gap / 2, y0 + gap / 2, z + layer * bz), (x0 + bx - gap / 2, y0 + 2 * inner_y / 4 - gap / 2,
                                                                     z + (layer + 1) * bz - gap), "blockwork")
    top = z + layers * bz
    for x in (-0.3, 0.3):
        for side in (-1, 1):
            y0, y1 = sorted((side * inner_y, side * 0.4))
            m.box((x - 0.008, y0, z), (x + 0.008, y1, top), "plastic")
        m.box((x - 0.008, -0.4, top - gap), (x + 0.008, 0.4, top), "plastic")
    return top


def block_stack_tall(piece, kit):
    m = Mesh()
    z = 0.0
    for _ in range(2):
        euro_pallet(m, z)
        z = _block_pack(m, z + PALLET_H, 4)
    return m


def pallet_stack_low(piece, kit):
    _, _, height = piece["size"]
    m = Mesh()
    euro_pallet(m)
    layers = 5
    bag_h = (height - PALLET_H) / layers
    for layer in range(layers):
        z = PALLET_H + layer * bag_h
        for x0 in (-0.6, 0.0):
            for y0 in (-0.4, 0.0):
                m.chamfer_box((x0 + 0.003, y0 + 0.003, z), (x0 + 0.597, y0 + 0.397, z + bag_h), 0.03, "plastic")
    return m


def panel_stack(piece, kit):
    lx, ly, height = piece["size"]
    m = Mesh()
    bearer = 0.1
    for x in (-1.0, 0.0, 1.0):
        m.box((x - 0.05, -ly / 2, 0.0), (x + 0.05, ly / 2, bearer), "timber")
    batten = 0.02
    bundle = (height - bearer - batten) / 2
    sheets = 4
    z = bearer
    for b in range(2):
        for k in range(sheets):
            z0 = z + k * bundle / sheets
            joint = 0.003 if k < sheets - 1 else 0.0
            m.box((-lx / 2, -ly / 2, z0), (lx / 2, ly / 2, z0 + bundle / sheets - joint), "timber")
        z += bundle
        if b == 0:
            for x in (-1.0, 0.0, 1.0):
                m.box((x - 0.025, -ly / 2, z), (x + 0.025, ly / 2, z + batten), "timber")
            z += batten
    return m


def _bearers(m, xs, width):
    for x in xs:
        m.box((x - 0.05, -width / 2, 0.0), (x + 0.05, width / 2, 0.1), "timber")


def bearers_timber(piece, kit):
    lx, ly, height = piece["size"]
    m = Mesh()
    _bearers(m, (-1.5, 0.0, 1.5), ly)
    joist_w, joist_t, gap = 0.15, 0.05, 0.02
    layers = int(round((height - 0.1) / joist_t))
    for layer in range(layers):
        count = 6 if layer < layers - 1 else 4
        span = count * joist_w + (count - 1) * gap
        z0 = 0.1 + layer * joist_t
        for k in range(count):
            y0 = -span / 2 + k * (joist_w + gap)
            joint = 0.002 if layer < layers - 1 else 0.0
            m.box((-lx / 2, y0, z0), (lx / 2, y0 + joist_w, z0 + joist_t - joint), "timber")
    return m


def bearers_rebar(piece, kit):
    import math

    lx, ly, height = piece["size"]
    m = Mesh()
    _bearers(m, (-2.5, 0.0, 2.5), ly)
    bar_r, ring = 0.0125, 0.025
    zc = height - ring - bar_r
    for yc in (-0.48, -0.36, -0.24, -0.12, 0.0, 0.12, 0.24, 0.36, 0.48):
        for k in range(7):
            if k == 6:
                dy = dz = 0.0
            else:
                angle = math.radians(30 + 60 * k)
                dy, dz = ring * math.cos(angle), ring * math.sin(angle)
            m.cylinder((-lx / 2, yc + dy, zc + dz), (lx / 2, yc + dy, zc + dz), bar_r, "painted_steel", sides=6)
    return m


BUILDERS = {
    "pallet_euro": pallet,
    "stack_block_tall": block_stack_tall,
    "stack_pallet_low_0p8": pallet_stack_low,
    "stack_panel": panel_stack,
    "bearers_timber": bearers_timber,
    "bearers_rebar": bearers_rebar,
}
