"""Scaffold family: a modular (rosette) system scaffold on 2.5 x 1.2 m bays.

Standards are 48.3 mm tubes on base plates with rosettes every 0.5 m (at 0.444 + 0.5 k, so a
ledger hung on the 1.444 m rosette has its tube bottom at the H1 height 1.42 m). Ledgers and
transoms end in wedge heads that stop at the rosette rim (centre-to-centre spacing minus the
0.124 m rosette), so placed pieces touch without overlapping. Ledger and transom tubes sit with
their bottom at z = 0 in the piece frame. Boards span standard centre to standard centre and
rest on the transoms; toe boards stand on the boards inside the standards.
"""
from common import Mesh

TUBE_D = 0.0483
ROSETTE_D = 0.124
ROSETTE_T = 0.009
ROSETTE_HEIGHTS = (0.444, 0.944, 1.444, 1.944)
BASE_PLATE_T = 0.008
HEAD_LENGTH = 0.06
WEDGE_TOP = 0.08


def standard(piece, kit):
    plate, _, height = piece["size"]
    m = Mesh()
    m.box((-plate / 2, -plate / 2, 0.0), (plate / 2, plate / 2, BASE_PLATE_T), "galvanized_steel")
    m.cylinder((0, 0, BASE_PLATE_T), (0, 0, 0.11), 0.019, "galvanized_steel", sides=8)  # base jack spindle
    m.cylinder((0, 0, 0.07), (0, 0, 0.10), 0.045, "painted_steel", sides=6)  # jack nut
    m.cylinder((0, 0, 0.11), (0, 0, height), TUBE_D / 2, "galvanized_steel", sides=10)
    for z in ROSETTE_HEIGHTS:
        m.cylinder((0, 0, z - ROSETTE_T / 2), (0, 0, z + ROSETTE_T / 2), ROSETTE_D / 2, "galvanized_steel",
                   sides=16)
    return m


def _horizontal(piece):
    """Tube between two wedge heads, tube bottom at z = 0 (ledger and transom)."""
    length, width, _ = piece["size"]
    half = length / 2
    m = Mesh()
    m.cylinder((-half + HEAD_LENGTH, 0, TUBE_D / 2), (half - HEAD_LENGTH, 0, TUBE_D / 2), TUBE_D / 2,
               "galvanized_steel", sides=10)
    for side in (-1, 1):
        inner, outer = side * (half - HEAD_LENGTH), side * half
        x0, x1 = min(inner, outer), max(inner, outer)
        m.box((x0, -width / 2, 0.0), (x1, width / 2, TUBE_D), "painted_steel")
        # The wedge driven through the head, leaning towards the tube.
        w0, w1 = side * (half - 0.018), side * (half - 0.032)
        m.hexa([(x, y, z) for z, xs in ((TUBE_D, (w1, w0)), (WEDGE_TOP, (w1 - side * 0.006, w0 - side * 0.006)))
                for y in (-0.01, 0.01) for x in sorted(xs)], "galvanized_steel")
    return m


def ledger(piece, kit):
    return _horizontal(piece)


def transom(piece, kit):
    return _horizontal(piece)


def board(piece, kit):
    length, width, thickness = piece["size"]
    cap = 0.04
    gap = 0.006
    plank = (width - 2 * gap) / 3
    m = Mesh()
    for k in range(3):
        y0 = -width / 2 + k * (plank + gap)
        m.box((-length / 2 + cap, y0, 0.0), (length / 2 - cap, y0 + plank, thickness), "timber")
    for side in (-1, 1):
        x0, x1 = sorted((side * (length / 2 - cap), side * length / 2))
        m.box((x0, -width / 2, 0.0), (x1, width / 2, thickness), "galvanized_steel")
    return m


def toe_board(piece, kit):
    length, thickness, height = piece["size"]
    m = Mesh()
    m.chamfer_box((-length / 2, -thickness / 2, 0.0), (length / 2, thickness / 2, height), 0.004, "timber")
    return m


BUILDERS = {
    "scaffold_standard": standard,
    "scaffold_ledger_2p5": ledger,
    "scaffold_transom_1p2": transom,
    "scaffold_board_2p5": board,
    "scaffold_toe_board_2p5": toe_board,
}
