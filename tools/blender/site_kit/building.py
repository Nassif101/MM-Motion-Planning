"""Building family: steel door frames set in a blockwork wall opening.

The wall pieces (wall_blockwork, column_rc, beam_rc, core_wall, slab_finish) are Unity
primitives and have no mesh. A door frame is the full wall height: two jambs whose inner faces
leave exactly the clear width, a frame head, and blockwork infill up to the wall top, so the
builder only leaves a gap of the frame's length in the wall.
"""
from common import Mesh

JAMB_M = 0.06
OPENING_HEIGHT_M = 2.10
HEAD_M = 0.06
WALL_THICKNESS_M = 0.20


def door_frame(piece, kit):
    length, depth, height = piece["size"]
    clear = length - 2 * JAMB_M
    m = Mesh()
    for side in (-1, 1):
        inner = side * clear / 2
        outer = side * (clear / 2 + JAMB_M)
        m.box((min(inner, outer), -depth / 2, 0.0), (max(inner, outer), depth / 2, OPENING_HEIGHT_M),
              "painted_steel")
    m.box((-length / 2, -depth / 2, OPENING_HEIGHT_M), (length / 2, depth / 2, OPENING_HEIGHT_M + HEAD_M),
          "painted_steel")
    m.box((-length / 2, -WALL_THICKNESS_M / 2, OPENING_HEIGHT_M + HEAD_M),
          (length / 2, WALL_THICKNESS_M / 2, height), "blockwork")
    return m


BUILDERS = {"door_frame_1p30": door_frame, "door_frame_1p05": door_frame}
