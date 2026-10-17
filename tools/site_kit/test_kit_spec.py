"""Kit spec of the second construction site (spec Sections 2-5).

Run: python3 -m pytest tools/site_kit -q
"""
import json
import math
from pathlib import Path

import pytest

KIT = Path(__file__).resolve().parent / "kit_v2.json"
SITE_KIT = Path(__file__).resolve().parents[2] / "motion-planning-sim/Assets/SiteKit"

PIECE_IDS = (
    "wall_blockwork", "column_rc", "beam_rc", "core_wall", "slab_finish",
    "scaffold_standard", "scaffold_ledger_2p5", "scaffold_transom_1p2", "scaffold_board_2p5",
    "scaffold_toe_board_2p5", "fence_panel_3p5", "hoarding_panel_2p4", "cabin_office",
    "cabin_welfare", "cabin_drying", "pallet_euro", "stack_block_tall", "stack_pallet_low_0p8",
    "stack_panel", "bearers_timber", "bearers_rebar", "barrier_block_0p8", "pipe_rack_support",
    "pipe_run", "pipe_bridge", "skip", "ibc_tank", "spoil_heap", "trench_barrier",
    "cable_protector_0p035", "road_plate_0p025", "rumble_strip_0p02", "door_frame_1p30",
    "door_frame_1p05",
)
PRIMITIVES = {"wall_blockwork", "column_rc", "beam_rc", "core_wall", "slab_finish"}
TIER2_PREFIXES = ("cable_protector", "road_plate", "rumble_strip")
DOOR_CLEAR_WIDTHS = {"door_frame_1p30": 1.30, "door_frame_1p05": 1.05}
MATERIALS = {
    "asphalt", "compacted_ground", "gravel", "concrete", "blockwork", "timber",
    "galvanized_steel", "painted_steel", "plastic", "cabin_paint", "lane_marking",
    "walkway_marking",
}
PROXY_SLACK_M = 0.02
LIDAR_BAND_M = 0.035
TOUCH_M = 0.005
EPS = 1e-6


@pytest.fixture(scope="module")
def kit():
    return json.loads(KIT.read_text())


@pytest.fixture(scope="module")
def pieces(kit):
    return kit["pieces"]


def box_bounds(proxy):
    """Axis-aligned (lo, hi) of a proxy in the piece frame, yaw included."""
    (cx, cy, cz), (sx, sy, sz), yaw = proxy["centre"], proxy["size"], proxy["yaw"]
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    hx, hy = (c * sx + s * sy) / 2, (s * sx + c * sy) / 2
    return (cx - hx, cy - hy, cz - sz / 2), (cx + hx, cy + hy, cz + sz / 2)


def union_bounds(proxies):
    boxes = [box_bounds(p) for p in proxies]
    lo = tuple(min(b[0][a] for b in boxes) for a in range(3))
    hi = tuple(max(b[1][a] for b in boxes) for a in range(3))
    return lo, hi


def piece_bounds(piece):
    sx, sy, sz = piece["size"]
    return (-sx / 2, -sy / 2, 0.0), (sx / 2, sy / 2, sz)


def test_ids_are_unique(pieces):
    ids = [p["id"] for p in pieces]
    assert len(ids) == len(set(ids))


def test_all_pieces_present(pieces):
    assert [p["id"] for p in pieces] == list(PIECE_IDS)


def test_piece_fields(pieces):
    for piece in pieces:
        assert len(piece["size"]) == 3 and min(piece["size"]) > 0, piece["id"]
        assert isinstance(piece["parametric_x"], bool) and isinstance(piece["round"], bool), piece["id"]
        assert piece["material"] in MATERIALS, piece["id"]
        assert piece["physics_material"] == "", piece["id"]
        for proxy in piece["proxies"]:
            assert len(proxy["centre"]) == 3 and len(proxy["size"]) == 3, piece["id"]
            assert min(proxy["size"]) > 0, piece["id"]


def test_meshes_exist_except_primitives(pieces):
    for piece in pieces:
        if piece["id"] in PRIMITIVES:
            assert piece["mesh"] == "", piece["id"]
        else:
            assert piece["mesh"] == f"Meshes/{piece['id']}.fbx"
            assert (SITE_KIT / piece["mesh"]).is_file(), piece["mesh"]


def test_only_slab_finish_has_no_proxies(pieces):
    assert [p["id"] for p in pieces if not p["proxies"]] == ["slab_finish"]


def test_proxies_enclose_the_mesh(pieces):
    for piece in pieces:
        if not piece["proxies"]:
            continue
        (ulo, uhi), (mlo, mhi) = union_bounds(piece["proxies"]), piece_bounds(piece)
        for axis in range(3):
            assert ulo[axis] <= mlo[axis] + EPS, (piece["id"], axis, "low side smaller than the mesh")
            assert uhi[axis] >= mhi[axis] - EPS, (piece["id"], axis, "high side smaller than the mesh")
            if not piece["round"]:
                assert mlo[axis] - ulo[axis] <= PROXY_SLACK_M + EPS, (piece["id"], axis, "low side")
                assert uhi[axis] - mhi[axis] <= PROXY_SLACK_M + EPS, (piece["id"], axis, "high side")


def test_proxies_of_a_piece_do_not_intersect(pieces):
    """The site validator reports boxes that overlap deeper than 5 mm; a piece must not cause that."""
    for piece in pieces:
        boxes = [box_bounds(p) for p in piece["proxies"]]
        for i, (alo, ahi) in enumerate(boxes):
            for blo, bhi in boxes[i + 1:]:
                depth = min(min(ahi[a], bhi[a]) - max(alo[a], blo[a]) for a in range(3))
                assert depth <= TOUCH_M + EPS, (piece["id"], depth)


def test_tier2_pieces_are_below_the_lidar_band(pieces):
    tier2 = [p for p in pieces if p["id"].startswith(TIER2_PREFIXES)]
    assert len(tier2) == 3
    for piece in tier2:
        assert piece["size"][2] <= LIDAR_BAND_M + EPS, piece["id"]
        assert union_bounds(piece["proxies"])[1][2] <= LIDAR_BAND_M + EPS, piece["id"]


def test_tier2_pieces_are_chamfered(pieces):
    """Stepped proxies follow the chamfered edges: each step is narrower than the one below."""
    for piece in (p for p in pieces if p["id"].startswith(TIER2_PREFIXES)):
        steps = sorted(piece["proxies"], key=lambda p: p["centre"][2])
        assert len(steps) >= 2, piece["id"]
        for lower, upper in zip(steps, steps[1:]):
            assert upper["size"][1] < lower["size"][1], piece["id"]


def test_door_frames_leave_their_clear_width(pieces):
    by_id = {p["id"]: p for p in pieces}
    for door_id, clear in DOOR_CLEAR_WIDTHS.items():
        jambs = [box_bounds(p) for p in by_id[door_id]["proxies"] if box_bounds(p)[0][2] < 1.0]
        left = max(hi[0] for lo, hi in jambs if hi[0] <= 0)
        right = min(lo[0] for lo, hi in jambs if lo[0] >= 0)
        assert right - left == pytest.approx(clear, abs=1e-6), door_id


def test_pipe_bridge_bottom_is_h3(kit, pieces):
    bridge = next(p for p in pieces if p["id"] == "pipe_bridge")
    over_centre = [box_bounds(p) for p in bridge["proxies"] if box_bounds(p)[0][0] < 0 < box_bounds(p)[1][0]]
    assert min(lo[2] for lo, hi in over_centre) == pytest.approx(kit["overhead"]["h3_bottom_m"], abs=1e-6)


def test_overhead_present(kit):
    overhead = kit["overhead"]
    assert overhead["h1_bottom_m"] > 0 and overhead["h3_bottom_m"] > overhead["h1_bottom_m"]
    assert {t["name"] for t in overhead["tops_m"]} >= {"home", "vertical_carry", "carry"}
