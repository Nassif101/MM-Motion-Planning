"""The MoveIt planning boxes agree with the Nav2 map exported from the same colliders."""
import json

import pytest
import yaml

from test_unity_nav2_map import MAP_ROOT, SITES, read_pgm, pixel_at_ros

SCENES = {
    "construction_site": "Assets/Scenes/ConstructionSiteV1.unity",
    "construction_site_v2": "Assets/Scenes/ConstructionSiteV2.unity",
}
# Overhead members sit above the 2D map band of V2, so they are boxes without map cells.
V2_MAP_BAND_TOP = round(json.loads(
    (MAP_ROOT / "construction_site_v2.metadata.json").read_text(encoding="utf-8"))["maximumUnityHeight"], 2)


def _load_boxes(site):
    return yaml.safe_load((MAP_ROOT / f"{site}.boxes.yaml").read_text(encoding="utf-8"))


def _bottom(box):
    return box["center"][2] - box["size"][2] / 2


@pytest.mark.parametrize("site", SITES)
def test_planning_boxes_lie_on_occupied_map_cells(site):
    boxes = _load_boxes(site)
    origin = SITES[site][:2]
    assert boxes["frame_id"] == "map"
    assert boxes["source_scene"] == SCENES[site]
    width, height, pixels = read_pgm(MAP_ROOT / f"{site}.pgm")
    assert boxes["boxes"]
    for box in boxes["boxes"]:
        x, y, z = box["center"]
        assert all(size > 0 for size in box["size"]), box["name"]
        assert z >= 0, box["name"]
        if site == "construction_site_v2" and _bottom(box) >= V2_MAP_BAND_TOP:
            continue  # overhead member: in the boxes, not in the 2D map
        assert pixel_at_ros(pixels, width, height, x, y, origin) == 0, box["name"]


def test_v2_disturbances_are_not_in_the_boxes():
    for box in _load_boxes("construction_site_v2")["boxes"]:
        assert "Disturbances" not in box["name"]


def test_v2_overhead_members_are_boxes_but_not_map_cells():
    boxes = _load_boxes("construction_site_v2")["boxes"]
    ledger = [b for b in boxes if "H1Ledger" in b["name"]]
    # The deck is the PipeBridge box that is both broad (4.1 m) and reaches down to 2.40 m.
    bridge = [b for b in boxes if "PipeBridge/" in b["name"] and _bottom(b) > 1.0]
    assert ledger and bridge
    assert any(abs(_bottom(b) - 1.42) < 0.011 for b in ledger)
    assert any(abs(_bottom(b) - 2.40) < 0.011 for b in bridge)
    width, height, pixels = read_pgm(MAP_ROOT / "construction_site_v2.pgm")
    origin = SITES["construction_site_v2"][:2]
    # H1 lane centre (x -6.5, y 7) and the pipe-bridge lane centre (x 16.3, y -14) stay free.
    assert pixel_at_ros(pixels, width, height, -6.5, 7.0, origin) == 254
    assert pixel_at_ros(pixels, width, height, 16.3, -14.0, origin) == 254
