"""The MoveIt planning boxes agree with the Nav2 map exported from the same colliders."""
from pathlib import Path

import yaml

from test_unity_nav2_map import MAP_ROOT, pixel_at_ros, read_pgm


def test_planning_boxes_lie_on_occupied_map_cells():
    boxes = yaml.safe_load((MAP_ROOT / "construction_site.boxes.yaml").read_text(encoding="utf-8"))
    assert boxes["frame_id"] == "map"
    assert boxes["source_scene"] == "Assets/Scenes/ConstructionSiteV1.unity"
    width, height, pixels = read_pgm(MAP_ROOT / "construction_site.pgm")
    assert boxes["boxes"]
    for box in boxes["boxes"]:
        x, y, z = box["center"]
        assert all(size > 0 for size in box["size"]), box["name"]
        assert z >= 0, box["name"]
        assert pixel_at_ros(pixels, width, height, x, y) == 0, box["name"]
