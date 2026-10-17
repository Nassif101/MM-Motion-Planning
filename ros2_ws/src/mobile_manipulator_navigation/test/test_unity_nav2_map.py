import json
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MAP_ROOT = PACKAGE_ROOT / "maps"


def read_pgm(path: Path):
    with path.open("rb") as stream:
        assert stream.readline().strip() == b"P5"
        line = stream.readline()
        while line.startswith(b"#"):
            line = stream.readline()
        width, height = (int(value) for value in line.split())
        assert int(stream.readline()) == 255
        pixels = stream.read()
    return width, height, pixels


# site -> (origin_x, origin_y, width_cells, height_cells)
SITES = {
    "construction_site": (-20.0, -20.0, 800, 800),
    "construction_site_v2": (-25.0, -20.0, 1000, 800),
}


def pixel_at_ros(pixels, width, height, ros_x, ros_y, origin=(-20.0, -20.0)):
    cell_x = int((ros_x - origin[0]) / 0.05)
    cell_y = int((ros_y - origin[1]) / 0.05)
    image_row = height - 1 - cell_y
    return pixels[(image_row * width) + cell_x]


@pytest.mark.parametrize("site", SITES)
def test_exported_map_contract(site):
    origin_x, origin_y, expected_width, expected_height = SITES[site]
    pgm = MAP_ROOT / f"{site}.pgm"
    yaml = MAP_ROOT / f"{site}.yaml"
    metadata = MAP_ROOT / f"{site}.metadata.json"
    assert pgm.is_file()
    assert yaml.is_file()
    assert metadata.is_file()

    width, height, pixels = read_pgm(pgm)
    assert (width, height) == (expected_width, expected_height)
    assert len(pixels) == width * height

    yaml_text = yaml.read_text(encoding="utf-8")
    assert f"image: {site}.pgm" in yaml_text
    assert "mode: trinary" in yaml_text
    assert "resolution: 0.050" in yaml_text
    assert f"origin: [{origin_x:.3f}, {origin_y:.3f}, 0.000]" in yaml_text

    contract = json.loads(metadata.read_text(encoding="utf-8"))
    assert contract["mapFrame"] == "map"
    assert contract["coordinateMapping"] == (
        "ros_x=unity_z; ros_y=-unity_x; ros_z=unity_y"
    )
    assert contract["widthCells"] == width
    assert contract["heightCells"] == height
    assert contract["originRosX"] == origin_x
    assert contract["originRosY"] == origin_y
    assert contract["sourceColliderCount"] > 0
    assert contract["occupiedCellCount"] > 0

    if site == "construction_site":
        # Unity start zone (x=10.5, z=15.0) -> ROS (x=15.0, y=-10.5).
        assert pixel_at_ros(pixels, width, height, 15.0, -10.5) == 254
        # Unity east-lane fence (x=9.1, z=11.1) -> ROS (x=11.1, y=-9.1).
        assert pixel_at_ros(pixels, width, height, 11.1, -9.1) == 0


def test_wide_comparison_gate_is_in_the_map():
    width, height, pixels = read_pgm(MAP_ROOT / "construction_site.pgm")
    # WideGate_1p30m posts: Unity x 15.15 / 16.85, z -3.0 -> ROS (x=-3.0, y=-15.15 / -16.85).
    assert pixel_at_ros(pixels, width, height, -3.0, -15.15) == 0
    assert pixel_at_ros(pixels, width, height, -3.0, -16.85) == 0
    # The 1.30 m opening between the post faces (ROS y -16.65 .. -15.35) is free.
    for y in (-16.55, -16.0, -15.45):
        assert pixel_at_ros(pixels, width, height, -3.0, y) == 254


V2_ORIGIN = (-25.0, -20.0)
V2_SOUTH_WALL_Y = 5.6  # building min y 5.5 + half of the 0.2 m wall


def _v2_pixel(x, y):
    width, height, pixels = read_pgm(MAP_ROOT / "construction_site_v2.pgm")
    return pixel_at_ros(pixels, width, height, x, y, V2_ORIGIN)


def test_v2_door_cells_are_free():
    for centre_x, clear in ((5.25, 1.30), (16.25, 1.05)):
        assert _v2_pixel(centre_x, V2_SOUTH_WALL_Y) == 254, f"door {clear}"
        for side in (-1, 1):
            jamb_x = centre_x + side * (clear / 2 + 0.2)
            assert _v2_pixel(jamb_x, V2_SOUTH_WALL_Y) == 0, f"jamb {clear} side {side}"


V2_APPROACH_LANE_WIDTH = 1.6  # SiteV2Layout.ApproachLaneWidth


def test_v2_door_approach_lanes_are_walled_and_free():
    # Block barriers (0.55 m wide) line both sides of a 1.6 m lane on each south door's axis from the
    # wall face (y 5.5) to y 3.0; the lane itself is free from the spine edge (y 2.5) to the door.
    for centre_x in (5.25, 16.25):
        for y in (2.6, 3.1, 4.25, 5.4):
            assert _v2_pixel(centre_x, y) == 254, f"lane {centre_x} y {y}"
            for side in (-1, 1):
                inside = centre_x + side * (V2_APPROACH_LANE_WIDTH / 2 - 0.1)
                assert _v2_pixel(inside, y) == 254, f"lane {centre_x} side {side} y {y}"
        for y in (3.1, 4.25, 5.4):
            for side in (-1, 1):
                barrier = centre_x + side * (V2_APPROACH_LANE_WIDTH / 2 + 0.275)
                assert _v2_pixel(barrier, y) == 0, f"barrier {centre_x} side {side} y {y}"


# Building fit-out doors (spec Section 4, "Interior layouts"): name, centre on the partition line,
# clear width, and the direction of the partition ("x": a wall along x, "y": along y).
V2_INTERIOR_DOORS = (
    ("R1", 4.15, 8.65, 1.30, "y"),
    ("R2", 10.05, 11.55, 1.30, "x"),
    ("S", 15.10, 9.025, 1.05, "y"),
    ("R3", 1.65, 13.75, 1.30, "x"),
    ("R4", 7.825, 13.75, 1.05, "x"),
    ("R5", 12.65, 13.75, 1.30, "x"),
)


@pytest.mark.parametrize("name, centre_x, centre_y, clear, along", V2_INTERIOR_DOORS)
def test_v2_interior_doors_are_free(name, centre_x, centre_y, clear, along):
    assert _v2_pixel(centre_x, centre_y) == 254, f"door {name}"
    for side in (-1, 1):
        offset = side * (clear / 2 + 0.2)
        x, y = (centre_x + offset, centre_y) if along == "x" else (centre_x, centre_y + offset)
        assert _v2_pixel(x, y) == 0, f"jamb {name} side {side}"
