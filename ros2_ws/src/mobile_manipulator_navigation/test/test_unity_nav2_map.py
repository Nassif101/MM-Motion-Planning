import json
from pathlib import Path


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


def pixel_at_ros(pixels, width, height, ros_x, ros_y):
    cell_x = int((ros_x + 20.0) / 0.05)
    cell_y = int((ros_y + 20.0) / 0.05)
    image_row = height - 1 - cell_y
    return pixels[(image_row * width) + cell_x]


def test_exported_map_contract():
    pgm = MAP_ROOT / "construction_site.pgm"
    yaml = MAP_ROOT / "construction_site.yaml"
    metadata = MAP_ROOT / "construction_site.metadata.json"
    assert pgm.is_file()
    assert yaml.is_file()
    assert metadata.is_file()

    width, height, pixels = read_pgm(pgm)
    assert (width, height) == (800, 800)
    assert len(pixels) == width * height

    yaml_text = yaml.read_text(encoding="utf-8")
    assert "image: construction_site.pgm" in yaml_text
    assert "mode: trinary" in yaml_text
    assert "resolution: 0.050" in yaml_text
    assert "origin: [-20.000, -20.000, 0.000]" in yaml_text

    contract = json.loads(metadata.read_text(encoding="utf-8"))
    assert contract["mapFrame"] == "map"
    assert contract["coordinateMapping"] == (
        "ros_x=unity_z; ros_y=-unity_x; ros_z=unity_y"
    )
    assert contract["widthCells"] == width
    assert contract["heightCells"] == height
    assert contract["sourceColliderCount"] > 0
    assert contract["occupiedCellCount"] > 0

    # Unity start zone (x=10.5, z=15.0) -> ROS (x=15.0, y=-10.5).
    assert pixel_at_ros(pixels, width, height, 15.0, -10.5) == 254
    # Unity east-lane fence (x=9.1, z=11.1) -> ROS (x=11.1, y=-9.1).
    assert pixel_at_ros(pixels, width, height, 11.1, -9.1) == 0
