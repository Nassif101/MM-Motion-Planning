"""Tests of the MuJoCo site model generator (MuJoCo plant plan Task 2; spec Sections 3.1 and 8 G1, G2, G5)."""
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

import generate_site_model as gsm

ROOT = Path(__file__).resolve().parents[2]
MAPS = ROOT / "ros2_ws/src/mobile_manipulator_navigation/maps"
BOXES = MAPS / "construction_site_v2.boxes.yaml"
METADATA = MAPS / "construction_site_v2.metadata.json"
PGM = MAPS / "construction_site_v2.pgm"
COMMITTED = ROOT / "ros2_ws/src/mobile_manipulator_plant/models/site_v2.xml"


@pytest.fixture(scope="module")
def boxes():
    return yaml.safe_load(BOXES.read_text())["boxes"]


@pytest.fixture(scope="module")
def model():
    return ET.fromstring(gsm.generate_site(BOXES, METADATA))


def site_geoms(model):
    return {g.get("name"): g for g in model.iter("geom") if g.get("name", "").startswith("site/")}


def floats(text):
    return [float(v) for v in text.split()]


def test_every_box_becomes_a_geom(boxes, model):
    geoms = site_geoms(model)
    assert len(boxes) == 604
    assert len(geoms) == 604
    for box in boxes:
        g = geoms["site/" + box["name"]]
        assert g.get("type", "box") == "box"
        np.testing.assert_allclose(floats(g.get("pos")), box["center"], atol=1e-6)
        np.testing.assert_allclose(floats(g.get("size")), [s / 2 for s in box["size"]], atol=1e-6)
        yaw = box["yaw"]
        np.testing.assert_allclose(floats(g.get("quat")), [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)], atol=1e-6)


def test_geoms_collide_only_with_the_robot(model):
    for g in site_geoms(model).values():
        assert g.get("class") == "site"
    site_default = next(d for d in model.iter("default") if d.get("class") == "site").find("geom")
    assert (site_default.get("contype"), site_default.get("conaffinity"), site_default.get("group")) == ("1", "2", "0")
    ground = next(g for g in model.iter("geom") if g.get("name") == "ground")
    assert ground.get("type") == "plane"


def test_checksum_and_generator_embedded(model):
    texts = {t.get("name"): t.get("data") for t in model.iter("text")}
    assert texts["site_checksum"] == "sha256:" + hashlib.sha256(BOXES.read_bytes()).hexdigest()
    assert texts["map_pgm_sha256"] == json.loads(METADATA.read_text())["pgmSha256"]
    assert texts["generator"] == "generate_site_model 1"


def test_committed_model_is_current():
    assert COMMITTED.read_text() == gsm.generate_site(BOXES, METADATA)


def read_pgm(path):
    data = path.read_bytes()
    fields, pos = [], 0
    while len(fields) < 4:  # magic, width, height, maxval; comments start with '#'
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            pos = data.index(b"\n", pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        fields.append(data[pos:end].decode())
        pos = end
    width, height = int(fields[1]), int(fields[2])
    pixels = np.frombuffer(data[pos + 1:pos + 1 + width * height], dtype=np.uint8).reshape(height, width)
    return pixels[::-1]  # row 0 = the map's lowest y (the PGM's first row is its highest)


def test_rasterization_matches_exported_map(boxes):
    """G2: the boxes rasterized by Nav2MapExporter's rule (each collider's axis-aligned bounds overlapping Unity height
    (0.02, 0.6) m, cells by Nav2MapGeometry.TryGetOverlappingCells with its 1e-4 cell tolerance) equal the exported
    PGM cell by cell; any mismatch lies on a box edge (within 1e-4 cell of a cell boundary) and is printed."""
    meta = json.loads(METADATA.read_text())
    occupied = gsm.rasterize(boxes, meta)
    pgm = read_pgm(PGM)
    exported = pgm < 128  # occupied cells are dark
    assert occupied.shape == exported.shape == (meta["heightCells"], meta["widthCells"])
    assert int(exported.sum()) == meta["occupiedCellCount"]
    mismatches = np.argwhere(occupied != exported)
    on_edges = gsm.cells_on_box_edges(boxes, meta, tolerance_cells=1e-4)
    off_edge = [tuple(c) for c in mismatches if tuple(c) not in on_edges]
    print(f"{len(mismatches)} mismatched cells, {len(off_edge)} off a box edge: {off_edge[:20]}")
    assert off_edge == []
    assert len(mismatches) <= 50  # rounding cases only; thousands would mean a systematic difference


def test_unity_yaw_undoes_the_export_rounding():
    """The export writes yaw with 4 decimals (90 degrees as 1.5708): the map was rasterized at the true angle."""
    assert gsm.unity_yaw(1.5708) == math.pi / 2
    assert gsm.unity_yaw(-1.5708) == -math.pi / 2
    assert gsm.unity_yaw(0.0) == 0.0
    assert gsm.unity_yaw(0.7854) == 0.7854  # not a right angle: unchanged


# Site fixtures (G5): (frame name, opening between the jambs m, lintel underside m). The scenario fixtures (gate posts,
# beams) are added at reset and checked by the plant's builder test (Task 4).
DOORS = [("DoorFrame105", 1.05, 2.10), ("DoorFrame130", 1.30, 2.10), ("DoorR4Frame", 1.05, 2.10)]


@pytest.mark.parametrize("frame, opening, underside", DOORS)
def test_door_fixtures(model, frame, opening, underside):
    geoms = {n.split("/")[-2] + "/" + n.split("/")[-1]: g for n, g in site_geoms(model).items()}
    jambs = [geoms[f"{frame}/proxy_{i}"] for i in (0, 1)]
    lintel = geoms[f"{frame}/proxy_2"]
    # the jambs' faces along the frame's own axis (yaw 0: x; yaw pi/2: y)
    yaw = 2 * math.atan2(floats(jambs[0].get("quat"))[3], floats(jambs[0].get("quat"))[0])
    axis = 0 if abs(math.sin(yaw)) < 0.5 else 1
    centres = sorted(floats(j.get("pos"))[axis] for j in jambs)
    half_width = floats(jambs[0].get("size"))[0]  # the jamb's local x half size lies along the frame's axis
    assert centres[1] - centres[0] - 2 * half_width == pytest.approx(opening, abs=1e-6)
    lintel_bottom = floats(lintel.get("pos"))[2] - floats(lintel.get("size"))[2]
    assert lintel_bottom == pytest.approx(underside, abs=1e-6)
