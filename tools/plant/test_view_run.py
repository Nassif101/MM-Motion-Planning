"""Tests of view_run.py's pure parts (MuJoCo plant plan Task 9)."""
import json

import view_run as vr


def corpus(tmp_path):
    path = tmp_path / "corpus.json"
    path.write_text(json.dumps({"cells": [{"cell": "v2_door_105_nav", "scenario": "v2_door_105_nav", "bag": "runs/door/bag"},
                                          {"cell": "v2_door_then_beam_nav", "scenario": "v2_door_then_beam_nav", "bag": None}]}))
    return path


def test_live_takes_the_recorded_route_of_a_corpus_cell(tmp_path):
    # a straight start-goal line runs through walls (door_105: into the south wall): a cell's recorded route instead
    assert vr.route_source("v2_door_105_nav", None, False, corpus(tmp_path)) == ("runs/door/bag", "recorded")


def test_explicit_bag_or_straight_wins(tmp_path):
    assert vr.route_source("v2_door_105_nav", "my/bag", False, corpus(tmp_path)) == ("my/bag", "recorded")
    assert vr.route_source("v2_door_105_nav", None, True, corpus(tmp_path)) == ("", "straight")


def test_cells_without_a_recording_go_straight(tmp_path):
    assert vr.route_source("v2_door_then_beam_nav", None, False, corpus(tmp_path)) == ("", "straight")
    assert vr.route_source("v2_open_yard_nav", None, False, corpus(tmp_path)) == ("", "straight")


def test_scan_colours_follow_the_hit_classes():
    assert set(vr.CLASS_RGBA) == set(range(len(vr.ps.HIT_CLASSES)))
    assert vr.CLASS_RGBA[vr.ps.HIT_CLASSES.index("ground")] == vr.CLASS_COLOURS["ground"]


class StubClient:
    def __init__(self, frames):
        self.frames = list(frames)

    def next(self, timeout=None):
        return self.frames.pop(0) if self.frames else None


def test_session_hello_raises_on_an_error_event():
    # a failed reset (unknown scenario, a bag the server cannot find) must not leave the viewer waiting forever
    import pytest
    client = StubClient([(vr.ps.EVENT, {"event": "error", "reason": "no bag at experiment_runs/x"}, [])])
    with pytest.raises(RuntimeError, match="no bag"):
        vr.session_hello(client, timeout_s=1.0)


def test_session_hello_times_out_and_returns_the_hello():
    import pytest
    hello = {"schema": 1, "scenario": "v2_open_yard_nav"}
    assert vr.session_hello(StubClient([(vr.ps.STATE, {}, []), (vr.ps.HELLO, hello, [])]), timeout_s=1.0) == hello
    with pytest.raises(TimeoutError):
        vr.session_hello(StubClient([]), timeout_s=0.2)


def test_scene_places_moved_reserved_bodies():
    # needs mujoco: the host's .venv-plant (the container has none), run directly:
    #   .venv-plant/bin/python -c "import sys; sys.path.insert(0, 'tools/plant'); import test_view_run as t; t.test_scene_places_moved_reserved_bodies()"
    import pytest
    pytest.importorskip("mujoco")
    hello = {"boxes": [], "movers": [], "reserved": [{"name": "reserved/0", "size": [0.6, 0.6, 1.8]}]}
    scene = vr.Scene(hello)
    state = [0.0] * 9
    scene.apply(vr.ps.STATE, {"t": 0.1, "movers": [], "bodies": [["reserved/0", 2.0, -3.0, 0.5]]}, [state, state, [0.0] * 8])
    body = scene.data.body("reserved/0")
    assert abs(body.xpos[0] - 2.0) < 1e-9 and abs(body.xpos[1] + 3.0) < 1e-9


def test_parked_reserved_bodies_do_not_push_the_near_clip_away():
    # bodies parked 1 km out inflate the model's extent, and MuJoCo's clip planes scale with it: the near plane must
    # stay a few centimetres from the camera, or nothing near the robot is drawn
    import pytest
    pytest.importorskip("mujoco")
    model, _ = vr.build_scene({"boxes": [], "movers": [], "reserved": [{"name": "reserved/0", "size": [0.6, 0.6, 1.8]}]})
    assert model.vis.map.znear * model.stat.extent <= 0.1
    assert model.vis.map.zfar * model.stat.extent >= 150.0


def test_live_refuses_a_server_with_another_site(tmp_path):
    import pytest
    site = tmp_path / "site.xml"
    site.write_text('<mujoco>\n  <custom>\n    <text name="site_checksum" data="abc123"/>\n  </custom>\n</mujoco>\n')
    assert vr.site_checksum(site) == "abc123"
    vr.check_site({"site_checksum": "abc123"}, site)
    with pytest.raises(RuntimeError, match="site"):
        vr.check_site({"site_checksum": "other"}, site)
