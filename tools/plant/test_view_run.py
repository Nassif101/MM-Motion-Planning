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
