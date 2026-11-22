"""Spec 4C Task 5: the render script's pure parts (disk estimate and refusal, frame count, the ffmpeg command)."""
from pathlib import Path

import pytest

import render_video as rv


def test_frame_count_follows_speed():
    assert rv.frame_count(0.02, 0.40, None, None, 1.0) == 23  # 0.38 s at 60 frames per second: 22 intervals
    assert rv.frame_count(0.0, 10.0, 2.0, 4.0, 0.5) == 241
    assert rv.frame_count(0.02, 0.40, 1.0, 2.0, 1.0) == 0  # a range after the log


def test_estimate_and_refusal():
    jpeg = rv.estimate_bytes(1000, png=False)
    png = rv.estimate_bytes(1000, png=True)
    assert png > jpeg > 0
    assert rv.check_disk(1000, png=False, free=jpeg * 2) is None
    reason = rv.check_disk(1000, png=True, free=jpeg * 2)  # RefusesWhenTheDiskIsTooSmall
    assert reason is not None and "disk" in reason


def test_ffmpeg_command():
    cmd = rv.ffmpeg_command(Path("/tmp/frames"), Path("/tmp/out.mp4"), png=False)
    assert cmd[0] == "ffmpeg"
    assert cmd[cmd.index("-framerate") + 1] == "60"
    assert cmd[cmd.index("-i") + 1] == "/tmp/frames/frame_%06d.jpg"
    assert cmd[cmd.index("-c:v") + 1] == "libx264"
    assert cmd[cmd.index("-pix_fmt") + 1] == "yuv420p"
    assert cmd[cmd.index("-crf") + 1] == "18"
    assert cmd[-1] == "/tmp/out.mp4"
    assert rv.ffmpeg_command(Path("/f"), Path("/o.mp4"), png=True)[cmd.index("-i") + 1] == "/f/frame_%06d.png"


def test_player_args():
    args = rv.player_args(Path("/r/demo.mmlog"), Path("/r/j.jsonl"), Path("/r/frames"), speed=0.5, start=3.0, end=None,
                          png=False, quality=None)
    assert args[:6] == ["-render", "/r/demo.mmlog", "-journal", "/r/j.jsonl", "-out", "/r/frames"]
    assert "-from" in args and "-to" not in args and "-png" not in args
    assert args[args.index("-screen-width") + 1] == "1920" and args[args.index("-screen-height") + 1] == "1080"


def test_render_timeout_scales_with_frames():
    assert rv.render_timeout(1201) >= 300
    assert rv.render_timeout(36000) > rv.render_timeout(1201)


def test_non_finite_numbers_are_refused():
    import argparse
    for bad in ("nan", "inf", "-inf"):
        with pytest.raises(argparse.ArgumentTypeError):
            rv.finite(bad)
    assert rv.finite("2.5") == 2.5
