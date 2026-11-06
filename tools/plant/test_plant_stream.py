"""Tests of the Python side of the plant's stream protocol (MuJoCo plant plan Task 9)."""
from pathlib import Path

import plant_stream as ps

FIXTURE = Path(__file__).parent / "fixtures" / "frame_v1.bin"


def test_codec_matches_cpp():
    # the same frame the C++ gtest Frames.MatchTheCommittedFixture encodes
    frame = ps.encode(ps.STATE, {"t": 1.25, "name": "fixture", "list": [1, 2, 3]}, [[1.5, -2.0], [], [3.25]])
    assert frame == FIXTURE.read_bytes()
    kind, header, blocks, consumed = ps.decode(frame)
    assert kind == ps.STATE and header["name"] == "fixture" and blocks == [[1.5, -2.0], [], [3.25]] and consumed == len(frame)


def test_decode_partial_buffer_returns_none():
    frame = FIXTURE.read_bytes()
    for n in range(len(frame)):
        assert ps.decode(frame[:n]) is None


def test_decode_a_stream_frame_by_frame():
    data = ps.encode(ps.HELLO, {"schema": 1}, []) + ps.encode(ps.EVENT, {"event": "goal_reached"}, [[0.5]])
    kind, header, _, used = ps.decode(data)
    assert kind == ps.HELLO
    kind, header, blocks, _ = ps.decode(data[used:])
    assert kind == ps.EVENT and header["event"] == "goal_reached" and blocks == [[0.5]]


def test_run_log_reader_reads_frames(tmp_path):
    import zlib, struct
    frames = [ps.encode(ps.HELLO, {"scenario": "x"}, []), ps.encode(ps.STATE, {"t": 0.02}, [[1.0] * 9])]
    raw = b"".join(frames)
    packed = zlib.compress(raw, 6)
    path = tmp_path / "run.mmlog"
    path.write_bytes(b"MMRUNLG1" + struct.pack("<II", len(raw), len(packed)) + packed)
    log = ps.RunLog(path)
    assert log.header == {"scenario": "x"}
    got = list(log.frames())
    assert len(got) == 1 and got[0][0] == ps.STATE and got[0][1]["t"] == 0.02


def test_hit_classes_match_the_plant_enum():
    # a Scan's class block holds the plant's HitClass values (Session casts the enum): the names in declaration order
    import re
    header = Path(__file__).resolve().parents[2] / "ros2_ws/src/mobile_manipulator_plant/include/mobile_manipulator_plant/lidar.hpp"
    body = re.search(r"enum class HitClass : uint8_t \{([^}]*)\}", header.read_text()).group(1)
    assert ps.HIT_CLASSES == [name.strip().lower() for name in body.split(",")]
