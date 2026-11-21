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


def test_decode_rejects_malformed_frames():
    import struct
    import pytest
    good = ps.encode(ps.STATE, {"t": 1.0}, [[1.0, 2.0]])
    bad_type = good[:4] + struct.pack("<H", 99) + good[6:]
    with pytest.raises(ValueError):
        ps.decode(bad_type)
    text = b'{"t":1.0}'
    long_json = struct.pack("<HI", ps.STATE, 999) + text + struct.pack("<I", 0)  # json_len beyond the frame
    with pytest.raises(ValueError):
        ps.decode(struct.pack("<I", len(long_json)) + long_json)
    many_floats = struct.pack("<HI", ps.STATE, len(text)) + text + struct.pack("<II", 1, 10 ** 6)  # floats beyond it
    with pytest.raises(ValueError):
        ps.decode(struct.pack("<I", len(many_floats)) + many_floats)


def test_truncated_run_log_yields_what_it_has(tmp_path):
    import struct
    import zlib
    def chunk(raw):
        packed = zlib.compress(raw)
        return struct.pack("<II", len(raw), len(packed)) + packed
    hello = ps.encode(ps.HELLO, {"schema": 1}, [])
    state = ps.encode(ps.STATE, {"t": 0.02}, [[1.0]])
    whole = chunk(hello + state)
    cut = chunk(state * 3)[:-7]  # the writer died mid-chunk
    path = tmp_path / "run.mmlog"
    path.write_bytes(b"MMRUNLG1" + whole + cut)
    log = ps.RunLog(path)
    assert [h["t"] for k, h, b in log.frames()] == [0.02]


def test_command_sends_with_the_clients_own_timeout():
    # next(timeout=0.05) leaves a short socket timeout behind: a command must not be cut mid-frame by it
    calls = []

    class Sock:
        def settimeout(self, t):
            calls.append(("timeout", t))

        def sendall(self, data):
            calls.append(("send", len(data)))

    client = ps.Client.__new__(ps.Client)
    client.sock, client.buffer, client.timeout = Sock(), b"", 10.0
    client.command("pause")
    assert calls[0] == ("timeout", 10.0) and calls[1][0] == "send"


def test_run_log_reads_schema_1_to_3(tmp_path):
    # E1 bumped the schema to 2 (the decision trace), spec 2 Task 9 to 3 (Goal and Route frames); the git-ignored
    # baseline logs are schema 1 and stay readable
    import struct
    import pytest
    import zlib
    for schema, readable in ((1, True), (2, True), (3, True), (4, False)):
        raw = ps.encode(ps.HELLO, {"schema": schema}, [])
        packed = zlib.compress(raw)
        path = tmp_path / f"s{schema}.mmlog"
        path.write_bytes(b"MMRUNLG1" + struct.pack("<II", len(raw), len(packed)) + packed)
        if readable:
            assert ps.RunLog(path).header["schema"] == schema
        else:
            with pytest.raises(ValueError):
                ps.RunLog(path)


FIXTURES = Path(__file__).parent / "fixtures"


def test_schema3_goal_and_route_records_match_cpp():
    # the frames the C++ gtest Frames.GoalAndRouteRecordsMatchTheCommittedFixtures encodes
    goal = (FIXTURES / "goal_v3.bin").read_bytes()
    kind, header, blocks, used = ps.decode(goal)
    assert kind == ps.GOAL and used == len(goal) and blocks == []
    assert header["event"] == "goal_accepted" and header["goal"]["goal_id"] == 4
    assert header["goal"]["target"] == [1.5, -2.25, 0.5] and header["goal"]["state"] == "planning"
    assert header["request"] == {"target": [1.5, -2.25, 0.5], "timeout_s": 60.0}
    assert ps.encode(ps.GOAL, header, blocks) == goal  # the same bytes from Python
    rejected = (FIXTURES / "goal_rejected_v3.bin").read_bytes()  # a refused request: no goal, its request kept
    kind, header, blocks, used = ps.decode(rejected)
    assert kind == ps.GOAL and header["goal"] is None and header["event"] == "goal_rejected"
    assert header["request"] == {"target": [101.0, -2.25, 0.5], "timeout_s": 20.0}
    assert ps.encode(ps.GOAL, header, blocks) == rejected
    route = (FIXTURES / "route_v3.bin").read_bytes()
    kind, header, blocks, used = ps.decode(route)
    assert kind == ps.ROUTE and used == len(route)
    path, points = ps.route_path(blocks)
    assert len(path) == header["path_poses"] == 3 and len(points) == header["route_points"] == 3
    assert path[-1] == (1.5, -2.25, 0.25) and points[1] == (0.5, -0.5)
    assert ps.encode(ps.ROUTE, header, blocks) == route


def test_route_blocks_must_match_their_layout():
    import pytest
    with pytest.raises(ValueError):
        ps.route_path([[1.0, 2.0], []])


def test_logs_of_all_three_schemas_are_read(tmp_path):
    import zlib, struct
    for schema in (1, 2, 3):
        raw = ps.encode(ps.HELLO, {"schema": schema}, []) + ps.encode(ps.GOAL, {"event": "goal_accepted"}, [])
        packed = zlib.compress(raw, 6)
        path = tmp_path / f"s{schema}.mmlog"
        path.write_bytes(b"MMRUNLG1" + struct.pack("<II", len(raw), len(packed)) + packed)
        log = ps.RunLog(path)
        assert log.header["schema"] == schema
        assert [k for k, _, _ in log.frames()][-1] == ps.GOAL


def test_runlog_fixture_reads_back():
    """Spec 4C Task 1: the shared schema-3 run-log fixture (the viewer's C# reader reads the same file)."""
    log = ps.RunLog(Path(__file__).parent / "fixtures" / "runlog_v3.mmlog")
    assert log.header["schema"] == 3 and log.header["scenario"] == "v2_door_105_nav"
    assert set(log.header["capabilities"]) >= {"b5_prediction", "plan_display_samples", "scan_point_times"}
    frames = list(log.frames())
    assert len(frames) == 24  # 25 with the Hello
    states = [h["t"] for k, h, _ in frames if k == ps.STATE]
    assert states == [round(0.02 * i, 2) for i in range(1, 21)]
    kinds = [k for k, _, _ in frames]
    assert kinds.count(ps.PLANS) == 1 and kinds.count(ps.PREDICTION) == 2 and kinds.count(ps.GOAL) == 1
    plans = next(b for k, _, b in frames if k == ps.PLANS)
    assert len(plans) == 2 and len(plans[1]) % 10 == 0  # samples and display samples
    data = (Path(__file__).parent / "fixtures" / "runlog_v3.mmlog").read_bytes()
    import struct
    raw0, packed0 = struct.unpack_from("<II", data, 8)
    assert 8 + 8 + packed0 < len(data), "two chunks"
