"""Tests of the rollout tool (E2 plan Tasks 2-3): synthetic run logs, B5's model replayed against the logged motion."""
import gc
import math
import struct
import sys
import threading
import zlib
from pathlib import Path

import pytest

import model_rollout as mr
import plant_stream as ps

DT = 0.02


def write_log(path, frames, header=None):
    raw = ps.encode(ps.HELLO, header or {"schema": 2, "scenario": "v2_x", "seed": 1, "mode": "realistic"}, [])
    raw += b"".join(ps.encode(k, h, b) for k, h, b in frames)
    packed = zlib.compress(raw)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"MMRUNLG1" + struct.pack("<II", len(raw), len(packed)) + packed)
    return path


def unicycle(batch):
    """A fake model step: the unicycle with arm joint velocities, explicit midpoint (exact for constant omega = 0)."""
    out = []
    for dt, x, u in batch:
        v, w = u[0], u[1]
        yaw_mid = x[2] + 0.5 * w * dt
        out.append([x[0] + v * dt * math.cos(yaw_mid), x[1] + v * dt * math.sin(yaw_mid), x[2] + w * dt] +
                   [x[3 + i] + u[2 + i] * dt for i in range(6)])
    return out


def state_frame(t, pose, cmd, twist=None, qd_ref=None, goal=0, diag=(0.0, 0.0, None), pair="", grad=None, kink=None,
                diagnostics=True):
    h = {"t": t, "goal": goal, "status": "ok", "gate": [0.0, 0.0, 0], "fallback": ""}
    if diagnostics:
        h.update({"diag": list(diag), "diag_pair": pair, "diag_grad": grad or [0.0] * 9, "diag_kink": kink or [0.0] * 9,
                  "diag_cmd": list(cmd)})
    executed = list(twist if twist is not None else cmd[:2]) + list(qd_ref if qd_ref is not None else cmd[2:])
    return (ps.STATE, h, [list(pose), list(pose), executed])


def drive(commands, start=None, effect=None, t0=0.0):
    """Frames from integrating the fake model: record j carries the command of the transition into it (the time
    convention); `effect(u)` is the plant's response to the command (default: as sent)."""
    x = list(start or [0.0] * 9)
    frames = [state_frame(t0, x, [0.0] * 8)]
    for j, u in enumerate(commands, start=1):
        executed = effect(u) if effect else list(u)
        x = unicycle([(DT, x, executed)])[0]
        frames.append(state_frame(t0 + j * DT, x, u, twist=executed[:2], qd_ref=executed[2:]))
    return frames


def changing_commands(n):
    return [[0.1 + 0.02 * j, 0.3 * math.sin(0.4 * j), 0.05 * math.cos(0.3 * j), 0, 0, 0, 0, 0] for j in range(n)]


def test_rollout_reproduces_a_unicycle_log(tmp_path):
    log = write_log(tmp_path / "run.mmlog", drive(changing_commands(40)))
    _, recs = mr.records(log)
    wins = mr.windows(recs, 0.1)
    assert len(wins) > 20
    worst = 0.0
    for i0, i1 in wins:
        e = mr.error(recs, i0, i1, mr.rollout(recs, i0, i1, "cmd", unicycle))
        worst = max(worst, abs(e["forward"]), abs(e["lateral"]), abs(e["yaw"]), *map(abs, e["q"]))
    assert worst < 1e-6  # the poses are float32 in the log
    # the convention matters: the command of the record before is a cycle off
    shifted = [dict(r) for r in recs]
    for j in range(len(shifted) - 1, 0, -1):
        shifted[j]["cmd"] = recs[j - 1]["cmd"]
    off = max(abs(mr.error(shifted, i0, i1, mr.rollout(shifted, i0, i1, "cmd", unicycle))["forward"]) +
              abs(mr.error(shifted, i0, i1, mr.rollout(shifted, i0, i1, "cmd", unicycle))["yaw"]) for i0, i1 in wins)
    assert off > 1e-4


def test_lateral_error_is_in_the_body_frame(tmp_path):
    frames, x = [], [0.0, 0.0, 1.0] + [0.0] * 6
    for j in range(11):  # 1 mm of body-left drift per frame, zero commands
        frames.append(state_frame(j * DT, list(x), [0.0] * 8))
        x[0] -= 0.001 * math.sin(1.0)
        x[1] += 0.001 * math.cos(1.0)
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", frames))
    e = mr.error(recs, 0, 5, mr.rollout(recs, 0, 5, "cmd", unicycle))
    assert e["lateral"] == pytest.approx(0.005, abs=1e-6)
    assert e["forward"] == pytest.approx(0.0, abs=1e-6)


def test_twist_and_cmd_inputs_differ(tmp_path):
    understeer = lambda u: [u[0], 0.79 * u[1]] + list(u[2:])  # noqa: E731
    frames = drive([[0.2, 0.5, 0, 0, 0, 0, 0, 0]] * 10, effect=understeer)
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", frames))
    by_twist = mr.error(recs, 1, 6, mr.rollout(recs, 1, 6, "twist", unicycle))  # from 1: record 0 is at rest
    by_cmd = mr.error(recs, 1, 6, mr.rollout(recs, 1, 6, "cmd", unicycle))
    assert by_twist["yaw"] == pytest.approx(0.0, abs=1e-6)
    assert by_cmd["yaw"] == pytest.approx(-0.21 * 0.5 * 0.1, abs=1e-6)


def test_twist_inputs_use_each_series_at_its_sample_time(tmp_path):
    # Task 1 response check R2: record j's measured twist and arm-reference velocity are samples at t[j] (after the
    # transition into j), not values held over it. The JTC holds the reference sampled at the interval's start, so the
    # arm moves by qd_ref[j-1] dt; the base's speed ramps, so the trapezoid of the two twist samples is its mean.
    a, frames, x = 0.5, [], [0.0] * 9
    for j in range(16):
        t = j * DT
        frames.append(state_frame(t, list(x), [0.0] * 8, twist=[a * t, 0.0], qd_ref=[0.01 * j] + [0.0] * 5))
        x = list(x)
        x[0] += a * (t + 0.5 * DT) * DT  # the exact distance under v = a t
        x[3] += 0.01 * j * DT            # the held reference sample
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", frames))
    for i0, i1 in mr.windows(recs, 0.1):
        e = mr.error(recs, i0, i1, mr.rollout(recs, i0, i1, "twist", unicycle))
        assert abs(e["forward"]) < 1e-6 and abs(e["q"][0]) < 1e-7, (i0, e)


def test_windows_skip_goal_changes_and_gaps(tmp_path):
    frames = drive(changing_commands(30))
    for k in range(16, len(frames)):  # the goal changes at record 16
        frames[k][1]["goal"] = 1
    gap = drive(changing_commands(10), start=frames[-1][2][0], t0=frames[-1][1]["t"] + 0.1)[1:]  # 0.1 s without frames
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", frames + gap))
    wins = mr.windows(recs, 0.1)
    assert wins
    for i0, i1 in wins:
        assert recs[i0]["goal"] == recs[i1]["goal"]
        assert all(recs[j]["t"] - recs[j - 1]["t"] <= 0.03 for j in range(i0 + 1, i1 + 1))
        assert recs[i1]["t"] - recs[i0]["t"] == pytest.approx(0.1, abs=1e-6)


def test_log_without_diagnostics_is_named(tmp_path):
    frames = [state_frame(j * DT, [0.0] * 9, [0.0] * 8, diagnostics=False) for j in range(5)]
    log = write_log(tmp_path / "plain.mmlog", frames)
    with pytest.raises(ValueError, match="without --diagnostics"):
        mr.records(log)


def test_records_carry_the_diagnostics(tmp_path):
    frames = [state_frame(0.0, [0.0] * 9, [0.1] * 8, diag=(0.006, 0.005, 0.001), pair="3|2|wall",
                          grad=[0.0, 1.0] + [0.0] * 7, kink=[0.0, 0.0, -1.0] + [0.0] * 6)]
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", frames))
    r = recs[0]
    assert r["depth"] == pytest.approx(0.006) and r["depth_stage1"] == pytest.approx(0.005)
    assert r["pair"] == "3|2|wall" and r["grad"][1] == 1.0 and r["kink"][2] == -1.0
    assert len(r["pose"]) == len(r["b5"]) == 9 and len(r["cmd"]) == 8 and len(r["qd_ref"]) == 6


def test_missing_diagnostics_stay_unavailable(tmp_path):
    # Task 2 review F3: a field the log does not carry is None (unavailable), never a measured zero
    base = state_frame(0.0, [0.0] * 9, [0.1] * 8, diag=(0.006, 0.005, None), pair="12|13|floor",
                       grad=[0.0, 1.0] + [0.0] * 7, kink=[0.5] + [0.0] * 8)
    for key, field in (("diag", "depth"), ("diag_pair", "pair"), ("diag_grad", "grad"), ("diag_kink", "kink")):
        kind, header, blocks = base
        header = {k: v for k, v in header.items() if k != key}
        _, recs = mr.records(write_log(tmp_path / key / "run.mmlog", [(kind, header, blocks)]))
        assert recs[0][field] is None, key
        if key == "diag":
            assert recs[0]["depth_stage1"] is None
        others = {"depth": 0.006, "pair": "12|13|floor", "grad": [0.0, 1.0] + [0.0] * 7, "kink": [0.5] + [0.0] * 8}
        for other, value in others.items():
            if other != field:
                assert recs[0][other] == pytest.approx(value) if other != "pair" else recs[0][other] == value, (key, other)


def test_log_without_diagnostics_names_the_file(tmp_path):
    log = write_log(tmp_path / "plain.mmlog", [state_frame(0.0, [0.0] * 9, [0.0] * 8, diagnostics=False)])
    with pytest.raises(ValueError, match="plain.mmlog"):
        mr.records(log)


# A stand-in for model_step speaking its line protocol: 9 numbers per line, flushed per line; exit 2 on a bad line.
FAKE_CHILD = r"""
import sys
for n, line in enumerate(sys.stdin, start=1):
    v = line.split()
    if len(v) != 18:
        sys.stderr.write("fake: line %d\\n" % n); sys.exit(2)
    print(" ".join(v[1:10]), flush=True)
"""
SHORT_CHILD = r"""
import sys
for line in sys.stdin:
    print(" ".join(line.split()[1:9]), flush=True)
"""


def fake_stepper(code=FAKE_CHILD):
    return mr._Stepper([sys.executable, "-c", code], None)


def call_with_timeout(fn, timeout_s=20.0):
    box = {}

    def run():
        try:
            box["result"] = fn()
        except Exception as e:  # noqa: BLE001 - the test inspects it
            box["error"] = e
    worker = threading.Thread(target=run, daemon=True)
    worker.start()
    worker.join(timeout_s)
    return worker.is_alive(), box


def test_a_batch_larger_than_the_pipes_does_not_deadlock():
    # Task 2 review F1: requests written while responses are read; 20000 lines are about 7 MB each way
    batch = [(0.02, [0.12345678901234567] * 9, [0.02345678901234567] * 8)] * 20000
    with fake_stepper() as step:
        hung, box = call_with_timeout(lambda: step(batch))
        if hung:
            step.proc.kill()
        assert not hung, "the stepper deadlocked on a large batch"
        assert len(box["result"]) == 20000 and all(len(x) == 9 for x in box["result"])


def test_a_bad_line_in_a_large_batch_raises_and_does_not_hang():
    batch = [(0.02, [0.0] * 9, [0.0] * 8)] * 5000 + [(0.02, [0.0] * 9, [0.0] * 7)] + [(0.02, [0.0] * 9, [0.0] * 8)] * 5000
    with fake_stepper() as step:
        hung, box = call_with_timeout(lambda: step(batch))
        if hung:
            step.proc.kill()
        assert not hung
        assert isinstance(box.get("error"), RuntimeError), box


def test_a_short_answer_is_an_error():
    with fake_stepper(SHORT_CHILD) as step:
        with pytest.raises(RuntimeError, match="9"):
            step([(0.02, [0.0] * 9, [0.0] * 8)])


STALLED_CHILD = r"""
import sys, time
sys.stdin.readline()
time.sleep(0.2)
print("0 " * 8, flush=True)
time.sleep(30)
"""


def test_an_error_with_a_blocked_writer_cleans_up_in_time():
    # Task 2 response check R1: the child answers wrongly and then stops reading while the writer is blocked on a full
    # pipe: the original error reaches the caller, and neither the writer nor the child outlives the cleanup
    batch = [(0.02, [0.12345678901234567] * 9, [0.02345678901234567] * 8)] * 20000
    step = fake_stepper(STALLED_CHILD)
    proc = step.proc
    hung, box = call_with_timeout(lambda: step(batch), timeout_s=15.0)
    if hung:
        proc.kill()
    assert not hung, "cleanup blocked behind the writer"
    assert isinstance(box.get("error"), RuntimeError) and "8 numbers" in str(box["error"]), box
    assert proc.poll() is not None
    assert step._writer is None or not step._writer.is_alive()


def test_the_model_process_ends_with_its_owner():
    # Task 2 review F2: a context, an exception inside it, and a discarded owner all end the child
    with fake_stepper() as step:
        step([(0.02, [0.0] * 9, [0.0] * 8)])
        proc = step.proc
    assert proc.poll() is not None
    with pytest.raises(KeyError):
        with fake_stepper() as step:
            proc = step.proc
            raise KeyError("boom")
    assert proc.poll() is not None
    step = fake_stepper()
    proc = step.proc
    del step
    gc.collect()
    proc.wait(timeout=5)  # raises TimeoutExpired if the discarded owner left it running
    step = fake_stepper()
    step.close()
    step.close()  # idempotent


@pytest.mark.skipif(not mr.BINARY_PATH.exists(), reason="model_step is not built here (runs in the container)")
def test_container_step_follows_the_constant_twist_arc():
    # B5's RK4 against the exact arc of a constant (v, omega): RK4's truncation over 20 ms is far below 1e-9
    import random
    rng = random.Random(7)
    batch = []
    for _ in range(20):
        x = [rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-3, 3)] + [rng.uniform(-1, 1) for _ in range(6)]
        u = [rng.uniform(-0.5, 0.5), rng.uniform(-0.8, 0.8)] + [rng.uniform(-0.3, 0.3) for _ in range(6)]
        batch.append((DT, x, u))
    with mr.container_step(container=None) as step:
        got = step(batch)
    assert len(got) == len(batch)
    for (dt, x, u), y in zip(batch, got):
        assert len(y) == 9
        v, w, yaw = u[0], u[1], x[2]
        arc = [x[0] + v / w * (math.sin(yaw + w * dt) - math.sin(yaw)), x[1] - v / w * (math.cos(yaw + w * dt) - math.cos(yaw)),
               yaw + w * dt] + [x[3 + i] + u[2 + i] * dt for i in range(6)]
        assert max(abs(p - q) for p, q in zip(y, arc)) < 1e-9


@pytest.mark.skipif(not mr.BINARY_PATH.exists(), reason="model_step is not built here (runs in the container)")
def test_container_step_matches_the_fake_on_a_unicycle():
    import random
    rng = random.Random(4)
    batch = []
    for _ in range(20):
        x = [rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-3, 3)] + [rng.uniform(-1, 1) for _ in range(6)]
        u = [rng.uniform(-0.5, 0.5), 0.0] + [rng.uniform(-0.3, 0.3) for _ in range(6)]  # omega 0: the fake is exact
        batch.append((DT, x, u))
    with mr.container_step(container=None) as step:
        got = step(batch)
    assert len(got) == len(batch)
    for a, b in zip(got, unicycle(batch)):
        assert len(a) == 9
        assert max(abs(p - q) for p, q in zip(a, b)) < 1e-9
