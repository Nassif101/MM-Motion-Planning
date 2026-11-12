"""Tests of the rollout tool (E2 plan Tasks 2-3): synthetic run logs, B5's model replayed against the logged motion."""
import gc
import json
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


def test_records_carry_the_plans_first_control(tmp_path):
    kind, h, b = state_frame(0.0, [0.0] * 9, [0.07, 0.0] + [0.0] * 6)
    h["diag_plan_u0"] = [0.004, 0.0] + [0.0] * 6
    _, recs = mr.records(write_log(tmp_path / "a" / "run.mmlog", [(kind, h, b)]))
    assert recs[0]["plan_u0"][0] == pytest.approx(0.004)
    _, recs = mr.records(write_log(tmp_path / "b" / "run.mmlog", [state_frame(0.0, [0.0] * 9, [0.0] * 8)]))
    assert recs[0]["plan_u0"] is None  # logs from before Task 4


def test_records_carry_the_nearest_pair_probe(tmp_path):
    kind, h, b = state_frame(0.0, [0.0] * 9, [0.0] * 8)
    h["diag_nearest"] = ["base_link", "door", 0.004, 0, 1, 3]
    _, recs = mr.records(write_log(tmp_path / "a" / "run.mmlog", [(kind, h, b)]))
    assert recs[0]["nearest"] == {"link": "base_link", "obstacle": "door", "distance": 0.004, "in_stage1": False,
                                  "in_rows": True, "dropped": 3}
    _, recs = mr.records(write_log(tmp_path / "b" / "run.mmlog", [state_frame(0.0, [0.0] * 9, [0.0] * 8)]))
    assert recs[0]["nearest"] is None
    # Task 4b review F3: a cycle the probe did not fill (no solved, gated cycle) is unavailable, not a zero clearance
    kind, h, b = state_frame(0.0, [0.0] * 9, [0.0] * 8)
    h["diag_nearest"] = ["", "", 0.0, 0, 0, 0]
    _, recs = mr.records(write_log(tmp_path / "c" / "run.mmlog", [(kind, h, b)]))
    assert recs[0]["nearest"] is None


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


# ---- Task 3: the reports -------------------------------------------------------------------------------------------

FINE = 0.001


def first_order(values, tau, h=FINE):
    """A first-order filter (time constant tau) of a series sampled every h, zero-order hold."""
    if tau <= 0:
        return list(values)
    a, z, out = math.exp(-h / tau), 0.0, []
    for u in values:
        out.append(z)
        z = a * z + (1 - a) * u
    return out


def simulate(n, cmd, delay=0.0, tau=0.0, gain_w=1.0, slip=0.0, slip_tau=0.0, arm=None, b5_arm_offset=0.0):
    """Frames of a base run simulated at 1 ms: record j's command cmd(j) (8) is applied over [t[j-1], t[j]) (the time
    convention); the base's v and omega follow it delayed by `delay` through a `tau` filter (omega times gain_w); the
    body-lateral velocity is a `slip_tau` filter of slip * v * omega. `arm(t_fine, qd_ref_samples) -> joint 0 velocity`
    with qd_ref samples given by arm.ref(j) overrides joint 0's motion."""
    per = int(round(DT / FINE))
    steps = n * per
    u_fine = [cmd(k // per + 1) for k in range(steps)]  # the command applied during fine step k
    shift = int(round(delay / FINE))
    v_in = [u_fine[k - shift][0] if k >= shift else 0.0 for k in range(steps)]
    w_in = [gain_w * (u_fine[k - shift][1] if k >= shift else 0.0) for k in range(steps)]
    v, w = first_order(v_in, tau), first_order(w_in, tau)
    lat = first_order([slip * a * b for a, b in zip(v, w)], slip_tau)
    x = [0.0] * 9
    frames = [state_frame(0.0, list(x), [0.0] * 8, twist=[0.0, 0.0], qd_ref=[0.0] * 6)]
    for k in range(steps):
        yaw = x[2]
        x[0] += (v[k] * math.cos(yaw) - lat[k] * math.sin(yaw)) * FINE
        x[1] += (v[k] * math.sin(yaw) + lat[k] * math.cos(yaw)) * FINE
        x[2] += w[k] * FINE
        if arm is not None:
            x[3] += arm.velocity(k * FINE) * FINE
        else:
            for i in range(6):
                x[3 + i] += u_fine[k][2 + i] * FINE
        if (k + 1) % per == 0:
            j = (k + 1) // per
            qd_ref = [arm.ref(j)] + [0.0] * 5 if arm is not None else list(cmd(j)[2:])
            frame = state_frame(j * DT, list(x), cmd(j), twist=[v[k], w[k]], qd_ref=qd_ref)
            frame[2][1] = list(x[:3]) + [q + b5_arm_offset for q in x[3:]]  # B5's state: the arm reference differs
            frames.append(frame)
    return frames


def steps_of(values, hold_records):
    return lambda j: values[(j // hold_records) % len(values)]


def test_twist_bins_recover_a_turn_gain(tmp_path):
    segments = [[0.2, w, 0, 0, 0, 0, 0, 0] for w in (0.03, 0.1, 0.4)]
    effect = lambda u: [u[0], (0.79 if 0.07 <= abs(u[1]) < 0.2 else 1.0) * u[1]] + list(u[2:])  # noqa: E731
    commands = [segments[min(j // 100, 2)] for j in range(300)]
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", drive(commands, effect=effect)))
    bins = mr.twist_bins(recs, unicycle)["bins"]
    by_w = {}
    for b in bins:
        if b["v"] == [0.1, 0.3]:
            by_w[tuple(b["omega"])] = b
    assert by_w[(0.07, 0.2)]["omega_ratio"] == pytest.approx(0.79, abs=0.01)
    assert by_w[(0.02, 0.07)]["omega_ratio"] == pytest.approx(1.0, abs=0.01)
    assert by_w[(0.2, 0.6)]["omega_ratio"] == pytest.approx(1.0, abs=0.01)
    assert by_w[(0.07, 0.2)]["v_ratio"] == pytest.approx(1.0, abs=0.01)
    assert by_w[(0.07, 0.2)]["yaw"] < 0  # the plant turns less than the model


def test_alignment_recovers_delay_and_lag(tmp_path):
    levels = [[s, 0, 0, 0, 0, 0, 0, 0] for s in (0.0, 0.3, 0.1, 0.25, 0.05, 0.2)]
    frames = simulate(500, steps_of(levels, 15), delay=0.04, tau=0.06)
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", frames))
    fit = mr.alignment(recs)["v"]
    assert fit["delay"] == pytest.approx(0.04, abs=1e-9)  # exact interval means (Task 3 review): on the grid point
    assert fit["tau"] == pytest.approx(0.06, abs=0.0051)
    assert fit["gain"] == pytest.approx(1.0, abs=0.02)
    assert mr.alignment(recs)["omega"] is None  # no turn: nothing to fit


class RampedHeldArm:
    """Joint 0: the reference is the command ramped over 0.04 s, sampled at each record (qd_ref) and held by the JTC
    from that sample; the joint moves at the held reference delayed by 0.03 s."""

    def __init__(self, cmd):
        self.cmd = cmd
        self.samples = {}
        ref, rate = 0.0, None
        for k in range(0, 600 * 20 + 1):
            t = k * FINE
            target = cmd(int(t / DT) + 1)[2]  # the command applied at t
            step = 0.3 / 0.04 * FINE  # ramps a 0.3 rad/s change in 0.04 s
            ref += max(-step, min(step, target - ref))
            if k % 20 == 0:
                self.samples[k // 20] = ref

    def ref(self, j):
        return self.samples.get(j, 0.0)

    def velocity(self, t):
        t = t - 0.03
        return self.ref(int(math.floor(t / DT + 1e-9))) if t >= 0 else 0.0


def test_alignment_separates_pipeline_and_servo(tmp_path):
    levels = [[0, 0, q, 0, 0, 0, 0, 0] for q in (0.0, 0.3, -0.1, 0.2, 0.05, -0.25)]
    cmd = steps_of(levels, 15)
    frames = simulate(500, cmd, arm=RampedHeldArm(cmd), b5_arm_offset=1e-3)
    frames[200][2][1][3:] = list(frames[200][2][0][3:])  # one reanchored cycle: B5's arm joints are the measured ones
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", frames))
    fit = mr.alignment(recs, qd_limits=[0.6] * 6)["q0"]
    assert fit["servo"]["delay"] == pytest.approx(0.03, abs=0.01)
    # the trajectory's ramp and the JTC's sample-and-hold add lag the pipeline fit carries as delay and time constant
    assert fit["pipeline"]["delay"] > fit["servo"]["delay"]
    assert fit["pipeline"]["delay"] + fit["pipeline"]["tau"] > fit["servo"]["delay"] + fit["servo"]["tau"] + 0.02
    assert fit["servo_excluded"] == 3  # the intervals next to the two reanchored records (0 and 200)
    assert fit["servo"]["n"] == 497
    assert fit["resolution_s"] == pytest.approx(0.01)


def diag_frames(n, dx_per_frame, grad, kink=None, depth0=0.006, ddepth=0.0, dyaw=0.0, pair="1|2|wall"):
    frames, x = [], [0.0] * 9
    for j in range(n):
        frames.append(state_frame(j * DT, list(x), [0.0] * 8, twist=[0.0, 0.0], qd_ref=[0.0] * 6,
                                  diag=(depth0 + ddepth * j, depth0 + ddepth * j, None), pair=pair, grad=list(grad),
                                  kink=list(kink or [0.0] * 9)))
        x = [a + b for a, b in zip(x, dx_per_frame)]
        x[2] += dyaw
    return frames


def test_gradient_projection_splits_the_depth_change(tmp_path):
    # the pair's distance falls as the base moves left (body-lateral, yaw 0): 1 mm of drift per 0.1 s, no command
    frames = diag_frames(30, [0.0, 0.0002] + [0.0] * 7, grad=[0.0, -1.0] + [0.0] * 7, ddepth=0.0002)
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", frames))
    g = mr.gradient_projection(recs, step=unicycle)
    assert g["smooth"]["windows"] > 10
    split = g["smooth"]["b5"]
    assert split["lateral"] == pytest.approx(0.001, rel=0.05)
    assert max(abs(split["forward"]), abs(split["yaw"]), abs(split["arm"])) < 1e-6
    assert g["smooth"]["depth_change"] == pytest.approx(0.001, rel=0.05)
    assert max(abs(v) for v in g["smooth"]["model"].values()) < 1e-9  # B5's model, commanded nothing, predicts nothing
    assert g["feature_switch"]["count"] == 0


def test_gradient_projection_does_not_split_feature_switch_windows(tmp_path):
    smooth = diag_frames(20, [0.0, 0.0002] + [0.0] * 7, grad=[0.0, -1.0] + [0.0] * 7, ddepth=0.0002)
    kinked = diag_frames(20, [0.0] * 9, grad=[0.0, -1.0, 0.0] + [0.0] * 6, kink=[0.0, 0.0, -1.0] + [0.0] * 6,
                         ddepth=0.0002, dyaw=0.0004)
    for k, (kind, h, b) in enumerate(kinked):
        h["t"] = (20 + k) * DT
        h["goal"] = 1
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", smooth + kinked))
    g = mr.gradient_projection(recs, step=unicycle)
    assert g["feature_switch"]["count"] > 5
    assert g["feature_switch"]["depth_change"] == pytest.approx(0.001, rel=0.05)
    assert all("b5" not in w for w in g["per_window"] if w["feature_switch"])
    assert g["smooth"]["windows"] == sum(1 for w in g["per_window"] if not w["feature_switch"])
    assert g["smooth"]["b5"]["yaw"] == pytest.approx(0.0, abs=1e-9)  # the turning windows are not in the pool


def test_gradient_projection_skips_unavailable_fields(tmp_path):
    frames = diag_frames(20, [0.0, 0.0002] + [0.0] * 7, grad=[0.0, -1.0] + [0.0] * 7, ddepth=0.0002)
    for kind, h, b in frames:
        del h["diag_kink"]
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", frames))
    g = mr.gradient_projection(recs, step=unicycle)
    assert g["smooth"]["windows"] == 0 and g["feature_switch"]["count"] == 0
    assert g["skipped"]["kink"] > 5


def test_build_time_recovers_a_first_order_sideslip(tmp_path):
    levels = [[v, w, 0, 0, 0, 0, 0, 0] for v, w in ((0.3, 0.4), (0.3, 0.0), (0.2, -0.5), (0.0, 0.0), (0.3, 0.3))]
    frames = simulate(600, steps_of(levels, 60), slip=0.05, slip_tau=0.3)
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", frames))
    assert mr.build_time(recs)["executed"]["tau"] == pytest.approx(0.3, abs=0.05)
    straight = simulate(100, lambda j: [0.2, 0.0, 0, 0, 0, 0, 0, 0])
    _, recs = mr.records(write_log(tmp_path / "straight.mmlog", straight))
    assert mr.build_time(recs) is None  # no transitions


@pytest.mark.skipif(not mr.BINARY_PATH.exists(), reason="model_step is not built here (runs in the container)")
def test_report_writes_json_and_markdown(tmp_path):
    levels = [[v, w, 0.05, 0, 0, 0, 0, 0] for v, w in ((0.3, 0.4), (0.3, 0.0), (0.2, -0.5), (0.0, 0.0))]
    logs = [write_log(tmp_path / f"r{k}" / "run.mmlog", simulate(200, steps_of(levels, 30 + 5 * k), slip=0.02,
                                                                    slip_tau=0.1)) for k in range(2)]
    out = tmp_path / "report"
    assert mr.main(["report", *map(str, logs), "--out", str(out), "--container", "none"]) == 0
    data = json.loads((out / "rollout.json").read_text())
    keys = {"horizon_table", "twist_bins", "alignment", "gradient_projection", "build_time"}
    assert set(data["pooled"]) >= keys
    assert len(data["runs"]) == 2 and all(set(r) >= keys for r in data["runs"].values())
    text = (out / "rollout.md").read_text()
    for heading in ("Horizons", "Twist bins", "Alignment", "Gradient projection", "Sideslip build-up"):
        assert heading in text


# ---- Task 3 review ---------------------------------------------------------------------------------------------------

SLIP_LEVELS = [[v, w, 0, 0, 0, 0, 0, 0] for v, w in ((0.3, 0.4), (0.3, 0.0), (0.2, -0.5), (0.0, 0.0), (0.3, 0.3))]


def test_build_time_separates_base_lag_and_delay_from_sideslip(tmp_path):
    # review F1: a base that lags or delays its command, with an instantaneous sideslip, has no lateral build-up; only
    # the fit against the executed twist says so (the command fit carries the base's delay and lag)
    cases = {"lateral lag": dict(slip_tau=0.3), "base lag": dict(tau=0.2), "base delay": dict(delay=0.12)}
    got = {}
    for name, kw in cases.items():
        frames = simulate(600, steps_of(SLIP_LEVELS, 60), slip=0.05, **kw)
        _, recs = mr.records(write_log(tmp_path / name / "run.mmlog", frames))
        got[name] = mr.build_time(recs)
    assert got["lateral lag"]["executed"]["tau"] == pytest.approx(0.3, abs=0.05)
    assert got["base lag"]["executed"]["tau"] < 0.03
    assert got["base delay"]["executed"]["tau"] < 0.03
    assert got["base delay"]["executed"]["delay"] < 0.03
    assert got["base delay"]["command"]["delay"] + got["base delay"]["command"]["tau"] > 0.1  # the effective response
    assert got["lateral lag"]["executed"]["transitions"] >= 3


def test_build_time_counts_transitions_within_segments(tmp_path):
    # review F4: four runs of one constant twist each: no transition inside any, so no build-up time
    frames, t0 = [], 0.0
    for v, w in ((0.3, 0.3), (0.3, -0.3), (0.3, 0.4), (0.3, -0.4)):
        run = simulate(60, lambda j, v=v, w=w: [v, w, 0, 0, 0, 0, 0, 0], slip=0.05)
        frames += run  # each run's time restarts at 0: a new segment
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", frames))
    assert len(mr.segments(recs)) == 4
    assert mr.build_time(recs) is None


def test_alignment_continues_across_a_goal_change_while_moving(tmp_path):
    # review F2: a mission's next goal does not reset the robot: the fit runs on through the goal change
    levels = [[s, 0, 0, 0, 0, 0, 0, 0] for s in (0.25, 0.3, 0.1, 0.25, 0.05, 0.2)]
    frames = simulate(500, steps_of(levels, 15), delay=0.04, tau=0.2)
    for kind, h, b in frames[250:]:
        h["goal"] = 1
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", frames))
    fit = mr.alignment(recs)["v"]
    assert fit["delay"] == pytest.approx(0.04, abs=1e-9)
    assert fit["tau"] == pytest.approx(0.2, abs=0.0051)
    assert fit["gain"] == pytest.approx(1.0, abs=0.01)


def test_alignment_warms_up_after_a_gap(tmp_path):
    # review F2: after a gap mid-run the robot is moving and its filter state unknown: the first outputs of the new
    # part are left out of the fit (a warm-up), not compared with a model starting from rest
    levels = [[s, 0, 0, 0, 0, 0, 0, 0] for s in (0.25, 0.3, 0.1, 0.25, 0.05, 0.2)]
    frames = simulate(500, steps_of(levels, 15), delay=0.04, tau=0.2)
    frames = frames[:200] + frames[210:]  # 0.2 s of frames lost, the base moving at about 0.2 m/s
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", frames))
    fit = mr.alignment(recs)["v"]
    assert fit["delay"] == pytest.approx(0.04, abs=0.0051)
    assert fit["tau"] == pytest.approx(0.2, abs=0.011)
    assert fit["gain"] == pytest.approx(1.0, abs=0.01)


def test_servo_is_inapplicable_in_ideal_mode(tmp_path):
    # review F5: the ideal plant has no arm reference (B5's state is the measured one): no servo fit, said so
    levels = [[0, 0, q, 0, 0, 0, 0, 0] for q in (0.0, 0.3, -0.1, 0.2)]
    frames = simulate(200, steps_of(levels, 15))
    header = {"schema": 2, "scenario": "v2_x", "seed": 1, "mode": "ideal"}
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", frames, header=header))
    q = mr.alignment(recs)["q0"]
    assert q["servo"] is None
    assert q["servo_excluded_by"]["ideal"] == 200 and q["servo_excluded_by"]["equal_states"] == 0
    assert q["pipeline"]["gain"] == pytest.approx(1.0, abs=0.01)


@pytest.mark.skipif(not mr.BINARY_PATH.exists(), reason="model_step is not built here (runs in the container)")
def test_report_keeps_runs_with_the_same_directory_name(tmp_path):
    # review F3: the same seed under two variants: two runs, keyed by their path under the logs' common root
    frames = simulate(100, steps_of(SLIP_LEVELS, 30))
    logs = [write_log(tmp_path / variant / "seed1" / "run.mmlog", frames) for variant in ("before", "after")]
    runs, _ = mr.report(logs, tmp_path / "report", container=None, qd_limits=[0.6] * 6)
    assert sorted(runs) == ["after/seed1", "before/seed1"]


def test_missing_field_counts_count_every_field(tmp_path):
    # review F6: a window missing both the gradient and the kink counts under both
    frames = diag_frames(20, [0.0, 0.0002] + [0.0] * 7, grad=[0.0, -1.0] + [0.0] * 7, ddepth=0.0002)
    for kind, h, b in frames:
        del h["diag_kink"]
        del h["diag_grad"]
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", frames))
    g = mr.gradient_projection(recs)
    assert g["skipped"]["grad"] == g["skipped"]["kink"] > 5


def test_gradient_projection_tells_a_lost_pair_from_a_new_one_and_sums_the_split(tmp_path):
    # review: a pair that loses its slack (no deepest pair at the window's end) is not a different pair; and the
    # per-window contributions are also summed (medians of parts need not add up to the median total)
    frames = diag_frames(30, [0.0, 0.0002] + [0.0] * 7, grad=[0.0, -1.0] + [0.0] * 7, ddepth=0.0002)
    for kind, h, b in frames[12:16]:
        h["diag_pair"], h["diag"] = "", [0.0, 0.0, None]
    for kind, h, b in frames[20:24]:
        h["diag_pair"] = "9|9|beam"
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", frames))
    g = mr.gradient_projection(recs, step=unicycle)
    assert g["pair_lost"] > 0 and g["pair_changed"] > 0
    sums = g["smooth"]["sums"]
    assert sums["b5"]["total"] == pytest.approx(sums["depth_change"], rel=0.05)
    assert sums["b5"]["lateral"] == pytest.approx(sums["b5"]["total"])


def test_twist_bins_leave_out_reversing_windows(tmp_path):
    # review: a window whose command changes sign has a small net mean that would land below the breakaway bins
    commands = [[0.2, 0.4 if (j // 3) % 2 else -0.4, 0, 0, 0, 0, 0, 0] for j in range(60)]
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", drive(commands)))
    result = mr.twist_bins(recs, unicycle)
    assert all(b["omega"][0] >= 0.2 for b in result["bins"]), result  # only the windows that did not reverse
    assert result["reversing"] > 10


# ---- Task 4: the open-loop excitation and the arm feedforward ---------------------------------------------------------

def test_excitation_segments_cover_the_breakaway_range():
    segments = mr.excitation_inputs()
    commanded = [(abs(c[1]), abs(c[2])) for s in segments for c in s["input"]["base_commands"]]
    vs, ws = {v for v, _ in commanded}, {w for _, w in commanded}
    assert any(0 < v < 0.005 for v in vs) and any(v > 0.005 for v in vs)  # around the plant's breakaway
    assert any(0 < w < 0.02 for w in ws) and any(w > 0.02 for w in ws)
    turns = [s for s in segments if s["name"].startswith("turn")]
    assert len(turns) == len(mr.EXCITATION_TURNS)
    for s in turns:  # straight before and after the turn
        cmds = s["input"]["base_commands"]
        assert cmds[0][2] == 0.0 and cmds[-1][2] == 0.0 and any(c[2] != 0.0 for c in cmds)
    assert {s["name"].split("_")[0] for s in segments} >= {"step", "ramp", "turn"}
    assert all(s["input"]["sample_period_s"] == 0.02 and s["input"]["world"] == "ground" for s in segments)


def test_excitation_records_follow_the_time_convention():
    # plant_replay's samples at 0, 0.02, ...; the command stamped t[j-1] drove the interval into record j
    seg = {"input": {"base_commands": [[0.0, 0.1, 0.0], [0.02, 0.2, 0.0], [0.04, 0.3, 0.5]], "start": {"q": [0.1] * 6}}}
    out = {"t": [0.0, 0.02, 0.04, 0.06], "base": [[0, 0, 0], [0.002, 0, 0], [0.006, 0, 0], [0.012, 0, 0.01]],
           "twist": [[0, 0], [0.1, 0], [0.2, 0], [0.3, 0.5]], "q": [[0.1] * 6] * 4, "q_ref": [[0.1] * 6] * 4}
    recs = mr.excitation_records(seg, out)
    assert [r["cmd"][0] for r in recs] == [0.0, 0.1, 0.2, 0.3]
    assert recs[3]["cmd"][1] == 0.5 and recs[3]["pose"][:3] == [0.012, 0, 0.01] and recs[3]["pose"][3:] == [0.1] * 6
    assert all(r["mode"] == "realistic" and r["goal"] == 0 and r["depth"] is None for r in recs)


def test_arm_tracking_summary():
    out = {"t": [0.0, 0.1, 0.2], "q": [[0.0] * 6, [0.01] + [0.0] * 5, [0.0] * 6],
           "q_ref": [[0.0] * 6, [0.0] * 6, [0.0] * 6]}
    s = mr.arm_tracking(out)
    assert s["max_mrad"][0] == pytest.approx(10.0) and s["max_mrad"][1] == 0.0
    assert s["rms_mrad"][0] == pytest.approx(10.0 / math.sqrt(3))


def test_same_run_ignores_only_the_diagnostics(tmp_path):
    # Task 4: a diagnostics rerun must reproduce its original's State and Event frames, the diagnostics aside
    plain = [state_frame(j * DT, [0.001 * j] + [0.0] * 8, [0.1] * 8, diagnostics=False) for j in range(5)]
    plain.append((ps.EVENT, {"t": 0.1, "event": "contact"}, []))
    diag = [state_frame(j * DT, [0.001 * j] + [0.0] * 8, [0.1] * 8, diag=(0.002, 0.001, None), pair="1|2|w")
            for j in range(5)] + [plain[-1]]
    a, b = write_log(tmp_path / "a.mmlog", plain), write_log(tmp_path / "b.mmlog", diag)
    assert mr.same_run(a, b) is None
    moved = list(diag)
    kind, h, blocks = moved[3]
    moved[3] = (kind, h, [[9.0] + blocks[0][1:]] + blocks[1:])
    c = write_log(tmp_path / "c.mmlog", moved)
    assert "State frame 3" in mr.same_run(a, c)
    fewer = write_log(tmp_path / "d.mmlog", diag[:4])
    assert "frames" in mr.same_run(a, fewer)


def test_same_run_ignores_the_tracking_estimate(tmp_path):
    # E2 Task 7: diag_track (the tracking model a cycle planned with) is a diagnostic like the others
    plain = [state_frame(j * DT, [0.001 * j] + [0.0] * 8, [0.1] * 8, diagnostics=False) for j in range(3)]
    diag = []
    for j in range(3):
        kind, h, blocks = state_frame(j * DT, [0.001 * j] + [0.0] * 8, [0.1] * 8, diag=(0.002, 0.001, None))
        h["diag_track"] = [0.98, 0.03, 0.2]
        diag.append((kind, h, blocks))
    a, b = write_log(tmp_path / "a.mmlog", plain), write_log(tmp_path / "b.mmlog", diag)
    assert mr.same_run(a, b) is None


def test_build_time_counts_ramped_transitions(tmp_path):
    # Task 4: closed-loop commands ramp (rate limits), they do not jump: a change of a quarter of the range within
    # 0.5 s is a transition, counted once per change
    def ramped(j):
        t = j * DT
        phase = t % 4.0  # 1 s up, 1 s turning, 1 s down, 1 s straight
        w = 0.4 * (min(1.0, phase) if phase < 2.0 else max(0.0, 3.0 - phase))
        return [0.3, w, 0, 0, 0, 0, 0, 0]
    frames = simulate(800, ramped, slip=0.05, slip_tau=0.3)
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", frames))
    bt = mr.build_time(recs)
    assert bt is not None and 6 <= bt["executed"]["transitions"] <= 10  # 4 cycles, each up and down
    assert bt["executed"]["tau"] == pytest.approx(0.3, abs=0.05)


# ---- Task 4 review: the term shares and the build-up profile -------------------------------------------------------

def test_term_shares_remove_a_known_turn_gain_and_arm_delay(tmp_path):
    # review F1: the CP1 share table as a tested function: a plant turning at half the command and an arm two cycles
    # late; the matching variants remove those errors, the baseline does not
    levels = [[0.2, w, q, 0, 0, 0, 0, 0] for w, q in ((0.3, 0.1), (0.1, -0.2), (0.4, 0.0), (0.0, 0.3))]
    cmd = steps_of(levels, 15)
    frames = simulate(300, cmd, gain_w=0.5)
    # the arm two cycles late: rewrite joint 0 from the command of record j-2
    q0 = 0.0
    for j, (kind, h, b) in enumerate(frames):
        if j:
            q0 += cmd(max(1, j - 2))[2] * DT if j > 2 else 0.0
        b[0][3] = q0
        b[1][3] = q0
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", frames))
    variants = {"baseline": {}, "turn gain": {"turn_gain": [(9.0, 0.5)]}, "arm delay": {"arm_delay_cycles": 2}}
    shares = mr.term_shares([recs], variants)
    base, turn, arm = (shares["all"][k] for k in ("baseline", "turn gain", "arm delay"))
    assert base["yaw"]["rms"] > 1e-3 and turn["yaw"]["rms"] < 1e-5
    assert base["arm_worst_joint"]["rms"] > 1e-4 and arm["arm_worst_joint"]["rms"] < 1e-6
    assert shares["windows"]["all"] > 0 and shares["definitions"]["arm_worst_joint"]
    assert set(shares) >= {"all", "turning", "windows", "definitions", "variants"}


def test_build_time_reports_its_tau_profile(tmp_path):
    # review F2: the build-up fit carries its sensitivity: the rms at fixed time constants and with no lateral model
    frames = simulate(600, steps_of(SLIP_LEVELS, 60), slip=0.05, slip_tau=0.3)
    _, recs = mr.records(write_log(tmp_path / "run.mmlog", frames))
    ex = mr.build_time(recs)["executed"]
    prof = ex["tau_profile"]
    assert set(prof) == {"0", "0.1", "0.2", "0.3"}
    assert prof["0.3"] < prof["0"] and prof["0.3"] <= prof["0.1"]
    assert ex["rms_zero_model"] > prof["0"]


@pytest.mark.skipif(not mr.BINARY_PATH.exists(), reason="model_step is not built here (runs in the container)")
def test_container_step_takes_the_tracking_map():
    # E2 Task 5: model_step with the five map parameters; all zeros is the identity, a turn delta slows the turn
    batch = [(0.1, [0.0] * 9, [0.0, 0.045] + [0.0] * 6)]
    with mr.container_step(container=None) as plain, mr.container_step(container=None, params=[0, 0, 0.05, 0, 0]) as zero, \
            mr.container_step(container=None, params=[0.0, -0.95, 0.025, 0.0, 0.0]) as turn:
        a, b, c = plain(batch)[0], zero(batch)[0], turn(batch)[0]
    assert a == b
    assert c[2] == pytest.approx(0.1 * (0.045 - 0.95 * 0.025 * math.tanh(1.8)), abs=1e-12)


@pytest.mark.skipif(not mr.BINARY_PATH.exists(), reason="model_step is not built here (runs in the container)")
def test_model_step_rejects_malformed_map_parameters():
    # Task 5 review F1: 18 numbers or 23, nothing in between and nothing malformed: a first map parameter that fails to
    # convert (consumed by the conversion) must not turn into an identity request
    import subprocess
    base = "0.1 " + " ".join(["0"] * 9) + " 0.3 0.4 " + " ".join(["0"] * 6)
    def run(line):
        p = subprocess.run(["bash", "-lc", f"exec {mr.BINARY_PATH}"], input=line + "\n", text=True, capture_output=True)
        return p.returncode
    assert run(base) == 0
    assert run(base + " 0 -0.95 0.025 0.17 0.2") == 0
    for suffix in ("+", ".", "1e", "1e999", "garbage", "0", "0 0", "0 0 0", "0 0 0 0", "0 0 0.05 0 0 0",
                   "0 -0.95 0.025 0.17 0.2 x", "0 -0.95 0.025 0.17 nan"):
        assert run(base + " " + suffix) == 2, suffix
