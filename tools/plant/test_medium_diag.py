"""Tests of medium_diag.py (the medium-planner diagnosis, plan docs/superpowers/plans/2026-11-22-medium-diagnosis.md)."""
import json
import struct
import zlib

import pytest

import medium_diag as md
import plant_stream as ps


def write_log(path, frames, split=None):
    """A run log of encoded frames, in two chunks when `split` (the index of the second chunk's first frame)."""
    parts = [frames] if split is None else [frames[:split], frames[split:]]
    data = b"MMRUNLG1"
    for part in parts:
        raw = b"".join(part)
        packed = zlib.compress(raw)
        data += struct.pack("<II", len(raw), len(packed)) + packed
    path.write_bytes(data)
    return path


def hello(**extra):
    return ps.encode(ps.HELLO, {"schema": 3, **extra}, [])


def state(t, x=0.0):
    return ps.encode(ps.STATE, {"t": t}, [[x, 0.0, 0.0]])


def test_fingerprint_ignores_the_hello_only(tmp_path):
    body = [state(0.02), state(0.04, 0.1), state(0.06, 0.2)]
    a = write_log(tmp_path / "a.mmlog", [hello(provenance={"git_commit": "aaa"})] + body)
    b = write_log(tmp_path / "b.mmlog", [hello(provenance={"git_commit": "bbb"}, config={"seed_detail": False})] + body,
                  split=2)
    c = write_log(tmp_path / "c.mmlog", [hello()] + body[:2] + [state(0.06, 0.3)])
    assert md.log_fingerprint(a) == md.log_fingerprint(b)  # another Hello, another chunking: the same frames
    assert md.log_fingerprint(a) != md.log_fingerprint(c)  # one frame changed


# --- Task 2: cycles, states, stall onset, fork point, classes, report ---

def plan(t, status="ok", seeds=None, seed="previous", waiting=False):
    rows = seeds if seeds is not None else [["previous", 1.0, 1 if status == "ok" else 0, 1.0, 0]]
    return ps.encode(ps.PLANS, {"t": t, "status": status, "seed": seed, "full_multistart": False, "waiting": waiting,
                                "seeds": rows}, [[], []])


def pose_state(t, x, status="ok", fallback="", gate=(0.0, 0.0, 0), q0=0.0):
    return ps.encode(ps.STATE, {"t": t, "status": status, "fallback": fallback, "gate": list(gate)},
                     [[x, 0.0, 0.0, q0] + [0.0] * 5, [0.0] * 9, [0.0] * 8])


def run_frames(t_end, x_of, plans):
    """States every 0.1 s to t_end with x = x_of(t); `plans` are (t, status) pairs."""
    frames, k = [hello()], 0
    while k * 0.1 <= t_end + 1e-9:
        t = round(k * 0.1, 6)
        frames.append(pose_state(t, x_of(t)))
        frames += [plan(pt, st) for pt, st in plans if abs(pt - t) < 1e-9]
        k += 1
    return frames


def moving_until(t_stop):
    return lambda t: 0.5 * min(t, t_stop)  # 0.5 m/s, then still


def plans_every(t0, t1, status):
    return [(round(t0 + 0.4 * i, 6), status) for i in range(int(round((t1 - t0) / 0.4)) + 1)]


def load_frames(tmp_path, frames, name="run.mmlog"):
    return md.frames_of(write_log(tmp_path / name, frames))


def test_persistent_infeasibility_is_the_fork(tmp_path):
    frames = load_frames(tmp_path, run_frames(30.0, moving_until(12.4), plans_every(0.0, 11.6, "ok") + plans_every(12.0, 30.0, "infeasible")))
    fork = md.fork_point(md.cycles(frames), md.states(frames), reached=False)
    assert fork == {"kind": "infeasible", "t": 12.0}


def test_a_late_recovery_is_not_persistent(tmp_path):
    plans = plans_every(0.0, 11.6, "ok") + plans_every(12.0, 14.0, "infeasible") + [(14.4, "ok"), (14.8, "ok"),
             (15.2, "ok"), (15.6, "ok")] + plans_every(16.0, 30.0, "infeasible")
    frames = load_frames(tmp_path, run_frames(30.0, moving_until(16.0), plans))
    assert md.fork_point(md.cycles(frames), md.states(frames), reached=False) == {"kind": "infeasible", "t": 16.0}


def test_no_infeasibility_forks_at_the_stall_onset(tmp_path):
    frames = load_frames(tmp_path, run_frames(60.0, moving_until(20.0), plans_every(0.0, 60.0, "ok")))
    onset = md.stall_onset(md.states(frames))
    assert onset == pytest.approx(19.9, abs=0.11)  # the last 0.05 m step before the robot stops at x 10
    assert md.fork_point(md.cycles(frames), md.states(frames), reached=False) == {"kind": "stall_onset", "t": onset}


def test_an_infeasible_streak_long_after_the_stall_is_not_the_cause(tmp_path):
    # the robot stops at 20 s on ok plans; F2 goes infeasible only at 30 s: the stall's onset is the fork
    frames = load_frames(tmp_path, run_frames(60.0, moving_until(20.0), plans_every(0.0, 29.6, "ok") + plans_every(30.0, 60.0, "infeasible")))
    assert md.fork_point(md.cycles(frames), md.states(frames), reached=False)["kind"] == "stall_onset"


def test_reached_run_has_no_fork(tmp_path):
    frames = load_frames(tmp_path, run_frames(20.0, moving_until(20.0), plans_every(0.0, 20.0, "ok")))
    assert md.fork_point(md.cycles(frames), md.states(frames), reached=True) is None


def test_short_and_detailed_rows_parse(tmp_path):
    detailed = ["fold", 2.0, 0, 0.5, 0, 0.03, 0.03, 0.0, 0.0, 0.0, 15, 0, 1, "left", "panel", "brace", 0.02, 1.28, 1]
    frames = load_frames(tmp_path, [hello(), plan(0.4, "infeasible", seeds=[["previous", 1.0, 0, 1.0, 0], detailed])])
    seeds = md.cycles(frames)[0]["seeds"]
    assert seeds[0]["name"] == "previous" and seeds[0]["collision_m"] is None and seeds[0]["interrupted"] is None
    assert seeds[1] == {"name": "fold", "merit": 2.0, "feasible": False, "progress_m": 0.5, "merged": False,
                        "slack": 0.03, "collision_m": 0.03, "torque": 0.0, "speed_mps": 0.0, "hull": 0.0,
                        "iterations": 15, "converged": False, "resumed": True, "homotopy": "left",
                        "deepest_link": "panel", "deepest_obstacle": "brace", "deepest_distance": 0.02,
                        "deepest_t": 1.28, "interrupted": True}


def write_run(root, name, frames, summary):
    d = root / name
    d.mkdir(parents=True)
    write_log(d / "run.mmlog", frames)
    (d / "summary.json").write_text(json.dumps(summary))
    return d


def test_unreadable_and_b5_only_logs_are_classified(tmp_path):
    runs = tmp_path / "runs"
    write_run(runs, "c1-f2-realistic-s1", run_frames(20.0, moving_until(20.0), plans_every(0.0, 20.0, "ok")),
              {"cell": "c1", "seed": 1, "reached": True})
    bad = write_run(runs, "c1-f2-realistic-s2", run_frames(20.0, moving_until(5.0), []), {"cell": "c1", "seed": 2, "reached": False})
    (bad / "run.mmlog").write_bytes(b"MMRUNLG1" + b"\x00" * 3)  # no whole chunk: no Hello
    write_run(runs, "c1-b5-realistic-s3", run_frames(30.0, moving_until(5.0), []), {"cell": "c1", "seed": 3, "reached": False})
    records = md.diagnose_runs(runs)
    assert {r["run"]: r["class"] for r in records} == {"c1-f2-realistic-s1": "reached", "c1-f2-realistic-s2": "unreadable",
                                                       "c1-b5-realistic-s3": "no_medium"}


def stalled_run(stop, plan_status, b5_status="ok", fallback="", t_end=60.0, plan_waiting=False):
    """The base moves at 0.5 m/s until `stop`; afterwards B5 reports `b5_status`/`fallback`; F2's plans before the stop
    are ok and from 1.0 s before it `plan_status` (with `plan_waiting`)."""
    frames = [hello()]
    for k in range(int(round(t_end / 0.1)) + 1):
        t = round(k * 0.1, 6)
        after = t > stop
        frames.append(pose_state(t, 0.5 * min(t, stop), status=b5_status if after else "ok",
                                 fallback=fallback if after else "", gate=(0.01, 0.0, 1) if after and b5_status == "holding" else (0.0, 0.0, 0)))
        if k % 4 == 0:
            st = plan_status if t >= stop - 1.0 else "ok"
            frames.append(plan(t, st, waiting=plan_waiting and t >= stop - 1.0))
    return frames


def test_classes_of_stalls(tmp_path):
    # Task 2 review I2: the classes name the layer that stopped (F2 infeasible with B5's gate holding or on its
    # fallback; F2 ok with B5 held, B5 creeping, or F2 waiting); B5's "infeasible" fallback is not a hold
    root = tmp_path / "runs"
    cases = {1: stalled_run(20.0, "infeasible", "holding"), 2: stalled_run(20.0, "infeasible", "ok", "infeasible"),
             3: stalled_run(20.0, "ok", "holding"), 4: stalled_run(20.0, "ok", "ok"),
             5: stalled_run(20.0, "ok", "ok", plan_waiting=True), 6: stalled_run(58.0, "ok", "ok"),
             7: stalled_run(20.0, "ok", "ok", "infeasible")}
    for seed, frames in cases.items():
        write_run(root, f"c-f2-realistic-s{seed}", frames, {"cell": "c", "seed": seed, "reached": False})
    classes = {r["seed"]: r["class"] for r in md.diagnose_runs(root)}
    assert classes == {1: "infeasible_gate_hold", 2: "infeasible_b5_fallback", 3: "ok_plan_b5_held",
                       4: "ok_plan_b5_creep", 5: "f2_waiting", 6: "timeout_moving", 7: "ok_plan_b5_creep"}


def test_arm_drift_is_not_progress(tmp_path):
    # Task 2 review C1: the base parks at 20 s while a joint drifts 0.06 rad every 5 s to the end: the robot stopped at
    # 20 s (not at the last drift), and the run did not time out moving
    frames = [hello()]
    for k in range(601):
        t = round(k * 0.1, 6)
        frames.append(pose_state(t, 0.5 * min(t, 20.0), q0=0.06 * int(t // 5.0)))
        if k % 4 == 0:
            frames.append(plan(t, "ok"))
    frames = load_frames(tmp_path, frames)
    assert md.stall_onset(md.states(frames)) == pytest.approx(19.9, abs=0.11)
    assert md.classify(md.cycles(frames), md.states(frames), md.fork_point(md.cycles(frames), md.states(frames), False)) \
        == "ok_plan_b5_creep"


def test_the_streak_counts_until_the_stop(tmp_path):
    # Task 2 review I1: F2 infeasible from 12 s to the stop (12.4 s), then ok and waiting cycles in the 45 s after it:
    # the streak runs to the stop, so the fork is 12.0 s
    plans = plans_every(0.0, 11.6, "ok") + plans_every(12.0, 12.4, "infeasible") + plans_every(12.8, 60.0, "ok")
    frames = load_frames(tmp_path, run_frames(60.0, moving_until(12.4), plans))
    assert md.fork_point(md.cycles(frames), md.states(frames), reached=False) == {"kind": "infeasible", "t": 12.0}


def test_bad_or_missing_summaries_are_reported_not_fatal(tmp_path):
    # Task 2 review M3: a bad summary, a directory without one, a summary without `reached` and a log without States
    root = tmp_path / "runs"
    write_run(root, "c-f2-realistic-s1", run_frames(20.0, moving_until(20.0), plans_every(0.0, 20.0, "ok")),
              {"cell": "c", "seed": 1, "reached": True})
    (write_run(root, "c-f2-realistic-s2", run_frames(5.0, moving_until(5.0), []), {}) / "summary.json").write_text("{")
    (root / "c-f2-realistic-s3").mkdir()
    write_run(root, "c-f2-realistic-s4", run_frames(20.0, moving_until(5.0), plans_every(0.0, 20.0, "ok")), {"cell": "c", "seed": 4})
    write_run(root, "c-f2-realistic-s5", [hello(), plan(0.4, "ok")], {"cell": "c", "seed": 5, "reached": False})
    classes = {r["run"]: r["class"] for r in md.diagnose_runs(root)}
    assert classes == {"c-f2-realistic-s1": "reached", "c-f2-realistic-s2": "unreadable", "c-f2-realistic-s3": "unreadable",
                       "c-f2-realistic-s4": "unreadable", "c-f2-realistic-s5": "unreadable"}


def test_report_counts_per_cell(tmp_path):
    root = tmp_path / "runs"
    write_run(root, "a-f2-realistic-s1", run_frames(20.0, moving_until(20.0), plans_every(0.0, 20.0, "ok")),
              {"cell": "a", "seed": 1, "reached": True})
    write_run(root, "a-f2-realistic-s2", run_frames(60.0, moving_until(12.4), plans_every(0.0, 11.6, "ok") + plans_every(12.0, 60.0, "infeasible")),
              {"cell": "a", "seed": 2, "reached": False})
    out = tmp_path / "out"
    md.write_report(md.diagnose_runs(root), out)
    text = (out / "report.md").read_text()
    assert "| a | 2 | 1 |" in text and "infeasible_b5_fallback 1" in text
    points = json.loads((out / "fork_points.json").read_text())
    # review M5: where the robot stopped, when, and B5's state there
    assert points == [{"run": "a-f2-realistic-s2", "cell": "a", "seed": 2, "kind": "infeasible", "t": 12.0,
                       "class": "infeasible_b5_fallback", "stop_t": pytest.approx(12.3, abs=0.11),
                       "stop_x": pytest.approx(6.15, abs=0.11), "stop_y": 0.0, "b5_status": "ok", "b5_fallback": ""}]


# --- Task 3: the seed table and the layer view ---

def test_seed_table_lists_only_seeds_tried(tmp_path):
    rows = [["previous", 1.0, 1, 1.0, 0], ["fold", 3.0, 0, 0.4, 0, 0.02, 0.02, 0.0, 0.0, 0.0, 15, 0, 0, "lowered",
                                            "panel", "beam", 0.03, 1.6, 0]]
    frames = load_frames(tmp_path, [hello(), plan(0.4, "ok", seeds=rows), plan(0.8, "ok"), plan(1.2, "ok")])
    table = md.seed_table(md.cycles(frames), 0.0, 1.0)
    assert [r["t"] for r in table] == [0.4, 0.8]
    assert table[0]["winner"] == "previous"
    assert set(table[0]["seeds"]) == {"previous", "fold"} and "fold_back" not in table[0]["seeds"]
    assert table[0]["seeds"]["fold"]["deepest_obstacle"] == "beam" and table[0]["seeds"]["fold"]["feasible"] is False


def test_layer_at_reports_b5_holding_on_a_feasible_plan(tmp_path):
    frames = [hello(), plan(10.0, "ok")]
    frames += [pose_state(10.0 + 0.1 * k, 5.0, status="holding", gate=(0.02, 0.01, 1)) for k in range(5)]
    frames = load_frames(tmp_path, frames)
    view = md.layer_at(md.cycles(frames), md.states(frames), 10.3)
    assert view == {"t": 10.3, "medium": "ok", "plan_feasible": True, "winner": "previous", "b5_status": "holding",
                    "fallback": "", "gate": [0.02, 0.01, 1], "x": 5.0, "y": 0.0}
