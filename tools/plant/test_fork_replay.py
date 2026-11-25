"""Tests of fork_replay.py (medium diagnosis Task 5: the replay matrix)."""
import json

import fork_replay as fr
import plant_stream as ps
from test_medium_diag import hello, plan, pose_state, write_log


def points(spec):
    """Fork points from (cell, seed, class, kind, t) tuples."""
    return [{"run": f"{c}-f2-realistic-m050-s{s}", "cell": c, "seed": s, "class": k, "kind": kind, "t": t,
             "stop_t": t + 1.0} for c, s, k, kind, t in spec]


def test_select_takes_five_per_cell_in_seed_order():
    pts = points([("a", s, "infeasible_b5_fallback", "infeasible", 10.0) for s in (12, 3, 7, 1, 20, 9, 4)]
                 + [("b", 2, "infeasible_gate_hold", "infeasible", 5.0), ("b", 5, "ok_plan_b5_creep", "stall_onset", 6.0)])
    got = fr.select(pts, {"infeasible_b5_fallback", "infeasible_gate_hold"}, per_cell=5)
    assert [(p["cell"], p["seed"]) for p in got] == [("a", 1), ("a", 3), ("a", 4), ("a", 7), ("a", 9), ("b", 2)]


def test_each_setting_changes_one_thing():
    for settings in (fr.ONE_CYCLE, fr.CLOSED_LOOP):
        tags = dict(settings)
        assert tags["control"] == {}
        for tag, setting in settings:
            if tag == "control":
                continue
            partner = fr.PAIRED.get(tag, "control")
            base = tags[partner] if partner in tags else dict(fr.ONE_CYCLE)[partner]
            diff = {k for k in set(setting) | set(base) if setting.get(k) != base.get(k)}
            assert len(diff) == 1, (tag, partner, diff)


def test_fork_runs_carry_the_original_arguments_and_the_fork(tmp_path):
    runs = tmp_path / "baseline"
    d = runs / "a-f2-realistic-m050-s3"
    d.mkdir(parents=True)
    (d / "summary.json").write_text(json.dumps({"args": ["--scenario", "a", "--mode", "realistic", "--seed", "3",
                                                         "--prediction", "--seed-detail"]}))
    pts = points([("a", 3, "infeasible_b5_fallback", "infeasible", 12.4)])
    [one] = fr.fork_runs(pts, [("m030", {"medium_arm_margin": "0.03"})], runs, cycles=1)
    assert one["name"] == "a-f2-realistic-m050-s3-m030-1c"
    a = one["args"]
    assert a[:6] == ["--scenario", "a", "--mode", "realistic", "--seed", "3"]
    assert a.count("--prediction") == 1 and a.count("--seed-detail") == 1 and "--diagnostics" in a
    assert a[a.index("--fork-at") + 1] == "12.4" and a[a.index("--fork-set") + 1] == "medium_arm_margin=0.03"
    assert a[a.index("--fork-cycles") + 1] == "1"
    [closed] = fr.fork_runs(pts, [("control", {})], runs, cycles=0)
    assert closed["name"].endswith("-control-cl") and "--fork-cycles" not in closed["args"]
    assert "--fork-set" not in closed["args"] and "--fork-at" in closed["args"]


def forked_log(path, t_fork, status, feasible, collision):
    rows = [["previous", 3.0, 1 if feasible else 0, 1.0, 0, collision, collision, 0.0, 0.0, 0.0, 15, 0, 0, "comfort",
             "panel" if collision > 0 else "", "brace" if collision > 0 else "", 0.05 - collision if collision > 0 else None,
             2.4 if collision > 0 else None, 0, {"progress": 1.0}]]
    frames = [hello(), pose_state(t_fork - 0.4, 1.0), plan(t_fork - 0.4, "infeasible"), pose_state(t_fork, 1.0),
              plan(t_fork, status, seeds=rows)]
    write_log(path, frames)


def write_fork(root, name, summary, log=None):
    d = root / name
    d.mkdir(parents=True)
    (d / "summary.json").write_text(json.dumps(summary))
    if log:
        log(d / "run.mmlog")
    return d


def test_closed_loop_only_after_a_feasible_fork(tmp_path):
    root = tmp_path / "forks"
    pt = points([("a", 3, "infeasible_b5_fallback", "infeasible", 12.4)])[0]
    for tag, status, feasible in (("m040", "infeasible", False), ("m030", "ok", True), ("control", "infeasible", False)):
        write_fork(root, f"{pt['run']}-{tag}-1c",
                   {"fork": {"applied_t": 12.4, "cycle_wall_s": [0.3], "deadline_cut_cycles": 0}, "end_status": "forked"},
                   lambda p, s=status, f=feasible: forked_log(p, 12.4, s, f, 0.0 if f else 0.004))
    results = fr.one_cycle_results([pt], [("control", {}), ("m040", {}), ("m030", {})], root)
    assert [(r["tag"], r["feasible"]) for r in results] == [("control", False), ("m040", False), ("m030", True)]
    assert results[1]["collision_m"] == 0.004 and results[1]["deepest"] == "panel/brace"
    follow = fr.closed_loop_followups(results)
    assert follow == {pt["run"]: ["m030"]}


def test_control_must_reproduce_the_original(tmp_path):
    original = {"reached": False, "time_s": 66.1, "min_clearance_m": 0.02, "plant_contacts": 0}
    same = dict(original, fork={"applied_t": 14.4, "cycle_wall_s": [], "deadline_cut_cycles": 0})
    other = dict(same, time_s=70.0)
    assert fr.reproduces(original, same)
    assert not fr.reproduces(original, other)


def test_deadline_cut_cycles_are_excluded(tmp_path):
    root = tmp_path / "forks"
    pt = points([("a", 3, "infeasible_b5_fallback", "infeasible", 12.4)])[0]
    write_fork(root, f"{pt['run']}-b300-1c",
               {"fork": {"applied_t": 12.4, "cycle_wall_s": [10.2], "deadline_cut_cycles": 1}, "end_status": "forked"},
               lambda p: forked_log(p, 12.4, "ok", True, 0.0))
    [r] = fr.one_cycle_results([pt], [("b300", {})], root)
    assert r["excluded"] == "deadline_cut" and r["feasible"] is None


def test_the_iteration_settings_raise_the_supply_only():
    # the iteration supply test changes only iteration keys, each above the default
    keys = {"iteration_budget", "max_iter", "waiting_iterations"}
    for tag, setting in fr.ITERATION[1:]:
        assert set(setting) <= keys and setting.get("iteration_budget") == "300", tag
