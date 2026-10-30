import json

import pytest

import medium_bench as mb


def test_corpus_paths_relative_and_present(tmp_path):
    good = {"cells": [{"cell": "a", "scenario": "s", "bag": "experiment_runs/x/bag"},
                      {"cell": "b", "scenario": "t", "synthetic": True}]}
    path = tmp_path / "c.json"
    path.write_text(json.dumps(good))
    assert len(mb.load_corpus(path)["cells"]) == 2
    for bad in ({"cell": "a", "scenario": "s", "bag": "/abs/bag"}, {"cell": "a", "scenario": "s", "bag": "../x/bag"},
                {"cell": "b", "scenario": "t", "synthetic": True, "bag": "experiment_runs/x"}):
        path.write_text(json.dumps({"cells": [bad]}))
        with pytest.raises(ValueError):
            mb.load_corpus(path)


def test_the_phase5a_corpus_loads():
    corpus = mb.load_corpus(mb.ROOT / "docs/experiments/medium-horizon/phase5a/corpus.json")
    cells = {c["cell"] for c in corpus["cells"]}
    assert {"v2_room_r4_nav", "v2_store_room_nav", "v2_gate_right_swing_nav", "v2_door_105_nav", "v2_low_beam_123_nav",
            "v2_low_beam_130_nav", "v2_door_then_beam_nav"} <= cells


def test_summary_statistics():
    values = [0.01 * i for i in range(1, 101)]
    s = mb.statistics(values)
    assert s["median"] == pytest.approx(0.50)
    assert s["p90"] == pytest.approx(0.90)
    assert s["p99"] == pytest.approx(0.99)
    assert s["max"] == pytest.approx(1.00)
    assert s["mean"] == pytest.approx(0.505)
    assert mb.statistics([]) is None


def record(total, converged=6, seeds=6, status="ok", capped=False):
    return {"total_s": total, "linearize_s": 0.7 * total, "qp_s": 0.2 * total, "seeds": seeds,
            "converged_seeds": converged, "feasible_seeds": seeds, "status": status, "iterations": 10, "capped": capped}


def test_summarize_records_shares_and_rates():
    row = mb.summarize_records([record(1.0, converged=3), record(1.0, converged=6, status="infeasible")])
    assert row["linearize_share"] == pytest.approx(0.7)
    assert row["qp_share"] == pytest.approx(0.2)
    assert row["convergence"] == pytest.approx(0.75)
    assert row["ok"] == pytest.approx(0.5)
    assert mb.summarize_records([record(1.0, capped=True), record(1.0)])["capped"] == pytest.approx(0.5)


def test_choose_backend_rule():
    def row(p99, convergence):
        return {"total_s": {"p99": p99}, "convergence": convergence}
    # the fastest is not eligible when its convergence trails the best by more than 1 %
    rows = {"hpipm_1e-6": row(0.30, 0.90), "osqp_1e-6": row(0.10, 0.80), "daqp_1e-6": row(0.20, 0.895)}
    assert mb.choose_backend(rows) == "daqp_1e-6"
    rows["osqp_1e-6"] = row(0.10, 0.895)
    assert mb.choose_backend(rows) == "osqp_1e-6"
    assert mb.choose_backend({}) is None


def test_choose_backend_ignores_variants_at_the_cap():
    # a variant at the bench cap finished fewer seeds, so its convergence share is not comparable: it is out
    def row(p99, convergence, capped):
        return {"total_s": {"p99": p99}, "convergence": convergence, "capped": capped}
    rows = {"hpipm_1e-6": row(1.36, 0.24, 0.0), "osqp_1e-6": row(11.0, 0.31, 0.9)}
    assert mb.choose_backend(rows) == "hpipm_1e-6"


def test_variants_cover_the_backends_and_factors():
    names = set(mb.VARIANTS)
    for backend in ("hpipm", "osqp", "daqp"):
        assert {f"{backend}_1e-6", f"{backend}_1e-8"} <= names
    assert {"seg6", "seg8", "h6", "h10", "dt015", "joints6", "torque04"} <= names


def test_progressing_counts_ok_plans_that_move():
    # ok and at least half the reference reach (or half the route left, near the goal): a feasible plan that waits in
    # front of a tight place is not a good plan (Task 9 CP: slower seeds bought "ok" plans that barely moved)
    def rec(status, length, remaining):
        r = record(1.0, status=status)
        r.update(plan_length_m=length, remaining_m=remaining)
        return r
    rows = [rec("ok", 1.5, 10.0), rec("ok", 0.2, 10.0), rec("ok", 0.3, 0.5), rec("infeasible", 1.9, 10.0),
            rec("ok", 0.0, 0.05)]
    row = mb.summarize_records(rows)
    assert row["progressing"] == pytest.approx(3 / 5)
    assert mb.progressing(rec("ok", 0.97, 10.0), reach_m=1.92)
    assert not mb.progressing(rec("ok", 0.97, 10.0), reach_m=2.0)


def closed(cell, **fields):
    record = {"cell": cell, "reached": True, "executed_slack_plans": 0, "medium_solve_s": [0.05] * 100,
              "homotopy_switches_after_first": 0}
    record.update(fields)
    return record


def test_gate_rule():
    # The kinematic gate (plan Global Constraints, spec Section 6.2): every cell reaches its goal, no executed plan with
    # collision slack, medium p99 <= 150 ms over every cycle, no swing-side switch after the first choice in the
    # swing-side cells; each violation fails the gate and names its cell.
    good = [closed(c) for c in ("v2_room_r4_nav", "v2_gate_right_swing_nav", "v2_door_then_beam_nav")]
    ok, failures = mb.gate(good)
    assert ok and failures == []
    for bad, why in ((closed("v2_room_r4_nav", reached=False), "goal"),
                     (closed("v2_room_r4_nav", executed_slack_plans=2), "slack"),
                     (closed("v2_room_r4_nav", medium_solve_s=[0.05] * 98 + [0.2] * 2), "p99"),
                     (closed("v2_gate_right_swing_nav", homotopy_switches_after_first=1), "switch")):
        ok, failures = mb.gate([bad] + good[1:] if bad["cell"] == "v2_room_r4_nav" else [good[0], bad, good[2]])
        assert not ok and any(why in f and bad["cell"] in f for f in failures), (why, failures)
    # a switch in a cell without a swing-side choice is not the gate's
    ok, _ = mb.gate([closed("v2_room_r4_nav", homotopy_switches_after_first=1)] + good[1:])
    assert ok
    # a cell missing from the run fails
    ok, failures = mb.gate(good[:2], cells=[c["cell"] for c in good])
    assert not ok and any("v2_door_then_beam_nav" in f and "missing" in f for f in failures)
