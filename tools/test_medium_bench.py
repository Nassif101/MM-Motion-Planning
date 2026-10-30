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
