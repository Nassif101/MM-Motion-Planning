"""Tests of the variant judge (E1 plan Task 5): Fisher's exact test, A3's regression rules, A1's gate locks."""
import json
import struct
import zlib

import pytest

import compare_variants as cv
import plant_stream as ps


def test_fisher_matches_known_values():
    # one-sided: the variant's rate (2) lower than the baseline's (1)
    assert cv.fisher_greater(20, 20, 15, 20) == pytest.approx(0.02356, abs=1e-3)
    assert cv.fisher_greater(5, 20, 5, 20) > 0.5
    assert cv.fisher_greater(10, 20, 15, 20) == pytest.approx(1.0, abs=0.1)  # the variant is better


def summaries(root, tag, cell, kind, reached, contacts=0, clearance=0.1, n=20, mode="realistic"):
    for seed in range(1, n + 1):
        d = root / (f"{cell}-{kind}-{mode}" + (f"-{tag}" if tag else "") + f"-s{seed}")
        d.mkdir(parents=True)
        (d / "summary.json").write_text(json.dumps({
            "cell": cell, "kind": kind, "reached": seed <= reached, "plant_contacts": 5 if seed <= contacts else 0,
            "plant_min_clearance_m": clearance}))


def regressed(tmp_path, base, variant):
    summaries(tmp_path / "base", None, "c", "b5", **base)
    summaries(tmp_path / "var", "abs", "c", "b5", **variant)
    rows = cv.regressions(tmp_path / "base", tmp_path / "var", variant_tag="abs")
    assert len(rows) == 1 and rows[0]["cell"] == "c" and rows[0]["kind"] == "b5"
    return rows[0]["regressed"]


def test_regression_rules(tmp_path):
    assert regressed(tmp_path / "1", {"reached": 18}, {"reached": 12})
    assert not regressed(tmp_path / "2", {"reached": 18}, {"reached": 16})
    assert regressed(tmp_path / "3", {"reached": 20}, {"reached": 20, "contacts": 1})
    assert regressed(tmp_path / "4", {"reached": 20, "clearance": 0.100}, {"reached": 20, "clearance": 0.094})
    assert not regressed(tmp_path / "5", {"reached": 20, "clearance": 0.100}, {"reached": 20, "clearance": 0.096})


def test_regressions_pair_like_tags_only(tmp_path):
    # the grip variants: baseline tag mu0.4 against variant tag abs_mu0.4; twins (-n) and other tags are left out
    summaries(tmp_path / "base", "mu0.4", "c", "f2", reached=10)
    summaries(tmp_path / "base", None, "c", "f2", reached=20)
    summaries(tmp_path / "var", "abs_mu0.4", "c", "f2", reached=10)
    (tmp_path / "var" / "c-f2-realistic-abs_mu0.4-s1-n").mkdir()
    rows = cv.regressions(tmp_path / "base", tmp_path / "var", baseline_tag="mu0.4", variant_tag="abs_mu0.4")
    assert len(rows) == 1 and rows[0]["base_runs"] == 20 and rows[0]["variant_runs"] == 20
    assert not rows[0]["regressed"]


def log_with(path, frames):
    raw = ps.encode(ps.HELLO, {"schema": 2}, []) + b"".join(ps.encode(ps.STATE, h, [[0.0] * 9]) for h in frames)
    packed = zlib.compress(raw)
    path.mkdir(parents=True)
    (path / "run.mmlog").write_bytes(b"MMRUNLG1" + struct.pack("<II", len(raw), len(packed)) + packed)


def held(n, violation, slack=0.004):
    return [{"t": 0.02 * k, "status": "holding", "gate": [slack, violation, 1]} for k in range(n)]


def test_locks_separates_the_step_lock_from_deep_holds(tmp_path):
    moving = [{"t": 0.0, "status": "ok", "gate": [0.0, 0.0, 0]}] * 100
    log_with(tmp_path / "lock-s1", moving + held(500, 0.001))
    log_with(tmp_path / "deep-s1", moving + held(500, 0.02))
    log_with(tmp_path / "outside-s1", moving + held(500, 0.0))
    log_with(tmp_path / "moves-s1", held(400, 0.001) + moving)
    found = {r["run"]: r["kind"] for r in cv.locks(tmp_path)}
    assert found == {"lock-s1": "lock", "deep-s1": "deep", "outside-s1": "outside"}


def test_holds_without_gate_fields_are_not_classified(tmp_path):
    # the baseline replicas predate the State frames' gate field: such a hold is listed as "unknown", never "outside"
    log_with(tmp_path / "old-s1", [{"t": 0.02 * k, "status": "holding"} for k in range(500)])
    assert [(r["run"], r["kind"]) for r in cv.locks(tmp_path)] == [("old-s1", "unknown")]


def test_an_empty_comparison_is_no_pass(tmp_path):
    # Task 5 review: a wrong tag matched nothing and passed; no rows is exit 2, one-sided groups are listed
    summaries(tmp_path / "base", None, "c", "b5", reached=20)
    summaries(tmp_path / "base", None, "d", "b5", reached=20)
    summaries(tmp_path / "var", "abs", "c", "b5", reached=20)
    report = tmp_path / "r.md"
    assert cv.main(["regressions", str(tmp_path / "base"), str(tmp_path / "var"), "--tag", "nonexistent",
                    "--report", str(report)]) == 2
    assert cv.main(["regressions", str(tmp_path / "base"), str(tmp_path / "var"), "--tag", "abs",
                    "--report", str(report)]) == 0
    assert "d b5 realistic: baseline only" in report.read_text()


def test_runs_that_left_no_summary_are_flagged(tmp_path):
    summaries(tmp_path / "base", None, "c", "b5", reached=20)
    summaries(tmp_path / "var", "abs", "c", "b5", reached=19, n=19)
    (tmp_path / "var" / "c-b5-realistic-abs-s20").mkdir()  # a run that crashed: no summary
    rows = cv.regressions(tmp_path / "base", tmp_path / "var", variant_tag="abs")
    assert rows[0]["variant_missing"] == 1 and rows[0]["regressed"]
    assert any("without a summary" in reason for reason in rows[0]["reasons"])


def test_locks_name_a_passing_hold_and_skip_twins(tmp_path):
    # a hold whose gate passes (the resume dwell, solver failures while holding) is "passing", not "outside";
    # twins (-n) and other tags are left out
    log_with(tmp_path / "c-b5-realistic-s1", [{"t": 0.0, "status": "holding", "gate": [0.0, 0.0, 0]}] * 500)
    log_with(tmp_path / "c-b5-realistic-s1-n", held(500, 0.001))
    log_with(tmp_path / "c-b5-realistic-abs-s1", held(500, 0.001))
    assert [(r["run"], r["kind"]) for r in cv.locks(tmp_path)] == [("c-b5-realistic-s1", "passing")]
    assert [(r["run"], r["kind"]) for r in cv.locks(tmp_path, tag="abs")] == [("c-b5-realistic-abs-s1", "lock")]


def test_a_hold_asking_over_the_tolerance_is_no_step_lock(tmp_path):
    # Task 7: the step's lock fails a plan asking at most T at a pair less than T deep (absolute would pass it); a plan
    # asking more than T there is "over" (just over the absolute bound, or far more where both rules hold alike)
    log_with(tmp_path / "step-s1", held(500, 0.002, slack=0.0035))
    log_with(tmp_path / "bound-s1", held(500, 0.0045, slack=0.0053))
    assert {r["run"]: r["kind"] for r in cv.locks(tmp_path)} == {"step-s1": "lock", "bound-s1": "over"}
