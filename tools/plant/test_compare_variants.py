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


def held(n, violation):
    return [{"t": 0.02 * k, "status": "holding", "gate": [0.006, violation, 1]} for k in range(n)]


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
