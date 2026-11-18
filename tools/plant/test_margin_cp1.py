"""Tests of CP1's tables (E3 plan Task 5): fabricated records."""
import random

import pytest

import margin_cp1 as cp


def record(run, e, kind="panel", la=0.5, regime="arm", history=True, e_true=None, **f):
    return {"run": run, "pair": "12|13|brace_left", "type": kind, "lookahead_s": la, "regime": regime,
            "features": {"v2": f.get("v2", 0.1), "w2": f.get("w2", 0.2), "qd2": f.get("qd2", 0.3)},
            "history": {"v2": 0.4, "w2": 0.5, "qd2": 0.6} if history else None,
            "e_track": e, "e_track_true": e if e_true is None else e_true, "e_state": 0.001, "e_lin": 0.0002,
            "e_total": 0.0003}


def test_compact_and_expand_keep_what_the_analysis_reads():
    r = record("a", 0.004, e_true=0.001)
    back = cp.expand("a", cp.compact(r))
    for key in ("run", "type", "lookahead_s", "regime", "features", "history", "e_track", "e_track_true", "e_state",
                "e_lin", "e_total"):
        assert back[key] == r[key], key
    assert cp.expand("a", cp.compact(record("a", 0.004, history=False)))["history"] is None


def test_per_run_summary_is_per_run_first_and_needs_rows_and_runs():
    # six runs of 30 rows with a per-run p95 of 1..6 mm; a seventh with 5 rows of 1 m must not count
    recs = [record(f"r{k}", 0.001 * k) for k in range(1, 7) for _ in range(30)] + [record("thin", 1.0)] * 5
    s = cp.per_run_summary(recs, "e_track")
    assert (s["n"], s["median"], s["min"], s["max"]) == (6, pytest.approx(0.0035), pytest.approx(0.001), pytest.approx(0.006))
    assert cp.per_run_summary(recs[:4 * 30], "e_track") is None          # four runs: unavailable
    by = cp.per_run_summary(recs, "e_track", by_regime=True)
    assert set(by) == {"arm"} and by["arm"]["n"] == 6


def data_of(n_runs=40, rows=300):
    rng = random.Random(3)
    data = {"tag": "m050", "runs": {}, "rows": {}, "dropped": {"not executed|5": 7, "first frame|1": 2}}
    for k in range(n_runs):
        run = f"e3_gap_{'straight' if k % 2 else 'turn'}_1-f2-realistic-m050-s{k}"
        data["runs"][run] = {"reached": k % 4 != 0, "geometry": "gap", "motion": "straight" if k % 2 else "turn"}
        recs = []
        for _ in range(rows):
            w2 = rng.uniform(0, 1)
            for la in cp.LOOKAHEADS:
                recs.append(record(run, la * (0.004 + 0.04 * w2 + rng.gauss(0, 0.001)), la=la, w2=w2, e_true=0.0005))
        data["rows"][run] = [cp.compact(r) for r in recs]
    return data


def test_run_sets_split_the_runs_by_outcome_and_motion():
    sets = cp.run_sets(data_of(n_runs=8, rows=1))
    assert len(sets["reached, all cells"]) == 6 and len(sets["did not reach"]) == 2
    assert len(sets["reached, straight cells"]) == 4 and len(sets["reached, turn cells"]) == 2
    assert sets["reached, gap"] == sets["reached, all cells"] and not sets["reached, beam"]


def test_the_report_states_size_predictability_and_the_denominators():
    text = cp.markdown(data_of())
    assert "40 runs, 30 reached" in text
    # e_track = la x (4 mm + 40 mm x w2): the per-run p95 is about 21 mm at 0.5 s and 4 mm at 0.1 s
    assert "Size holds (median over the runs that reached at least 5 mm) at: panel 0.2 s, panel 0.5 s, panel 1 s." in text
    row = next(l for l in text.splitlines() if l.startswith("| panel | 0.5 | e_track |"))
    assert "| yes |" in row                                              # a saving above 30 % on a w2-driven error
    flat = next(l for l in text.splitlines() if l.startswith("| panel | 0.5 | e_track_true |"))
    assert "| no |" in flat                                              # a constant error: nothing to save
    assert "| arm | 0.5 | e_track | - | - | - | - | - | unavailable | - | - | - |" in text
    assert "| not executed | 5 | 7 |" in text
    assert "| did not reach | panel | 0.5 |" in text


def test_the_decision_sections_use_only_the_runs_that_reached():
    # failed runs carry 1 m errors and their own feature range, and runs have unequal lengths: any of them leaking into
    # Regimes, Predictability, History or Linearization shows as a number of hundreds of mm
    data = data_of(n_runs=40, rows=300)
    for k, (run, meta) in enumerate(data["runs"].items()):
        if not meta["reached"]:
            data["rows"][run] = [cp.compact(record(run, 1.0, la=la, e_true=1.0, w2=5.0) | {"e_state": 1.0, "e_lin": 1.0,
                                                                                         "e_total": 1.0})
                                 for la in cp.LOOKAHEADS for _ in range(50 + 10 * k)]
    text = cp.markdown(data)
    decision = text[text.index("## Regimes (descriptive)"):text.index("## Rows left out")]
    assert "1000.0" not in decision
    size = text[:text.index("## Regimes (descriptive)")]
    assert "| did not reach | panel | 0.5 | 1000.0 |" in size        # reported apart, in Size only
