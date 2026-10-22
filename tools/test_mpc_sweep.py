"""Tests of the Phase 4B-B sweep driver's pure parts (overlays, shortlist rules)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mpc_sweep  # noqa: E402


def run(cell="c", p99=0.008, worst=0.015, failures=0, progress=1.0, contour=0.02, clearance=None, shortfall=False):
    per_cycle = {"true_clearance_m": clearance if clearance is not None else [0.1] * 100}
    return {"cell": cell, "timing": {"total_s": {"p50": 0.004, "p95": 0.006, "p99": p99, "max": worst}},
            "solver_failures": failures, "progress_median_mps": progress, "contour_mean_m": contour,
            "per_cycle": per_cycle, "flags": {"shortfall": shortfall, "timing_heavy": False}}


def variant(name, switches, **kwargs):
    # Both runs share the variant's progress and contour (the rules pool them); the second is the shortfall run.
    shared = {k: kwargs[k] for k in ("progress", "contour") if k in kwargs}
    return {"name": name, "switches": switches,
            "runs": [run(**kwargs), run(cell="s", shortfall=True, clearance=kwargs.get("clearance"), **shared)]}


FROZEN = [run(p99=0.06, worst=0.075), run(cell="s", shortfall=True, clearance=[0.04] * 100, p99=0.06, worst=0.075)]


def test_merge_overlay_nested():
    base = {"solver": {"backend": "osqp", "eps": 1e-3, "hpipm": {"mode": "balance"}}, "horizon": 20}
    merged = mpc_sweep.merge_overlay(base, {"solver": {"backend": "hpipm", "hpipm": {"partial_condensing_blocks": 4}}})
    assert merged == {"solver": {"backend": "hpipm", "eps": 1e-3,
                                 "hpipm": {"mode": "balance", "partial_condensing_blocks": 4}}, "horizon": 20}
    assert base["solver"]["backend"] == "osqp", "the base is not modified"
    assert mpc_sweep.count_switches({"solver": {"backend": "hpipm", "hpipm": {"partial_condensing_blocks": 4}},
                                     "qp_iterations": 2}) == 3


def test_shortlist_rule_order():
    # A has the best p99 but misses the clearance rule on the shortfall run; B and C pass with the same p99, C with
    # fewer switches.
    a = variant("a", 1, p99=0.004, clearance=[0.044] * 100)
    b = variant("b", 2, p99=0.008)
    c = variant("c", 1, p99=0.008)
    ranked = mpc_sweep.shortlist([a, b, c], FROZEN)
    assert [r["name"] for r in ranked] == ["c", "b"]
    assert all(r["failed"] == [] for r in ranked)


def test_shortlist_rules_each_fail():
    assert mpc_sweep.shortlist([variant("t", 1, p99=0.011)], FROZEN)[0]["failed"] == [3]
    assert mpc_sweep.shortlist([variant("w", 1, worst=0.021)], FROZEN)[0]["failed"] == [3]
    assert mpc_sweep.shortlist([variant("f", 1, failures=1)], FROZEN)[0]["failed"] == [1]
    assert mpc_sweep.shortlist([variant("p", 1, progress=0.94)], FROZEN)[0]["failed"] == [4]
    assert mpc_sweep.shortlist([variant("k", 1, contour=0.0221)], FROZEN)[0]["failed"] == [4]
    assert mpc_sweep.shortlist([variant("x", 1)], FROZEN, fidelity_ok=False)[0]["failed"] == [1]
    # 99 % of the shortfall run's cycles at 0.045 m or more passes; 98 % does not.
    ok = [0.045] * 99 + [0.03]
    assert mpc_sweep.shortlist([variant("o", 1, clearance=ok)], FROZEN)[0]["failed"] == []
    short = [0.045] * 98 + [0.03] * 2
    assert mpc_sweep.shortlist([variant("s", 1, clearance=short)], FROZEN)[0]["failed"] == [2]


def test_shortlist_none_passing_returns_closest_with_failed_rules():
    one = variant("one", 1, p99=0.012)
    two = variant("two", 1, p99=0.012, failures=3)
    ranked = mpc_sweep.shortlist([two, one], FROZEN)
    assert [r["name"] for r in ranked] == ["one", "two"]
    assert ranked[0]["failed"] == [3] and ranked[1]["failed"] == [1, 3]


def test_shortlist_p99_ties_within_a_quarter_millisecond_go_to_fewer_switches():
    ranked = mpc_sweep.shortlist([variant("many", 3, p99=0.0080), variant("few", 1, p99=0.0082)], FROZEN)
    assert [r["name"] for r in ranked] == ["few", "many"]
    ranked = mpc_sweep.shortlist([variant("many", 3, p99=0.0070), variant("few", 1, p99=0.0090)], FROZEN)
    assert [r["name"] for r in ranked] == ["many", "few"]


def test_shortlist_survives_runs_without_a_plan_quality_number():
    v = variant("v", 1)
    v["runs"][0]["progress_median_mps"] = None
    v["runs"][0]["contour_mean_m"] = None
    assert mpc_sweep.shortlist([v], FROZEN)[0]["failed"] == []


def test_grid_order_interleaves_variants_per_run():
    order = mpc_sweep.grid_order(["r1", "r2"], ["a", "b"])
    assert order == [("r1", "a"), ("r1", "b"), ("r2", "a"), ("r2", "b")]


def test_summarize_pools_a_variants_runs():
    v = variant("v", 2, p99=0.006, worst=0.012)
    v["runs"][0]["iterations"] = {"p50": 25, "p99": 150, "max": 300}
    v["runs"][1]["iterations"] = {"p50": 20, "p99": 100, "max": 500}
    for r in v["runs"]:
        r.update({"holding_cycles": 3, "misses": 1, "cycles": 100})
    row = mpc_sweep.summarize(v, FROZEN)
    assert row["name"] == "v" and row["switches"] == 2
    assert row["p99_max_ms"] == 8.0 and row["max_ms"] == 15.0  # the second run keeps the helper's 8 and 15 ms
    assert row["iterations_max"] == 500 and row["solver_failures"] == 0 and row["holds"] == 6
    assert row["shortfall_share"] == 1.0  # the shortfall run's clearance is 0.1 everywhere
    assert row["failed"] == []
    table = mpc_sweep.markdown_table([row])
    assert table.splitlines()[0].startswith("| variant |") and "| v |" in table
