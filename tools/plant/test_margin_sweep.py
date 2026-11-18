"""Tests of the fixed-margin sweep's tables (E3 plan Task 4): fabricated run summaries."""
import json

import pytest

import margin_sweep as ms


def write_runs(root, cell, tag, outcomes, mode="realistic"):
    """One run directory per outcome (reached, contacts, min clearance m), seeds from 1."""
    for seed, (reached, contacts, clearance) in enumerate(outcomes, start=1):
        d = root / f"{cell}-f2-{mode}-{tag}-s{seed}"
        d.mkdir(parents=True)
        (d / "summary.json").write_text(json.dumps({"cell": cell, "kind": "f2", "seed": seed, "reached": reached,
                                                    "plant_contacts": contacts, "plant_min_clearance_m": clearance}))


def cell_row(table, cell, tag):
    return next(r for r in table if r["cell"] == cell and r["tag"] == tag)


def test_table_counts_reached_contacts_and_clearance(tmp_path):
    write_runs(tmp_path, "e3_gap_straight_1", "m050", [(True, 0, 0.043), (True, 0, 0.041), (False, 0, 0.060)])
    row = cell_row(ms.sweep_table(tmp_path), "e3_gap_straight_1", "m050")
    assert (row["n"], row["reached"], row["contacts"], row["margin"], row["mode"]) == (3, 2, 0, 0.05, "realistic")
    assert row["min_clearance"] == pytest.approx(0.041)
    assert row["safe"] is True


def test_safe_needs_zero_contacts_and_clearance(tmp_path):
    write_runs(tmp_path, "a", "m030", [(True, 0, 0.020), (True, 0, 0.031)])   # exactly at the floor: safe
    write_runs(tmp_path, "b", "m030", [(True, 0, 0.019), (True, 0, 0.031)])   # one run below 0.02 m
    write_runs(tmp_path, "c", "m030", [(True, 1, 0.031), (True, 0, 0.031)])   # one run with a contact
    table = ms.sweep_table(tmp_path)
    assert cell_row(table, "a", "m030")["safe"] is True
    assert cell_row(table, "b", "m030")["safe"] is False
    assert cell_row(table, "c", "m030")["safe"] is False
    assert cell_row(table, "c", "m030")["contacts"] == 1  # runs with any contact


def test_best_fixed_requires_safe_everywhere(tmp_path):
    for cell in ("a", "b"):
        write_runs(tmp_path, cell, "m050", [(False, 0, 0.05)] * 4)
        write_runs(tmp_path, cell, "m030", [(True, 0, 0.03)] * 3 + [(False, 0, 0.03)])
    write_runs(tmp_path, "a", "m020", [(True, 0, 0.022)] * 4)
    write_runs(tmp_path, "b", "m020", [(True, 0, 0.012)] * 4)  # most passages, unsafe on b
    table = ms.sweep_table(tmp_path)
    assert ms.best_fixed(table) == "m030"
    assert ms.best_fixed([r for r in table if r["tag"] == "m020"]) is None


def test_pair_trade_reports_the_difference(tmp_path):
    for tag, straight, turn in (("m050", 0.05, 0.05), ("m030", 0.03, 0.03), ("m020", 0.021, 0.015)):
        write_runs(tmp_path, "e3_gap_straight_2", tag, [(True, 0, straight)] * 3)
        write_runs(tmp_path, "e3_gap_turn_2", tag, [(True, 0, turn)] * 3)
    trade = ms.pair_trade(ms.sweep_table(tmp_path))
    assert trade == [{"geometry": "gap", "step": 2, "mode": "realistic", "straight": 0.02, "turn": 0.03,
                      "difference": pytest.approx(0.01)}]


def test_mode_trade_reports_fisher_and_the_safe_margins(tmp_path):
    cell = "e3_gap_straight_3"
    write_runs(tmp_path, cell, "m050", [(False, 0, 0.05)] * 10)
    write_runs(tmp_path, cell, "i050", [(True, 0, 0.05)] * 10, mode="ideal")
    write_runs(tmp_path, cell, "m020", [(True, 0, 0.012)] * 10)       # passes, unsafe
    write_runs(tmp_path, cell, "i020", [(True, 0, 0.021)] * 10, mode="ideal")
    rows = ms.mode_trade(ms.sweep_table(tmp_path))
    by_margin = {r["margin"]: r for r in rows["cells"]}
    assert (by_margin[0.05]["realistic"], by_margin[0.05]["ideal"]) == ("0/10", "10/10")
    assert by_margin[0.05]["p"] < 1e-4          # the realistic plant passes significantly fewer
    assert by_margin[0.02]["p"] == pytest.approx(1.0)
    assert rows["safe_margin"] == [{"cell": cell, "realistic": 0.05, "ideal": 0.02}]


def test_a_cell_all_passing_or_all_failing_is_uninformative(tmp_path):
    for tag in ("m050", "m040", "m030", "m020"):
        write_runs(tmp_path, "all_pass", tag, [(True, 0, 0.04)] * 20)
        write_runs(tmp_path, "all_fail", tag, [(False, 0, 0.06)] * 20)
    write_runs(tmp_path, "step", "m050", [(False, 0, 0.06)] * 20)
    write_runs(tmp_path, "step", "m040", [(True, 0, 0.04)] * 8 + [(False, 0, 0.05)] * 12)
    write_runs(tmp_path, "step", "m030", [(True, 0, 0.03)] * 19 + [(False, 0, 0.05)])
    write_runs(tmp_path, "step", "m020", [(True, 0, 0.03)] * 20)
    info = ms.informative(ms.sweep_table(tmp_path))
    assert info == {("all_fail", "realistic"): False, ("all_pass", "realistic"): False, ("step", "realistic"): True}


def test_a_cell_not_ordered_in_the_margin_is_uninformative(tmp_path):
    # a smaller margin should pass at least as often: 12, 18, 9, 20 of 20 from 0.05 down is not ordered; a reversal of
    # up to 2 runs (a tenth of 20) is noise, not disorder
    for tag, k in (("m050", 12), ("m040", 18), ("m030", 9), ("m020", 20)):
        write_runs(tmp_path, "disordered", tag, [(True, 0, 0.04)] * k + [(False, 0, 0.05)] * (20 - k))
    for tag, k in (("m050", 12), ("m040", 18), ("m030", 16), ("m020", 20)):
        write_runs(tmp_path, "noisy", tag, [(True, 0, 0.04)] * k + [(False, 0, 0.05)] * (20 - k))
    info = ms.informative(ms.sweep_table(tmp_path))
    assert info[("disordered", "realistic")] is False
    assert info[("noisy", "realistic")] is True


def test_report_writes_the_tables(tmp_path):
    write_runs(tmp_path / "runs", "e3_beam_straight_1", "m050", [(True, 0, 0.04)] * 2)
    write_runs(tmp_path / "runs", "e3_beam_straight_1", "i050", [(True, 0, 0.05)] * 2, mode="ideal")
    assert ms.main([str(tmp_path / "runs"), "--out", str(tmp_path / "sweep.md")]) == 0
    text = (tmp_path / "sweep.md").read_text()
    assert "| e3_beam_straight_1 | realistic | 0.05 | 2/2 | 0 | 40.0 | yes |" in text
    assert "uninformative" in text


# ---- Task 4 review: incomplete evidence, the pooled table, paired steps ----

def test_a_run_without_a_summary_or_a_clearance_is_not_safe(tmp_path):
    write_runs(tmp_path, "null", "m050", [(True, 0, None), (True, 0, 0.04)])     # a clearance that was not measured
    write_runs(tmp_path, "dead", "m050", [(True, 0, 0.04)] * 2)
    (tmp_path / "dead-f2-realistic-m050-s3").mkdir()                               # a run that never wrote a summary
    write_runs(tmp_path, "nan", "m050", [(True, 0, float("nan")), (True, 0, 0.04)])
    d = tmp_path / "nocontact-f2-realistic-m050-s1"
    d.mkdir()
    (d / "summary.json").write_text(json.dumps({"seed": 1, "reached": True, "plant_min_clearance_m": 0.04}))
    table = ms.sweep_table(tmp_path)
    for cell in ("null", "dead", "nan", "nocontact"):
        assert cell_row(table, cell, "m050")["safe"] is None, cell                 # unavailable, not safe
    assert cell_row(table, "dead", "m050")["missing"] == 1
    assert cell_row(table, "null", "m050")["unmeasured"] == 1
    assert ms.best_fixed(table) is None
    assert "incomplete" in ms.markdown(table)


def test_best_fixed_refuses_a_partial_grid(tmp_path):
    write_runs(tmp_path, "a", "m050", [(False, 0, 0.05)] * 4)
    write_runs(tmp_path, "b", "m050", [(True, 0, 0.05)] * 4)
    write_runs(tmp_path, "a", "m040", [(True, 0, 0.04)] * 4)          # m040 never ran cell b
    write_runs(tmp_path, "a", "m030", [(True, 0, 0.03)] * 4)
    write_runs(tmp_path, "b", "m030", [(True, 0, 0.03)] * 2)          # m030 ran two seeds fewer on b, and passes most
    assert ms.best_fixed(ms.sweep_table(tmp_path)) == "m050"


def test_rows_count_unsafe_runs_and_the_safe_passages(tmp_path):
    write_runs(tmp_path, "e3_gap_turn_1", "m030", [(True, 0, 0.031), (True, 0, 0.012), (False, 0, 0.008), (False, 1, -0.001)])
    row = cell_row(ms.sweep_table(tmp_path), "e3_gap_turn_1", "m030")
    assert (row["unsafe_runs"], row["unsafe_reached"], row["safe_reached"]) == (3, 1, 1)


def test_obstacle_table_pools_the_cells_of_an_obstacle(tmp_path):
    write_runs(tmp_path, "e3_gap_straight_1", "m040", [(True, 0, 0.030), (True, 0, 0.034), (False, 0, 0.010)])
    write_runs(tmp_path, "e3_gap_turn_1", "m040", [(True, 0, 0.038), (False, 1, -0.002)])
    write_runs(tmp_path, "e3_beam_turn_1", "m040", [(True, 0, 0.05)])
    rows = ms.obstacle_table(ms.sweep_table(tmp_path))
    gap = next(r for r in rows if r["geometry"] == "gap")
    assert (gap["mode"], gap["margin"], gap["n"], gap["reached"], gap["contacts"]) == ("realistic", 0.04, 5, 3, 1)
    assert (gap["unsafe_runs"], gap["unsafe_reached"], gap["safe_reached"]) == (2, 0, 3)
    assert gap["clearance_median"] == pytest.approx(0.034)            # of the runs that reached
    assert gap["clearance_p05"] == pytest.approx(0.0304)              # interpolated, as margin_analysis.quantile
    assert gap["clearance_min"] == pytest.approx(0.030)
    assert len(rows) == 2


def test_paired_steps_compare_the_same_seeds_at_adjacent_margins(tmp_path):
    # seeds 1-10 at 0.05 m: the first 2 reach; at 0.04 m: the first 9 reach -> 7 seeds only at the smaller margin
    write_runs(tmp_path, "e3_gap_turn_2", "m050", [(k < 2, 0, 0.05) for k in range(10)])
    write_runs(tmp_path, "e3_gap_turn_2", "m040", [(k < 9, 0, 0.04) for k in range(10)])
    (step,) = ms.paired_steps(ms.sweep_table(tmp_path))
    assert (step["cell"], step["mode"], step["larger"], step["smaller"]) == ("e3_gap_turn_2", "realistic", 0.05, 0.04)
    assert (step["only_smaller"], step["only_larger"], step["seeds"]) == (7, 0, 10)
    assert step["p"] == pytest.approx(0.5 ** 7)                        # one-sided exact sign test: the smaller passes more


def test_the_report_names_the_mode_comparison_as_plant_and_model(tmp_path):
    write_runs(tmp_path, "e3_beam_straight_1", "m050", [(True, 0, 0.04)] * 2)
    write_runs(tmp_path, "e3_beam_straight_1", "i050", [(True, 0, 0.05)] * 2, mode="ideal")
    text = ms.markdown(ms.sweep_table(tmp_path))
    assert "what differs at a cell and margin is tracking" not in text
    assert "B5's prediction model" in text


def test_best_fixed_checks_the_expected_cells_and_seeds(tmp_path):
    # E3 final review I3: one cell x four margins x one seed, all measured: complete only against itself
    for tag in ("m050", "m040", "m030", "m020"):
        write_runs(tmp_path, "e3_gap_straight_1", tag, [(True, 0, 0.045)])
    table = ms.sweep_table(tmp_path)
    assert ms.best_fixed(table) == "m050"                                   # over the observed cells only
    assert ms.best_fixed(table, cells={"e3_gap_straight_1", "e3_gap_turn_1"}, seeds=range(1, 21)) is None
    assert ms.best_fixed(table, cells={"e3_gap_straight_1"}, seeds=range(1, 21)) is None   # 19 seeds absent
    assert ms.best_fixed(table, cells={"e3_gap_straight_1"}, seeds=[1]) == "m050"
    text = ms.markdown(table)
    assert "observed cells only" in text
    text = ms.markdown(table, cells={"e3_gap_straight_1", "e3_gap_turn_1"}, seeds=range(1, 21))
    assert "incomplete against the expected 2 cells x 20 seeds" in text


def test_main_takes_the_corpus_and_seeds(tmp_path):
    import json as js
    write_runs(tmp_path / "runs", "e3_beam_straight_1", "m050", [(True, 0, 0.04)] * 2)
    (tmp_path / "corpus.json").write_text(js.dumps({"cells": [{"cell": "e3_beam_straight_1"}, {"cell": "e3_beam_turn_1"}]}))
    assert ms.main([str(tmp_path / "runs"), "--out", str(tmp_path / "s.md"), "--corpus", str(tmp_path / "corpus.json"),
                    "--seeds", "2"]) == 0
    assert "incomplete against the expected 2 cells x 2 seeds" in (tmp_path / "s.md").read_text()
