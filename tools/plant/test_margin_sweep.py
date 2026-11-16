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
