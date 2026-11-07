"""Tests of the cross-simulator agreement (MuJoCo plant plan Task 11, R as amended 2026-11-06 Section 6)."""
import compare_campaigns as cc


def rec(cell, layer, reached, clearance=0.05, contact=False, time_s=40.0):
    return {"cell": cell, "layer": layer, "reached": reached, "clearance_m": clearance, "contact": contact,
            "time_s": time_s, "run": f"{cell}-{layer}", "cm_actions": []}


def unity():
    return ([rec("door", "F2", True)] * 3 + [rec("door", "B5", True)] * 3 +
            [rec("beam", "F2", True)] * 3 + [rec("beam", "B5", False)] * 3)


def plant(n=20, **kw):
    return ([rec("door", "F2", True, **kw)] * n + [rec("door", "B5", True, **kw)] * n +
            [rec("beam", "F2", True, **kw)] * n + [rec("beam", "B5", False, **kw)] * n)


def test_agreement_passes_identical():
    a = cc.agreement(unity(), plant())
    assert a["ranking_holds"] and a["to_investigate"] == []
    assert a["cells"]["beam"]["plant"]["F2"]["rate"] == 1.0 and a["cells"]["beam"]["plant"]["B5"]["rate"] == 0.0


def test_agreement_flags_ranking_flip():
    flipped = [r if not (r["cell"] == "beam" and r["layer"] == "F2") else {**r, "reached": False} for r in plant()]
    flipped += [rec("beam", "B5", True)] * 20
    a = cc.agreement(unity(), flipped)
    assert not a["ranking_holds"] and a["cells"]["beam"]["ranking"] == "B5 > F2"
    assert any(i["cell"] == "beam" and i["what"] == "outcome" for i in a["to_investigate"])


def test_agreement_clearance_tolerance():
    near = cc.agreement(unity(), plant(clearance=0.059))  # 9 mm from Unity's median: within +-10 mm
    assert not [i for i in near["to_investigate"] if i["what"] == "clearance"]
    far = cc.agreement(unity(), plant(clearance=0.061))
    flagged = [i for i in far["to_investigate"] if i["what"] == "clearance"]
    assert {(i["cell"], i["layer"]) for i in flagged} == {("door", "F2"), ("door", "B5"), ("beam", "F2"), ("beam", "B5")}
    assert far["ranking_holds"]  # clearances are reported, not gated (amendment Section 6)


def test_agreement_contact_rule():
    touching = plant()
    touching[0] = {**touching[0], "contact": True}  # one plant door F2 run touches; Unity's never did
    a = cc.agreement(unity(), touching)
    assert [(i["cell"], i["layer"], i["what"]) for i in a["to_investigate"]] == [("door", "F2", "contact")]
    both = [r if r["cell"] != "door" else {**r, "contact": True} for r in unity()]
    assert not [i for i in cc.agreement(both, touching)["to_investigate"] if i["what"] == "contact"]


def test_outcome_difference_over_a_quarter_is_investigated():
    # Unity has 3 runs a cell: no test separates its rate from the plant's 20 but the extremes, so a difference in the
    # success rate over 0.25 is investigated (R's gate is the ranking): 3/3 against 18/20 is not, against 10/20 it is
    close = [r if not (r["cell"] == "door" and r["layer"] == "F2") else r for r in plant()]
    close = [r for r in close if not (r["cell"] == "door" and r["layer"] == "F2")] + \
        [rec("door", "F2", True)] * 18 + [rec("door", "F2", False)] * 2
    assert not [i for i in cc.agreement(unity(), close)["to_investigate"] if i["what"] == "outcome"]
    half = [r for r in plant() if not (r["cell"] == "door" and r["layer"] == "F2")] + \
        [rec("door", "F2", True)] * 10 + [rec("door", "F2", False)] * 10
    assert [(i["cell"], i["layer"]) for i in cc.agreement(unity(), half)["to_investigate"] if i["what"] == "outcome"] == [("door", "F2")]


def test_unity_records_from_run_summaries(tmp_path):
    import json
    d = tmp_path / "20261101T163508Z-v2_door_105_nav-wholebody-50hz-gff-mff-bff-f2"
    d.mkdir()
    (d / "summary.json").write_text(json.dumps({
        "scenario": {"name": "v2_door_105_nav"}, "medium": {"layer": "bezier"}, "clearance_3d_m": 0.0484,
        "contacts": {"contact": False},
        "task": {"status": "succeeded", "time_s": 46.12, "collision_monitor_activations": [{"action": "stop"}]}}))
    r = cc.unity_records([d])[0]
    assert (r["cell"], r["layer"], r["reached"], r["clearance_m"], r["contact"], r["cm_actions"]) == \
        ("v2_door_105_nav", "F2", True, 0.0484, False, ["stop"])


def test_clearances_beyond_the_cut_off_count_as_the_cut_off(tmp_path):
    # the plant's ground truth reports no clearance beyond 2 m: dropping those runs biased open cells' medians low
    # (a missing Unity clearance stays unknown)
    import json
    for seed, clearance in ((1, 1.5), (2, None), (3, None)):
        d = tmp_path / f"v2_open-f2-realistic-s{seed}"
        d.mkdir()
        (d / "summary.json").write_text(json.dumps({"reached": True, "time_s": 1.0, "plant_min_clearance_m": clearance}))
    rs = cc.plant_records(tmp_path)
    assert cc._stats(rs)["clearance_median_m"] == cc.CLEARANCE_CUT_OFF_M


def test_agreement_lists_short_ensembles():
    short = [r for r in plant() if not (r["cell"] == "door" and r["layer"] == "B5")] + [rec("door", "B5", True)] * 5
    assert cc.agreement(unity(), short)["short_ensembles"] == [{"cell": "door", "layer": "B5", "runs": 5}]
    assert cc.agreement(unity(), plant())["short_ensembles"] == []


def test_table_runs_reads_both_campaign_tables(tmp_path):
    a = tmp_path / "a.md"
    a.write_text("| Cell | Layer | Run | Result |\n|---|---|---|---|\n| v2_x | B5 | 20261101T160919Z | aborted |\n"
                 "| v2_x | F2 | not a run | - |\n")
    b = tmp_path / "b.md"
    b.write_text("| # | Cell | Run | Result |\n|---:|---|---|---|\n| 1 | v2_y | 20261101T200543Z | success |\n")
    assert cc.table_runs(a) == [("v2_x", "B5", "20261101T160919Z")]
    assert cc.table_runs(b) == [("v2_y", "F2", "20261101T200543Z")]


def test_plant_records_read_variants(tmp_path):
    import json
    for name in ("v2_store_room_nav-b5-realistic-s3", "v2_store_room_nav-b5-realistic-mu0.4-s2", "v2_x-f2-ideal-planar-s1"):
        (tmp_path / name).mkdir()
        (tmp_path / name / "summary.json").write_text(json.dumps({"reached": True, "time_s": 1.0}))
    got = {(r["cell"], r["layer"], r["mode"], r["variant"], r["seed"]) for r in cc.plant_records(tmp_path)}
    assert got == {("v2_store_room_nav", "B5", "realistic", "", 3), ("v2_store_room_nav", "B5", "realistic", "mu0.4", 2),
                   ("v2_x", "F2", "ideal", "planar", 1)}


def test_grip_table_marks_a_missing_variant(tmp_path):
    import json
    corpus = tmp_path / "grip.json"
    corpus.write_text(json.dumps({"cells": [{"cell": "c"}]}))
    rows = [{**rec("c", layer, True), "mode": "realistic", "variant": v} for layer in ("F2", "B5") for v in ("mu0.4", "mu0.8")]
    rows += [{**rec("c", "F2", True), "mode": "realistic", "variant": ""}]  # no B5 at mu 0.6
    line = next(l for l in cc.grip_table(rows, corpus) if l.startswith("| c | B5"))
    assert "| - |" in line and "F2 >= B5 / - / F2 >= B5" in line


def test_clearances_beyond_the_cut_off_are_no_difference():
    # the plant cannot measure past 2 m: Unity's 2.4 m against the plant's cut-off is not a difference to investigate
    u = [rec("spine", "F2", True, clearance=2.41)]
    p = [rec("spine", "F2", True, clearance=cc.CLEARANCE_CUT_OFF_M)] * 20
    assert not [i for i in cc.agreement(u, p)["to_investigate"] if i["what"] == "clearance"]
