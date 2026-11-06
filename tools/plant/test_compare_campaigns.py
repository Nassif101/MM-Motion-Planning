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
