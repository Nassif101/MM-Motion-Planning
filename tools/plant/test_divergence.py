"""Tests of the twin-run divergence tool (E1 plan Task 4): synthetic run logs, the first diverging decision, flips."""
import json
import struct
import zlib

import pytest

import divergence as dv
import plant_stream as ps


def hello(**over):
    h = {"schema": 2, "scenario": "v2_x", "bag": "b", "seed": 3, "mode": "realistic",
         "config": {"b5_config_sha256": "aa", "gate_rule": "increment", "start_nudge_rad": 0.0}}
    h.update(over)
    return h


def state(t, status="ok", gate=(0.0, 0.0, 0), dwell=0, fallback="", pose=None, schema=2):
    h = {"t": t, "status": status, "gate": list(gate), "fallback": fallback}
    if schema >= 2:
        h["dwell"] = dwell
    return (ps.STATE, h, [pose or [0.0] * 9, [0.0] * 9, [0.0] * 8])


def plans(t, seed="previous", homotopy="comfort", status="ok", waiting=False, seeds=None, schema=2):
    h = {"t": t, "plan_id": 1, "route_id": 1, "status": status, "homotopy": homotopy, "capped": False}
    if schema >= 2:
        h.update({"seed": seed, "waiting": waiting, "full_multistart": False, "homotopy_switched": False,
                  "seeds": seeds if seeds is not None else [[seed, 1.0, 1, 2.0]]})
    return (ps.PLANS, h, [[0.0] * 9])


def write_log(path, header, frames, summary=None):
    raw = ps.encode(ps.HELLO, header, []) + b"".join(ps.encode(k, h, b) for k, h, b in frames)
    packed = zlib.compress(raw)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"MMRUNLG1" + struct.pack("<II", len(raw), len(packed)) + packed)
    if summary is not None:
        (path.parent / "summary.json").write_text(json.dumps(summary))
    return path


DONE = {"reached": True, "end_status": "goal_reached", "plant_contacts": 0}


def run_frames(n=10, change=None):
    """n cycles with a plan every 5th; `change` = {cycle: state() overrides, ("plan", cycle): plans() overrides}."""
    change = change or {}
    out = []
    for k in range(n):
        if k % 5 == 0:
            out.append(plans(0.02 * k, **change.get(("plan", k), {})))
        out.append(state(0.02 * (k + 1), **change.get(k, {})))
    return out


def test_identical_logs_do_not_diverge(tmp_path):
    a = write_log(tmp_path / "a" / "run.mmlog", hello(), run_frames(), DONE)
    b = write_log(tmp_path / "b" / "run.mmlog", hello(), run_frames(), DONE)
    r = dv.compare(a, b)
    assert r["first"] is None and r["truncated"] is None
    assert r["distance_end"] == {"base_m": 0.0, "yaw_rad": 0.0, "joints_rad": 0.0, "cycle": 9}
    assert not dv.flipped(r["outcomes"]["a"], r["outcomes"]["b"])


def test_first_differing_decision_is_reported(tmp_path):
    a = write_log(tmp_path / "a" / "run.mmlog", hello(), run_frames(), DONE)
    held = {k: {"status": "holding", "gate": (0.006, 0.0, 1)} for k in range(7, 10)}
    b = write_log(tmp_path / "b" / "run.mmlog", hello(), run_frames(change=held), DONE)
    r = dv.compare(a, b)
    assert r["first"]["cycle"] == 7 and r["first"]["decision"] == "status"
    assert r["first"]["a"] == "ok" and r["first"]["b"] == "holding"
    assert r["first"]["context_b"]["gate"] == [0.006, 0.0, 1]


def test_plan_seed_divergence_has_the_seed_tables(tmp_path):
    ta = [["previous", 1.00, 1, 2.0], ["fold", 1.01, 1, 1.9]]
    tb = [["previous", 1.02, 1, 2.0], ["fold", 1.01, 1, 1.9]]
    a = write_log(tmp_path / "a" / "run.mmlog", hello(),
                  run_frames(change={("plan", 5): {"seed": "previous", "seeds": ta}}), DONE)
    b = write_log(tmp_path / "b" / "run.mmlog", hello(),
                  run_frames(change={("plan", 5): {"seed": "fold", "seeds": tb}}), DONE)
    r = dv.compare(a, b)
    assert r["first"]["decision"] == "plan_seed" and r["first"]["cycle"] == 5
    assert r["first"]["context_a"]["seeds"] == ta and r["first"]["context_b"]["seeds"] == tb


def test_truncated_log_is_reported_not_diverged(tmp_path):
    # Review Focus 1: a run that died after 5 of 10 cycles (no summary) is truncated, not a divergence
    a = write_log(tmp_path / "a" / "run.mmlog", hello(), run_frames(10), DONE)
    b = write_log(tmp_path / "b" / "run.mmlog", hello(), run_frames(5))
    r = dv.compare(a, b)
    assert r["truncated"] == {"side": "b", "cycle": 5}
    assert r["first"] is None


def test_a_finished_run_ending_earlier_diverges_at_its_end(tmp_path):
    a = write_log(tmp_path / "a" / "run.mmlog", hello(), run_frames(10), DONE)
    b = write_log(tmp_path / "b" / "run.mmlog", hello(), run_frames(6), DONE)
    r = dv.compare(a, b)
    assert r["truncated"] is None and r["first"]["decision"] == "end" and r["first"]["cycle"] == 6


CONFIG = {"b5_config_sha256": "aa", "gate_rule": "increment", "start_nudge_rad": 0.0}


@pytest.mark.parametrize("field,value,name", [
    ("seed", 4, "seed"), ("scenario", "v2_y", "scenario"), ("bag", "c", "bag"), ("mode", "ideal", "mode"),
    ("medium", False, "medium"), ("profile_sha256", "pp", "profile_sha256"),
    ("robot_model_sha256", "rr", "robot_model_sha256"), ("site_model_sha256", "ss", "site_model_sha256"),
    ("config", dict(CONFIG, b5_config_sha256="bb"), "b5_config_sha256"),
    ("config", dict(CONFIG, stall_s=45.0), "stall_s"), ("config", dict(CONFIG, timeout_s=60.0), "timeout_s"),
    ("config", dict(CONFIG, iteration_budget=15), "iteration_budget"), ("config", dict(CONFIG, lidar=True), "lidar"),
    ("config", dict(CONFIG, medium_config="m.yaml"), "medium_config")])
def test_mismatched_pair_is_refused(tmp_path, field, value, name):
    # Review Focus 2: only the nudge may differ between twins (Task 4 review: the layer, the models, the settings)
    a = write_log(tmp_path / "a" / "run.mmlog", hello(), run_frames(), DONE)
    b = write_log(tmp_path / "b" / "run.mmlog", hello(**{field: value}), run_frames(), DONE)
    with pytest.raises(ValueError, match=name):
        dv.compare(a, b)


def test_a_missing_stall_setting_is_no_stall_detector(tmp_path):
    # runs before 5bd4300 (the baseline replicas) had none: their missing stall_s equals an explicit 0
    a = write_log(tmp_path / "a" / "run.mmlog", hello(), run_frames(), DONE)
    b = write_log(tmp_path / "b" / "run.mmlog", hello(config=dict(CONFIG, stall_s=0.0)), run_frames(), DONE)
    assert dv.compare(a, b)["first"] is None


def test_a_dead_run_is_truncated_even_after_a_divergence(tmp_path):
    a = write_log(tmp_path / "a" / "run.mmlog", hello(), run_frames(10), DONE)
    held = {k: {"status": "holding"} for k in range(3, 6)}
    b = write_log(tmp_path / "b" / "run.mmlog", hello(), run_frames(6, change=held))  # no summary: it died
    r = dv.compare(a, b)
    assert r["first"]["cycle"] == 3 and r["truncated"] == {"side": "b", "cycle": 6}
    assert r["distance_end"]["cycle"] == 5  # the last cycle both have


def test_a_nudge_is_allowed(tmp_path):
    a = write_log(tmp_path / "a" / "run.mmlog", hello(), run_frames(), DONE)
    nudged = hello(config={"b5_config_sha256": "aa", "gate_rule": "increment", "start_nudge_rad": 1e-12})
    b = write_log(tmp_path / "b" / "run.mmlog", nudged, run_frames(), DONE)
    assert dv.compare(a, b)["first"] is None


def test_schema_1_against_schema_2_skips_trace_fields(tmp_path):
    # Review Focus 3: the baseline replicas are schema 1 (A5's equivalence check)
    old = [state(0.02 * (k + 1), schema=1) for k in range(4)]
    old.insert(0, plans(0.0, schema=1))
    a = write_log(tmp_path / "a" / "run.mmlog", {"schema": 1, "scenario": "v2_x", "bag": "b", "seed": 3,
                                                 "mode": "realistic", "config": {}}, old, DONE)
    new = [state(0.02 * (k + 1)) for k in range(4)]
    new.insert(0, plans(0.0))
    b = write_log(tmp_path / "b" / "run.mmlog", hello(), new, DONE)
    r = dv.compare(a, b)
    assert {"dwell", "plan_seed", "plan_waiting"} <= set(r["skipped"])
    assert r["first"] is None


def test_state_distance_at_the_end(tmp_path):
    a = write_log(tmp_path / "a" / "run.mmlog", hello(), run_frames(3), DONE)
    moved = [0.3, 0.4, 0.1] + [0.0] * 5 + [0.2]
    b = write_log(tmp_path / "b" / "run.mmlog", hello(), run_frames(3, change={2: {"pose": moved}}), DONE)
    d = dv.compare(a, b)["distance_end"]
    assert d["base_m"] == pytest.approx(0.5, abs=1e-6)
    assert d["yaw_rad"] == pytest.approx(0.1, abs=1e-6)
    assert d["joints_rad"] == pytest.approx(0.2, abs=1e-6)


def test_flip_rules():
    reached = {"reached": True, "plant_contacts": 0}
    failed = {"reached": False, "plant_contacts": 0}
    touched = {"reached": False, "plant_contacts": 12}
    assert dv.flipped(reached, failed)
    assert dv.flipped(failed, touched)
    assert not dv.flipped(reached, dict(reached))
    assert not dv.flipped(failed, dict(failed))


def test_ensemble_table_and_histogram():
    results = [
        {"cell": "c1", "kind": "b5", "seed": 1, "flipped": True, "first": {"decision": "status"}},
        {"cell": "c1", "kind": "b5", "seed": 2, "flipped": False, "first": {"decision": "plan_seed"}},
        {"cell": "c2", "kind": "f2", "seed": 1, "flipped": False, "first": None},
    ]
    results.append({"cell": "c2", "kind": "f2", "seed": 2, "flipped": False, "first": None,
                    "truncated": {"side": "b", "cycle": 9}})  # a run that died: incomplete, not a pair
    report = dv.ensemble_report(results)
    rows = {(r["cell"], r["kind"]): r for r in report["cells"]}
    assert rows[("c1", "b5")] == {"cell": "c1", "kind": "b5", "pairs": 2, "flips": 1, "flip_rate": 0.5, "incomplete": 0}
    assert rows[("c2", "f2")]["flip_rate"] == 0.0 and rows[("c2", "f2")]["incomplete"] == 1
    assert report["first_divergence"] == {"b5": {"status": 1, "plan_seed": 1}, "f2": {"none": 1}}
    assert report["pooled"] == {"pairs": 3, "flips": 1, "flip_rate": pytest.approx(1 / 3), "incomplete": 1}
    md = dv.ensemble_markdown(report)
    assert "| c1 | b5 | 2 | 1 | 0.50 | 0 |" in md and "status" in md


def test_baseline_logs_without_the_gate_field_skip_it(tmp_path):
    # the baseline replicas' State frames predate the gate field (93a4d91): gate_unsafe is skipped, not diverged
    old = []
    for k in range(4):
        kind, h, blocks = state(0.02 * (k + 1), schema=1)
        del h["gate"]
        old.append((kind, h, blocks))
    a = write_log(tmp_path / "a" / "run.mmlog", {"schema": 1, "scenario": "v2_x", "bag": "b", "seed": 3,
                                                 "mode": "realistic", "config": {}}, old, DONE)
    b = write_log(tmp_path / "b" / "run.mmlog", hello(), [state(0.02 * (k + 1)) for k in range(4)], DONE)
    r = dv.compare(a, b)
    assert "gate_unsafe" in r["skipped"] and r["first"] is None
