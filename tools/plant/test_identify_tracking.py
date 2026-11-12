"""Tests of the tracking-model identification (E2 plan Task 6): a synthetic plant applies a known map to the commands."""
import math
import random

import pytest

import identify_tracking as it
import model_rollout as mr

DT = 0.02
TRUE = {"turn_delta_inf": -0.03, "turn_delta_zero": -0.9, "turn_scale": 0.03, "speed_turn": 0.15, "sideslip": 0.2}


def plant_map(v, w, p):
    """The tracking map of tracking_model.hpp: the executed (forward, lateral, yaw rate) for a command."""
    g_inf, g_0, c = 1 + p.get("turn_delta_inf", 0.0), 1 + p.get("turn_delta_zero", 0.0), p.get("turn_scale", 0.05)
    wf = g_inf * w - (g_inf - g_0) * c * math.tanh(w / c)
    vf = v * (1 - p.get("speed_turn", 0.0) * (math.sqrt(wf * wf + 1e-4) - 0.01))
    return vf, p.get("sideslip", 0.0) * vf * wf, wf


def synthetic_records(commands, p, seed=0):
    """Rollout records of a plant executing `commands` (one per 20 ms record, the time convention) through the map,
    integrated exactly per interval at the midpoint heading."""
    rng = random.Random(seed)
    x = [0.0] * 9
    recs = [{"t": 0.0, "goal": 0, "mode": "realistic", "pose": list(x), "b5": list(x), "twist": [0.0, 0.0],
             "qd_ref": [0.0] * 6, "cmd": [0.0] * 8, "plan_u0": None}]
    for j, (v, w) in enumerate(commands, start=1):
        vf, vl, wf = plant_map(v, w, p)
        mid = x[2] + 0.5 * wf * DT
        x = [x[0] + (vf * math.cos(mid) - vl * math.sin(mid)) * DT, x[1] + (vf * math.sin(mid) + vl * math.cos(mid)) * DT,
             x[2] + wf * DT] + x[3:]
        recs.append({"t": j * DT, "goal": 0, "mode": "realistic", "pose": list(x), "b5": list(x), "twist": [vf, wf],
                     "qd_ref": [0.0] * 6, "cmd": [v, w] + [0.0] * 6, "plan_u0": None})
    return recs


def varied_commands(n, seed):
    rng = random.Random(seed)
    out, v, w = [], 0.0, 0.0
    for j in range(n):
        if j % 40 == 0:
            v, w = rng.choice([0.0, 0.1, 0.2, 0.3]), rng.choice([0.0, 0.01, 0.03, 0.05, 0.1, 0.2, 0.4, -0.03, -0.1, -0.4])
        out.append((v, w))
    return out


def segments_from(p, count=8):
    return [{"name": f"s{k}", "role": "fit" if k % 2 == 0 else "held_out", "records": synthetic_records(varied_commands(600, k), p, k)}
            for k in range(count)]


def test_fit_recovers_known_parameters():
    result = it.fit(segments_from(TRUE), ["turn_delta_inf", "turn_delta_zero", "turn_scale", "speed_turn", "sideslip"])
    for name in ("turn_delta_zero", "turn_scale", "speed_turn", "sideslip"):
        assert result["parameters"][name] == pytest.approx(TRUE[name], rel=0.05), name
    assert result["parameters"]["turn_delta_inf"] == pytest.approx(TRUE["turn_delta_inf"], abs=0.01)


def test_terms_not_listed_stay_zero():
    result = it.fit(segments_from(TRUE), ["turn_delta_inf", "turn_delta_zero", "turn_scale"])
    assert result["parameters"]["speed_turn"] == 0.0 and result["parameters"]["sideslip"] == 0.0
    assert result["parameters"]["turn_delta_zero"] == pytest.approx(TRUE["turn_delta_zero"], rel=0.05)


def test_fit_keeps_the_loader_ranges():
    # a plant that does not turn below its breakaway at all would ask for a zero slope; the loader floors g_0 at 0.01
    p = dict(TRUE, turn_delta_zero=-1.0, turn_scale=0.03)
    result = it.fit(segments_from(p), ["turn_delta_inf", "turn_delta_zero", "turn_scale"])
    assert result["parameters"]["turn_delta_zero"] >= -0.99


@pytest.mark.skipif(not mr.BINARY_PATH.exists(), reason="model_step is not built here (runs in the container)")
def test_held_out_error_falls():
    result = it.fit(segments_from(TRUE), ["turn_delta_inf", "turn_delta_zero", "turn_scale", "speed_turn", "sideslip"],
                    held_out_check=True, container=None)
    before, after = result["held_out"]["before"], result["held_out"]["after"]
    assert after["yaw"] < 0.2 * before["yaw"]
    assert after["lateral"] < 0.2 * before["lateral"]
    assert after["forward"] <= before["forward"]
    assert result["held_out"]["windows"] > 100


def test_write_model_is_deterministic_and_loadable(tmp_path):
    params = dict(TRUE, base_delay_s=0.0, arm_delay_s=0.04)
    provenance = {"date": "2026-11-12", "source": "synthetic", "terms": ["turn_delta_zero"], "held_out": {}}
    a, b = tmp_path / "a.yaml", tmp_path / "b.yaml"
    it.write_model(a, params, provenance)
    it.write_model(b, params, provenance)
    assert a.read_bytes() == b.read_bytes()
    keys = [line.split(":")[0] for line in a.read_text().splitlines() if line and not line.startswith("#")]
    assert keys == list(it.MODEL_KEYS)
    assert "turn_delta_zero: -0.9" in a.read_text()


def test_excitation_alternates_fit_and_held_out_and_stays_in_the_envelope():
    segs = it.excitation(poses=("carry", "home"))
    roles = [s["role"] for s in segs]
    assert roles[:4] == ["fit", "held_out", "fit", "held_out"]
    assert {s["pose"] for s in segs} == {"carry", "home"}
    for s in segs:
        for _, v, w in s["input"]["base_commands"]:
            assert abs(v) + 1.5 * abs(w) <= 1.12 + 1e-9, s["name"]
    names = {s["name"].split("_")[0] for s in segs}
    assert {"steady", "step", "ramp", "turn"} <= names
