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


# ---- Task 6 review ---------------------------------------------------------------------------------------------------

def test_one_free_gain_subtracts_the_fixed_one():
    # review F1: with g_0 fixed at its default (1), an identity plant gives g_inf 1, and a plant with g_inf 0.9 gives 0.9
    ident = it.fit(segments_from({}), ["turn_delta_inf"])
    assert ident["parameters"]["turn_delta_inf"] == pytest.approx(0.0, abs=1e-6)
    p = {"turn_delta_inf": -0.1, "turn_scale": 0.05}
    assert it.fit(segments_from(p), ["turn_delta_inf"])["parameters"]["turn_delta_inf"] == pytest.approx(-0.1, abs=1e-6)


def test_scale_only_returns_the_scored_model():
    # review F1: the turn scale alone changes nothing of an identity map: the gains stay at their defaults
    r = it.fit(segments_from(TRUE), ["turn_scale"])
    assert r["parameters"]["turn_delta_inf"] == 0.0 and r["parameters"]["turn_delta_zero"] == 0.0
    r = it.fit(segments_from(dict(TRUE, turn_delta_inf=0.0)), ["turn_delta_zero", "turn_scale"])
    assert r["parameters"]["turn_delta_zero"] == pytest.approx(TRUE["turn_delta_zero"], rel=0.05)
    assert r["parameters"]["turn_scale"] == pytest.approx(TRUE["turn_scale"], rel=0.05)


def test_an_active_bound_refits_the_other_gain():
    # review F1: a plant whose unconstrained g_0 lies below the loader's floor: g_0 sits on the floor and g_inf is the
    # least-squares optimum given it, not the unconstrained value clipped
    p = {"turn_delta_inf": -0.1, "turn_delta_zero": -1.1, "turn_scale": 0.05}  # g_0 = -0.1, outside the range
    segs = segments_from(p)
    r = it.fit(segs, ["turn_delta_inf", "turn_delta_zero"])
    assert r["parameters"]["turn_delta_zero"] == pytest.approx(-0.99)
    data = [d for d in it.intervals(segs, "fit") if d[0] or d[1]]
    c, g0 = 0.05, 0.01
    a = [w - c * math.tanh(w / c) for _, w, *_ in data]
    b = [c * math.tanh(w / c) for _, w, *_ in data]
    y = [d[4] for d in data]
    g_inf = sum(x * (z - g0 * u) for x, u, z in zip(a, b, y)) / sum(x * x for x in a)
    assert r["parameters"]["turn_delta_inf"] == pytest.approx(g_inf - 1.0, abs=1e-9)


def log_from(tmp_path, name, recs, profile="aa"):
    import struct
    import zlib
    import plant_stream as ps
    frames = []
    for r in recs:
        h = {"t": r["t"], "goal": 0, "status": "ok", "gate": [0, 0, 0], "fallback": "", "diag": [0.0, 0.0, None],
             "diag_pair": "", "diag_grad": [0.0] * 9, "diag_kink": [0.0] * 9, "diag_cmd": r["cmd"]}
        frames.append((ps.STATE, h, [r["pose"], r["b5"], r["twist"] + r["qd_ref"]]))
    raw = ps.encode(ps.HELLO, {"schema": 2, "mode": "realistic", "profile_sha256": profile}, [])
    raw += b"".join(ps.encode(k, h, b) for k, h, b in frames)
    path = tmp_path / name / "run.mmlog"
    path.parent.mkdir(parents=True)
    packed = zlib.compress(raw)
    path.write_bytes(b"MMRUNLG1" + struct.pack("<II", len(raw), len(packed)) + packed)
    return str(path)


def test_closed_loop_split_groups_cases_and_keeps_their_roles(tmp_path):
    # review F2: the same physical case under two configs (identical records) is one case; roles follow the case, not
    # the list position, so they agree across grips whatever runs are present
    recs = synthetic_records(varied_commands(200, 1), TRUE)
    other = synthetic_records(varied_commands(200, 2), TRUE)
    logs = [log_from(tmp_path, "v2_door_105_mission-f2-realistic-abs-s20", recs),
            log_from(tmp_path, "v2_door_105_mission-f2-realistic-frozen-s20", recs),
            log_from(tmp_path, "v2_gate_right_swing_nav-f2-realistic-absinf-s4", other),
            log_from(tmp_path, "v2_store_room_nav-b5-realistic-abs-s1", other)]
    segs = it.closed_loop_segments(logs)
    assert len(segs) == 3  # the identical twin is dropped
    roles = {s["case"]: s["role"] for s in segs}
    again = {s["case"]: s["role"] for s in it.closed_loop_segments(logs[2:] + logs[:1])}
    assert all(roles[c] == again[c] for c in again)
    assert set(roles) == {"v2_door_105_mission-f2-s20", "v2_gate_right_swing_nav-f2-s4", "v2_store_room_nav-b5-s1"}


def test_closed_loop_profile_comes_from_the_logs(tmp_path):
    # review F4: the identified plant is the one that made the logs; mixed plants are refused
    recs = synthetic_records(varied_commands(50, 1), TRUE)
    a = log_from(tmp_path, "a-f2-realistic-x-s1", recs, profile="35783b95")
    b = log_from(tmp_path, "b-f2-realistic-x-s1", recs, profile="35783b95")
    c = log_from(tmp_path, "c-f2-realistic-x-s1", recs, profile="f45215f8")
    assert it.logs_profile([a, b]) == "35783b95"
    with pytest.raises(ValueError, match="profile"):
        it.logs_profile([a, c])


def test_write_model_keeps_numbers_numbers():
    # review F3: the shared YAML resolver reads 1e-06 as a string; the writer spells exponents with a decimal point
    assert it.yaml_number(1e-6) == "1.0e-06"
    assert it.yaml_number(-0.99) == "-0.99"
    assert it.yaml_number(0.0) == "0"
    assert it.yaml_number(2.5e-7) == "2.5e-07"


@pytest.mark.skipif(not mr.BINARY_PATH.exists(), reason="model_step is not built here (runs in the container)")
def test_written_model_loads_in_b5(tmp_path):
    # review F3: the C++ loader itself (model_step --check-model) accepts what write_model writes
    import subprocess
    for value in (1e-6, -2.5e-7, 0.0, -0.99, 0.5):
        params = dict(it.DEFAULTS, sideslip=value if -0.5 <= value <= 0.5 else 0.0, turn_delta_zero=-0.99)
        path = tmp_path / f"m{value}.yaml"
        it.write_model(path, params, {"date": "2026-11-12", "source": "t", "terms": [], "held_out": {}})
        p = subprocess.run(["bash", "-lc", f"exec {mr.BINARY_PATH} --check-model {path}"], capture_output=True, text=True)
        assert p.returncode == 0, (value, p.stderr)
    bad = tmp_path / "bad.yaml"
    bad.write_text('sideslip: "0.2"\n')  # a string: the loader must refuse it, so the check is real
    p = subprocess.run(["bash", "-lc", f"exec {mr.BINARY_PATH} --check-model {bad}"], capture_output=True, text=True)
    assert p.returncode == 2 and "sideslip" in p.stderr


def test_excitation_records_round_trip(tmp_path):
    # review F5: the excitation's responses are saved with their roles and profile, and a fit can start from them
    segs = segments_from(TRUE, count=4)
    for s in segs:
        s["pose"] = "carry"
    path = tmp_path / "excitation_records.json.gz"
    it.save_segments(path, segs, {"profile_sha256": "aa", "source": "synthetic"})
    back, meta = it.load_segments(path)
    assert meta["profile_sha256"] == "aa"
    assert [s["role"] for s in back] == [s["role"] for s in segs]
    assert it.fit(back, ["turn_delta_zero", "turn_scale"])["parameters"] == it.fit(segs, ["turn_delta_zero", "turn_scale"])["parameters"]


# ---- Task 6b review: the horizon check (T11) ---------------------------------------------------------------------

TRACKING_REPLAY = mr.ROOT / "ros2_ws/install/mobile_manipulator_mpc/lib/mobile_manipulator_mpc/tracking_replay"


@pytest.mark.skipif(not TRACKING_REPLAY.exists(), reason="tracking_replay is not built here (runs in the container)")
def test_estimates_follow_a_slippery_plant(tmp_path):
    # the C++ estimator replayed over a run: from the prior it moves towards the plant's g_inf (1.27) and never sees the
    # future: the estimate in force at record j uses the transitions up to j only
    prior = tmp_path / "prior.yaml"
    it.write_model(prior, dict(it.DEFAULTS, turn_delta_inf=-0.018, turn_delta_zero=-0.975, turn_scale=0.0225,
                               speed_turn=0.21, sideslip=0.196), {"date": "x", "source": "t", "terms": [], "held_out": {}})
    plant = dict(TRUE, turn_delta_inf=0.27, turn_delta_zero=-0.975, turn_scale=0.0225, speed_turn=0.21, sideslip=0.083)
    recs = synthetic_records([(0.3, 0.2)] * 600, plant)
    est = it.replay_estimates(recs, prior, container=None)
    assert len(est) == len(recs)
    assert est[0][0] == pytest.approx(0.982)
    assert est[-1][0] == pytest.approx(1.27, abs=0.05)
    cut = it.replay_estimates(recs[:300], prior, container=None)
    assert cut == est[:300]  # causal


def test_horizon_slices():
    # the slices of the horizon check: moving starts, the first second of motion, after an in-place slow turn
    cmds = [(0.0, 0.0)] * 50 + [(0.3, 0.2)] * 100 + [(0.0, 0.05)] * 100 + [(0.3, 0.2)] * 100
    recs = synthetic_records(cmds, TRUE)
    s = it.horizon_slices(recs)
    assert 50 < min(s["first_second"]) and max(s["first_second"]) <= 100  # records 51-100: the first second of motion
    assert all(r > 250 for r in s["after_in_place"])  # the turn after the in-place stretch
    assert set(s["first_second"]) <= set(s["moving"]) and 10 not in s["moving"]
