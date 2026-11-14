"""Tests of the tracking-model identification (E2 plan Task 6): a synthetic plant applies a known map to the commands."""
import math
import random

import pytest

import identify_tracking as it
import model_rollout as mr

DT = 0.02
TRUE = {"turn_delta_inf": -0.03, "turn_delta_zero": -0.9, "turn_scale": 0.03, "speed_turn": 0.15, "sideslip": 0.2}


def plant_map(v, w, p, com=0.0):
    """The tracking map of tracking_model.hpp: the executed (forward, lateral, yaw rate) for a command; `com` the centre
    of mass's x for the turning-centre offset (spec Amendment 3)."""
    g_inf, g_0, c = 1 + p.get("turn_delta_inf", 0.0), 1 + p.get("turn_delta_zero", 0.0), p.get("turn_scale", 0.05)
    wf = g_inf * w - (g_inf - g_0) * c * math.tanh(w / c)
    vf = v * (1 - p.get("speed_turn", 0.0) * (math.sqrt(wf * wf + 1e-4) - 0.01))
    offset = p.get("turn_offset", 0.0) + p.get("turn_offset_com", 0.0) * com
    return vf, p.get("sideslip", 0.0) * vf * wf + offset * wf, wf


def synthetic_records(commands, p, seed=0, arm=None, com=0.0):
    """Rollout records of a plant executing `commands` (one per 20 ms record, the time convention) through the map,
    integrated exactly per interval at the midpoint heading; the arm held at `arm` (centre of mass x `com`)."""
    rng = random.Random(seed)
    x = [0.0] * 3 + list(arm or [0.0] * 6)
    recs = [{"t": 0.0, "goal": 0, "mode": "realistic", "pose": list(x), "b5": list(x), "twist": [0.0, 0.0],
             "qd_ref": [0.0] * 6, "cmd": [0.0] * 8, "plan_u0": None}]
    for j, (v, w) in enumerate(commands, start=1):
        vf, vl, wf = plant_map(v, w, p, com)
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


# ---- E2 Task 9: the closed-loop check (T1) ----

def with_track(recs, p):
    """The records with the diag_track B5 would log for a fixed model p (g_inf, g_0, k)."""
    for r in recs:
        r["track"] = [1 + p["turn_delta_inf"], 1 + p["turn_delta_zero"], p["sideslip"]]
    return recs


def test_b5_parameters_come_from_the_record_after_the_start():
    # the time convention: the cycle at record i0 planned with the model logged in record i0 + 1
    prior = dict(TRUE, turn_offset=0.003, turn_offset_com=-2.23)  # the offsets are the prior's (not estimated)
    recs = [{"track": None}, {"track": [0.9, 0.1, 0.3]}, {"track": [0.8, 0.1, 0.2]}]
    assert it.b5_parameters(recs, 0, prior) == pytest.approx([-0.1, -0.9, TRUE["turn_scale"], TRUE["speed_turn"], 0.3,
                                                              0.003, -2.23])
    assert it.b5_parameters(recs, 2, prior) == it.map_parameters(prior)  # no next record: the prior
    assert it.b5_parameters([{"track": None}, {"track": None}], 0, prior) == it.map_parameters(prior)


def test_closed_loop_errors_score_b5s_own_model():
    # a plant with TRUE's map; B5 logged TRUE (as with adaptation converged), the fixed prior is wrong: B5's column
    # predicts the plant, the unicycle and the prior do not
    recs = with_track(synthetic_records(varied_commands(600, 3), TRUE, 3), TRUE)
    prior = dict(TRUE, turn_delta_inf=-0.3, sideslip=0.0)
    res = it.closed_loop_errors([recs], prior, container=None)
    assert res["windows"] > 50
    assert res["b5"]["yaw"] < 1e-4 and res["b5"]["lateral"] < 1e-5
    assert res["unicycle"]["yaw"] > 10 * res["b5"]["yaw"]
    assert res["prior"]["yaw"] > 10 * res["b5"]["yaw"]


def test_depth_direction_agrees_for_the_model_that_drove_the_plant():
    # the deepest pair's depth grows with y (world): each model's predicted depth change against the measured one
    recs = with_track(synthetic_records([(0.2, 0.3)] * 200, TRUE, 0), TRUE)
    for r in recs:
        r.update({"depth": 0.01 + r["pose"][1], "depth_stage1": 0.0, "pair": "1|2|wall", "kink": [0.0] * 9,
                  "grad": [0.0, -1.0] + [0.0] * 7})
    res = it.depth_direction([recs], TRUE, container=None)
    assert res["windows"] > 10
    assert res["b5"] == 1.0
    assert set(res) >= {"unicycle", "prior", "b5"}


def test_prediction_error_against_depth_change_clears_only_a_model_that_predicts_the_plant():
    # Task 10 (T5): in each deep window, B5's own one-stage prediction error projected on the deepest pair's gradient,
    # against that pair's measured depth change. B5 logged the plant's model: the error is far below the change and
    # every window is cleared. A B5 that believed in 3x the plant's sideslip errs by more than the change in some
    recs = synthetic_records([(0.2, 0.3)] * 200, TRUE, 0)
    for j, r in enumerate(recs):  # the logs' convention: record j's diagnostics are evaluated at record j - 1's state
        r.update({"depth": 0.01 + recs[max(j - 1, 0)]["pose"][1], "depth_stage1": 0.0, "pair": "1|2|wall",
                  "kink": [0.0] * 9, "grad": [0.0, -1.0] + [0.0] * 7})
    res = it.prediction_against_depth([with_track(recs, TRUE)], TRUE, container=None)
    assert res["windows"] > 10
    assert res["cleared"] == 1.0
    assert res["median_abs_error_m"] < 1e-5 < res["median_abs_depth_change_m"]
    wrong = it.prediction_against_depth([with_track(recs, dict(TRUE, sideslip=3 * TRUE["sideslip"] + 0.2))], TRUE,
                                        container=None)
    assert wrong["windows"] == res["windows"]
    assert wrong["median_abs_error_m"] > 100 * res["median_abs_error_m"]
    assert wrong["cleared"] < 1.0  # review M1: an always-cleared classifier fails here
    assert set(wrong) >= {"cleared", "max_abs_error_m", "per_window", "dof_non_overlapping", "max_depth_m"}
    # review I4: the per-DOF sums use non-overlapping windows; on this straight-line deepening (dy per 0.1 s) the
    # measured total equals the net depth change over those windows, and the largest depth is the log's
    dof = wrong["dof_non_overlapping"]
    assert dof["measured"]["total"] == pytest.approx(dof["depth_change_m"], rel=1e-3)
    assert dof["windows"] <= wrong["windows"] / 4
    assert wrong["max_depth_m"] == pytest.approx(max(r["depth"] for r in recs))


def offset_plant_records(segments, d, k, dt=0.02, sub=20):
    """Records of a base whose frame moves sideways at d * w + k * v * w (body frame) while driving at (v, w): each
    segment ((v, w), seconds), integrated finely."""
    x, y, yaw, t = 0.0, 0.0, 0.0, 0.0
    recs = [{"t": t, "pose": [x, y, yaw] + [0.0] * 6}]
    for (v, w), seconds in segments:
        for _ in range(int(round(seconds / dt))):
            h = dt / sub
            for _ in range(sub):
                lat = d * w + k * v * w
                x += h * (v * math.cos(yaw) - lat * math.sin(yaw))
                y += h * (v * math.sin(yaw) + lat * math.cos(yaw))
                yaw += h * w
            t += dt
            recs.append({"t": t, "pose": [x, y, yaw] + [0.0] * 6})
    return recs


def test_offset_fit_recovers_the_turning_centre_with_midpoint_velocities():
    # Task 10 review I1: the lateral response fit uses the interval's mid heading (model_rollout._body) and
    # non-overlapping 0.1 s windows; a base turning about a point 0.08 m behind its frame, with sideslip 0.1, is
    # recovered, in place and moving; a start-heading convention would bias k by about 0.5 * 0.1 s
    segs = [((0.0, 0.3), 3.0), ((0.0, -0.2), 3.0), ((0.25, 0.3), 3.0), ((0.2, -0.4), 3.0), ((0.3, 0.1), 3.0)]
    res = it.lateral_offset_fit([offset_plant_records(segs, d=0.08, k=0.1)])
    assert res["slow"]["n"] > 50 and res["moving"]["n"] > 50
    assert res["slow"]["d_alone"] == pytest.approx(0.08, abs=1e-3)
    assert res["all"]["d"] == pytest.approx(0.08, abs=2e-3) and res["all"]["k"] == pytest.approx(0.1, abs=0.01)
    assert res["all"]["rms_joint"] < 0.1 * res["all"]["rms_raw"]
    assert res["_definition"]["velocity"] == "mid-heading body frame (model_rollout._body)"


def test_replay_includes_the_transition_from_the_start_pose(tmp_path):
    # Task 9: a log's first State frame comes after B5's first cycle, so B5 also observed the transition from the start
    # pose (the hello's) into record 0. With `start` the replay includes it: a log missing its first record, replayed
    # from that record's pose, gives the full log's estimates (per record, aligned)
    prior = tmp_path / "prior.yaml"
    it.write_model(prior, dict(it.DEFAULTS, turn_delta_inf=-0.018, turn_delta_zero=-0.975, turn_scale=0.0225,
                               speed_turn=0.21, sideslip=0.196), {"date": "x", "source": "t", "terms": [], "held_out": {}})
    plant = dict(TRUE, turn_delta_inf=0.27, turn_delta_zero=-0.975, turn_scale=0.0225, speed_turn=0.21, sideslip=0.083)
    full = synthetic_records([(0.3, 0.2)] * 400, plant)
    est = it.replay_estimates(full, prior, container=None)
    tail = it.replay_estimates(full[1:], prior, container=None, start=full[0]["pose"][:3])
    assert len(tail) == len(full) - 1
    assert tail == est[1:]
    assert it.replay_estimates(full[1:], prior, container=None) != est[1:]  # without it: another window phase


def test_horizon_slices_count_the_launch_after_the_reset():
    # Task 9 review I1: a log starts at a reset, at rest. Its first motion is a launch even when it begins before
    # 0.5 s of quiet records; later motion after 0.5 s at rest is a restart; first_second is both
    cmds = [(0.0, 0.0)] * 10 + [(0.3, 0.2)] * 100 + [(0.0, 0.0)] * 40 + [(0.3, 0.2)] * 100
    recs = synthetic_records(cmds, TRUE)
    s = it.horizon_slices(recs)
    assert min(s["launch"]) == 11 and max(s["launch"]) == 60
    assert min(s["restart"]) == 151 and max(s["restart"]) == 200
    assert s["first_second"] == s["launch"] + s["restart"]
    short = synthetic_records([(0.0, 0.0)] * 10 + [(0.3, 0.2)] * 100 + [(0.0, 0.0)] * 10 + [(0.3, 0.2)] * 100, TRUE)
    assert it.horizon_slices(short)["restart"] == []  # 0.2 s at rest is no restart


def test_a_log_must_have_used_the_model_it_is_scored_with(tmp_path):
    # Task 9 review I4: "B5's own model" and the estimator replay need the file the run used (its hash in the hello);
    # another file is a counterfactual, refused unless asked for
    model = tmp_path / "m.yaml"
    model.write_text("sideslip: 0.1\n")
    import hashlib
    sha = hashlib.sha256(model.read_bytes()).hexdigest()
    used = [{"config": {"tracking_model_sha256": sha}}] * 2
    assert it.model_provenance(used, model) == {"model": str(model), "model_sha256": sha, "counterfactual": False}
    other = used + [{"config": {"tracking_model_sha256": "0" * 64}}]
    with pytest.raises(ValueError, match="counterfactual"):
        it.model_provenance(other, model)
    assert it.model_provenance(other, model, counterfactual=True)["counterfactual"] is True


# ---- spec Amendment 3: the turning-centre offset (Task 10b) ----

ARM_POSES = {"home": [0.0] * 6, "level_extension": [0.0, math.pi / 2, 0.0, 0.0, -math.pi / 2, 0.0],
             "carry": [0.0, 0.968, -1.735, 0.0, 0.767, 0.0]}
needs_model_step = pytest.mark.skipif(not mr.BINARY_PATH.exists(), reason="model_step is not built here (container)")


def test_map_parameters_carry_the_offset():
    p = it.map_parameters(dict(it.DEFAULTS, turn_offset=0.003, turn_offset_com=-2.23))
    assert len(p) == 7 and p[5:] == [0.003, -2.23]
    assert {"turn_offset", "turn_offset_com"} <= set(it.MODEL_KEYS)


@needs_model_step
def test_com_x_comes_from_the_generated_model():
    c = mr.com_x([ARM_POSES["home"], ARM_POSES["level_extension"]], container=None)
    assert c == pytest.approx([-0.0284, 0.0603], abs=1e-3)


@needs_model_step
def test_model_step_takes_the_offsets():
    # an in-place turn with turn_offset 0.05: the frame moves left by about 0.05 x w x dt
    params = it.map_parameters(dict(it.DEFAULTS, turn_offset=0.05))
    with mr.container_step(container=None, params=params) as step:
        x = step([(0.1, [0.0] * 9, [0.0, 0.3] + [0.0] * 6)])[0]
    assert x[1] == pytest.approx(0.05 * math.sin(0.03), abs=1e-9) and x[2] == pytest.approx(0.03)


@needs_model_step
def test_fit_recovers_the_turning_centre_offset():
    # segments at three arm poses, a plant whose turning centre follows the centre of mass (0.003 - 2.23 x_com) with
    # sideslip 0.2: the fit recovers both offset terms and the sideslip
    truth = dict(TRUE, turn_offset=0.003, turn_offset_com=-2.23)
    coms = dict(zip(ARM_POSES, mr.com_x(list(ARM_POSES.values()), container=None)))
    segs = []
    for k in range(9):
        pose = list(ARM_POSES)[k % 3]
        segs.append({"name": f"s{k}", "pose": pose, "role": "fit" if k < 6 else "held_out",
                     "records": synthetic_records(varied_commands(600, k), truth, k, ARM_POSES[pose], coms[pose])})
    res = it.fit(segs, ["turn_delta_inf", "turn_delta_zero", "turn_scale", "sideslip", "turn_offset",
                        "turn_offset_com"], container=None)
    p = res["parameters"]
    assert p["turn_offset"] == pytest.approx(0.003, abs=2e-3)
    assert p["turn_offset_com"] == pytest.approx(-2.23, abs=0.05)
    assert p["sideslip"] == pytest.approx(0.2, abs=0.02)


@pytest.mark.skipif(not TRACKING_REPLAY.exists(), reason="tracking_replay is not built here (runs in the container)")
def test_replay_takes_the_centre_of_mass_from_b5s_state(tmp_path):
    # Task 10c: the controller passes com_x of B5's state, whose arm is the streamed reference, not the physical joints
    # the log's pose carries. With an offset model the replay must use the record's b5 arm: a log whose pose arm differs
    # (home) replays like one whose pose arm is B5's (carry), and unlike one at home throughout
    prior = tmp_path / "prior.yaml"
    it.write_model(prior, dict(it.DEFAULTS, turn_delta_inf=-0.018, turn_delta_zero=-0.975, turn_scale=0.0225,
                               sideslip=0.0, turn_offset=0.01, turn_offset_com=-2.15),
                   {"date": "x", "source": "t", "terms": [], "held_out": {}})
    plant = dict(TRUE, turn_offset=0.05, sideslip=0.0)
    recs = synthetic_records([(0.3, 0.2)] * 400, plant, 0, ARM_POSES["carry"])
    mixed = [dict(r, pose=r["pose"][:3] + ARM_POSES["home"]) for r in recs]  # b5 keeps carry
    home = [dict(r, pose=r["pose"][:3] + ARM_POSES["home"], b5=r["b5"][:3] + ARM_POSES["home"]) for r in recs]
    est = it.replay_estimates(recs, prior, container=None)
    assert it.replay_estimates(mixed, prior, container=None) == est
    assert it.replay_estimates(home, prior, container=None) != est


def test_an_estimate_becomes_parameters_with_the_priors_offsets():
    # Task 10c: the online correction's estimate (g_inf, g_0, k) as map parameters keeps the prior's turn scale, speed
    # term and turning-centre offsets (not estimated); both B5's per-cycle model (T1) and the horizon's adapted model
    # (T11) are built this way
    prior = dict(TRUE, turn_offset=0.003, turn_offset_com=-2.23)
    assert it.estimate_parameters([0.9, 0.1, 0.3], prior) == pytest.approx(
        [-0.1, -0.9, TRUE["turn_scale"], TRUE["speed_turn"], 0.3, 0.003, -2.23])


@pytest.mark.skipif(not TRACKING_REPLAY.exists(), reason="tracking_replay is not built here (runs in the container)")
def test_replay_freezes_the_sideslip_as_the_run_did(tmp_path):
    # Task 10c: a run with tracking_adapt_sideslip false learned the turn gain alone; the replay must too, read from
    # the log's hello (absent: on, the Task 10 runs)
    prior = tmp_path / "prior.yaml"
    it.write_model(prior, dict(it.DEFAULTS, turn_delta_inf=-0.018, turn_delta_zero=-0.975, turn_scale=0.0225,
                               sideslip=0.0), {"date": "x", "source": "t", "terms": [], "held_out": {}})
    recs = synthetic_records([(0.3, 0.2)] * 400, dict(TRUE, sideslip=0.3), 0)
    learned = it.replay_estimates(recs, prior, container=None)
    frozen = it.replay_estimates(recs, prior, container=None, adapt_sideslip=False)
    assert learned[-1][2] > 0.1
    assert all(e[2] == 0.0 for e in frozen)
    assert frozen[-1][0] == pytest.approx(learned[-1][0], abs=0.05)  # the turn gain still learns
    assert it.adapt_sideslip_of({"config": {"tracking_adapt_sideslip": False}}) is False
    assert it.adapt_sideslip_of({"config": {}}) is True
