"""Tests of the plant calibrator's pure parts (MuJoCo plant plan Task 7) on synthetic signals."""
import json
from pathlib import Path

import numpy as np
import pytest

import calibrate_plant as cp


def test_cross_correlation_recovers_delay():
    t = np.arange(0.0, 10.0, 0.002)
    cmd = np.sin(1.3 * t) + 0.5 * np.sin(4.1 * t)
    out = np.interp(t - 0.04, t, cmd)  # the response 40 ms late
    assert cp.cross_correlation_delay(t, cmd, t, out) == pytest.approx(0.04, abs=0.002)


def test_first_order_fit_recovers_tau_and_gain():
    dt = 0.002
    t = np.arange(0.0, 20.0, dt)
    cmd = np.where((t % 4.0) < 2.0, 0.3, 0.0)
    y, out = 0.0, []
    a = 1.0 - np.exp(-dt / 0.15)
    for u in cmd:
        y += a * (0.93 * u - y)
        out.append(y)
    tau, k = cp.fit_first_order(t, cmd, t, np.array(out), delay=0.0)
    assert tau == pytest.approx(0.15, rel=0.02)
    assert k == pytest.approx(0.93, rel=0.02)


def test_split_keeps_cells_whole():
    runs = [Path(f"2026110{d}T000000Z-{cell}-wholebody-50hz-gff-mff-bff") for d in range(1, 4)
            for cell in ("v2_a_nav", "v2_b_nav", "v2_c_nav", "v2_d_nav", "v2_e_nav")]
    train, test = cp.split_by_cell(runs, held_out=0.2, seed=0)
    cells = lambda rs: {cp.cell_of(r) for r in rs}
    assert cells(train).isdisjoint(cells(test))
    assert len(cells(test)) == 1 and len(train) + len(test) == len(runs)


def test_monitor_windows_found():
    t = np.arange(0.0, 5.0, 0.02)
    smoothed = np.full_like(t, 0.3)
    sent = smoothed.copy()
    sent[(t > 1.0) & (t < 1.5)] = 0.0  # the collision monitor stopped the base
    windows = cp.monitor_windows(t, sent, t, smoothed)
    assert len(windows) == 1
    assert windows[0][0] == pytest.approx(1.0, abs=0.03) and windows[0][1] == pytest.approx(1.5, abs=0.03)


def test_select_runs_filters_configuration(tmp_path):
    for name, bag in (("20261101T000000Z-v2_a_nav-wholebody-50hz-gff-mff-bff", True),
                      ("20261101T000001Z-v2_a_nav-wholebody-50hz-gff-mff-bff-f2", True),
                      ("20261024T000000Z-v2_a_nav-wholebody-50hz-gff-mff", True),       # before the base feedforward
                      ("20261101T000002Z-v2_b_nav-wholebody-50hz-gff-mff-bff", False),  # no bag
                      ("20261101T000003Z-v2_a_nav-rpp", True)):                        # not B5
        d = tmp_path / name
        (d / "bag").mkdir(parents=True)
        (d / "summary.json").write_text("{}")
        if bag:
            (d / "bag" / "metadata.yaml").write_text("x")
    names = sorted(r.name for r in cp.select_runs(tmp_path))
    assert names == ["20261101T000000Z-v2_a_nav-wholebody-50hz-gff-mff-bff",
                     "20261101T000001Z-v2_a_nav-wholebody-50hz-gff-mff-bff-f2"]


def test_replay_input_from_series():
    series = {
        "t0": 100.0,
        "cmd_vel": [(99.5, 0.0, 0.0), (100.0, 0.0, 0.0), (100.02, 0.1, 0.05)],  # before the task: dropped
        "trajectories": [(100.1, [(0.0, [0.0] * 6, [0.0] * 6), (0.5, [0.1] * 6, [0.0] * 6)])],
        "odom": [(99.0, (0.0, 0.0, 0.0), (0.0, 0.0)), (100.0, (1.0, 2.0, 0.3), (0.0, 0.0))],
        "joints": [(99.0, [0.0] * 6), (100.0, [0.0, 0.968, -1.735, 0.0, 0.767, 0.0])],
        "end": 103.0,
    }
    replay = cp.replay_input(series)
    assert replay["start"] == {"base": [1.0, 2.0, 0.3], "q": [0.0, 0.968, -1.735, 0.0, 0.767, 0.0]}
    assert replay["duration_s"] == pytest.approx(3.0)
    assert replay["base_commands"] == [[0.0, 0.0, 0.0], [pytest.approx(0.02), 0.1, 0.05]]
    stream = replay["arm_streams"][0]
    assert stream["t"] == pytest.approx(0.1) and "start" not in stream  # stamp 0: on receipt
    assert stream["points"][1] == {"t": 0.5, "q": [0.1] * 6, "v": [0.0] * 6}
    json.dumps(replay)


def test_metrics_compare_like_for_like():
    t = np.arange(0.0, 4.0, 0.02)
    unity = {"t": t, "twist": np.stack([np.full_like(t, 0.3), np.zeros_like(t)], 1),
             "base": np.stack([0.3 * t, np.zeros_like(t), np.zeros_like(t)], 1),
             "q": np.zeros((len(t), 6)), "q_ref": np.zeros((len(t), 6))}
    plant = {k: v.copy() for k, v in unity.items()}
    plant["twist"][:, 0] += 0.01
    plant["base"][:, 0] = 0.31 * t
    plant["q"][:, 0] = 0.002
    m = cp.metrics(plant, unity)
    assert m["base_speed_rmse"] == pytest.approx(0.01, abs=1e-9)
    assert m["drift_2s_max"] == pytest.approx(0.02, abs=1e-3)  # 0.01 m/s for 2 s
    assert m["arm_p95"][0]["plant"] == pytest.approx(0.002)
    assert m["arm_p95"][0]["unity"] == pytest.approx(0.0)


def test_spread_takes_cells_in_turn():
    runs = [Path(f"2026110{d}T000000Z-{cell}-wholebody-50hz-gff-mff-bff") for cell in ("v2_a_nav", "v2_b_nav") for d in range(1, 5)]
    picked = cp.spread(runs, 3)
    assert [cp.cell_of(r) for r in picked] == ["v2_a_nav", "v2_b_nav", "v2_a_nav"]


def test_skid_map_recovers_the_turn_dependence():
    rng = np.random.default_rng(1)
    v = rng.uniform(0.06, 0.3, 20000)
    w = rng.uniform(-0.8, 0.8, 20000)
    share = np.abs(w) * 0.75 / v
    kv = np.interp(share, [0.0, 1.0, 2.0], [0.93, 0.9, 0.7])
    kw = np.interp(share, [0.0, 1.0, 2.0], [0.8, 1.0, 1.2])
    samples = np.stack([v, w, kv * v, kw * w, np.arange(20000) // 50], 1)  # 400 segments of 50 samples
    skid = cp.fit_skid_map(samples, knots=[0.15, 0.5, 1.0, 1.5, 2.0])
    assert skid["share"] == [0.15, 0.5, 1.0, 1.5, 2.0]
    assert np.interp(1.0, skid["share"], skid["v"]) == pytest.approx(0.9, abs=0.02)
    assert np.interp(1.0, skid["share"], skid["w"]) == pytest.approx(1.0, abs=0.02)
    assert np.interp(2.0, skid["share"], skid["w"]) == pytest.approx(1.2, abs=0.03)
    assert skid["w_in_place_source"].startswith("Unity measurement")  # no samples turning in place


def segment_series(pieces, period=0.02, odom_lag=0.0):
    """A recording made of (duration, v, w) pieces: commands every 20 ms, odometry achieving k_v v and k_w w."""
    t, cmd, odom, now = [], [], [], 0.0
    for duration, v, w, kv, kw in pieces:
        for k in range(int(round(duration / period))):
            cmd.append((now, v, w))
            odom.append((now + odom_lag, (0.0, 0.0, 0.0), (kv * v, kw * w)))
            now += period
    return {"cmd_vel": cmd, "odom": odom}


def test_steady_samples_need_the_command_held_through_the_reading():
    # 2 s of 0.3 m/s, then a jump to 0.1: samples whose reading window meets the jump are not steady
    s = segment_series([(2.0, 0.3, 0.0, 0.9, 1.0), (2.0, 0.1, 0.0, 0.9, 1.0)])
    samples = cp.steady_samples(s)
    assert len(samples) > 0
    assert np.allclose(samples[:, 2] / samples[:, 0], 0.9)  # no sample mixes the two commands
    assert set(np.unique(samples[:, 4]).astype(int)) == {0, 1}  # two distinct segments


def test_steady_samples_use_relative_tolerances():
    # a small yaw command that wanders by more than itself is not steady
    t = np.arange(0.0, 4.0, 0.02)
    s = {"cmd_vel": [(x, 0.3, 0.03 * (1 + np.sin(5 * x))) for x in t], "odom": [(x, (0, 0, 0), (0.3, 0.03)) for x in t]}
    assert len(cp.steady_samples(s)) == 0


def test_skid_knots_need_distinct_segments():
    # many samples from one segment do not make a knot: the bin takes its neighbour's value
    rows = [[0.3, 0.0, 0.27, 0.0, i // 50] for i in range(500)] + [[0.2, 0.3, 0.19, 0.33, 100 + i // 50] for i in range(500)]
    skid = cp.fit_skid_map(np.array(rows), knots=[0.0, 1.0])
    assert skid["v"][0] == pytest.approx(0.9)
    one_segment = [[0.2, 0.3, 0.1, 0.6, 7]] * 500
    skid = cp.fit_skid_map(np.array(rows[:500] + one_segment), knots=[0.0, 1.0])
    assert skid["v"][1] == skid["v"][0]  # filled, not fitted from one segment
    assert skid["bins"][1]["segments"] == 1


def test_in_place_without_data_takes_unitys_measurement():
    skid = cp.fit_skid_map(np.array([[0.3, 0.0, 0.27, 0.0, 0]] * 100), knots=[0.0])
    assert skid["w_in_place"] == pytest.approx(0.442 / 0.4)
    assert skid["w_in_place_source"] == "Unity measurement (docs/unity-skid-steer-base-controller.md)"


def test_transport_latency_from_stamps():
    clock = [(10.0 + 0.1 * k, 5.0 + 0.1 * k) for k in range(100)]  # bag time = sim time + 5 s
    stamped = [(10.0 + 0.1 * k + 0.012, 5.0 + 0.1 * k) for k in range(90)]  # received 12 ms after their stamp
    assert cp.transport_latency(stamped, clock) == pytest.approx(0.012, abs=1e-9)


def test_task_window_from_the_sim_clock():
    clock = [(10.0 + 0.1 * k, 5.0 + 0.1 * k) for k in range(300)]
    assert cp.task_window({"started_s": 7.0, "finished_s": 20.0}, clock) == pytest.approx((12.0, 25.0))


def test_stiffness_keeps_unitys_damping_ratio():
    params = cp.expand({"stiffness.1": 4.0, "arm_command_delay_s": 0.02})
    assert params["kp.1"] == pytest.approx(4.0 * cp.UNITY_KP[1])
    assert params["kd.1"] == pytest.approx(2.0 * cp.UNITY_KD[1])  # sqrt(4): same damping ratio
    assert params["arm_command_delay_s"] == 0.02 and "stiffness.1" not in params


def test_write_profile_is_idempotent(tmp_path, monkeypatch):
    profile = tmp_path / "profile.yaml"
    profile.write_text(cp.PROFILE.read_text())
    monkeypatch.setattr(cp, "PROFILE", profile)
    fitted = {"base_command_delay_s": 0.02, "arm_command_delay_s": 0.01, "base_state_delay_s": 0.011,
              "arm_state_delay_s": 0.02, "base_lag_s": [0.1, 0.05], "base_slip": [1.0, 1.0],
              "base_skid": {"share": [0.0, 1.0], "v": [0.93, 0.8], "w": [0.9, 1.2], "w_in_place": 1.1},
              "stiffness.0": 2.0}
    cp.write_profile(fitted, "fit.json", "2026-11-05", ["run_a", "run_b"])
    once = profile.read_text()
    cp.write_profile(fitted, "fit.json", "2026-11-05", ["run_a", "run_b"])
    assert profile.read_text() == once
    import yaml
    loaded = yaml.safe_load(once)
    assert loaded["realistic"]["base_skid"]["v"] == [0.93, 0.8]
    assert loaded["arm"]["kp"][0] == pytest.approx(2.0 * cp.UNITY_KP[0])
    assert once.count("Calibrated by tools/plant/calibrate_plant.py") == 1 and "Placeholders" not in once


def test_arm_objective_is_the_joint_trajectory_and_the_tracking_error():
    # the arm is fitted on where the joints went (RMSE against Unity's) and on how far behind the reference they were
    # (Section 7.3's p95): the size of the error alone lets a soft, late arm move differently; the joints alone miss the
    # JTC's view of its own lag
    m = {"base_speed_rmse": 0.01, "yaw_rate_rmse": 0.02, "arm_q_rmse": [0.001, 0.002, 0.003, 0.0, 0.0, 0.0],
         "arm_p95": [{"plant": 0.004, "unity": 0.01}] + [{"plant": 0.0, "unity": 0.0}] * 5}
    assert cp.objective(m, "arm") == pytest.approx(0.001 + 0.006 / 6)
    assert cp.objective(m, "base") == pytest.approx(0.03)


def test_search_keeps_the_start_on_a_flat_objective():
    # a value moves away from its start (Unity's gains) only for a real improvement: >= 1 % of the objective
    score = lambda cases, p, part: 1.0 - 0.001 * abs(p["stiffness.0"] - 1.0) - 0.05 * (p["arm_command_delay_s"] == 0.03)
    params, best = cp.coordinate_search([], {"stiffness.0": 1.0, "arm_command_delay_s": 0.0}, "arm",
                                        [("stiffness.0", [0.125, 1.0, 8.0]), ("arm_command_delay_s", [0.0, 0.03])],
                                        evaluate_fn=score)
    assert params == {"stiffness.0": 1.0, "arm_command_delay_s": 0.03}


def test_arm_allowance_covers_unitys_jitter_floor():
    # Unity's tracking error has a 1-2.7 mrad floor at rest (PhysX solver jitter) that the plant does not reproduce:
    # the absolute allowance is 3 mrad (user, CP3 2026-11-06)
    quiet = {"base_speed_rmse": 0.0, "yaw_rate_rmse": 0.0, "drift_2s_max": 0.0,
             "arm_p95": [{"plant": 0.0001, "unity": 0.0027}] + [{"plant": 0.0, "unity": 0.0}] * 5}
    assert cp.passes(quiet)["arm"]
    quiet["arm_p95"][0] = {"plant": 0.0001, "unity": 0.0032}
    assert not cp.passes(quiet)["arm"]


def test_arm_fit_replays_whole_tasks():
    # the base fit's 40 s limit keeps the long base search cheap; the arm fit needs the arm's motion, often later in a task
    cases = [({"duration_s": 120.0, "full_duration_s": 120.0}, None)]
    limited = cp.limit_cases(cases, 40.0)
    assert limited[0][0]["duration_s"] == 40.0
    assert cp.limit_cases(limited, None)[0][0]["duration_s"] == 120.0


def test_plant_replay_runs_the_calibrated_profile(monkeypatch):
    # the binary's default is the URDF-based profile.yaml (a gain rule, wheels): the calibration replays its own profile
    import json as _json
    seen = {}

    def fake_run(cmd, **kwargs):
        seen["cmd"] = cmd
        out = cmd[cmd.index("--out") + 1]
        Path(out).write_text(_json.dumps({"t": [0.0]}))

    monkeypatch.setattr(cp.subprocess, "run", fake_run)
    cp.plant_replay({"duration_s": 1.0}, {})
    assert seen["cmd"][seen["cmd"].index("--profile") + 1] == str(cp.PROFILE)
    assert cp.PROFILE.name == "profile_unity.yaml"
    cp.plant_replay({"duration_s": 1.0}, {}, profile="other.yaml")
    assert seen["cmd"][seen["cmd"].index("--profile") + 1] == "other.yaml"
