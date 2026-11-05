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
        "cmd_vel": [(100.0, 0.0, 0.0), (100.02, 0.1, 0.05)],
        "trajectories": [(100.1, [(0.0, [0.0] * 6, [0.0] * 6), (0.5, [0.1] * 6, [0.0] * 6)])],
        "odom": [(100.0, (1.0, 2.0, 0.3), (0.0, 0.0))],
        "joints": [(100.0, [0.0, 0.968, -1.735, 0.0, 0.767, 0.0])],
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
