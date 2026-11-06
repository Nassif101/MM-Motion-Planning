"""Tests of the plant commissioning's pure parts (MuJoCo plant amendment 2026-11-06, plan Task 7b) on synthetic outputs."""
import numpy as np
import pytest

import commission_plant as cm


def test_steady_input_holds_one_command():
    replay = cm.steady_input(0.3, 0.2, 8.0, [0.0] * 6)
    assert replay["world"] == "ground" and replay["duration_s"] == 8.0
    assert replay["base_commands"][0] == [0.0, 0.3, 0.2] and replay["base_commands"][-1][1:] == [0.3, 0.2]
    assert replay["base_commands"][-1][0] >= 7.9  # refreshed: the watchdog never ages it out
    assert replay["arm_streams"] == []


def test_steady_reading_averages_the_tail():
    t = np.arange(0.0, 8.0, 0.02)
    twist = np.stack([np.where(t < 2, t / 2 * 0.3, 0.3), np.where(t < 2, 0.0, 0.2)], 1)
    v, w = cm.steady_reading({"t": t, "twist": twist}, tail_s=2.0)
    assert v == pytest.approx(0.3) and w == pytest.approx(0.2)


def test_effective_track_minimizes_the_yaw_error():
    # achieved yaw rate = command * (b / 2.5): exact at 2.5
    grid = [(0.3, 0.2), (0.0, 0.4), (0.15, 0.4)]
    table = {b: [(v, w, v, w * b / 2.5) for v, w in grid] for b in (2.0, 2.5, 3.0)}
    best, scores = cm.fit_effective_track(table)
    assert best == 2.5 and scores[2.5] == pytest.approx(0.0)


def test_breakaway_is_the_smallest_command_that_moves():
    moved = {0.01: False, 0.02: False, 0.03: True, 0.05: True}
    assert cm.breakaway(moved) == 0.03
    assert cm.breakaway({0.01: False}) is None


def test_stop_time_from_the_command():
    t = np.arange(0.0, 4.0, 0.02)
    v = np.where(t < 2.0, 0.3, np.maximum(0.0, 0.3 - 0.8 * (t - 2.0)))
    assert cm.stop_time({"t": t, "twist": np.stack([v, 0 * v], 1)}, 2.0, tol=0.01) == pytest.approx(0.375, abs=0.02)


def test_reachable_envelope():
    assert cm.rim_speed(8.0, 0.14) == pytest.approx(1.12)
    assert cm.reachable(0.3, 0.4, 2.5, 8.0, 0.14) and not cm.reachable(0.5, 0.8, 2.5, 8.0, 0.14)


def test_highest_stable_bandwidth():
    results = {5.0: {"stable": True}, 10.0: {"stable": True}, 15.0: {"stable": False}}
    assert cm.choose_bandwidth(results) == 10.0


def test_stability_judgement():
    t = np.arange(0.0, 3.0, 0.002)
    calm = {"t": t, "q": np.zeros((len(t), 6)), "q_ref": np.zeros((len(t), 6)), "qd": np.zeros((len(t), 6))}
    assert cm.stable(calm)
    ringing = {k: v.copy() for k, v in calm.items()}
    ringing["qd"][:, 2] = 0.01 * np.sin(2 * np.pi * 40 * t)  # a 40 Hz limit cycle at rest
    assert not cm.stable(ringing)


def test_write_values_is_idempotent(tmp_path, monkeypatch):
    import commission_plant as c2
    profile = tmp_path / "profile.yaml"
    profile.write_text(c2.PROFILE.read_text())
    monkeypatch.setattr(c2, "PROFILE", profile)
    c2.write_values(2.75, 12.0, "2026-11-06", 80.0)
    once = profile.read_text()
    c2.write_values(2.75, 12.0, "2026-11-06", 80.0)
    assert profile.read_text() == once
    import yaml
    loaded = yaml.safe_load(once)
    assert loaded["base"]["effective_track_m"] == 2.75 and loaded["arm"]["gain_rule"]["bandwidth_hz"] == 12.0
    assert loaded["base"]["wheel_kv"] == 80.0
    assert once.count("Commissioned by tools/plant/commission_plant.py") == 3


def test_wheel_kv_is_the_stiffest_that_follows_the_ramp():
    # MuJoCo's soft contact: a wheel loop much stiffer than it overshoots the limiter's ramp
    assert cm.choose_wheel_kv({30.0: 0.004, 50.0: 0.007, 100.0: 0.024, 200.0: 0.058}, tol=0.02) == 50.0
    assert cm.choose_wheel_kv({100.0: 0.03}, tol=0.02) is None


def test_bandwidth_keeps_the_proposal_when_the_rule_does_not_bind():
    # every candidate stable with errors within 1 mrad of each other: the stated assumption stands (not the grid's top)
    flat = {b: {"stable": True, "still": {"max_error_rad": [0.011] * 6}, "driving": {"max_error_rad": [0.0115] * 6}}
            for b in (5.0, 10.0, 15.0)}
    assert cm.bandwidth_ruling(flat, proposed=10.0) == (10.0, "does not bind")
    binding = {**flat, 15.0: {"stable": False, "still": {"max_error_rad": [0.011] * 6}, "driving": {"max_error_rad": [0.02] * 6}}}
    assert cm.bandwidth_ruling(binding, proposed=10.0) == (10.0, "highest stable")


def test_replay_runs_the_urdf_based_profile(monkeypatch):
    seen = {}
    monkeypatch.setattr(cm.cp, "plant_replay", lambda inp, params, profile=None: seen.setdefault("profile", profile))
    cm.replay({}, {})
    assert seen["profile"] == cm.PROFILE and cm.PROFILE.name == "profile.yaml"
