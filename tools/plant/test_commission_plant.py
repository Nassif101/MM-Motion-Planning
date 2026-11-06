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


def test_bandwidth_keeps_the_stated_value_and_reports_the_margins():
    # the user's ruling (2026-11-06): the stated 10 Hz stands while it is stable and well damped (every joint's step
    # overshoot <= 5 %); the stability and damping limits are reported, not chosen
    def r(stable, overshoot):
        return {"stable": stable, "joints": [{"overshoot": overshoot}] * 6}
    results = {5.0: r(True, 0.0), 10.0: r(True, 0.01), 15.0: r(True, 0.02), 20.0: r(True, 0.17), 30.0: r(True, 0.39),
               40.0: r(False, 0.53)}
    chosen, margins = cm.bandwidth_ruling(results, proposed=10.0)
    assert chosen == 10.0 and margins == {"stable_to_hz": 30.0, "well_damped_to_hz": 15.0}
    chosen, _ = cm.bandwidth_ruling(results, proposed=20.0)  # not well damped: the highest that is
    assert chosen == 15.0


def test_replay_runs_the_urdf_based_profile(monkeypatch):
    seen = {}
    monkeypatch.setattr(cm.cp, "plant_replay", lambda inp, params, profile=None: seen.setdefault("profile", profile))
    cm.replay({}, {})
    assert seen["profile"] == cm.PROFILE and cm.PROFILE.name == "profile.yaml"


def test_step_metrics():
    # an underdamped second-order step 0 -> 0.05 rad at t = 0.1 s: 30 % overshoot, settles within 2 %
    t = np.arange(0.0, 2.0, 0.002)
    zeta, wn = 0.358, 40.0  # overshoot exp(-pi zeta / sqrt(1 - zeta^2)) = 0.30
    tau = np.clip(t - 0.1, 0.0, None)
    wd = wn * np.sqrt(1 - zeta ** 2)
    y = 0.05 * (1 - np.exp(-zeta * wn * tau) * (np.cos(wd * tau) + zeta / np.sqrt(1 - zeta ** 2) * np.sin(wd * tau)))
    m = cm.step_metrics(t, y, 0.0, 0.05, 0.1)
    assert m["overshoot"] == pytest.approx(0.30, abs=0.01)
    assert m["settling_s"] == pytest.approx(4.0 / (zeta * wn), rel=0.25)  # the 2 % envelope rule of thumb
    assert cm.step_metrics(t, np.full_like(t, 0.0), 0.0, 0.05, 0.1)["settling_s"] is None  # never got there


def test_sine_metrics():
    t = np.arange(0.0, 4.0, 0.002)
    ref = 0.05 * np.sin(2 * np.pi * 1.0 * t)
    q = 0.8 * 0.05 * np.sin(2 * np.pi * 1.0 * t - 0.2)
    m = cm.sine_metrics(t, q, ref, 1.0)
    assert m["gain"] == pytest.approx(0.8, abs=1e-3) and m["phase_lag_rad"] == pytest.approx(0.2, abs=1e-3)


def test_arm_move_measures_the_servo_alone(monkeypatch):
    # the reference reaches the drives arm_command_delay_s late: the servo test removes the delay, or the error is the
    # delay's (the Task 7b review)
    seen = {}

    def fake(inp, params):
        seen.update(params)
        n = 10
        return {"t": np.linspace(0, 1, n), "q": np.zeros((n, 6)), "q_ref": np.zeros((n, 6)), "qd": np.zeros((n, 6))}

    monkeypatch.setattr(cm, "replay", fake)
    cm.arm_move(10.0, {})
    assert seen["arm_command_delay_s"] == 0.0 and seen["gain_bandwidth_hz"] == 10.0


def test_within_limits():
    lower, upper = np.array([-1.0] * 6), np.array([1.0] * 6)
    assert cm.within_limits(np.zeros((5, 6)), lower, upper)
    q = np.zeros((5, 6))
    q[3, 2] = 1.01
    assert not cm.within_limits(q, lower, upper)


def test_acceptance_c_flags_turn_ratios():
    base = {"turns": [(0.3, 0.1, 0.3, 0.079), (0.3, 0.3, 0.3, 0.29), (0.0, 0.6, 0.0, 0.52), (0.0, 0.2, 0.0, 0.2)],
            "straight": {"stop_s": 0.5, "limit_stop_s": 0.375}, "drift_30s": {"xy_m": 1e-5, "yaw_rad": 1e-5},
            "breakaway": {"yaw_radps": 0.02}}
    arm = {"stable_everywhere": True}
    unity = {"held_out": [{"arm_stable": True, "within_limits": True}], "fresh": [{"arm_stable": True, "within_limits": True}]}
    rows = {r["check"]: r for r in cm.acceptance_c(base, arm, unity, assumptions={"a": {"checked": [1, 2]}})}
    turns = rows["steady commands within +-10 % (|omega| <= 0.6)"]
    assert not turns["pass"] and "(0.3, 0.1)" in turns["detail"] and "(0.0, 0.6)" in turns["detail"]
    assert rows["turning in place from 0.2 rad/s"]["pass"]
    assert rows["stops within the deceleration limit + 0.2 s, drift < 1 mm/min"]["pass"]
    assert rows["arm stable at rest and on every recorded stream, inside the URDF limits"]["pass"]
    assert rows["every assumption has a source and a sensitivity result"]["pass"]
    missing = cm.acceptance_c(base, arm, unity, assumptions={"a": {"checked": [1, 2]}, "b": {"value": 1}})
    assert not next(r for r in missing if r["check"].startswith("every assumption"))["pass"]


def test_settled_judges_a_saturating_move_by_its_end():
    # an effort-limited move lags far behind its reference on the way: stable means it ends on target, at rest
    t = np.linspace(0.0, 3.0, 1501)
    target = np.array([0.5] * 6)
    q = np.outer(np.clip(t / 0.8, 0.0, 1.0), target)            # lags 0.4 rad behind a 0.15 s reference
    qd = np.gradient(q, t, axis=0)
    assert cm.settled({"t": t, "q": q, "qd": qd}, target)
    wobble = q + 0.01 * np.sin(2 * np.pi * 20 * t)[:, None]     # a sustained 20 Hz oscillation
    assert not cm.settled({"t": t, "q": wobble, "qd": np.gradient(wobble, t, axis=0)}, target)


def test_acceptance_c_shows_the_accepted_deviations():
    base = {"turns": [(0.3, 0.1, 0.3, 0.079), (0.0, 0.2, 0.0, 0.2)], "straight": {"stop_s": 0.3, "limit_stop_s": 0.375},
            "drift_30s": {"xy_m": 0.0, "yaw_rad": 0.0}, "breakaway": {"yaw_radps": 0.02}}
    unity = {"held_out": [{"arm_stable": True, "within_limits": True}]}
    rows = {r["check"]: r for r in cm.acceptance_c(base, {"stable_everywhere": True}, unity, assumptions={"torque_speed": {"value": "flat"}})}
    turns = rows["steady commands within +-10 % (|omega| <= 0.6)"]
    assert not turns["pass"] and "user 2026-11-06" in turns["accepted"]
    assert "user 2026-11-06" in rows["every assumption has a source and a sensitivity result"]["accepted"]
    assert "accepted" not in rows["turning in place from 0.2 rad/s"]


def test_acceptance_is_pinned_to_what_the_user_accepted():
    # the ruling covers the two turns at their measured ratios and torque_speed alone: any other failure is a failure
    def rows(turns, assumptions):
        base = {"turns": turns, "straight": {"stop_s": 0.3, "limit_stop_s": 0.375},
                "drift_30s": {"xy_m": 0.0, "yaw_rad": 0.0}, "breakaway": {"yaw_radps": 0.02}}
        unity = {"held_out": [{"arm_stable": True, "within_limits": True}]}
        return {r["check"]: r for r in cm.acceptance_c(base, {"stable_everywhere": True}, unity, assumptions)}
    accepted_turns = [(0.3, 0.1, 0.3, 0.079), (0.0, 0.6, 0.0, 0.52), (0.0, 0.2, 0.0, 0.2)]
    turns = "steady commands within +-10 % (|omega| <= 0.6)"
    assert "accepted" in rows(accepted_turns, {})[turns]
    worse = accepted_turns + [(0.3, 0.3, 0.3, 0.15)]  # a new failure (x0.5)
    assert "accepted" not in rows(worse, {})[turns]
    drifted = [(0.3, 0.1, 0.3, 0.065), (0.0, 0.6, 0.0, 0.52), (0.0, 0.2, 0.0, 0.2)]  # x0.65: beyond the accepted x0.79
    assert "accepted" not in rows(drifted, {})[turns]
    assumption = "every assumption has a source and a sensitivity result"
    assert "accepted" in rows(accepted_turns, {"torque_speed": {"value": "flat"}})[assumption]
    assert "accepted" not in rows(accepted_turns, {"torque_speed": {"value": "flat"}, "new": {"value": 1}})[assumption]
