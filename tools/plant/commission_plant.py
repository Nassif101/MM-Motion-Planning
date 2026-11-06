"""Commission the URDF-based realistic plant (MuJoCo plant amendment 2026-11-06, Section 5; plan Task 7b).

The plant is measured the way a real robot is commissioned, through plant_replay with synthetic commands on the
ground-only world:
- base: the effective track that makes steady turns achieve their commanded yaw rate (least squares over a grid of
  candidates), the steady (v, omega) table, acceleration and stopping, drift at rest, breakaway from rest, the reachable
  envelope under the wheel-speed cap against the planners' envelope, and the sensitivity to the ground's friction;
- arm: the servo bandwidth (the highest that stays stable under the torque clamp, moving and at rest, with the base
  still or driving), and the sensitivity to the armature;
- the Unity comparison: the held-out and fresh runs of calibrate_plant.py replayed on this profile (reported, not
  gated: docs/superpowers/specs/2026-11-06-mujoco-plant-amendment-urdf-realistic.md Section 6).
Writes docs/experiments/mujoco-plant/commissioning.{md,json}; --write-profile writes the effective track and the
bandwidth into the profile with their provenance.

Runs in the container (the plant_replay binary, numpy, rosbags for the Unity comparison):
    python3 tools/plant/commission_plant.py all --out docs/experiments/mujoco-plant/ [--write-profile] [--skip-unity]
"""
import argparse
import datetime as dt
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

import calibrate_plant as cp

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / "ros2_ws/src/mobile_manipulator_plant/robots/mobile_manipulator/profile.yaml"
ENVELOPE = ROOT / "ros2_ws/src/mobile_manipulator_navigation/config/nav_operating_envelope.yaml"
PAYLOAD = ROOT / "ros2_ws/src/mobile_manipulator_control/config/qualified_payload.json"
TRACKS = [1.5, 2.0, 2.25, 2.5, 2.75, 3.0, 3.5, 4.0]
WHEEL_KVS = [30.0, 50.0, 80.0, 100.0, 150.0, 200.0, 500.0, 1000.0]
PROPOSED_BANDWIDTH_HZ = 10.0  # the profile's stated assumption
TURNS = [(0.3, 0.1), (0.3, 0.2), (0.3, 0.3), (0.3, 0.4), (0.15, 0.3), (0.15, 0.4), (0.0, 0.2), (0.0, 0.4), (0.0, 0.6)]
BANDWIDTHS = [5.0, 8.0, 10.0, 12.0, 15.0, 20.0, 25.0, 30.0, 40.0]  # up to beyond the stability limit
STEP = [0.3, -0.2, 0.3, 0.3, 0.3, 0.5]  # the arm's test move from carry (rad per joint)


# --------------------------------------------------------------------------------------------------------- pure parts
def steady_input(v, w, duration_s, q, base_start=(0.0, 0.0, 0.0)):
    """A replay holding (v, w) for duration_s (refreshed every 20 ms), the arm holding q, on the ground alone."""
    return {"start": {"base": list(base_start), "q": list(q)}, "duration_s": duration_s, "sample_period_s": 0.02,
            "world": "ground", "base_commands": [[round(t, 6), v, w] for t in np.arange(0.0, duration_s, 0.02)],
            "arm_streams": []}


def steady_reading(out, tail_s=2.0):
    """The mean (v, omega) over the output's last tail_s."""
    t, twist = np.asarray(out["t"]), np.asarray(out["twist"])
    tail = t >= t[-1] - tail_s
    return float(twist[tail, 0].mean()), float(twist[tail, 1].mean())


def fit_effective_track(table):
    """table: {b: [(v, w, v_achieved, w_achieved)]} -> (the b whose turns (|w| >= 0.1) best achieve their commanded yaw
    rate in the least-squares sense of the ratio, {b: mean squared ratio error})."""
    scores = {b: float(np.mean([(wa / w - 1.0) ** 2 for v, w, va, wa in rows if abs(w) >= 0.1])) for b, rows in table.items()}
    return min(scores, key=scores.get), scores


def breakaway(moved):
    """The smallest command from which every larger one moved the base ({command: moved}); None if the largest didn't."""
    keys = sorted(moved)
    best = None
    for k in reversed(keys):
        if not moved[k]:
            break
        best = k
    return best


def stop_time(out, t_stop, tol=0.01):
    """Seconds from t_stop until |v| < tol (None if never)."""
    t, v = np.asarray(out["t"]), np.asarray(out["twist"])[:, 0]
    idx = np.where((t >= t_stop) & (np.abs(v) < tol))[0]
    return float(t[idx[0]] - t_stop) if len(idx) else None


def rim_speed(wheel_speed_max, wheel_radius_m):
    return wheel_speed_max * wheel_radius_m


def reachable(v, w, track, wheel_speed_max, wheel_radius_m):
    """Whether (v, w) needs no wheel above the cap: |v| + |w| b / 2 <= cap r."""
    return abs(v) + abs(w) * track / 2.0 <= rim_speed(wheel_speed_max, wheel_radius_m) + 1e-12


def stable(out, tail_s=1.0, rest_qd=1e-3, max_error=0.1):
    """At rest at the end: finite, the joints' velocity RMS over the last tail_s below rest_qd on every joint, and the
    tracking error never above max_error."""
    t, q, q_ref, qd = (np.asarray(out[k]) for k in ("t", "q", "q_ref", "qd"))
    if not (np.isfinite(q).all() and np.isfinite(qd).all()):
        return False
    tail = t >= t[-1] - tail_s
    return bool(np.sqrt(np.mean(qd[tail] ** 2, 0)).max() < rest_qd and np.abs(q - q_ref).max() < max_error)


def settled(out, target, tail_s=1.0, rest_qd=1e-3, tol=2e-3):
    """A move that saturated the efforts (its error on the way is the effort limit's, not instability): stable when it
    ends at rest on target: finite, the joints' velocity RMS over the last tail_s below rest_qd, within tol rad."""
    t, q, qd = (np.asarray(out[k]) for k in ("t", "q", "qd"))
    if not (np.isfinite(q).all() and np.isfinite(qd).all()):
        return False
    tail = t >= t[-1] - tail_s
    return bool(np.sqrt(np.mean(qd[tail] ** 2, 0)).max() < rest_qd and np.abs(q[-1] - np.asarray(target)).max() < tol)


def choose_bandwidth(results):
    """The highest bandwidth judged stable ({bandwidth: {"stable": bool, ...}})."""
    ok = [b for b, r in results.items() if r["stable"]]
    return max(ok) if ok else None


def choose_wheel_kv(ramp_errors, tol=0.02):
    """The stiffest wheel-loop kv whose base follows the limiter's acceleration ramp within tol m/s ({kv: max error}):
    MuJoCo's contact is soft (20 ms), and a wheel loop much stiffer than it overshoots; None if none does."""
    ok = [kv for kv, err in ramp_errors.items() if err <= tol]
    return max(ok) if ok else None


WELL_DAMPED_OVERSHOOT = 0.05  # every joint's small-step overshoot at most this


def bandwidth_ruling(results, proposed):
    """(bandwidth, margins). The user's ruling (2026-11-06): the stated bandwidth stands while it is stable and well
    damped (every joint's step overshoot <= 5 %); else the highest that is. The stability limit (highest stable) and the
    damping limit (highest stable and well damped) are reported, not chosen: in simulation the PD loop stays stable far
    above what a geared arm's position loop does (here to 30 Hz, the wrists ringing from 20 Hz)."""
    ok = {b for b, r in results.items() if r["stable"]}
    damped = {b for b in ok if all(j["overshoot"] <= WELL_DAMPED_OVERSHOOT for j in results[b]["joints"])}
    margins = {"stable_to_hz": max(ok) if ok else None, "well_damped_to_hz": max(damped) if damped else None}
    if proposed in damped:
        return proposed, margins
    below = [b for b in damped if b <= proposed]
    return (max(below) if below else margins["well_damped_to_hz"]), margins

def write_values(track, bandwidth, date, wheel_kv):
    """Writes the effective track, the wheel loop's kv and the arm bandwidth into the profile, each with a provenance
    line; idempotent."""
    text = PROFILE.read_text()
    text = re.sub(r"^  # Commissioned by tools/plant/commission_plant\.py.*\n", "", text, flags=re.M)
    line = f"  # Commissioned by tools/plant/commission_plant.py on {date} (docs/experiments/mujoco-plant/commissioning.md)\n"
    text, n1 = re.subn(r"^  effective_track_m: .*\n", f"{line}  effective_track_m: {track}\n", text, count=1, flags=re.M)
    text, n2 = re.subn(r"^  gain_rule: \{bandwidth_hz: [^,]*,", f"{line}  gain_rule: {{bandwidth_hz: {bandwidth},", text, count=1, flags=re.M)
    text, n3 = re.subn(r"^  wheel_kv: .*\n", f"{line}  wheel_kv: {wheel_kv}\n", text, count=1, flags=re.M)
    if n1 != 1 or n2 != 1 or n3 != 1:
        raise RuntimeError("profile: no effective_track_m, gain_rule or wheel_kv line")
    PROFILE.write_text(text)


# ------------------------------------------------------------------------------------------------------- measurement
def pose(name):
    return json.loads(PAYLOAD.read_text())["poses_rad"][name]


def carry():
    return pose("carry")


def replay(inp, params):
    return cp.plant_replay(inp, params, profile=PROFILE)


def turn_table(track, params, turns=TURNS, seconds=8.0, q=None):
    rows = []
    for v, w in turns:
        out = replay(steady_input(v, w, seconds, q or carry()), {**params, "effective_track_m": track})
        rows.append((v, w, *steady_reading(out)))
    return rows


def ramp_error(kv, params):
    """The largest |v - ramp| while the limiter ramps 0 -> 0.3 m/s at 0.5 m/s^2 (1.5 s)."""
    out = replay(steady_input(0.3, 0.0, 1.5, carry()), {**params, "wheel_kv": kv})
    t, v = np.asarray(out["t"]), np.asarray(out["twist"])[:, 0]
    return float(np.abs(v - np.minimum(0.3, 0.5 * t)).max())


def base_commissioning(params):
    result = {}
    errors = {kv: ramp_error(kv, {**params, "effective_track_m": 2.5}) for kv in WHEEL_KVS}
    result["wheel_kv_ramp_error"] = {str(k): v for k, v in errors.items()}
    result["wheel_kv"] = kv = choose_wheel_kv(errors)
    if kv is None:
        raise RuntimeError(f"no wheel kv follows the ramp within 0.02 m/s: {errors}")
    params = {**params, "wheel_kv": kv}
    table = {b: turn_table(b, params) for b in TRACKS}
    best, scores = fit_effective_track(table)
    result["track_scores"] = scores
    result["effective_track_m"] = best
    result["turns"] = table[best]
    p = {**params, "effective_track_m": best}
    # straight: acceleration to 0.3 m/s, then stop
    inp = steady_input(0.3, 0.0, 6.0, carry())
    inp["base_commands"] = [c if c[0] < 4.0 else [c[0], 0.0, 0.0] for c in inp["base_commands"]]
    out = replay(inp, p)
    t, v = np.asarray(out["t"]), np.asarray(out["twist"])[:, 0]
    result["straight"] = {"v_at_4s": float(v[t <= 4.0][-1]), "rise_90_s": float(t[np.argmax(v >= 0.27)]),
                          "stop_s": stop_time(out, 4.0), "limit_stop_s": 0.3 / 0.8}
    # drift at rest
    out = replay(steady_input(0.0, 0.0, 30.0, carry()), p)
    b0, b1 = np.asarray(out["base"])[0], np.asarray(out["base"])[-1]
    result["drift_30s"] = {"xy_m": float(np.hypot(*(b1[:2] - b0[:2]))), "yaw_rad": float(abs(b1[2] - b0[2]))}
    # breakaway from rest: moved more than 2 mm (2 mrad) in 3 s
    moved_v, moved_w = {}, {}
    for c in (0.005, 0.01, 0.02, 0.03, 0.0375, 0.05):
        base = np.asarray(replay(steady_input(c, 0.0, 3.0, carry()), p)["base"])
        moved_v[c] = bool(np.hypot(*(base[-1, :2] - base[0, :2])) > 0.002)
    for c in (0.02, 0.04, 0.07, 0.1, 0.15):
        base = np.asarray(replay(steady_input(0.0, c, 3.0, carry()), p)["base"])
        moved_w[c] = bool(abs(base[-1, 2] - base[0, 2]) > 0.002)
    result["breakaway"] = {"linear_mps": breakaway(moved_v), "yaw_radps": breakaway(moved_w),
                           "moved_v": {str(k): x for k, x in moved_v.items()}, "moved_w": {str(k): x for k, x in moved_w.items()}}
    # the arm's pose moves the load between the wheels: the turns at the other qualified poses (fitted in carry)
    result["poses"] = {name: turn_table(best, params, turns=[(0.3, 0.3), (0.15, 0.4), (0.0, 0.4), (0.3, 0.1)], q=pose(name))
                       for name in ("home", "vertical_carry", "level_extension", "fold")}
    # sensitivity to the wheel loop (an assumption: commissioned, not measured on hardware)
    result["wheel_kv_sensitivity"] = {str(k): turn_table(best, {**params, "wheel_kv": k}, turns=[(0.3, 0.3), (0.0, 0.4)])
                                      for k in (0.5 * kv, 2.0 * kv)}
    # sensitivity to the ground
    result["mu"] = {str(mu): turn_table(best, {**params, "contact_mu": mu}, turns=[(0.3, 0.3), (0.0, 0.4)]) for mu in (0.4, 0.8)}
    return result


def servo_params(bandwidth, params):
    """The servo alone: the reference reaches the drives arm_command_delay_s late, so with the delay the error is the
    delay's (v x 30 ms), not the servo's (the Task 7b review); the servo tests take it out."""
    return {**params, "gain_bandwidth_hz": bandwidth, "arm_command_delay_s": 0.0}


def step_metrics(t, y, y0, y1, t_step, band=0.02):
    """A step response y(t) from y0 to y1 at t_step: overshoot (a fraction of the step) and the time from t_step after
    which |y - y1| stays within band x |step| (None if it ends outside)."""
    t, y = np.asarray(t), np.asarray(y)
    amp = y1 - y0
    after = t >= t_step
    over = float(max(0.0, (np.max(np.sign(amp) * (y[after] - y1))) / abs(amp)))
    outside = np.abs(y - y1) > band * abs(amp)
    if outside[-1]:
        return {"overshoot": over, "settling_s": None}
    idx = np.flatnonzero(outside & after)
    return {"overshoot": over, "settling_s": float(t[idx[-1] + 1] - t_step) if len(idx) else 0.0}


def sine_metrics(t, q, ref, freq_hz, skip_periods=1.0):
    """Tracking of a sine reference: the amplitude ratio and the phase lag (rad) of q against ref, least squares on
    sin/cos after skip_periods."""
    t, q, ref = np.asarray(t), np.asarray(q), np.asarray(ref)
    keep = t >= t[0] + skip_periods / freq_hz
    w = 2 * np.pi * freq_hz
    basis = np.column_stack([np.sin(w * t[keep]), np.cos(w * t[keep]), np.ones(keep.sum())])
    (aq, bq, _), (ar, br, _) = (np.linalg.lstsq(basis, x[keep], rcond=None)[0] for x in (q, ref))
    gain = float(np.hypot(aq, bq) / np.hypot(ar, br))
    lag = float(np.remainder(np.arctan2(br, ar) - np.arctan2(bq, aq) + np.pi, 2 * np.pi) - np.pi)
    return {"gain": gain, "phase_lag_rad": lag}


def within_limits(q, lower, upper, tol=1e-6):
    q = np.asarray(q)
    return bool((q >= np.asarray(lower) - tol).all() and (q <= np.asarray(upper) + tol).all())


def urdf_limits():
    """The arm joints' position limits from the URDF (lower, upper)."""
    import xml.etree.ElementTree as ET
    root = ET.parse(ROOT / "ros2_ws/src/mobile_manipulator_description/urdf/mobile_manipulator.urdf").getroot()
    joints = {j.get("name"): j.find("limit") for j in root.iter("joint")}
    return (np.array([float(joints[n].get("lower")) for n in cp.ARM]),
            np.array([float(joints[n].get("upper")) for n in cp.ARM]))


def arm_move(bandwidth, params, drive=(0.0, 0.0), hold_s=3.0, move_s=1.5):
    """The test move from carry over move_s (1.5 s; 0.15 s saturates the URDF efforts), held hold_s, with the base still
    or driving; the servo alone (servo_params)."""
    q0 = carry()
    q1 = [a + b for a, b in zip(q0, STEP)]
    inp = steady_input(drive[0], drive[1], 0.5 + move_s + hold_s, q0)
    inp["arm_streams"] = [{"t": 0.5, "points": [{"t": 0.0, "q": q0, "v": [0.0] * 6}, {"t": move_s, "q": q1, "v": [0.0] * 6}]}]
    inp["sample_period_s"] = 0.002
    if drive != (0.0, 0.0):  # stop the base for the last 2 s, so "at rest" is at rest
        end = inp["duration_s"] - 2.0
        inp["base_commands"] = [c if c[0] < end else [c[0], 0.0, 0.0] for c in inp["base_commands"]]
    out = replay(inp, servo_params(bandwidth, params))
    err = np.abs(np.asarray(out["q"]) - np.asarray(out["q_ref"]))
    saturating = move_s < 0.5  # judged by its end (settled), not by its error on the way
    return {"stable": settled(out, q1) if saturating else stable(out), "max_error_rad": [float(x) for x in err.max(0)]}


def joint_responses(bandwidth, params, step_rad=0.05, sine_hz=1.0, sine_rad=0.05):
    """Per joint (amendment Section 5): a near-instant small step (10 ms) and a 1 Hz sine, from carry, the servo alone."""
    rows = []
    for j in range(6):
        q0 = carry()
        q1 = list(q0)
        q1[j] += step_rad
        inp = steady_input(0.0, 0.0, 2.5, q0)
        inp["arm_streams"] = [{"t": 0.5, "points": [{"t": 0.0, "q": q0, "v": [0.0] * 6}, {"t": 0.01, "q": q1, "v": [0.0] * 6}]}]
        inp["sample_period_s"] = 0.002
        out = replay(inp, servo_params(bandwidth, params))
        step = step_metrics(out["t"], np.asarray(out["q"])[:, j], q0[j], q1[j], 0.51)
        period, n = 0.02, int(4.0 / 0.02)
        points = []
        for k in range(n + 1):
            tk = k * period
            q = list(q0)
            q[j] += sine_rad * np.sin(2 * np.pi * sine_hz * tk)
            v = [0.0] * 6
            v[j] = sine_rad * 2 * np.pi * sine_hz * np.cos(2 * np.pi * sine_hz * tk)
            points.append({"t": tk, "q": q, "v": v})
        inp = steady_input(0.0, 0.0, 4.5, q0)
        inp["arm_streams"] = [{"t": 0.5, "points": points}]
        inp["sample_period_s"] = 0.002
        out = replay(inp, servo_params(bandwidth, params))
        t = np.asarray(out["t"])
        during = (t >= 0.5) & (t <= 4.5)
        sine = sine_metrics(t[during], np.asarray(out["q"])[during, j], np.asarray(out["q_ref"])[during, j], sine_hz)
        rows.append({"joint": cp.ARM[j], **step, **sine})
    return rows


def arm_commissioning(params):
    result = {"bandwidth": {}}
    for b in BANDWIDTHS:
        still, driving = arm_move(b, params), arm_move(b, params, drive=(0.3, 0.3))
        fast = arm_move(b, params, move_s=0.15)  # saturates the URDF efforts
        joints = joint_responses(b, params)
        settles = all(r["settling_s"] is not None for r in joints)
        result["bandwidth"][str(b)] = {"stable": still["stable"] and driving["stable"] and fast["stable"] and settles,
                                       "still": still, "driving": driving, "fast": fast, "joints": joints}
    chosen, margins = bandwidth_ruling({float(k): v for k, v in result["bandwidth"].items()}, PROPOSED_BANDWIDTH_HZ)
    result["bandwidth_hz"], result["bandwidth_margins"] = chosen, margins
    base_armature = [0.5, 0.5, 0.3, 0.1, 0.1, 0.1]
    result["armature"] = {str(f): arm_move(chosen, {**params, "armature": [a * f for a in base_armature]}) for f in (0.5, 1.5)
                          } if chosen else {}
    return result


def unity_comparison(params, limit_s=None):
    """calibrate_plant's held-out and fresh runs on this profile (no Unity-fitted overrides)."""
    runs = cp.select_runs()
    train, test = cp.split_by_cell(runs)
    fit = json.loads((ROOT / "docs/experiments/mujoco-plant/calibration_fit.json").read_text())
    used = set(fit["runs"]) | set(fit["start"]["skid_runs"])
    limits = urdf_limits()
    rows = {}
    for name, group in (("held_out", test), ("fresh", cp.fresh_runs(train, used))):
        rows[name] = []
        for run, (inp, series) in zip(group, cp.load_cases(group, limit_s)):
            out = replay(inp, params)
            m = cp.metrics(out, cp.unity_on(series, out["t"]))
            q, q_ref = np.asarray(out["q"]), np.asarray(out["q_ref"])
            arm_ok = bool(np.isfinite(q).all() and np.abs(q - q_ref).max() < 0.1)  # the delay's error included
            rows[name].append({"run": run.name, "duration_s": round(inp["duration_s"], 2),
                               "hard_turn_fraction": round(cp.hard_turn_fraction(series), 4), **m, "passes": cp.passes(m),
                               "arm_stable": arm_ok, "within_limits": within_limits(q, *limits)})
    return rows


# -------------------------------------------------------------------------------------------------------------- report
def acceptance_c(base, arm, unity, assumptions):
    """The amendment's Section 6 C checks: [{check, pass, detail}]."""
    rows = []
    off = [f"({v}, {w}): v x{va / v:.2f}, omega x{wa / w:.2f}" if v else f"({v}, {w}): omega x{wa / w:.2f}"
           for v, w, va, wa in base["turns"] if abs(w) <= 0.6 and
           ((v and abs(va / v - 1) > 0.1) or (w and abs(wa / w - 1) > 0.1))]
    rows.append({"check": "steady commands within +-10 % (|omega| <= 0.6)", "pass": not off,
                 "detail": "; ".join(off) if off else "every turn"})
    in_place = [wa / w for v, w, va, wa in base["turns"] if v == 0.0 and w == 0.2]
    ok = bool(in_place) and abs(in_place[0] - 1) <= 0.1 and base["breakaway"]["yaw_radps"] <= 0.2
    rows.append({"check": "turning in place from 0.2 rad/s", "pass": ok,
                 "detail": f"omega x{in_place[0]:.2f} at 0.2 rad/s; breakaway {base['breakaway']['yaw_radps']} rad/s" if in_place else "not measured"})
    s, d = base["straight"], base["drift_30s"]
    per_min = 2.0 * d["xy_m"]
    rows.append({"check": "stops within the deceleration limit + 0.2 s, drift < 1 mm/min",
                 "pass": s["stop_s"] is not None and s["stop_s"] <= s["limit_stop_s"] + 0.2 and per_min < 1e-3,
                 "detail": (f"stop {s['stop_s']:.2f} s" if s["stop_s"] is not None else "no stop")
                           + f" (limit {s['limit_stop_s']:.3f} + 0.2), drift {1000 * per_min:.3f} mm/min"})
    streams = [r for part in ("held_out", "fresh") for r in (unity or {}).get(part, [])]
    bad = [r.get("run", "?") for r in streams if not (r.get("arm_stable") and r.get("within_limits"))]
    rows.append({"check": "arm stable at rest and on every recorded stream, inside the URDF limits",
                 "pass": bool(arm.get("stable_everywhere")) and bool(streams) and not bad,
                 "detail": f"{len(streams) - len(bad)}/{len(streams)} recorded streams" + (f"; not: {', '.join(bad)}" if bad else "")})
    unchecked = [k for k, a in assumptions.items() if "checked" not in a]
    rows.append({"check": "every assumption has a source and a sensitivity result", "pass": not unchecked,
                 "detail": "unchecked: " + ", ".join(unchecked) if unchecked else "all"})
    failing = {(v, w): wa / w for v, w, va, wa in base["turns"] if abs(w) <= 0.6 and
               ((v and abs(va / v - 1) > 0.1) or (w and abs(wa / w - 1) > 0.1))}
    evidence = {rows[0]["check"]: failing, rows[4]["check"]: unchecked}
    for r in rows:
        if not r["pass"] and r["check"] in ACCEPTED and ACCEPTED[r["check"]][0](evidence[r["check"]]):
            r["accepted"] = ACCEPTED[r["check"]][1]
    return rows


def _accepted_turns(failing, tol=0.03):
    """Only the turns the user accepted, at their measured ratios (2026-11-06): (0.3, 0.1) x0.79, (0.0, 0.6) x0.87."""
    accepted = {(0.3, 0.1): 0.79, (0.0, 0.6): 0.87}
    return set(failing) <= set(accepted) and all(abs(r - accepted[k]) <= tol for k, r in failing.items())


# the user's rulings on the C checks that fail (2026-11-06)
# (whether the failure is the one accepted, the note): a different failure of the same check stays a failure
ACCEPTED = {
    "steady commands within +-10 % (|omega| <= 0.6)": (_accepted_turns,
        "accepted deviation (user 2026-11-06, as at CP3b): the wheel-contact base understeers gentle turns and the "
        "fastest in-place turn; the contact variants did not remove it. Task 11 brackets its effect on the planners "
        "(Unity's sliding base against this gripping one) and runs the grip ensemble"),
    "every assumption has a source and a sensitivity result": (lambda unchecked: unchecked == ["torque_speed"],
        "accepted (user 2026-11-06): torque_speed has no motor data. The planners' fastest corner needs 0.90 m/s at the "
        "rim, 36 % of the URDF's 18 rad/s wheel speed, where a motor's torque droop is small"),
}


def envelope_check(track, wheel_speed_max=8.0, radius=0.14):
    import yaml
    env = yaml.safe_load(ENVELOPE.read_text())
    rows = []
    for name, prof in env["profiles"].items():
        v, w = max(prof["max_forward_mps"], prof["max_reverse_mps"]), prof["max_yaw_radps"]
        rows.append((name, v, w, abs(v) + abs(w) * track / 2.0, reachable(v, w, track, wheel_speed_max, radius)))
    return rows


def report(base, arm, unity, out_dir, date):
    b = base["effective_track_m"]
    lines = ["# MuJoCo plant commissioning (URDF-based realistic mode)", "",
             f"Generated by `tools/plant/commission_plant.py` on {date} (amendment 2026-11-06, plan Task 7b). The plant is "
             "measured through `plant_replay` with synthetic commands on the ground alone, the arm in carry. Assumptions and "
             "their sources: the profile's `assumptions` section.", "",
             "## Base", "",
             f"**Wheel loop kv: {base['wheel_kv']} N m s/rad**, the stiffest of {WHEEL_KVS} whose base follows the limiter's "
             "acceleration ramp (0.5 m/s^2 to 0.3 m/s) within 0.02 m/s. MuJoCo's contact is soft (a 20 ms time constant at the "
             "2 ms step); a wheel loop much stiffer than it winds the contact up and overshoots the ramp (a simulation "
             "artefact: 0.5 ms steps or stiffer contacts shrink it). Max |v - ramp| per kv: "
             + ", ".join(f"{k}: {v:.3f}" for k, v in base["wheel_kv_ramp_error"].items()) + " m/s.", "",
             f"**Effective track: {b} m** ({b / 0.64:.2f}x the geometric 0.64 m; Clearpath's A300 4WD uses 1.75x). Mean "
             "squared yaw-ratio error of the turns (|omega| >= 0.1) per candidate: "
             + ", ".join(f"{k}: {v:.4f}" for k, v in base["track_scores"].items()) + ".", "",
             "| Command (v, omega) | Achieved v | Achieved omega | v ratio | omega ratio |", "|---|---|---|---|---|"]
    for v, w, va, wa in base["turns"]:
        lines.append(f"| ({v}, {w}) | {va:.3f} | {wa:.3f} | {va / v:.2f} | {wa / w:.2f} |" if v else
                     f"| ({v}, {w}) | {va:.3f} | {wa:.3f} | - | {wa / w:.2f} |")
    s = base["straight"]
    lines += ["", f"Straight 0.3 m/s: {s['v_at_4s']:.3f} m/s after 4 s, 90 % after {s['rise_90_s']:.2f} s (limiter: 0.54 s); "
              f"stopped {s['stop_s']:.2f} s after the stop command (deceleration limit alone: {s['limit_stop_s']:.2f} s). Drift at "
              f"rest over 30 s: {1000 * base['drift_30s']['xy_m']:.2f} mm, {1000 * base['drift_30s']['yaw_rad']:.2f} mrad.", "",
              f"Breakaway from rest (moved > 2 mm or 2 mrad in 3 s, every larger command too): linear "
              f"{base['breakaway']['linear_mps']} m/s, yaw {base['breakaway']['yaw_radps']} rad/s (the planners' envelope "
              "assumes Unity's 0.0375 m/s and 0.07 rad/s).", "",
              "Wheel loop sensitivity (the effective track fixed; kv halved and doubled): "
              + "; ".join(f"kv {k}: " + ", ".join(f"({v}, {w}) -> ({va:.2f}, {wa:.2f})" for v, w, va, wa in rows)
                          for k, rows in base.get("wheel_kv_sensitivity", {}).items()) + ".", "",
              "Ground friction sensitivity (the effective track fixed): "
              + "; ".join(f"mu {mu}: " + ", ".join(f"({v}, {w}) -> ({va:.2f}, {wa:.2f})" for v, w, va, wa in rows)
                          for mu, rows in base["mu"].items()) + ".", "",
              "Arm pose sensitivity (the effective track fitted in carry; omega achieved / commanded): "
              + "; ".join(f"{name}: " + ", ".join(f"({v}, {w}) {wa / w:.2f}" for v, w, va, wa in rows)
                          for name, rows in base["poses"].items()) + ".", "",
              "### Reachable envelope", "",
              f"The wheel cap (8 rad/s, 1.12 m/s at the rim) limits |v| + |omega| b/2. The planners' envelope "
              "(nav_operating_envelope.yaml) against it:", "",
              "| Profile | v max | omega max | Rim speed needed (m/s) | Reachable |", "|---|---|---|---|---|"]
    for name, v, w, need, ok in envelope_check(b):
        lines.append(f"| {name} | {v} | {w} | {need:.2f} | {'yes' if ok else 'NO'} |")
    lines += ["", "## Arm", "",
              f"**Bandwidth: {arm['bandwidth_hz']} Hz**, the stated assumption, kept (user ruling 2026-11-06): stable up to "
              f"{arm['bandwidth_margins']['stable_to_hz']} Hz and well damped (every joint's step overshoot <= "
              f"{100 * WELL_DAMPED_OVERSHOOT:.0f} %) up to {arm['bandwidth_margins']['well_damped_to_hz']} Hz, of {BANDWIDTHS}. "
              "Stable: at rest after the moves (joint-velocity RMS < 1 mrad/s; error < 0.1 rad) with the base still and "
              "driving (0.3 m/s, 0.3 rad/s), the saturating move settled on target, every joint's step settled. The "
              "amendment's 'highest stable' would take the stability limit, where the wrists ring; a geared arm's position "
              "loop sits well below it. Every arm test here runs the servo alone (the 30 ms command "
              "delay set to 0: with it, the error is the delay's). The 0.15 s move saturates the URDF efforts. "
              f"The test move: {STEP} rad from carry in 1.5 s.", "",
              "| Bandwidth (Hz) | Stable | Max error still (mrad) | Max error driving (mrad) | Max error, 0.15 s move (mrad) |",
              "|---|---|---|---|---|"]
    for k, r in arm["bandwidth"].items():
        lines.append(f"| {k} | {'yes' if r['stable'] else 'no'} | {', '.join(f'{1000 * x:.1f}' for x in r['still']['max_error_rad'])} | "
                     f"{', '.join(f'{1000 * x:.1f}' for x in r['driving']['max_error_rad'])} | "
                     f"{', '.join(f'{1000 * x:.0f}' for x in r['fast']['max_error_rad'])} |")
    lines += ["", "Per-joint responses (the servo alone): a 0.05 rad step in 10 ms (overshoot, 2 % settling time) and a 1 Hz, "
              "0.05 rad sine (gain, phase lag).", "",
              "| Bandwidth (Hz) | " + " | ".join(j.replace("_joint", "") for j in cp.ARM) + " |", "|---|" + "---|" * 6]
    for k, r in arm["bandwidth"].items():
        cells = [f"{100 * j['overshoot']:.0f} %, " + (f"{1000 * j['settling_s']:.0f} ms" if j["settling_s"] is not None else "no settle")
                 + f"; x{j['gain']:.2f}, {1000 * j['phase_lag_rad'] / (2 * 3.14159265):.0f} ms" for j in r["joints"]]
        lines.append(f"| {k} | " + " | ".join(cells) + " |")
    if arm.get("armature"):
        lines += ["", "Armature sensitivity at the chosen bandwidth (the gains follow the inertia): "
                  + "; ".join(f"x{f}: max error {', '.join(f'{1000 * x:.1f}' for x in r['max_error_rad'])} mrad, "
                              f"{'stable' if r['stable'] else 'UNSTABLE'}" for f, r in arm["armature"].items()) + "."]
    if unity:
        lines += ["", "## Comparison with Unity (reported, not gated)", "",
                  "calibrate_plant.py's runs replayed open loop on this profile, whole tasks. The targets are the Task 7 "
                  "ones, kept for comparison: differences are simulator differences (docs/experiments/mujoco-plant/calibration.md).", "",
                  "### Held out", ""] + cp._table(unity["held_out"]) + ["", "### Fresh", ""] + cp._table(unity["fresh"])
    import yaml
    assumptions = yaml.safe_load(PROFILE.read_text()).get("assumptions", {})
    lines += ["", "## Acceptance C (amendment Section 6)", "", "| Check | Pass | Detail |", "|---|---|---|"]
    for r in acceptance_c(base, arm, unity, assumptions):
        verdict = "yes" if r["pass"] else ("accepted" if "accepted" in r else "NO")
        lines.append(f"| {r['check']} | {verdict} | {r['detail']}" + (f"; {r['accepted']}" if "accepted" in r else "") + " |")
    notes = out_dir / "commissioning_notes.md"  # hand-written analysis, appended
    if notes.exists():
        lines += ["", notes.read_text().rstrip()]
    (out_dir / "commissioning.md").write_text("\n".join(lines) + "\n")
    (out_dir / "commissioning.json").write_text(json.dumps({"base": base, "arm": arm, "unity": unity}, indent=1) + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", choices=["all"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--write-profile", action="store_true")
    ap.add_argument("--skip-unity", action="store_true")
    args = ap.parse_args()
    out_dir = ROOT / args.out
    out_dir.mkdir(parents=True, exist_ok=True)
    date = dt.date.today().isoformat()
    base = base_commissioning({})
    print("effective track", base["effective_track_m"], flush=True)
    print("wheel kv", base["wheel_kv"], flush=True)
    arm = arm_commissioning({"effective_track_m": base["effective_track_m"], "wheel_kv": base["wheel_kv"]})
    print("bandwidth", arm["bandwidth_hz"], arm["bandwidth_margins"], flush=True)
    params = {"effective_track_m": base["effective_track_m"], "wheel_kv": base["wheel_kv"], "gain_bandwidth_hz": arm["bandwidth_hz"]}
    unity = None if args.skip_unity else unity_comparison(params)
    chosen = arm["bandwidth"].get(str(arm["bandwidth_hz"]), {})
    arm["stable_everywhere"] = bool(chosen.get("stable")) and all(r["stable"] for r in arm.get("armature", {}).values())
    report(base, arm, unity, out_dir, date)
    if args.write_profile:
        write_values(base["effective_track_m"], arm["bandwidth_hz"], date, base["wheel_kv"])
        print(f"wrote the effective track and the bandwidth into {PROFILE.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
