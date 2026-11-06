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
TRACKS = [1.5, 2.0, 2.25, 2.5, 2.75, 3.0, 3.5]
TURNS = [(0.3, 0.1), (0.3, 0.2), (0.3, 0.3), (0.3, 0.4), (0.15, 0.3), (0.15, 0.4), (0.0, 0.2), (0.0, 0.4), (0.0, 0.6)]
BANDWIDTHS = [5.0, 8.0, 10.0, 12.0, 15.0]
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


def choose_bandwidth(results):
    """The highest bandwidth judged stable ({bandwidth: {"stable": bool, ...}})."""
    ok = [b for b, r in results.items() if r["stable"]]
    return max(ok) if ok else None


def write_values(track, bandwidth, date):
    """Writes the effective track and the arm bandwidth into the profile, each with a provenance line; idempotent."""
    text = PROFILE.read_text()
    text = re.sub(r"^  # Commissioned by tools/plant/commission_plant\.py.*\n", "", text, flags=re.M)
    line = f"  # Commissioned by tools/plant/commission_plant.py on {date} (docs/experiments/mujoco-plant/commissioning.md)\n"
    text, n1 = re.subn(r"^  effective_track_m: .*\n", f"{line}  effective_track_m: {track}\n", text, count=1, flags=re.M)
    text, n2 = re.subn(r"^  gain_rule: \{bandwidth_hz: [^,]*,", f"{line}  gain_rule: {{bandwidth_hz: {bandwidth},", text, count=1, flags=re.M)
    if n1 != 1 or n2 != 1:
        raise RuntimeError("profile: no effective_track_m or gain_rule line")
    PROFILE.write_text(text)


# ------------------------------------------------------------------------------------------------------- measurement
def carry():
    return json.loads(PAYLOAD.read_text())["poses_rad"]["carry"]


def replay(inp, params):
    return cp.plant_replay(inp, params)


def turn_table(track, params, turns=TURNS, seconds=8.0):
    rows = []
    for v, w in turns:
        out = replay(steady_input(v, w, seconds, carry()), {**params, "effective_track_m": track})
        rows.append((v, w, *steady_reading(out)))
    return rows


def base_commissioning(params):
    result = {}
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
    # sensitivity to the ground
    result["mu"] = {str(mu): turn_table(best, {**params, "contact_mu": mu}, turns=[(0.3, 0.3), (0.0, 0.4)]) for mu in (0.4, 0.8)}
    return result


def arm_move(bandwidth, params, drive=(0.0, 0.0), hold_s=3.0):
    """The test move from carry over 1.5 s, held hold_s, with the base still or driving."""
    q0 = carry()
    q1 = [a + b for a, b in zip(q0, STEP)]
    inp = steady_input(drive[0], drive[1], 0.5 + 1.5 + hold_s, q0)
    inp["arm_streams"] = [{"t": 0.5, "points": [{"t": 0.0, "q": q0, "v": [0.0] * 6}, {"t": 1.5, "q": q1, "v": [0.0] * 6}]}]
    inp["sample_period_s"] = 0.002
    if drive != (0.0, 0.0):  # stop the base for the last 2 s, so "at rest" is at rest
        end = inp["duration_s"] - 2.0
        inp["base_commands"] = [c if c[0] < end else [c[0], 0.0, 0.0] for c in inp["base_commands"]]
    out = replay(inp, {**params, "gain_bandwidth_hz": bandwidth})
    err = np.abs(np.asarray(out["q"]) - np.asarray(out["q_ref"]))
    return {"stable": stable(out), "max_error_rad": [float(x) for x in err.max(0)],
            "kp": out["params"]["kp"] if "params" in out else None}


def arm_commissioning(params):
    result = {"bandwidth": {}}
    for b in BANDWIDTHS:
        still, driving = arm_move(b, params), arm_move(b, params, drive=(0.3, 0.3))
        result["bandwidth"][str(b)] = {"stable": still["stable"] and driving["stable"], "still": still, "driving": driving}
    chosen = choose_bandwidth({float(k): v for k, v in result["bandwidth"].items()})
    result["bandwidth_hz"] = chosen
    base_armature = [0.5, 0.5, 0.3, 0.1, 0.1, 0.1]
    result["armature"] = {f: arm_move(chosen, {**params, "armature": [a * f for a in base_armature]}) for f in ("0.5", "1.5")
                          } if chosen else {}
    return result


def unity_comparison(params, limit_s=None):
    """calibrate_plant's held-out and fresh runs on this profile (no Unity-fitted overrides)."""
    runs = cp.select_runs()
    train, test = cp.split_by_cell(runs)
    fit = json.loads((ROOT / "docs/experiments/mujoco-plant/calibration_fit.json").read_text())
    used = set(fit["runs"]) | set(fit["start"]["skid_runs"])
    rows = {}
    for name, group in (("held_out", test), ("fresh", cp.fresh_runs(train, used))):
        rows[name] = []
        for run, (inp, series) in zip(group, cp.load_cases(group, limit_s)):
            out = cp.plant_replay(inp, params)
            m = cp.metrics(out, cp.unity_on(series, out["t"]))
            rows[name].append({"run": run.name, "duration_s": round(inp["duration_s"], 2),
                               "hard_turn_fraction": round(cp.hard_turn_fraction(series), 4), **m, "passes": cp.passes(m)})
    return rows


# -------------------------------------------------------------------------------------------------------------- report
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
              "Ground friction sensitivity (the effective track fixed): "
              + "; ".join(f"mu {mu}: " + ", ".join(f"({v}, {w}) -> ({va:.2f}, {wa:.2f})" for v, w, va, wa in rows)
                          for mu, rows in base["mu"].items()) + ".", "",
              "### Reachable envelope", "",
              f"The wheel cap (8 rad/s, 1.12 m/s at the rim) limits |v| + |omega| b/2. The planners' envelope "
              "(nav_operating_envelope.yaml) against it:", "",
              "| Profile | v max | omega max | Rim speed needed (m/s) | Reachable |", "|---|---|---|---|---|"]
    for name, v, w, need, ok in envelope_check(b):
        lines.append(f"| {name} | {v} | {w} | {need:.2f} | {'yes' if ok else 'NO'} |")
    lines += ["", "## Arm", "",
              f"**Bandwidth: {arm['bandwidth_hz']} Hz**, the highest of {BANDWIDTHS} that stays stable (at rest after the "
              "move: joint-velocity RMS < 1 mrad/s; error < 0.1 rad) with the base still and driving (0.3 m/s, 0.3 rad/s). "
              f"The test move: {STEP} rad from carry in 1.5 s.", "",
              "| Bandwidth (Hz) | Stable | Max error still (mrad) | Max error driving (mrad) |", "|---|---|---|---|"]
    for k, r in arm["bandwidth"].items():
        lines.append(f"| {k} | {'yes' if r['stable'] else 'no'} | {', '.join(f'{1000 * x:.1f}' for x in r['still']['max_error_rad'])} | "
                     f"{', '.join(f'{1000 * x:.1f}' for x in r['driving']['max_error_rad'])} |")
    if arm.get("armature"):
        lines += ["", "Armature sensitivity at the chosen bandwidth (the gains follow the inertia): "
                  + "; ".join(f"x{f}: max error {', '.join(f'{1000 * x:.1f}' for x in r['max_error_rad'])} mrad, "
                              f"{'stable' if r['stable'] else 'UNSTABLE'}" for f, r in arm["armature"].items()) + "."]
    if unity:
        lines += ["", "## Comparison with Unity (reported, not gated)", "",
                  "calibrate_plant.py's runs replayed open loop on this profile, whole tasks. The targets are the Task 7 "
                  "ones, kept for comparison: differences are simulator differences (docs/experiments/mujoco-plant/calibration.md).", "",
                  "### Held out", ""] + cp._table(unity["held_out"]) + ["", "### Fresh", ""] + cp._table(unity["fresh"])
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
    arm = arm_commissioning({"effective_track_m": base["effective_track_m"]})
    print("bandwidth", arm["bandwidth_hz"], flush=True)
    params = {"effective_track_m": base["effective_track_m"], "gain_bandwidth_hz": arm["bandwidth_hz"]}
    unity = None if args.skip_unity else unity_comparison(params)
    report(base, arm, unity, out_dir, date)
    if args.write_profile:
        write_values(base["effective_track_m"], arm["bandwidth_hz"], date)
        print(f"wrote the effective track and the bandwidth into {PROFILE.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
