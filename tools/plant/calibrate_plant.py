"""Calibrate the MuJoCo plant's realistic mode against Unity recordings (MuJoCo plant spec Section 7, plan Task 7).

The training set is the whole-body runs with today's Unity actuators (gravity, motion and base friction feedforward:
directory suffix `-gff-mff-bff`), split by cell (about 80 % fit, 20 % held out). Base: the delay by cross-correlating
/cmd_vel (what Unity received) with the /odom twist, a first-order lag and slip as starting values, then a coordinate
search with plant_replay. Arm: the delay and the servo gains by coordinate search with plant_replay against the
recorded joints. Windows where the collision monitor changed the command (/cmd_vel != /cmd_vel_smoothed) are counted
and reported. Validation replays the held-out runs open loop with the fitted values.

Runs in the container (rosbags, numpy, the plant_replay binary):
    python3 tools/plant/calibrate_plant.py all --out docs/experiments/mujoco-plant/ [--write-profile] [--runs N]
"""
import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
RUNS = ROOT / "experiment_runs"
PROFILE = ROOT / "ros2_ws/src/mobile_manipulator_plant/robots/mobile_manipulator/profile.yaml"
ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
CONFIG = "-gff-mff-bff"
TARGETS = {"arm_p95_rel": 0.30, "arm_p95_abs": 0.002, "base_speed_rmse": 0.02, "yaw_rate_rmse": 0.03, "drift_2s": 0.03}


# ---------------------------------------------------------------------------------------------------------- selection
def cell_of(run):
    name = Path(run).name
    return name.split("-", 1)[1].split("-wholebody")[0]


def select_runs(experiment_runs=RUNS):
    """Whole-body runs with today's Unity actuator configuration, a summary and a bag."""
    out = []
    for d in sorted(Path(experiment_runs).iterdir()):
        if "-wholebody-" not in d.name or CONFIG not in d.name:
            continue
        if (d / "summary.json").exists() and (d / "bag" / "metadata.yaml").exists():
            out.append(d)
    return out


def spread(runs, n):
    """Up to n runs taken round-robin across their cells (no cell dominates)."""
    by_cell = {}
    for r in runs:
        by_cell.setdefault(cell_of(r), []).append(r)
    out, k = [], 0
    while len(out) < n and any(k < len(v) for v in by_cell.values()):
        for cell in sorted(by_cell):
            if k < len(by_cell[cell]) and len(out) < n:
                out.append(by_cell[cell][k])
        k += 1
    return out


def split_by_cell(runs, held_out=0.2, seed=0):
    """(train, test): whole cells to one side; about `held_out` of the cells held out."""
    cells = sorted({cell_of(r) for r in runs})
    order = np.random.default_rng(seed).permutation(len(cells))
    n_test = max(1, int(round(held_out * len(cells))))
    test_cells = {cells[i] for i in order[:n_test]}
    train = [r for r in runs if cell_of(r) not in test_cells]
    test = [r for r in runs if cell_of(r) in test_cells]
    return train, test


# --------------------------------------------------------------------------------------------------- signal analysis
def _grid(t_a, t_b, step):
    lo, hi = max(t_a[0], t_b[0]), min(t_a[-1], t_b[-1])
    return np.arange(lo, hi, step)


def cross_correlation_delay(t_cmd, cmd, t_out, out, max_s=0.2, step=0.002):
    """The lag (s, >= 0) at which `out` best follows `cmd`: the largest Pearson correlation of the overlapping parts on
    a common grid (normalized per lag, so the shrinking overlap does not favour long lags)."""
    g = _grid(np.asarray(t_cmd), np.asarray(t_out), step)
    a = np.interp(g, t_cmd, cmd)
    b = np.interp(g, t_out, out)
    best, best_lag = -np.inf, 0
    for lag in range(int(round(max_s / step)) + 1):
        n = len(g) - lag
        if n < 10:
            break
        x, y = a[:n] - a[:n].mean(), b[lag:lag + n] - b[lag:lag + n].mean()
        den = float(np.linalg.norm(x) * np.linalg.norm(y))
        c = float(np.dot(x, y)) / den if den > 0 else -np.inf
        if c > best:
            best, best_lag = c, lag
    return best_lag * step


def fit_first_order(t_cmd, cmd, t_out, out, delay, step=0.002):
    """(tau, k) of y' = (k u(t - delay) - y) / tau by least squares on the discretized lag."""
    g = _grid(np.asarray(t_cmd) + delay, np.asarray(t_out), step)
    u = np.interp(g - delay, t_cmd, cmd)
    y = np.interp(g, t_out, out)
    A = np.stack([y[:-1], u[:-1]], 1)
    (c1, c2), *_ = np.linalg.lstsq(A, y[1:], rcond=None)
    a = 1.0 - c1
    if not (0.0 < a < 1.0):
        return 0.0, float(c2)  # no measurable lag
    return -step / math.log(1.0 - a), float(c2 / a)


def monitor_windows(t_sent, sent, t_smoothed, smoothed, tol=1e-6, merge_s=0.1):
    """[(start, end)] where the command Unity received differs from B5's (the collision monitor acted)."""
    t_sent, sent = np.asarray(t_sent), np.asarray(sent)
    idx = np.searchsorted(t_smoothed, t_sent, side="right") - 1
    held = np.asarray(smoothed)[np.clip(idx, 0, len(smoothed) - 1)]
    differs = np.abs(sent - held) > tol
    windows, start, last = [], None, None
    for t, d in zip(t_sent, differs):
        if d:
            if start is None or (last is not None and t - last > merge_s):
                if start is not None:
                    windows.append((start, last))
                start = t
            last = t
    if start is not None:
        windows.append((start, last))
    return windows


def steady_samples(series, settle_s=0.3, hold_s=0.5):
    """[(v_cmd, w_cmd, v_achieved, w_achieved)] where the command held within 0.02 m/s and 0.05 rad/s for `hold_s`,
    the achieved twist read `settle_s` later (past the lag)."""
    tc = np.array([x[0] for x in series["cmd_vel"]]); vc = np.array([x[1] for x in series["cmd_vel"]]); wc = np.array([x[2] for x in series["cmd_vel"]])
    to = np.array([x[0] for x in series["odom"]]); vo = np.array([x[2][0] for x in series["odom"]]); wo = np.array([x[2][1] for x in series["odom"]])
    if len(tc) < 10 or len(to) < 10:
        return np.zeros((0, 4))
    g = np.arange(max(tc[0], to[0]) + hold_s, min(tc[-1], to[-1]) - settle_s, 0.02)
    v, w = np.interp(g, tc, vc), np.interp(g, tc, wc)
    steady = (np.abs(v - np.interp(g - hold_s, tc, vc)) < 0.02) & (np.abs(w - np.interp(g - hold_s, tc, wc)) < 0.05)
    return np.stack([v, w, np.interp(g + settle_s, to, vo), np.interp(g + settle_s, to, wo)], 1)[steady]


def fit_skid_map(samples, knots=(0.0, 0.15, 0.5, 0.85, 1.25, 2.0), track=1.5, min_n=30):
    """The base's skid map from steady samples: the median achieved / commanded speed and yaw rate in bins around each
    knot of the inner wheels' share |w| (track / 2) / v (driving forward, v >= 0.05), and the median yaw factor turning
    in place (|v| < 0.02, |w| >= 0.1). A bin with too few samples takes its neighbour's value; identity without any."""
    samples = np.asarray(samples)
    knots = list(knots)
    v, w, va, wa = samples.T if len(samples) else (np.zeros(0),) * 4
    driving = v >= 0.05
    share = np.where(driving, np.abs(w) * 0.5 * track / np.maximum(v, 1e-9), np.inf)
    edges = [0.0] + [(a + b) / 2 for a, b in zip(knots, knots[1:])] + [np.inf]
    kv, kw = [], []
    for lo, hi in zip(edges, edges[1:]):
        m = driving & (share >= lo) & (share < hi)
        turning = m & (np.abs(w) >= 0.02)
        kv.append(float(np.median(va[m] / v[m])) if m.sum() >= min_n else None)
        kw.append(float(np.median(wa[turning] / w[turning])) if turning.sum() >= min_n else None)
    def fill(values):
        known = [x for x in values if x is not None]
        if not known:
            return [1.0] * len(values)
        out, last = [], None
        for x in values:
            last = x if x is not None else last
            out.append(last)
        first = next(x for x in out if x is not None)
        return [first if x is None else x for x in out]
    in_place = (np.abs(v) < 0.02) & (np.abs(w) >= 0.1)
    w_in_place = float(np.median(wa[in_place] / w[in_place])) if in_place.sum() >= min_n else 1.0
    return {"share": knots, "v": fill(kv), "w": fill(kw), "w_in_place": w_in_place}


# ------------------------------------------------------------------------------------------------------- bag series
def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_series(bag):
    """The signals a replay and its comparison need, on the bag's receive clock (s)."""
    from rosbags.highlevel import AnyReader  # container only
    s = {"cmd_vel": [], "cmd_vel_smoothed": [], "trajectories": [], "odom": [], "joints": [], "controller": []}
    with AnyReader([Path(bag)]) as r:
        conns = [c for c in r.connections if c.topic in ("/cmd_vel", "/cmd_vel_smoothed", "/arm_controller/joint_trajectory",
                                                         "/odom", "/joint_states", "/arm_controller/controller_state")]
        for c, t_ns, raw in r.messages(connections=conns):
            t = t_ns * 1e-9
            m = r.deserialize(raw, c.msgtype)
            if c.topic in ("/cmd_vel", "/cmd_vel_smoothed"):
                s[c.topic[1:]].append((t, m.linear.x, m.angular.z))
            elif c.topic == "/arm_controller/joint_trajectory":
                if m.header.stamp.sec or m.header.stamp.nanosec:
                    raise NotImplementedError(f"{bag}: a stamped arm stream (only on-receipt streams are replayed)")
                order = [list(m.joint_names).index(j) for j in ARM]
                pts = [(p.time_from_start.sec + 1e-9 * p.time_from_start.nanosec,
                        [float(p.positions[i]) for i in order],
                        [float(p.velocities[i]) if len(p.velocities) else 0.0 for i in order]) for p in m.points]
                s["trajectories"].append((t, pts))
            elif c.topic == "/odom":
                p = m.pose.pose
                s["odom"].append((t, (p.position.x, p.position.y, _yaw(p.orientation)), (m.twist.twist.linear.x, m.twist.twist.angular.z)))
            elif c.topic == "/joint_states":
                names = list(m.name)
                if all(j in names for j in ARM):
                    s["joints"].append((t, [float(m.position[names.index(j)]) for j in ARM]))
            else:
                order = [list(m.joint_names).index(j) for j in ARM]
                s["controller"].append((t, [float(m.reference.positions[i]) for i in order],
                                        [float(m.feedback.positions[i]) for i in order]))
    s["t0"] = min(x[0][0] for x in (s["odom"], s["joints"]) if x)
    s["end"] = s["odom"][-1][0]
    return s


def replay_input(series):
    """plant_replay's input from a recording's series: start where it starts, every command on its receipt time."""
    t0 = series["t0"]
    first_odom = series["odom"][0]
    return {
        "start": {"base": list(first_odom[1]), "q": list(series["joints"][0][1])},
        "duration_s": series["end"] - t0,
        "sample_period_s": 0.02,
        "world": "ground",
        "base_commands": [[t - t0, v, w] for t, v, w in series["cmd_vel"]],
        "arm_streams": [{"t": t - t0, "points": [{"t": pt, "q": q, "v": v} for pt, q, v in pts]}
                        for t, pts in series["trajectories"]],
    }


def unity_on(series, t_grid):
    """The recorded Unity signals resampled on the replay's time grid (from t0)."""
    t0 = series["t0"]
    to = np.array([x[0] for x in series["odom"]]) - t0
    pose = np.array([x[1] for x in series["odom"]])
    twist = np.array([x[2] for x in series["odom"]])
    tj = np.array([x[0] for x in series["joints"]]) - t0
    q = np.array([x[1] for x in series["joints"]])
    tc = np.array([x[0] for x in series["controller"]]) - t0
    ref = np.array([x[1] for x in series["controller"]])
    yaw = np.unwrap(pose[:, 2])
    return {
        "t": t_grid,
        "base": np.stack([np.interp(t_grid, to, pose[:, 0]), np.interp(t_grid, to, pose[:, 1]), np.interp(t_grid, to, yaw)], 1),
        "twist": np.stack([np.interp(t_grid, to, twist[:, i]) for i in range(2)], 1),
        "q": np.stack([np.interp(t_grid, tj, q[:, i]) for i in range(6)], 1),
        "q_ref": np.stack([np.interp(t_grid, tc, ref[:, i]) for i in range(6)], 1),
    }


# ----------------------------------------------------------------------------------------------------------- metrics
def metrics(plant, unity, window_s=2.0):
    """Section 7.3's comparison of a replay with its recording, both on the same time grid."""
    t = np.asarray(plant["t"])
    dt = float(np.median(np.diff(t)))
    w = max(1, int(round(window_s / dt)))
    pb, ub = np.asarray(plant["base"]), np.asarray(unity["base"])
    drifts = []
    for k in range(0, len(t) - w, w):
        def local(b):
            d = b[k + w, :2] - b[k, :2]
            c, s = math.cos(b[k, 2]), math.sin(b[k, 2])
            return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1]])
        drifts.append(float(np.linalg.norm(local(pb) - local(ub))))
    pt, ut = np.asarray(plant["twist"]), np.asarray(unity["twist"])
    arm = []
    for j in range(6):
        pe = np.abs(np.asarray(plant["q"])[:, j] - np.asarray(plant["q_ref"])[:, j])
        ue = np.abs(np.asarray(unity["q"])[:, j] - np.asarray(unity["q_ref"])[:, j])
        arm.append({"plant": float(np.percentile(pe, 95)), "unity": float(np.percentile(ue, 95))})
    return {
        "base_speed_rmse": float(np.sqrt(np.mean((pt[:, 0] - ut[:, 0]) ** 2))),
        "yaw_rate_rmse": float(np.sqrt(np.mean((pt[:, 1] - ut[:, 1]) ** 2))),
        "drift_2s_max": max(drifts) if drifts else 0.0,
        "drift_2s_p95": float(np.percentile(drifts, 95)) if drifts else 0.0,
        "arm_p95": arm,
        "arm_q_rmse": [float(np.sqrt(np.mean((np.asarray(plant["q"])[:, j] - np.asarray(unity["q"])[:, j]) ** 2))) for j in range(6)],
    }


def passes(m):
    """Section 7.3's targets on one run's metrics."""
    arm_ok = all(abs(a["plant"] - a["unity"]) <= max(TARGETS["arm_p95_rel"] * a["unity"], TARGETS["arm_p95_abs"]) for a in m["arm_p95"])
    return {"arm": arm_ok, "base_speed": m["base_speed_rmse"] <= TARGETS["base_speed_rmse"],
            "yaw_rate": m["yaw_rate_rmse"] <= TARGETS["yaw_rate_rmse"], "drift": m["drift_2s_max"] <= TARGETS["drift_2s"]}


# ------------------------------------------------------------------------------------------------------------ replay
def plant_replay(replay, params):
    """Runs the plant_replay binary (sourced workspace) on one input with parameter overrides; its output as arrays."""
    exe = Path(os.environ.get("ROS_WS", "/workspaces/mm-motion-planning/ros2_ws")) / \
        "install/mobile_manipulator_campaign/lib/mobile_manipulator_campaign/plant_replay"
    with tempfile.TemporaryDirectory() as tmp:
        src, dst = Path(tmp) / "in.json", Path(tmp) / "out.json"
        src.write_text(json.dumps({**replay, "params": params}))
        subprocess.run([str(exe), "--input", str(src), "--out", str(dst)], check=True, stdout=subprocess.DEVNULL)
        out = json.loads(dst.read_text())
    return {k: np.asarray(v) for k, v in out.items() if k != "params"}


def evaluate(cases, params, part):
    """The mean objective of `part` ("base": twist RMSE; "arm": joint RMSE against Unity) over the cases."""
    total = 0.0
    for replay, series in cases:
        out = plant_replay(replay, params)
        unity = unity_on(series, out["t"])
        m = metrics(out, unity)
        total += (m["base_speed_rmse"] + m["yaw_rate_rmse"]) if part == "base" else float(np.mean(m["arm_q_rmse"]))
    return total / len(cases)


def coordinate_search(cases, params, part, axes, passes_n=2):
    """Each axis in turn over its candidate values, keeping the best; `axes`: [(key, [values])]."""
    best = evaluate(cases, params, part)
    for _ in range(passes_n):
        for key, values in axes:
            for v in values:
                trial = {**params, key: v}
                score = evaluate(cases, trial, part)
                if score < best - 1e-9:
                    best, params = score, trial
    return params, best


# --------------------------------------------------------------------------------------------------------------- CLI
def load_cases(runs, limit_s=None):
    cases = []
    for run in runs:
        series = read_series(run / "bag")
        replay = replay_input(series)
        if limit_s:
            replay["duration_s"] = min(replay["duration_s"], limit_s)
        cases.append((replay, series))
    return cases


def fit(train, out_dir, n_runs, limit_s):
    all_train = train
    train = spread(train, n_runs)
    cases = load_cases(train, limit_s)
    # starting values from the signals
    delays, lags_v, slips_v, lags_w, slips_w, windows = [], [], [], [], [], 0
    for _, s in cases:
        tc = np.array([x[0] for x in s["cmd_vel"]]); vc = np.array([x[1] for x in s["cmd_vel"]]); wc = np.array([x[2] for x in s["cmd_vel"]])
        to = np.array([x[0] for x in s["odom"]]); vo = np.array([x[2][0] for x in s["odom"]]); wo = np.array([x[2][1] for x in s["odom"]])
        d = cross_correlation_delay(tc, vc, to, vo)
        delays.append(d)
        tv, kv = fit_first_order(tc, vc, to, vo, d)
        tw, kw = fit_first_order(tc, wc, to, wo, d)
        lags_v.append(tv); slips_v.append(kv); lags_w.append(tw); slips_w.append(kw)
        ts = np.array([x[0] for x in s["cmd_vel_smoothed"]]); vs = np.array([x[1] for x in s["cmd_vel_smoothed"]])
        if len(ts):
            windows += len(monitor_windows(tc, vc, ts, vs))
    # the skid map from the steady parts of more runs (a cheap read, no replay), the slip folded into it
    skid_runs = spread(all_train, 24)
    samples = np.concatenate([steady_samples(read_series(r / "bag")) for r in skid_runs])
    skid = fit_skid_map(samples)
    start = {"base_command_delay_s": float(np.median(delays)), "base_lag_s": [float(np.median(lags_v)), float(np.median(lags_w))],
             "base_slip": [1.0, 1.0], "base_skid": skid, "skid_samples": int(len(samples)), "skid_runs": len(skid_runs)}
    base_start = {k: start[k] for k in ("base_command_delay_s", "base_lag_s", "base_slip", "base_skid")}
    lags = [0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5]
    base_params, base_score = coordinate_search(cases, base_start, "base", [
        ("base_command_delay_s", [0.0, 0.01, 0.02, 0.04, 0.06, 0.08, 0.1, 0.13]),
        ("base_lag_s", [[a, b] for a in lags for b in lags]),
    ], passes_n=2)
    arm_params, arm_score, resolved = fit_arm(cases, base_params)
    fitted = {**base_params, **arm_params}
    edges = [key for key, values in resolved if values and fitted.get(key) in (values[0], values[-1])]
    result = {"runs": [r.name for r in train], "replay_limit_s": limit_s, "start": start, "fitted": fitted,
              "base_objective": base_score, "arm_objective": arm_score, "monitor_windows": windows, "arm_at_grid_edge": edges}
    (out_dir / "calibration_fit.json").write_text(json.dumps(result, indent=1) + "\n")
    return result


def fit_arm(cases, base_params):
    """The arm's delays and servo gains by coordinate search (the base fixed): the state delay too, since Unity's JTC
    sees the joints through the transport (its tracking error includes that latency)."""
    arm_axes = [("arm_command_delay_s", [0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.08]),
                ("state_delay_s", [0.0, 0.01, 0.02, 0.03, 0.04, 0.06])]
    for j in range(6):
        arm_axes += [(f"kp.{j}", None), (f"kd.{j}", None)]
    import yaml  # container
    profile = yaml.safe_load(PROFILE.read_text())
    resolved = []
    for key, values in arm_axes:
        if values is None:
            base, j = key.split(".")
            v0 = profile["arm"][base][int(j)]
            factors = (0.5, 0.7, 1.0, 1.4, 2.0, 2.8, 4.0) if base == "kp" else (0.25, 0.35, 0.5, 0.7, 1.0, 1.4, 2.0)
            values = [round(v0 * f, 3) for f in factors]
        resolved.append((key, values))
    arm_params, arm_score = coordinate_search(cases, dict(base_params), "arm", resolved, passes_n=2)
    return {k: v for k, v in arm_params.items() if k not in base_params or k == "state_delay_s"}, arm_score, resolved


def validate(test, fitted, out_dir, n_runs, limit_s):
    test = spread(test, n_runs)
    rows = []
    for replay, series in load_cases(test, limit_s):
        out = plant_replay(replay, fitted)
        m = metrics(out, unity_on(series, out["t"]))
        rows.append({"run": None, **m, "passes": passes(m)})
    for row, run in zip(rows, test):
        row["run"] = run.name
    (out_dir / "calibration_validation.json").write_text(json.dumps(rows, indent=1) + "\n")
    return rows


def report(fit_result, rows, out_dir):
    """docs/experiments/mujoco-plant/calibration.md: the runs, the starting and fitted values, the held-out table."""
    f = fit_result["fitted"]
    lines = ["# MuJoCo plant calibration against Unity", "",
             "Generated by `tools/plant/calibrate_plant.py` (plan Task 7, spec Section 7). Unity runs with today's actuators "
             f"(`{CONFIG}`), split by cell; each replay covers at most {fit_result['replay_limit_s']:.0f} s of its run, open "
             "loop, in the ground-only world.", "",
             "## Fit", "", f"Runs ({len(fit_result['runs'])}): " + ", ".join(f"`{r}`" for r in fit_result["runs"]), "",
             f"Collision-monitor windows in the fit runs (/cmd_vel differing from /cmd_vel_smoothed): {fit_result['monitor_windows']}.", "",
             "| Parameter | Signal estimate | Fitted |", "|---|---|---|"]
    start = fit_result["start"]
    for key in ("base_command_delay_s", "base_lag_s", "base_slip"):
        lines.append(f"| {key} | {start.get(key)} | {f.get(key, start.get(key))} |")
    skid = f.get("base_skid", start.get("base_skid"))
    if skid:
        lines.append(f"| base_skid (from {start.get('skid_samples')} steady samples of {start.get('skid_runs')} runs) | share {skid['share']} | "
                     f"v {[round(x, 3) for x in skid['v']]}, w {[round(x, 3) for x in skid['w']]}, in place {round(skid['w_in_place'], 3)} |")
    for key in sorted(k for k in f if k not in start and k != "base_skid"):
        lines.append(f"| {key} | profile | {f[key]} |")
    lines += ["", f"Objectives after the search: base (speed + yaw-rate RMSE) {fit_result['base_objective']:.4f}, "
              f"arm (mean joint RMSE against Unity) {fit_result['arm_objective']:.5f} rad. Arm values at an end of their "
              f"search grid: {fit_result.get('arm_at_grid_edge') or 'none'}.", "",
              "## Held-out validation (Section 7.3)", "",
              "| Run | Speed RMSE (m/s) | Yaw-rate RMSE (rad/s) | 2 s drift max (m) | Arm p95 plant / Unity (mrad) | Pass |",
              "|---|---|---|---|---|---|"]
    for r in rows:
        arm = ", ".join(f"{1000 * a['plant']:.1f}/{1000 * a['unity']:.1f}" for a in r["arm_p95"])
        ok = ", ".join(k for k, v in r["passes"].items() if not v) or "all"
        lines.append(f"| `{r['run']}` | {r['base_speed_rmse']:.4f} | {r['yaw_rate_rmse']:.4f} | {r['drift_2s_max']:.4f} | {arm} | "
                     f"{'yes' if ok == 'all' else 'fails ' + ok} |")
    n_ok = sum(all(r["passes"].values()) for r in rows)
    lines += ["", f"Targets: arm p95 within ±30 % of Unity's or 2 mrad, speed RMSE ≤ {TARGETS['base_speed_rmse']} m/s, yaw-rate "
              f"RMSE ≤ {TARGETS['yaw_rate_rmse']} rad/s, 2 s drift ≤ {TARGETS['drift_2s']} m. Held-out runs passing every "
              f"target: {n_ok} of {len(rows)}.", ""]
    (out_dir / "calibration.md").write_text("\n".join(lines))


def write_profile(fitted, fit_file):
    """Writes the fitted values into the profile (after the user's review at CP3), with their provenance."""
    import yaml  # container
    text = PROFILE.read_text()
    prof = yaml.safe_load(text)
    def put(key, value, section):
        nonlocal text
        old = f"  {key}: {json.dumps(prof[section][key]) if isinstance(prof[section][key], list) else prof[section][key]}"
        new = f"  {key}: {json.dumps(value) if isinstance(value, list) else value}"
        if old not in text:
            raise RuntimeError(f"profile: cannot find '{old}'")
        text = text.replace(old, new, 1)
    for key in ("base_command_delay_s", "arm_command_delay_s", "state_delay_s", "base_lag_s", "base_slip"):
        if key in fitted:
            put(key, fitted[key], "realistic")
    if "base_skid" in fitted:
        skid = fitted["base_skid"]
        old = ("  base_skid:\n    share: [0.0]\n    v: [1.0]\n    w: [1.0]\n    w_in_place: 1.0")
        new = (f"  base_skid:\n    share: {json.dumps([round(x, 4) for x in skid['share']])}\n    v: {json.dumps([round(x, 4) for x in skid['v']])}\n"
               f"    w: {json.dumps([round(x, 4) for x in skid['w']])}\n    w_in_place: {round(skid['w_in_place'], 4)}")
        if old not in text:
            raise RuntimeError("profile: the identity base_skid block is not there")
        text = text.replace(old, new, 1)
    for gain in ("kp", "kd"):
        values = list(prof["arm"][gain])
        for j in range(6):
            values[j] = fitted.get(f"{gain}.{j}", values[j])
        put(gain, [float(v) for v in values], "arm")
    text = text.replace("realistic:\n", f"realistic:\n  # Calibrated by tools/plant/calibrate_plant.py against Unity ({fit_file}); see "
                        "docs/experiments/mujoco-plant/calibration.md\n", 1)
    PROFILE.write_text(text)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", choices=["fit", "fit-arm", "validate", "all"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--runs", type=int, default=8, help="runs per side (fit, validation)")
    ap.add_argument("--limit-s", type=float, default=40.0, help="replay at most this much of each run")
    ap.add_argument("--fitted", help="calibration_fit.json for validate alone")
    ap.add_argument("--write-profile", action="store_true", help="write the fitted values into the profile (after CP3)")
    args = ap.parse_args()
    out_dir = ROOT / args.out
    out_dir.mkdir(parents=True, exist_ok=True)
    runs = select_runs()
    train, test = split_by_cell(runs)
    print(f"{len(runs)} runs ({len(train)} train, {len(test)} held out: {sorted({cell_of(r) for r in test})})")
    fit_result = None
    if args.step in ("fit", "all"):
        fit_result = fit(train, out_dir, args.runs, args.limit_s)
        print("fitted", json.dumps(fit_result["fitted"]))
    fit_file = Path(args.fitted) if args.fitted else out_dir / "calibration_fit.json"
    if args.step == "fit-arm":  # the arm again on the fit's runs, the base as fitted
        prior = json.loads(fit_file.read_text())
        base_keys = ("base_command_delay_s", "base_lag_s", "base_slip", "base_skid")
        base_params = {k: prior["fitted"][k] for k in base_keys if k in prior["fitted"]}
        cases = load_cases([RUNS / r for r in prior["runs"]], args.limit_s)
        arm_params, arm_score, resolved = fit_arm(cases, base_params)
        prior["fitted"] = {**base_params, **arm_params}
        prior["arm_objective"] = arm_score
        prior["arm_at_grid_edge"] = [k for k, values in resolved if values and prior["fitted"].get(k) in (values[0], values[-1])]
        fit_file.write_text(json.dumps(prior, indent=1) + "\n")
        print("arm fitted", json.dumps(arm_params))
        return 0
    fit_result = fit_result or json.loads(fit_file.read_text())
    if args.step in ("validate", "all"):
        rows = validate(test, fit_result["fitted"], out_dir, args.runs, args.limit_s)
        for r in rows:
            print(r["run"], r["passes"], round(r["base_speed_rmse"], 4), round(r["yaw_rate_rmse"], 4), round(r["drift_2s_max"], 4))
        report(fit_result, rows, out_dir)
    if args.write_profile:
        write_profile(fit_result["fitted"], fit_file.relative_to(ROOT) if fit_file.is_absolute() else fit_file)
        print(f"wrote the fitted values into {PROFILE.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
