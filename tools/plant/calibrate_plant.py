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
import datetime as dt
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
PROFILE = ROOT / "ros2_ws/src/mobile_manipulator_plant/robots/mobile_manipulator/profile_unity.yaml"  # the Unity-calibrated profile
ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
CONFIG = "-gff-mff-bff"
TARGETS = {"arm_p95_rel": 0.30, "arm_p95_abs": 0.003, "base_speed_rmse": 0.02, "yaw_rate_rmse": 0.03, "drift_2s": 0.03}


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
    """[(v_cmd, w_cmd, v_achieved, w_achieved, segment)] at grid times g (20 ms) where the command held over the whole
    window [g - hold_s, g + settle_s], within max(0.02 m/s, 10 % |v|) and max(0.03 rad/s, 15 % |w|) (the band's width,
    commands held between messages); the achieved twist is the odometry at g + settle_s (past the lag). Consecutive
    steady grid times form one segment (0, 1, ...): samples of one segment are one observation, not many."""
    tc = np.array([x[0] for x in series["cmd_vel"]]); vc = np.array([x[1] for x in series["cmd_vel"]]); wc = np.array([x[2] for x in series["cmd_vel"]])
    to = np.array([x[0] for x in series["odom"]]); vo = np.array([x[2][0] for x in series["odom"]]); wo = np.array([x[2][1] for x in series["odom"]])
    if len(tc) < 10 or len(to) < 10:
        return np.zeros((0, 5))
    g = tc[0] + hold_s + 0.02 * np.arange(int((min(tc[-1], to[-1]) - settle_s - tc[0] - hold_s) / 0.02))
    i0 = np.clip(np.searchsorted(tc, g - hold_s, side="right") - 1, 0, None)
    i1 = np.searchsorted(tc, g + settle_s, side="right")
    ig = np.searchsorted(tc, g, side="right") - 1
    io = np.clip(np.searchsorted(to, g + settle_s, side="right") - 1, 0, None)
    rows, segment, last = [], -1, -2
    for k in range(len(g)):
        v, w = vc[ig[k]], wc[ig[k]]
        vs, ws = vc[i0[k]:i1[k]], wc[i0[k]:i1[k]]
        if np.ptp(vs) > max(0.02, 0.1 * abs(v)) or np.ptp(ws) > max(0.03, 0.15 * abs(w)):
            continue
        if k != last + 1:
            segment += 1
        last = k
        rows.append((v, w, vo[io[k]], wo[io[k]], segment))
    return np.array(rows) if rows else np.zeros((0, 5))


# Unity's own in-place yaw trial (docs/unity-skid-steer-base-controller.md: (0, 0.4) commanded, 0.442 rad/s achieved)
UNITY_W_IN_PLACE = 0.442 / 0.4


def fit_skid_map(samples, knots=(0.0, 0.15, 0.5, 0.85, 1.25, 2.0), track=1.5, min_n=30, min_segments=5):
    """The base's skid map from steady samples (steady_samples' rows, segment ids unique across runs): the median
    achieved / commanded speed and yaw rate in bins around each knot of the inner wheels' share |w| (track / 2) / v
    (driving forward, v >= 0.05; the yaw ratio only where |w| >= 0.05), and the median yaw factor turning in place
    (|v| < 0.02, |w| >= 0.1). A bin needs min_n samples from min_segments distinct segments, else it takes its
    neighbour's value (identity without any); turning in place without the data takes Unity's own measurement.
    `bins` reports n, segments and the ratios' IQR per bin."""
    samples = np.asarray(samples).reshape(-1, 5)
    knots = list(knots)
    v, w, va, wa, seg = samples.T
    driving = v >= 0.05
    share = np.where(driving, np.abs(w) * 0.5 * track / np.maximum(v, 1e-9), np.inf)
    edges = [0.0] + [(a + b) / 2 for a, b in zip(knots, knots[1:])] + [np.inf]
    def stat(mask, ratio):
        n, segments = int(mask.sum()), len(np.unique(seg[mask]))
        r = ratio[mask]
        fitted = n >= min_n and segments >= min_segments
        iqr = float(np.subtract(*np.percentile(r, [75, 25]))) if n else None
        return (float(np.median(r)) if fitted else None), {"n": n, "segments": segments, "iqr": iqr, "fitted": fitted}
    kv, kw, bins = [], [], []
    with np.errstate(divide="ignore", invalid="ignore"):
        rv, rw = va / v, wa / w
    for knot, lo, hi in zip(knots, edges, edges[1:]):
        m = driving & (share >= lo) & (share < hi)
        a, sv = stat(m, rv)
        b, sw = stat(m & (np.abs(w) >= 0.05), rw)
        kv.append(a); kw.append(b)
        bins.append({"share": knot, **sv, "w": sw})
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
    wp, sp = stat(in_place, rw)
    if wp is None:
        wp, source = UNITY_W_IN_PLACE, "Unity measurement (docs/unity-skid-steer-base-controller.md)"
    else:
        source = f"fitted ({sp['n']} samples, {sp['segments']} segments)"
    return {"share": knots, "v": fill(kv), "w": fill(kw), "w_in_place": wp, "w_in_place_source": source,
            "bins": bins, "in_place": sp}


def transport_latency(stamped, clock):
    """The median receive latency (s) of a stamped topic: its bag time minus the bag time of its sim-time stamp, read
    through /clock ([(bag time, sim time)]); stamps outside the clock's span are skipped."""
    tb, ts = np.array([x[0] for x in clock]), np.array([x[1] for x in clock])
    order = np.argsort(ts, kind="stable")
    tb, ts = tb[order], ts[order]
    rb, rs = np.array([x[0] for x in stamped]), np.array([x[1] for x in stamped])
    inside = (rs >= ts[0]) & (rs <= ts[-1])
    return float(np.median(rb[inside] - np.interp(rs[inside], ts, tb)))


def task_window(task, clock):
    """(start, end) of a run's task on the bag clock: task.json's started_s / finished_s (sim time) through /clock."""
    tb, ts = np.array([x[0] for x in clock]), np.array([x[1] for x in clock])
    order = np.argsort(ts, kind="stable")
    return tuple(float(np.interp(task[k], ts[order], tb[order])) for k in ("started_s", "finished_s"))


# ------------------------------------------------------------------------------------------------------- bag series
def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_series(bag):
    """The signals a replay and its comparison need, on the bag's receive clock (s). Also /clock ([(bag time, sim
    time)]) and the /odom stamps ([(bag time, sim stamp)]), which measure the odometry's transport latency."""
    from rosbags.highlevel import AnyReader  # container only
    s = {"cmd_vel": [], "cmd_vel_smoothed": [], "trajectories": [], "odom": [], "odom_stamps": [], "joints": [],
         "controller": [], "clock": []}
    with AnyReader([Path(bag)]) as r:
        conns = [c for c in r.connections if c.topic in ("/cmd_vel", "/cmd_vel_smoothed", "/arm_controller/joint_trajectory",
                                                         "/odom", "/joint_states", "/arm_controller/controller_state", "/clock")]
        for c, t_ns, raw in r.messages(connections=conns):
            t = t_ns * 1e-9
            m = r.deserialize(raw, c.msgtype)
            if c.topic in ("/cmd_vel", "/cmd_vel_smoothed"):
                s[c.topic[1:]].append((t, m.linear.x, m.angular.z))
            elif c.topic == "/clock":
                s["clock"].append((t, m.clock.sec + 1e-9 * m.clock.nanosec))
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
                s["odom_stamps"].append((t, m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec))
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


def window_on_task(series, run):
    """The series' replay window set to the run's task (task.json through /clock), clipped to the recording."""
    task = json.loads((Path(run) / "task.json").read_text())
    if task.get("started_s") is None or task.get("finished_s") is None or not series["clock"]:
        return series
    t0, end = task_window(task, series["clock"])
    series["t0"], series["end"] = max(t0, series["t0"]), min(end, series["end"])
    return series


def _at(samples, t):
    """The latest sample at or before t (the first one if none)."""
    times = [x[0] for x in samples]
    return samples[max(0, int(np.searchsorted(times, t, side="right")) - 1)]


def replay_input(series):
    """plant_replay's input from a recording's series: from t0 (the task start) to its end, the state at t0, every
    command after t0 on its receipt time."""
    t0 = series["t0"]
    return {
        "start": {"base": list(_at(series["odom"], t0)[1]), "q": list(_at(series["joints"], t0)[1])},
        "duration_s": series["end"] - t0,
        "sample_period_s": 0.02,
        "world": "ground",
        "base_commands": [[t - t0, v, w] for t, v, w in series["cmd_vel"] if t >= t0],
        "arm_streams": [{"t": t - t0, "points": [{"t": pt, "q": q, "v": v} for pt, q, v in pts]}
                        for t, pts in series["trajectories"] if t >= t0],
    }


def unity_on(series, t_grid):
    """The recorded Unity signals resampled on the replay's time grid (from t0). The arm from the JTC's own state
    message: q its feedback (the joints as the JTC saw them), q_ref its reference, so Unity's tracking error is one
    message's reference - feedback."""
    t0 = series["t0"]
    to = np.array([x[0] for x in series["odom"]]) - t0
    pose = np.array([x[1] for x in series["odom"]])
    twist = np.array([x[2] for x in series["odom"]])
    tc = np.array([x[0] for x in series["controller"]]) - t0
    ref = np.array([x[1] for x in series["controller"]])
    fb = np.array([x[2] for x in series["controller"]])
    yaw = np.unwrap(pose[:, 2])
    return {
        "t": t_grid,
        "base": np.stack([np.interp(t_grid, to, pose[:, 0]), np.interp(t_grid, to, pose[:, 1]), np.interp(t_grid, to, yaw)], 1),
        "twist": np.stack([np.interp(t_grid, to, twist[:, i]) for i in range(2)], 1),
        "q": np.stack([np.interp(t_grid, tc, fb[:, i]) for i in range(6)], 1),
        "q_ref": np.stack([np.interp(t_grid, tc, ref[:, i]) for i in range(6)], 1),
    }


def hard_turn_fraction(series, track=1.5, share=1.0):
    """The fraction of the task's driving commands (v >= 0.05) whose inner wheels' share is at least `share` (inner
    wheels at rest or reversing)."""
    c = np.array([(v, w) for t, v, w in series["cmd_vel"] if series["t0"] <= t <= series["end"] and v >= 0.05]).reshape(-1, 2)
    return float(np.mean(np.abs(c[:, 1]) * 0.5 * track / c[:, 0] >= share)) if len(c) else 0.0


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
# Unity's ArticulationBody drive gains (docs/unity-arm-controller.md), the arm fit's centre
UNITY_KP = [2000.0, 6000.0, 3500.0, 500.0, 650.0, 300.0]
UNITY_KD = [240.0, 360.0, 180.0, 40.0, 35.0, 28.0]
SKID_KEYS = ("share", "v", "w", "w_in_place")


def expand(params):
    """plant_replay's overrides from the fit's parameters: a joint's stiffness factor f gives kp = f kp_Unity and
    kd = sqrt(f) kd_Unity (Unity's damping ratio kept); the skid map without its report fields."""
    out = {}
    for key, value in params.items():
        if key.startswith("stiffness."):
            j = int(key.split(".")[1])
            out[f"kp.{j}"] = value * UNITY_KP[j]
            out[f"kd.{j}"] = math.sqrt(value) * UNITY_KD[j]
        elif key == "base_skid":
            out[key] = {k: value[k] for k in SKID_KEYS}
        else:
            out[key] = value
    return out


def plant_replay(replay, params):
    """Runs the plant_replay binary (sourced workspace) on one input with parameter overrides (expand); its output as
    arrays."""
    params = expand(params)
    exe = Path(os.environ.get("ROS_WS", "/workspaces/mm-motion-planning/ros2_ws")) / \
        "install/mobile_manipulator_campaign/lib/mobile_manipulator_campaign/plant_replay"
    with tempfile.TemporaryDirectory() as tmp:
        src, dst = Path(tmp) / "in.json", Path(tmp) / "out.json"
        src.write_text(json.dumps({**replay, "params": params}))
        subprocess.run([str(exe), "--input", str(src), "--out", str(dst)], check=True, stdout=subprocess.DEVNULL)
        out = json.loads(dst.read_text())
    return {k: np.asarray(v) for k, v in out.items() if k != "params"}


def objective(m, part):
    """One run's objective: "base" the speed + yaw-rate RMSE; "arm" Section 7.3's two arm parts, in rad, averaged over
    the joints: the joint-position RMSE against Unity's (the JTC's feedback: where the joints went) plus |tracking-error
    p95 plant - Unity| (how far behind the reference the JTC saw them). Either alone is degenerate: the error's size
    lets a soft, late arm move differently; the joints alone miss the staleness of the JTC's feedback."""
    if part == "base":
        return m["base_speed_rmse"] + m["yaw_rate_rmse"]
    return float(np.mean(m["arm_q_rmse"]) + np.mean([abs(a["plant"] - a["unity"]) for a in m["arm_p95"]]))


def evaluate(cases, params, part):
    """The mean objective of `part` over the cases."""
    total = 0.0
    for replay, series in cases:
        out = plant_replay(replay, params)
        total += objective(metrics(out, unity_on(series, out["t"])), part)
    return total / len(cases)


def coordinate_search(cases, params, part, axes, passes_n=2, min_gain=0.01, evaluate_fn=None):
    """Each axis in turn over its candidate values, keeping the best; `axes`: [(key, [values])]. A value replaces the
    current one only if it lowers the objective by at least `min_gain` of it: on a flat objective the start (Unity's
    gains, the signal estimates) stays instead of drifting to a grid edge."""
    evaluate_fn = evaluate_fn or evaluate
    best = evaluate_fn(cases, params, part)
    for _ in range(passes_n):
        for key, values in axes:
            for v in values:
                trial = {**params, key: v}
                score = evaluate_fn(cases, trial, part)
                if score < best * (1.0 - min_gain):
                    best, params = score, trial
    return params, best


# --------------------------------------------------------------------------------------------------------------- CLI
def limit_cases(cases, limit_s):
    """The cases with each replay cut to at most limit_s of its task (None: the whole task)."""
    out = []
    for replay, series in cases:
        full = replay.get("full_duration_s", replay["duration_s"])
        out.append(({**replay, "full_duration_s": full, "duration_s": min(full, limit_s) if limit_s else full}, series))
    return out


def load_cases(runs, limit_s=None):
    """(replay, series) per run, the replay on the run's task window (at most limit_s of it)."""
    cases = []
    for run in runs:
        series = window_on_task(read_series(run / "bag"), run)
        cases.append((replay_input(series), series))
    return limit_cases(cases, limit_s)


def _signals(s):
    tc = np.array([x[0] for x in s["cmd_vel"]]); vc = np.array([x[1] for x in s["cmd_vel"]]); wc = np.array([x[2] for x in s["cmd_vel"]])
    to = np.array([x[0] for x in s["odom"]]); vo = np.array([x[2][0] for x in s["odom"]]); wo = np.array([x[2][1] for x in s["odom"]])
    return tc, vc, wc, to, vo, wo


def fit(train, out_dir, n_runs, limit_s):
    all_train = train
    train = spread(train, n_runs)
    whole = load_cases(train)               # the arm fit: whole tasks (the arm's motion is often late in a task)
    cases = limit_cases(whole, limit_s)     # the base fit: at most limit_s of each (its search is long)
    # the odometry's transport latency, measured (its stamps against /clock): the base's state delay, not searched
    latencies = [transport_latency(s["odom_stamps"], s["clock"]) for _, s in cases if s["clock"]]
    base_state_delay = round(float(np.median(latencies)), 4)
    # starting values from the signals: the command-to-odometry delay less the measured latency seeds the command delay
    delays, lags_v, lags_w, windows = [], [], [], 0
    for _, s in cases:
        tc, vc, wc, to, vo, wo = _signals(s)
        d = cross_correlation_delay(tc, vc, to, vo)
        delays.append(max(0.0, d - base_state_delay))
        lags_v.append(fit_first_order(tc, vc, to, vo, d)[0])
        lags_w.append(fit_first_order(tc, wc, to, wo, d)[0])
        ts = np.array([x[0] for x in s["cmd_vel_smoothed"]]); vs = np.array([x[1] for x in s["cmd_vel_smoothed"]])
        if len(ts):
            windows += len(monitor_windows(tc, vc, ts, vs))
    # the skid map from the steady parts of more training runs (a cheap read, no replay); segment ids unique per run
    skid_runs = spread(all_train, 24)
    parts, offset = [], 0
    for r in skid_runs:
        part = steady_samples(read_series(r / "bag"))
        if len(part):
            part[:, 4] += offset
            offset = int(part[:, 4].max()) + 1
            parts.append(part)
    samples = np.concatenate(parts) if parts else np.zeros((0, 5))
    skid = fit_skid_map(samples)
    start = {"base_command_delay_s": float(np.median(delays)), "base_lag_s": [float(np.median(lags_v)), float(np.median(lags_w))],
             "base_slip": [1.0, 1.0], "base_skid": skid, "base_state_delay_s": base_state_delay,
             "odom_latency_s": [round(x, 6) for x in latencies], "skid_samples": int(len(samples)),
             "skid_segments": int(len(np.unique(samples[:, 4]))), "skid_runs": [r.name for r in skid_runs]}
    base_start = {k: start[k] for k in ("base_command_delay_s", "base_state_delay_s", "base_lag_s", "base_slip", "base_skid")}
    lags = [0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5]
    base_params, base_score = coordinate_search(cases, base_start, "base", [
        ("base_command_delay_s", [0.0, 0.01, 0.02, 0.04, 0.06, 0.08, 0.1, 0.13]),
        ("base_lag_s", [[a, b] for a in lags for b in lags]),
    ], passes_n=2)
    arm_params, arm_score, resolved = fit_arm(whole, base_params)
    fitted = {**base_params, **arm_params}
    result = {"runs": [r.name for r in train], "replay_limit_s": limit_s, "start": start, "fitted": fitted,
              "base_objective": base_score, "arm_objective": arm_score, "monitor_windows": windows,
              "arm_at_grid_edge": at_grid_edge(fitted, resolved)}
    (out_dir / "calibration_fit.json").write_text(json.dumps(result, indent=1) + "\n")
    return result


# The arm's search grids: one stiffness factor per joint around Unity's gains (sqrt(2) steps, 1/8 .. 8), the delays
STIFFNESS = [round(2 ** (k / 2), 4) for k in range(-6, 7)]
ARM_AXES = [("arm_command_delay_s", [0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.1]),
            ("arm_state_delay_s", [0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.1])] + \
           [(f"stiffness.{j}", STIFFNESS) for j in range(6)]


def at_grid_edge(fitted, resolved):
    """The searched keys whose fitted value is an end of their grid (0 delays excepted: a delay cannot be negative)."""
    return [k for k, values in resolved if fitted.get(k) == values[-1] or (fitted.get(k) == values[0] and values[0] != 0.0)]


def fit_arm(cases, base_params):
    """The arm's delays and per-joint stiffness factors by coordinate search (the base fixed), from Unity's gains and
    no delay. The state delay too: Unity's JTC sees the joints through the transport, so its tracking error includes
    that latency (Unity's raw joint states are not recorded, so it is fitted, not measured)."""
    start = {**base_params, "arm_command_delay_s": 0.0, "arm_state_delay_s": 0.0, **{f"stiffness.{j}": 1.0 for j in range(6)}}
    arm_params, arm_score = coordinate_search(cases, start, "arm", ARM_AXES, passes_n=2)
    return {k: v for k, v in arm_params.items() if k not in base_params}, arm_score, ARM_AXES


def fresh_runs(train, used):
    """One training run per training cell that neither the replay fit nor the skid map read (a check the fit did not
    see, from cells it did)."""
    by_cell = {}
    for r in train:
        if r.name not in used:
            by_cell.setdefault(cell_of(r), r)
    return [by_cell[c] for c in sorted(by_cell)]


def validate(runs, fitted, out_file, limit_s=None):
    """Whole task windows (limit_s None) replayed with the fitted values against their recordings."""
    rows = []
    for run, (replay, series) in zip(runs, load_cases(runs, limit_s)):
        out = plant_replay(replay, fitted)
        m = metrics(out, unity_on(series, out["t"]))
        rows.append({"run": run.name, "duration_s": round(replay["duration_s"], 2),
                     "hard_turn_fraction": round(hard_turn_fraction(series), 4), **m, "passes": passes(m)})
    out_file.write_text(json.dumps(rows, indent=1) + "\n")
    return rows


def _table(rows):
    lines = ["| Run | Task (s) | Hard turns | Speed RMSE (m/s) | Yaw-rate RMSE (rad/s) | 2 s drift max (m) | "
             "Arm p95 plant / Unity (mrad) | Arm q RMSE (mrad) | Pass |", "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        arm = ", ".join(f"{1000 * a['plant']:.1f}/{1000 * a['unity']:.1f}" for a in r["arm_p95"])
        rmse = ", ".join(f"{1000 * x:.1f}" for x in r["arm_q_rmse"])
        failed = ", ".join(k for k, v in r["passes"].items() if not v)
        lines.append(f"| `{r['run']}` | {r['duration_s']:.0f} | {100 * r['hard_turn_fraction']:.0f} % | {r['base_speed_rmse']:.4f} | "
                     f"{r['yaw_rate_rmse']:.4f} | {r['drift_2s_max']:.4f} | {arm} | {rmse} | {'fails ' + failed if failed else 'yes'} |")
    tally = {k: sum(r["passes"][k] for r in rows) for k in ("arm", "base_speed", "yaw_rate", "drift")}
    lines += ["", f"Passing every target: {sum(all(r['passes'].values()) for r in rows)} of {len(rows)}. Per target: "
              + ", ".join(f"{k} {v}/{len(rows)}" for k, v in tally.items()) + "."]
    return lines


def report(fit_result, held_out, fresh, out_dir, n_runs, n_train, n_test):
    """docs/experiments/mujoco-plant/calibration.md: the data, the starting and fitted values, the two validations;
    calibration_notes.md beside it (hand-written analysis) is appended when present."""
    f, start = fit_result["fitted"], fit_result["start"]
    skid = f["base_skid"]
    cells = lambda rows: len({cell_of(Path(r["run"])) for r in rows})
    lines = ["# MuJoCo plant calibration against Unity", "",
             "Generated by `tools/plant/calibrate_plant.py` (plan Task 7, spec Section 7). Do not edit by hand; the analysis is "
             "in the notes at the end.", "",
             "## Data", "",
             f"Unity whole-body runs with today's actuators (`{CONFIG}`): {n_runs}, split by cell into {n_train} training runs and "
             f"{n_test} held-out runs ({cells(held_out)} cells). Every replay starts at the run's task start (task.json through "
             "/clock), open loop, in the ground-only world.", "",
             f"- Replay fit: {len(fit_result['runs'])} training runs; the base on at most {fit_result['replay_limit_s']:.0f} s of each task, "
             "the arm on the whole tasks: "
             + ", ".join(f"`{r}`" for r in fit_result["runs"]) + ".",
             f"- Skid map: the steady parts of {len(start['skid_runs'])} training runs ({start['skid_samples']} samples in "
             f"{start['skid_segments']} steady segments).",
             f"- Fresh check: {len(fresh)} more training runs, one per training cell, read by neither fit.",
             f"- Held out: {len(held_out)} runs, whole task windows.", "",
             f"Collision-monitor windows in the fit runs (/cmd_vel differing from /cmd_vel_smoothed): {fit_result['monitor_windows']}.", "",
             "## Fit", "",
             "Odometry transport latency (receive time against the receive time of the stamp's /clock tick), measured per "
             "fit run: " + ", ".join(f"{1000 * x:.3f}" for x in start["odom_latency_s"]) + f" ms; the median, "
             f"{1000 * start['base_state_delay_s']:.1f} ms, is the base's state delay.", "",
             "| Parameter | Signal estimate | Fitted |", "|---|---|---|"]
    for key in ("base_command_delay_s", "base_state_delay_s", "base_lag_s", "base_slip"):
        lines.append(f"| {key} | {start.get(key)} | {f.get(key)} |")
    for key in ("arm_command_delay_s", "arm_state_delay_s"):
        lines.append(f"| {key} | searched from 0 | {f.get(key)} |")
    for j in range(6):
        k = f.get(f"stiffness.{j}", 1.0)
        lines.append(f"| {ARM[j]} stiffness | Unity kp {UNITY_KP[j]:.0f}, kd {UNITY_KD[j]:.0f} | x{k}: kp {k * UNITY_KP[j]:.1f}, "
                     f"kd {math.sqrt(k) * UNITY_KD[j]:.2f} |")
    lines += ["", f"Objectives after the search: base (speed + yaw-rate RMSE) {fit_result['base_objective']:.4f}, arm (mean "
              f"joint-position RMSE against Unity's + mean |tracking-error p95 plant - Unity|) {fit_result['arm_objective']:.5f} rad. Values at an end of their "
              f"search grid: {fit_result.get('arm_at_grid_edge') or 'none'}.", "",
              "### Skid map", "",
              "Achieved / commanded by the inner wheels' share |w| (b_eff / 2) / v (median; a knot needs 30 samples from 5 steady "
              "segments, else it takes its neighbour's value).", "",
              "| Share | v ratio | n | segments | IQR | w ratio | n | segments | IQR |", "|---|---|---|---|---|---|---|---|---|"]
    fmt = lambda x: "-" if x is None else f"{x:.3f}"
    for b, v, w in zip(skid["bins"], skid["v"], skid["w"]):
        lines.append(f"| {b['share']} | {v:.4f}{'' if b['fitted'] else ' (filled)'} | {b['n']} | {b['segments']} | {fmt(b['iqr'])} | "
                     f"{w:.4f}{'' if b['w']['fitted'] else ' (filled)'} | {b['w']['n']} | {b['w']['segments']} | {fmt(b['w']['iqr'])} |")
    lines += ["", f"Turning in place: {skid['w_in_place']:.4f}, {skid['w_in_place_source']}.", "",
              "## Held-out validation (Section 7.3)", ""] + _table(held_out) + \
             ["", "## Fresh check on unused training runs", ""] + _table(fresh) + \
             ["", f"Targets: arm p95 within ±30 % of Unity's or 3 mrad (2 mrad in the spec; 3 since CP3, 2026-11-06, for Unity's PhysX jitter floor), speed RMSE ≤ {TARGETS['base_speed_rmse']} m/s, yaw-rate "
              f"RMSE ≤ {TARGETS['yaw_rate_rmse']} rad/s, 2 s drift ≤ {TARGETS['drift_2s']} m. Hard turns: the share of the task's "
              "driving commands with the inner wheels at rest or reversing (share ≥ 1).", ""]
    notes = out_dir / "calibration_notes.md"
    if notes.exists():
        lines += [notes.read_text().rstrip(), ""]
    (out_dir / "calibration.md").write_text("\n".join(lines))


def write_profile(fitted, fit_file, date, runs):
    """Writes the fitted values into the profile (after the user's review at CP3) with their provenance; idempotent:
    every value is replaced whatever it was, and the provenance line replaces any earlier one."""
    import re
    text = PROFILE.read_text()
    def put(key, value):
        nonlocal text
        rendered = json.dumps(value) if isinstance(value, list) else repr(value)
        text, n = re.subn(rf"^(  {key}: ).*$", lambda m: m.group(1) + rendered, text, count=1, flags=re.M)
        if n != 1:
            raise RuntimeError(f"profile: no '{key}:' line")
    for key in ("base_command_delay_s", "arm_command_delay_s", "base_state_delay_s", "arm_state_delay_s", "base_lag_s", "base_slip"):
        if key in fitted:
            put(key, fitted[key])
    if "base_skid" in fitted:
        skid = fitted["base_skid"]
        block = (f"  base_skid:\n    share: {json.dumps([round(x, 4) for x in skid['share']])}\n"
                 f"    v: {json.dumps([round(x, 4) for x in skid['v']])}\n    w: {json.dumps([round(x, 4) for x in skid['w']])}\n"
                 f"    w_in_place: {round(skid['w_in_place'], 4)}\n")
        text, n = re.subn(r"^  base_skid:\n(?:    .*\n?)+", block, text, count=1, flags=re.M)
        if n != 1:
            raise RuntimeError("profile: no base_skid block")
    import yaml  # container
    prof = yaml.safe_load(text)
    for gain, unity in (("kp", UNITY_KP), ("kd", UNITY_KD)):
        values = [float(v) for v in prof["arm"][gain]]
        for j in range(6):
            if f"stiffness.{j}" in fitted:
                k = fitted[f"stiffness.{j}"]
                values[j] = round(k * unity[j] if gain == "kp" else math.sqrt(k) * unity[j], 3)
        put(gain, values)
    text = re.sub(r"^  # (Calibrated by tools/plant/calibrate_plant\.py|Placeholders until the calibration).*\n", "", text, flags=re.M)
    text = text.replace("realistic:\n", f"realistic:\n  # Calibrated by tools/plant/calibrate_plant.py on {date} from {len(runs)} Unity "
                        f"runs ({fit_file}); see docs/experiments/mujoco-plant/calibration.md\n", 1)
    PROFILE.write_text(text)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", choices=["fit", "fit-arm", "validate", "all"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--runs", type=int, default=8, help="training runs the replay fit uses")
    ap.add_argument("--limit-s", type=float, default=40.0, help="the fit replays at most this much of each task")
    ap.add_argument("--fitted", help="calibration_fit.json for fit-arm / validate alone")
    ap.add_argument("--write-profile", action="store_true", help="write the fitted values into the profile (after CP3)")
    args = ap.parse_args()
    out_dir = ROOT / args.out
    out_dir.mkdir(parents=True, exist_ok=True)
    runs = select_runs()
    train, test = split_by_cell(runs)
    print(f"{len(runs)} runs ({len(train)} train, {len(test)} held out: {sorted({cell_of(r) for r in test})})", flush=True)
    fit_file = Path(args.fitted) if args.fitted else out_dir / "calibration_fit.json"
    fit_result = None
    if args.step in ("fit", "all"):
        fit_result = fit(train, out_dir, args.runs, args.limit_s)
        print("fitted", json.dumps({k: v for k, v in fit_result["fitted"].items() if k != "base_skid"}), flush=True)
    if args.step == "fit-arm":  # the arm again on the fit's runs, the base as fitted
        prior = json.loads(fit_file.read_text())
        base_keys = ("base_command_delay_s", "base_state_delay_s", "base_lag_s", "base_slip", "base_skid")
        base_params = {k: prior["fitted"][k] for k in base_keys if k in prior["fitted"]}
        cases = load_cases([RUNS / r for r in prior["runs"]])  # whole tasks
        arm_params, arm_score, resolved = fit_arm(cases, base_params)
        prior["fitted"] = {**base_params, **arm_params}
        prior["arm_objective"] = arm_score
        prior["arm_at_grid_edge"] = at_grid_edge(prior["fitted"], resolved)
        fit_file.write_text(json.dumps(prior, indent=1) + "\n")
        print("arm fitted", json.dumps(arm_params))
        return 0
    fit_result = fit_result or json.loads(fit_file.read_text())
    if args.step in ("validate", "all"):
        held_out = validate(test, fit_result["fitted"], out_dir / "calibration_validation.json")
        used = set(fit_result["runs"]) | set(fit_result["start"]["skid_runs"])
        fresh = validate(fresh_runs(train, used), fit_result["fitted"], out_dir / "calibration_fresh.json")
        for r in held_out + fresh:
            print(r["run"], r["passes"], round(r["base_speed_rmse"], 4), round(r["yaw_rate_rmse"], 4), round(r["drift_2s_max"], 4))
        report(fit_result, held_out, fresh, out_dir, len(runs), len(train), len(test))
    if args.write_profile:
        write_profile(fit_result["fitted"], fit_file.relative_to(ROOT) if fit_file.is_absolute() else fit_file,
                      dt.date.today().isoformat(), fit_result["runs"])
        print(f"wrote the fitted values into {PROFILE.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
