"""B5's prediction model replayed against the MuJoCo plant's logged motion (E2 plan Tasks 2-3; spec
docs/superpowers/specs/2026-11-11-e2-tracking-prediction-design.md Section 3.1).

A run recorded with `plant_campaign --diagnostics` carries, per State frame, the command B5 requested (`diag_cmd`) and
the deepest gated pair (`diag_pair`, `diag_grad`, `diag_kink`). This tool rolls B5's own model (the `model_step`
binary, the generated function B5 uses) from a frame's pose with the commands that followed and compares with the
measured pose: how far, and along which degree of freedom, the plant departs from what B5 predicts.

Time convention (E2 Task 1 review F2): record j's diagnostics were evaluated and its command sent at t[j] - 0.02, at
record j-1's B5 state; the transition pose[j-1] -> pose[j] ran under cmd[j]. A rollout from i0 to i1 uses
cmd[i0+1 .. i1]. The measured twist and the arm reference velocity are samples at t[j] (after the transition), so the
"twist" inputs take them at their sample times (`control`).

Standard library only (the host's Python 3.9).
"""
import argparse
import math
import shlex
import subprocess
import sys
import threading
from pathlib import Path

import plant_stream as ps

ROOT = Path(__file__).resolve().parents[2]
CONTAINER = "mm-motion-planning-ma-robot-sim-1"
WORKSPACE = "/workspaces/mm-motion-planning"
BINARY = "ros2_ws/install/mobile_manipulator_mpc/lib/mobile_manipulator_mpc/model_step"
BINARY_PATH = ROOT / BINARY
MAX_GAP_S = 0.03  # consecutive frames further apart than this break a window (a cycle is 0.02 s)


def records(log_path):
    """(hello, records): one dict per State frame with the pose, B5's state, the measured twist, the arm trajectory's
    velocity, the requested command and the gate diagnostics (depth, depth_stage1, pair, grad, kink: None where the
    frame does not carry them, e.g. logs from before diag_kink). ValueError, naming the file, for a log recorded
    without --diagnostics. Input invariant: the producer's State frames come every 0.02 s in time order."""
    log = ps.RunLog(log_path)
    out = []
    for kind, header, blocks in log.frames():
        if kind != ps.STATE:
            continue
        if "diag_cmd" not in header:
            raise ValueError(f"{log_path}: recorded without --diagnostics (no diag_cmd)")
        diag = header.get("diag")
        executed = blocks[2]
        grad, kink = header.get("diag_grad"), header.get("diag_kink")
        # a field the log does not carry stays None (unavailable), never a measured zero (Task 2 review F3): the
        # analyses that need it skip or reject the frame, naming the field
        out.append({"t": header["t"], "goal": header.get("goal"), "mode": log.header.get("mode"),
                    "pose": list(blocks[0]), "b5": list(blocks[1]),
                    "twist": list(executed[:2]), "qd_ref": list(executed[2:8]), "cmd": list(header["diag_cmd"]),
                    "depth": diag[0] if diag else None, "depth_stage1": diag[1] if diag else None,
                    "pair": header.get("diag_pair"), "grad": list(grad) if grad is not None else None,
                    "kink": list(kink) if kink is not None else None})
    return log.header, out


def windows(recs, horizon_s, stride=1):
    """(i0, i1) index pairs horizon_s apart (in 0.02 s cycles) with the same goal throughout and no gap between
    consecutive frames."""
    n = max(1, int(round(horizon_s / 0.02)))
    out = []
    for i0 in range(0, len(recs) - n, stride):
        i1 = i0 + n
        if any(recs[j]["goal"] != recs[i0]["goal"] for j in range(i0 + 1, i1 + 1)):
            continue
        if any(not 0.0 < recs[j]["t"] - recs[j - 1]["t"] <= MAX_GAP_S for j in range(i0 + 1, i1 + 1)):
            continue
        out.append((i0, i1))
    return out


def control(recs, j, inputs):
    """The control of the transition records[j-1] -> records[j], each series at its own sample time (Task 1 response
    check R2): "cmd" = record j's requested command (sent at the interval's start); "twist" = the mean of the measured
    twist sampled at both ends (a trapezoid: the base's motion between samples is not logged) with the arm reference
    velocity sampled at the interval's start (record j-1's), which the JTC holds until its next tick. The sub-cycle
    phase of the arm command delay stays in what the plant does; Task 3's alignment fits it."""
    if inputs == "cmd":
        return list(recs[j]["cmd"])
    if inputs == "twist":
        a, b = recs[j - 1], recs[j]
        return [0.5 * (a["twist"][i] + b["twist"][i]) for i in range(2)] + list(a["qd_ref"])
    raise ValueError(f"inputs must be 'cmd' or 'twist', not {inputs!r}")


def rollout_many(recs, wins, inputs, step):
    """The model's state at each window's i1 from pose[i0], all windows stepped in lockstep (one `step` batch per
    cycle). `step(list[(dt, x, u)]) -> list[x_next]`."""
    states = [list(recs[i0]["pose"]) for i0, _ in wins]
    longest = max((i1 - i0 for i0, i1 in wins), default=0)
    for k in range(1, longest + 1):
        live = [w for w, (i0, i1) in enumerate(wins) if i0 + k <= i1]
        batch = []
        for w in live:
            j = wins[w][0] + k
            batch.append((recs[j]["t"] - recs[j - 1]["t"], states[w], control(recs, j, inputs)))
        for w, x in zip(live, step(batch)):
            states[w] = list(x)
    return states


def rollout(recs, i0, i1, inputs, step):
    """The model's state at i1 from pose[i0] with the controls of records i0+1 .. i1."""
    return rollout_many(recs, [(i0, i1)], inputs, step)[0]


def error(recs, i0, i1, predicted):
    """Measured minus predicted at i1: forward and lateral (m) in the body frame at i0, yaw (rad, wrapped), q (6)."""
    measured, yaw0 = recs[i1]["pose"], recs[i0]["pose"][2]
    dx, dy = measured[0] - predicted[0], measured[1] - predicted[1]
    c, s = math.cos(yaw0), math.sin(yaw0)
    return {"forward": c * dx + s * dy, "lateral": -s * dx + c * dy,
            "yaw": math.remainder(measured[2] - predicted[2], 2 * math.pi),
            "q": [measured[3 + i] - predicted[3 + i] for i in range(6)]}


class _Stepper:
    """A persistent model_step process (Task 2 review F1, F2). Each call writes its batch from a thread while it reads
    the answers, so a batch larger than the pipes cannot deadlock. Use it as a context manager (or call close()); a
    discarded stepper closes its process when collected. After an error the process is gone and the stepper closed."""

    WIDTH = 9

    def __init__(self, argv, params):
        self.params = list(params or [])
        self.argv = argv
        self.proc = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self._closed = False
        self._writer = None  # the active call's writer thread

    def __call__(self, batch):
        if self._closed:
            raise RuntimeError("model_step: the stepper is closed")
        if not batch:
            return []
        lines = [" ".join(repr(float(v)) for v in [dt] + list(x) + list(u) + self.params) + "\n" for dt, x, u in batch]
        failed = []

        def write():
            try:
                for line in lines:
                    self.proc.stdin.write(line)
                self.proc.stdin.flush()
            except (BrokenPipeError, OSError, ValueError) as e:  # the child ended: the reader reports why
                failed.append(e)

        writer = self._writer = threading.Thread(target=write, daemon=True)
        writer.start()
        out = []
        try:
            for n in range(len(batch)):
                line = self.proc.stdout.readline()
                if not line:
                    code = self.proc.wait(timeout=5)
                    raise RuntimeError(f"model_step ended (exit {code}) after {n} of {len(batch)} lines: see its stderr")
                values = [float(v) for v in line.split()]
                if len(values) != self.WIDTH:
                    raise RuntimeError(f"model_step answered {len(values)} numbers, not {self.WIDTH}: {line.strip()!r}")
                out.append(values)
        except BaseException:
            self.close()
            raise
        writer.join(timeout=5)
        self._writer = None
        return out

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def close(self, timeout_s=5.0):
        """Ends the process: EOF on its input, then terminate and kill if it does not exit in time. With a call's writer
        still active (an error or an interruption mid-batch) the child is killed and reaped first: the writer may hold
        the input stream blocked on a full pipe, and only the child's death releases it (Task 2 response check R1).
        Idempotent."""
        if self._closed:
            return
        self._closed = True
        writer, self._writer = self._writer, None
        if writer is not None and writer.is_alive():
            self.proc.kill()
            self.proc.wait()
            writer.join(timeout=timeout_s)
        try:
            self.proc.stdin.close()
        except (BrokenPipeError, OSError, ValueError):
            pass
        try:
            self.proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        if self.proc.stdout:
            self.proc.stdout.close()

    def __del__(self):  # a discarded owner: close its process (best effort during interpreter shutdown)
        try:
            self.close(timeout_s=1.0)
        except Exception:  # noqa: BLE001 - nothing to report to at collection time
            pass


def container_step(container=CONTAINER, params=None):
    """The `step` callable running model_step: through docker exec from the host, or directly inside the container
    (container=None). `params` (Task 5): the tracking model's map parameters appended to every line."""
    inner = f"source {WORKSPACE}/ros2_ws/install/setup.bash && exec {shlex.quote(f'{WORKSPACE}/{BINARY}')}"
    if container:
        argv = ["docker", "exec", "-i", container, "bash", "-lc", inner]
    else:
        argv = ["bash", "-lc", inner.replace(WORKSPACE, str(ROOT))]
    return _Stepper(argv, params)


# ---- Task 3: the reports ------------------------------------------------------------------------------------------

ENVELOPE = ROOT / "ros2_ws/src/mobile_manipulator_navigation/config/nav_operating_envelope.yaml"
FINE_S = 0.005                                   # the fits' time grid and delay step
DELAYS = [k * FINE_S for k in range(41)]         # 0 .. 0.2 s
RESOLUTION_S = 0.01                              # what the 20 ms sampling resolves of a delay (stated with each fit)
GROUPS = ("forward", "lateral", "yaw", "arm")
WARMUP_S = 0.5  # outputs left out at the start of a part that does not start from rest (review F2)


def _quantiles(values):
    if not values:
        return None
    v = sorted(values)

    def q(p):
        return v[min(len(v) - 1, int(round(p * (len(v) - 1))))]
    return {"p50": q(0.5), "p90": q(0.9), "max": v[-1]}


def _median(values):
    return _quantiles(values)["p50"] if values else None


def segments(recs, by_goal=True):
    """Index ranges [a, b) of contiguous records: time advancing by at most MAX_GAP_S and, with by_goal, the same goal.
    A goal change does not reset the robot (Session::finish_goal keeps the plant), so the dynamics fits pass
    by_goal=False (review F2)."""
    out, a = [], 0
    for j in range(1, len(recs) + 1):
        if (j == len(recs) or (by_goal and recs[j]["goal"] != recs[j - 1]["goal"])
                or not 0.0 < recs[j]["t"] - recs[j - 1]["t"] <= MAX_GAP_S):
            if j - a >= 2:
                out.append((a, j))
            a = j
    return out


def _from_rest(recs, a):
    """A part starting at a run's first record (the plant starts at rest; pooled runs restart their clock) needs no
    warm-up; one starting after a gap mid-run does (its filter state is unknown)."""
    return a == 0 or recs[a]["t"] <= recs[a - 1]["t"]


def _warm(recs, a, ta):
    """Is the output interval starting at ta past the part's warm-up?"""
    return _from_rest(recs, a) or ta >= recs[a]["t"] + WARMUP_S - 1e-9


def _body(x0, x1, dt):
    """Forward and lateral speed in the body frame at the interval's mid heading, and the yaw rate."""
    yaw = x0[2] + 0.5 * math.remainder(x1[2] - x0[2], 2 * math.pi)
    dx, dy = x1[0] - x0[0], x1[1] - x0[1]
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * dx + s * dy) / dt, (-s * dx + c * dy) / dt, math.remainder(x1[2] - x0[2], 2 * math.pi) / dt


def _fit_first_order(parts, delays, taus):
    """Least squares of y ~ gain * first_order(u(t - delay), tau), over the grids. parts: [(inputs, outputs)], inputs
    [(t_apply, u)] held from t_apply (zero before the first), outputs [(ta, tb, y)] interval means of the response.
    Returns {"delay", "tau", "gain", "rms", "n"} or None when the input never moves."""
    prepared = []
    for inputs, outputs in parts:
        if not outputs:
            continue
        lo = min([ta for ta, _, _ in outputs] + [t for t, _ in inputs]) - max(delays)
        lo = math.floor(lo / FINE_S) * FINE_S
        hi = max(tb for _, tb, _ in outputs)
        m = int(math.ceil((hi - lo) / FINE_S)) + 1
        u, k = [0.0] * m, 0
        for i in range(m):
            t = lo + i * FINE_S
            while k < len(inputs) and inputs[k][0] <= t + 1e-9:
                k += 1
            u[i] = inputs[k - 1][1] if k else 0.0
        idx = [(int(round((ta - lo) / FINE_S)), int(round((tb - lo) / FINE_S)), y) for ta, tb, y in outputs]
        prepared.append((u, idx))
    if not prepared:
        return None
    best = None
    for tau in taus:
        a = math.exp(-FINE_S / tau) if tau > 0 else 0.0
        mean_gain = tau / FINE_S * (1 - a) if tau > 0 else 0.0
        sums = [[0.0, 0.0, 0.0] for _ in delays]  # syy, sym, smm per delay
        for u, idx in prepared:
            z, prefix = 0.0, [0.0]
            for value in u:  # the exact mean of the filter's output over each grid step under the held input
                prefix.append(prefix[-1] + value + (z - value) * mean_gain)
                z = a * z + (1 - a) * value
            for di, d in enumerate(delays):
                shift = int(round(d / FINE_S))
                acc = sums[di]
                for k0, k1, y in idx:
                    s0, s1 = max(0, k0 - shift), max(0, k1 - shift)
                    model = (prefix[s1] - prefix[s0]) / (k1 - k0) if k1 > k0 else 0.0
                    acc[0] += y * y
                    acc[1] += y * model
                    acc[2] += model * model
        for di, (syy, sym, smm) in enumerate(sums):
            if smm <= 1e-18:
                continue
            rss = max(0.0, syy - sym * sym / smm)
            if best is None or rss < best[0] - 1e-15:
                best = (rss, delays[di], tau, sym / smm)
    if best is None:
        return None
    n = sum(len(idx) for _, idx in prepared)
    return {"delay": best[1], "tau": best[2], "gain": best[3], "rms": math.sqrt(best[0] / n), "n": n}


def _fit_refined(parts, delays, tau_max):
    """_fit_first_order over a coarse tau grid (0.02 s), then 0.005 s around the best."""
    coarse = _fit_first_order(parts, delays, [k * 0.02 for k in range(int(round(tau_max / 0.02)) + 1)])
    if coarse is None:
        return None
    t0 = coarse["tau"]
    fine = [t0 + k * FINE_S for k in range(-4, 5) if 0.0 <= t0 + k * FINE_S <= tau_max + 1e-9]
    return _fit_first_order(parts, delays, fine)


def _reanchored(rec):
    return all(abs(rec["b5"][3 + i] - rec["pose"][3 + i]) < 1e-9 for i in range(6))


def alignment(recs, qd_limits=None):
    """Delay, first-order lag and gain per channel, each series on its own clock (plan Task 3): the base's v and
    omega and the arm joints' `pipeline` against cmd (applied from t[j] - 0.02); the arm joints' `servo` against qd_ref
    (sampled at t[j], held from then), leaving out intervals of the ideal mode, next to equal B5 and measured arm
    joints (a reanchor) or at a velocity limit (a possible clip), counted per reason. Responses: finite-difference
    interval means. Parts run through goal changes; after a gap mid-run the first WARMUP_S of responses are left out
    (review F2). A delay resolves to +-RESOLUTION_S; a fit is an effective common model, not a confidence interval."""
    segs = segments(recs, by_goal=False)
    measured, cmd_in, ref_in, masked = [], [], [], []
    excluded_by = {"ideal": 0, "equal_states": 0, "at_limit": 0}
    for a, b in segs:
        out, ci, ri, mk = [], [], [], set()
        for j in range(a + 1, b):
            r0, r1 = recs[j - 1], recs[j]
            ci.append((r0["t"], r1["cmd"]))
            if not _warm(recs, a, r0["t"]):
                continue
            dt = r1["t"] - r0["t"]
            fwd, _, yaw_rate = _body(r0["pose"], r1["pose"], dt)
            joints = [(r1["pose"][3 + i] - r0["pose"][3 + i]) / dt for i in range(6)]
            out.append((r0["t"], r1["t"], [fwd, yaw_rate] + joints))
            # servo: the ideal plant has no arm reference; B5's arm joints equal to the measured ones mark a reanchor
            # (or a coincidence: the realistic reference sits mrad off the servo's joints); a reference at a joint's
            # velocity limit may be clipped (review F5)
            reason = ("ideal" if "ideal" in (r0["mode"], r1["mode"])
                      else "equal_states" if _reanchored(r0) or _reanchored(r1)
                      else "at_limit" if qd_limits is not None and any(abs(r["qd_ref"][i]) >= qd_limits[i] - 1e-6
                                                                      for r in (r0, r1) for i in range(6))
                      else None)
            if reason:
                mk.add(len(out) - 1)
                excluded_by[reason] += 1
        ri = [(recs[j]["t"], recs[j]["qd_ref"]) for j in range(a, b)]
        measured.append(out)
        cmd_in.append(ci)
        ref_in.append(ri)
        masked.append(mk)

    def parts(channel, inputs, value, skip=None):
        return [([(t, value(u)) for t, u in ins], [(ta, tb, y[channel]) for k, (ta, tb, y) in enumerate(outs)
                                                   if skip is None or k not in skip[n]])
                for n, (ins, outs) in enumerate(zip(inputs, measured))]

    result = {"v": _fit_refined(parts(0, cmd_in, lambda u: u[0]), DELAYS, 0.5),
              "omega": _fit_refined(parts(1, cmd_in, lambda u: u[1]), DELAYS, 0.5), "resolution_s": RESOLUTION_S}
    excluded = sum(len(m) for m in masked)
    for i in range(6):
        result[f"q{i}"] = {
            "pipeline": _fit_refined(parts(2 + i, cmd_in, lambda u, i=i: u[2 + i]), DELAYS, 0.5),
            "servo": _fit_refined(parts(2 + i, ref_in, lambda u, i=i: u[i], skip=masked), DELAYS, 0.5),
            "servo_excluded": excluded, "servo_excluded_by": dict(excluded_by), "resolution_s": RESOLUTION_S}
    return result


def horizon_table(recs, step, horizons=(0.02, 0.1, 0.5), inputs=("cmd", "twist")):
    """Per horizon and input: p50, p90 and max of |forward|, |lateral|, |yaw| and each joint's error."""
    table = {}
    for h in horizons:
        wins = windows(recs, h)
        row = {}
        for inp in inputs:
            preds = rollout_many(recs, wins, inp, step) if wins else []
            errs = [error(recs, i0, i1, p) for (i0, i1), p in zip(wins, preds)]
            row[inp] = {"n": len(errs),
                        **{k: _quantiles([abs(e[k]) for e in errs]) for k in ("forward", "lateral", "yaw")},
                        "q": [_quantiles([abs(e["q"][i]) for e in errs]) for i in range(6)]}
        table[f"{h:g}"] = row
    return table


def _bin(value, edges):
    for k in range(len(edges) - 1):
        if edges[k] <= value < edges[k + 1]:
            return k
    return None


def twist_bins(recs, step, v_edges=(0, 0.005, 0.0375, 0.1, 0.3, 1.0), w_edges=(0, 0.02, 0.07, 0.2, 0.6, 2.0)):
    """{"bins": [...], "reversing": n}: per (|v|, |omega|) bin of the 0.1 s windows' net mean requested command: the
    count, the mean signed forward, lateral and yaw error of B5's model, and the command-magnitude-weighted ratio
    measured / commanded (sum over the bin, signed by the command) for v and omega. Windows whose command changes sign
    inside are left out and counted (`reversing`)."""
    wins = windows(recs, 0.1)
    preds = rollout_many(recs, wins, "cmd", step) if wins else []
    bins, reversing = {}, 0
    for (i0, i1), p in zip(wins, preds):
        n = i1 - i0
        signs = [{math.copysign(1.0, recs[j]["cmd"][c]) for j in range(i0 + 1, i1 + 1) if recs[j]["cmd"][c] != 0.0}
                 for c in (0, 1)]
        if any(len(sg) > 1 for sg in signs):  # the command reversed inside the window: its net mean is no level
            reversing += 1
            continue
        v = sum(recs[j]["cmd"][0] for j in range(i0 + 1, i1 + 1)) / n
        w = sum(recs[j]["cmd"][1] for j in range(i0 + 1, i1 + 1)) / n
        key = (_bin(abs(v), v_edges), _bin(abs(w), w_edges))
        if None in key:
            continue
        span = recs[i1]["t"] - recs[i0]["t"]
        x0, x1 = recs[i0]["pose"], recs[i1]["pose"]
        c, s = math.cos(x0[2]), math.sin(x0[2])
        v_meas = (c * (x1[0] - x0[0]) + s * (x1[1] - x0[1])) / span
        w_meas = math.remainder(x1[2] - x0[2], 2 * math.pi) / span
        e = error(recs, i0, i1, p)
        b = bins.setdefault(key, {"count": 0, "forward": 0.0, "lateral": 0.0, "yaw": 0.0, "vm": 0.0, "vc": 0.0,
                                  "wm": 0.0, "wc": 0.0})
        b["count"] += 1
        for k in ("forward", "lateral", "yaw"):
            b[k] += e[k]
        b["vm"] += v_meas * math.copysign(1.0, v) if v else 0.0
        b["vc"] += abs(v)
        b["wm"] += w_meas * math.copysign(1.0, w) if w else 0.0
        b["wc"] += abs(w)
    out = []
    for (kv, kw), b in sorted(bins.items()):
        n = b["count"]
        out.append({"v": [v_edges[kv], v_edges[kv + 1]], "omega": [w_edges[kw], w_edges[kw + 1]], "count": n,
                    "forward": b["forward"] / n, "lateral": b["lateral"] / n, "yaw": b["yaw"] / n,
                    "v_ratio": b["vm"] / b["vc"] if b["vc"] > 1e-12 else None,
                    "omega_ratio": b["wm"] / b["wc"] if b["wc"] > 1e-12 else None})
    return {"bins": out, "reversing": reversing}


def _split(grad, x0, x1):
    """-grad . dx per group (the depth change each degree of freedom carries): forward and lateral resolved in the
    body frame at x0, yaw, the arm joints together."""
    dx = [b - a for a, b in zip(x0, x1)]
    c, s = math.cos(x0[2]), math.sin(x0[2])
    along, across = c * dx[0] + s * dx[1], -s * dx[0] + c * dx[1]
    g_along, g_across = c * grad[0] + s * grad[1], -s * grad[0] + c * grad[1]
    out = {"forward": -g_along * along, "lateral": -g_across * across,
           "yaw": -grad[2] * math.remainder(dx[2], 2 * math.pi), "arm": -sum(grad[3 + i] * dx[3 + i] for i in range(6))}
    out["total"] = sum(out[k] for k in GROUPS)
    return out


def gradient_projection(recs, step=None, depth_min=0.005, kink_max=1e-3):
    """The deepest pair's measured depth change per 0.1 s window, split per degree of freedom (plan Task 3). A window
    from record i0 to i1 uses the diagnostics of records i0+1 and i1+1 (evaluated at the B5 states of i0 and i1).
    Windows with a kink beyond kink_max are feature_switch windows: no split. Windows missing a field are skipped and
    counted per field. Windows whose end has another deepest pair (`pair_changed`) or none with slack (`pair_lost`) are
    censored and counted: the pooled values describe the windows that keep their pair, not every deep window. The
    split is a local projection at the start's B5 state. `step`: B5's model, for the split of what it predicted."""
    skipped = {"depth": 0, "pair": 0, "grad": 0, "kink": 0}
    changed, lost, rows, smooth_wins = 0, 0, [], []
    for i0, i1 in windows(recs, 0.1):
        a, b = i0 + 1, i1 + 1
        if b >= len(recs) or recs[b]["goal"] != recs[i1]["goal"] or not 0.0 < recs[b]["t"] - recs[i1]["t"] <= MAX_GAP_S:
            continue
        ra, rb = recs[a], recs[b]
        missing = [f for f, bad in (("depth", ra["depth"] is None or rb["depth"] is None or ra["depth_stage1"] is None),
                                    ("pair", ra["pair"] is None or rb["pair"] is None), ("grad", ra["grad"] is None),
                                    ("kink", ra["kink"] is None)) if bad]
        if missing:  # every missing field is counted (review F6)
            if ra["depth"] is None or ra["depth"] >= depth_min:
                for f in missing:
                    skipped[f] += 1
            continue
        if ra["depth"] < depth_min:
            continue
        if ra["pair"] != rb["pair"]:  # censored: the window's end has another deepest pair, or none with slack
            if rb["pair"] == "":
                lost += 1
            else:
                changed += 1
            continue
        row = {"t": ra["t"], "pair": ra["pair"], "depth": ra["depth"], "depth_change": rb["depth"] - ra["depth"],
               "stage1_prediction": ra["depth_stage1"] - ra["depth"],
               "feature_switch": max(abs(k) for k in ra["kink"]) > kink_max}
        if not row["feature_switch"]:
            row["b5"] = _split(ra["grad"], recs[i0]["b5"], recs[i1]["b5"])
            row["pose"] = _split(ra["grad"], recs[i0]["pose"], recs[i1]["pose"])
            smooth_wins.append(((i0, i1), row))
        rows.append(row)
    if step is not None and smooth_wins:
        preds = rollout_many(recs, [w for w, _ in smooth_wins], "cmd", step)
        for ((i0, _), row), p in zip(smooth_wins, preds):
            row["model"] = _split(recs[i0 + 1]["grad"], recs[i0]["pose"], p)
    smooth = [r for r in rows if not r["feature_switch"]]
    fs = [r for r in rows if r["feature_switch"]]

    def pooled(key, reduce):
        if not smooth or key not in smooth[0]:
            return None
        return {g: reduce([r[key][g] for r in smooth]) for g in GROUPS + ("total",)}
    # medians per group need not add up to the median total; the sums do (review): use them for explained fractions
    sums = {"depth_change": sum(r["depth_change"] for r in smooth),
            "stage1_prediction": sum(r["stage1_prediction"] for r in smooth),
            **{k: pooled(k, sum) for k in ("b5", "pose", "model")}}
    return {"smooth": {"windows": len(smooth), "depth_change": _median([r["depth_change"] for r in smooth]),
                       "stage1_prediction": _median([r["stage1_prediction"] for r in smooth]),
                       "b5": pooled("b5", _median), "pose": pooled("pose", _median), "model": pooled("model", _median),
                       "sums": sums},
            "feature_switch": {"count": len(fs), "depth_change": _median([r["depth_change"] for r in fs]),
                               "stage1_prediction": _median([r["stage1_prediction"] for r in fs])},
            "skipped": skipped, "pair_changed": changed, "pair_lost": lost, "per_window": rows}


def build_time(recs):
    """The sideslip's build-up: the delay and first-order time constant of the body-lateral velocity against v * omega,
    or None with fewer than 3 transitions of the requested v * omega (changes by more than a quarter of its largest
    magnitude) inside the parts. Two fits (review F1): `executed`, against the measured twist's product (interval
    means): the lateral dynamics alone, the CP1 evidence for a lateral state; `command`, against the requested
    product: the effective response, which also carries the base's own delay and lag. Each with its gain, rms, n."""
    executed, command, transitions = [], [], 0
    for a, b in segments(recs, by_goal=False):
        ex_in, cmd_in, out, series = [], [], [], []
        for j in range(a + 1, b):
            r0, r1 = recs[j - 1], recs[j]
            fwd, lateral, yaw_rate = _body(r0["pose"], r1["pose"], r1["t"] - r0["t"])
            u = r1["cmd"][0] * r1["cmd"][1]
            series.append(u)
            ex_in.append((r0["t"], fwd * yaw_rate))
            cmd_in.append((r0["t"], u))
            if _warm(recs, a, r0["t"]):
                out.append((r0["t"], r1["t"], lateral))
        top = max((abs(u) for u in series), default=0.0)
        if top > 0.0:
            transitions += sum(1 for p, q in zip(series, series[1:]) if abs(q - p) > 0.25 * top)
        executed.append((ex_in, out))
        command.append((cmd_in, out))
    if transitions < 3:
        return None
    fits = {"executed": _fit_refined(executed, DELAYS, 1.0), "command": _fit_refined(command, DELAYS, 1.0)}
    if fits["executed"] is None:
        return None
    for f in fits.values():
        if f is not None:
            f["transitions"] = transitions
    return fits


def envelope_qd_limits(path=ENVELOPE):
    """The arm's joint velocity limits (rad/s) from nav_operating_envelope.yaml (the clip the arm path applies)."""
    for line in Path(path).read_text().splitlines():
        if line.strip().startswith("max_joint_velocity_radps:"):
            return [float(v) for v in line.split("[", 1)[1].split("]", 1)[0].split(",")]
    raise ValueError(f"{path}: no max_joint_velocity_radps")


def analyse(recs, step, qd_limits):
    return {"horizon_table": horizon_table(recs, step), "twist_bins": twist_bins(recs, step),
            "alignment": alignment(recs, qd_limits), "gradient_projection": gradient_projection(recs, step),
            "build_time": build_time(recs)}


def _fmt(v, scale=1.0, digits=2):
    return "-" if v is None else f"{v * scale:.{digits}f}"


def _build_line(bt):
    if not bt:
        return "Pooled: not identifiable (fewer than 3 transitions of v * omega)."

    def one(f):
        return "-" if f is None else f"delay {f['delay']:.3f} s, tau {f['tau']:.3f} s, gain {f['gain']:.4f}, rms {f['rms']:.4f}"
    return (f"Pooled, against the executed twist (the lateral dynamics; CP1's evidence): {one(bt['executed'])}. Against the "
            f"requested twist (the effective response, with the base's delay and lag): {one(bt['command'])}. "
            f"Transitions: {bt['executed']['transitions']}.")


def markdown(runs, pooled):
    lines = ["# Rollout report", "", f"Runs: {len(runs)}. Errors are measured minus B5's model prediction; mm and mrad.",
             "", "## Horizons", "", "| Horizon (s) | Input | n | forward p50/p90/max | lateral p50/p90/max | yaw p50/p90/max |",
             "|---|---|---|---|---|---|"]
    for h, row in pooled["horizon_table"].items():
        for inp, r in row.items():
            cells = ["/".join(_fmt(r[k][q], 1000, 2) for q in ("p50", "p90", "max")) if r[k] else "-"
                     for k in ("forward", "lateral", "yaw")]
            lines.append(f"| {h} | {inp} | {r['n']} | " + " | ".join(cells) + " |")
    lines += ["", "## Twist bins", "", "| abs v (m/s) | abs omega (rad/s) | n | forward | lateral | yaw | v ratio | omega ratio |",
              "|---|---|---|---|---|---|---|---|"]
    lines.insert(len(lines) - 2, f"Windows whose command reversed inside, left out: {pooled['twist_bins']['reversing']}.")
    lines.insert(len(lines) - 2, "")
    for b in pooled["twist_bins"]["bins"]:
        lines.append(f"| {b['v'][0]}-{b['v'][1]} | {b['omega'][0]}-{b['omega'][1]} | {b['count']} | "
                     f"{_fmt(b['forward'], 1000)} | {_fmt(b['lateral'], 1000)} | {_fmt(b['yaw'], 1000)} | "
                     f"{_fmt(b['v_ratio'])} | {_fmt(b['omega_ratio'])} |")
    al = pooled["alignment"]
    lines += ["", "## Alignment", "", f"Delays resolve to +-{al['resolution_s']} s (20 ms sampling).", "",
              "| Channel | Input | delay (s) | tau (s) | gain | rms | n |", "|---|---|---|---|---|---|---|"]

    def fit_row(name, inp, f):
        if f is None:
            return f"| {name} | {inp} | - | - | - | - | - |"
        return f"| {name} | {inp} | {f['delay']:.3f} | {f['tau']:.3f} | {f['gain']:.3f} | {f['rms']:.4f} | {f['n']} |"
    lines.append(fit_row("v", "cmd", al["v"]))
    lines.append(fit_row("omega", "cmd", al["omega"]))
    for i in range(6):
        q = al[f"q{i}"]
        lines.append(fit_row(f"q{i}", "pipeline (cmd)", q["pipeline"]))
        why = ", ".join(f"{k} {v}" for k, v in q["servo_excluded_by"].items() if v) or "none"
        lines.append(fit_row(f"q{i}", f"servo (qd_ref; left out: {why})", q["servo"]))
    g = pooled["gradient_projection"]
    sm = g["smooth"]
    lines += ["", "## Gradient projection", "",
              f"Smooth windows {sm['windows']}, feature-switch windows {g['feature_switch']['count']} (no split); censored: "
              f"pair changed {g['pair_changed']}, pair lost {g['pair_lost']}; skipped {g['skipped']}. The values describe "
              "the windows that keep their pair. Medians per 0.1 s, mm (depth growth positive); the sums row adds up.", "",
              "| Split | forward | lateral | yaw | arm | total |", "|---|---|---|---|---|---|"]
    for key in ("b5", "pose", "model"):
        if sm[key]:
            lines.append(f"| {key} (median) | " + " | ".join(_fmt(sm[key][k], 1000) for k in GROUPS + ("total",)) + " |")
            lines.append(f"| {key} (sum) | " + " | ".join(_fmt(sm["sums"][key][k], 1000) for k in GROUPS + ("total",)) + " |")
    lines += ["", f"Measured depth change {_fmt(sm['depth_change'], 1000)}, the plans' stage-1 prediction "
              f"{_fmt(sm['stage1_prediction'], 1000)}; feature-switch windows: measured "
              f"{_fmt(g['feature_switch']['depth_change'], 1000)}, predicted {_fmt(g['feature_switch']['stage1_prediction'], 1000)}.",
              f" Sums: measured {_fmt(sm['sums']['depth_change'], 1000)}, predicted {_fmt(sm['sums']['stage1_prediction'], 1000)}.",
              "", "## Sideslip build-up", "", _build_line(pooled["build_time"]), "",
              "| Run | build-up vs executed twist (s) | 0.1 s cmd forward p90 (mm) | 0.1 s cmd yaw p90 (mrad) | smooth / switch windows |",
              "|---|---|---|---|---|"]
    for name, r in runs.items():
        h = r["horizon_table"].get("0.1", {}).get("cmd", {})
        gp = r["gradient_projection"]
        bt = r["build_time"]["executed"]["tau"] if r["build_time"] else None
        lines.append(f"| {name} | {_fmt(bt, 1.0, 3)} | "
                     f"{_fmt(h['forward']['p90'] if h.get('forward') else None, 1000)} | "
                     f"{_fmt(h['yaw']['p90'] if h.get('yaw') else None, 1000)} | "
                     f"{gp['smooth']['windows']} / {gp['feature_switch']['count']} |")
    return "\n".join(lines) + "\n"


def run_names(logs):
    """Each log's run key: its directory relative to the logs' common root (review F3: the same seed under two
    variants must not overwrite); ValueError for two logs in one directory."""
    import os
    dirs = [Path(log).resolve().parent for log in logs]
    root = Path(os.path.commonpath([str(d) for d in dirs])) if len(dirs) > 1 else dirs[0].parent
    names = [d.relative_to(root).as_posix() or d.name for d in dirs]
    if len(set(names)) != len(names):
        raise ValueError(f"two logs share a run directory: {sorted(n for n in names if names.count(n) > 1)}")
    return names


def report(logs, out_dir, container=CONTAINER, qd_limits=None):
    import json
    qd_limits = qd_limits or envelope_qd_limits()
    runs, pooled_recs = {}, []
    names = run_names(logs)
    with container_step(container=container) as step:
        for n, log in enumerate(logs):
            _, recs = records(log)
            runs[names[n]] = analyse(recs, step, qd_limits)
            for r in recs:  # pooled: each run's goals are its own segments
                pooled_recs.append(dict(r, goal=(n, r["goal"])))
        pooled = analyse(pooled_recs, step, qd_limits)
    pooled["gradient_projection"].pop("per_window")  # the runs keep theirs; the pool would repeat them
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / "rollout.json").write_text(json.dumps({"runs": runs, "pooled": pooled}, indent=1, default=str))
    (out / "rollout.md").write_text(markdown(runs, pooled))
    return runs, pooled


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("report", help="the rollout reports over --diagnostics logs: DIR/rollout.json, rollout.md")
    r.add_argument("logs", nargs="+")
    r.add_argument("--out", required=True)
    r.add_argument("--container", default=CONTAINER, help="'none': run model_step directly (inside the container)")
    args = parser.parse_args(argv)
    if args.cmd == "report":
        report(args.logs, args.out, container=None if args.container == "none" else args.container)
    return 0


if __name__ == "__main__":
    sys.exit(main())
