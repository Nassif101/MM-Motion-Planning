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
                    "kink": list(kink) if kink is not None else None,
                    "plan_u0": list(header["diag_plan_u0"]) if "diag_plan_u0" in header else None,
                    "nearest": _nearest(header.get("diag_nearest"))})
    return log.header, out


def _nearest(probe):
    """The Task 4b probe [link, obstacle, distance, in stage 1, in any row, rows dropped] as a dict; None without it and
    for a cycle the probe did not fill (empty link: no solved, gated cycle; Task 4b review F3). Row membership is by
    link and obstacle, not primitive: exact for links with one collision primitive (the wheels)."""
    if probe is None or not probe[0]:
        return None
    return {"link": probe[0], "obstacle": probe[1], "distance": probe[2], "in_stage1": bool(probe[3]),
            "in_rows": bool(probe[4]), "dropped": probe[5]}


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


def _transitions(series, span=25):
    """Changes of a series by more than a quarter of its largest magnitude within `span` samples (0.5 s at 50 Hz),
    each counted once (the rising edges of that condition): a step and a rate-limited ramp count alike."""
    top = max((abs(u) for u in series), default=0.0)
    if top <= 0.0:
        return 0
    count, was = 0, False
    for j in range(1, len(series)):
        now = abs(series[j] - series[max(0, j - span)]) > 0.25 * top
        count += now and not was
        was = now
    return count


def build_time(recs):
    """The sideslip's build-up: the delay and first-order time constant of the body-lateral velocity against v * omega,
    or None with fewer than 3 transitions of the requested v * omega inside the parts (`_transitions`: changes by more
    than a quarter of its largest magnitude within 0.5 s). Two fits (review F1): `executed`, against the measured twist's product (interval
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
        transitions += _transitions(series)
        executed.append((ex_in, out))
        command.append((cmd_in, out))
    if transitions < 3:
        return None
    fits = {"executed": _fit_refined(executed, DELAYS, 1.0), "command": _fit_refined(command, DELAYS, 1.0)}
    if fits["executed"] is None:
        return None
    # the fit's sensitivity (Task 4 review F2): the best rms at fixed time constants (delay and gain refitted) and the
    # rms of no lateral model at all; a zero optimum is a best fit, not a bound
    for name, parts in (("executed", executed), ("command", command)):
        if fits[name] is None:
            continue
        fits[name]["tau_profile"] = {f"{tau:g}": (_fit_first_order(parts, DELAYS, [tau]) or {}).get("rms")
                                     for tau in (0.0, 0.1, 0.2, 0.3)}
        ys = [y for _, outs in parts for _, _, y in outs]
        fits[name]["rms_zero_model"] = math.sqrt(sum(y * y for y in ys) / len(ys)) if ys else None
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


DIAGNOSTIC_KEYS = ("diag", "diag_pair", "diag_grad", "diag_kink", "diag_cmd", "diag_plan_u0", "diag_nearest",
                   "diag_track")


def same_run(log_a, log_b):
    """None when two logs have the same State and Event frames apart from the diagnostics' header keys (a --diagnostics
    rerun against its original, Task 4), else the first difference."""
    def frames(path):
        out = []
        for kind, header, blocks in ps.RunLog(path).frames():
            if kind in (ps.STATE, ps.EVENT):
                out.append((kind, {k: v for k, v in header.items() if k not in DIAGNOSTIC_KEYS}, blocks))
        return out
    fa, fb = frames(log_a), frames(log_b)
    counts = {ps.STATE: 0, ps.EVENT: 0}
    for (ka, ha, ba), (kb, hb, bb) in zip(fa, fb):
        name = "State" if ka == ps.STATE else "Event"
        if ka != kb or ha != hb or ba != bb:
            return f"{name} frame {counts[ka]} differs (t {ha.get('t')})"
        counts[ka] += 1
    if len(fa) != len(fb):
        return f"{len(fa)} against {len(fb)} State and Event frames"
    return None


# The hello config keys a rerun may change (E2 Task 7: the diagnostics and the tracking model entries)
HELLO_RERUN_KEYS = ("diagnostics", "tracking_model", "tracking_model_sha256", "tracking_adaptation")


def same_hello(log_a, log_b, allowed=HELLO_RERUN_KEYS):
    """None when two logs' hello config entries match apart from the `allowed` keys, else what differs (a hello
    without its config is a difference). E2 Task 7 review M2."""
    ca, cb = (ps.RunLog(path).header.get("config") for path in (log_a, log_b))
    if ca is None or cb is None:
        return "a hello without its config"
    differ = sorted(k for k in set(ca) | set(cb) if k not in allowed and ca.get(k) != cb.get(k))
    return f"hello config differs in {', '.join(differ)}" if differ else None


# ---- Task 4 review: the CP1 term shares ----------------------------------------------------------------------------

# The CP1 variants (Task 4 review F1): each a term added to the command B5 sent. turn_gain: [(upper abs omega, gain)]
# from the closed-loop twist bins; sideslip: lateral = k v omega (k open-loop 0.049, closed-loop 0.20); arm delay: the
# arm joints driven by the command of n cycles before; speed_loss: v (1 - c abs omega) (twist bin 0.94 at 0.2-0.6 rad/s);
# deadband: a hard zero below (v, omega) (the bin edges, not a measured plant breakaway).
CLOSED_LOOP_TURN_GAIN = [(0.02, 0.10), (0.07, 0.52), (0.2, 0.85), (0.6, 0.95), (9.0, 0.95)]
CP1_VARIANTS = {
    "unicycle (B5 today)": {},
    "turn gain": {"turn_gain": CLOSED_LOOP_TURN_GAIN},
    "sideslip 0.049": {"sideslip": 0.049},
    "sideslip 0.20": {"sideslip": 0.20},
    "speed loss in turns 0.17": {"speed_loss": 0.17},
    "hard deadband 0.005 m/s, 0.02 rad/s": {"deadband": (0.005, 0.02)},
    "arm delay 2 cycles (0.04 s)": {"arm_delay_cycles": 2},
    "turn gain + sideslip 0.20 + speed loss": {"turn_gain": CLOSED_LOOP_TURN_GAIN, "sideslip": 0.20, "speed_loss": 0.17},
}
SHARE_DEFINITIONS = {
    "model": "explicit-midpoint unicycle with the variant's terms on the sent command, stepped per record (the time "
             "convention); checked against B5's generated model (model_step) to about 1e-8 per state component on the "
             "unicycle (Task 4 review)",
    "windows": "non-overlapping 0.1 s windows (windows(recs, 0.1, stride=5)); `turning`: abs(mean over the window of "
               "v x omega of the sent command) > 0.02",
    "forward, lateral, yaw": "measured minus predicted at the window's end, body frame at its start; rms and p90 of the "
                             "absolute value over the windows",
    "arm_worst_joint": "per window the largest absolute joint error over the six joints; rms and p90 of that over the "
                       "windows (not the worst of the per-joint rms, which `arm_joints` gives)",
    "arm_joints": "per joint the rms over the windows",
    "share": "1 - variant / baseline; an in-sample estimate (the turn gains come from the same runs), not held-out",
}


def _variant_step(params):
    turn = params.get("turn_gain")
    slip, loss = params.get("sideslip", 0.0), params.get("speed_loss", 0.0)
    dead = params.get("deadband")
    delay = params.get("arm_delay_cycles", 0)

    def gain(w):
        for edge, g in turn:
            if abs(w) < edge:
                return g
        return turn[-1][1]

    def run(recs, i0, i1):
        x = list(recs[i0]["pose"])
        for j in range(i0 + 1, i1 + 1):
            dt = recs[j]["t"] - recs[j - 1]["t"]
            v, w = recs[j]["cmd"][0], recs[j]["cmd"][1]
            if dead and abs(v) < dead[0] and abs(w) < dead[1]:
                v = w = 0.0
            w_eff = w * gain(w) if turn else w
            v_eff = v * (1.0 - loss * abs(w))
            lateral = slip * v_eff * w_eff
            mid = x[2] + 0.5 * w_eff * dt
            x[0] += (v_eff * math.cos(mid) - lateral * math.sin(mid)) * dt
            x[1] += (v_eff * math.sin(mid) + lateral * math.cos(mid)) * dt
            x[2] += w_eff * dt
            arm = recs[max(0, j - delay)]["cmd"] if j - delay >= 1 else [0.0] * 8
            for i in range(6):
                x[3 + i] += arm[2 + i] * dt
        return x
    return run


def term_shares(recs_list, variants=None, turning_min=0.02):
    """The CP1 share table (Task 4 review F1): per variant, over all non-overlapping 0.1 s windows and over the turning
    ones, rms and p90 of the forward, lateral and yaw error, of the worst arm joint per window, and per joint rms."""
    variants = variants or CP1_VARIANTS
    subsets = {"all": [], "turning": []}
    for recs in recs_list:
        for i0, i1 in windows(recs, 0.1, stride=5):
            subsets["all"].append((recs, i0, i1))
            mean_vw = sum(recs[j]["cmd"][0] * recs[j]["cmd"][1] for j in range(i0 + 1, i1 + 1)) / (i1 - i0)
            if abs(mean_vw) > turning_min:
                subsets["turning"].append((recs, i0, i1))

    def stats(values):
        if not values:
            return None
        a = sorted(abs(v) for v in values)
        return {"rms": math.sqrt(sum(v * v for v in a) / len(a)), "p90": a[int(0.9 * (len(a) - 1))]}
    out = {"windows": {k: len(v) for k, v in subsets.items()}, "definitions": SHARE_DEFINITIONS,
           "variants": {k: {p: list(v) if isinstance(v, tuple) else v for p, v in params.items()} for k, params in variants.items()}}
    for subset, wins in subsets.items():
        out[subset] = {}
        for name, params in variants.items():
            run = _variant_step(params)
            errs = [error(recs, i0, i1, run(recs, i0, i1)) for recs, i0, i1 in wins]
            out[subset][name] = {k: stats([e[k] for e in errs]) for k in ("forward", "lateral", "yaw")}
            out[subset][name]["arm_worst_joint"] = stats([max(abs(q) for q in e["q"]) for e in errs])
            out[subset][name]["arm_joints"] = [stats([e["q"][i] for e in errs]) for i in range(6)]
    return out


def shares_markdown(sh):
    lines = ["# CP1 term shares", "", f"Windows: all {sh['windows']['all']}, turning {sh['windows']['turning']}. In "
             "brackets: the share of the baseline's rms / p90 the variant removes (negative: it adds error).", "",
             "Definitions:", ""] + [f"- **{k}**: {v}" for k, v in sh["definitions"].items()]
    for subset in ("all", "turning"):
        base = sh[subset].get("unicycle (B5 today)")
        lines += ["", f"## {subset.capitalize()} windows", "",
                  "| Variant | forward rms / p90 (mm) | lateral rms / p90 (mm) | yaw rms / p90 (mrad) | arm worst joint rms / p90 (mrad) |",
                  "|---|---|---|---|---|"]
        for name, r in sh[subset].items():
            cells = []
            for k in ("forward", "lateral", "yaw", "arm_worst_joint"):
                if r[k] is None:
                    cells.append("-")
                    continue
                share = ""
                if base and base[k] and name != "unicycle (B5 today)":
                    share = (f" (removes {100 * (1 - r[k]['rms'] / base[k]['rms']):.0f} % / "
                             f"{100 * (1 - r[k]['p90'] / base[k]['p90']):.0f} %)")
                cells.append(f"{1000 * r[k]['rms']:.3f} / {1000 * r[k]['p90']:.3f}{share}")
            lines.append(f"| {name} | " + " | ".join(cells) + " |")
    return "\n".join(lines) + "\n"


# ---- Task 4: the open-loop excitation and the arm feedforward ---------------------------------------------------------

PAYLOAD = ROOT / "ros2_ws/src/mobile_manipulator_control/config/qualified_payload.json"
PLANT_PROFILE = ROOT / "ros2_ws/src/mobile_manipulator_plant/robots/mobile_manipulator/profile.yaml"
# commission_plant.TURNS (copied: that module needs numpy, this one runs on the host's standard library)
EXCITATION_TURNS = [(0.3, 0.1), (0.3, 0.2), (0.3, 0.3), (0.3, 0.4), (0.15, 0.3), (0.15, 0.4), (0.0, 0.2), (0.0, 0.4),
                    (0.0, 0.6)]
STEP_V = (0.002, 0.005, 0.01, 0.02, 0.0375, 0.1, 0.3)
STEP_W = (0.01, 0.02, 0.03, 0.05, 0.07, 0.1, 0.2, 0.4, 0.6)
ARM_MOVE = [0.3, -0.2, 0.3, 0.3, 0.3, 0.5]  # commission_plant.STEP: the arm's test move from carry


def arm_pose(name="carry"):
    import json
    return json.loads(PAYLOAD.read_text())["poses_rad"][name]


def _replay_input(command, duration_s, q, sample_period_s=0.02):
    """A plant_replay input on the ground alone: `command(t) -> (v, omega)` refreshed every 20 ms, the arm holding q."""
    n = int(round(duration_s / 0.02))
    return {"start": {"base": [0.0, 0.0, 0.0], "q": list(q)}, "duration_s": duration_s, "sample_period_s": sample_period_s,
            "world": "ground", "base_commands": [[round(k * 0.02, 6), *command(k * 0.02)] for k in range(n)],
            "arm_streams": []}


def excitation_inputs(q=None):
    """The open-loop excitation (plan Task 4): steps from rest (after 0.5 s at rest) to each of STEP_V and STEP_W for
    4 s; ramps 0 -> 0.05 m/s and 0 -> 0.1 rad/s over 20 s, and down again from 3 s at that speed; each of
    EXCITATION_TURNS entered from 3 s straight and left into 3 s straight. Each {"name", "input"}."""
    q = q or arm_pose()
    segs = []
    for v in STEP_V:
        segs.append({"name": f"step_v{v:g}", "input": _replay_input(lambda t, v=v: (v if t >= 0.5 else 0.0, 0.0), 4.5, q)})
    for w in STEP_W:
        segs.append({"name": f"step_w{w:g}", "input": _replay_input(lambda t, w=w: (0.0, w if t >= 0.5 else 0.0), 4.5, q)})
    segs.append({"name": "ramp_v_up", "input": _replay_input(lambda t: (min(0.05, 0.05 * t / 20.0), 0.0), 20.0, q)})
    segs.append({"name": "ramp_w_up", "input": _replay_input(lambda t: (0.0, min(0.1, 0.1 * t / 20.0)), 20.0, q)})
    segs.append({"name": "ramp_v_down",
                 "input": _replay_input(lambda t: (0.05 * max(0.0, 1.0 - max(0.0, t - 3.0) / 20.0), 0.0), 23.0, q)})
    segs.append({"name": "ramp_w_down",
                 "input": _replay_input(lambda t: (0.0, 0.1 * max(0.0, 1.0 - max(0.0, t - 3.0) / 20.0)), 23.0, q)})
    for v, w in EXCITATION_TURNS:
        segs.append({"name": f"turn_v{v:g}_w{w:g}",
                     "input": _replay_input(lambda t, v=v, w=w: (v, w if 3.0 <= t < 11.0 else 0.0), 14.0, q)})
    return segs


def excitation_records(seg, out):
    """plant_replay's output as rollout records: the command stamped at or before t[j-1] drove the interval into
    record j (the time convention); B5's state carries the arm reference (q_ref); no gate diagnostics."""
    t = [float(x) for x in out["t"]]
    cmds = seg["input"]["base_commands"]
    recs, k = [], 0
    for j, tj in enumerate(t):
        cmd = [0.0, 0.0]
        if j > 0:
            while k + 1 < len(cmds) and cmds[k + 1][0] <= t[j - 1] + 1e-9:
                k += 1
            if cmds and cmds[k][0] <= t[j - 1] + 1e-9:
                cmd = [cmds[k][1], cmds[k][2]]
        base = [float(x) for x in out["base"][j]]
        recs.append({"t": tj, "goal": 0, "mode": "realistic", "plan_u0": None, "nearest": None, "pose": base + [float(x) for x in out["q"][j]],
                     "b5": base + [float(x) for x in out["q_ref"][j]], "twist": [float(x) for x in out["twist"][j]],
                     "qd_ref": [0.0] * 6, "cmd": cmd + [0.0] * 6, "depth": None, "depth_stage1": None, "pair": None,
                     "grad": None, "kink": None})
    return recs


def _plant_replay(inp, params):
    import calibrate_plant as cp  # numpy and the plant_replay binary: the container
    out = cp.plant_replay(inp, params, profile=PLANT_PROFILE)
    return {k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in out.items()}


def excite(out_dir):
    """Runs the excitation through plant_replay on the realistic profile and writes DIR/excitation.json (per segment
    its commands and the plant's response) and the reports over it (excitation_report.json / .md; segments are runs,
    each from rest). Runs in the container."""
    import json
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    raw, runs, pooled_recs = {}, {}, []
    with container_step(container=None) as step:
        for n, seg in enumerate(excitation_inputs()):
            result = _plant_replay(seg["input"], {})
            raw[seg["name"]] = {"commands": seg["input"]["base_commands"], **{k: result[k] for k in ("t", "base", "twist")}}
            recs = excitation_records(seg, result)
            runs[seg["name"]] = analyse(recs, step, envelope_qd_limits())
            pooled_recs += [dict(r, goal=(n, 0)) for r in recs]
        pooled = analyse(pooled_recs, step, envelope_qd_limits())
    pooled["gradient_projection"].pop("per_window")
    (out / "excitation.json").write_text(json.dumps(raw))
    (out / "excitation_report.json").write_text(json.dumps({"runs": runs, "pooled": pooled}, indent=1, default=str))
    (out / "excitation_report.md").write_text(markdown(runs, pooled))
    return runs, pooled


def arm_tracking(out):
    """Per joint the largest and the RMS |q - q_ref| (mrad) over a plant_replay output."""
    q, ref = out["q"], out["q_ref"]
    err = [[abs(a - b) for a, b in zip(qa, qb)] for qa, qb in zip(q, ref)]
    n = len(err)
    return {"max_mrad": [1000 * max(e[i] for e in err) for i in range(6)],
            "rms_mrad": [1000 * math.sqrt(sum(e[i] ** 2 for e in err) / n) for i in range(6)]}


def arm_feedforward(out_dir):
    """T8: the arm's tracking error with the plant's motion feedforward on and off (plant_replay's motion_ff
    override), on the realistic profile with its command delay: the commissioning move from carry over 1.5 s and over
    0.15 s, the base still and driving (0.3 m/s, 0.3 rad/s). Writes DIR/arm_ff.json and arm_ff.md. Container."""
    import json
    q0 = arm_pose()
    q1 = [a + b for a, b in zip(q0, ARM_MOVE)]
    rows = []
    for move_s in (1.5, 0.15):
        for drive in ((0.0, 0.0), (0.3, 0.3)):
            inp = _replay_input(lambda t, d=drive: d, 0.5 + move_s + 3.0, q0, sample_period_s=0.002)
            inp["arm_streams"] = [{"t": 0.5, "points": [{"t": 0.0, "q": q0, "v": [0.0] * 6},
                                                        {"t": move_s, "q": q1, "v": [0.0] * 6}]}]
            for ff in (True, False):
                result = _plant_replay(inp, {} if ff else {"motion_ff": False})
                rows.append({"move_s": move_s, "drive": list(drive), "motion_ff": ff, **arm_tracking(result)})
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / "arm_ff.json").write_text(json.dumps(rows, indent=1))
    lines = ["# Arm motion feedforward (T8)", "", "| Move (s) | Base (v, omega) | motion_ff | max error per joint (mrad) | "
             "RMS per joint (mrad) |", "|---|---|---|---|---|"]
    for r in rows:
        lines.append(f"| {r['move_s']} | {tuple(r['drive'])} | {'on' if r['motion_ff'] else 'off'} | "
                     + ", ".join(f"{x:.1f}" for x in r["max_mrad"]) + " | " + ", ".join(f"{x:.1f}" for x in r["rms_mrad"]) + " |")
    (out / "arm_ff.md").write_text("\n".join(lines) + "\n")
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("report", help="the rollout reports over --diagnostics logs: DIR/rollout.json, rollout.md")
    r.add_argument("logs", nargs="+")
    r.add_argument("--out", required=True)
    r.add_argument("--container", default=CONTAINER, help="'none': run model_step directly (inside the container)")
    e = sub.add_parser("excite", help="the open-loop excitation on the realistic plant and its reports (container)")
    e.add_argument("--out", required=True)
    a = sub.add_parser("arm-ff", help="the arm's tracking with the motion feedforward on and off (container)")
    a.add_argument("--out", required=True)
    sh = sub.add_parser("shares", help="the CP1 term shares over logs: DIR/shares.json, shares.md")
    sh.add_argument("logs", nargs="+")
    sh.add_argument("--out", required=True)
    c = sub.add_parser("same-run", help="exit 0 when two logs match frame by frame, the diagnostics aside")
    c.add_argument("a")
    c.add_argument("b")
    h = sub.add_parser("same-hello", help="exit 0 when two logs' hello configs match but the rerun keys")
    h.add_argument("a")
    h.add_argument("b")
    args = parser.parse_args(argv)
    if args.cmd == "report":
        report(args.logs, args.out, container=None if args.container == "none" else args.container)
    elif args.cmd == "excite":
        excite(args.out)
    elif args.cmd == "arm-ff":
        arm_feedforward(args.out)
    elif args.cmd == "shares":
        import json
        result = term_shares([records(log)[1] for log in args.logs])
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        (out / "shares.json").write_text(json.dumps(result, indent=1))
        (out / "shares.md").write_text(shares_markdown(result))
    elif args.cmd == "same-run":
        diff = same_run(args.a, args.b)
        print(diff or "identical (diagnostics aside)")
        return 1 if diff else 0
    elif args.cmd == "same-hello":
        diff = same_hello(args.a, args.b)
        print(diff or "hello config identical (rerun keys aside)")
        return 1 if diff else 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
