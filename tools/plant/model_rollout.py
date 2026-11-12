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
        out.append({"t": header["t"], "goal": header.get("goal"), "pose": list(blocks[0]), "b5": list(blocks[1]),
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
        if any(recs[j]["t"] - recs[j - 1]["t"] > MAX_GAP_S for j in range(i0 + 1, i1 + 1)):
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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_subparsers(dest="cmd", required=True)
    args = parser.parse_args(argv)
    return 0 if args else 1


if __name__ == "__main__":
    sys.exit(main())
