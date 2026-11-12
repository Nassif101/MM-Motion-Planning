"""B5's prediction model replayed against the MuJoCo plant's logged motion (E2 plan Tasks 2-3; spec
docs/superpowers/specs/2026-11-11-e2-tracking-prediction-design.md Section 3.1).

A run recorded with `plant_campaign --diagnostics` carries, per State frame, the command B5 requested (`diag_cmd`) and
the deepest gated pair (`diag_pair`, `diag_grad`, `diag_kink`). This tool rolls B5's own model (the `model_step`
binary, the generated function B5 uses) from a frame's pose with the commands that followed and compares with the
measured pose: how far, and along which degree of freedom, the plant departs from what B5 predicts.

Time convention (E2 Task 1 review F2): record j's diagnostics were evaluated and its command sent at t[j] - 0.02, at
record j-1's B5 state; the transition pose[j-1] -> pose[j] ran under cmd[j]. A rollout from i0 to i1 uses
cmd[i0+1 .. i1].

Standard library only (the host's Python 3.9).
"""
import argparse
import math
import shlex
import subprocess
import sys
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
    velocity, the requested command and the gate diagnostics. ValueError for a log recorded without --diagnostics."""
    log = ps.RunLog(log_path)
    out = []
    for kind, header, blocks in log.frames():
        if kind != ps.STATE:
            continue
        if "diag_cmd" not in header:
            raise ValueError(f"{log_path}: recorded without --diagnostics (no diag_cmd)")
        diag = header.get("diag") or [0.0, 0.0, None]
        executed = blocks[2]
        out.append({"t": header["t"], "goal": header.get("goal"), "pose": list(blocks[0]), "b5": list(blocks[1]),
                    "twist": list(executed[:2]), "qd_ref": list(executed[2:8]), "cmd": list(header["diag_cmd"]),
                    "depth": diag[0], "depth_stage1": diag[1], "pair": header.get("diag_pair", ""),
                    "grad": list(header.get("diag_grad") or [0.0] * 9), "kink": list(header.get("diag_kink") or [0.0] * 9)})
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


def control(rec, inputs):
    """The control of the transition into `rec`: the requested command, or the measured twist with the arm
    trajectory's velocity."""
    if inputs == "cmd":
        return list(rec["cmd"])
    if inputs == "twist":
        return list(rec["twist"]) + list(rec["qd_ref"])
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
            batch.append((recs[j]["t"] - recs[j - 1]["t"], states[w], control(recs[j], inputs)))
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
    """A persistent model_step process: each call writes a batch of lines and reads as many back."""

    def __init__(self, argv, params):
        self.params = list(params or [])
        self.proc = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)

    def __call__(self, batch):
        if not batch:
            return []
        lines = []
        for dt, x, u in batch:
            lines.append(" ".join(repr(float(v)) for v in [dt] + list(x) + list(u) + self.params))
        self.proc.stdin.write("\n".join(lines) + "\n")
        self.proc.stdin.flush()
        out = []
        for _ in batch:
            line = self.proc.stdout.readline()
            if not line:
                raise RuntimeError(f"model_step ended (exit {self.proc.poll()}): see its stderr")
            out.append([float(v) for v in line.split()])
        return out

    def close(self):
        if self.proc.poll() is None:
            self.proc.stdin.close()
            self.proc.wait(timeout=10)


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
