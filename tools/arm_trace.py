#!/usr/bin/env python3
"""What the whole-body MPC (B5) chose for the arm, per run, from summary.json's mpc.arm_trace.

wholebody_mpc_node samples the measured arm joints, the panel's roll, pitch and yaw in
base_footprint (URDF convention, from the model's panel pose) and the tightness at 5 Hz while a goal
is active; tools/run_nav_scenario.py copies the trace into summary.json ("mpc" -> "arm_trace"). For
each run this prints:

  - the poses the arm held: samples clustered at --radius (max joint difference, default 0.1 rad)
    around the first sample of each cluster, kept when held at least --min-dwell seconds (default
    1 s; the rest is transit), with the cluster's mean joints, the time held and the named
    qualified pose it matches (qualified_payload.json poses_rad, within the radius);
  - the range of each joint and of the panel's roll, pitch and yaw (unwrapped along the run), and
    the tightness range.

  python3 tools/arm_trace.py experiment_runs/<run> [...]     # run directories or summary files
  python3 tools/arm_trace.py --json experiment_runs/<run>    # the same as JSON
"""
import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
QUALIFIED_PAYLOAD = ROOT / "ros2_ws/src/mobile_manipulator_control/config/qualified_payload.json"
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")
PERIOD_S = 0.2  # the node's arm_trace period


def named_poses(path=QUALIFIED_PAYLOAD):
    return {name: list(values) for name, values in json.loads(Path(path).read_text())["poses_rad"].items()}


def unwrap(values):
    """Angles made continuous along the run (no +-pi jumps)."""
    out = []
    for value in values:
        if out:
            value = out[-1] + math.remainder(value - out[-1], 2 * math.pi)
        out.append(value)
    return out


def distance(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def clusters(trace, radius):
    """Leader clustering in the joint space: each sample joins the first cluster whose leader is within radius."""
    found = []
    for sample in trace:
        for cluster in found:
            if distance(sample["q"], cluster["leader"]) <= radius:
                cluster["members"].append(sample)
                break
        else:
            found.append({"leader": sample["q"], "members": [sample]})
    return found


def summarize(trace, poses=None, radius=0.1, min_dwell_s=1.0):
    """The poses held, joint ranges and panel orientation ranges of one run's arm trace."""
    poses = named_poses() if poses is None else poses
    if not trace:
        return {"samples": 0}
    held, transit = [], 0
    for cluster in clusters(trace, radius):
        members = cluster["members"]
        dwell = len(members) * PERIOD_S
        if dwell + 1e-9 < min_dwell_s:
            transit += len(members)
            continue
        mean = [sum(m["q"][j] for m in members) / len(members) for j in range(6)]
        name = next((name for name, q in poses.items() if distance(mean, q) <= radius), None)
        held.append({"pose": name, "q": [round(v, 3) for v in mean], "held_s": round(dwell, 1),
                     "first_t": round(members[0]["t"] - trace[0]["t"], 1)})
    held.sort(key=lambda pose: pose["first_t"])
    joints = {name: [round(min(s["q"][j] for s in trace), 3), round(max(s["q"][j] for s in trace), 3)]
              for j, name in enumerate(JOINTS)}
    panel = {}
    for j, name in enumerate(("roll", "pitch", "yaw")):
        series = unwrap([s["panel_rpy"][j] for s in trace])
        panel[name] = [round(min(series), 3), round(max(series), 3)]
    tightness = [s["tightness"] for s in trace]
    return {"samples": len(trace), "duration_s": round(trace[-1]["t"] - trace[0]["t"], 1),
            "held": held, "transit_s": round(transit * PERIOD_S, 1), "joints_rad": joints, "panel_rpy_rad": panel,
            "tightness": [round(min(tightness), 3), round(max(tightness), 3)]}


def load(path):
    path = Path(path)
    return json.loads((path / "summary.json" if path.is_dir() else path).read_text())


def label(run, path):
    options = (run.get("mpc") or {}).get("options") or {}
    parts = [f"{key} {value}" for key, value in options.items()]
    if run.get("slowdown_zone") is False:
        parts.append("no slowdown zone")
    return f"{Path(path).name}" + (f" ({', '.join(parts)})" if parts else "")


def text(summary):
    if not summary["samples"]:
        return "  no arm trace"
    lines = [f"  {summary['samples']} samples over {summary['duration_s']} s"]
    for pose in summary["held"]:
        lines.append(f"  held {pose['pose'] or '(unnamed)':16} q {pose['q']} for {pose['held_s']} s "
                     f"from t={pose['first_t']} s")
    lines.append(f"  transit {summary['transit_s']} s")
    lines.append("  joints rad: " + ", ".join(f"{name} [{lo}, {hi}]" for name, (lo, hi) in summary["joints_rad"].items()))
    lines.append("  panel rad: " + ", ".join(f"{name} [{lo}, {hi}]"
                                             for name, (lo, hi) in summary["panel_rpy_rad"].items())
                 + f"; tightness [{summary['tightness'][0]}, {summary['tightness'][1]}]")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="+", help="run directories or summary.json files")
    parser.add_argument("--radius", type=float, default=0.1, help="pose cluster radius, rad (max joint difference)")
    parser.add_argument("--min-dwell", type=float, default=1.0, help="seconds a pose must be held to be listed")
    parser.add_argument("--json", action="store_true", help="print JSON instead of text")
    args = parser.parse_args()
    poses = named_poses()
    results = {}
    for path in args.runs:
        run = load(path)
        trace = (run.get("mpc") or {}).get("arm_trace")
        if trace is None:
            print(f"{path}: no mpc.arm_trace (not a B5 run, or recorded before the arm trace)", file=sys.stderr)
            continue
        summary = summarize(trace, poses, args.radius, args.min_dwell)
        results[str(path)] = summary
        if not args.json:
            print(label(run, path))
            print(text(summary))
    if args.json:
        print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
