#!/usr/bin/env python3
"""What the whole-body MPC (B5) chose for the arm, per run, from summary.json's mpc.arm_trace.

wholebody_mpc_node samples the measured arm joints, the panel's roll, pitch and yaw in
base_footprint (URDF convention, from the model's panel pose) and the tightness at 5 Hz while a goal
is active; tools/run_nav_scenario.py copies the trace into summary.json ("mpc" -> "arm_trace"). For
each run this prints:

  - the poses the arm held: stretches where it was still (largest joint speed over a 1 s window at
    most --still, default 0.05 rad/s) for at least --min-dwell seconds (default 1 s), with stretches
    whose mean joints lie within --radius (max joint difference, default 0.1 rad) merged into one
    pose; per pose its mean joints, the time held, when it was first reached and the named qualified
    pose it matches (qualified_payload.json poses_rad, within the radius). The rest is transit;
  - the side picker's choices (Phase 4B-A: left, right or comfort, with their time spans);
  - the range of each joint and of the panel's roll, pitch and yaw (unwrapped along the run), and
    the tightness range. Vertical carry holds the panel at pitch pi/2, where roll and yaw are not
    defined (gimbal lock): samples within --lock of it are left out of the roll and yaw ranges (and
    counted), and the tilt of the panel normal from vertical, acos(cos(roll) cos(pitch)), is given
    as the well-defined measure (0 flat as in home, pi/2 upright as in vertical carry).

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
LOCK_RAD = 0.05  # |pitch| this close to pi/2: roll and yaw are not defined
WINDOW_S = 1.0   # joint speed window (the measured joints jitter from sample to sample)


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


def still(trace, speed):
    """Per sample: the largest joint speed over a WINDOW_S window centred on it is at most `speed`."""
    half = max(1, round(WINDOW_S / 2 / PERIOD_S))
    flags = []
    for i in range(len(trace)):
        a, b = max(0, i - half), min(len(trace) - 1, i + half)
        # over the whole window even where it is cut short (the run's ends): joint jitter of a
        # couple of samples must not read as motion
        span = max(trace[b]["t"] - trace[a]["t"], WINDOW_S)
        flags.append(distance(trace[b]["q"], trace[a]["q"]) / span <= speed)
    return flags


def held_poses(trace, poses, radius, min_dwell_s, speed):
    """Still stretches of at least min_dwell_s, merged by mean joints within radius, in order of first reach."""
    stretches, current = [], []
    for sample, is_still in zip(trace, still(trace, speed)):
        if is_still:
            current.append(sample)
            continue
        stretches.append(current)
        current = []
    stretches.append(current)
    held = []
    for stretch in stretches:
        if len(stretch) * PERIOD_S + 1e-9 < min_dwell_s:
            continue
        mean = [sum(m["q"][j] for m in stretch) / len(stretch) for j in range(6)]
        pose = next((p for p in held if distance(p["mean"], mean) <= radius), None)
        if pose is None:
            held.append({"mean": mean, "count": len(stretch), "first": stretch[0]["t"]})
        else:
            total = pose["count"] + len(stretch)
            pose["mean"] = [(pose["mean"][j] * pose["count"] + mean[j] * len(stretch)) / total for j in range(6)]
            pose["count"] = total
    out = []
    for pose in held:
        name = next((name for name, q in poses.items() if distance(pose["mean"], q) <= radius), None)
        out.append({"pose": name, "q": [round(v, 3) for v in pose["mean"]], "held_s": round(pose["count"] * PERIOD_S, 1),
                    "first_t": round(pose["first"] - trace[0]["t"], 1)})
    return out


def summarize(trace, poses=None, radius=0.1, min_dwell_s=1.0, speed=0.05, lock_rad=LOCK_RAD):
    """The poses held, joint ranges and panel orientation ranges of one run's arm trace."""
    poses = named_poses() if poses is None else poses
    if not trace:
        return {"samples": 0}
    held = held_poses(trace, poses, radius, min_dwell_s, speed)
    transit = len(trace) - round(sum(pose["held_s"] for pose in held) / PERIOD_S)
    joints = {name: [round(min(s["q"][j] for s in trace), 3), round(max(s["q"][j] for s in trace), 3)]
              for j, name in enumerate(JOINTS)}
    panel = {}
    free = [s for s in trace if abs(s["panel_rpy"][1]) < math.pi / 2 - lock_rad]
    for j, name in enumerate(("roll", "pitch", "yaw")):
        series = unwrap([s["panel_rpy"][j] for s in (trace if name == "pitch" else free)])
        panel[name] = [round(min(series), 3), round(max(series), 3)] if series else None
    tilt = [math.acos(max(-1.0, min(1.0, math.cos(s["panel_rpy"][0]) * math.cos(s["panel_rpy"][1]))))
            for s in trace]
    panel["normal_tilt"] = [round(min(tilt), 3), round(max(tilt), 3)]
    panel["locked_samples"] = len(trace) - len(free)
    tightness = [s["tightness"] for s in trace]
    choices, previous = [], "none"
    for sample in trace:  # the side picker's choice (Phase 4B-A; absent in older traces), as segments
        choice = sample.get("choice", "none")
        if choice != "none" and choice == previous:
            choices[-1]["to_t"] = round(sample["t"], 3)
        elif choice != "none":
            choices.append({"choice": choice, "from_t": round(sample["t"], 3), "to_t": round(sample["t"], 3)})
        previous = choice
    return {"samples": len(trace), "duration_s": round(trace[-1]["t"] - trace[0]["t"], 1),
            "held": held, "transit_s": round(transit * PERIOD_S, 1), "joints_rad": joints, "panel_rpy_rad": panel,
            "tightness": [round(min(tightness), 3), round(max(tightness), 3)], "choices": choices}


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
    panel = summary["panel_rpy_rad"]
    ranges = ", ".join(f"{name} " + (f"[{panel[name][0]}, {panel[name][1]}]" if panel[name] else "-")
                       for name in ("roll", "pitch", "yaw", "normal_tilt"))
    lines.append(f"  panel rad: {ranges} (roll and yaw without the {panel['locked_samples']} samples at "
                 f"pitch +-pi/2); tightness [{summary['tightness'][0]}, {summary['tightness'][1]}]")
    if summary.get("choices"):
        lines.append("  choices: " + ", ".join(f"{c['choice']} {c['from_t']}-{c['to_t']} s" for c in summary["choices"]))
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="+", help="run directories or summary.json files")
    parser.add_argument("--radius", type=float, default=0.1, help="pose cluster radius, rad (max joint difference)")
    parser.add_argument("--min-dwell", type=float, default=1.0, help="seconds a pose must be held to be listed")
    parser.add_argument("--still", type=float, default=0.05,
                        help="largest joint speed (rad/s, over a 1 s window) that counts as holding a pose")
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
        summary = summarize(trace, poses, args.radius, args.min_dwell, args.still)
        results[str(path)] = summary
        if not args.json:
            print(label(run, path))
            print(text(summary))
    if args.json:
        print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
