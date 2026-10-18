#!/usr/bin/env python3
"""Qualify the arm for whole-body (B5) motion in the running Unity/ROS simulation.

Suites (each run is recorded by the Unity arm recorder for the physical checks and judged
with the arm-qualification thresholds; the large per-tick CSV is discarded after its summary
is stored in docs/experiments/wholebody-mpc/arm-qualification.json):
  cpu       reference transition home <-> vertical_carry under --cpu-load-threads N
  combined  the same transitions while driving the compact base sequence, at joint speed scales
  stream    JTC streaming of 1 s sinusoid segments at 20 and 50 Hz, and a stop mid-motion
  poses     held poses beyond the named ones, with and without base motion
  carry     the bent carry pose: held still and while driving, transitions to vertical_carry and home
            at the wholebody_arm limits (still and driving), JTC streaming at 50 Hz around it
  vertical_right  vertical_carry_right (the mirror of vertical_carry, Phase 4B-A): held still and while
            driving, transitions with carry at the wholebody_arm limits (still and driving), JTC streaming
            at 50 Hz around it; the same acceptance as carry
Requires the construction-site scene in Play, arm control active, no Nav2 (/cmd_vel must have
one publisher) and the cleared open test area. Simulation only; never use on hardware.
"""
import argparse
import csv
import gzip
import json
import math
import subprocess
from pathlib import Path

from run_nav_scenario import Runner, last_json

ROOT = Path(__file__).resolve().parents[1]
RECORDINGS = ROOT / "docs/experiments/arm-controller/qualification"
RESULTS = ROOT / "docs/experiments/wholebody-mpc/arm-qualification.json.gz"
HOME = [0.0] * 6
VERTICAL = [math.pi / 2, 0, 0, 0, math.pi / 2, 0]
CARRY = [0.0, 0.968, -1.735, 0.0, 0.767, 0.0]
VERTICAL_RIGHT = [-math.pi / 2, 0, 0, 0, math.pi / 2, 0]  # carry_pose_search --mirror-of vertical_carry
REFERENCE_DURATION = 8  # s, the qualified synchronized cubic
# nav_operating_envelope.yaml wholebody_arm limits, the same on every joint.
LIMIT_VELOCITY, LIMIT_ACCEL = 0.60, 0.61
SAG_ALLOWANCE = 0.01  # rad: the held start can sit this far beyond the nominal pose (elbow 7.5 mrad in carry)
OK_STATES = {"HOLD", "EXTERNAL_CONTROL"}


def physical(csv_path):
    """Watchdog, tilt, clearance and saturation facts of one Unity recording."""
    with open(csv_path, newline="") as stream:
        rows = list(csv.DictReader(stream))
    joints = {}
    for row in rows:
        joints.setdefault(row["joint"], []).append(float(row["estimated_saturated"]))
    first = [r for r in rows if r["joint"] == rows[0]["joint"]]
    return {
        "states": sorted({r["state"] for r in rows}),
        "watchdog_or_fault": any(r["state"] not in OK_STATES for r in rows),
        "max_base_tilt_degrees": max(float(r["base_tilt_degrees"]) for r in rows),
        "min_panel_bottom_m": min(float(r["panel_bottom"]) for r in rows),
        "max_panel_penetration_m": max(float(r["panel_penetration_m"]) for r in rows),
        "max_position_error_rad": max(abs(float(r["position_error"])) for r in rows),
        "max_base_speed_mps": max(abs(float(r["base_velocity"])) for r in rows),
        "max_saturation_fraction": max(sum(v) / len(v) for v in joints.values()),
        "max_wall_gap_between_physics_samples_s": max(
            (float(b["wall_time"]) - float(a["wall_time"]) for a, b in zip(first, first[1:])), default=0.0),
        "max_command_age_s": max(float(r["command_age"]) for r in rows),
    }


def limit_duration(start, goal):
    """--duration of the synchronized cubic whose largest joint move peaks at the wholebody_arm limits.

    The tool starts the cubic at 0.1 s, so the segment is duration - 0.1 s; a cubic over d in T peaks at
    1.5 d / T and 6 d / T^2. The duration is rounded up to 10 ms.
    """
    d = max(abs(b - a) for a, b in zip(start, goal)) + SAG_ALLOWANCE
    segment = max(1.5 * d / LIMIT_VELOCITY, math.sqrt(6 * d / LIMIT_ACCEL))
    return math.ceil((segment + 0.1) * 100) / 100


class Qualifier(Runner):
    def __init__(self, container, prefix):
        super().__init__(container)
        self.prefix = prefix
        self.results = json.loads(gzip.decompress(RESULTS.read_bytes())) if RESULTS.exists() else {}

    def tool(self, *arguments, timeout=240):
        command = ("ros2 run mobile_manipulator_control arm_experiment " + " ".join(str(a) for a in arguments))
        result = self.ros(command, timeout=timeout, check=False)
        try:
            report = last_json(result, "arm_experiment")
        except RuntimeError:
            lines = [line for line in (result.stderr or result.stdout).splitlines() if not line.startswith("[WARN]")]
            report = {"failed": "\n".join(lines)[-400:]}
        return report, result.returncode

    def move(self, pose, duration=REFERENCE_DURATION):
        """Unrecorded positioning at the standstill, as the scenario runner does."""
        report, code = self.tool("--positions", *pose, "--duration", duration, "--hold-seconds", 2)
        if code:
            raise RuntimeError(f"Positioning failed: {report}")

    def place_open(self):
        self.unity("arm_test_place_open")

    def measure(self, suite, name, *arguments, **meta):
        """One recorded tool run; stores and returns the summary."""
        stem = f"{self.prefix}-{name}"
        if stem in self.results.get(suite, {}):
            raise RuntimeError(f"Refusing to overwrite {stem}")
        csv_path = RECORDINGS / (stem + ".csv")
        self.unity("arm_test_record", "--name", stem)
        try:
            report, code = self.tool(*arguments)
        finally:
            self.unity("arm_test_end")
        try:
            facts = physical(csv_path)
        finally:
            csv_path.unlink(missing_ok=True)
        entry = {"arguments": [str(a) for a in arguments], "exit_code": code, **meta, "report": report,
                 "physical": facts}
        self.results.setdefault(suite, {})[stem] = entry
        RESULTS.parent.mkdir(parents=True, exist_ok=True)
        RESULTS.write_bytes(gzip.compress((json.dumps(self.results, separators=(",", ":")) + "\n").encode()))
        path = max(report.get("max_path_error_rad", [float("nan")]))
        print(f"{stem}: exit {code} path {path:.4f} jumps {report.get('position_jumps')} "
              f"watchdog {facts['watchdog_or_fault']} tilt {facts['max_base_tilt_degrees']:.2f} "
              f"bottom {facts['min_panel_bottom_m']:.3f}", flush=True)
        return entry

    def jtc_settings(self):
        out = {}
        for name in ("interpolate_from_desired_state", "allow_nonzero_velocity_at_trajectory_end"):
            text = self.ros(f"ros2 param get /arm_controller {name} 2>/dev/null").stdout
            out[name] = text.strip().split(":")[-1].strip()
        return out

    # --- suites -------------------------------------------------------------------------

    def cpu(self, loads, repeats):
        for load in loads:
            for repeat in range(1, repeats + 1):
                for direction, goal in (("up", VERTICAL), ("down", HOME)):
                    self.measure("cpu", f"cpu{load}-r{repeat}-{direction}", "--positions", *goal,
                                 "--duration", REFERENCE_DURATION, "--hold-seconds", 2,
                                 "--cpu-load-threads", load, load=load, repeat=repeat, direction=direction)

    def combined(self, scales, repeats, schedule):
        for scale in scales:
            duration = round(REFERENCE_DURATION / scale, 3)
            for repeat in range(1, repeats + 1):
                for direction, goal in (("up", VERTICAL), ("down", HOME)):
                    self.place_open()
                    self.measure("combined", f"s{round(scale * 100)}-r{repeat}-{direction}", "--positions", *goal,
                                 "--duration", duration, "--hold-seconds", 2, "--disturbance", schedule,
                                 "--drive-during-motion", scale=scale, repeat=repeat, direction=direction,
                                 duration=duration)

    def stream(self):
        settings = self.jtc_settings()
        self.place_open()
        for name, rate, extra in (("rate20", 20, []), ("rate50", 50, []),
                                  ("stop-mid-motion", 20, ["--stream-stop-after", 7.3])):
            self.measure("stream", name, "--stream-rate", rate, "--stream-duration",
                         12 if extra else 20, *extra, rate=rate, jtc=settings)
            self.move(HOME)

    def poses(self, lowered_shoulder, repeats, only, modes, wrist2_probes):
        pan, wrist_2 = VERTICAL[0], VERTICAL[4]
        poses = {
            "vertical-left": (VERTICAL, [pan + 0.5, 0, 0, 0, wrist_2, 0], "compact"),
            "vertical-right": (VERTICAL, [pan - 0.5, 0, 0, 0, wrist_2, 0], "compact"),
            "tilt-wrist2-plus": (VERTICAL, [pan, 0, 0, 0, wrist_2 + 0.4, 0], "compact"),
            "tilt-wrist2-minus": (VERTICAL, [pan, 0, 0, 0, wrist_2 - 0.4, 0], "compact"),
            "tilt-wrist3-plus": (VERTICAL, [pan, 0, 0, 0, wrist_2, 0.4], "compact"),
            "tilt-wrist3-minus": (VERTICAL, [pan, 0, 0, 0, wrist_2, -0.4], "compact"),
            "lowered": (HOME, [0, lowered_shoulder, 0, 0, 0, 0], "extended"),
        }
        for offset in wrist2_probes:  # refine a failing wrist_2 offset
            poses[f"probe-wrist2-plus-{round(offset * 100)}"] = (VERTICAL, [pan, 0, 0, 0, wrist_2 + offset, 0], "compact")
        for name, (origin, pose, schedule) in poses.items():
            if only and name not in only:
                continue
            for driven in modes:
                for repeat in range(1, repeats + 1):
                    self.move(origin)
                    self.place_open()
                    extra = ["--disturbance", schedule, "--drive-during-motion"] if driven else []
                    self.measure("poses", f"{name}-{'drive' if driven else 'still'}-r{repeat}",
                                 "--positions", *pose, "--duration", REFERENCE_DURATION,
                                 "--hold-seconds", 20, *extra, pose=name, target=pose, driven=driven,
                                 schedule=schedule if driven else "none", repeat=repeat)
                    self.move(origin)

    def carry(self, repeats, parts, schedules):
        """The bent carry pose, judged like Task 2 (path error <= 0.10 rad, no watchdog or tilt flag)."""
        if "hold" in parts:
            for mode in ["still", *schedules]:
                for repeat in range(1, repeats + 1):
                    self.move(HOME)
                    self.place_open()
                    extra = [] if mode == "still" else ["--disturbance", mode, "--drive-during-motion"]
                    self.measure("carry", f"hold-{mode}-r{repeat}", "--positions", *CARRY,
                                 "--duration", REFERENCE_DURATION, "--hold-seconds", 20, *extra,
                                 part="hold", mode=mode, repeat=repeat)
        if "transitions" in parts:
            legs = (("carry-to-vertical", CARRY, VERTICAL), ("vertical-to-carry", VERTICAL, CARRY),
                    ("carry-to-home", CARRY, HOME), ("home-to-carry", HOME, CARRY))
            for mode in ["still", schedules[0]]:
                for repeat in range(1, repeats + 1):
                    self.move(CARRY)
                    for name, start, goal in legs:
                        duration = limit_duration(start, goal)
                        self.place_open()
                        extra = [] if mode == "still" else ["--disturbance", mode, "--drive-during-motion"]
                        self.measure("carry", f"{name}-{mode}-r{repeat}", "--positions", *goal,
                                     "--duration", duration, "--hold-seconds", 2, *extra,
                                     part="transition", leg=name, mode=mode, repeat=repeat, duration=duration)
        if "stream" in parts:
            settings = self.jtc_settings()
            for repeat in range(1, repeats + 1):
                self.move(CARRY)
                self.place_open()
                self.measure("carry", f"stream50-r{repeat}", "--stream-rate", 50, "--stream-duration", 20,
                             part="stream", rate=50, repeat=repeat, jtc=settings)
        self.move(HOME)

    def vertical_right(self, repeats, parts, schedules):
        """vertical_carry_right, judged like carry (spec Phase 4B-A Section 3)."""
        settings = self.jtc_settings() if "stream" in parts else None
        for run in vertical_right_plan(repeats, parts, schedules):
            self.move(HOME)  # unrecorded positioning through home, as the runner does
            if run["start"] != HOME:
                self.move(run["start"])
            self.place_open()
            if run["goal"] is None:
                self.measure("vertical_right", run["name"], *run["extra"], **run["meta"], jtc=settings)
            else:
                self.measure("vertical_right", run["name"], "--positions", *run["goal"], "--duration", run["duration"],
                             "--hold-seconds", run["hold"], *run["extra"], **run["meta"])
        self.move(HOME)


def vertical_right_plan(repeats, parts, schedules):
    """The vertical_right suite's runs: name, start pose (positioned unrecorded), goal, duration, extra tool
    arguments and the stored metadata."""
    runs = []
    if "hold" in parts:
        for mode in ["still", *schedules]:
            for repeat in range(1, repeats + 1):
                extra = [] if mode == "still" else ["--disturbance", mode, "--drive-during-motion"]
                runs.append(dict(name=f"hold-{mode}-r{repeat}", start=HOME, goal=VERTICAL_RIGHT,
                                 duration=REFERENCE_DURATION, hold=20, extra=extra,
                                 meta=dict(part="hold", mode=mode, repeat=repeat)))
    if "transitions" in parts:
        legs = (("carry-to-vertical-right", CARRY, VERTICAL_RIGHT), ("vertical-right-to-carry", VERTICAL_RIGHT, CARRY))
        for mode in ["still", schedules[0]]:
            for repeat in range(1, repeats + 1):
                for name, start, goal in legs:
                    extra = [] if mode == "still" else ["--disturbance", mode, "--drive-during-motion"]
                    duration = limit_duration(start, goal)
                    runs.append(dict(name=f"{name}-{mode}-r{repeat}", start=start, goal=goal, duration=duration,
                                     hold=2, extra=extra, meta=dict(part="transition", leg=name, mode=mode,
                                                                    repeat=repeat, duration=duration)))
    if "stream" in parts:
        for repeat in range(1, repeats + 1):
            runs.append(dict(name=f"stream50-r{repeat}", start=VERTICAL_RIGHT, goal=None, duration=None, hold=None,
                             extra=["--stream-rate", 50, "--stream-duration", 20],
                             meta=dict(part="stream", rate=50, repeat=repeat)))
    return runs


def pose_table(results, prefix, suite):
    """One line per run of a pose batch (carry, vertical_right), judged against the Task 2 acceptance."""
    lines = []
    for stem, entry in results.get(suite, {}).items():
        if not stem.startswith(prefix + "-"):
            continue
        report, facts = entry["report"], entry["physical"]
        path = max(report.get("max_path_error_rad", [float("nan")]))
        hold = max(report.get("hold_max_error", [0.0]))
        velocity = max(report.get("max_reference_velocity_radps", [0.0]))
        ok = (entry["exit_code"] == 0 and path <= 0.10 and not facts["watchdog_or_fault"]
              and facts["max_base_tilt_degrees"] < 3 and facts["min_panel_bottom_m"] >= 0.15
              and facts["max_panel_penetration_m"] <= 0 and hold <= 0.06)
        lines.append(f"{stem[len(prefix) + 1:]:28s} {'ok ' if ok else 'FAIL'} path {path:.4f} hold {hold:.4f} "
                     f"vref {velocity:.3f} jumps {report.get('position_jumps')} "
                     f"watchdog {facts['watchdog_or_fault']} tilt {facts['max_base_tilt_degrees']:.2f} "
                     f"bottom {facts['min_panel_bottom_m']:.3f} speed {facts['max_base_speed_mps']:.3f} "
                     f"sat {facts['max_saturation_fraction']:.2f} gap {facts['max_wall_gap_between_physics_samples_s']:.3f}")
    return "\n".join(lines)


def carry_table(results, prefix):
    return pose_table(results, prefix, "carry")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("suite", choices=["cpu", "combined", "stream", "poses", "carry", "vertical_right"])
    parser.add_argument("--container", default="mm-motion-planning-ma-robot-sim-1")
    parser.add_argument("--prefix", required=True, help="Run name prefix, new for every batch")
    parser.add_argument("--loads", type=int, nargs="+", default=[0, 1, 2, 4, 6])
    parser.add_argument("--scales", type=float, nargs="+", default=[1.0, 0.75, 0.5])
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--schedule", default="compact", choices=["compact", "extended"])
    parser.add_argument("--poses", nargs="+", help="poses suite: only these poses")
    parser.add_argument("--modes", nargs="+", choices=["still", "drive"], default=["still", "drive"])
    parser.add_argument("--wrist2-probes", type=float, nargs="*", default=[],
                        help="poses suite: extra wrist_2 offsets above vertical_carry (select with --poses)")
    parser.add_argument("--parts", nargs="+", choices=["hold", "transitions", "stream"],
                        default=["hold", "transitions", "stream"], help="carry suite: which parts to run")
    parser.add_argument("--schedules", nargs="+", choices=["compact", "extended"], default=["compact", "extended"],
                        help="carry suite: base sequences for the driving holds (the first also for the transitions)")
    parser.add_argument("--table", action="store_true",
                        help="print the carry (or, with the vertical_right suite, its) table of --prefix and exit")
    parser.add_argument("--lowered-shoulder", type=float, default=1.29,
                        help="shoulder_lift of the lowered pose (panel bottom about 0.25 m)")
    args = parser.parse_args()
    if not args.prefix.replace("-", "").isalnum():
        parser.error("Prefix must contain only letters, numbers and hyphens")
    qualifier = Qualifier(args.container, args.prefix)
    if args.table:
        print(pose_table(qualifier.results, args.prefix, "vertical_right" if args.suite == "vertical_right" else "carry"))
        return
    snapshot = qualifier.unity("arm_test_snapshot")
    if not snapshot["playing"] or snapshot["state"] != "HOLD" or not 0 <= snapshot["age"] < .5:
        raise RuntimeError("Requires Play and fresh controlled HOLD: " + str(snapshot))
    qualifier.place_open()
    if args.suite not in ("poses", "carry", "vertical_right"):
        qualifier.move(HOME)
    if args.suite == "cpu":
        qualifier.cpu(args.loads, args.repeats)
    elif args.suite == "combined":
        qualifier.combined(args.scales, args.repeats, args.schedule)
    elif args.suite == "stream":
        qualifier.stream()
    elif args.suite == "carry":
        qualifier.carry(args.repeats, args.parts, args.schedules)
        print(carry_table(qualifier.results, args.prefix))
    elif args.suite == "vertical_right":
        qualifier.vertical_right(args.repeats, args.parts, args.schedules)
        print(pose_table(qualifier.results, args.prefix, "vertical_right"))
    else:
        qualifier.poses(args.lowered_shoulder, args.repeats, args.poses, [m == "drive" for m in args.modes], args.wrist2_probes)


if __name__ == "__main__":
    main()
