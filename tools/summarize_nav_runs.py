#!/usr/bin/env python3
"""Summarise navigate_to_pose and mission scenario runs per scenario and controller as Markdown.

Missions (B3) get a second table: mission success and times, and over all reconfiguration
steps the planning time, motion duration, planned clearance to the (inflated) known
obstacles, JTC path and hold error, plus Unity's panel ground clearance and base tilt.

Rows are grouped by scenario, controller and footprint strategy (Phase 3, ADR 0010):
[dyn] / [dyndisc] for the dynamic footprint (mesh or disc model), [fp:<profile>] for a static
launch footprint other than the scenario's, nothing for the default. "Failures" counts the
failure modes; a failed drive with robot-environment contact counts as "collision".
"Fp updates" counts dynamic footprint publishes during drives, "Fp us" its compute time
(mean/max), "Hull refusals" HULL_IN_COLLISION reconfiguration steps.

B5 (whole-body MPC) runs add four columns to the drive table when any run has them: "MPC p99 ms"
(99th percentile of the node's whole cycle), "misses" (cycles over the period), "near-misses"
(cycles over 80 % of the period; both summed over runs) and "3D clearance m" (minimum distance
between the robot primitives and the known boxes, clearance_3d). A B5 mission scenario is driven
as one navigate task, so it is in the drive table; a [ref] run used the scripted panel references.

Reads run summaries written by tools/run_nav_scenario.py: run directories
(experiment_runs/<run>/summary.json) or copied summary files (*-summary.json). Cells
show the median over runs and, for several runs, the range in parentheses. Time, path
length, and final position error use successful runs only; every other column uses all
runs. A drive Nav2 reports reached counts as a success only if the base ended within the
goal checker's tolerances plus 0.02 (navigate_run.hpp drive_status): the checker is stateful,
so once inside it only the heading is checked while the robot may keep moving (MPPI ended up
to 3.8 m away). Summaries written before drive_status existed are rescored the same way.

  python3 tools/summarize_nav_runs.py docs/experiments/nav2-navigation/runs/*-summary.json
"""
import argparse
import json
import re
import statistics
import sys
from collections import defaultdict
from pathlib import Path

NAV2_NAVIGATION = (Path(__file__).resolve().parents[1] / "ros2_ws/src/mobile_manipulator_navigation/config"
                   / "nav2_navigation.yaml")
SETTLE_MARGIN = 0.02  # navigate_run.hpp kGoalSettleMargin


def goal_tolerance():
    text = NAV2_NAVIGATION.read_text()
    return tuple(float(re.search(rf"^\s*{key}:\s*([0-9.]+)", text, re.M).group(1))
                 for key in ("xy_goal_tolerance", "yaw_goal_tolerance"))


def rescore(drive, tolerance):
    """Apply drive_status to a drive recorded before it existed (no nav2_status)."""
    if "nav2_status" in drive or drive.get("status") != "succeeded":
        return False
    position, yaw = drive.get("final_position_error_m"), drive.get("final_yaw_error_rad")
    if (position is not None and yaw is not None and position <= tolerance[0] + SETTLE_MARGIN
            and yaw <= tolerance[1] + SETTLE_MARGIN):
        return False
    drive["nav2_status"], drive["status"] = "succeeded", "off_goal"
    return True

def footprint_label(run):
    footprint = run.get("footprint") or {}
    if footprint.get("mode") == "dynamic":
        return " [dyn]" if footprint.get("model") in (None, "mesh") else f" [dyn{footprint['model']}]"
    profile = footprint.get("nav2_profile")
    return f" [fp:{profile}]" if profile and profile != run["scenario"].get("footprint_profile") else ""


def contact(run):
    return bool(run.get("contacts") and run["contacts"].get("contact"))


def won(run):
    """A run succeeds only without robot-environment contact (spec: a contact is a collision)."""
    return run["task"].get("status") == "succeeded" and not contact(run)


def criteria_4bb(run, rate_hz):
    """Phase 4B-B per-run criteria (spec Section 7): True passes, False fails, None not measured.

    Tracking comes from Unity's arm recording (missions only); sign flips from the bag analysis
    (`arm_sign_flips`), when the run has them.
    """
    mpc = run.get("mpc") or {}
    clearances = [c for c in (run.get("clearance_3d_m"), run.get("clearance_self_m")) if c is not None]
    p99 = (mpc.get("total_s") or {}).get("p99")
    tracking = (run.get("arm_physical") or {}).get("max_path_error_rad")
    flips = run.get("arm_sign_flips")
    return {
        "contact": not contact(run),
        "success": run["task"].get("status") == "succeeded",
        "clearance": min(clearances) >= 0.045 if clearances else None,
        "p99": p99 < 1.0 / rate_hz if p99 is not None else None,
        "misses": mpc["deadline_misses"] < 0.01 * mpc["cycles"] if mpc.get("cycles") else None,
        "tracking": tracking <= 0.10 if tracking is not None else None,
        "sign_flips": flips == 0 if flips is not None else None,
    }


def failures(runs):
    """Failure modes over runs: collision for any run with contact, else the task status."""
    modes = defaultdict(int)
    for run in runs:
        if not won(run):
            modes["collision" if contact(run) else run["task"].get("status")] += 1
    return ", ".join(f"{mode} {count}" for mode, count in sorted(modes.items())) or "-"


def drive_windows(run):
    task = run["task"]
    drives = [s for s in task["steps"] if s["type"] == "navigate"] if "steps" in task else [task]
    return [(d["started_s"], d["finished_s"]) for d in drives if "started_s" in d and "finished_s" in d]


def footprint_updates(runs):
    """Dynamic footprint publishes during drives, summed over runs ('-' without the stats)."""
    counted = [sum(any(a <= t <= b for a, b in drive_windows(run)) for t in run["footprint_stats"]["publish_times_s"])
               for run in runs if run.get("footprint_stats") and drive_windows(run)]
    return str(sum(counted)) if counted else "-"


def footprint_compute(runs):
    stats = [run["footprint_stats"].get("compute_us") for run in runs if run.get("footprint_stats")]
    stats = [s for s in stats if s]
    if not stats:
        return "-"
    return f"{statistics.mean(s['mean'] for s in stats):.0f}/{max(s['max'] for s in stats):.0f}"


COLUMNS = ("Scenario", "Controller", "Success", "Failures", "Contact", "Time s", "Path m",
           "Final error m", "Cross-track p95 m", "Min clearance m", "Obstacle clearance m",
           "Mover clearance m", "Mover waited s", "Recoveries", "Monitor stop/slow/appr", "Controller CPU %", "Loop misses",
           "Controller errors", "Fp updates", "Fp us")
WHOLEBODY_COLUMNS = ("MPC p99 ms", "misses", "near-misses", "3D clearance m")


def has_wholebody(runs):
    """Whether any run carries B5 statistics (the MPC timing summary or the 3D clearance)."""
    return any(run.get("mpc") or run.get("clearance_3d_m") is not None for run in runs)


def columns(wide):
    return COLUMNS + WHOLEBODY_COLUMNS if wide else COLUMNS


def mpc_p99_ms(run):
    p99 = ((run.get("mpc") or {}).get("total_s") or {}).get("p99")
    return None if p99 is None else p99 * 1e3


def mpc_total(runs, key):
    """Sum of an MPC count over the runs that have the stats, '-' without any."""
    counts = [run["mpc"][key] for run in runs if run.get("mpc") and key in run["mpc"]]
    return str(sum(counts)) if counts else "-"


def wholebody_cells(runs):
    return (spread([mpc_p99_ms(run) for run in runs], 1), mpc_total(runs, "deadline_misses"),
            mpc_total(runs, "near_misses"), spread([run.get("clearance_3d_m") for run in runs], 3))


def controller_label(run):
    """Controller as grouped: the baseline's name, + global-obstacle mode, footprint strategy, [ref]."""
    # Runs before 2026-10-01 stored true for live global obstacles.
    mode = {True: "live", False: "static", None: "static"}.get(run.get("global_obstacles"), run.get("global_obstacles"))
    label = (run.get("controller") or "rpp") + {"live": " +global obstacles",
                                                "persistent": " +persistent global"}.get(mode, "")
    return label + footprint_label(run) + (" [ref]" if run.get("references") == "scripted" else "")


def is_mission_table(run):
    """A mission with reconfigure steps (B3, B4); a B5 mission scenario is one navigate task."""
    return run["scenario"]["task"] == "mission" and "steps" in run["task"]


def load(paths):
    runs, tolerance = [], goal_tolerance()
    for path in map(Path, paths):
        path = path / "summary.json" if path.is_dir() else path
        if not path.is_file():
            continue
        summary = json.loads(path.read_text())
        if summary.get("host_woke_during_run"):
            print(f"skipping {path}: the host slept during the run", file=sys.stderr)
            continue
        if summary.get("container_oom_kills_during_run"):
            print(f"skipping {path}: the container killed a process for memory during the run", file=sys.stderr)
            continue
        if summary["scenario"].get("task") in ("navigate_to_pose", "mission") and "status" in summary["task"]:
            task = summary["task"]
            drives = [step for step in task["steps"] if step["type"] == "navigate"] if "steps" in task else [task]
            if any([rescore(drive, tolerance) for drive in drives]):
                print(f"rescored {path}: a drive Nav2 reported reached ended off the goal", file=sys.stderr)
                if "steps" in task:
                    task["status"] = "failed"
            runs.append(summary)
    return runs


MISSION_COLUMNS = ("Scenario", "Controller", "Success", "Failures", "Contact", "Total s", "Drives s", "Reconfig. s",
                   "Planning s", "Re-planned", "Motion s", "Planned clearance m", "Path error rad", "Hold error rad",
                   "move_group CPU %", "move_group MB",
                   "Panel bottom m", "Base tilt deg", "Arm checks", "Hull refusals", "Fp updates", "Fp us")


def mission_row(scenario, controller, runs):
    tasks = [run["task"] for run in runs]
    successes = [run["task"] for run in runs if won(run)]
    steps = [step for task in tasks for step in task["steps"]
             if step["type"] == "reconfigure" and step["result"].get("error_code") == "SUCCESS"]
    results = [step["result"] for step in steps]
    physical = [run.get("arm_physical") or {} for run in runs]
    refusals = sum(step["type"] == "reconfigure" and step["result"].get("error_code") == "HULL_IN_COLLISION"
                   for task in tasks for step in task["steps"])
    return (scenario, controller, f"{len(successes)}/{len(runs)}", failures(runs),
            str(sum(contact(run) for run in runs)),
            spread([t["total_time_s"] for t in successes], 1),
            spread([t["drive_time_s"] for t in successes], 1),
            spread([t["reconfigure_time_s"] for t in successes], 1),
            spread([r.get("planning_time_s") for r in results], 2),
            f"{sum((r.get('planning_requests') or 1) > 1 for r in results)}/{len(results)}",
            spread([r.get("trajectory_duration_s") for r in results], 2),
            spread([r.get("min_planned_clearance_m") for r in results], 3),
            spread([r.get("max_path_error_rad") for r in results], 3),
            spread([r.get("hold_error_rad") for r in results], 3),
            spread([(s.get("cpu_percent_of_core") or {}).get("move_group") for s in steps], 1),
            spread([(s.get("max_rss_mb") or {}).get("move_group") for s in steps], 0),
            spread([p.get("min_panel_bottom_m") for p in physical], 3),
            spread([p.get("max_base_tilt_degrees") for p in physical], 2),
            f"{sum(bool(p.get('passed')) for p in physical)}/{len(runs)}",
            str(refusals), footprint_updates(runs), footprint_compute(runs))


def spread(values, digits):
    values = [v for v in values if v is not None]
    if not values:
        return "-"
    middle = f"{statistics.median(values):.{digits}f}"
    if len(values) == 1 or min(values) == max(values):
        return middle
    return f"{middle} ({min(values):.{digits}f}-{max(values):.{digits}f})"


def cpu_of_controller(task):
    """CPU % of a core of the process that runs the local controller (Nav2's server, or B5's MPC node)."""
    cpu = task["cpu_percent_of_core"]
    return cpu.get("controller_server", cpu.get("wholebody_mpc_node"))


def row(scenario, controller, runs, wide=None):
    tasks = [run["task"] for run in runs]
    successes = [run["task"] for run in runs if won(run)]
    actions = [a["action"] for task in tasks for a in task["collision_monitor_activations"]]
    logs = [run.get("controller_log") or {} for run in runs]
    wide = has_wholebody(runs) if wide is None else wide
    return (scenario, controller,
            f"{len(successes)}/{len(runs)}", failures(runs),
            str(sum(contact(run) for run in runs)),
            spread([t["time_s"] for t in successes], 1),
            spread([t["path_length_m"] for t in successes], 2),
            spread([t["final_position_error_m"] for t in successes], 3),
            spread([(t["cross_track_m"] or {}).get("p95") for t in tasks], 3),
            spread([t["min_footprint_clearance_to_static_map_m"] for t in tasks], 2),
            spread([t.get("min_footprint_clearance_to_obstacles_m") for t in tasks], 2),
            spread([t.get("min_footprint_clearance_to_movers_m") for t in tasks], 2),
            spread([sum(m["blockedSeconds"] for m in run["movers"]) if run.get("movers") else None
                    for run in runs], 1),
            spread([t["recoveries"] for t in tasks], 0),
            "/".join(str(actions.count(a)) for a in ("stop", "slowdown", "approach")),
            spread([cpu_of_controller(t) for t in tasks], 1),
            str(sum(log.get("loop_rate_misses", 0) for log in logs)),
            str(sum(log.get("errors", 0) for log in logs)),
            footprint_updates(runs), footprint_compute(runs)) + (wholebody_cells(runs) if wide else ())


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("paths", nargs="+", help="run directories or summary JSON files")
    args = parser.parse_args()

    groups, missions = defaultdict(list), defaultdict(list)
    for run in load(args.paths):
        (missions if is_mission_table(run) else groups)[(run["scenario"]["name"], controller_label(run))].append(run)
    order = ("rpp", "dwb", "mppi", "wholebody")  # bring-up first, then B1, B2 and B5

    def ordered(table):
        return sorted(table, key=lambda k: (k[0], "+" in k[1],
                                            order.index(k[1].split()[0]) if k[1].split()[0] in order else 9))

    if groups:
        wide = has_wholebody([run for runs in groups.values() for run in runs])
        print("| " + " | ".join(columns(wide)) + " |")
        print("|" + "---|" * len(columns(wide)))
        for key in ordered(groups):
            print("| " + " | ".join(row(*key, groups[key], wide=wide)) + " |")
    if missions:
        if groups:
            print()
        print("| " + " | ".join(MISSION_COLUMNS) + " |")
        print("|" + "---|" * len(MISSION_COLUMNS))
        for key in ordered(missions):
            print("| " + " | ".join(mission_row(*key, missions[key])) + " |")


if __name__ == "__main__":
    main()
