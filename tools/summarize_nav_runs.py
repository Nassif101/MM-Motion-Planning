#!/usr/bin/env python3
"""Summarise navigate_to_pose and mission scenario runs per scenario and controller as Markdown.

Missions (B3) get a second table: mission success and times, and over all reconfiguration
steps the planning time, motion duration, planned clearance to the (inflated) known
obstacles, JTC path and hold error, plus Unity's panel ground clearance and base tilt.

Reads run summaries written by tools/run_nav_scenario.py: run directories
(experiment_runs/<run>/summary.json) or copied summary files (*-summary.json). Cells
show the median over runs and, for several runs, the range in parentheses. Time, path
length, and final position error use successful runs only; every other column uses all
runs. A success can end outside the goal checker's xy tolerance: the checker is stateful,
so once inside it only the heading is checked while the robot may keep moving.

  python3 tools/summarize_nav_runs.py docs/experiments/nav2-navigation/runs/*-summary.json
"""
import argparse
import json
import statistics
from collections import defaultdict
from pathlib import Path

COLUMNS = ("Scenario", "Controller", "Success", "Contact", "Time s", "Path m",
           "Final error m", "Cross-track p95 m", "Min clearance m", "Obstacle clearance m",
           "Mover clearance m", "Mover waited s", "Recoveries", "Monitor stop/slow/appr", "Controller CPU %", "Loop misses",
           "Controller errors")


def load(paths):
    runs = []
    for path in map(Path, paths):
        path = path / "summary.json" if path.is_dir() else path
        if not path.is_file():
            continue
        summary = json.loads(path.read_text())
        if summary["scenario"].get("task") in ("navigate_to_pose", "mission") and "status" in summary["task"]:
            runs.append(summary)
    return runs


MISSION_COLUMNS = ("Scenario", "Controller", "Success", "Contact", "Total s", "Drives s", "Reconfig. s",
                   "Planning s", "Motion s", "Planned clearance m", "Path error rad", "Hold error rad",
                   "Panel bottom m", "Base tilt deg", "Arm checks")


def mission_row(scenario, controller, runs):
    tasks = [run["task"] for run in runs]
    won = [task for task in tasks if task["status"] == "succeeded"]
    results = [step["result"] for task in tasks for step in task["steps"]
               if step["type"] == "reconfigure" and step["result"].get("error_code") == "SUCCESS"]
    physical = [run.get("arm_physical") or {} for run in runs]
    return (scenario, controller, f"{len(won)}/{len(runs)}",
            str(sum(bool(run["contacts"] and run["contacts"]["contact"]) for run in runs)),
            spread([t["total_time_s"] for t in won], 1),
            spread([t["drive_time_s"] for t in won], 1),
            spread([t["reconfigure_time_s"] for t in won], 1),
            spread([r.get("planning_time_s") for r in results], 2),
            spread([r.get("trajectory_duration_s") for r in results], 2),
            spread([r.get("min_planned_clearance_m") for r in results], 3),
            spread([r.get("max_path_error_rad") for r in results], 3),
            spread([r.get("hold_error_rad") for r in results], 3),
            spread([p.get("min_panel_bottom_m") for p in physical], 3),
            spread([p.get("max_base_tilt_degrees") for p in physical], 2),
            f"{sum(bool(p.get('passed')) for p in physical)}/{len(runs)}")


def spread(values, digits):
    values = [v for v in values if v is not None]
    if not values:
        return "-"
    middle = f"{statistics.median(values):.{digits}f}"
    if len(values) == 1 or min(values) == max(values):
        return middle
    return f"{middle} ({min(values):.{digits}f}-{max(values):.{digits}f})"


def row(scenario, controller, runs):
    tasks = [run["task"] for run in runs]
    won = [task for task in tasks if task["status"] == "succeeded"]
    actions = [a["action"] for task in tasks for a in task["collision_monitor_activations"]]
    logs = [run.get("controller_log") or {} for run in runs]
    return (scenario, controller,
            f"{len(won)}/{len(runs)}",
            str(sum(bool(run["contacts"] and run["contacts"]["contact"]) for run in runs)),
            spread([t["time_s"] for t in won], 1),
            spread([t["path_length_m"] for t in won], 2),
            spread([t["final_position_error_m"] for t in won], 3),
            spread([(t["cross_track_m"] or {}).get("p95") for t in tasks], 3),
            spread([t["min_footprint_clearance_to_static_map_m"] for t in tasks], 2),
            spread([t.get("min_footprint_clearance_to_obstacles_m") for t in tasks], 2),
            spread([t.get("min_footprint_clearance_to_movers_m") for t in tasks], 2),
            spread([sum(m["blockedSeconds"] for m in run["movers"]) if run.get("movers") else None
                    for run in runs], 1),
            spread([t["recoveries"] for t in tasks], 0),
            "/".join(str(actions.count(a)) for a in ("stop", "slowdown", "approach")),
            spread([t["cpu_percent_of_core"].get("controller_server") for t in tasks], 1),
            str(sum(log.get("loop_rate_misses", 0) for log in logs)),
            str(sum(log.get("errors", 0) for log in logs)))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("paths", nargs="+", help="run directories or summary JSON files")
    args = parser.parse_args()

    groups, missions = defaultdict(list), defaultdict(list)
    for run in load(args.paths):
        # Runs before 2026-10-01 stored true for live global obstacles.
        mode = {True: "live", False: "static", None: "static"}.get(run.get("global_obstacles"), run.get("global_obstacles"))
        controller = (run.get("controller") or "rpp") + {"live": " +global obstacles",
                                                         "persistent": " +persistent global"}.get(mode, "")
        (missions if run["scenario"]["task"] == "mission" else groups)[(run["scenario"]["name"], controller)].append(run)
    order = ("rpp", "dwb", "mppi")  # bring-up first, then B1 and B2

    def ordered(table):
        return sorted(table, key=lambda k: (k[0], "+" in k[1],
                                            order.index(k[1].split()[0]) if k[1].split()[0] in order else 9))

    if groups:
        print("| " + " | ".join(COLUMNS) + " |")
        print("|" + "---|" * len(COLUMNS))
        for key in ordered(groups):
            print("| " + " | ".join(row(*key, groups[key])) + " |")
    if missions:
        if groups:
            print()
        print("| " + " | ".join(MISSION_COLUMNS) + " |")
        print("|" + "---|" * len(MISSION_COLUMNS))
        for key in ordered(missions):
            print("| " + " | ".join(mission_row(*key, missions[key])) + " |")


if __name__ == "__main__":
    main()
