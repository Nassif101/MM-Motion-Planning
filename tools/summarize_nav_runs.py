#!/usr/bin/env python3
"""Summarise navigate_to_pose scenario runs per scenario and controller as Markdown.

Reads run summaries written by tools/run_nav_scenario.py: run directories
(experiment_runs/<run>/summary.json) or copied summary files (*-summary.json). Cells
show the median over runs and, for several runs, the range in parentheses. Time and
path length use successful runs only; every other column uses all runs.

  python3 tools/summarize_nav_runs.py docs/experiments/nav2-navigation/runs/*-summary.json
"""
import argparse
import json
import statistics
from collections import defaultdict
from pathlib import Path

COLUMNS = ("Scenario", "Controller", "Success", "Contact", "Time s", "Path m",
           "Cross-track p95 m", "Min clearance m", "Recoveries", "Monitor stop/slow/appr",
           "Controller CPU %", "Loop misses", "Controller errors")


def load(paths):
    runs = []
    for path in map(Path, paths):
        path = path / "summary.json" if path.is_dir() else path
        if not path.is_file():
            continue
        summary = json.loads(path.read_text())
        if summary["scenario"].get("task") == "navigate_to_pose" and "status" in summary["task"]:
            runs.append(summary)
    return runs


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
            spread([(t["cross_track_m"] or {}).get("p95") for t in tasks], 3),
            spread([t["min_footprint_clearance_to_static_map_m"] for t in tasks], 2),
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

    groups = defaultdict(list)
    for run in load(args.paths):
        groups[(run["scenario"]["name"], run.get("controller") or "rpp")].append(run)
    print("| " + " | ".join(COLUMNS) + " |")
    print("|" + "---|" * len(COLUMNS))
    order = ("rpp", "dwb", "mppi")  # bring-up first, then B1 and B2
    for key in sorted(groups, key=lambda k: (k[0], order.index(k[1]) if k[1] in order else 9, k[1])):
        print("| " + " | ".join(row(*key, groups[key])) + " |")


if __name__ == "__main__":
    main()
