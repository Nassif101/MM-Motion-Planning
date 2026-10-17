#!/usr/bin/env python3
"""Plan-only check of the V2 scenario set: does each scenario plan with each footprint profile?

For every scenario and every profile (home, vertical_carry, carry) this runs

    tools/run_nav_scenario.py <scenario> --plan-only --footprint-profile <profile>

(planners GridBased and Lattice; the runner switches Unity to the scenario's site), reads the run's
task.json, and compares "planned" (every planner succeeded) with
docs/experiments/site-v2/plan-expectations.yaml. It prints one table (success, length, total turn and
clearance per planner) and the mismatches, and exits 1 on a mismatch or a run that failed.

    python3 tools/check_site_plans.py [--scenarios NAME ...] [--profiles P ...] [--repeats N]
                                      [--no-new-epoch] [--container NAME]

Plan-only results depend on the 2D map alone (band 0.05-0.60 m): overhead members and door heads are not in
it, so the H1 scaffold ledger never blocks a plan. The first run opens a new Play epoch on the V2 scene
unless --no-new-epoch (Unity already playing). Run under `caffeinate -i`; it takes about a minute per run.
Simulation only.
"""
import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "tools" / "run_nav_scenario.py"
EXPECTATIONS = ROOT / "docs" / "experiments" / "site-v2" / "plan-expectations.yaml"
SCENARIOS = ROOT / "ros2_ws" / "src" / "mobile_manipulator_navigation" / "config" / "scenarios_site_v2.yaml"
PROFILES = ("home", "vertical_carry", "carry")


def parse_expectations(text):
    """{scenario: {profile: bool}} from the expectations file.

    The file is YAML but each entry is one flow-style line, `name: {home: true, vertical_carry: false,
    carry: true}`, so it is read here without PyYAML (the host has none). `#` starts a comment.
    """
    result = {}
    for number, raw in enumerate(text.splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line or line == "expectations:":
            continue
        match = re.fullmatch(r"(v2_\w+):\s*\{(.*)\}", line)
        if not match:
            raise ValueError(f"line {number}: expected 'name: {{home: bool, ...}}', got {raw!r}")
        entry = {}
        for pair in match.group(2).split(","):
            key, _, value = (part.strip() for part in pair.partition(":"))
            if key not in PROFILES or value not in ("true", "false"):
                raise ValueError(f"line {number}: bad entry {pair.strip()!r}")
            entry[key] = value == "true"
        if set(entry) != set(PROFILES):
            raise ValueError(f"line {number}: {match.group(1)} needs exactly {', '.join(PROFILES)}")
        result[match.group(1)] = entry
    return result


def scenario_names(text):
    """Scenario names of scenarios_site_v2.yaml, in file order."""
    return re.findall(r"^  (v2_\w+):\s*$", text, re.M)


def planned(task):
    """True when every planner of a compute_path task.json succeeded in every query."""
    planners = task["planners"]
    return bool(planners) and all(result["success_rate"] == 1.0 for result in planners.values())


def describe(task):
    """One cell per planner: success rate, path length, total turn and clearance of the first query."""
    cells = {}
    for name, result in task["planners"].items():
        run = result["runs"][0]
        cells[name] = (f"{result['success_rate']:.2f} {run['length_m']:.1f}m turn {run.get('total_turn_deg', float('nan')):.0f}"
                       f" clr {run['min_static_clearance_m']}")
    return cells


def mismatches(expected, observed):
    """[(scenario, profile, expected, observed)] where the observed plan outcome differs."""
    return [(name, profile, expected[name][profile], got)
            for name, profiles in observed.items() for profile, got in profiles.items()
            if got != expected[name][profile]]


def run_plan(scenario, profile, args, first):
    """Run one scenario plan-only; the run's task.json, or None with the runner's output tail on a failure."""
    command = [sys.executable, str(RUNNER), scenario, "--plan-only", "--footprint-profile", profile,
               "--repeats", str(args.repeats), "--container", args.container]
    if first and not args.no_new_epoch:
        command.append("--new-epoch")
    result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
    runs = re.findall(r"^  (experiment_runs/\S+)$", result.stdout, re.M)
    task = ROOT / runs[-1] / "task.json" if runs else None
    if result.returncode or task is None or not task.exists():
        return None, (result.stdout[-1500:] + result.stderr[-1500:]).strip()
    return json.loads(task.read_text()), runs[-1]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--scenarios", nargs="+", default=None, help="scenario names (default: all of the V2 set)")
    parser.add_argument("--profiles", nargs="+", choices=PROFILES, default=list(PROFILES))
    parser.add_argument("--repeats", type=int, default=1, help="planner queries per planner")
    parser.add_argument("--no-new-epoch", action="store_true", help="Unity is already in Play on the V2 scene")
    parser.add_argument("--container", default="mm-motion-planning-ma-robot-sim-1")
    args = parser.parse_args()

    expected = parse_expectations(EXPECTATIONS.read_text())
    names = scenario_names(SCENARIOS.read_text())
    missing = [name for name in names if name not in expected]
    extra = [name for name in expected if name not in names]
    if missing or extra:
        sys.exit(f"plan-expectations.yaml out of step with scenarios_site_v2.yaml: missing {missing}, extra {extra}")
    chosen = args.scenarios or names
    unknown = [name for name in chosen if name not in names]
    if unknown:
        sys.exit(f"unknown scenarios: {unknown}")

    observed, failed, first = {}, [], True
    print(f"{'scenario':32s} {'profile':15s} {'expect':6s} {'plan':6s} planners", flush=True)
    for scenario in chosen:
        for profile in args.profiles:
            task, detail = run_plan(scenario, profile, args, first)
            first = False
            if task is None:
                failed.append((scenario, profile, detail))
                print(f"{scenario:32s} {profile:15s} {str(expected[scenario][profile]):6s} ERROR  see below", flush=True)
                continue
            outcome = planned(task)
            observed.setdefault(scenario, {})[profile] = outcome
            cells = "; ".join(f"{planner} {cell}" for planner, cell in describe(task).items())
            print(f"{scenario:32s} {profile:15s} {str(expected[scenario][profile]):6s} {str(outcome):6s} {cells}", flush=True)

    wrong = mismatches(expected, observed)
    for scenario, profile, detail in failed:
        print(f"\nERROR {scenario} / {profile}:\n{detail}")
    for scenario, profile, want, got in wrong:
        print(f"MISMATCH {scenario} / {profile}: expected planned={want}, got planned={got}")
    if wrong or failed:
        sys.exit(1)
    print(f"\n{sum(len(p) for p in observed.values())} plan checks match plan-expectations.yaml")


if __name__ == "__main__":
    main()
