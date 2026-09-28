#!/usr/bin/env python3
"""Run deterministic navigation scenarios against the live Unity/ROS simulation.

Host-side orchestrator (Unity CLI + the Dev Container). For each scenario it:

 1. optionally starts a new simulation epoch (stop ROS arm/Nav2 nodes, restart Play,
    restart arm control) as ADR 0001 requires after a clock reset;
 2. stops the running Nav2 stack so no costmap survives the teleport;
 3. moves the arm to the scenario pose through home with the qualified 8 s transitions;
 4. checks the start footprint is free in the static map, then teleports the stopped
    robot with `scenario_place`;
 5. checks /cmd_vel ownership (ADR 0006) and relaunches Nav2 with the scenario's
    footprint profile;
 6. records a rosbag, runs the scenario task, and writes a run summary.

Output goes to experiment_runs/<UTC time>-<scenario>/ (git-ignored). Simulation only.
"""
import argparse
import datetime
import json
import math
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = "/workspaces/mm-motion-planning"
QUALIFIED_POSES = {
    "home": [0.0] * 6,
    "vertical_carry": [math.pi / 2, 0, 0, 0, math.pi / 2, 0],
}
NAV_PROCESSES = ["[g]lobal_planning.launch", "[p]lanner_server", "[m]ap_server",
                 "[l]ifecycle_manager_global_planning"]
BAG_TOPICS = ["/clock", "/tf", "/tf_static", "/joint_states", "/cmd_vel", "/plan",
              "/global_costmap/costmap", "/global_costmap/published_footprint", "/map"]


class Runner:
    def __init__(self, container):
        self.container = container

    def unity(self, command, *parameters, timeout=60):
        result = subprocess.run(["unity", "command", command, *parameters, "--format", "json"],
                                text=True, capture_output=True, timeout=timeout)
        if result.returncode:
            raise RuntimeError(f"unity {command} failed: {result.stdout}{result.stderr}")
        envelope = json.loads(result.stdout)
        if not envelope["success"]:
            raise RuntimeError(result.stdout)
        return envelope["data"]["result"]

    def ros(self, script, timeout=120, check=True):
        """Run a bash snippet in the container with the workspace sourced."""
        result = subprocess.run(
            ["docker", "exec", self.container, "bash", "-c",
             'source "$ROS_WS/install/setup.bash" && ' + script],
            text=True, capture_output=True, timeout=timeout)
        if check and result.returncode:
            raise RuntimeError(f"container command failed ({result.returncode}):\n{script}\n"
                               f"{result.stdout}\n{result.stderr}")
        return result

    def start(self, name, command):
        """Start a background process group in the container, recorded in /tmp/mm_<name>.pid."""
        self.ros(f"setsid nohup {command} > /tmp/mm_{name}.log 2>&1 & echo $! > /tmp/mm_{name}.pid")

    def stop(self, name, patterns):
        """Stop a recorded process group and any other matching process; fail if one survives.

        Patterns use the '[x]yz' form so they never match the shell running them.
        """
        self.ros(f'if [ -f /tmp/mm_{name}.pid ]; then kill -INT -- -$(cat /tmp/mm_{name}.pid) '
                 f'2>/dev/null; rm -f /tmp/mm_{name}.pid; fi', check=False)
        regex = "|".join(patterns)
        for attempt in range(40):
            alive = self.ros(f"ps -eo args | grep -E '{regex}' || true").stdout.strip()
            if not alive:
                return
            if attempt in (5, 20):  # processes started outside this runner, or stuck on SIGINT
                signal = "INT" if attempt == 5 else "TERM"
                self.ros(f"pkill -{signal} -f '{regex}' || true", check=False)
            time.sleep(1)
        raise RuntimeError(f"{name} processes still running; stop them before retrying:\n{alive}")

    def playing(self):
        try:
            return self.unity("arm_test_snapshot")["playing"]
        except RuntimeError as error:
            if "Enter Play" in str(error):
                return False
            raise

    def wait_for_hold(self, seconds=60):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            snapshot = self.unity("arm_test_snapshot")
            if snapshot["state"] == "HOLD" and 0 <= snapshot["age"] < 0.5:
                return snapshot
            time.sleep(1)
        raise RuntimeError("Arm did not reach fresh controlled HOLD")

    def new_epoch(self):
        self.stop("nav", NAV_PROCESSES)
        self.stop("arm", ["[a]rm_control.launch", "[r]os2_control_node",
                          "[c]ontrol_description", "[c]ontroller_manager/spawner"])
        if self.playing():
            self.unity("editor_stop")
            time.sleep(2)
        self.unity("editor_play")
        time.sleep(5)
        self.start("arm", "ros2 launch mobile_manipulator_control arm_control.launch.py")
        return self.wait_for_hold()

    def move_arm(self, target):
        """Qualified transitions only: through home, 8 s synchronized cubic, zero end velocity."""
        def at(pose, q):
            return all(abs(a - b) <= 0.04 for a, b in zip(q, QUALIFIED_POSES[pose]))

        current = self.wait_for_hold()["q"]
        if at(target, current):
            return
        steps = ([] if at("home", current) else ["home"]) + ([] if target == "home" else [target])
        for pose in steps:
            q = QUALIFIED_POSES[pose]
            result = self.ros(
                "python3 $ROS_WS/src/mobile_manipulator_control/scripts/arm_experiment.py "
                "--positions " + " ".join(str(v) for v in q) + " --duration 8 --hold-seconds 2",
                timeout=90)
            report = json.loads(result.stdout.strip().splitlines()[-1])
            if report["status"] != 4 or report["error_code"] != 0:
                raise RuntimeError(f"Arm transition failed: {report}")
        self.wait_for_hold()


def git_state():
    revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True,
                              capture_output=True).stdout.strip()
    dirty = bool(subprocess.run(["git", "status", "--porcelain"], cwd=ROOT, text=True,
                                capture_output=True).stdout.strip())
    return {"commit": revision, "dirty": dirty}


def run_scenario(runner, name, args):
    resolved = runner.ros(f"ros2 run mobile_manipulator_navigation scenario_spec.py {name}",
                          check=False)
    spec = json.loads(resolved.stdout.strip().splitlines()[-1])
    if resolved.returncode:
        raise RuntimeError(f"Scenario {name} rejected: {spec}")
    scenario = spec["scenario"]
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = f"experiment_runs/{stamp}-{name}"
    (ROOT / run_dir).mkdir(parents=True)
    print(f"== {name}: {scenario['description']}", flush=True)

    runner.stop("nav", NAV_PROCESSES)
    runner.move_arm(scenario["arm_pose"])
    x, y, yaw = scenario["start"]
    runner.unity("scenario_place", "--x", str(x), "--y", str(y), "--yaw", str(yaw),
                 "--arm_pose", scenario["arm_pose"])
    time.sleep(1.0)
    runner.ros("ros2 run mobile_manipulator_navigation check_cmd_vel_ownership.py")

    runner.start("nav", "ros2 launch mobile_manipulator_navigation global_planning.launch.py "
                        f"footprint_profile:={scenario['footprint_profile']}")
    runner.ros("for i in $(seq 1 60); do ros2 lifecycle get /planner_server 2>/dev/null "
               "| grep -q '^active' && exit 0; sleep 1; done; exit 1", timeout=90)
    count = runner.ros("ps -eo args | grep -c '[p]lanner_server'").stdout.strip()
    if count != "1":
        raise RuntimeError(f"Expected one planner_server, found {count}")

    topics = BAG_TOPICS + (["/livox/lidar"] if args.record_lidar else [])
    runner.start("bag", f"ros2 bag record -o {WORKSPACE}/{run_dir}/bag " + " ".join(topics))
    time.sleep(2.0)
    started = time.monotonic()
    try:
        if scenario["task"] != "compute_path":
            raise RuntimeError(f"Unsupported task {scenario['task']}")
        start, goal = scenario["start"], scenario["goal"]
        runner.ros("ros2 run mobile_manipulator_navigation plan_scenario_task.py "
                   f"--start {start[0]} {start[1]} {start[2]} --goal {goal[0]} {goal[1]} {goal[2]} "
                   f"--planners {' '.join(scenario['planners'])} --repeats {args.repeats} "
                   f"--output {WORKSPACE}/{run_dir}/task.json", timeout=300)
    finally:
        runner.stop("bag", ["[r]os2 bag record"])
    task = json.loads((ROOT / run_dir / "task.json").read_text())
    summary = {"scenario": scenario, "footprint_polygon": spec["polygon"], "git": git_state(),
               "utc": stamp, "wall_seconds": round(time.monotonic() - started, 2),
               "bag": f"{run_dir}/bag", "task": task}
    (ROOT / run_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    for planner, result in task["planners"].items():
        best = result["runs"][0]
        print(f"   {planner:10s} success={result['success_rate']:.2f} length={best['length_m']} m "
              f"clearance={best['min_static_clearance_m']} m", flush=True)
    return run_dir


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scenarios", nargs="*", help="scenario names (default: all)")
    parser.add_argument("--container", default="mm-motion-planning-ma-robot-sim-1")
    parser.add_argument("--new-epoch", action="store_true",
                        help="restart Play and arm control before the first scenario")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--record-lidar", action="store_true",
                        help="also record /livox/lidar (large bags)")
    args = parser.parse_args()

    runner = Runner(args.container)
    names = args.scenarios or json.loads(runner.ros(
        "ros2 run mobile_manipulator_navigation scenario_spec.py --list").stdout.strip().splitlines()[-1])
    if args.new_epoch:
        runner.new_epoch()
    elif not runner.playing():
        sys.exit("Unity is not in Play; use --new-epoch")
    runs = [run_scenario(runner, name, args) for name in names]
    print("Runs written:", *runs, sep="\n  ")


if __name__ == "__main__":
    main()
