#!/usr/bin/env python3
"""Run deterministic navigation scenarios against the live Unity/ROS simulation.

Host-side orchestrator (Unity CLI + the Dev Container). For each scenario it:

 1. optionally starts a new simulation epoch (stop ROS arm/Nav2 nodes, restart the
    ROS-TCP endpoint, restart Play, restart arm control) as ADR 0001 requires after a
    clock reset;
 2. stops the running Nav2 stack so no costmap survives the teleport;
 3. moves the arm to the scenario pose through home with the qualified 8 s transitions;
 4. checks the start footprint is free in the static map, then teleports the stopped
    robot with `scenario_place`;
 5. checks /cmd_vel ownership (ADR 0006) and relaunches Nav2 (global planning for
    compute_path, the full navigation stack with the selected --controller for
    navigate_to_pose) with the scenario's footprint profile;
 6. records a rosbag, runs the scenario task, and writes a run summary. After a
    navigate_to_pose task it confirms the base has stopped and adds the controller
    server's loop-rate warnings and errors from the launch log.

Output goes to experiment_runs/<UTC time>-<scenario>[-<controller>]/ (git-ignored).
Simulation only.
"""
import argparse
import collections
import datetime
import json
import math
import re
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
NAV_PROCESSES = ["[g]lobal_planning.launch", "[n]avigation.launch", "[p]lanner_server",
                 "[m]ap_server", "[c]ontroller_server", "[b]ehavior_server", "[v]elocity_smoother",
                 "[c]ollision_monitor", "[b]t_navigator", "[l]ivox_robot_filter",
                 "[l]ifecycle_manager_global_planning", "[l]ifecycle_manager_navigation",
                 "[n]av_telemetry"]
BAG_TOPICS = ["/clock", "/tf", "/tf_static", "/joint_states", "/odom", "/cmd_vel", "/plan",
              "/cmd_vel_nav", "/cmd_vel_smoothed", "/collision_monitor_state",
              "/local_costmap/costmap", "/local_costmap/published_footprint",
              "/global_costmap/costmap", "/global_costmap/published_footprint", "/map"]
LAUNCH = {"compute_path": ("global_planning.launch.py", "planner_server"),
          "navigate_to_pose": ("navigation.launch.py", "bt_navigator")}
MANAGERS = {"compute_path": ["lifecycle_manager_global_planning"],
            "navigate_to_pose": ["lifecycle_manager_global_planning", "lifecycle_manager_navigation"]}
ARM_PROCESSES = ["[a]rm_control.launch", "[r]os2_control_node", "[c]ontrol_description",
                 "[c]ontroller_manager/spawner"]
CONTROLLERS = ("rpp", "dwb", "mppi")
LOOP_MISS = re.compile(r"Control loop missed its desired rate of [\d.]+ ?Hz\. "
                       r"Current loop rate is ([\d.]+) ?Hz")


class Runner:
    def __init__(self, container):
        self.container = container

    def unity(self, command, *parameters, timeout=60):
        for attempt in range(10):
            result = subprocess.run(["unity", "command", command, *parameters, "--format", "json"],
                                    text=True, capture_output=True, timeout=timeout)
            # The Pipeline server is briefly unreachable while the Editor enters Play.
            if "No Unity Editor instances found" not in result.stdout or attempt == 9:
                break
            time.sleep(3)
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
        raise RuntimeError("Arm did not reach fresh controlled HOLD: " + json.dumps(
            {key: snapshot[key] for key in ("state", "age", "accepted", "rejected")}))

    def restart_arm_control(self):
        """Restart arm control in the same Play epoch (no clock reset, so ADR 0001 allows it)."""
        self.stop("arm", ARM_PROCESSES)
        self.start("arm", "ros2 launch mobile_manipulator_control arm_control.launch.py")
        return self.wait_for_hold()

    def controller_log(self):
        """Loop-rate misses and errors the controller server logged since the last launch."""
        lines = [line for line in self.ros("cat /tmp/mm_nav.log 2>/dev/null || true").stdout.splitlines()
                 if line.startswith("[controller_server")]
        rates = [float(match.group(1)) for line in lines if (match := LOOP_MISS.search(line))]
        errors = collections.Counter(line.split("]: ", 1)[-1][:120] for line in lines if "[ERROR]" in line)
        return {"loop_rate_misses": len(rates),
                "lowest_loop_rate_hz": round(min(rates), 2) if rates else None,
                "errors": sum(errors.values()),
                "top_errors": [{"count": n, "message": text} for text, n in errors.most_common(5)]}

    def wait_until_stopped(self, seconds=10):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self.unity("arm_test_snapshot")["speed"] < 0.05:
                return
            time.sleep(0.5)
        raise RuntimeError("Base did not stop after the navigation task")

    def new_epoch(self):
        self.stop("nav", NAV_PROCESSES)
        self.stop("arm", ARM_PROCESSES)
        if self.playing():
            self.unity("editor_stop")
            time.sleep(2)
        # A long-running endpoint was measured at higher CPU than a fresh one
        # (docs/experiments/lidar-transport); restart it so every epoch starts alike.
        self.stop("endpoint", ["[l]ib/mobile_manipulator_control/unity_control_endpoint.py",
                               "[r]os2 run mobile_manipulator_control unity_control_endpoint"])
        self.start("endpoint", "ros2 run mobile_manipulator_control unity_control_endpoint.py "
                               "--ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000")
        time.sleep(3)
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
    navigating = scenario["task"] == "navigate_to_pose"
    controller = args.controller if navigating else None
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = f"experiment_runs/{stamp}-{name}" + (f"-{controller}" if navigating else "")
    (ROOT / run_dir).mkdir(parents=True)
    print(f"== {name}{f' ({controller})' if navigating else ''}: {scenario['description']}", flush=True)

    runner.stop("nav", NAV_PROCESSES)
    arm_restarts = 0
    try:
        runner.wait_for_hold(seconds=10)
    except RuntimeError as error:
        # The arm hardware latches a fault when Unity feedback pauses for more than 0.5 s
        # (seen at run transitions); restart arm control once before moving the arm.
        print(f"   {error}; restarting arm control", flush=True)
        runner.restart_arm_control()
        arm_restarts = 1
    runner.move_arm(scenario["arm_pose"])
    x, y, yaw = scenario["start"]
    runner.unity("scenario_place", "--x", str(x), "--y", str(y), "--yaw", str(yaw),
                 "--arm_pose", scenario["arm_pose"])
    time.sleep(1.0)
    runner.ros("ros2 run mobile_manipulator_navigation check_cmd_vel_ownership.py")

    launch_file, last_node = LAUNCH[scenario["task"]]
    for attempt in (1, 2):
        runner.start("nav", f"ros2 launch mobile_manipulator_navigation {launch_file} "
                            f"footprint_profile:={scenario['footprint_profile']}"
                            + (f" controller:={controller}" if navigating else ""))
        try:
            runner.ros(f"for i in $(seq 1 60); do ros2 lifecycle get /{last_node} 2>/dev/null "
                       "| grep -q '^active' && exit 0; sleep 1; done; exit 1", timeout=90)
            # Every manager must report all its nodes active: a planner whose costmap failed
            # to activate still accepts goals and plans on an empty costmap.
            for manager in MANAGERS[scenario["task"]]:
                reply = runner.ros(f"timeout 20 ros2 service call /{manager}/is_active "
                                   "std_srvs/srv/Trigger", timeout=40, check=False).stdout
                if "success=True" not in reply:
                    raise RuntimeError(f"{manager} did not bring up all its nodes")
            break
        except (RuntimeError, subprocess.TimeoutExpired):
            # A lifecycle reply lost during DDS discovery leaves a Jazzy lifecycle manager
            # waiting forever (it has no service timeout), and a costmap whose transform wait
            # starts before /clock arrives times out at once; relaunch the stack once.
            print(f"   Nav2 did not come up (attempt {attempt}); relaunching", flush=True)
            runner.stop("nav", NAV_PROCESSES)
            if attempt == 2:
                raise
    count = runner.ros("ps -eo args | grep -c '[p]lanner_server'").stdout.strip()
    if count != "1":
        raise RuntimeError(f"Expected one planner_server, found {count}")
    if navigating:
        # A misspelled controller parameter would be ignored silently; refuse the run instead.
        runner.ros(f"ros2 run mobile_manipulator_navigation check_controller_params.py {controller}",
                   timeout=60)
        # Label the run in the Unity telemetry window (best effort; not part of the result).
        runner.ros(f"timeout 15 ros2 param set /nav_telemetry scenario {name}", check=False)

    topics = BAG_TOPICS + (["/livox/lidar"] if args.record_lidar else [])
    runner.start("bag", f"ros2 bag record -o {WORKSPACE}/{run_dir}/bag " + " ".join(topics))
    time.sleep(2.0)
    started = time.monotonic()
    contacts = None
    start, goal = scenario["start"], scenario["goal"]
    poses = f"--start {start[0]} {start[1]} {start[2]} --goal {goal[0]} {goal[1]} {goal[2]}"
    try:
        if scenario["task"] == "compute_path":
            runner.ros("ros2 run mobile_manipulator_navigation plan_scenario_task.py "
                       f"{poses} --planners {' '.join(scenario['planners'])} "
                       f"--repeats {args.repeats} --output {WORKSPACE}/{run_dir}/task.json",
                       timeout=300)
        elif scenario["task"] == "navigate_to_pose":
            runner.unity("scenario_contacts_reset")
            # Exit code 3 means the goal ran but did not succeed; that is a result, not an error.
            result = runner.ros("ros2 run mobile_manipulator_navigation navigate_scenario_task.py "
                                f"{poses} --footprint-profile {scenario['footprint_profile']} "
                                f"--timeout {scenario['timeout_s']} "
                                f"--output {WORKSPACE}/{run_dir}/task.json",
                                timeout=scenario["timeout_s"] * 4 + 120, check=False)
            if result.returncode not in (0, 3):
                raise RuntimeError(f"Navigation task failed to run:\n{result.stdout}\n{result.stderr}")
        else:
            raise RuntimeError(f"Unsupported task {scenario['task']}")
    finally:
        runner.stop("bag", ["[r]os2 bag record"])
        if navigating:
            contacts = runner.unity("scenario_contacts")
            runner.wait_until_stopped()
    task = json.loads((ROOT / run_dir / "task.json").read_text())
    if "preflight_failed" in task:
        raise RuntimeError(f"Navigation preflight failed: {task['preflight_failed']}")
    summary = {"scenario": scenario, "footprint_polygon": spec["polygon"], "git": git_state(),
               "controller": controller, "nav_launch_attempts": attempt,
               "arm_control_restarts": arm_restarts,
               "controller_log": runner.controller_log() if navigating else None,
               "contacts": contacts if navigating else None,
               "utc": stamp, "wall_seconds": round(time.monotonic() - started, 2),
               "bag": f"{run_dir}/bag", "task": task}
    (ROOT / run_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    if scenario["task"] == "compute_path":
        for planner, result in task["planners"].items():
            best = result["runs"][0]
            print(f"   {planner:10s} success={result['success_rate']:.2f} length={best['length_m']} m "
                  f"clearance={best['min_static_clearance_m']} m", flush=True)
    else:
        cross = task["cross_track_m"] or {}
        print(f"   {task['status']} in {task['time_s']} s, path {task['path_length_m']} m, "
              f"final error {task['final_position_error_m']} m / {task['final_yaw_error_rad']} rad, "
              f"cross-track p95 {cross.get('p95')} m, min clearance "
              f"{task['min_footprint_clearance_to_static_map_m']} m, recoveries {task['recoveries']}, "
              f"monitor {len(task['collision_monitor_activations'])}, "
              f"lidar gaps>0.5s {task['lidar_gaps_over_0p5s']}", flush=True)
        log = summary["controller_log"]
        print(f"   controller {controller}: cpu {task['cpu_percent_of_core'].get('controller_server')} % "
              f"of a core, loop-rate misses {log['loop_rate_misses']} "
              f"(lowest {log['lowest_loop_rate_hz']} Hz), errors {log['errors']}", flush=True)
        if contacts["contact"]:
            worst = contacts["contacts"][0]
            print(f"   CONTACT: {worst['robot']} with {worst['other']}, "
                  f"{worst['maxPenetration']} m at t={worst['firstTime']} s "
                  f"({len(contacts['contacts'])} pairs)", flush=True)
        else:
            print("   no robot-environment contact", flush=True)
    return run_dir, bool(contacts and contacts["contact"])


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scenarios", nargs="*", help="scenario names (default: all)")
    parser.add_argument("--container", default="mm-motion-planning-ma-robot-sim-1")
    parser.add_argument("--new-epoch", action="store_true",
                        help="restart Play and arm control before the first scenario")
    parser.add_argument("--repeats", type=int, default=3,
                        help="planner queries per planner (compute_path tasks)")
    parser.add_argument("--runs", type=int, default=1, help="runs per scenario")
    parser.add_argument("--controller", choices=CONTROLLERS, default="rpp",
                        help="local controller for navigate_to_pose scenarios")
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
    runs = [run_scenario(runner, name, args) for name in names for _ in range(args.runs)]
    print("Runs written:", *(run for run, _ in runs), sep="\n  ")
    touched = [run for run, contact in runs if contact]
    if touched:
        sys.exit("Robot-environment contact in: " + ", ".join(touched))


if __name__ == "__main__":
    main()
