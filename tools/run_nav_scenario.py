#!/usr/bin/env python3
"""Run deterministic navigation scenarios against the live Unity/ROS simulation.

Host-side orchestrator (Unity CLI + the Dev Container). For each scenario it:

 1. optionally starts a new simulation epoch (stop ROS arm/Nav2 nodes, restart the
    ROS-TCP endpoint, restart Play, restart arm control) as ADR 0001 requires after a
    clock reset;
 2. stops the running Nav2 stack so no costmap survives the teleport;
 3. checks the start footprint is free in the static map, then teleports the stopped
    robot with `scenario_place` in the qualified pose the arm holds (restored with MoveIt
    first if a mission left the arm in an unqualified IK configuration);
 4. moves the arm to the scenario pose through home with the qualified 8 s transitions and
    places the scenario's unmapped obstacles and movers (removed again after the run;
    mover progress and waiting time go into the summary);
 5. checks /cmd_vel ownership (ADR 0006) and relaunches Nav2 (global planning for
    compute_path, the full navigation stack with the selected --controller for
    navigate_to_pose) with the scenario's footprint profile;
 6. records a rosbag, runs the scenario task, and writes a run summary. After a
    navigate_to_pose task it confirms the base has stopped and adds the controller
    server's loop-rate warnings and errors from the launch log.

A mission scenario (B3) also starts manipulation.launch.py after the teleport (MoveIt with
the scenario's obstacles, ReconfigurePanel starting from the scenario's footprint profile),
runs mission_scenario_task, records the arm in Unity for the physical checks
(arm.csv.gz, judged by tools/analyze_arm_qualification.py), and stops MoveIt afterwards.

Output goes to experiment_runs/<UTC time>-<scenario>[-<controller>]/ (git-ignored).
Simulation only.
"""
import argparse
import collections
import datetime
import gzip
import json
import math
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

from analyze_arm_qualification import analyze as analyze_arm

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
              "/global_costmap/costmap", "/global_costmap/published_footprint", "/map",
              "/scenario/movers"]
LAUNCH = {"compute_path": ("global_planning.launch.py", "planner_server"),
          "navigate_to_pose": ("navigation.launch.py", "bt_navigator"),
          "mission": ("navigation.launch.py", "bt_navigator")}
MANAGERS = {"compute_path": ["lifecycle_manager_global_planning"],
            "navigate_to_pose": ["lifecycle_manager_global_planning", "lifecycle_manager_navigation"],
            "mission": ["lifecycle_manager_global_planning", "lifecycle_manager_navigation"]}
MOVEIT_PROCESSES = ["[m]anipulation.launch", "[m]ove_group", "[r]econfigure_panel_server",
                    "[p]lanning_scene_loader"]
MISSION_BAG_TOPICS = ["/planning_scene", "/trajectory_execution_event", "/arm_controller/controller_state",
                      "/reconfigure_panel/_action/status"]
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
        self.stop("endpoint", ["[l]ib/mobile_manipulator_control/unity_control_endpoint",
                               "[r]os2 run mobile_manipulator_control unity_control_endpoint"])
        self.start("endpoint", "ros2 run mobile_manipulator_control unity_control_endpoint "
                               "--ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000")
        time.sleep(3)
        self.unity("editor_play")
        time.sleep(5)
        self.start("arm", "ros2 launch mobile_manipulator_control arm_control.launch.py")
        return self.wait_for_hold()

    def qualified_pose(self):
        """Name of the qualified pose the arm holds, or None."""
        held = self.wait_for_hold()["q"]
        return next((name for name, q in QUALIFIED_POSES.items()
                     if all(abs(a - b) <= 0.04 for a, b in zip(held, q))), None)

    def restore_qualified_pose_with_moveit(self, poses=("vertical_carry", "home")):
        """Plan the arm into the first reachable qualified pose of `poses` with MoveIt.

        A mission that stops mid-way leaves the arm in MoveIt's IK solution for a panel pose,
        not a qualified pose; the qualified straight transition from there is neither
        qualified nor collision-checked, and scenario_place needs a qualified pose. Vertical
        carry is tried first because it fits where vertical-carry drives stop (a gate throat).
        Call it after leftover scenario boxes are removed: its scene has no scenario boxes.
        """
        self.stop("moveit", MOVEIT_PROCESSES)
        self.start("moveit", "ros2 launch mobile_manipulator_manipulation manipulation.launch.py "
                             "initial_footprint_profile:=home")
        try:
            self.ros("for i in $(seq 1 90); do grep -q 'ReconfigurePanel ready' /tmp/mm_moveit.log && "
                     "grep -q 'Loaded [0-9]* static' /tmp/mm_moveit.log && exit 0; sleep 1; done; exit 1",
                     timeout=120)
            for pose in poses:
                result = self.ros(f"ros2 run mobile_manipulator_manipulation reconfigure_panel --named {pose} "
                                  f"--profile {pose} --planning-time 10", timeout=180, check=False)
                if result.returncode == 0:
                    break
        finally:
            self.stop("moveit", MOVEIT_PROCESSES)
        if result.returncode:
            raise RuntimeError(f"MoveIt could not reach a qualified pose:\n{result.stdout[-1500:]}")
        return self.qualified_pose()

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
                "ros2 run mobile_manipulator_control arm_experiment "
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
    resolved = runner.ros(f"ros2 run mobile_manipulator_navigation scenario_spec {name}",
                          check=False)
    spec = json.loads(resolved.stdout.strip().splitlines()[-1])
    if resolved.returncode:
        raise RuntimeError(f"Scenario {name} rejected: {spec}")
    scenario = spec["scenario"]
    navigating = scenario["task"] in ("navigate_to_pose", "mission")
    mission = scenario["task"] == "mission"
    controller = args.controller if navigating else None
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = (f"experiment_runs/{stamp}-{name}" + (f"-{controller}" if navigating else "")
               + ({"static": "-staticglobal", "live": "-liveglobal"}.get(args.global_obstacles, "")
                  if navigating else "")
               + ("-octomap" if mission and args.scene_source == "octomap" else ""))
    (ROOT / run_dir).mkdir(parents=True)
    print(f"== {name}{f' ({controller})' if navigating else ''}: {scenario['description']}", flush=True)

    runner.stop("nav", NAV_PROCESSES)
    runner.stop("moveit", MOVEIT_PROCESSES)
    arm_restarts = 0
    try:
        runner.wait_for_hold(seconds=10)
    except RuntimeError as error:
        # The arm hardware latches a fault when Unity feedback pauses for more than 0.5 s
        # (seen at run transitions); restart arm control once before moving the arm.
        print(f"   {error}; restarting arm control", flush=True)
        runner.restart_arm_control()
        arm_restarts = 1
    runner.unity("scenario_obstacle_clear")  # leftovers from an interrupted run, before MoveIt moves the arm
    held_pose = runner.qualified_pose()
    if held_pose is None:
        print("   arm is not in a qualified pose (left by a mission); restoring one with MoveIt", flush=True)
        held_pose = runner.restore_qualified_pose_with_moveit()
    if held_pose != scenario["arm_pose"]:
        # The start is checked free for the scenario's profile; teleporting with another pose
        # needs that pose's profile to fit there too, otherwise change pose here with MoveIt.
        check = json.loads(runner.ros(f"ros2 run mobile_manipulator_navigation scenario_spec {name} "
                                      f"--start-free-for {held_pose}", check=False).stdout.strip().splitlines()[-1])
        if not check.get("start_free_for", {}).get("free", False):
            print(f"   start is not free for the held {held_pose} pose; changing to "
                  f"{scenario['arm_pose']} with MoveIt before the teleport", flush=True)
            held_pose = runner.restore_qualified_pose_with_moveit((scenario["arm_pose"],))
    # Teleport with the pose the arm holds, then change pose at the free start: a previous run
    # may have ended where the scenario pose does not fit (a mission stopped in a gate).
    x, y, yaw = scenario["start"]
    runner.unity("scenario_place", "--x", str(x), "--y", str(y), "--yaw", str(yaw),
                 "--arm_pose", held_pose)
    runner.move_arm(scenario["arm_pose"])
    for obstacle in scenario.get("obstacles", []):
        runner.unity("scenario_obstacle", "--name", obstacle["name"],
                     *(arg for key in ("x", "y", "size_x", "size_y", "height")
                       for arg in (f"--{key}", str(obstacle[key]))))
    for mover in scenario.get("movers", []):
        runner.unity("scenario_mover", "--name", mover["name"],
                     "--start_x", str(mover["start"][0]), "--start_y", str(mover["start"][1]),
                     "--end_x", str(mover["end"][0]), "--end_y", str(mover["end"][1]),
                     "--size_x", str(mover["size_x"]), "--size_y", str(mover["size_y"]),
                     "--height", str(mover["height"]), "--speed", str(mover["speed_mps"]),
                     "--trigger_distance", str(mover["trigger_distance_m"]),
                     "--crossings", str(mover["crossings"]))
    time.sleep(1.0)
    runner.ros("ros2 run mobile_manipulator_navigation check_cmd_vel_ownership")
    if mission:
        # After placement: the server learns the profile Nav2 starts with, and the scene loader
        # adds this scenario's obstacles.
        runner.start("moveit", "ros2 launch mobile_manipulator_manipulation manipulation.launch.py "
                               f"scenario:={name} initial_footprint_profile:={scenario['footprint_profile']} "
                               f"scene_source:={args.scene_source}")
        runner.ros("for i in $(seq 1 90); do grep -q 'Loaded [0-9]* static' /tmp/mm_moveit.log && "
                   "grep -q 'ReconfigurePanel ready' /tmp/mm_moveit.log && exit 0; sleep 1; done; "
                   "tail -30 /tmp/mm_moveit.log; exit 1", timeout=120)

    launch_file, last_node = LAUNCH[scenario["task"]]
    for attempt in (1, 2):
        runner.start("nav", f"ros2 launch mobile_manipulator_navigation {launch_file} "
                            f"footprint_profile:={scenario['footprint_profile']}"
                            + (f" controller:={controller}" if navigating else "")
                            + (f" global_obstacles:={args.global_obstacles}" if navigating else "")
                            # Missions switch the footprint profile at standstill; the collision
                            # monitor's zones must follow ReconfigurePanel, not the launch profile.
                            + (" dynamic_monitor_zones:=true" if mission else ""))
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
        runner.ros(f"ros2 run mobile_manipulator_navigation check_controller_params {controller}",
                   timeout=60)
        # Label the run in the Unity telemetry window (best effort; not part of the result).
        runner.ros(f"timeout 15 ros2 param set /nav_telemetry scenario {name}", check=False)

    octomap_check = None
    if mission and args.scene_source == "octomap":
        # What the lidar Octomap holds in each scenario box, with the robot stopped at the start
        # where the first reconfiguration happens (evidence of perception, e.g. a blind-zone box).
        time.sleep(3.0)
        checked = runner.ros(f"ros2 run mobile_manipulator_manipulation octomap_box_check --scenario {name}",
                             timeout=60, check=False)
        octomap_check = json.loads(checked.stdout.strip().splitlines()[-1]) if checked.returncode == 0 else None
        print(f"   octomap at the start: {octomap_check}", flush=True)
    topics = BAG_TOPICS + (["/livox/lidar"] if args.record_lidar else []) + (MISSION_BAG_TOPICS if mission else [])
    runner.start("bag", f"ros2 bag record -o {WORKSPACE}/{run_dir}/bag " + " ".join(topics))
    time.sleep(2.0)
    started = time.monotonic()
    contacts = movers = None
    start, goal = scenario["start"], scenario["goal"]
    poses = f"--start {start[0]} {start[1]} {start[2]} --goal {goal[0]} {goal[1]} {goal[2]}"
    try:
        if scenario["task"] == "compute_path":
            runner.ros("ros2 run mobile_manipulator_navigation plan_scenario_task "
                       f"{poses} --planners {' '.join(scenario['planners'])} "
                       f"--repeats {args.repeats} --output {WORKSPACE}/{run_dir}/task.json",
                       timeout=300)
        elif scenario["task"] == "navigate_to_pose":
            runner.unity("scenario_contacts_reset")
            # Exit code 3 means the goal ran but did not succeed; that is a result, not an error.
            result = runner.ros("ros2 run mobile_manipulator_navigation navigate_scenario_task "
                                f"{poses} --footprint-profile {scenario['footprint_profile']} "
                                f"--timeout {scenario['timeout_s']} "
                                f"--obstacles '{json.dumps(scenario.get('obstacles', []))}' "
                                f"--movers '{json.dumps(scenario.get('movers', []))}' "
                                f"--output {WORKSPACE}/{run_dir}/task.json",
                                timeout=scenario["timeout_s"] * 4 + 120, check=False)
            if result.returncode not in (0, 3):
                raise RuntimeError(f"Navigation task failed to run:\n{result.stdout}\n{result.stderr}")
        elif mission:
            runner.unity("scenario_contacts_reset")
            # Unity's arm recorder gives the physical checks (panel ground clearance, base tilt,
            # watchdog) for the whole mission; it writes under the arm qualification directory.
            arm_stem = f"mission-{stamp}-{name}".replace("_", "-")  # the recorder allows [A-Za-z0-9-]
            runner.unity("arm_test_record", "--name", arm_stem)
            drives = sum("navigate" in step for step in scenario["steps"])
            try:
                result = runner.ros("ros2 run mobile_manipulator_navigation mission_scenario_task "
                                    f"--scenario {name} --output {WORKSPACE}/{run_dir}/task.json",
                                    timeout=scenario["timeout_s"] * drives * 4 + 600, check=False)
            finally:
                runner.unity("arm_test_end")
                recording = ROOT / "docs/experiments/arm-controller/qualification" / (arm_stem + ".csv")
                if recording.exists():
                    with recording.open("rb") as raw, gzip.open(ROOT / run_dir / "arm.csv.gz", "wb") as packed:
                        shutil.copyfileobj(raw, packed)
                    recording.unlink()
            if result.returncode not in (0, 3):
                raise RuntimeError(f"Mission task failed to run:\n{result.stdout}\n{result.stderr}")
        else:
            raise RuntimeError(f"Unsupported task {scenario['task']}")
    finally:
        runner.stop("bag", ["[r]os2 bag record"])
        if mission:
            runner.ros("cp /tmp/mm_moveit.log " + f"{WORKSPACE}/{run_dir}/moveit.log", check=False)
            runner.stop("moveit", MOVEIT_PROCESSES)
        if navigating:
            contacts = runner.unity("scenario_contacts")
            if scenario.get("movers"):
                movers = runner.unity("scenario_movers")["movers"]
            runner.wait_until_stopped()
        if scenario.get("obstacles") or scenario.get("movers"):
            runner.unity("scenario_obstacle_clear")
    task = json.loads((ROOT / run_dir / "task.json").read_text())
    if "preflight_failed" in task:
        raise RuntimeError(f"Navigation preflight failed: {task['preflight_failed']}")
    arm_physical = None
    if mission and (ROOT / run_dir / "arm.csv.gz").exists():
        # The analyzer expects an action record next to the recording.
        (ROOT / run_dir / "arm.json").write_text(json.dumps({
            "status": 4 if task["status"] == "succeeded" else 6, "error_code": 0 if task["status"] == "succeeded" else 1,
            "hold_max_error": [s["result"].get("hold_error_rad") for s in task["steps"]
                               if s["type"] == "reconfigure" and isinstance(s.get("result"), dict)] or [0.0],
            "disturbance": "mission"}) + "\n")
        arm_physical = analyze_arm(ROOT / run_dir / "arm.csv.gz")
    summary = {"scenario": scenario, "footprint_polygon": spec["polygon"], "git": git_state(),
               "controller": controller, "nav_launch_attempts": attempt,
               "global_obstacles": args.global_obstacles if navigating else None,
               "arm_control_restarts": arm_restarts,
               "controller_log": runner.controller_log() if navigating else None,
               "contacts": contacts if navigating else None,
               "movers": movers,
               "utc": stamp, "wall_seconds": round(time.monotonic() - started, 2),
               "bag": f"{run_dir}/bag", "task": task, "arm_physical": arm_physical,
               "scene_source": args.scene_source if mission else None, "octomap_check": octomap_check}
    (ROOT / run_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    if mission:
        print(f"   mission {task['status']} in {task['total_time_s']} s (drives {task['drive_time_s']} s, "
              f"reconfigurations {task['reconfigure_time_s']} s)"
              + (f", failed at step {task['failed_step']}" if task["failed_step"] else ""), flush=True)
        for step in task["steps"]:
            if step["type"] == "navigate":
                print(f"   step {step['step']} drive {step['status']} in {step.get('time_s')} s, "
                      f"path {step.get('path_length_m')} m, min clearance "
                      f"{step.get('min_footprint_clearance_to_static_map_m')} m, profile {step['footprint_profile']}",
                      flush=True)
            else:
                r = step["result"]
                print(f"   step {step['step']} reconfigure {r.get('error_code')} "
                      f"(plan {r.get('planning_time_s')} s, motion {r.get('trajectory_duration_s')} s, "
                      f"clearance {r.get('min_planned_clearance_m')} m) -> {r.get('applied_footprint_profile')}",
                      flush=True)
        if arm_physical:
            print(f"   arm: panel bottom >= {arm_physical['min_panel_bottom_m']:.3f} m, tilt <= "
                  f"{arm_physical['max_base_tilt_degrees']:.2f} deg, path error <= "
                  f"{arm_physical['max_path_error_rad']:.3f} rad, checks "
                  f"{'pass' if arm_physical['passed'] else [k for k, v in arm_physical['checks'].items() if not v]}",
                  flush=True)
        if contacts["contact"]:
            worst = contacts["contacts"][0]
            print(f"   CONTACT: {worst['robot']} with {worst['other']}, {worst['maxPenetration']} m", flush=True)
        else:
            print("   no robot-environment contact", flush=True)
    elif scenario["task"] == "compute_path":
        for planner, result in task["planners"].items():
            best = result["runs"][0]
            print(f"   {planner:10s} success={result['success_rate']:.2f} length={best['length_m']} m "
                  f"clearance={best['min_static_clearance_m']} m", flush=True)
    else:
        cross = task["cross_track_m"] or {}
        print(f"   {task['status']} in {task['time_s']} s, path {task['path_length_m']} m, "
              f"final error {task['final_position_error_m']} m / {task['final_yaw_error_rad']} rad, "
              f"cross-track p95 {cross.get('p95')} m, min clearance "
              f"{task['min_footprint_clearance_to_static_map_m']} m"
              + (f" (obstacles {task['min_footprint_clearance_to_obstacles_m']} m)"
                 if task.get("min_footprint_clearance_to_obstacles_m") is not None else "")
              + (f" (movers {task['min_footprint_clearance_to_movers_m']} m)"
                 if task.get("min_footprint_clearance_to_movers_m") is not None else "")
              + f", recoveries {task['recoveries']}, "
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
        for mover in movers or []:
            print(f"   mover {mover['name']}: triggered at t={mover['triggerTime']} s, walked {mover['walked']} m, "
                  f"waited {mover['blockedSeconds']} s for the robot", flush=True)
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
    parser.add_argument("--global-obstacles", choices=("persistent", "live", "static"), default="persistent",
                        help="lidar obstacles in the global costmap: persistent ones (default), all "
                             "(live, 10 s decay), or none (static, the Phase 1 baseline)")
    parser.add_argument("--static-global-costmap", action="store_const", dest="global_obstacles",
                        const="static", help="same as --global-obstacles static")
    parser.add_argument("--scene-source", choices=("known", "octomap"), default="known",
                        help="MoveIt collision world for mission scenarios: known geometry (default) or "
                             "the lidar Octomap (scenario boxes are then not given to MoveIt)")
    parser.add_argument("--record-lidar", action="store_true",
                        help="also record /livox/lidar (large bags)")
    args = parser.parse_args()

    runner = Runner(args.container)
    names = args.scenarios or json.loads(runner.ros(
        "ros2 run mobile_manipulator_navigation scenario_spec --list").stdout.strip().splitlines()[-1])
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
