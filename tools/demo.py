#!/usr/bin/env python3
"""Demo launcher: one menu for bringing up the simulation, running scenarios and free driving.

Host-side, next to tools/run_nav_scenario.py (whose Runner it reuses). Unity must be open
with the scene loaded, and noVNC running in the container for RViz. Commands:

  up        start the endpoint, the description/TF stack, Unity Play and arm control
            (skips what is already up)
  rviz      open RViz in noVNC with the demo layout (map, costmaps, plans, footprint,
            collision-monitor zones, lidar, MoveIt scene and trajectory, 2D Goal Pose)
  run       run one scenario with tools/run_nav_scenario.py, then print its results table
  drive     free drive: Nav2 (+ MoveIt) where the robot stands; send goals with RViz's
            "2D Goal Pose" tool. --controller wholebody is B5, the whole-body MPC: it drives
            the base and the arm itself (no MoveIt, MPC at 50 Hz, map obstacles only);
            --no-slowdown-zone disables the collision monitor's slowdown zone (any controller);
            B5 takes --posture-mode, --comfort-pose, --clearance-weight and --no-slack-gate
            (wholebody_mpc.yaml's by default; --comfort-pose home --no-slack-gate is the recorded B5)
  arm POSE  in free drive, reconfigure the arm to home or vertical_carry with MoveIt (not under
            B5: the MPC owns the arm). carry is refused: it is qualified only through home with
            arm_experiment (run_nav_scenario.py --arm-pose carry) and as B5's comfort pose
  results   results table for the newest runs (default 5)
  stop      stop Nav2 and MoveIt
  down      stop everything, including Unity Play

Without a command it shows an interactive menu. Run results go to
experiment_runs/<UTC time>-<scenario>-<controller>[-dyn]/ (summary.json, task.json, bag/).
Simulation only.
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

from run_nav_scenario import (ARM_PROCESSES, CONTROLLERS, MOVEIT_PROCESSES, NAV_PROCESSES, WORKSPACE,
                              Runner, keep_host_awake, mpc_launch_arguments, mpc_overrides)

ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / "experiment_runs"
RVIZ_CONFIG = f"{WORKSPACE}/ros2_ws/src/mobile_manipulator_navigation/rviz/demo.rviz"
SCENARIOS = f"{WORKSPACE}/ros2_ws/src/mobile_manipulator_navigation/config/scenarios.yaml"
ENDPOINT = ["[l]ib/mobile_manipulator_control/unity_control_endpoint"]
DESCRIPTION = ["[s]imulation.launch.py", "[r]obot_state_publisher", "[s]tatic_transform_publisher.*map_to_odom"]
RVIZ = ["[r]viz2"]
FOOTPRINT_MODES = ("static", "profiles", "dynamic")
ARM_POSES = ("home", "vertical_carry", "carry")
# The poses MoveIt (reconfigure_panel) may plan into: carry was qualified only as a synchronized
# cubic through home (arm_experiment, the runner's move_arm) and as B5's comfort pose.
MOVEIT_ARM_POSES = ("home", "vertical_carry")
WHOLEBODY_MPC = ["[w]holebody_mpc_node"]
FREE_DRIVE_MPC_RATE_HZ = 50  # ruling R28
# Scenarios worth showing live, in the order of the menu; the rest are listed after them.
FEATURED = ["static_obstacle_detour_nav", "worker_crossing_nav", "narrow_gate_home_nav",
            "narrow_gate_vertical_carry_nav", "persistent_blockage_nav",
            "constrained_reconfiguration_mission", "wide_gate_mission"]


def running(runner, patterns):
    return bool(runner.ros(f"ps -eo args | grep -E '{'|'.join(patterns)}' || true").stdout.strip())


def up(runner):
    """Bring up whatever is not running yet; a full restart is `down` then `up`."""
    if not running(runner, DESCRIPTION):
        print("Starting the description/TF stack")
        runner.start("description", "ros2 launch mobile_manipulator_description simulation.launch.py")
    if runner.playing() and running(runner, ENDPOINT) and running(runner, ARM_PROCESSES):
        try:
            runner.wait_for_hold(seconds=10)
            print("Simulation already up (Unity in Play, arm holding)")
            return
        except RuntimeError as error:
            print(f"{error}; restarting arm control")
            runner.restart_arm_control()
            return
    print("Starting the endpoint, Unity Play and arm control (about 30 s)")
    runner.new_epoch()
    print("Simulation up")


def rviz(runner):
    if not runner.ros("pgrep -x Xvfb || true").stdout.strip():
        sys.exit("noVNC is not running; run `novnc` in a container terminal first")
    runner.stop("rviz", RVIZ)
    # The MoveIt displays read the URDF and SRDF from parameters; without them they wait for
    # move_group's SRDF topic and stay broken when RViz starts before MoveIt.
    runner.ros("python3 - <<'EOF'\n"
               "import yaml\n"
               "from ament_index_python.packages import get_package_share_directory as share\n"
               "read = lambda package, path: open(f'{share(package)}/{path}').read()\n"
               "yaml.safe_dump({'/**': {'ros__parameters': {\n"
               "    'robot_description': read('mobile_manipulator_description', 'urdf/mobile_manipulator.urdf'),\n"
               "    'robot_description_semantic': read('mobile_manipulator_moveit_config',\n"
               "                                       'config/mobile_manipulator.srdf')}}},\n"
               "               open('/tmp/mm_rviz_params.yaml', 'w'))\n"
               "EOF")
    runner.start("rviz", f'env DISPLAY="${{NOVNC_DISPLAY:-:99}}" rviz2 -d {RVIZ_CONFIG} '
                         "--ros-args -p use_sim_time:=true --params-file /tmp/mm_rviz_params.yaml")
    print("RViz started: http://localhost:6080/vnc.html?autoconnect=1&resize=scale")


def scenarios(runner):
    """Scenario name -> (task, description), featured scenarios first."""
    script = ("import json, yaml; s = yaml.safe_load(open('" + SCENARIOS + "'))['scenarios']; "
              "print(json.dumps({k: [v['task'], v['description']] for k, v in s.items()}))")
    table = json.loads(runner.ros(f'python3 -c "{script}"').stdout.strip().splitlines()[-1])
    order = [name for name in FEATURED if name in table] + [name for name in table if name not in FEATURED]
    return {name: table[name] for name in order}


def newest_runs(count):
    return sorted((path for path in RUNS.iterdir() if (path / "summary.json").exists()),
                  key=lambda path: path.name)[-count:] if RUNS.exists() else []


def results(paths):
    if not paths:
        print("No runs with a summary.json yet")
        return
    print("\nRun directories:", *(f"  {path.relative_to(ROOT)}" for path in paths), sep="\n", flush=True)
    subprocess.run([sys.executable, str(ROOT / "tools/summarize_nav_runs.py"), *map(str, paths)])


def run(runner, name, controller, footprint_mode):
    before = set(RUNS.iterdir()) if RUNS.exists() else set()
    command = [sys.executable, str(ROOT / "tools/run_nav_scenario.py"), name, "--controller", controller,
               *(["--footprint-mode", footprint_mode] if footprint_mode else [])]
    print("$", " ".join([str(Path(command[1]).relative_to(ROOT)), *command[2:]]), flush=True)
    code = subprocess.run(command, cwd=ROOT).returncode
    new = sorted(path for path in set(RUNS.iterdir()) - before if (path / "summary.json").exists())
    results(new)
    if code:
        print(f"(run_nav_scenario exited {code})")


def drive(runner, controller, footprint_mode, slowdown_zone=True, options=None):
    """Nav2 (and MoveIt, except under B5) where the robot stands, for goals clicked in RViz.

    `options` holds the B5 overrides of wholebody_mpc.yaml given on the command line (mpc_overrides).
    """
    wholebody = controller == "wholebody"
    options = options or {}
    if wholebody and footprint_mode not in (None, "wholebody"):
        sys.exit("--controller wholebody uses the wholebody footprint mode (base-only costmap footprint, "
                 f"swept-hull slowdown zone), not {footprint_mode}")
    footprint_mode = "wholebody" if wholebody else footprint_mode or "dynamic"
    runner.stop("nav", NAV_PROCESSES)
    runner.stop("moveit", MOVEIT_PROCESSES)
    runner.unity("scenario_obstacle_clear")
    pose = runner.qualified_pose()
    if pose is None:
        print("Arm is not in a qualified pose; restoring one with MoveIt")
        pose = runner.restore_qualified_pose_with_moveit()
    if footprint_mode not in ("static", "wholebody"):
        # ReconfigurePanel owns the footprint switch (profiles) or checks the hull (dynamic).
        runner.start("moveit", "ros2 launch mobile_manipulator_manipulation manipulation.launch.py "
                               f"initial_footprint_profile:={pose} footprint_mode:={footprint_mode}")
        runner.ros("for i in $(seq 1 90); do grep -q 'ReconfigurePanel ready' /tmp/mm_moveit.log && exit 0; "
                   "sleep 1; done; tail -30 /tmp/mm_moveit.log; exit 1", timeout=120)
    runner.ros("ros2 run mobile_manipulator_navigation check_cmd_vel_ownership")
    # B5: no scenario (the MPC sees the map boxes only), no references (RViz goals only), 50 Hz.
    runner.start("nav", "ros2 launch mobile_manipulator_navigation navigation.launch.py "
                        f"footprint_profile:={pose} footprint_mode:={footprint_mode} controller:={controller}"
                        + ("" if slowdown_zone else " slowdown_zone:=false")
                        + mpc_launch_arguments(options)
                        + (f" references:=none mpc_rate_hz:={FREE_DRIVE_MPC_RATE_HZ}" if wholebody else ""))
    runner.ros("for i in $(seq 1 60); do ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q '^active' "
               "&& exit 0; sleep 1; done; tail -30 /tmp/mm_nav.log; exit 1", timeout=90)
    if wholebody:
        runner.ros("ros2 run mobile_manipulator_navigation check_cmd_vel_ownership "
                   "--expect collision_monitor --expect-upstream wholebody_mpc", timeout=60)
    runner.ros("timeout 15 ros2 param set /nav_telemetry scenario free_drive", check=False)
    if wholebody:
        print(f"Free drive ready: WHOLEBODY (B5), {FREE_DRIVE_MPC_RATE_HZ} Hz, arm starts in {pose}"
              + "".join(f", {name} {value}" for name, value in options.items())
              + ("" if slowdown_zone else ", no slowdown zone") + ".\n"
              "Click '2D Goal Pose' in RViz; the MPC moves the arm itself, so `arm` is unavailable.")
        return
    print(f"Free drive ready: {controller.upper()}, footprint {footprint_mode}, arm {pose}.\n"
          "Click '2D Goal Pose' in RViz. Change the arm between goals with `arm home|vertical_carry`"
          + (" (needs footprint mode profiles or dynamic)." if footprint_mode == "static" else "."))


def arm(runner, pose):
    if pose not in MOVEIT_ARM_POSES:
        sys.exit(f"`{pose}` is not qualified with MoveIt; use the qualified path (home first, then "
                 f"arm_experiment: `python3 tools/run_nav_scenario.py <scenario> --arm-pose {pose}`) or B5 "
                 "(`drive --controller wholebody`, whose comfort pose it is). `arm` takes "
                 + " or ".join(MOVEIT_ARM_POSES))
    if running(runner, WHOLEBODY_MPC):
        sys.exit("The whole-body MPC (B5) owns the arm; `arm` is for the other controllers. "
                 "Restart `drive` with another controller to reconfigure the arm with MoveIt")
    if not running(runner, ["[r]econfigure_panel_server"]):
        sys.exit("MoveIt is not running; start `drive` with footprint mode profiles or dynamic")
    result = runner.ros(f"ros2 run mobile_manipulator_manipulation reconfigure_panel --named {pose} "
                        f"--profile {pose} --planning-time 10", timeout=180, check=False)
    try:
        report = json.loads(result.stdout[result.stdout.index("{"):])
    except ValueError:
        print(f"reconfigure_panel exited {result.returncode}:\n{result.stdout[-800:]}{result.stderr[-800:]}")
        return
    print(f"{report['error_code']}: {report['message']} (planning {report.get('planning_time_s')} s, "
          f"motion {report.get('execution_time_s')} s, footprint {report.get('applied_footprint_profile')})")


def stop(runner):
    runner.stop("nav", NAV_PROCESSES)
    runner.stop("moveit", MOVEIT_PROCESSES)
    print("Nav2 and MoveIt stopped")


def down(runner):
    stop(runner)
    runner.stop("rviz", RVIZ)
    runner.stop("arm", ARM_PROCESSES)
    if runner.playing():
        runner.unity("editor_stop")
    runner.stop("endpoint", ENDPOINT)
    runner.stop("description", DESCRIPTION)
    print("Everything stopped, Unity out of Play")


def choose(prompt, options, default=None):
    for number, option in enumerate(options, 1):
        print(f"  {number:2}) {option}")
    while True:
        answer = input(f"{prompt}{f' [{default}]' if default else ''}: ").strip()
        if not answer and default:
            return default
        if answer.isdigit() and 1 <= int(answer) <= len(options):
            return options[int(answer) - 1].split()[0]
        if answer.split(" ")[0] in [option.split()[0] for option in options]:
            return answer


def menu(runner):
    table = None
    actions = ["up        start the simulation", "rviz      open RViz in noVNC",
               "run       run a scenario", "drive     free drive (2D Goal Pose)",
               "arm       reconfigure the arm (free drive)", "results   newest run results",
               "stop      stop Nav2 and MoveIt", "down      stop everything", "quit"]
    while True:
        print("\n== Mobile manipulator demo")
        action = choose("Action", actions)
        try:
            if action == "quit":
                return
            if action == "up":
                up(runner)
            elif action == "rviz":
                rviz(runner)
            elif action == "run":
                table = table or scenarios(runner)
                name = choose("Scenario", [f"{name:38} {task:16} {description}"
                                           for name, (task, description) in table.items()])
                if table[name][0] == "compute_path":
                    controller, mode = "rpp", None
                else:
                    controller = choose("Controller", list(CONTROLLERS), "rpp")
                    modes = ["default"] + [m for m in FOOTPRINT_MODES
                                           if m != "profiles" or table[name][0] == "mission"]
                    mode = choose("Footprint mode", modes, "default")
                    mode = None if mode == "default" else mode
                run(runner, name, controller, mode)
            elif action == "drive":
                controller = choose("Controller", list(CONTROLLERS), "rpp")
                drive(runner, controller, None if controller == "wholebody"
                      else choose("Footprint mode", list(FOOTPRINT_MODES), "dynamic"))
            elif action == "arm":
                arm(runner, choose("Pose", list(MOVEIT_ARM_POSES)))
            elif action == "results":
                results(newest_runs(5))
            elif action == "stop":
                stop(runner)
            elif action == "down":
                down(runner)
        except (RuntimeError, subprocess.TimeoutExpired) as error:
            print(f"!! {error}")
        except SystemExit as error:
            print(f"!! {error}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", nargs="?", choices=("up", "rviz", "run", "drive", "arm", "results", "stop", "down"))
    parser.add_argument("target", nargs="?", help="scenario (run), pose (arm) or run count (results)")
    parser.add_argument("--container", default="mm-motion-planning-ma-robot-sim-1")
    parser.add_argument("--controller", choices=CONTROLLERS, default="rpp")
    parser.add_argument("--footprint-mode", choices=FOOTPRINT_MODES, default=None,
                        help="run: the runner's default per task; drive: dynamic (wholebody with B5)")
    parser.add_argument("--no-slowdown-zone", dest="slowdown_zone", action="store_false",
                        help="drive: disable the collision monitor's slowdown zone (stop zone and "
                             "FootprintApproach stay)")
    parser.add_argument("--posture-mode", choices=("blend", "comfort_only", "none"), default=None,
                        help="drive --controller wholebody: posture preference (default wholebody_mpc.yaml's)")
    parser.add_argument("--comfort-pose", choices=("home", "carry"), default=None,
                        help="drive --controller wholebody: comfort pose (default wholebody_mpc.yaml's)")
    parser.add_argument("--clearance-weight", type=float, default=None,
                        help="drive --controller wholebody: clearance cost weight (default wholebody_mpc.yaml's, "
                             "0 = off)")
    parser.add_argument("--no-slack-gate", dest="slack_gate", action="store_const", const=False, default=None,
                        help="drive --controller wholebody: execute every plan, also one that pushes into an "
                             "obstacle (--comfort-pose home --no-slack-gate is the recorded B5)")
    args = parser.parse_args()

    keep_host_awake()
    runner = Runner(args.container)
    if args.command is None:
        menu(runner)
    elif args.command == "up":
        up(runner)
    elif args.command == "rviz":
        rviz(runner)
    elif args.command == "run":
        if not args.target:
            parser.error("run needs a scenario name")
        run(runner, args.target, args.controller, args.footprint_mode)
    elif args.command == "drive":
        try:
            options = mpc_overrides(args, args.controller == "wholebody")
        except RuntimeError as error:
            parser.error(str(error))
        drive(runner, args.controller, args.footprint_mode, args.slowdown_zone, options)
    elif args.command == "arm":
        if args.target not in ARM_POSES:
            parser.error(f"arm needs one of {', '.join(ARM_POSES)}")
        arm(runner, args.target)
    elif args.command == "results":
        results(newest_runs(int(args.target or 5)))
    elif args.command == "stop":
        stop(runner)
    elif args.command == "down":
        down(runner)


if __name__ == "__main__":
    main()
