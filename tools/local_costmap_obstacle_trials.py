#!/usr/bin/env python3
"""Local-costmap obstacle trials: mark latency and residual cells after removal.

Host-side. Requires Unity in Play with the robot stopped at the open fixture
(`tools/run_nav_scenario.py --new-epoch open_space`), arm in the pose matching the
running `local_costmap.launch.py` footprint profile. Places and removes box obstacles
with `scenario_obstacle` / `scenario_obstacle_clear` ahead of the robot and reads the
costmap with `local_costmap_check.py`. Latencies are simulation time, from the Unity
placement tick to the stamp of the first costmap update showing the obstacle.
Usage: python3 tools/local_costmap_obstacle_trials.py [output.json]
"""
import json
import subprocess
import sys
import time

C = "mm-motion-planning-ma-robot-sim-1"


def unity(*args):
    out = subprocess.run(["unity", "command", *args, "--format", "json"], text=True, capture_output=True)
    envelope = json.loads(out.stdout)
    assert envelope["success"], out.stdout
    return envelope["data"]["result"]


def check(*args, background=False):
    command = ["docker", "exec", C, "bash", "-c",
               "source $ROS_WS/install/setup.bash; ros2 run mobile_manipulator_navigation "
               "local_costmap_check.py " + " ".join(args) + " 2>/dev/null"]
    process = subprocess.Popen(command, text=True, stdout=subprocess.PIPE)
    return process if background else last_json(process)


def last_json(process):
    text = process.communicate()[0]
    lines = [line for line in text.splitlines() if line.strip().startswith("{")]
    if not lines:
        raise RuntimeError("no JSON from checker: " + repr(text[-300:]))
    return json.loads(lines[-1])


def trial(name, x, y, size, height, timeout=8):
    half = str(size / 2 + 0.1)
    before = check("count", "--x", str(x), "--y", str(y), "--half", half)["lethal_cells"]
    waiter = check("wait", "--x", str(x), "--y", str(y), "--state", "lethal",
                   "--timeout", str(timeout), background=True)
    time.sleep(3.0)
    placed = unity("scenario_obstacle", "--name", name, "--x", str(x), "--y", str(y),
                   "--size_x", str(size), "--size_y", str(size), "--height", str(height))
    marked = last_json(waiter)
    result = {"name": name, "x": x, "y": y, "size": size, "height": height,
              "lethal_cells_before": before, "marked": "timeout" not in marked}
    if result["marked"]:
        result["mark_latency_s"] = round(marked["costmap_stamp"] - placed["physicsTime"], 3)
    result["lethal_cells_while_present"] = check(
        "count", "--x", str(x), "--y", str(y), "--half", half)["lethal_cells"]
    unity("scenario_obstacle_clear", "--name", name)
    time.sleep(6.0)
    result["lethal_cells_6s_after_removal"] = check(
        "count", "--x", str(x), "--y", str(y), "--half", half)["lethal_cells"]
    return result


if __name__ == "__main__":
    results = [trial(f"box50-{n}", 9.5, y, 0.5, 0.5) for n, y in enumerate((0.0, 1.2, -1.2))]
    results.append(trial("low20-far", 9.2, 0.6, 0.5, 0.2))
    results.append(trial("low20-near", 10.8, -0.6, 0.5, 0.2, timeout=6))
    for r in results:
        print(json.dumps(r))
    if len(sys.argv) > 1:
        open(sys.argv[1], "w").write(json.dumps(results, indent=2) + "\n")
