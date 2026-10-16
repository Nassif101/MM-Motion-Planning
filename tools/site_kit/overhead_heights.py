#!/usr/bin/env python3
"""Overhead member heights of the second construction site, from the robot's FK tops.

H1 (scaffold ledger) = carry top + 0.10 m and at least 0.10 m below the home and vertical-carry
tops. H3 (pipe bridge) = 2.40 m. H2 is dropped: no qualified pose is lower than carry.
Spec Section 4.

Run: python3 tools/site_kit/overhead_heights.py [--container NAME]
"""
import argparse
import json
import math
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
QUALIFIED = REPO / "ros2_ws/src/mobile_manipulator_control/config/qualified_payload.json"
KIT = Path(__file__).resolve().parent / "kit_v2.json"
NAMED = ("home", "vertical_carry", "carry")
CLEARANCE_M = 0.10
H3_BOTTOM_M = 2.40


class HeightConflict(Exception):
    """The poses admit no heights that satisfy the rules; the user decides."""


def _up(metres):
    """Round up to the millimetre, ignoring FK noise below 0.1 mm (1.32002 -> 1.320)."""
    return math.ceil(metres * 1000 - 0.1) / 1000


def overhead(tops, named=NAMED):
    """Bottom heights of H1 and H3 from `tops` (pose name -> highest z in base_footprint).

    `named` are the home, vertical-carry and carry pose names.
    """
    home, vertical, carry = (tops[name] for name in named)
    h1 = _up(carry + CLEARANCE_M)
    limit = min(home, vertical) - CLEARANCE_M
    if h1 > limit + 1e-9:
        summary = ", ".join(f"{name} {top:.3f}" for name, top in tops.items())
        raise HeightConflict(f"H1 needs {h1:.3f} m but must stay below {limit:.3f} m. Tops: {summary}")
    return {"h1_bottom_m": h1, "h3_bottom_m": H3_BOTTOM_M, "tops_m": dict(tops)}


def named_poses():
    poses = json.loads(QUALIFIED.read_text())["poses_rad"]
    return {name: poses[name] for name in NAMED}


def measure(poses, container):
    """FK tops of `poses` from robot_heights in the container."""
    script = ("cat > /tmp/robot_heights_poses.json && source \"$ROS_WS/install/setup.bash\" && "
              "ros2 run mobile_manipulator_navigation robot_heights --poses /tmp/robot_heights_poses.json")
    result = subprocess.run(["docker", "exec", "-i", container, "bash", "-c", script],
                            input=json.dumps(poses), text=True, capture_output=True)
    if result.returncode:
        raise RuntimeError(f"robot_heights failed ({result.returncode}):\n{result.stdout}\n{result.stderr}")
    return json.loads(result.stdout.strip().splitlines()[-1])


def write_overhead(result, path=KIT):
    kit = json.loads(path.read_text()) if path.exists() else {"version": 1, "blender": "5.2", "pieces": []}
    kit["overhead"] = {**result, "tops_m": [{"name": n, "top_m": round(t, 4)} for n, t in result["tops_m"].items()]}
    path.write_text(json.dumps(kit, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--container", default="mm-motion-planning-ma-robot-sim-1")
    args = parser.parse_args()
    tops = measure(named_poses(), args.container)
    for name, top in tops.items():
        print(f"{name:28s} {top:.4f} m")
    try:
        result = overhead(tops)
    except HeightConflict as conflict:
        raise SystemExit(f"HeightConflict: {conflict}")
    write_overhead(result)
    print(json.dumps({k: v for k, v in result.items() if k != "tops_m"}))


if __name__ == "__main__":
    main()
