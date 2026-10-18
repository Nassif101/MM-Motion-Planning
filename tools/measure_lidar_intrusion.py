#!/usr/bin/env python3
"""Measure how far noisy lidar points reach into an obstacle, as the collision monitor's StopZone counts them.

Phase 4B-A spec Section 2: B5's base margin is the wholebody StopZone margin plus q, the lidar intrusion. The
robot is parked in the V1 open test area (carry pose) with a tall wall in front, a tall wall beside it and a
door jamb in front, each at base-hull distances 0.05, 0.08, 0.11, 0.14 and 0.20 m. For 60 s per position
tools/lidar_intrusion_collect.py (run in the container) reads /livox/points_filtered in base_footprint and
reports, per scan, the k-th deepest point inside the obstacle's true face (k = the StopZone's min_points, 4)
among points in the 0.05-0.60 m band in front of the face patch the StopZone polygon covers. q is the 99.9th
percentile of those values, per position, rounded up to the centimetre; q is the largest over the positions whose
fixture does not touch the carry panel. Fixtures are 1.0 m tall, under the carry panel (2 m ones touched it and
pushed the robot back, which hid part of the intrusion).

Requires the Unity Editor (any scene; the V1 scene is opened) and the description stack; stops Nav2 and runs
livox_robot_filter itself. Simulation only.

Usage: python3 tools/measure_lidar_intrusion.py [--output FILE] [--seconds S]
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE_HALF_X, BASE_HALF_Y = 0.46, 0.385  # footprint_profiles.yaml base_only
STOP_MARGIN = 0.02                      # nav2_navigation.yaml StopZone.wholebody_margin_m
MIN_POINTS = 4                          # StopZone min_points
DISTANCES = (0.05, 0.08, 0.11, 0.14, 0.20)
START = (12.0, 0.0, 0.0)                # V1 open test area (open_space_nav's start, facing +x)
FILTER_PROCESS = "[l]ivox_robot_filter"


WINDOW = 0.25  # m in front of a face: farther points belong to other obstacles (the robot is self-filtered)


def kth_deepest_intrusion(points_xy, face_x, k):
    """The k-th largest intrusion face_x - x over points within WINDOW in front of the face; 0 with fewer."""
    depths = sorted((face_x - x for x, _ in points_xy if face_x - WINDOW < x < face_x), reverse=True)
    return depths[k - 1] if len(depths) >= k else 0.0


def q_from(per_scan):
    """The 99.9th percentile (linear interpolation) of the per-scan values, rounded up to 0.01 m."""
    values = sorted(per_scan)
    if not values:
        return 0.0
    rank = 0.999 * (len(values) - 1)
    low = math.floor(rank)
    high = min(low + 1, len(values) - 1)
    value = values[low] + (values[high] - values[low]) * (rank - low)
    return math.ceil(round(value * 100, 9)) / 100


PANEL_X, PANEL_HALF_Y, PANEL_BOTTOM = (-0.68, 0.52), 0.60, 1.27  # carry: footprint less allowance, Unity bottom
FIXTURE_HEIGHT = 1.0  # above the StopZone band (0.60 m) and below the carry panel: no contact at any distance


def overlaps_carry_panel(fixture, distance, height):
    """True when a fixture this tall at this hull distance would touch the carry panel (an invalid row)."""
    if height < PANEL_BOTTOM:
        return False
    face = (BASE_HALF_Y if fixture == "side_wall" else BASE_HALF_X) + distance
    return face < (PANEL_HALF_Y if fixture == "side_wall" else PANEL_X[1])


def q_from_rows(rows):
    """q: the largest per-row 99.9th percentile over the rows whose fixture does not touch the panel."""
    valid = [r["p999"] for r in rows if not overlaps_carry_panel(r["fixture"], r["hull_distance_m"], r["height"])]
    return max(valid, default=0.0)


def fixtures(distance):
    """(label, obstacle boxes in the map, face description for the collector) for one hull distance."""
    x0, y0, _ = START
    front = x0 + BASE_HALF_X + distance
    side = y0 + BASE_HALF_Y + distance
    return [
        ("front_wall", [dict(name="intrusion_front", x=front + 0.1, y=y0, size_x=0.2, size_y=2.0, height=FIXTURE_HEIGHT)],
         dict(axis="x", sign=1, face=BASE_HALF_X + distance, lateral=BASE_HALF_Y + STOP_MARGIN)),
        ("side_wall", [dict(name="intrusion_side", x=x0, y=side + 0.1, size_x=2.0, size_y=0.2, height=FIXTURE_HEIGHT)],
         dict(axis="y", sign=1, face=BASE_HALF_Y + distance, lateral=BASE_HALF_X + STOP_MARGIN)),
        ("door_jamb", [dict(name="intrusion_jamb", x=front + 0.1, y=y0 + 0.2, size_x=0.2, size_y=0.1, height=FIXTURE_HEIGHT)],
         dict(axis="x", sign=1, face=BASE_HALF_X + distance, lateral=BASE_HALF_Y + STOP_MARGIN)),
    ]


def main():
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from run_nav_scenario import ARM_PROCESSES, NAV_PROCESSES, Runner  # noqa: F401  (host only)

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", default=str(ROOT / "experiment_runs/lidar-intrusion.json"))
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--distances", type=float, nargs="+", default=list(DISTANCES))
    parser.add_argument("--container", default="mm-motion-planning-ma-robot-sim-1")
    args = parser.parse_args()

    runner = Runner(args.container)
    scene = json.loads((ROOT / "ros2_ws/src/mobile_manipulator_navigation/maps/construction_site.metadata.json")
                       .read_text())["sourceScene"]
    runner.stop("nav", NAV_PROCESSES)
    if not runner.playing():
        runner.new_epoch(scene=scene)
    else:
        runner.ensure_scene(scene, "construction_site")
    runner.unity("scenario_obstacle_clear")
    held = runner.qualified_pose()
    if held is None:
        held = runner.restore_qualified_pose_with_moveit(("home",))
    # The robot self-filter normally runs with navigation, which is stopped here.
    runner.stop("lidarfilter", [FILTER_PROCESS])
    runner.start("lidarfilter", "ros2 run mobile_manipulator_navigation livox_robot_filter "
                                "--ros-args -p use_sim_time:=true")
    x, y, yaw = START
    for attempt in range(3):  # the hold must be fresh at the call; a stream pause can make it stale
        try:
            runner.wait_for_hold()
            runner.unity("scenario_place", "--x", str(x), "--y", str(y), "--yaw", str(yaw), "--arm_pose", held)
            break
        except RuntimeError:
            if attempt == 2:
                raise
            time.sleep(2)
    runner.move_arm("carry")

    rows, per_scan = [], []
    for distance in args.distances:
        for label, boxes, face in fixtures(distance):
            runner.unity("scenario_obstacle_clear")
            for box in boxes:
                runner.unity("scenario_obstacle", *(a for key, value in box.items() for a in (f"--{key}", str(value))))
            result = runner.ros(
                "python3 /workspaces/mm-motion-planning/tools/lidar_intrusion_collect.py "
                f"--seconds {args.seconds} --axis {face['axis']} --face {face['face']} "
                f"--lateral {face['lateral']} --k {MIN_POINTS}", timeout=args.seconds + 60)
            values = json.loads(result.stdout.strip().splitlines()[-1])["per_scan"]
            if not values:
                raise RuntimeError(f"no /livox/points_filtered scan in base_footprint for {label} at {distance} m")
            per_scan.extend(values)
            ordered = sorted(values)
            rows.append(dict(fixture=label, hull_distance_m=distance, height=boxes[0]["height"], scans=len(values),
                             p50=ordered[len(ordered) // 2] if ordered else 0.0,
                             p999=q_from(values), max=ordered[-1] if ordered else 0.0))
            print(json.dumps(rows[-1]), flush=True)
    runner.unity("scenario_obstacle_clear")
    runner.stop("lidarfilter", [FILTER_PROCESS])
    q = q_from_rows(rows)
    report = dict(q_m=q, base_margin_m=round(STOP_MARGIN + q, 3), q_pooled_m=q_from(per_scan), min_points=MIN_POINTS, stop_margin_m=STOP_MARGIN, scans=len(per_scan), rows=rows)
    Path(args.output).write_text(json.dumps(report, indent=2) + "\n")
    print(f"q = {q:.2f} m over {len(per_scan)} scans; base_margin = {STOP_MARGIN + q:.2f} m")
    if q > 0.08:
        print("STOP: q above 0.08 m (spec Section 2 stop rule)")


if __name__ == "__main__":
    main()
