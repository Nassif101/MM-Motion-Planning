#!/usr/bin/env python3
"""Measure how far noisy lidar points reach in from obstacles, as the collision monitor's StopZone counts them.

Phase 4B-A spec Section 2: B5's base margin must keep the StopZone (the base hull + StopZone margin) from
tripping. The wholebody StopZone is the convex hull of the URDF base links, grown by dynamic_footprint.yaml
padding_m (0.02 m, giving the base_only hull 0.46 x 0.385 m half sizes) and by the StopZone's
wholebody_margin_m (0.02 m); it trips when at least min_points (4) points of /livox/points_filtered in the
0.05-0.60 m band lie inside it. So, per scan, tools/lidar_intrusion_collect.py (run in the container) takes the
4th-smallest distance of the band points from the padded hull; for fixtures whose faces all lie at hull
distance d the per-scan intrusion is d minus that distance. q is the 99.9th percentile of the intrusion per
position, rounded up to the centimetre, and the largest over the positions.

Fixtures (1.0 m tall: above the band and under the carry panel, which 2 m ones touched): a wall in front, a
wall to the left, a door jamb (0.1 m wide) ahead, walls on both sides (a corridor or door) and a wall in front
plus one to the left (a corner), at hull distances 0.08, 0.11 and 0.14 m. Each position checks the robot's
pose (TF map -> base_footprint) against the placement: a displaced robot aborts the run.

The MPC's base rows measure from the URDF base primitives, padding_m inside the hull, so B5's base margin is
padding_m + wholebody StopZone margin + q.

Requires the Unity Editor (any scene; the V1 scene is opened) and the description stack; stops Nav2 and runs
livox_robot_filter itself. Simulation only.

Usage: python3 tools/measure_lidar_intrusion.py [--output FILE] [--seconds S] [--distances D ...]
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HULL_HALF_X, HULL_HALF_Y = 0.46, 0.385  # base_only hull: URDF base links + padding_m (footprint_profiles.yaml)
PADDING = 0.02                          # dynamic_footprint.yaml padding_m
STOP_MARGIN = 0.02                      # nav2_navigation.yaml StopZone.wholebody_margin_m
MIN_POINTS = 4                          # StopZone min_points
DISTANCES = (0.08, 0.11, 0.14)
START = (12.0, 0.0, 0.0)                # V1 open test area (open_space_nav's start position), facing +x
FILTER_PROCESS = "[l]ivox_robot_filter"
FIXTURE_HEIGHT = 1.0
MAX_DRIFT_M, MAX_DRIFT_RAD = 0.005, math.radians(0.2)


def kth_deepest_intrusion(points_xy, face_x, k):
    """The k-th largest intrusion face_x - x over points within 0.25 m in front of the face; 0 with fewer."""
    depths = sorted((face_x - x for x, _ in points_xy if face_x - 0.25 < x < face_x), reverse=True)
    return depths[k - 1] if len(depths) >= k else 0.0


def kth_nearest_to_hull(points_xy, half_x, half_y, k):
    """The k-th smallest Euclidean distance of points from the axis-aligned hull |x| <= half_x, |y| <= half_y
    (0 inside); infinity with fewer than k points."""
    distances = sorted(math.hypot(max(abs(x) - half_x, 0.0), max(abs(y) - half_y, 0.0)) for x, y in points_xy)
    return distances[k - 1] if len(distances) >= k else math.inf


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


def fixtures(distance):
    """(label, obstacle boxes in the map) for one hull distance; every face lies at that hull distance."""
    x0, y0, _ = START
    front = x0 + HULL_HALF_X + distance
    left, right = y0 + HULL_HALF_Y + distance, y0 - HULL_HALF_Y - distance
    h = FIXTURE_HEIGHT
    front_wall = dict(name="intrusion_front", x=front + 0.1, y=y0, size_x=0.2, size_y=2.0, height=h)
    left_wall = dict(name="intrusion_left", x=x0, y=left + 0.1, size_x=2.0, size_y=0.2, height=h)
    right_wall = dict(name="intrusion_right", x=x0, y=right - 0.1, size_x=2.0, size_y=0.2, height=h)
    jamb = dict(name="intrusion_jamb", x=front + 0.1, y=y0 + 0.2, size_x=0.2, size_y=0.1, height=h)
    return [("front_wall", [front_wall]), ("side_wall", [left_wall]), ("door_jamb_ahead", [jamb]),
            ("both_sides", [left_wall, right_wall]), ("corner", [front_wall, left_wall])]


def drift(pose):
    """Distance and heading error of a measured base pose (x, y, yaw) from START."""
    x, y, yaw = pose
    return math.hypot(x - START[0], y - START[1]), abs(math.remainder(yaw - START[2], math.tau))


def main():
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from run_nav_scenario import NAV_PROCESSES, Runner  # host only

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
    rows, per_scan = [], []
    try:
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
        for distance in args.distances:
            for label, boxes in fixtures(distance):
                runner.unity("scenario_obstacle_clear")
                for box in boxes:
                    runner.unity("scenario_obstacle",
                                 *(a for key, value in box.items() for a in (f"--{key}", str(value))))
                result = runner.ros(
                    "python3 /workspaces/mm-motion-planning/tools/lidar_intrusion_collect.py "
                    f"--seconds {args.seconds} --half-x {HULL_HALF_X} --half-y {HULL_HALF_Y} --k {MIN_POINTS}",
                    timeout=args.seconds + 60)
                collected = json.loads(result.stdout.strip().splitlines()[-1])
                if collected["pose"] is None:
                    raise RuntimeError(f"no map -> base_footprint transform for {label} at {distance} m")
                off, turned = drift(collected["pose"])
                if off > MAX_DRIFT_M or turned > MAX_DRIFT_RAD:
                    raise RuntimeError(f"robot displaced for {label} at {distance} m: {off:.4f} m, "
                                       f"{math.degrees(turned):.2f} deg (pose {collected['pose']})")
                nearest = collected["per_scan"]
                if not nearest or sum(math.isfinite(v) for v in nearest) < 0.5 * len(nearest):
                    raise RuntimeError(f"too few scans with {MIN_POINTS} band points for {label} at {distance} m")
                values = [max(0.0, distance - v) if math.isfinite(v) else 0.0 for v in nearest]
                per_scan.extend(values)
                ordered = sorted(values)
                rows.append(dict(fixture=label, hull_distance_m=distance, height=FIXTURE_HEIGHT, scans=len(values),
                                 pose=collected["pose"], p50=ordered[len(ordered) // 2], p999=q_from(values),
                                 max=ordered[-1]))
                print(json.dumps(rows[-1]), flush=True)
    finally:
        runner.unity("scenario_obstacle_clear")
        runner.stop("lidarfilter", [FILTER_PROCESS])
    q = max(r["p999"] for r in rows)
    margin = round(PADDING + STOP_MARGIN + q, 3)
    report = dict(q_m=q, base_margin_from_urdf_m=margin, hull_reference="URDF base links + padding_m 0.02",
                  padding_m=PADDING, stop_margin_m=STOP_MARGIN, min_points=MIN_POINTS, scans=len(per_scan), rows=rows)
    Path(args.output).write_text(json.dumps(report, indent=2) + "\n")
    print(f"q = {q:.2f} m over {len(per_scan)} scans; base margin from the URDF primitives = {margin:.2f} m")
    if q > 0.08:
        print("STOP: q above 0.08 m (spec Section 2 stop rule)")


if __name__ == "__main__":
    main()
