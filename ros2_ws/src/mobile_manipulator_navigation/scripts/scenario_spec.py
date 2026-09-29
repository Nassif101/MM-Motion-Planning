#!/usr/bin/env python3
"""Resolve a navigation scenario and check that its start and goal poses are free in the static map.

Prints one JSON object: the scenario, its footprint polygon, and the start check. Exits
non-zero when the scenario is unknown or inconsistent, or any map cell under the
footprint (plus margin) at the start pose is not free. Scenario obstacles (unmapped boxes
placed in Play) must lie in free map space and clear of the start and goal footprints.
Pure file access; no ROS graph.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if not (PACKAGE_ROOT / "config" / "scenarios.yaml").is_file():  # installed copy
    from ament_index_python.packages import get_package_share_directory
    PACKAGE_ROOT = Path(get_package_share_directory("mobile_manipulator_navigation"))


def load(name):
    return yaml.safe_load((PACKAGE_ROOT / "config" / name).read_text(encoding="utf-8"))


def read_map():
    meta = load("../maps/construction_site.yaml")
    with (PACKAGE_ROOT / "maps" / meta["image"]).open("rb") as stream:
        assert stream.readline().strip() == b"P5"
        line = stream.readline()
        while line.startswith(b"#"):
            line = stream.readline()
        width, height = (int(v) for v in line.split())
        stream.readline()
        pixels = stream.read()
    return meta, width, height, pixels


def start_is_free(polygon, pose, margin=0.05):
    """True when every map cell whose centre lies under the posed, enlarged footprint is free."""
    meta, width, height, pixels = read_map()
    resolution = meta["resolution"]
    origin_x, origin_y = meta["origin"][0], meta["origin"][1]
    x_min = min(p[0] for p in polygon) - margin
    x_max = max(p[0] for p in polygon) + margin
    y_min = min(p[1] for p in polygon) - margin
    y_max = max(p[1] for p in polygon) + margin
    cos_yaw, sin_yaw = math.cos(pose[2]), math.sin(pose[2])
    reach = math.hypot(max(abs(x_min), abs(x_max)), max(abs(y_min), abs(y_max)))
    blocked = []
    col0 = int((pose[0] - reach - origin_x) / resolution)
    row0 = int((pose[1] - reach - origin_y) / resolution)
    span = int(2 * reach / resolution) + 2
    for col in range(col0, col0 + span):
        for row in range(row0, row0 + span):
            cx = origin_x + (col + 0.5) * resolution - pose[0]
            cy = origin_y + (row + 0.5) * resolution - pose[1]
            # Cell centre in the robot frame.
            bx, by = cos_yaw * cx + sin_yaw * cy, -sin_yaw * cx + cos_yaw * cy
            if not (x_min <= bx <= x_max and y_min <= by <= y_max):
                continue
            if not (0 <= col < width and 0 <= row < height):
                blocked.append((col, row, "outside map"))
                continue
            value = pixels[(height - 1 - row) * width + col]
            if value != 254:  # trinary map: 254 free, 0 occupied, 205 unknown
                blocked.append((round(cx + pose[0], 3), round(cy + pose[1], 3), value))
    return not blocked, blocked[:10]


OBSTACLE_KEYS = {"name", "x", "y", "size_x", "size_y", "height"}


def box_points(obstacle, spacing=0.05):
    """Points covering an axis-aligned obstacle box (ROS map frame), boundary and interior."""
    nx = max(2, int(round(obstacle["size_x"] / spacing)) + 1)
    ny = max(2, int(round(obstacle["size_y"] / spacing)) + 1)
    x0 = obstacle["x"] - obstacle["size_x"] / 2
    y0 = obstacle["y"] - obstacle["size_y"] / 2
    return [(x0 + i * obstacle["size_x"] / (nx - 1), y0 + j * obstacle["size_y"] / (ny - 1))
            for i in range(nx) for j in range(ny)]


def obstacle_problems(scenario, polygon, clearance=0.3):
    """Why the scenario's obstacles are invalid (empty when they are fine)."""
    meta, width, height, pixels = read_map()
    resolution, (origin_x, origin_y) = meta["resolution"], meta["origin"][:2]
    problems = []
    for obstacle in scenario.get("obstacles", []):
        if set(obstacle) != OBSTACLE_KEYS:
            problems.append(f"obstacle needs exactly {sorted(OBSTACLE_KEYS)}: {obstacle}")
            continue
        label = obstacle["name"]
        if not (0 < obstacle["size_x"] <= 5 and 0 < obstacle["size_y"] <= 5 and 0 < obstacle["height"] <= 3):
            problems.append(f"{label}: sizes must be in (0, 5] m and height in (0, 3] m")
        for x, y in box_points(obstacle):
            col, row = int((x - origin_x) / resolution), int((y - origin_y) / resolution)
            if not (0 <= col < width and 0 <= row < height):
                problems.append(f"{label}: outside the map")
                break
            if pixels[(height - 1 - row) * width + col] != 254:
                problems.append(f"{label}: overlaps mapped obstacle or unknown space at ({x:.2f}, {y:.2f})")
                break
        for which in ("start", "goal"):
            pose = scenario[which]
            c, s_ = math.cos(pose[2]), math.sin(pose[2])
            x_min = min(p[0] for p in polygon) - clearance
            x_max = max(p[0] for p in polygon) + clearance
            y_min = min(p[1] for p in polygon) - clearance
            y_max = max(p[1] for p in polygon) + clearance
            for x, y in box_points(obstacle):
                dx, dy = x - pose[0], y - pose[1]
                bx, by = c * dx + s_ * dy, -s_ * dx + c * dy
                if x_min <= bx <= x_max and y_min <= by <= y_max:
                    problems.append(f"{label}: within {clearance} m of the {which} footprint")
                    break
    return problems


def resolve(name):
    scenarios = load("scenarios.yaml")["scenarios"]
    if name not in scenarios:
        raise KeyError(f"Unknown scenario '{name}'; expected one of {sorted(scenarios)}")
    scenario = dict(scenarios[name], name=name)
    profiles = load("footprint_profiles.yaml")["profiles"]
    profile = profiles[scenario["footprint_profile"]]
    if profile["arm_pose"] != scenario["arm_pose"]:
        raise ValueError(f"{name}: footprint profile {scenario['footprint_profile']} "
                         f"describes arm pose {profile['arm_pose']}, not {scenario['arm_pose']}")
    free, blocked = start_is_free(profile["polygon"], scenario["start"])
    goal_free, goal_blocked = start_is_free(profile["polygon"], scenario["goal"])
    if scenario["task"] == "navigate_to_pose" and not scenario.get("timeout_s"):
        raise ValueError(f"{name}: navigate_to_pose needs timeout_s")
    problems = obstacle_problems(scenario, profile["polygon"])
    if problems:
        raise ValueError(f"{name}: " + "; ".join(problems))
    return {"scenario": scenario, "polygon": profile["polygon"],
            "start_free": free and goal_free, "blocked_cells": blocked + goal_blocked}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", nargs="?")
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()
    if args.list:
        print(json.dumps(sorted(load("scenarios.yaml")["scenarios"])))
        return
    try:
        resolved = resolve(args.scenario)
    except (KeyError, ValueError) as error:
        print(json.dumps({"error": str(error)}))
        sys.exit(2)
    print(json.dumps(resolved))
    sys.exit(0 if resolved["start_free"] else 1)


if __name__ == "__main__":
    main()
