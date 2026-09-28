#!/usr/bin/env python3
"""Read-only checks of /local_costmap/costmap for the perception contract.

snapshot: count lethal cells and report the nearest one to the robot.
wait:     wait until the cell at (x, y) in the costmap frame becomes lethal
          (--state lethal) or free (--state free) and print the simulated stamp of
          the first costmap update that shows it, for latency against a Unity event.
count:    count lethal (100) cells within --half metres of (x, y) in the next update.
"""
import argparse
import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

LETHAL = 99  # costmap_2d publishes inscribed (99) and lethal (100) as occupied


def grid_array(grid):
    return np.asarray(grid.data, dtype=np.int16).reshape(grid.info.height, grid.info.width)


def cell_value(grid, x, y):
    info = grid.info
    col = int((x - info.origin.position.x) / info.resolution)
    row = int((y - info.origin.position.y) / info.resolution)
    if not (0 <= col < info.width and 0 <= row < info.height):
        return None
    return int(grid.data[row * info.width + col])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["snapshot", "wait", "count"])
    parser.add_argument("--x", type=float)
    parser.add_argument("--y", type=float)
    parser.add_argument("--state", choices=["lethal", "free"], default="lethal")
    parser.add_argument("--half", type=float, default=0.35)
    parser.add_argument("--timeout", type=float, default=10.0, help="wall seconds")
    args = parser.parse_args()

    rclpy.init()
    node = Node("local_costmap_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
    buffer = Buffer()
    TransformListener(buffer, node)
    grids = []
    node.create_subscription(OccupancyGrid, "/local_costmap/costmap", grids.append, 10)
    deadline = time.monotonic() + args.timeout
    report = None

    while time.monotonic() < deadline and report is None:
        rclpy.spin_once(node, timeout_sec=0.05)
        if not grids:
            continue
        grid = grids[-1]
        stamp = grid.header.stamp.sec + grid.header.stamp.nanosec * 1e-9
        if args.mode == "count":
            data = grid_array(grid)
            rows, cols = np.nonzero(data == 100)
            res = grid.info.resolution
            xs = grid.info.origin.position.x + (cols + 0.5) * res
            ys = grid.info.origin.position.y + (rows + 0.5) * res
            near = (np.abs(xs - args.x) <= args.half) & (np.abs(ys - args.y) <= args.half)
            report = {"costmap_stamp": stamp, "lethal_cells": int(near.sum())}
            continue
        if args.mode == "wait":
            value = cell_value(grid, args.x, args.y)
            hit = value is not None and (value >= LETHAL if args.state == "lethal" else value < LETHAL)
            if hit:
                report = {"state": args.state, "cell_value": value, "costmap_stamp": stamp}
            grids.clear()
            continue
        if not buffer.can_transform(grid.header.frame_id, "base_footprint", Time()):
            continue
        robot = buffer.lookup_transform(grid.header.frame_id, "base_footprint", Time()).transform
        data = grid_array(grid)
        rows, cols = np.nonzero(data >= LETHAL)
        res = grid.info.resolution
        xs = grid.info.origin.position.x + (cols + 0.5) * res
        ys = grid.info.origin.position.y + (rows + 0.5) * res
        distances = np.hypot(xs - robot.translation.x, ys - robot.translation.y)
        report = {
            "costmap_stamp": stamp,
            "frame": grid.header.frame_id,
            "size_cells": [grid.info.width, grid.info.height],
            "lethal_cells": int(len(rows)),
            "nearest_lethal_m": round(float(distances.min()), 3) if len(rows) else None,
            "nonzero_cost_cells": int(np.count_nonzero(data > 0)),
        }

    rclpy.shutdown()
    if report is None:
        report = {"timeout": True, "mode": args.mode}
    print(json.dumps(report))
    raise SystemExit(0 if "timeout" not in report else 1)


if __name__ == "__main__":
    main()
