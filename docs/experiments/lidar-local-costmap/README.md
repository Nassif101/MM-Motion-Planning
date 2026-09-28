# Livox Mid-360 observations for the local costmap

Date: 2026-09-28. Stationary robot at the open fixture (ROS `(12, 0)`), arm in controlled
HOLD, one scan per pose. Read-only probe:

```bash
ros2 run mobile_manipulator_navigation lidar_self_return_probe.py --profile <profile> \
  --output docs/experiments/lidar-local-costmap/self-returns-<profile>.json
```

The simulated sensor is UnitySensors' `Mid-360` scan pattern: 20,000 points per scan at
10 Hz, 0.1-70 m range, elevation +52.2 deg to -7.2 deg, full azimuth, measurement origin
`livox_frame` 0.387 m above the ground and 0.24 m ahead of `base_footprint`.

## Results

| Pose / footprint profile | Misses (zero points) | Self-returns inside footprint | Self-return heights | Ground returns | Nearest ground return |
|---|---:|---:|---|---:|---:|
| `home` | 11,438 (57 %) | 2,130 (11 %) | 0.32-1.57 m | 1,665 | 3.03 m |
| `vertical_carry` | 10,891 (54 %) | 2,736 (14 %) | 0.32-1.42 m | 1,734 | 3.07 m |

The nearest non-robot return in both scans was the surrounding site geometry at 6.2 m.

## Findings that constrain the costmap

1. **Misses are zero points.** UnitySensors writes a miss or out-of-range return as
   `(0, 0, 0)` in `livox_frame` with zero intensity. A costmap that accepts them marks
   an obstacle at the sensor origin on every scan, and misses never provide max-range
   clearing rays.
2. **Self-returns every scan.** The chassis deck, arm column, and panel return 11-14 %
   of each scan, all inside the matching footprint polygon.
3. **Ground blind zone.** With a -7.2 deg lower limit at 0.387 m, the ground is seen only
   beyond about 3.0 m. An obstacle of height `h` below the sensor is seen only beyond
   `(0.387 - h) / tan(7.2 deg)` from the sensor: about 1.5 m for 0.2 m and 2.3 m for
   0.1 m. Low obstacles that the robot approaches drop out of view.

The resulting contract is recorded in the design ledger under
"Local-costmap perception contract".
