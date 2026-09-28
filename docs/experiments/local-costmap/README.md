# Local costmap qualification (filtered Livox, VoxelLayer)

Date: 2026-09-28. Robot stationary at the open fixture (ROS `(12, 0)`, heading pi, facing
-x), fresh epoch, `local_costmap.launch.py` running the `livox_robot_filter` node and a
standalone `nav2_costmap_2d` with `config/nav2_local_costmap.yaml` (6 x 6 m rolling
window, 0.05 m, VoxelLayer 16 x 0.125 m, inflation 1.0 m).

## Filter

`livox_robot_filter` drops zero-point misses and self-returns inside the active footprint
profile and republishes in `livox_frame`. It keeps ground and overhead returns: the
costmap applies its own 0.05-2.0 m marking band, and ground rays are the rays that clear
low voxels near the robot. An earlier revision that also dropped ground returns left every
removed obstacle marked, which exposed this dependency.

Processing (Python/numpy, 100-scan windows): p50 1.5-1.8 ms, p99 1.9-2.7 ms, max 3.0 ms;
31.8 % of points kept. This is far below the 20 ms threshold for moving it to C++.

## Self-marking

Snapshots of the stationary robot (three per profile, 2-5 s apart) with no obstacle in
the 6 x 6 m window: **0 lethal and 0 non-zero cells** for both `home` and
`vertical_carry`, each with the arm in the matching pose.

## Obstacles ([obstacles-home.json](obstacles-home.json), `home` profile)

`tools/local_costmap_obstacle_trials.py` places a box with `scenario_obstacle`, records
the simulated time until the first costmap update marks its centre, counts lethal cells
within the box plus 0.1 m, removes it, and counts again after 6 s.

| Trial | Box (x, y), size x height | Mark latency (sim s) | Lethal cells present | Lethal cells 6 s after removal |
|---|---|---:|---:|---:|
| box50-0 | (9.5, 0.0), 0.5 x 0.5 m | 0.30 | 32 | 21 |
| box50-1 | (9.5, 1.2), 0.5 x 0.5 m | 0.06 | 44 | 22 |
| box50-2 | (9.5, -1.2), 0.5 x 0.5 m | 0.04 | 42 | 25 |
| low20-far | (9.2, 0.6), 0.5 x 0.2 m | 0.16 | 37 | 0 |
| low20-near | (10.8, -0.6), 0.5 x 0.2 m | not marked | 0 | 0 |

The boxes are 2.3-2.6 m ahead of the sensor except `low20-near` (about 1.0 m).

## Findings

- **Marking:** obstacles in view are marked within 0.3 s of simulation time.
- **Blind zone confirmed:** a 0.2 m obstacle 1.0 m ahead is never seen, as predicted by
  the -7.2 deg lower field-of-view limit.
- **Ghosts after removal:** roughly 50-65 % of a 0.5 m obstacle's cells stay lethal after
  it is removed. Rays that clear a voxel must end on a real return behind it. In open
  directions the rays behind the obstacle are misses, which UnitySensors reports as zero
  points without direction, so only ground rays clear, and they pass only through the
  lowest voxel layers. Low obstacles (0.2 m) clear completely.

The ghosting is harmless for the static Phase 1 scenarios but would leave phantom
blockages behind moving obstacles (roadmap scenarios 3 and 10). Resolving it is an open
decision: time-decaying voxels (Spatio-Temporal Voxel Layer, available for Jazzy but not
installed) or encoding misses as max-range clearing rays in Unity at fixed message size.
