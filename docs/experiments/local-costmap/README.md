# Local costmap qualification (filtered Livox)

Date: 2026-09-28. Robot stationary at the open fixture (ROS `(12, 0)`, heading pi, facing
-x), fresh epoch, `local_costmap.launch.py` running the `livox_robot_filter` node and a
standalone `nav2_costmap_2d` with `config/nav2_local_costmap.yaml` (6 x 6 m rolling
window, 0.05 m, inflation 1.0 m). The trials below were run first with a VoxelLayer
(16 x 0.125 m) and then with the Spatio-Temporal Voxel Layer that replaced it.

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
blockages behind moving obstacles (roadmap scenarios 3 and 10).

## Spatio-Temporal Voxel Layer ([obstacles-home-stvl.json](obstacles-home-stvl.json))

Decision (2026-09-28): replace the VoxelLayer with the Spatio-Temporal Voxel Layer
(`ros-jazzy-spatio-temporal-voxel-layer` 2.5.5, added to the Dev Container image).
STVL does not ray-trace; voxels decay linearly over 10 s, and decay is accelerated
(5 1/s^2, about 2 s) for voxels inside a clearing frustum that models what the sensor
currently sees: full azimuth, a symmetric +/-7.2 deg band (the Mid-360's lower limit),
1.5-6.0 m. Voxels nearer than 1.5 m, i.e. the blind zone for low obstacles, decay only
by time, so an obstacle the robot has approached out of view is remembered for 10 s,
longer than the ~4 s needed to reach it at the qualified 0.3 m/s.

Same trials, `home` profile, STVL:

| Trial | Mark latency (sim s) | Lethal cells present | After 6 s | After 11 s |
|---|---:|---:|---:|---:|
| box50-0 (9.5, 0.0) | 0.30 | 27 | 0 | 0 |
| box50-1 (9.5, 1.2) | 0.40 | 45 | 6 | 0 |
| box50-2 (9.5, -1.2) | 0.42 | 42 | 8 | 0 |
| low20-far (9.2, 0.6) | 0.58 | 73 | 0 | 0 |
| low20-near (10.8, -0.6) | not marked | 0 | 0 | 0 |

Self-marking snapshots with STVL: 0 lethal and 0 non-zero cells. Every removed obstacle
cleared within 11 s; cells in view clear in about 2-6 s and the rest, just below the
frustum, expire with the 10 s decay. Mark latency is measured on the published costmap
(5 Hz), which adds up to 0.2 s that the controller, reading the costmap in-process at
10 Hz, does not see. Trade-off: an obstacle that stays out of view longer than 10 s is
forgotten; that is only possible in the near-field blind zone and is recorded for the
dynamic-obstacle scenarios.
