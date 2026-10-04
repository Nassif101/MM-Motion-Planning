# Nav2 navigation (Phase 1)

## 2026-09-28 bring-up: command chain without goals

`ros2 launch mobile_manipulator_navigation navigation.launch.py footprint_profile:=home`
with Unity in Play (robot stopped at the open fixture, arm in controlled `home` HOLD).

- Lifecycle: `map_server`, `planner_server`, `controller_server`, `behavior_server`,
  `velocity_smoother`, `collision_monitor`, and `bt_navigator` all reached `active`.
- Command chain (`ros2 topic info -v`), all `geometry_msgs/msg/Twist`:
  `/cmd_vel_nav` published by `controller_server` and `behavior_server`, read by
  `velocity_smoother`; `/cmd_vel_smoothed` published by `velocity_smoother`, read by
  `collision_monitor`; `/cmd_vel` published only by `collision_monitor`, read only by
  Unity's endpoint. `check_cmd_vel_ownership.py --expect collision_monitor` passed 3/3.
- Collision-monitor zones for `home`: stop `[[0.59, 0.67], [0.59, -0.67], [-0.75, -0.67],
  [-0.75, 0.67]]` (footprint + 0.05 m), slowdown to 50 % at footprint + 0.30 m.
- Robot remained in HOLD with base speed 0.001 m/s; no goal was sent.

Findings during bring-up:

- The stock `navigate_through_poses` tree requires the Spin server, which is excluded
  until qualified; only `NavigateToPose` is loaded.
- The first ownership-check version used a fixed 2 s discovery window and reported no
  publisher on a ~20-node graph; it now waits until the topic's endpoints are stable.
- Local-costmap sensor staleness: the 0.3 s `expected_update_rate` was tighter than
  occasional 0.32-0.40 s lidar gaps and is now 0.5 s. One burst of gaps up to 2.4 s
  occurred about 2 minutes after launch while many CLI processes were being started;
  none occurred in the following 2 idle minutes, and the filter's processing time was
  unchanged (p50 1.7 ms). While the source is stale the costmap is not current and the
  controller stops following, which is the safe failure. Occurrences are counted per
  run in the Phase 1 navigation measurements.

## 2026-09-28 first autonomous runs (RPP bring-up, Lattice planner)

`python3 tools/run_nav_scenario.py [--new-epoch] [--runs N] <scenario>`; each run is
placed by teleport, starts only after the navigation preflight (placed pose within
0.10 m, base stationary on `/odom`, local costmap publishing, collision monitor the only
`/cmd_vel` publisher), and is recorded to a rosbag. Per-run reports are in `runs/`.

### open_space_nav: 5/5 succeeded (plus one earlier bring-up run)

| Metric | Range over 5 runs |
|---|---|
| Time to goal | 15.54-15.60 s (4 m route) |
| Executed path | 3.849-3.854 m |
| Final position / heading error | 0.143-0.148 m / 0.000-0.010 rad (tolerance 0.15 / 0.15) |
| Cross-track p95 to `/plan` | 0.033-0.034 m |
| Minimum footprint clearance to static map | 2.53 m |
| Recoveries / collision-monitor activations / lidar gaps > 0.5 s | 0 / 0 / 0 |

In the first run the navigation processes used 3-8 % of a core each and 40-90 MB; peak
`/cmd_vel` acceleration was 0.56 m/s^2 (p95 0.18). The robot stops as soon as it is inside
the 0.15 m goal tolerance, so the final error sits just under it.

One attempted run was refused by the preflight (no `/cmd_vel` publisher discovered yet by
the fresh task node); no goal was sent. The preflight now waits for stable discovery.

### narrow_gate_vertical_carry_nav: aborted with panel contact

The first plan passed the 1.05 m gate nearly straight (heading within 2.2 deg) but 0.05-0.10 m
off the gate centre. Replanning at 1 Hz from the drifting pose produced Lattice paths with
a 26.6 deg primitive bend just before the gate; RPP followed it and the robot entered the
throat yawed 17.8 deg, where the 1.24 x 0.77 m footprint projects about 1.11 m. The
collision monitor slowed the robot three times and stopped it once, the tree ran 11
recoveries and aborted after 28 s (`PATIENCE_EXCEEDED`). After the run the panel's
bounds overlapped both gate posts and the shoulder-pan joint had been pushed from pi/2 to
0.44 rad, so the qualified return to home failed; a new epoch was needed.

Safety finding: `livox_robot_filter` removes every point inside the footprint rectangle
as a self-return, so an obstacle that enters that rectangle becomes invisible to the
collision monitor, and a rotating 1.24 m footprint sweeps its corners at about 0.3 m/s,
crossing the 0.05 m stop band in under two lidar scans.

### narrow_gate_home_nav: aborted near the goal, no contact

The robot took the 16 m detour as planned (minimum footprint clearance 0.22 m) and
reached the goal area after about 40 s, then oscillated there for about 70 s: RPP
reported "collision ahead" 136 times while trying to turn the 1.24 m square footprint to
the goal heading beside the gate, each stall exceeded controller patience, and the tree
aborted after 11 recoveries (116 s, 0.80 m and 1.96 rad from the goal). The arm stayed in
home and the panel had no contact. Ten collision-monitor activations were all slowdowns.
The controller loop reported three 10 Hz iterations against its 20 Hz target.

## 2026-09-28 fix 1: self-filter and contact safety

Changes:

- **Geometric, ray-based self-filter.** `livox_robot_filter` no longer removes the whole
  footprint column. It poses the 12 URDF collision primitives and the 1.2 x 1.2 x 0.04 m
  panel from TF each scan. A point is a self-return when it lies inside a primitive
  enlarged by 0.03 m, or when its ray from the sensor first hits the robot and the point
  lies at most 0.08 m (4 sigma of the simulated lidar's 0.02 m Gaussian range noise) in
  front of that surface. Obstacles between the sensor and the robot, or off the robot's
  rays, stay visible even inside the footprint rectangle. The filter is independent of
  the footprint profile.
- **Collision-monitor footprint approach zone.** `FootprintApproach` projects the local
  costmap's published footprint along the current command, including rotation, 1.2 s
  ahead in 0.1 s steps, in addition to the stop (+0.05 m) and slowdown (+0.30 m) zones.
- **Contact monitor.** `RobotContactMonitor` (Unity, started by `scenario_contacts_reset`,
  read by `scenario_contacts`) computes penetration between every robot collider,
  including the panel, and any non-trigger collider except the ground on every physics
  tick. The runner resets it before each `navigate_to_pose` goal, stores the result in the
  run summary, and exits non-zero if any run had contact.

Evidence:

- A first version with only a 0.03 m volume margin left 60-200 points per scan of the arm
  pedestal in the stop zone: raycasts from the lidar showed the pedestal surface exactly
  where modelled, and the leftover points were up to 0.07 m short of it, i.e. range noise
  (sigma 0.02 m). The robot never moved (stop zone active from t = 0.04 s). With the ray
  rule: 0 points in the stop zone over 50 stationary scans.
- Filter cost with per-primitive bounding-sphere culling: p50 about 14 ms, p99 16-20 ms,
  max 26 ms per scan (Python/numpy); 33-39 % of points kept.
- Contact monitor positive test: a 0.1 m post placed through the panel's rear overhang
  was reported as `PayloadPanel` / `contact-probe`, 0.11 m penetration, first 0.14 s after
  placement; after removal and reset no contact was reported. The arm stayed in HOLD.
- `open_space_nav` after the change: succeeded in 15.50-15.56 s, no monitor activations, no
  contact.
- Safety rerun of `narrow_gate_vertical_carry_nav` (replanning unchanged): **no contact**
  over 1,974 physics ticks; minimum footprint clearance to the static map 0.22 m (0.088 m
  before). The robot still did not pass the gate: RPP's own collision check stopped it about
  1 m before the gate line, yawed 0.24 rad, and the tree aborted after 11 recoveries; only
  slowdown activations occurred, so the approach zone was not exercised in this run.

## 2026-09-28 fix 2: path quality through narrow gaps

Changes:

- **Behaviour tree.** The default tree (`behavior_tree:=replan_if_invalid`,
  `navigate_to_pose_replan_if_invalid_wait_clear.xml`) keeps the current path until the goal
  changes or the path becomes invalid in the global costmap, instead of replanning at 1 Hz
  from the drifting pose. The 1 Hz tree remains available as `behavior_tree:=replan_1hz`.
- **Lattice straightness.** `cost_penalty` 2.0 -> 1.0 and `non_straight_penalty` 1.05 -> 1.2.

Evidence:

- With only the tree changed, the vertical-carry run kept a single plan, but that plan still
  ran 0.10 m off centre and S-bent 0.14 m laterally with up to 16.8 deg heading right before
  the gate throat (asymmetric angled approach walls skew the inflation cost). RPP's
  collision check refused it 72 times and the run aborted before the gate (no contact).
- Read-only sweep of Lattice penalties for the gate query (cost 2.0/1.0/0.5 x non-straight
  1.05/1.2 x change 0.05/0.3): with cost 1.0, and with non-straight 1.2 for most other
  combinations, the path through the gate zone is straight and centred (0.000 m offset,
  0.0 deg heading deviation); the previous setting gave 0.10 m / 16.8 deg.
- Planner-only clearances with the new penalties are unchanged or better: open space 2.925 m,
  home detour 0.902 m, vertical-carry gate 0.55 m (0.507 m before).

Navigation results with both changes:

| Scenario | Result |
|---|---|
| `narrow_gate_home_nav` | **succeeded**: 44.8 s, 10.96 m detour, final error 0.124 m / 0.132 rad, cross-track p95 0.061 m, min clearance 0.19 m, 0 recoveries, 2 slowdowns, no contact (previously aborted at the goal after 116 s) |
| `narrow_gate_vertical_carry_nav` | aborted at the throat after 36 s: heading error 0.03 rad, cross-track p95 0.022 m, min clearance 0.10 m, 11 recoveries, no contact |

The remaining vertical-carry failure is perception-limited, not a path problem: in the last
local costmap before abort, the free opening at the gate was -8.15 to -7.30 m, i.e. 0.85 m
against the true 1.05 m. The simulated Livox range noise (sigma 0.02 m, matching the
Mid-360's specified accuracy) plus 0.05 m cells place post returns up to about 0.1 m inside
the opening on each side, leaving 2-3 cm per side for the 0.79 m padded footprint, so RPP's
collision check stops. The geometric margin (0.14 m per side) is below what this
perception pipeline resolves.

## 2026-09-28 1.30 m comparison gate

Decision (roadmap open-decision log): the 1.05 m gate result with standard Nav2 is kept as a
baseline finding (perception-limited, see above), and a comparison gate
`WideGate_1p30m` was added. It copies the 1.05 m gate's design (0.4 m posts, 65 deg approach
walls, orientation, 3.0 m approach and 1.8 m exit) with a 1.30 m opening, centred at ROS
(-3, -16) in open ground: 0.265 m per side for the 0.77 m vertical-carry footprint and
0.03 m per side for the 1.24 m home footprint. It was added with the new `add_wide_gate`
Unity command (additive scene change only) and the map re-exported (28 colliders, 51,438
occupied cells).

| Scenario | NavFn plan | Lattice plan | Navigation (RPP, Lattice, replan-if-invalid) |
|---|---|---|---|
| `wide_gate_vertical_carry` | through the gate, 4.97 m, 0.64 m clearance | through, 4.80 m, 0.68 m | succeeded: 26.4 s, 4.65 m, final 0.143 m / 0.000 rad, cross-track p95 0.033 m, min clearance 0.29 m, 0 recoveries, no contact |
| `wide_gate_home` | **through the gate**, 5.00 m, 0.65 m | detour, 11.50 m, 0.99 m | succeeded via the detour: 47.5 s, 11.31 m, final 0.128 m / 0.128 rad, min clearance 0.28 m, 0 recoveries, no contact |

NavFn plans the 1.24 m home footprint through the 1.30 m opening because it checks only
the inscribed radius; Lattice checks the full footprint and detours. Together with the
1.05 m gate, the two gates bracket the perception-limited margin for the Phase 1 stack:
0.265 m per side passes, 0.14 m per side does not.

## 2026-09-28 cross-track metric correction

Run reports before this date measured cross-track error as the distance to the nearest
discrete `/plan` pose, which adds up to half the pose spacing (about 0.025 m for 0.05 m
Lattice poses). `navigate_scenario_task.py` now measures the distance to the path polyline,
as the telemetry window does. The same `open_space_nav` run type gives cross-track p95
0.0022 m with the polyline metric against 0.033 m with the point metric; treat the earlier
cross-track values in this document and in `runs/` as upper bounds.

## 2026-09-28 controller baselines: RPP, DWB (B1), and MPPI (B2)

`navigation.launch.py controller:=rpp|dwb|mppi` loads one block of
`config/nav2_controllers.yaml` as the controller server's `FollowPath` plugin. Everything
else is shared: Lattice planner, the replan-if-invalid tree, the STVL local costmap, the
goal checker (0.15 m / 0.15 rad), the progress checker (0.3 m in 10 s), the velocity
smoother, and the collision monitor.

Configuration: stock Nav2 Jazzy (1.3.12) values except where this robot needs otherwise.

- All three are forward-only and at or below the envelope's 0.3 m/s and 0.4 rad/s.
- Full-footprint collision checking: RPP `use_collision_detection`; DWB `ObstacleFootprint`
  instead of the stock `BaseObstacle` (which scores only the cell under the base origin);
  MPPI `CostCritic.consider_footprint`.
- DWB: `xy_goal_tolerance` 0.15 to match the goal checker; accelerations at the envelope;
  20 x 20 samples over `sim_time` 1.7 s (0.51 m at 0.3 m/s).
- MPPI: `DiffDrive`, 2000 samples x 56 steps x 0.05 s (2.8 s, 0.84 m at 0.3 m/s); braking
  model at the envelope (0.5 m/s^2), forward and yaw acceleration models stock (below).
- The runner refuses a run if any configured controller parameter is not declared by the
  running controller server (`check_controller_params.py`): ROS 2 would otherwise ignore a
  misspelled key silently.

### MPPI did not start from rest with the envelope's accelerations

With `ax_max: 0.45`, the first MPPI `open_space_nav` run held a steady 0.014 m/s command,
the base moved 0.018 m in 60 s, and the tree gave up after 6 recoveries ("Failed to make
progress"). Jazzy MPPI starts every rollout from the measured velocity and limits each
0.05 s step by its acceleration model, so from rest it can ask for at most 0.045 m/s, and
its weighted sample average was lower still. The base does not break away from rest below
0.0375 m/s (base-controller README, 2026-09-28 low-speed breakaway), so the measured
velocity stayed at zero. RPP and DWB are not affected: RPP commands its target speed
directly, and DWB's standard trajectory generator samples every command reachable within
`sim_time`.

The same limit applies to turning. With `az_max: 0.8` the first yaw-rate command from rest
is at most 0.08 rad/s, against a 0.07 rad/s breakaway. On `narrow_gate_home_nav`, whose
detour starts with a 1-1.5 rad turn (RPP and DWB turn within 2-3 s), MPPI sat still for
12 s, then drove in circles, left the footprint's static clearance at 0.0 m (no physical
contact), and aborted after 11 recoveries.

MPPI therefore keeps the stock acceleration models in its rollouts (`ax_max: 3.0`,
`az_max: 3.5`); its braking model (`ax_min: -0.5`) stays at the envelope, and the velocity
smoother still limits every controller's commands (see "Command limits" below).

### Bring-up race

One comparison run never started: the global-planning lifecycle manager sent
`change_state` (configure) to `map_server`, the reply was lost during DDS discovery
("failed to send response ... client will not receive response"), and Jazzy's lifecycle
manager, which has no service timeout, waited indefinitely; `bt_navigator` then could not
load its tree without `compute_path_to_pose`. A second launch failed once because the local
costmap's transform wait is timed on sim time, which can jump when `/clock` is discovered
late. The runner now relaunches the stack once when it does not come up and records
`nav_launch_attempts`; no navigation result depends on it.

### Comparison runs

Every `navigate_to_pose` scenario ran 3 times with each controller: 45 runs, none with
robot-environment contact. Each of three rounds started a new
simulation epoch and rotated the controller order (RPP-DWB-MPPI, DWB-MPPI-RPP,
MPPI-RPP-DWB). 40 runs are from commit `06fb7fd`; 5 were rerun at `3248511` after run
infrastructure faults (below), with the navigation configuration unchanged. Per-run
summaries are `runs/*-{rpp,dwb,mppi}-summary.json`; the table is
`python3 tools/summarize_nav_runs.py docs/experiments/nav2-navigation/runs/*-{rpp,dwb,mppi}-summary.json`.

| Scenario | Controller | Success | Contact | Time s | Path m | Final error m | Cross-track p95 m | Min clearance m | Recoveries | Monitor stop/slow/appr | Controller CPU % | Loop misses | Controller errors |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| narrow_gate_home_nav | rpp | 3/3 | 0 | 46.3 (43.8-47.6) | 11.00 (10.91-11.02) | 0.133 (0.130-0.138) | 0.085 (0.052-0.092) | 0.22 (0.19-0.23) | 0 | 0/5/0 | 9.1 (8.9-9.4) | 0 | 0 |
| narrow_gate_home_nav | dwb | 3/3 | 0 | 45.4 (45.4-45.5) | 10.97 (10.92-10.99) | 0.122 (0.119-0.137) | 0.090 (0.073-0.093) | 0.18 (0.18-0.19) | 0 | 0/7/0 | 27.1 (27.0-27.6) | 0 | 0 |
| narrow_gate_home_nav | mppi | 3/3 | 0 | 43.2 (41.8-43.7) | 11.39 (11.16-11.41) | 0.435 (0.293-0.500) | 0.255 (0.173-0.330) | 0.10 (0.07-0.12) | 0 | 0/8/0 | 27.9 (26.6-29.4) | 1 | 0 |
| narrow_gate_vertical_carry_nav | rpp | 2/3 | 0 | 26.8 (26.7-26.8) | 4.65 (4.65-4.66) | 0.145 (0.144-0.146) | 0.005 (0.004-0.011) | 0.16 (0.10-0.16) | 0 (0-11) | 2/4/0 | 9.2 (8.9-9.4) | 4 | 10 |
| narrow_gate_vertical_carry_nav | dwb | 2/3 | 0 | 34.0 (33.4-34.6) | 4.66 (4.66-4.66) | 0.143 (0.143-0.144) | 0.045 (0.009-0.050) | 0.14 (0.10-0.14) | 0 (0-10) | 15/24/0 | 24.8 (23.0-26.4) | 4 | 7 |
| narrow_gate_vertical_carry_nav | mppi | 1/3 | 0 | 27.0 | 4.66 | 0.133 | 0.016 (0.015-0.018) | 0.12 (0.12-0.15) | 9 (0-9) | 113/124/0 | 34.1 (30.1-40.6) | 12 | 14 |
| open_space_nav | rpp | 3/3 | 0 | 15.6 (15.6-15.6) | 3.85 (3.85-3.85) | 0.147 (0.146-0.147) | 0.002 (0.002-0.003) | 2.53 (2.53-2.53) | 0 | 0/0/0 | 7.9 (7.7-8.2) | 0 | 0 |
| open_space_nav | dwb | 3/3 | 0 | 15.8 (15.8-15.9) | 3.85 (3.85-3.86) | 0.147 (0.147-0.149) | 0.017 (0.015-0.022) | 2.49 (2.49-2.49) | 0 | 0/0/0 | 25.5 (22.6-25.7) | 1 | 0 |
| open_space_nav | mppi | 3/3 | 0 | 14.5 (14.4-14.5) | 3.86 (3.86-3.87) | 0.130 (0.129-0.134) | 0.024 (0.013-0.030) | 2.49 (2.46-2.50) | 0 | 0/0/0 | 24.3 (23.9-26.8) | 0 | 0 |
| wide_gate_home_nav | rpp | 3/3 | 0 | 45.6 (45.0-45.8) | 11.29 (11.27-11.31) | 0.115 (0.111-0.131) | 0.068 (0.046-0.070) | 0.29 (0.28-0.30) | 0 | 0/3/0 | 9.7 (9.6-10.4) | 0 | 0 |
| wide_gate_home_nav | dwb | 3/3 | 0 | 44.3 (43.8-44.5) | 11.15 (11.13-11.21) | 0.130 (0.116-0.132) | 0.084 (0.082-0.084) | 0.29 (0.25-0.30) | 0 | 0/3/0 | 27.5 (27.2-29.2) | 0 | 0 |
| wide_gate_home_nav | mppi | 3/3 | 0 | 41.5 (41.3-42.9) | 11.26 (11.26-11.57) | 0.213 (0.181-0.324) | 0.164 (0.159-0.166) | 0.20 (0.18-0.20) | 0 (0-3) | 0/9/0 | 26.4 (26.0-26.4) | 0 | 0 |
| wide_gate_vertical_carry_nav | rpp | 3/3 | 0 | 26.6 (26.5-26.7) | 4.65 | 0.147 (0.146-0.147) | 0.004 (0.003-0.006) | 0.29 (0.28-0.29) | 0 | 0/3/0 | 10.8 (10.1-11.1) | 0 | 0 |
| wide_gate_vertical_carry_nav | dwb | 3/3 | 0 | 27.3 (27.3-27.6) | 4.65 (4.65-4.66) | 0.147 (0.146-0.149) | 0.039 (0.039-0.048) | 0.28 (0.23-0.28) | 0 | 0/4/0 | 27.2 (27.1-29.2) | 1 | 0 |
| wide_gate_vertical_carry_nav | mppi | 3/3 | 0 | 27.1 (27.0-27.3) | 4.66 (4.65-4.66) | 0.134 (0.133-0.140) | 0.014 (0.013-0.018) | 0.28 (0.27-0.28) | 0 | 0/3/0 | 54.3 (53.1-57.7) | 2 | 0 |

Cells are the median and, in parentheses, the range over the three runs. Time, path, and
final error cover successful runs only.

- **Open space.** All three succeed every time. RPP tracks the plan almost exactly
  (cross-track p95 0.002 m); DWB and MPPI deviate by 0.013-0.030 m. MPPI arrives about
  1 s sooner because it does not slow for the goal: 0.3 m before it, MPPI still moves at
  0.28 m/s, RPP at 0.15 m/s and DWB at 0.17 m/s.
- **Detours with the home footprint (both gates).** All succeed. MPPI is 2-4 s faster but
  cuts corners: cross-track p95 0.16-0.33 m against 0.05-0.09 m, and the lowest static
  clearance of all runs (0.075 m, against 0.18-0.30 m for RPP and DWB).
- **Goal approach.** On the detours MPPI stops 0.18-0.50 m from the goal although the
  tolerance is 0.15 m. The detour reaches the goal heading south (-pi/2) while the goal
  heading is pi. RPP and DWB stop and rotate in place. Forward-only MPPI, in the 0.50 m
  run, passed within 0.06 m of the goal, where the stateful goal checker latches the
  position, and made the remaining turn of about 0.9 rad on a forward arc. Where the path
  already ends at the goal heading, MPPI stops within tolerance (0.13 m).
- **1.05 m gate in vertical carry.** Passed by RPP 2/3, DWB 2/3, MPPI 1/3. The RPP failure
  after fix 2 was one draw from this distribution: the lidar-noise-narrowed opening
  (0.85 m in the local costmap, 2026-09-28 fix 2) leaves the throat marginal for every
  controller. Failed runs stop at the throat under the collision monitor's stop zone
  (MPPI: 113 stop and 124 slowdown activations over its three runs) and end by abort or
  by the 90 s timeout. When they pass, RPP takes 26.7-26.8 s, MPPI 27.0 s, and DWB
  33.4-34.6 s.
- **1.30 m gate in vertical carry.** All pass in 26.5-27.6 s with 0.23-0.29 m clearance.
- **Compute (controller server, share of one core).** RPP 8-11 %, DWB 23-29 %, MPPI 24 %
  in open space and up to 58 % near the 1.30 m gate walls, where the CostCritic checks the
  full footprint for every sampled pose inside the circumscribed radius. Loop-rate misses
  (below 20 Hz) were rare, at most 2 per run, except in runs blocked at the 1.05 m throat
  (4-7 per run, lowest 10-12.5 Hz).
- **Command limits.** Across the runs without a collision-monitor action, the largest step
  between consecutive `/cmd_vel` messages was +0.0225 / -0.025 m/s and 0.05 rad/s per
  0.05 s for every controller (0.45 / 0.5 m/s^2 and 1.0 rad/s^2, the smoother's limits),
  and speeds stayed at or below 0.3 m/s and 0.4 rad/s; MPPI's own output rose by up to
  0.056 m/s per cycle. When the collision monitor acts it scales or zeroes the command in
  one step (up to 0.16 m/s and 0.23 rad/s), downstream of the smoother by design
  (ADR 0006); the Unity actuator limiter still shapes those steps.

### Run infrastructure faults (not navigation results)

- **Partial bring-up.** In two runs the global-planning lifecycle manager aborted because
  the planner's costmap failed to activate (transform wait timed out). The planner server
  had already activated its action server, so it accepted goals and planned straight
  through the gate on an empty costmap; the robot drove at the gate until the local
  costmap and collision monitor stopped it (0.024 m and 0.087 m clearance, no contact).
  These runs are excluded and were rerun. The runner now requires both lifecycle managers'
  `is_active` before a run and relaunches otherwise (3 of the 45 runs needed one relaunch),
  and both managers start 3 s after their nodes.
- **Arm control lost between runs.** Three runs could not start because the arm was not in
  a fresh HOLD. The ROS arm hardware had latched "Unity feedback stale" and deactivated.
  A ROS-side monitor saw `/clock` and `/arm/state` pause together 57 times in about 40
  minutes, excluding Play restarts: 53 pauses of 0.25-0.4 s, 1 of 0.4-0.5 s, and 3 of
  0.63-1.0 s, clustered around Nav2 launches and shutdowns. The arm hardware's 0.5 s
  feedback timeout trips on the longest. The three runs were rerun, and the runner now
  restarts arm control once in the same Play epoch when this happens.

## 2026-09-30 obstacle scenarios with stock Nav2 (roadmap scenarios 2 and 7)

Scenarios can now list unmapped obstacle boxes, which the runner places in Play after the
teleport and removes after the run; the task also reports the footprint's clearance to
them. All runs used Cyclone DDS (ADR 0008), 3 rounds per controller with a fresh epoch
per round. The scenario definitions were not yet committed when these runs were made
(the summaries record them in full).

- `static_obstacle_detour_nav` (scenario 2): 8 m through the open band from (16, 0) to
  (8, 0) with the home footprint, past an unmapped 0.6 x 0.6 x 0.8 m box on the path at
  (12, 0).
- `persistent_blockage_nav` (scenario 7): the vertical-carry route through the 1.30 m gate,
  whose opening is filled by an unmapped 0.3 x 1.2 x 1.0 m box; the longer detour stays
  open.

| Scenario | Controller | Success | Contact | Time s | Path m | Final error m | Cross-track p95 m | Min clearance m | Obstacle clearance m | Recoveries | Monitor stop/slow/appr | Controller CPU % | Loop misses | Controller errors |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| persistent_blockage_nav | rpp | 0/3 | 0 | - | - | - | 0.002 (0.002-0.003) | 0.44 (0.43-0.44) | 0.36 (0.34-0.36) | 11 | 0/20/0 | 15.2 (8.8-15.8) | 19 | 29 |
| persistent_blockage_nav | dwb | 0/3 | 0 | - | - | - | 0.024 (0.011-0.027) | 0.32 (0.29-0.32) | 0.16 (0.16-0.16) | 13 (11-13) | 0/20/0 | 40.8 (21.6-47.3) | 81 | 29 |
| persistent_blockage_nav | mppi | 0/3 | 0 | - | - | - | 0.022 (0.019-0.048) | 0.34 (0.29-0.37) | 0.28 (0.26-0.28) | 12 (11-12) | 0/26/0 | 59.7 (47.4-69.8) | 327 | 27 |
| static_obstacle_detour_nav | rpp | 0/3 | 0 | - | - | - | 0.004 (0.002-0.005) | 2.92 (2.92-2.92) | 0.37 (0.35-0.37) | 11 | 0/0/0 | 11.0 (10.7-11.6) | 19 | 30 |
| static_obstacle_detour_nav | dwb | 0/3 | 0 | - | - | - | 0.045 (0.007-0.047) | 2.92 (2.92-2.92) | 0.21 (0.19-0.22) | 9 | 0/14/0 | 37.7 (36.4-42.5) | 39 | 21 |
| static_obstacle_detour_nav | mppi | 0/3 | 0 | - | - | - | 0.066 (0.015-0.080) | 2.93 (2.92-2.93) | 0.24 (0.20-0.29) | 9 (9-10) | 0/23/0 | 74.3 (69.5-76.0) | 555 | 20 |

Every run failed without contact: the robot stopped 0.16-0.37 m short of the box and the
tree gave up ("Failed to make progress", "Controller patience exceeded") or the timeout
ended it. Nav2's global costmap in this Phase 1 setup contains only the static map, so
the planned path runs straight through an unmapped obstacle and is never marked invalid:
the replan-if-invalid tree keeps it, and a 1 Hz replan would return the same path. Only
the local costmap sees the box. RPP cannot leave its path; DWB and MPPI score closeness to
the path heavily and found no way around within their horizons (1.7 s and 2.8 s at
0.3 m/s). Leaving a blocked route therefore needs obstacle information in the global
costmap (next section). The stock configuration stays the default for comparison.

## 2026-09-30 obstacle scenarios with live obstacles in the global costmap

`navigation.launch.py global_obstacles:=true` (runner `--global-obstacles`) adds an STVL
layer fed by the filtered Livox cloud to the global costmap, with the same sensor contract
as the local layer (0.05-2.0 m marking band, 10 s decay, frustum clearing). The default
stays static-map only. All 24 runs from commit `14966cc`, 3 rounds for the obstacle
scenarios and one regression round of open space and the 1.30 m vertical-carry gate.

| Scenario | Controller | Success | Contact | Time s | Path m | Final error m | Cross-track p95 m | Min clearance m | Obstacle clearance m | Recoveries | Monitor stop/slow/appr | Controller CPU % | Loop misses | Controller errors |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| open_space_nav | rpp +global obstacles | 1/1 | 0 | 15.5 | 3.85 | 0.146 | 0.003 | 2.53 | - | 0 | 0/0/0 | 8.2 | 0 | 0 |
| open_space_nav | dwb +global obstacles | 1/1 | 0 | 15.6 | 3.85 | 0.146 | 0.011 | 2.48 | - | 0 | 0/0/0 | 33.0 | 0 | 0 |
| open_space_nav | mppi +global obstacles | 1/1 | 0 | 14.5 | 3.87 | 0.130 | 0.010 | 2.50 | - | 0 | 0/0/0 | 29.9 | 0 | 0 |
| persistent_blockage_nav | rpp +global obstacles | 3/3 | 0 | 41.9 (41.9-42.3) | 10.49 (10.44-10.69) | 0.121 (0.117-0.143) | 0.071 (0.068-0.072) | 0.22 (0.20-0.27) | 0.52 (0.47-0.52) | 0 (0-1) | 0/6/0 | 10.3 (9.4-10.5) | 1 | 1 |
| persistent_blockage_nav | dwb +global obstacles | 3/3 | 0 | 40.4 (39.7-44.7) | 10.63 (10.37-10.74) | 0.131 (0.127-0.136) | 0.071 (0.071-0.090) | 0.32 (0.17-0.34) | 0.55 (0.52-0.58) | 0 | 0/1/0 | 29.8 (28.0-31.1) | 0 | 0 |
| persistent_blockage_nav | mppi +global obstacles | 3/3 | 0 | 39.1 (38.9-39.4) | 10.88 (10.65-10.94) | 0.343 (0.189-0.356) | 0.128 (0.114-0.152) | 0.26 (0.24-0.26) | 0.71 (0.67-0.73) | 0 | 0/1/0 | 36.4 (34.9-37.7) | 2 | 0 |
| static_obstacle_detour_nav | rpp +global obstacles | 3/3 | 0 | 33.3 (33.1-33.9) | 8.47 (8.47-8.50) | 0.128 (0.126-0.136) | 0.043 (0.039-0.045) | 2.31 (2.30-2.32) | 0.37 (0.37-0.38) | 0 | 0/0/0 | 8.2 (8.0-8.4) | 0 | 0 |
| static_obstacle_detour_nav | dwb +global obstacles | 3/3 | 0 | 34.8 (34.7-35.2) | 8.48 (8.47-8.52) | 0.124 (0.115-0.135) | 0.063 (0.062-0.067) | 2.32 (2.28-2.32) | 0.39 (0.38-0.40) | 0 | 0/0/0 | 29.9 (29.3-30.3) | 1 | 0 |
| static_obstacle_detour_nav | mppi +global obstacles | 3/3 | 0 | 31.3 (31.3-31.8) | 8.55 (8.49-8.72) | 0.109 (0.095-0.178) | 0.093 (0.077-0.117) | 2.29 (2.14-2.38) | 0.36 (0.36-0.36) | 0 | 0/0/0 | 34.5 (33.3-35.9) | 2 | 0 |
| wide_gate_vertical_carry_nav | rpp +global obstacles | 1/1 | 0 | 26.6 | 4.65 | 0.145 | 0.006 | 0.29 | - | 0 | 0/1/0 | 10.7 | 0 | 0 |
| wide_gate_vertical_carry_nav | dwb +global obstacles | 1/1 | 0 | 27.2 | 4.66 | 0.145 | 0.046 | 0.24 | - | 0 | 0/1/0 | 32.8 | 0 | 0 |
| wide_gate_vertical_carry_nav | mppi +global obstacles | 1/1 | 0 | 27.2 | 4.66 | 0.130 | 0.015 | 0.27 | - | 0 | 0/1/0 | 68.0 | 24 | 0 |

- Both obstacle scenarios succeeded in every run with every controller, without contact
  (18/18 against 0/18 with the static global costmap). Once the box is in the global
  costmap, the replan-if-invalid tree finds the path blocked and replans: around the box
  in the open (8.5 m, 31-35 s, 0.36-0.40 m from the box) and onto the detour when the gate
  is blocked (10.4-10.9 m, 39-42 s, 0.47-0.73 m from the blocker).
- MPPI is again fastest and least precise at the goal: on the blockage route it ends
  0.19-0.36 m from the goal (the goal-turn behaviour recorded above).
- Regression: open space and the 1.30 m vertical-carry gate matched the static-map runs
  (15.5 and 26.6-27.2 s), so lidar noise around the gate posts did not close the gate in
  the global costmap. MPPI's controller used 68 % of a core with 24 loop-rate misses at
  the gate in this single run, against 54 % and 1 miss before; the obstacle layer runs in
  the planner server, so this is more likely run-to-run load than an effect of the layer.

## 2026-10-01 worker crossing (roadmap scenario 3)

`worker_crossing_nav` drives the open band from (16, 0) to (8, 0) with the home footprint
while a 0.5 x 0.5 x 1.8 m worker (a Unity scenario mover) crosses the route once at
x = 11 m, from y = +2.5 to -2.5 m at 0.8 m/s. The worker starts when the robot comes within
2.5 m of the crossing point, so it is on the route when the robot is about 1.6 m away, and
it would wait if the robot were within 0.1 m of its next position (it never had to).
Navigation sees the worker only through the lidar; the clearance to it comes from Unity's
ground truth on `/scenario/movers`. Runs from the working tree committed as `84ecfaa` and
`d2a1bdd`, with the C++ scenario tools, three rounds per controller and global-costmap
setting (one extra RPP smoke run with global obstacles).

| Scenario | Controller | Success | Contact | Time s | Path m | Final error m | Cross-track p95 m | Min clearance m | Obstacle clearance m | Mover clearance m | Mover waited s | Recoveries | Monitor stop/slow/appr | Controller CPU % | Loop misses | Controller errors |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| worker_crossing_nav | rpp | 3/3 | 0 | 30.2 (30.2-30.3) | 7.85 (7.85-7.86) | 0.147 (0.147-0.148) | 0.003 (0.003-0.004) | 2.53 | - | 0.50 (0.49-0.51) | 0.0 | 1 | 0/0/0 | 7.9 (6.4-8.1) | 2 | 3 |
| worker_crossing_nav | dwb | 3/3 | 0 | 39.7 (39.3-39.8) | 7.87 (7.86-7.87) | 0.145 (0.145-0.147) | 0.029 (0.020-0.044) | 2.49 (2.48-2.52) | - | 0.54 (0.53-0.54) | 0.0 | 0 | 0/0/0 | 29.0 (28.6-29.1) | 0 | 0 |
| worker_crossing_nav | mppi | 3/3 | 0 | 38.9 (38.5-87.1) | 7.96 (7.91-20.71) | 0.139 (0.132-1.171) | 0.200 (0.130-2.115) | 2.51 (0.33-2.52) | - | 0.62 (0.61-0.64) | 0.0 | 0 (0-1) | 0/2/0 | 35.2 (33.2-37.6) | 3 | 1 |
| worker_crossing_nav | rpp +global obstacles | 2/4 | 0 | 61.8 (59.1-64.4) | 13.38 (13.23-13.54) | 0.134 (0.130-0.137) | 0.058 (0.044-0.069) | 2.32 (2.25-2.92) | - | 0.27 (0.20-0.29) | 0.0 | 2 (0-11) | 0/14/0 | 6.5 (6.2-8.0) | 7 | 13 |
| worker_crossing_nav | dwb +global obstacles | 1/3 | 0 | 40.1 | 8.86 | 0.138 | 0.065 (0.051-0.159) | 2.92 (2.35-2.92) | - | 0.33 (0.19-0.65) | 0.0 | 6 (2-6) | 0/41/0 | 25.6 (25.4-26.4) | 6 | 11 |
| worker_crossing_nav | mppi +global obstacles | 3/3 | 0 | 59.4 (39.1-85.1) | 13.31 (7.98-20.37) | 0.425 (0.133-0.623) | 1.747 (0.042-2.567) | 0.34 (0.26-2.51) | - | 0.59 (0.58-0.63) | 0.0 | 2 | 0/11/0 | 32.7 (32.2-34.6) | 2 | 3 |

- **With the static global costmap (Phase 1 baseline) all 9 runs succeeded** on a
  near-straight route (7.85-7.96 m, 30-40 s, 0.49-0.64 m from the worker). The worker
  enters only the local costmap; the robot slows or pauses while it passes and its 10 s
  trail decays, then continues on the unchanged global path. One MPPI run wandered off the
  route (20.7 m, 87 s, 1.17 m final error) but arrived without contact.
- **With live obstacles in the global costmap (the current default) only 6 of 10
  succeeded, and the successes detoured** (13.2-20.4 m, 39-85 s). The global layer marks
  the worker's walked line as a wall across the band for the 10 s decay, and the
  replan-if-invalid tree immediately routes around the wall's far end, which is where the
  worker is walking to. In three of the four failures (RPP 1, DWB 2) the robot followed
  the worker south and stalled beside its final position at (10.8-12.1, -3.2 to -3.9) until
  the tree gave up ("Controller patience exceeded", "Failed to make progress") or the 90 s
  timeout ended the run. In the fourth (RPP) the detour reached the goal position (0.13 m)
  but the timeout ended the final turn 0.39 rad from the goal heading (see the heading
  tolerance limitation below). The trail itself decays as configured: in the aborted RPP
  run the recorded costmaps marked the route along y = 0 only until about 160 s, 10 s after
  the crossing, by when the robot was already 2 m south of the route on its detour.
- No run had contact, and the worker never waited for the robot. Closest approach to the
  worker was 0.19-0.33 m in the detouring runs against about 0.5 m on the straight route.
- The live global obstacle layer therefore helps a persistent blockage (scenario 7, 9/9
  against 0/9) and hurts a transient crossing (scenario 3, 6/10 against 9/9). This matches
  the roadmap's intent that transient workers stay local reactive obstacles; how the
  global costmap should treat them is recorded as an open decision in the roadmap.

## 2026-10-01 persistent obstacles in the global costmap

To keep the route reaction to blockages without chasing passing workers, the global costmap
now uses `PersistentObstacleLayer` (`global_obstacles:=persistent`, the default since
commit `1d04eb4`) instead of the live STVL layer. It reads the same filtered Livox cloud,
marking band (0.05-2.0 m), 5 m range and 10 s decay, but marks a cell only after 2 s of
observations with gaps of at most 1 s. A worker walking past covers any one cell for about
0.6 s, so it stays in the local costmap only; a box, or a worker who stops, is confirmed
after 2 s and changes the route. All 33 runs from commit `1d04eb4`, three rounds of the
obstacle and crossing scenarios per controller and one regression round.

| Scenario | Controller | Success | Contact | Time s | Path m | Final error m | Cross-track p95 m | Min clearance m | Obstacle clearance m | Mover clearance m | Mover waited s | Recoveries | Monitor stop/slow/appr | Controller CPU % | Loop misses | Controller errors |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| open_space_nav | rpp +persistent global | 1/1 | 0 | 15.5 | 3.85 | 0.148 | 0.002 | 2.53 | - | - | - | 0 | 0/0/0 | 7.3 | 0 | 0 |
| open_space_nav | dwb +persistent global | 1/1 | 0 | 15.7 | 3.86 | 0.146 | 0.013 | 2.48 | - | - | - | 0 | 0/0/0 | 29.1 | 0 | 0 |
| open_space_nav | mppi +persistent global | 1/1 | 0 | 14.4 | 3.86 | 0.137 | 0.020 | 2.49 | - | - | - | 0 | 0/0/0 | 27.1 | 0 | 0 |
| persistent_blockage_nav | rpp +persistent global | 3/3 | 0 | 41.3 (39.3-42.2) | 10.54 (10.53-10.63) | 0.128 (0.105-0.131) | 0.074 (0.062-0.090) | 0.28 (0.23-0.30) | 0.53 (0.51-0.56) | - | - | 0 | 0/0/0 | 9.7 (9.5-9.8) | 0 | 0 |
| persistent_blockage_nav | dwb +persistent global | 3/3 | 0 | 41.6 (40.5-45.0) | 10.53 (10.46-10.66) | 0.118 (0.118-0.136) | 0.081 (0.071-0.276) | 0.26 (0.25-0.28) | 0.54 (0.54-0.54) | - | - | 0 (0-1) | 0/0/0 | 28.7 (28.4-30.0) | 1 | 1 |
| persistent_blockage_nav | mppi +persistent global | 3/3 | 0 | 39.3 (38.8-40.9) | 10.63 (10.58-11.00) | 0.241 (0.151-0.493) | 0.147 (0.112-0.148) | 0.22 (0.20-0.24) | 0.70 (0.69-0.75) | - | - | 0 | 0/4/0 | 38.3 (35.5-38.6) | 2 | 0 |
| static_obstacle_detour_nav | rpp +persistent global | 3/3 | 0 | 33.8 (33.8-34.1) | 8.52 (8.47-8.58) | 0.131 (0.129-0.133) | 0.038 (0.037-0.040) | 2.31 (2.30-2.35) | 0.38 (0.38-0.47) | - | - | 0 | 0/0/0 | 7.2 (6.4-7.5) | 0 | 0 |
| static_obstacle_detour_nav | dwb +persistent global | 3/3 | 0 | 34.8 (34.7-35.4) | 8.51 (8.46-8.60) | 0.121 (0.099-0.145) | 0.060 (0.047-0.068) | 2.31 (2.28-2.31) | 0.39 (0.38-0.47) | - | - | 0 | 0/0/0 | 29.6 (25.1-30.4) | 1 | 0 |
| static_obstacle_detour_nav | mppi +persistent global | 3/3 | 0 | 32.1 (32.0-34.3) | 8.79 (8.55-9.31) | 0.216 (0.036-0.752) | 0.109 (0.096-0.287) | 2.10 (1.56-2.31) | 0.36 (0.34-0.37) | - | - | 0 | 0/1/0 | 33.4 (31.9-35.1) | 0 | 0 |
| wide_gate_vertical_carry_nav | rpp +persistent global | 1/1 | 0 | 26.6 | 4.66 | 0.145 | 0.003 | 0.28 | - | - | - | 0 | 0/1/0 | 10.2 | 0 | 0 |
| wide_gate_vertical_carry_nav | dwb +persistent global | 1/1 | 0 | 27.3 | 4.65 | 0.147 | 0.036 | 0.28 | - | - | - | 0 | 0/1/0 | 30.4 | 0 | 0 |
| wide_gate_vertical_carry_nav | mppi +persistent global | 1/1 | 0 | 27.1 | 4.65 | 0.142 | 0.019 | 0.27 | - | - | - | 0 | 0/1/0 | 60.9 | 3 | 0 |
| worker_crossing_nav | rpp +persistent global | 3/3 | 0 | 30.2 (30.2-30.3) | 7.85 (7.85-7.85) | 0.148 (0.145-0.148) | 0.003 (0.002-0.003) | 2.53 (2.53-2.53) | - | 0.49 (0.49-0.50) | 0.0 | 1 | 0/0/0 | 7.5 (7.4-7.9) | 1 | 3 |
| worker_crossing_nav | dwb +persistent global | 3/3 | 0 | 39.6 (39.4-39.6) | 7.86 (7.86-7.87) | 0.147 (0.146-0.148) | 0.047 (0.044-0.048) | 2.52 (2.52-2.53) | - | 0.55 (0.54-0.55) | 0.0 | 0 | 0/0/0 | 29.4 (28.6-30.1) | 0 | 0 |
| worker_crossing_nav | mppi +persistent global | 3/3 | 0 | 38.7 (38.4-79.3) | 7.97 (7.93-19.61) | 0.137 (0.136-1.813) | 0.222 (0.111-1.384) | 2.52 (0.66-2.52) | - | 0.62 (0.60-0.62) | 0.0 | 0 (0-1) | 0/0/0 | 35.9 (34.9-36.7) | 2 | 1 |

- **Every run succeeded without contact (33/33).** The crossing now behaves like the static
  costmap (straight 7.85-7.97 m route, 30-40 s, 0.49-0.62 m from the worker, 9/9), while the
  box detour (8.5-9.3 m, 32-35 s) and the blocked gate (10.5-11.0 m, 39-45 s) are taken as
  with the live layer (9/9 each). Open space and the 1.30 m vertical-carry gate match
  earlier runs.

| Global costmap | Box on the route (2) | Worker crossing (3) | Blocked gate (7) |
|---|---|---|---|
| Static map only | 0/9 | 9/9, 7.9 m | 0/9 |
| Live lidar obstacles, 10 s decay | 9/9 | 6/10, 13-20 m | 9/9 |
| Persistent lidar obstacles (2 s) | 9/9 | 9/9, 7.9 m | 9/9 |

- One MPPI crossing run wandered off the route (19.6 m, 79 s, 1.8 m final error) as one did
  with the static costmap (20.7 m); this is MPPI's own behaviour, not the global layer.
- Limits of the 2 s threshold: anything that stays in one place for 2 s becomes a route
  obstacle until 10 s after it was last seen, including a worker who stops on the route or
  a slow, long cart (a 2 m cart at 0.5 m/s covers a cell for 4 s); a worker who stops briefly
  and walks on can make the route detour for up to 10 s. These are the intended semantics
  (it stayed, so the route avoids it), but the threshold has only been tested with the
  scenarios above.

## 2026-10-04 full rerun (current B1/B2 baseline)

Every Phase 1 scenario rerun on the code the B3 missions use, so B1 (DWB), B2 (MPPI) and the RPP
bring-up sit on the same footing as B3. Since the earlier tables in this file the stack changed in
four ways: the lidar draws independent range noise per point and scan with a 6 sigma self-filter
band (the earlier runs had a repeating noise sequence and a 4 sigma band, which marked the robot's
own cell in some sessions), the persistent global layer is the default for every scenario (the gate
scenarios above were recorded with the static or live layer), the sleeping-base odometry twist is
zero, and a drive counts as reached only if the base ends within the goal checker's tolerances plus
0.02 (0.17 m, 0.17 rad). The stateful goal checker stops checking the position once the base has
passed within 0.15 m; a drive Nav2 reports reached while the base ended farther away is `off_goal`.
The summarizer applies the same rule to the earlier summaries in `runs/` (18 of them, all MPPI,
change from success to `off_goal`), so earlier MPPI success counts in this file are too high.

Commit `849f19b` (clean tree), 2026-10-04 00:13-02:04 UTC, `--new-epoch` per controller, three
runs of each of the eight driving scenarios per controller (72 runs) and one run of each plan-only
scenario. Real-time factor 0.995-1.036, no host sleep, no process killed for memory, no
global-costmap frame marked a cell within 0.3 m of the robot centre. Summaries:
`runs-2026-10-04/`; table from
`python3 tools/summarize_nav_runs.py docs/experiments/nav2-navigation/runs-2026-10-04/*_nav-*-summary.json`.

| Scenario | Controller | Success | Contact | Time s | Path m | Final error m | Cross-track p95 m | Min clearance m | Obstacle clearance m | Mover clearance m | Mover waited s | Recoveries | Monitor stop/slow/appr | Controller CPU % | Loop misses | Controller errors |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| narrow_gate_home_nav | rpp +persistent global | 3/3 | 0 | 44.4 (44.4-45.2) | 11.01 (10.99-11.05) | 0.136 (0.130-0.158) | 0.071 (0.070-0.072) | 0.18 (0.16-0.19) | - | - | - | 0 | 0/6/0 | 6.9 (6.8-7.0) | 0 | 0 |
| narrow_gate_home_nav | dwb +persistent global | 3/3 | 0 | 45.2 (44.6-45.9) | 10.90 (10.88-10.94) | 0.141 (0.110-0.143) | 0.097 (0.089-0.101) | 0.17 (0.16-0.18) | - | - | - | 0 | 0/6/0 | 21.6 (21.4-21.9) | 0 | 0 |
| narrow_gate_home_nav | mppi +persistent global | 0/3 | 0 | - | - | - | 0.177 (0.164-0.203) | 0.16 (0.15-0.20) | - | - | - | 0 | 0/9/0 | 23.0 (22.6-24.4) | 0 | 0 |
| narrow_gate_vertical_carry_nav | rpp +persistent global | 2/3 | 0 | 26.7 (26.6-26.7) | 4.65 (4.65-4.65) | 0.146 (0.145-0.147) | 0.004 (0.002-0.011) | 0.16 (0.13-0.16) | - | - | - | 0 (0-13) | 0/7/0 | 6.9 (6.2-7.4) | 6 | 10 |
| narrow_gate_vertical_carry_nav | dwb +persistent global | 3/3 | 0 | 29.1 (28.2-29.4) | 4.66 (4.65-4.66) | 0.145 (0.144-0.147) | 0.026 (0.023-0.043) | 0.14 (0.13-0.14) | - | - | - | 0 | 0/10/0 | 20.5 (20.4-20.7) | 0 | 0 |
| narrow_gate_vertical_carry_nav | mppi +persistent global | 3/3 | 0 | 27.0 (26.9-28.8) | 4.66 (4.66-4.69) | 0.135 (0.134-0.142) | 0.025 (0.024-0.064) | 0.14 (0.12-0.15) | - | - | - | 0 | 0/4/0 | 34.4 (33.2-35.7) | 0 | 0 |
| open_space_nav | rpp +persistent global | 3/3 | 0 | 15.6 (15.5-15.6) | 3.86 (3.85-3.86) | 0.146 (0.145-0.147) | 0.003 (0.003-0.004) | 2.53 (2.53-2.53) | - | - | - | 0 | 0/0/0 | 6.3 (5.9-6.7) | 0 | 0 |
| open_space_nav | dwb +persistent global | 3/3 | 0 | 15.7 (15.6-15.8) | 3.86 (3.85-3.86) | 0.148 (0.147-0.149) | 0.014 (0.012-0.045) | 2.49 (2.49-2.52) | - | - | - | 0 | 0/0/0 | 20.8 (19.5-21.5) | 0 | 0 |
| open_space_nav | mppi +persistent global | 3/3 | 0 | 14.4 (14.3-14.4) | 3.86 (3.85-3.87) | 0.137 (0.130-0.141) | 0.011 (0.011-0.013) | 2.50 (2.48-2.51) | - | - | - | 0 | 0/0/0 | 20.1 (20.0-20.7) | 0 | 0 |
| persistent_blockage_nav | rpp +persistent global | 3/3 | 0 | 41.2 (40.2-43.7) | 10.62 (10.51-10.64) | 0.137 (0.127-0.141) | 0.081 (0.063-0.097) | 0.29 (0.28-0.29) | 0.53 (0.53-0.54) | - | - | 0 | 0/0/0 | 8.4 (7.6-8.6) | 0 | 0 |
| persistent_blockage_nav | dwb +persistent global | 3/3 | 0 | 40.6 (40.4-41.8) | 10.61 (10.53-10.70) | 0.117 (0.110-0.126) | 0.078 (0.057-0.111) | 0.26 (0.25-0.29) | 0.56 (0.54-0.56) | - | - | 0 | 0/0/0 | 21.3 (20.8-21.3) | 0 | 0 |
| persistent_blockage_nav | mppi +persistent global | 0/3 | 0 | - | - | - | 0.100 (0.094-0.125) | 0.24 (0.23-0.28) | 0.70 (0.66-0.73) | - | - | 0 | 0/0/0 | 25.8 (25.7-27.5) | 0 | 0 |
| static_obstacle_detour_nav | rpp +persistent global | 2/3 | 0 | 33.5 (33.4-33.6) | 8.54 (8.54-8.55) | 0.133 (0.125-0.140) | 0.042 (0.034-0.044) | 2.30 (2.29-2.31) | 0.47 (0.47-0.47) | - | - | 0 (0-6) | 0/0/0 | 6.1 (6.1-6.6) | 1 | 5 |
| static_obstacle_detour_nav | dwb +persistent global | 3/3 | 0 | 34.6 (34.2-35.0) | 8.49 (8.47-8.52) | 0.128 (0.118-0.133) | 0.063 (0.062-0.066) | 2.30 (2.30-2.32) | 0.38 (0.36-0.39) | - | - | 0 | 0/0/0 | 21.1 (20.6-21.5) | 0 | 0 |
| static_obstacle_detour_nav | mppi +persistent global | 2/3 | 0 | 31.7 (31.6-31.8) | 8.67 (8.60-8.73) | 0.114 (0.082-0.146) | 0.093 (0.088-0.150) | 2.17 (2.04-2.24) | 0.35 (0.35-0.47) | - | - | 0 | 0/0/0 | 24.5 (20.5-25.0) | 0 | 0 |
| wide_gate_home_nav | rpp +persistent global | 3/3 | 0 | 44.8 (43.7-45.1) | 11.26 (11.25-11.37) | 0.128 (0.124-0.141) | 0.080 (0.062-0.081) | 0.31 (0.25-0.31) | - | - | - | 0 | 0/3/0 | 7.5 (7.2-7.5) | 0 | 0 |
| wide_gate_home_nav | dwb +persistent global | 1/3 | 0 | 43.1 | 11.24 | 0.136 | 0.074 (0.074-0.091) | 0.30 (0.27-0.33) | - | - | - | 11 (0-11) | 0/3/0 | 18.9 (18.8-22.6) | 0 | 20 |
| wide_gate_home_nav | mppi +persistent global | 0/3 | 0 | - | - | - | 0.220 (0.148-0.521) | 0.15 (0.12-0.20) | - | - | - | 0 | 0/4/0 | 23.4 (23.0-23.5) | 0 | 0 |
| wide_gate_vertical_carry_nav | rpp +persistent global | 3/3 | 0 | 26.6 (26.6-26.6) | 4.65 (4.65-4.66) | 0.146 (0.145-0.147) | 0.005 (0.003-0.005) | 0.29 (0.28-0.29) | - | - | - | 0 | 0/4/0 | 7.8 (7.7-8.0) | 0 | 0 |
| wide_gate_vertical_carry_nav | dwb +persistent global | 3/3 | 0 | 27.3 (27.1-27.4) | 4.65 (4.65-4.66) | 0.147 (0.147-0.148) | 0.036 (0.035-0.046) | 0.28 (0.24-0.28) | - | - | - | 0 | 0/3/0 | 22.3 (21.8-22.8) | 0 | 0 |
| wide_gate_vertical_carry_nav | mppi +persistent global | 3/3 | 0 | 27.1 (26.9-27.1) | 4.66 (4.66-4.66) | 0.136 (0.134-0.140) | 0.018 (0.015-0.023) | 0.27 (0.26-0.28) | - | - | - | 0 | 0/4/0 | 45.0 (44.6-47.9) | 3 | 0 |
| worker_crossing_nav | rpp +persistent global | 3/3 | 0 | 30.3 (30.3-30.3) | 7.85 (7.85-7.86) | 0.146 | 0.003 (0.003-0.003) | 2.53 (2.53-2.53) | - | 0.50 (0.49-0.50) | 0.0 | 1 | 0/0/0 | 6.3 (6.0-6.3) | 3 | 3 |
| worker_crossing_nav | dwb +persistent global | 3/3 | 0 | 39.5 (39.4-39.7) | 7.86 (7.86-7.87) | 0.147 (0.146-0.148) | 0.045 (0.031-0.046) | 2.52 (2.49-2.52) | - | 0.55 (0.53-0.56) | 0.0 | 0 | 0/0/0 | 21.2 (20.8-22.1) | 0 | 0 |
| worker_crossing_nav | mppi +persistent global | 1/3 | 0 | 38.4 | 7.91 | 0.129 | 0.208 (0.096-3.366) | 1.61 (0.47-2.52) | - | 0.64 (0.64-0.66) | 0.0 | 0 (0-11) | 0/0/0 | 25.9 (22.1-26.1) | 0 | 10 |

- **RPP 22/24, DWB 22/24, MPPI 12/24, no contact in any run.**
- **MPPI:** 11 of its 12 failures are `off_goal`, drives Nav2 reported reached while the base ended
  0.17-0.77 m from the goal (both home-footprint gate detours 0/3, the blocked gate 0/3, one box
  detour and one crossing): stock MPPI passes the goal inside the tolerance and keeps driving on a
  forward arc while it turns to the goal heading (roadmap decision 2026-09-28, MPPI baseline). The
  twelfth is a crossing run that wandered off the route (aborted 3.4 m from the goal, error 103), as
  in the earlier runs. Where MPPI reaches the goal it is the fastest controller (open space 14.4 s,
  box detour 31.7 s).
- **DWB:** `wide_gate_home_nav` 1/3: twice the base stopped inside the position tolerance (0.11 and
  0.13 m) with 0.17 and 0.31 rad of heading error and the tree aborted (error 103) after 11
  recoveries, the heading limitation below.
- **RPP:** one `narrow_gate_vertical_carry_nav` run stopped 2.2 m short of the goal at the 1.05 m
  throat (error 104 after 13 recoveries; the Smac Lattice plan through that gate S-bends depending
  on the start pose, docs/experiments/moveit-arm "Throat plans"), and one
  `static_obstacle_detour_nav` run reached the goal position (0.13 m) but stayed 0.45 rad off the
  goal heading until the 90 s timeout ("Failed to make progress" six times).
- The 1.05 m throat in vertical carry passed 8 of 9 (RPP 2/3, DWB 3/3, MPPI 3/3), against 5 of 9
  in the controller comparison above; the 1.30 m gate passed 9/9. Box detour clearance 0.35-0.47 m,
  blocked-gate detour 10.5-10.7 m, worker crossing 0.49-0.56 m from the worker for RPP and DWB.
- Plan-only scenarios (`open_space`, `narrow_gate_home`, `narrow_gate_vertical_carry`,
  `wide_gate_home`, `wide_gate_vertical_carry`) plan on the static map only and gave the same paths
  as before (the wide-gate cases identical to the millimetre).

## Known limitation: goal heading tolerance and the yaw breakaway

The goal checker accepts 0.15 rad of heading error, and the base does not start turning
from rest below 0.07 rad/s (base-controller README, low-speed breakaway). When a
controller's last heading correction is smaller than a breakaway-sized turn, the command
is not executed and the heading freezes. DWB's final heading errors in the controller
comparison were 0.144-0.150 rad, right at the tolerance, and in the Cyclone DDS
confirmation round DWB stopped 0.144 m from the goal (inside the 0.15 m position
tolerance) with 0.151 rad of heading error; the progress checker then failed, replans to
the reached goal returned empty paths, and the tree aborted. RPP (rotate-to-heading at
0.4 rad/s) and MPPI (stock yaw model) were not affected. The tolerance is left unchanged
so the baseline stays comparable; a controller that finishes with sub-breakaway heading
corrections can abort just short of the goal.

