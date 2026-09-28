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
