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
