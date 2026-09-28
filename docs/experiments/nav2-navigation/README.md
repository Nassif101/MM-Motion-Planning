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
