# ADR 0007: Navigation scenario reset and runner

Date: 2026-09-28. Status: accepted for Phase 1 benchmarking.

## Context

The roadmap's benchmark chapter needs many repeated runs of fixed scenarios (Section
7.5). ADR 0001 treats a scene reset or robot teleport as a new simulation epoch and
restarts ROS simulation nodes rather than preserving odometry continuity. Restarting
Play for every run is slow and resets `/clock`, which also forces the arm controller
to restart. Teleporting the robot inside one Play session keeps the clock monotonic but
makes `odom -> base_footprint` jump, which invalidates anything that integrated the old
pose: costmaps, planners, controllers, and future odometry filters.

## Options considered

1. New Play session per run.
2. Teleport inside one Play session and clear Nav2 costmaps through its services.
3. Teleport inside one Play session and restart every pose-dependent ROS consumer.

## Decision

Choose option 3, with option 1 available as an explicit epoch restart.

- **Epoch (clock) reset** (`--new-epoch`, or any Play restart): stop Nav2 and arm control,
  restart Play, restart `arm_control.launch.py`, wait for fresh controlled HOLD.
  `robot_state_publisher`, the static `map -> odom`, the endpoint, and rosbridge keep
  running because they hold no time-integrated state.
- **Scenario reset** (every run, same epoch): stop the Nav2 stack, move the arm to the
  scenario pose through home with the qualified 8 s transitions, check the posed
  footprint is free in the static map, teleport the stopped robot with the compiled
  `scenario_place` command (ROS map pose, fresh stopped HOLD in the named pose required),
  verify `/cmd_vel` ownership, relaunch Nav2 with the scenario's footprint profile, and
  verify a single `planner_server` instance.
- The robot's placed TF pose must match the scenario start within 0.10 m before the
  task runs.
- Scenarios are data in `mobile_manipulator_navigation/config/scenarios.yaml`; the
  orchestrator is `tools/run_nav_scenario.py`. Each run records a rosbag and a summary
  with the git commit and dirty flag under the git-ignored `experiment_runs/`; curated
  results are copied into `docs/experiments/`.

## Consequences

- Scenario turnaround is about 10-15 s plus arm transitions, without clock resets.
- No Nav2 state survives a teleport, so runs are independent; the cost is a relaunch
  per run, which is also measured start-up behaviour rather than hidden state.
- Arm transitions reuse only qualified poses and trajectories; new scenario poses need
  qualification first.
- The runner stops processes by recorded process group and by name, and refuses to
  continue if an old instance survives. This addresses stale duplicate `planner_server`
  instances observed during development.

## Validation

- EditMode tests cover the ROS-to-Unity pose conversion against both qualification
  fixtures and heading conventions.
- `test_scenarios.py` checks every scenario is consistent and starts in free space, and
  that the free-space check rejects a start inside a gate post and respects footprint
  rotation.
- 2026-09-28: `open_space`, `narrow_gate_home`, and `narrow_gate_vertical_carry` ran end
  to end after `--new-epoch`, with placement error at most 0.002 m.

## Revisit when

- The task grows beyond planner-only queries (`navigate_to_pose` with the controller
  server); controller and local-costmap nodes join the scenario reset set.
- Dynamic obstacles (workers, carts) need scripted Unity-side reset.
- An odometry filter or localization is introduced (continuity then matters).
