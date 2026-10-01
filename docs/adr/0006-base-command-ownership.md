# ADR 0006: Single owner of the base command topic

Date: 2026-09-28. Status: accepted for Nav2 integration (Phases 1-3); revisit at Phase 4.

## Context

Unity's `SkidSteerBaseController` executes whatever arrives on `/cmd_vel`
(`geometry_msgs/msg/Twist`, depth-one latest-only endpoint subscription) and does no
arbitration (ADR 0003). The roadmap requires exactly one component with authority over
`/cmd_vel` at a time. Several publishers already exist or are planned: the Nav2
controller server, Nav2 behavior server recoveries, the velocity smoother, the collision
monitor, `ros2 topic pub` smoke tests, `base_step_test.py`, `arm_experiment.py`
disturbance schedules, the qualification runner, and later a whole-body MPC. Unity's
keyboard teleop overrides `/cmd_vel` inside Unity and is invisible to ROS.

Jazzy Nav2 publishes unstamped `Twist` unless `enable_stamped_cmd_vel` is true; a
`TwistStamped` publisher would not connect to Unity's subscription and the base would
silently stop after the 0.5 s watchdog.

## Options considered

1. Let every source publish `/cmd_vel` and rely on "last message wins".
2. Add `twist_mux` with priorities between Nav2, teleop, and test tools.
3. Chain Nav2's own stages through private topics so that only the final safety stage
   publishes `/cmd_vel`, and run test tools only when Nav2 is not running.

## Decision

Choose option 3.

- Command chain: `controller_server` and `behavior_server` publish `cmd_vel_nav`
  (remapped); `velocity_smoother` consumes `cmd_vel_nav` and publishes
  `cmd_vel_smoothed`; `collision_monitor` consumes `cmd_vel_smoothed` and is the only
  node publishing `/cmd_vel`.
- Every Nav2 node sets `enable_stamped_cmd_vel: false` explicitly; the Unity contract is
  `geometry_msgs/msg/Twist`.
- Velocity-smoother and controller limits come from
  `mobile_manipulator_navigation/config/nav_operating_envelope.yaml`.
- Manual and test publishers (`ros2 topic pub`, `base_step_test.py`, `arm_experiment.py`
  with a base disturbance, the qualification runner) run only when no Nav2 command chain
  is active. `base_step_test.py` already refuses to start with another publisher.
- Unity keyboard teleop stays disabled for every ROS test and experiment (ADR 0003).
- `check_cmd_vel_ownership` is the gate before any experiment: it fails unless
  `/cmd_vel` has at most one publisher, from the expected node, of type `Twist`.
- No `twist_mux` for now.

## Consequences

- Command authority is structural and inspectable with one graph query; there is no
  priority table to tune or misconfigure.
- The collision monitor remains the last safety layer for every Nav2 controller baseline
  (DWB, MPPI) without changing Unity.
- Switching between Nav2 and a manual tool requires stopping one before starting the
  other. That is intended during experiments; an operator-override workflow would need
  a mux and a new decision.

## Validation

- `ros2 run mobile_manipulator_navigation check_cmd_vel_ownership --expect collision_monitor`
  passes with the full chain active and fails with a second publisher.
- `ros2 topic info -v /cmd_vel` shows one publisher and Unity's endpoint subscriber.

Validated 2026-09-28 with `navigation.launch.py`: the chain is wired as above and the
ownership check passes ([bring-up evidence](../experiments/nav2-navigation/README.md)).

## Revisit when

- Phase 4 introduces the whole-body MPC. Preferred starting point: the MPC publishes to
  the collision monitor's input so the single-publisher rule and the safety layer stay
  intact; bypassing the collision monitor needs its own decision.
- Operator teleop must pre-empt autonomy during runs (then evaluate `twist_mux`).
- The base command boundary moves to `TwistStamped` or `ros2_control`.
