# Phase 5A: medium-horizon layer (F2, piecewise Bézier) above B5

Spec: [2026-10-29-phase5a-medium-horizon-design.md](../../superpowers/specs/2026-10-29-phase5a-medium-horizon-design.md).
Plan: [2026-10-29-phase5a-medium-horizon.md](../../superpowers/plans/2026-10-29-phase5a-medium-horizon.md).
Branch `phase5a-medium`; B5 baseline: tag `b5-frozen-2026-10-24`.

## Base friction feedforward (plan Task 1, 2026-10-29)

The Unity wheel drives only damp velocity, so from rest the base does not start below the stiction breakaway
(0.0375 m/s, 0.07 rad/s; [base-controller](../base-controller/README.md#2026-09-28-low-speed-breakaway)). Once
moving it loses only 5-10 % of the command, so the deficit is static friction, not a constant drag.

- **Change.** Each wheel's velocity target gets an offset in the command's direction that fades linearly to nothing
  as the measured wheel speed reaches the offset (`SkidSteerKinematics.FrictionFeedforwardTarget`); the offset is the
  measured linear breakaway as wheel speed, 0.0375 / 0.14 = 0.268 rad/s. A zero command stays zero, and a moving
  wheel tracks the plain command.
- **Switch.** On by default (`SkidSteerBaseController.frictionFeedforward`); `unity command base_friction_feedforward
  --enabled false` in Play, runner `--no-base-friction-feedforward` (run suffix `-bff` when on).

Measured with the 2026-09-28 low-speed procedure (V1 open fixture, `home` HOLD, panel attached, two runs, four
starts per command; `base_step_test --breakaway-speeds ... --breakaway-yaw-rates ...`, files
`../base-controller/lowspeed-bff-*`, `step-bff-*`):

| Command from rest | Starts before (of 4) | Starts with feedforward (of 4) | Steady speed / command with feedforward |
|---|---:|---:|---|
| 0.01 m/s | 0 | 4 | 1.89-2.06 |
| 0.02 m/s | 1 | 4 | 1.16-1.27 |
| 0.03 m/s | 1 | 4 | 1.03-1.05 |
| 0.0325 m/s | 0 | 4 | 0.96-0.98 |
| 0.035 m/s | 3 | 4 | 0.91-0.98 |
| 0.0375 m/s | 4 | 4 | 0.95-1.01 |
| 0.04 m/s | 4 | 4 | 0.94-0.96 |
| 0.05 m/s | 4 | 4 | 0.89-0.91 |
| 0.02 rad/s | 0 | 4 | 0.61-1.13 |
| 0.05 rad/s | 0 | 4 | 0.80-1.02 |
| 0.06 rad/s | 0 | 4 | 0.94-1.01 |
| 0.07 rad/s | 4 | 4 | 0.91-1.05 |
| 0.08 rad/s | 4 | 4 | 0.88-0.97 |
| 0.1 rad/s | 4 | 4 | 0.95-1.01 |

- **Deadband gone.** Every command starts the base, and from 0.03 m/s and 0.06 rad/s the base tracks within
  0.88-1.05 of the command (before: 0.57-0.91 just above the breakaway).
- **Steps unchanged.** 0.3 m/s steady 0.277-0.279 (before 0.277-0.280), 0.4 rad/s 0.453 (before 0.453-0.455).
- **Creep.** A command below about 0.025 m/s now drives the base at about 0.02 m/s (the kick does not fade fully
  below half the breakaway speed). Controllers that zero tiny commands (B5's output stage below 0.005 m/s and
  0.01 rad/s) are unaffected; a controller that sends a steady 0.01 m/s now moves.
- **B5 smoke** (50 Hz, feedforward on, one run each, 2026-10-29), against campaign 2:

  | Scenario | Result | Time (s) | 3D clearance (m) | MPC p99 (ms) | Campaign 2 time / clearance |
  |---|---|---:|---:|---:|---|
  | `v2_open_yard_nav` | succeeded, no contact | 39.7 | 1.724 | 2.4 | 39.7 / 1.728 |
  | `v2_door_105_nav` | succeeded, no contact | 45.2 | 0.119 | 4.1 | 45.3 / 0.118 |
  | `v2_chicane_nav` | succeeded, no contact | 94.5 | 0.120 | 4.8 | 95.5 / 0.100 |

  B5 already commands at least the breakaway from rest, so its runs barely change.
- **Envelope.** `nav_operating_envelope.yaml` keeps `breakaway_linear_mps` 0.0375 and `breakaway_yaw_radps` 0.07 in
  Phase 5A (the golden test reads them); the user rules whether they change before the final baseline run.

## Problems and resolutions

| # | Problem | Cause | Resolution | Where |
|---|---|---|---|---|
| 1 | The plan's friction feedforward (target + sign · jointFriction / damping) does not fit the plant | PhysX joint friction is a coefficient on the joint reaction force, not a torque, and the deficit is stiction (breakaway 0.0375 m/s, kinetic loss 5-10 %): a constant offset would overdrive a moving base | A breakaway offset that fades with the measured wheel speed | Task 1 |
