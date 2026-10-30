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
- **Creep.** A slow command keeps part of the kick: the wheel settles near (command + offset) / 2 less its kinetic
  loss, so every command below the breakaway (0.0375 m/s) runs a little fast, about 0.02 m/s for a 0.01 m/s command.
  Commands below 0.005 m/s (B5's dead band) get only a proportional share of the offset (Task 1 review: sign noise
  around zero, the inner wheel of a pivot or a planner's tiny residual commands, must not flip the target between
  +-offset); that scaling leaves every measured case above unchanged (the smallest, 0.01 m/s and 0.02 rad/s, are
  0.071 and 0.107 rad/s at the wheel, above the 0.036 rad/s threshold). B1-B4's controllers send unfiltered small
  commands near the goal (`min_vel_x`, smoother dead band 0) and will now move on them: the final baseline run
  measures that.
- **B5 smoke** (50 Hz, feedforward on, one run each, 2026-10-29), against campaign 2:

  | Scenario | Result | Time (s) | 3D clearance (m) | MPC p99 (ms) | Campaign 2 time / clearance |
  |---|---|---:|---:|---:|---|
  | `v2_open_yard_nav` | succeeded, no contact | 39.7 | 1.724 | 2.4 | 39.7 / 1.728 |
  | `v2_door_105_nav` | succeeded, no contact | 45.2 | 0.119 | 4.1 | 45.3 / 0.118 |
  | `v2_chicane_nav` | succeeded, no contact | 94.5 | 0.120 | 4.8 | 95.5 / 0.100 |

  B5 already commands at least the breakaway from rest, so its runs barely change.
- **B5's own breakaway handling** (spec 5.1 re-check). B5 still raises a command from rest to the envelope's
  breakaway (0.0375 m/s, 0.07 rad/s; `breakaway_output`, and the blocked detector's `own_*` speeds), and Unity now adds
  its kick on top, so B5 starts at about twice the breakaway wheel target, fading as the wheel turns. The smoke runs
  show no effect (times and clearances as campaign 2). Lowering B5's breakaway changes the golden fixture, so it is a
  user ruling with the envelope below.
- **Envelope.** `nav_operating_envelope.yaml` keeps `breakaway_linear_mps` 0.0375 and `breakaway_yaw_radps` 0.07 in
  Phase 5A (the golden test reads them). User decision 2026-10-30 (Task 1 checkpoint): keep the envelope and B5's
  breakaway handling for Phase 5A; decide before the final baseline run.

## Fold pose and torque limits (plan Task 2, 2026-10-30)

**Static torque.** `StaticTorqueModel` (`mobile_manipulator_mpc/statics.hpp`) computes the gravity torque about each
arm joint of everything below it (URDF link masses plus the 3 kg panel); `carry_pose_search` now uses it (its output is
unchanged, diffed before and after). The medium layer's torque constraint will use it.

**Fold.** `fold` = `[0, 1.236, -2.156, 0, 0.920, 0]` rad, Phase 4B-A's fold limit
([phase4b-a.md](../wholebody-mpc/phase4b-a.md#why-the-local-mpc-does-not-fold-the-arm-comfort-only-sweep)): level panel,
top 1.200 m (carry 1.320 m), inside the footprint. Qualified like `carry` and `vertical_carry_right`
(`run_wholebody_arm_qualification.py fold --prefix fold`, 3 runs per row, base friction feedforward on, V1 open test
area; results in [arm-qualification.json.gz](../wholebody-mpc/arm-qualification.json.gz), suite `fold`):

| Row (3 runs each) | Path error, rad | Hold error, rad | Tilt max, deg | Panel bottom min, m | Result |
|---|---:|---:|---:|---:|---|
| hold (8 s cubic from home, 20 s) still | 0.0085-0.0104 | 0.0007-0.0014 | 0.03 | 1.159 | pass |
| hold, compact drive (0.3 m/s, 0.4 rad/s) | 0.0090-0.0164 | 0.0033-0.0035 | 0.11 | 1.158 | pass |
| hold, extended drive | 0.0131-0.0161 | 0.0026-0.0048 | 0.10 | 1.158 | pass |
| carry -> fold at the wholebody_arm limits, still | 0.0065-0.0134 | 0.0006-0.0044 | 0.05 | 1.159 | pass |
| fold -> carry, still | 0.0062-0.0068 | 0.0009-0.0010 | 0.02 | 1.159 | pass |
| carry -> fold, driving (compact) | 0.0066-0.0069 | 0.0037-0.0038 | 0.15 | 1.158 | pass |
| fold -> carry, driving (compact) | 0.0070-0.0071 | 0.0036-0.0049 | 0.13 | 1.156 | pass |
| JTC stream at 50 Hz, +-0.2 rad on all joints, 20 s | 0.0155-0.0245 | - | 0.03 | 0.566 | pass |

Every row passes the carry acceptance (path <= 0.10 rad, hold <= 0.06 rad, no watchdog, tilt < 3 deg, panel bottom
>= 0.15 m, no penetration), so `fold` is registered: `qualified_payload.json` `poses_rad`, the MoveIt SRDF, the
runner's `QUALIFIED_POSES` and Unity's `scenario_place` poses. It is a seed of the medium layer, not B5's set of
poses (B5's posture targets are unchanged).

**Static torque of the qualified poses** (N m, `carry_pose_search --eval`; shoulder_pan carries none):

| Pose | shoulder_lift | elbow | wrist_2 | Driving pose |
|---|---:|---:|---:|---|
| home | 0.0 | 0.0 | 0.0 | yes (B1-B4) |
| vertical_carry / vertical_carry_right | 9.7 | 9.7 | 9.7 | yes |
| carry | 19.3 | -27.2 | 0.0 | yes |
| fold | 22.2 | -31.2 | 0.0 | yes (qualified driving) |
| level_extension | 95.7 | 39.2 | 0.0 | no (hold only) |
| B5's low-beam side pivot (Phase 4B-A) | 71-79 | - | - | not qualified |

**Default torque limits (plan rule: 1.10 x the largest |torque| over the driving poses):** shoulder_lift 24.4,
elbow 34.3, wrist_2 10.7 N m; none on shoulder_pan. `level_extension` (95.7 N m) and the side pivot are outside them,
so the extend seed can only end in a pose inside the limits. wrist_1 and wrist_3 are fixed in the 4-joint mask; their
limits (for the 6-joint variant of Task 9) follow the same rule once the medium package can print all six (Task 7).

## Problems and resolutions

| # | Problem | Cause | Resolution | Where |
|---|---|---|---|---|
| 1 | The plan's friction feedforward (target + sign · jointFriction / damping) does not fit the plant | PhysX joint friction is a coefficient on the joint reaction force, not a torque, and the deficit is stiction (breakaway 0.0375 m/s, kinetic loss 5-10 %): a constant offset would overdrive a moving base | A breakaway offset that fades with the measured wheel speed | Task 1 |
| 2 | The offset jumped at zero command (+-1e-6 rad/s got the full +-0.268 rad/s) | Sign of the command only | Below 0.005 m/s the offset scales with the command | Task 1 review |
