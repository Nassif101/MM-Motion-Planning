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
| fold_back | -6.2 | 37.5 | 0.0 | yes (qualified driving) |
| level_extension | 95.7 | 39.2 | 0.0 | no (hold only) |
| B5's low-beam side pivot (Phase 4B-A) | 71-79 | - | - | not qualified |

**Backward fold (Task 2 checkpoint, user decision 2026-10-30).** `fold` clears the 1.23 m beam
(`v2_low_beam_123_nav`) by only 0.030 m, below the 0.05 m margin. `carry_pose_search --top-max 1.18` finds a
backward fold inside the footprint, `fold_back` = `[0, -0.883, 2.155, 0, -1.272, 0]` rad: level panel, top 1.180 m
(exactly the margin under 1.23 m), static torque shoulder -6.2, elbow 37.5 N m. Qualified the same way (prefix
`foldback`, suite `fold_back`), every row passes:

| Row (3 runs each) | Path error, rad | Hold error, rad | Tilt max, deg | Panel bottom min, m | Result |
|---|---:|---:|---:|---:|---|
| hold (8 s cubic from home, 20 s) still | 0.0086-0.0149 | 0.0006 | 0.03 | 1.139 | pass |
| hold, compact drive | 0.0091-0.0095 | 0.0020-0.0030 | 0.12 | 1.138 | pass |
| hold, extended drive | 0.0091-0.0166 | 0.0015-0.0028 | 0.09 | 1.138 | pass |
| carry -> fold_back at the wholebody_arm limits, still | 0.0146-0.0206 | 0.0015-0.0021 | 0.08 | 1.138 | pass |
| fold_back -> carry, still | 0.0120-0.0124 | 0.0010-0.0028 | 0.08 | 1.139 | pass |
| carry -> fold_back, driving (compact) | 0.0236-0.0240 | 0.0019-0.0033 | 0.15 | 1.137 | pass |
| fold_back -> carry, driving (compact) | 0.0131-0.0240 | 0.0036-0.0040 | 0.11 | 1.135 | pass |
| JTC stream at 50 Hz, +-0.2 rad on all joints, 20 s | 0.0153-0.0199 | - | 0.03 | 0.510 | pass |

It is registered like `fold` and is a driving pose and a seed of the medium layer.

**Torque limits (plan rule: 1.10 x the largest |torque| over the driving poses carry, vertical_carry,
vertical_carry_right, fold, fold_back; user decisions 2026-10-30):** shoulder_lift 24.4, elbow 41.3, wrist_2 10.7 N m;
wrist_1 and wrist_3 take wrist_2's 10.7 N m (gravity has no torque about them in any driving pose, so the rule would
give 0 and the 6-joint variant could not move them); none on shoulder_pan. `level_extension` (95.7 N m) and B5's side
pivot (71-79 N m) are outside them.

*Superseded (user decision 2026-10-30, Task 9):* 60 % of the URDF efforts, which are the Unity arm drives' force
limits: shoulder_lift 96, elbow 54, wrist_1 24, wrist_2 18, wrist_3 12 N m (40 % left for dynamic loads); every
qualified pose, the hold-only `level_extension` included, is within them. The panel's tilt is not bounded.

## Open-loop bench, solver and seeds (plan Task 9, 2026-10-30)

`medium_bench` (open: a recorded B5 run's snapshots; synthetic: the door-then-beam cell) plans every snapshot cold (a
fresh planner, every seed) and warm (one planner carried along); `--warm-only --spacing-s 0.2` is the node's 5 Hz cycle,
`--oracle` seeds the SCP with the motion B5 then executed, `qpgap` solves identical QPs with every backend.
`tools/medium_bench.py` sweeps and summarizes; a plan counts as **progressing** when it is ok and covers half of
min(reach at the reference speed, route left): open-loop feasibility alone rewards plans that wait. Corpus:
[phase5a/corpus.json](phase5a/corpus.json), 278 cold snapshots at 2 s; 2442 cycles at 5 Hz on the six recorded cells.

### Backend (stage A, 4 segments, 0.1 s samples)

| Backend, tolerance | Cold mean / median / p99 (s) | QP share | Note |
|---|---|---:|---|
| DAQP 1e-6 | 0.933 / 1.015 / 1.183 | 3 % | chosen |
| HPIPM dense 1e-6 | - / - / 1.360 | 16 % | |
| OSQP 1e-6 | ~10.3 (98 % of cycles at the 10 s bench cap) | 95 % | out; 1e-8 not run |

On 177 identical QPs (39 variables, ~1550 rows): DAQP 1e-6 is within 6.7e-8 of its 1e-8 objective in 0.50 ms
(median); HPIPM 1e-6 within 2.3e-5 in 3.0 ms; OSQP solves 170 of 177 in 257 ms. `qp_backend: daqp`, tolerance 1e-6.

### Where the time went

| Step | R4 cold cycle (mean) | Change |
|---|---:|---|
| Stage A, finite-difference torque Jacobian | 1.013 s | torque 71 %, merit 14 %, collision 8 %, QP 3 % |
| CasADi-generated torque and exact Jacobian | 0.228 s | torque 1.7 % (identical plans) |
| The merit's collision pass reused by the next linearization | x 0.65 | identical QPs |
| Pose seeds in turn beside the incumbent (`seeds_per_cycle` 1) | - | the periodic full multi-start was the p99 |

5 Hz, six recorded cells, final defaults: **mean 37 ms, p99 55 ms, max 72 ms, none over 150 ms** (gate p99 <= 150 ms).

### Feasibility: what it was and what fixed it

| Finding | Evidence | Outcome |
|---|---|---|
| Infeasible plans were not the start state's | 9 of 49 R4 infeasible started > 1 mm inside a margin, none over a torque limit | - |
| They intrude 2-6 cm; more iterations or a 10x slack weight barely help | max_iter 30: seed convergence 24 -> 76 %, ok 32 -> 38/81; + slack x10: 42 | - |
| The SCP's exits were trust-region collapses on the collision rows | 432 of 462 intruding exits; rejected steps' misprediction almost all collision | contact-point rows (below) |
| One closest-point row cannot see a turn at parallel faces (31 %); the separating-axis gradient is wrong when the axis is the robot's (9 %) | +-dp evaluations at the worst rows (panel and wheels at R4's door) | `contact_points` in B5's collision model, off in B5; tests prove a consistent model, but on R4: ok 29 vs 32 at +45 % time: off in the medium layer for now |
| The "oracle" (B5's executed motion as the seed) was mostly a standing robot | B5 moved a median 0.08 m per 8 s window in that recording | correction: the lever was speed, not geometry |
| Seeds drove the route's polyline at a constant speed with the heading snapping at corners | oracle variants: route at the recorded progress 62/73, route at constant speed 29/73 | `paced_route`: a skid-steer's pace, turning in place beyond 0.5 rad; seeds slowed where the robot on the route has little clearance (seeds only: a slower cost reference made waiting cheap and stalled the beam and gate tests); floor 0.5 (0.2 gave more ok plans, fewer progressing) |
| The clearance hinge (weight 20) made each cycle defer a door passage | angled-door test: 0 and 20 stall, 5-12 pass, 2 loses the gate's side | weight 8 |
| Forward-only speed bound from the first sample, beyond braking | 25 of 30 fixable R4/store failures started reversing (B5 creeps back, up to 77 mm/s) | bounds from the measured speed within decel |
| The remaining mid-plan reversals were the 4-segment curve ringing at stops | stage B: speed failures 26 -> 3 with 8 segments | 8 segments |

### Representation (stage B, one factor at a time, then pairs)

| Variant | Cold ok (of 278) | Fixable left | 5 Hz p99 | 5 Hz ok (of 2442) |
|---|---:|---:|---:|---:|
| 4 segments, 0.1 s (stage A default) | 185 | 37 | 69 ms | 1793 |
| 6 / 8 / 10 segments | 205 / 217 / 224 | 19 / 7 / 5 | 75 / 88 / 109 ms | 1885 / 1905 / 1911 |
| horizon 6 s / 10 s | 199 / 151 | 22 / 69 | 52 / 82 ms | 1873 / 1577 |
| samples 0.15 s | 198 | 24 | 45 ms | 1877 |
| 6 joints | 182 | 39 | 75 ms | 1804 |
| torque rows 0.4 s | 187 | 35 | 61 ms | 1781 |
| **8 segments + 0.15 s samples** | **227** | **3** | **61 ms** | **1942** |

Every ok plan progressed (within 3). Defaults: 8 segments of 1 s, samples every 0.16 s (the horizon's whole fraction
nearest 0.15 s), 8 s horizon (6 s scores higher but shortens the look-ahead the layer exists for), DAQP.

With the final defaults and the 60 % torque limits (more low-beam snapshots become achievable):

| Arm mask | Cold ok | 5 Hz ok | Seeds converged (cold / warm) | 5 Hz p99 |
|---|---:|---:|---:|---:|
| **4 joints, 8 segments** (default) | 224 | 2010 | 27.0 / 25.6 % | 55 ms |
| 6 joints, 8 segments | 222 | 2035 | 22.2 / 22.9 % | 86 ms |
| 6 joints, 6 segments | 222 | 2012 | 24.3 / 25.6 % | 62 ms |

The plan's rule (6 joints only if clearance or convergence improve at a p99 within 10 %) keeps 4 joints the default;
the plans' smallest clearance beyond the margins is 0 in both (they keep exactly the margin in tight places). Both
masks stay options: their flexibility is judged in the closed loop (Task 12) and in free drive, and the cells that
would need the wrists (tilted openings) are Phase 6 (roadmap).

## Problems and resolutions

| # | Problem | Cause | Resolution | Where |
|---|---|---|---|---|
| 1 | The plan's friction feedforward (target + sign · jointFriction / damping) does not fit the plant | PhysX joint friction is a coefficient on the joint reaction force, not a torque, and the deficit is stiction (breakaway 0.0375 m/s, kinetic loss 5-10 %): a constant offset would overdrive a moving base | A breakaway offset that fades with the measured wheel speed | Task 1 |
| 2 | The offset jumped at zero command (+-1e-6 rad/s got the full +-0.268 rad/s) | Sign of the command only | Below 0.005 m/s the offset scales with the command | Task 1 review |
| 3 | Only 45 % of open-loop plans feasible at stage A | The SCP's collision model mispredicted at parallel faces and penetrating pairs; the seeds drove at a constant speed with snapping headings; the clearance hinge deferred passages; the speed bound ignored a reversing start; 4 segments rang at stops | Contact-point rows (available, off), paced and clearance-slowed seeds, hinge 8, speed bounds from the measured speed, 8 segments | Task 9 |
| 4 | A full multi-start took 0.93 s | Finite-difference torque Jacobian (71 %), two collision passes per step, all seeds at once | CasADi torque, the merit's collision pass reused, seeds in turn | Task 9 |
| 5 | The bench's "oracle" suggested the seeds' geometry was the gap | B5 hardly moved in that recording: feasible meant standing | Progress scored beside feasibility | Task 9 |
| 6 | The angled-door test passed by accident | The comfort incumbent went infeasible at the door, so the right swing won; once feasible it deferred the passage every cycle | Clearance hinge 20 -> 8 | Task 9 |
| 7 | 0.15 s samples are not a whole fraction of the 8 s horizon | Stage B set the option directly, past the loader's check | 0.16 s, re-verified on the bench | Task 9 |
