# Phase 5A: medium-horizon layer (F2, piecewise Bézier) above B5

Spec: [2026-10-29-phase5a-medium-horizon-design.md](../../superpowers/specs/2026-10-29-phase5a-medium-horizon-design.md).
Plan: [2026-10-29-phase5a-medium-horizon.md](../../superpowers/plans/2026-10-29-phase5a-medium-horizon.md).
Branch `phase5a-medium`; B5 baseline: tag `b5-frozen-2026-10-24`.

**Outcome (2026-11-02).** F2 frozen at the local tag `f2-frozen-2026-11-02` ([ADR 0015](../../adr/0015-medium-horizon-layer.md)).
Campaign A: F2 20/21 against B5 alone 12/21 on the four limitations' cells. Campaign B: 33/34 on campaign 2's exit
set against 30/34, no contact. Accepted and handed on: the store_room wedge at door S and the clearance at the tight
passages (about 0.045 m, 7 of 34 runs below it; the execution work), the stopping worker (5C).

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

5 Hz, six recorded cells, final defaults: **mean 37 ms, p99 55 ms, max 65 ms, none over 150 ms** (gate p99 <= 150 ms).

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

Every ok cold plan of the recorded cells progressed (within 3); the synthetic cell's and the warm records carry no
route-left length and count by their status. Defaults: 8 segments of 1 s, samples every 0.16 s (the horizon's whole fraction
nearest 0.15 s), 8 s horizon (6 s scores higher but shortens the look-ahead the layer exists for), DAQP.

With the final defaults, the 60 % torque limits (more low-beam snapshots become achievable) and seeds that start
from the measured speed and yaw rate:

| Arm mask | Cold ok (of 278) | 5 Hz ok (of 2442) | Seeds converged (cold / warm) | 5 Hz p99 / max |
|---|---:|---:|---:|---:|
| **4 joints, 8 segments** (default) | 223 | 2021 | 27.2 / 25.8 % | 55 / 65 ms |
| 6 joints, 8 segments | 223 | 2027 | 22.7 / 23.1 % | 91 / 208 ms (1 cycle over 150 ms) |
| 6 joints, 6 segments | 220 | 2084 | 24.2 / 25.5 % | 65 / 71 ms |

At 10 SCP iterations, the plan's rule (6 joints only if clearance or convergence improve at a p99 within 10 %) kept 4 joints the default
(6 joints with 6 segments plans the most 5 Hz cycles, +63, at +18 % p99: the 6-joint configuration to show);
the plans' smallest clearance beyond the margins is 0 in both (they keep exactly the margin in tight places). Both
masks stay options: their flexibility is judged in the closed loop (Task 12) and in free drive, and the cells that
would need the wrists (tilted openings) are Phase 6 (roadmap).

### Arm mask and iteration budget (follow-up, user decisions 2026-10-30)

Six joints converged less at 10 SCP iterations. Separating the iteration budget from the seeds (wrist seeds: carry
with the panel rolled +-0.5 rad by wrist_3 or pitched +-0.3 rad by wrist_1, `pose+joint:offset` seed names):

| Configuration | Cold ok | 5 Hz ok | Converged cold / warm | 5 Hz p99 / max (uncapped) |
|---|---:|---:|---:|---:|
| 4 joints, 8 segments, 10 / 15 iterations | 223 / 228 | 2021 / 2145 | 27.2-25.8 / 33.8-36.0 % | 55-65 / 85-102 ms |
| 6 joints, 6 segments, 10 / 15 iterations | 220 / 226 | 2084 / 2087 | 24.2-25.5 / 37.0-37.0 % | 65-71 / 91-104 ms |
| ... with the wrist seeds, 15 iterations | 226 | 2052 | 40.3-39.9 % | 95-101 ms |
| 6 joints, 8 segments, 15 iterations | 233 | 2172 | 31.9-33.0 % | 128-138 ms |

The budget was the cause: at 15 iterations six joints converge more often than four. The wrist seeds raise
convergence further and win about 8 % of the plans, but with nothing in the corpus that needs them they dilute the
seeds in turn (fewer plans); they stay available, out of the default list. At 15 iterations 6 joints / 6 segments meets
the plan's 6-joint rule (convergence +3 points at +7 % p99): **the default is now all six joints, 6 segments, 15
iterations** (user decision); 4 joints / 8 segments is the comparison option.

With six joints the symmetric-gate test switched sides once: the plan kept the comfort homotopy (panel pitched) up to
the gate, and when the incumbent turned infeasible the one seed in turn was the left swing. A cycle whose incumbent
turns infeasible now also solves every other pose seed (only on the turn: widening every infeasible cycle put 358
cycles over 150 ms). Under the node's cap, 0.14 s (10 ms left for the last iteration's overrun):

| Configuration, cap 0.14 s | Cold ok | 5 Hz ok | 5 Hz mean / p99 / max | Capped cycles |
|---|---:|---:|---:|---:|
| **6 joints, 6 segments, 15 iterations** (default) | 226 | 2096 | 58 / 142 / 145 ms | 114 (4.7 %) |
| 4 joints, 8 segments, 15 iterations | 231 | 2174 | 76 / 142 / 145 ms | 81 (3.3 %) |

No cycle over 150 ms; the cap loses no plan against the uncapped runs.

## Closed-loop kinematic bench and gate (plan Task 12, 2026-10-31)

`medium_bench closed` runs B5's controller at 50 Hz on the kinematic plant (ideal actuators with the base's static
friction) following the medium planner at 5 Hz under the node's cap (0.14 s); each plan reaches B5 at its start plus its
measured compute time, rounded up to the next B5 cycle. Route and start: each recording's first goal (the synthetic
door-then-beam cell: start to goal). `tools/medium_bench.py gate` checks the kinematic gate. Defaults as set by Task 9
(6 joints, 6 segments, 15 iterations), untuned (ruling; see below). Swing-side cells as spec 7.1: gate_right_swing and
door_105. Side switches are counted over the plans that have a side (a plan without one, comfort or lowered, keeps the
last side: the first count used the planner's plan-to-plan flag and missed right -> lowered -> left; Task 12 review).

| Cell | F2: reached, time | Min clearance (m) | Following | Medium p99 (s) | Capped | Sides | B5 alone on the same plant |
|---|---|---:|---:|---:|---:|---|---|
| door_105 | yes, 42.1 s | 0.047 | 100 % | 0.088 | 1 | right | yes, 41.9 s, 0.128 m |
| door_then_beam | yes, 33.1 s | 0.041 | 100 % | 0.087 | 1 | right | yes, 32.5 s, 0.043 m |
| gate_right_swing | yes, 38.0 s | 0.048 | 100 % | 0.081 | 0 | **left > right** | yes, 28.8 s, 0.071 m |
| low_beam_123 | yes, 32.3 s | 0.112 | 100 % | 0.068 | 0 | - | **no** (holding, 7 slack plans, 0.039 m) |
| low_beam_130 | yes, 32.3 s | 0.145 | 100 % | 0.060 | 0 | - | **no** (holding, 7 slack plans) |
| room_r4 | yes, 62.0 s | 0.047 | 100 % | 0.088 | 1 | left | yes, 59.9 s, 0.109 m |
| store_room | yes, 58.8 s | 0.047 | 100 % | 0.090 | 2 | left > right > left | **no** (120 s timeout) |

Gate: **FAIL** on one criterion: gate_right_swing switches side once after its first choice. Every cell reaches its
goal, no executed plan carries collision slack, no infeasible medium plan was followed, medium p99 0.060-0.142 s. The
cause (per-cycle trace): the incumbent is not chosen left; it starts in comfort and its shoulder pan drifts past the
label's +0.5 rad during the approach (t 5.8-10.4 s, 2-3 m before the gate), back to comfort, and at the gate the right
swing seed wins by more than the 15 % hysteresis (t 16.4 s): the decision at the gate is the cell's right swing, but the
arm swung left and back on the way. The 4-joint option on the same cell holds left and does not reach the goal (120 s).
store_room (not a swing-side cell) goes left > right > left through its doors. door_then_beam's smallest true clearance,
0.041 m for F2 and 0.043 m for B5 alone, is below the campaigns' 0.045 m (B5's between-stage dip, Phase 4B-B).

### Tuning: the conflict-aware comfort pull (user decision 2026-10-31)

The failing criterion was tuned (criteria unchanged): `weights.comfort_free` pulls the arm to the comfort pose where the
comfort pose has clearance on the route and releases the pull from 1.5 m before a place where it would conflict
(`RouteTask::comfort_scale`, computed with the speed schedule from the route, not from the plan). Swept on the gate:

| comfort_free | gate_right_swing sides | door_then_beam clearance (m) | Gate |
|---:|---|---:|---|
| 0 (before) | left > right | 0.041 | FAIL |
| 0.5 | left > right | 0.040 | FAIL |
| 2.0 | left > right | 0.042 | FAIL |
| **5.0** | **left** | **0.046** | **PASS** |

At 5.0 every cell reaches its goal (door_105 42.0 s, door_then_beam 32.9 s, gate_right_swing 33.4 s, low_beam_123 and
low_beam_130 32.3 s, room_r4 63.2 s, store_room 58.5 s), no executed plan with collision slack, no followed infeasible
plan, every smallest clearance >= 0.045 m. The side choices: gate_right_swing holds **left** (the planner's own choice,
margins kept; the cell was designed around B5's right swing); room_r4 (right > left) and store_room (right > left >
right > left) change sides between their doors (not swing-side cells; to watch in the campaign). `comfort_free: 5.0` in
medium.yaml and medium_4joint.yaml.

## Campaign A, first attempt (plan Task 13, 2026-11-01)

`docs/experiments/medium-horizon/phase5a/campaign_a.sh` (7 cells, F2 and B5 interleaved, 3 rounds) ran from 09:09Z;
the user stopped it after round 1 and half of round 2 (21 valid runs, scored by `tools/medium_bench.py campaign`; the
table adds the two F2 runs just before it: the 09:03Z door_105 smoke run and the 09:05Z room_r4 run).
Two script faults first: macOS bash 3.2 treats an empty array as unbound under `set -u` (the first attempt ended after
one run), and a run stalled in a door left the arm where MoveIt cannot plan, so the next 39 runs failed before their
drive; the script now retries such a run once in a new Play epoch (the Phase 4B driver's rule) and resumes by count.

| Cell | F2 | B5 alone |
|---|---|---|
| room_r4 | 1/3 (aborted twice at the R4 door) | 1/2 |
| store_room | 0/2 (aborted at door B; the plans flipped sides 48 and 64 times while it stood) | 2/2 |
| gate_right_swing | 0/2 (timed out in front of the gate) | 2/2 |
| door_105 | 1/2 (the 09:03 smoke run; aborted at door B) | 1/1 |
| low_beam_123, low_beam_130, door_then_beam | 1/1 each (3D clearance 0.043, 0.145, 0.037 m) | 0/1, 0/1, 1/1 |

Every F2 failure stopped in front of a 1.05 m door or the gate, and those runs also broke B5's p99 (11-18 ms) and the
fallback share (route_id fallbacks while Nav2 replanned around the stall).

**Diagnosis** (per-second timelines from the bags; `medium_bench closed --trace` records every plan's seeds):
- *A waiting plan never left.* The incumbent comfort plan crept up to the door, feasible and about as cheap as a swing
  through it (R4: 1.8 against 1.7), so the homotopy hysteresis kept it; the one pose seed a cycle got what the
  incumbent left of the 0.14 s cap, stopped at 11 of 15 iterations and was dropped. At the gate a feasible comfort plan
  0.3 m long waited for 100 s; nothing triggered a wider search (it was feasible, and Nav2's re-routes were same-goal).
- *Live compute was half the bench's.* At the doors the live cycles hit the cap almost every cycle (bench: 1 of 211,
  p90 0.07 s); free-space medium cycles took 16-18 ms in the first runs and 21-33 ms later as the fanless host heated
  (B5's median 2 -> 3.3 ms).
- The bench reproduced the stalls at `--cap-s 0.07`; `comfort_free 0` only moved them to other cells.

**Planner fixes** (commits 22f0efd, 94c5074): a pose seed the cap cuts short resumes from its last iterate on its
next turn; a published plan that waits (the bench's progressing rule, a plan short for a turn in place excepted) widens
the next cycle in turn order with seeds that hold the base while the arm reconfigures, and leaves the incumbent 5
iterations; when the cheapest feasible plan slows below 75 % of its reach, a feasible plan reaching 0.25 m further wins,
with the homotopy hysteresis among those and on the last published side. The bench gained an SCP iteration budget per
cycle (06b2e33): runs repeat exactly on any host, and a budget stands for the live compute (26 iterations a cycle at
the doors on the cool host, about 15 hot).

**Compute options** (closed-loop gate without timing; reach the goal of 7):

| Option (same CPU share) | Cool host | Hot host |
|---|---|---|
| 5 Hz, before the fixes | 3 | 1 |
| 5 Hz, the fixes | 5 | 1 |
| 5 Hz, 4 joints / 8 segments (1.17x per iteration) | 5 (slack plans, side flips) | 2 |
| 5 Hz, 0.25 s collision samples (1.64x) | 7 (r4: 9 slack plans) | 6 |
| 5 Hz, 4 segments (1.42x) | 7 | 3 |
| 5 Hz, both (2.13x) | 7 | 7 (door_then_beam: 4 slack plans; 3 gate switches) |
| **2.5 Hz, cap 0.28 s, full problem** | **7** | **7 (gate passes)** |

Budgets: 26 / 15 at 5 Hz scaled by each option's time per iteration (door B on the idle bench: 2.98 ms default), 52 /
30 at 2.5 Hz. A medium arm margin of 0.06 m (B5's plus 1 cm) cleared door B at 5 Hz but stalled door_then_beam and
store_room; not adopted.

**Decision (user, 2026-11-01; spec Section 12):** `medium.yaml` runs at 2.5 Hz with a 0.28 s cap on this host;
`medium_5hz.yaml` keeps 5 Hz with 0.14 s for a faster host (the user's PC); both plan the full problem. B5's
`medium_stale_s` follows the profile (period + cap + 0.16 s: 0.5 s / 0.84 s, set by `navigation.launch.py`); the medium
p99 criteria are 75 % of the period; summaries record the profile (`medium.rate_hz`, `medium.solve_cap_s`).

The 2.5 Hz default on the closed-loop gate:

| Cell | Budget 52 | Budget 30 | Cap 0.28 s (wall clock) |
|---|---|---|---|
| door_105 | 42.6 s, 0.046 m | 42.6 s, 0.046 m | 42.3 s, 0.047 m, p99 0.098 s |
| door_then_beam | 33.7 s, 0.040 m | 33.7 s, 0.040 m | 34.2 s, 0.043 m, p99 0.166 s |
| gate_right_swing | 29.7 s, 0.049 m, **left > right** | 30.3 s, 0.044 m, left | 29.4 s, 0.049 m, **left > right > left** |
| low_beam_123 | 33.1 s, 0.091 m | 34.8 s, 0.043 m | 32.6 s, 0.047 m, p99 0.147 s |
| low_beam_130 | 33.6 s, 0.047 m | 34.9 s, 0.038 m | 32.4 s, 0.141 m, p99 0.149 s |
| room_r4 | 64.7 s, 0.047 m | 64.9 s, 0.047 m | 64.3 s, 0.047 m, p99 0.092 s |
| store_room | 60.3 s, 0.047 m | 59.9 s, 0.047 m | 59.3 s, 0.047 m, p99 0.099 s |

Every cell reaches its goal with no executed plan with collision slack in all three; the gate passes at budget 30 and
fails on gate_right_swing's side switch at budget 52 and on the wall clock. The smallest true clearances on the
kinematic plant (0.038-0.043 m in door_then_beam and the low beams) are below the live criterion: the live check
measures them.

**Side hysteresis.** The switch: the comfort plan's pan drifted past the side label's 0.5 rad (left) on the approach,
then a right swing 24 % cheaper took over at 12.8 s, past the 15 % homotopy hysteresis. A swing-side switch is a full
arm swing, so a plan on the other side than the last published one now has to be `side_hysteresis` cheaper:

| side_hysteresis | Budget 52 | Budget 30 | Wall clock |
|---|---|---|---|
| 0.15 (as the homotopy one) | gate_right_swing left > right | PASS | gate_right_swing left > right > left |
| 0.3 | PASS | PASS | low_beam_123 holds (7 slack plans) |
| **0.5** | **PASS** | **PASS** | **PASS** |

`side_hysteresis: 0.5` in every profile; gate_right_swing holds left (as at Task 12). `ClosedLoop.DoorBOnTheHostProfileAtTheHotBudget`
runs door B on the V2 scene with the installed profile at budget 30 (it fails with medium_5hz.yaml at 15).

### Live check on the 2.5 Hz profile (2026-11-01, 14:04-14:25Z)

F2 once per cell, each in a new Play epoch, Play stopped and 150 s of cooling between runs (`tools/medium_bench.py
campaign` scoring):

| Cell | Result | 3D clearance (m) | B5 MPC p99 (ms) | Medium p99 (s) | Fallback (cycles) | Sides |
|---|---|---:|---:|---:|---|---|
| door_105 | success, 46.7 s | 0.0479 | 8.2 | 0.243 | 9 (no_plan) | left |
| gate_right_swing | success, 31.9 s | 0.0521 | 4.8 | 0.214 | 16 (no_plan): 1.0 % | **left > right** |
| room_r4 | success, 66.8 s | 0.0456 | **10.1** (3 misses) | 0.282 | 4 (no_plan) | left |
| store_room | success, 66.4 s | **0.0412** | 9.3 | 0.211 | 70 (17 no_plan, 53 route_id) | right |

Four of four (one of five F2 runs on these cells at 5 Hz). The medium p99 is within 75 % of the 0.4 s period
everywhere. The fallbacks were the wait for the timer after a route: at 2.5 Hz up to a period plus the cap (the first
plan, and each Nav2 replan in the store run); the node then planned on a new route when it arrived (72c2242; for a new
goal that delayed the first plan instead, see campaign A). Open: gate_right_swing
switched sides once live (the bench holds left), store_room's clearance at its door S frame (as B5's executed shortfall
in Phase 4B-C), room_r4's B5 p99 just above 10 ms.

## Campaign A (plan Task 13, 2026-11-01, 16:04-19:22Z)

`campaign_a.sh` at 503cab7: the 2.5 Hz profile, 7 cells x F2 / B5 alone x 3 rounds interleaved, every run in a new
Play epoch and 120 s of cooling with Play stopped between runs; 42 of 42 runs recorded, no infrastructure failure.
Scored with `tools/medium_bench.py campaign --since 20261101T160406Z`; every run:
[phase5a/campaign_a_table.md](phase5a/campaign_a_table.md).

| Cell | F2 | B5 alone |
|---|---|---|
| room_r4 | 3/3 | 1/3 (aborted twice at the R4 door) |
| store_room | 2/3 (round 1 aborted at door S) | 2/3 |
| gate_right_swing | 3/3 | 3/3 |
| door_105 | 3/3 | 3/3 |
| low_beam_123 | 3/3 | 0/3 (timed out under the beam) |
| low_beam_130 | 3/3 | 0/3 |
| door_then_beam | 3/3 | 3/3 |
| **All** | **20/21** | **12/21** |

Section 7.1, F2:

| Criterion | Result |
|---|---|
| Door alignment: room_r4, store_room 3/3 each | room_r4 3/3; **store_room 2/3** |
| Swing side: 3/3 each, no switch after the first choice | door_105 3/3, 0 switches; gate_right_swing 3/3 with **1 and 2 switches in rounds 2 and 3** |
| Low pass: 3/3, peak static shoulder torque within the limit | 3/3 each; peak 0.99 of the limit (low_beam_123 round 1) |
| Sequencing: door_then_beam 3/3 | 3/3 |
| No contact | every run |
| 3D clearance >= 0.045 m | **5 runs below**: store_room r1 0.036 (the abort), door_then_beam r1 0.035 and r2 0.043, room_r4 r2 0.041 and r3 0.044 |
| B5 MPC p99 <= 10 ms, misses < 1 % | every run but the store_room abort (10.2 ms) |
| Medium cycle p99 <= 75 % of the period (300 ms) | every run (at most 0.284 s) |
| Fallback < 1 % of the moving time | **18 of 21 runs above** (passing: room_r4 r2 and r3, door_105 r2): the start's 23-26 no_plan cycles everywhere, and in store_room the route_id cycles after Nav2's replans too (36 and 46 in the successes, 118 in the abort) |
| Arm tracking <= 0.10 rad | every run (at most 0.034 rad) |

**Causes of the failures** (bags, `experiment_runs/_diag/timeline.py`):
- *store_room round 1:* through door B, then west to door S in comfort; the plans slowed and turned infeasible at 48 s
  with the base aligned in door S (x 14.95, the frame at x 15.10; corrected in the Task 14 review, first written "0.5 m in
  front") and no seed found a feasible way on; the base crept to about 0.5 m past the frame, B5 held from 58 s and the
  run aborted.
  Give-way and the waiting widening act on feasible plans only: once wedged, nothing backs the forward-only base off
  (Phase 4B's crooked-door dead end, which 5A reduces but does not remove).
- *gate_right_swing switches:* the first "left" is the comfort plan's shoulder pan drifting past the side label's
  0.5 rad on the approach (7-9 s), not a swing; the swing through the gate is right (round 2, 13.4 s). Round 3 adds
  one infeasible right plan for one cycle past the gate (19.4 s).
- *Clearance:* at the door frames, as Phase 4B-C's execution shortfall (the state below the margin before each plan,
  the arm's lag, the dips between collision samples); F2 threads the panel at the margin there by design.
- *Fallback share:* the start, caused by 72c2242 (Task 13 review). For a new goal B5 announces the route before its
  first cycle message, so the node's immediate cycle found B5 not tracking and the timer reset after it put the first
  plan a whole period later: route at 46.42 s, first plan planned at 46.82 s (+0.400 s), solved in 0.04-0.07 s (bags of
  room_r4 r1 and gate r1); 23-26 no_plan cycles in every F2 run, against 4-17 in the live check before 72c2242. Fixed
  after the campaign: the node plans at B5's first cycle after such a route. After the start, route_id fallbacks follow
  Nav2's replans (6-46 cycles in the successes; store_room's successes fail on these alone, 1.1 and 1.5 %). The share is
  scored over all of B5's cycles (`tools/medium_bench.py`), not only the moving ones: a stalled run dilutes it.

**Rulings (user, 2026-11-01).** No rerun of campaign A: campaign B (F2 on campaign 2's exit set) confirms the fixes
made after it (the start fallbacks, the hysteresis, the side labels).
- *store_room 2/3:* accepted and reported; the wedge in front of door S (no feasible plan left, a forward-only base)
  goes to 5C / the execution work.
- *gate_right_swing side switches:* fixed. A side needs a real swing: the label's pan threshold is 0.8 rad (the comfort
  plan drifted to 0.50-0.63 rad, the swings through the gate reach 0.98-1.39 rad), and sides are counted over the ok
  plans only (the bench, `tools/medium_run_stats.py`, the planner's last side). On campaign A's three gate runs this
  leaves one side each (left, right, left).
- *3D clearance below 0.045 m in 5 runs:* accepted and reported (the execution shortfall), as B5's in Phase 4B-B; the
  fix is the execution work after the MuJoCo plant (roadmap "Priorities after Phase 5A").
- *Fallback share:* measured again in campaign B after the C1 fix.

## Campaign B (plan Task 14, 2026-11-01, 20:05-22:40Z with the last cooling)

`campaign_b.sh` at 0b6ba0a: F2 on campaign 2's exit set (34 runs, [Phase 4B-B](../wholebody-mpc/phase4b-b.md#campaign-2-re-freeze-2026-10-24)),
the 2.5 Hz profile, every run in a new Play epoch and 120 s of cooling with Play stopped between runs; 34 of 34 runs
recorded, no retry. Campaign 2's three scripted-reference runs of `constrained_reconfiguration_mission` ran
unscripted (the medium layer gives B5 its references), so that cell has five runs. Every run:
[phase5a/campaign_b_table.md](phase5a/campaign_b_table.md).

**Against campaign 2, with the base change stated.** Campaign 2 ran B5 alone on the base without the friction
feedforward; campaign B runs F2 above B5 with it (Task 1). B5 on the new base is not rerun on this set (Section 7.2; the
final baseline run does that).

| Criterion (Section 7.2) | Campaign 2 | Campaign B |
|---|---|---|
| Successes, at least 30/34 | 30/34 | **33/34**: only `v2_store_room_nav` run 2 aborted |
| No new failure mode | - | Pass by ruling: the abort is new against campaign 2 and covered by campaign A's store_room ruling (below) |
| Contacts, at most campaign 2's | 1 (`overhead_course_worker_nav`, the reversing worker) | **0** |

| Cell | Campaign 2 | Campaign B |
|---|---|---|
| `v2_room_r4_nav` | 2/3 (crooked-door dead end) | 3/3 |
| `v2_store_room_nav` | 2/3 (crooked-door dead end) | 2/3 (wedged aligned in door S) |
| `v2_worker_late_crossing_nav` | 0/1 (accepted, 4B-A) | 1/1 |
| `overhead_course_worker_nav` | 1/2 (the contact) | 2/2 |
| Every other cell | all | all |

**The abort** (`v2_store_room_nav` 20261101T220707Z, `experiment_runs/_diag/timeline.py`): through door B, then west
toward door S. The plans slowed from 40 s. At 46.2 s the panel passed the south post at 0.0419 m (the run's minimum,
inside B5's 0.05 m margin) and B5 held from 46 s; the plans turned infeasible at 48 s and no seed found a feasible way
on. The base stopped aligned in the door's throat (15.16, 8.95, yaw 3.09, 3° off the axis; the frame at x 15.10, the
footprint's front through it) and Nav2 aborted after 48 recoveries (157.7 s; MPC p99 12.5 ms against Section 7.1's
10 ms, no deadline miss). The probable cause is the execution shortfall: a state below the margin that no plan from it
can satisfy, with a forward-only base.
- *Against campaign 2 this is new.* Campaign 2's store abort (20261024T192243Z) is the crooked-door dead end: it stopped
  0.37 m short of the frame, about 36° off the axis (15.47, 8.96, yaw 2.51).
- *Against campaign A it is close.* Campaign A round 1 also wedged aligned at door S with no feasible plan, but with the
  base through the door (x 14.95 at 48 s, stopped about 0.5 m past the frame). Its ruling (accepted, to 5C and the
  execution work) covers it only if "the wedge at door S" includes a stop in the throat (ruled: it does, below).

**The worker cells** (B5's job in 5A, Section 7.2; footprint clearance to the worker):

| Cell | Campaign 2 | Campaign B |
|---|---|---|
| `overhead_course_worker_nav` | 0.11-0.12 m, one contact | 0.126 and 0.151 m, no contact |
| `v2_worker_crossing_nav` | 0.34 m | 0.33 m |
| `v2_worker_late_crossing_nav` | 0.00 m, aborted | 0.00 m, succeeded |
| `v2_worker_stops_in_lane_nav` | 0.06 m | **0.025 m** |

The stopping worker came 3.5 cm closer: recorded for 5C, not fixed in 5A.

**The other metrics (Section 7.1's, reported):**
- *3D clearance below 0.045 m in 7 runs*: `overhead_course_worker_nav` run 2 0.0407 m, `v2_door_105_nav` 0.0421 m,
  room_r4 runs 2 and 3 0.0444 and 0.0423 m, store_room 0.0441, 0.0419 (the abort) and 0.0432 m; all the panel at a door
  frame or gate post. On the 31 runs both campaigns share (campaign 2's three scripted runs left out), campaign 2 had one
  below (store S 0.0427 m) and its lowest was 0.0427 m; campaign B's lowest is 0.0407 m. F2 threads the panel at the
  margin by design wherever the arm must move for a passage, so clearance falls from campaign 2's B5 alone to about
  0.045 m at every tight passage: narrow_gate 0.150-0.153 to 0.045-0.046 m, overhead_course 0.130-0.155 to 0.045-0.047 m,
  door_105 0.118 to 0.042-0.047 m, room_r4 0.049-0.106 to 0.042-0.046 m, store_room 0.043-0.110 to 0.042-0.044 m. The
  execution shortfall (Phase 4B-C) then takes it below 0.045 m. The constrained mission rose from 0.064-0.065 m
  (campaign 2, unscripted) to 0.078-0.089 m. Ruled for campaign A (accepted, to the execution work).
- *Fallback share after the C1 fix:* the start is 3-6 no_plan cycles in every nav run (campaign A: 23-26). 27 of 34
  runs are under 1 %. Above it: the five constrained missions (9-11 no_plan cycles of about 670, a 13.5 s run) and two
  store_room runs on the route_id cycles after Nav2's replans (60 in run 1, 430 in the abort).
- *Side switches* (no Section 7.2 criterion): `v2_gate_right_swing_nav` one side (left), `v2_door_105_mission` one
  (right). Two runs count a switch: `overhead_course_nav` run 1, where the plans unwinding the start pose's pan (+1.57
  rad, 0-1.7 s) count as left before the gate's right swing (16.9 s), and store_room run 3, a right swing through both
  doors, then a left fold at 57.5 s past door S (pan 0.80 rad, at the label threshold) after its near-wedge there. Neither flips a side within a
  passage; the count is over the whole run.
- *Timing and tracking:* no deadline miss in any run; MPC p99 2.9-8.0 ms in every run but the abort (12.5 ms, above
  Section 7.1's 10 ms); medium p99 at most 0.285 s (75 % of the period is 0.30 s); arm reference offset at most 0.043 rad.
- *Load, plans and torque:* B5's node used 13-24 % of a core (campaign 2: 14-29 %), the abort 33 %. Infeasible plans
  in 8 runs, most of them in the abort (252 of 366 plans), store_room run 3 (20 of 169, a near-wedge at door S that got
  through) and `overhead_course_worker_nav` run 2 (7 of 144). Capped medium cycles at most 6 per run. Peak static
  shoulder torque at most 0.70 of the limit (store_room run 3).
- *Times* match campaign 2's within a few seconds (door_105 46.3 against 45.3 s, room_r4 66 against 64 s, scaffold
  tunnel 72 against 76 s, through_building 17.8 against 21.5 s), except `overhead_course_worker_nav` run 2: 57.4 against
  50.3 s, blocked by the worker for 4.3 s.

**Rulings (user, 2026-11-02).** Campaign B passes Section 7.2.
- *The store_room abort:* campaign A's store_room ruling covers a wedge in door S's throat too. The mechanism is the
  same (the panel inside the margin, no plan from that state, a forward-only base); it goes to 5C and the execution
  work.
- *The clearance at the tight passages:* accepted as for campaign A, and stated in ADR 0015 and the thesis: F2 buys
  success at the passages with the clearance margin (about 0.045 m where B5 alone kept 0.10-0.15 m) until the execution
  work closes the shortfall.
- *The side count:* left as it is, with the note above (it counts over the whole run, not per passage).

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
| 8 | Six joints converged less than four | 10 SCP iterations: the larger problem needs more steps (not the seeds) | 15 iterations | Task 9 follow-up |
| 9 | With six joints the symmetric gate switched sides once | Seeds in turn: when the incumbent turned infeasible the one seed solved was the wrong side | Widen the cycle to every seed on that turn | Task 9 follow-up |
| 10 | Capped cycles ended at 152 ms | The last SCP iteration overruns the deadline by 2-4 ms | Cap 0.14 s | Task 9 follow-up |
| 11 | The gate's swing-side switch count read 0 where the arm went left and back | The planner's homotopy_switched compares a plan with the last one only (right -> lowered -> left is no flip) | Count over the plans with a side (SideTracker); the gate then fails on gate_right_swing | Task 12 review |
| 12 | The closed-loop gtests flaked (0.0443 m against 0.045 m) | The medium plans' latency and cap ran on the wall clock (RUN_SERIAL alone did not help) | The gtests plan with a fixed 0.14 s latency and no cap (ClosedLoopOptions::fixed_latency_s); the bench keeps the measured latency and the node's cap | Task 12 review |
| 13 | The six-joint plan swung left and back on the approach to gate_right_swing | The weak comfort pull let the pan drift past the side label's 0.5 rad far from any conflict | weights.comfort_free 5.0 (released before conflicts) | Task 12 tuning |
| 14 | Campaign A stopped after its first run | macOS bash 3.2: an empty array is unbound under set -u | `${epoch[@]+"${epoch[@]}"}` | Task 13 |
| 15 | 39 campaign runs failed before their drive | A run stalled in a door left the arm where MoveIt cannot plan; the script had no new-epoch recovery | Retry once in a new Play epoch; resume by count | Task 13 |
| 16 | F2 waited in front of the 1.05 m doors and the gate live | A feasible waiting incumbent; one pose seed a cycle, cut by the cap and dropped; waiting about as cheap as a swing; the hysteresis | Seeds resume; waiting widens with hold-first seeds; a moving plan wins (rows 17-18) | Task 13 diagnosis |
| 17 | The comfort plan still won while it slowed toward door B | Give-way only for a plan that waits; it turned infeasible before it waited | Give way below 75 % of the reach | Task 13 |
| 18 | Swing seeds never resumed in a waiting cycle | The waiting incumbent took 15 iterations; the widened cycle dropped the iterates of seeds it never reached | Incumbent 5 iterations while waiting; an iterate is dropped only once its seed is solved | Task 13 |
| 19 | The bench could not compare variants under a tight cap | The wall-clock cap: the same run passed or failed by chance | An SCP iteration budget per cycle (reproducible) | Task 13 |
| 20 | At the hot host's compute F2 stalled at 5 Hz whatever the selection | Half the bench's compute live on the fanless host | 2.5 Hz with a 0.28 s cap on this host; medium_5hz.yaml for a faster one (user decision) | Task 13 |
| 21 | gate_right_swing switched sides once on the 2.5 Hz profile | A pan drift labelled the comfort plan left; a right swing 24 % cheaper passed the 15 % hysteresis | side_hysteresis 0.5 for a switch of swing side | Task 13 |
| 22 | Live, F2 fell back for up to 0.68 s after each new route at 2.5 Hz | The node planned on its timer only: a period plus the cap before B5 had a plan for the route | Plan when a new route arrives, the timer restarted | Task 13 live check |
| 23 | Every campaign A F2 run started with 23-26 no_plan cycles | 72c2242's immediate cycle met B5 not yet tracking (B5 announces a new goal's route before its first cycle) and its timer reset delayed the first plan by a period | Plan at B5's first cycle after a pending route; reset the timer only after a published plan (test_5) | Task 13 review |
| 24 | A hysteresis could bring back a plan that slows while the cheapest moves | Give-way acted only when the cheapest plan slowed; both hysteresis checks then chose among every feasible plan | When the cheapest moves, plans that slow are not eligible; the last side only from ok plans | Task 13 review |
| 25 | The gate's comfort plan counted as a left swing | The pan side label at 0.5 rad caught the comfort plan's drift (0.50-0.63 rad); one infeasible plan for one cycle counted too | Label at 0.8 rad; sides over ok plans only | Task 13 rulings |
