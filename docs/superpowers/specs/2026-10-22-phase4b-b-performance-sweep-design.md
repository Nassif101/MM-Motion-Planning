# Phase 4B-B design: B5 performance sweep and re-freeze for Phase 5

**Date:** 2026-10-22
**Branch:** `phase4b-b-sweep` (from `phase4b-fixes` at `9796dbf`)
**Base formulation:** `b5-frozen-2026-10-22` (`ae3f640`)
**Roadmap:** Phase 4B, "Handed to Phase 4B-B (2026-10-22)"; open decision "MPC rate: 50 or 20 Hz" (2026-10-13);
Phase 4B-A spec Section 12

## 1. Purpose and scope

4B-B gets B5 ready for Phase 5. It settles the MPC rate, horizon, prediction grid and solver with measured timing
headroom, fixes the two items 4B-A handed over (the 3D clearance shortfall and the missing panel-against-arm-link
rows), and re-freezes B5 as the formulation Phase 5 stacks on. Thesis tables come out of it, but they are secondary
(user decision 2026-10-22).

### Decisions taken in the design discussion (2026-10-22)

| Item | Decision |
|---|---|
| Primary deliverable | Phase 5 readiness: the configuration Phase 5 builds on, re-frozen and tagged. |
| Branch | New branch `phase4b-b-sweep` off `phase4b-fixes`; nothing on `main` (the user merges). |
| Live budget | About two campaigns: one shortlist confirmation (Section 7), one re-freeze campaign (Section 8). |
| Method | Approach A: an offline replay bench (Section 3) carries the sweep; the candidate changes are in-house switches (Section 4); live runs confirm and freeze. |
| B6 (acados SQP-RTI) | **Deferred to Phase 6, optional.** Its value is external credibility of the thesis (a standard toolchain as a reference row). For Phase 5 readiness it adds little: HPIPM, partial condensing and a second iteration are built in-house on the same formulation, while B5's collision rows (C++ distance rows, pair prefilter and budget, per-row slack weights, slack gate, side picker) are not CasADi expressions and would need external functions or a reimplementation. Two acados features are taken in-house: the preparation/feedback split (4.5) and a step check on the second iteration (4.4). If the in-house HPIPM backend disappoints on the bench, acados is the fallback. |
| 100 Hz | Offline compute timing only (does p99 fit 10 ms). Unity physics and the controller manager run at 50 Hz, so 100 Hz only shortens the state-to-command latency. |
| Two-segment prediction grid | Out, with one trigger. The user left this call to the designer; reasoning in Section 5.5. |
| Bench timing target | p99 at most half the 50 Hz period (10 ms). The user left this call to the designer; reasoning in Section 5.5. |
| F4 (B-spline) | Decision only, no code: 4B-B delivers a short F4 brief and the user decides before Phase 5 (Section 9). |

### In scope

- The replay bench `mpc_replay` and its Python driver.
- Switches, each defaulting to the frozen B5: HPIPM backend (full and partial condensing), warm start, a second QP
  iteration with a step check, a preparation/feedback split, panel-against-arm-link rows, a deadline-aware solve cap.
- The offline sweep: solver, horizon N, uniform grid spacing, warm start, iterations, split, cap, self rows,
  collision fidelity (pair budget, activation distance, row stride).
- Campaign 1 (shortlist x 50 Hz, best x 20 Hz), the rate and configuration decision, a look at the deadline misses
  that occur while p99 is below the period (host scheduling).
- Campaign 2 (re-freeze), the new tag, V2 timing at the chosen rate.
- Decisions: MPC rate, B6 (recorded above), F4 brief.

### Out of scope

- A live 100 Hz run; any acados or B6 implementation.
- The documented B5 limitations handed to Phase 5: the crooked-door dead end, the late-crossing worker deadlock,
  low passes by lowering (fold or extend), the gravel goal stop.
- Perception (distance field from the lidar), the MoveIt Octomap fault, new scenarios.
- Changes to B1-B4 (frozen at `baselines-2026-10-13`).

## 2. Baseline identity

- Every new behaviour sits behind a `wholebody_mpc.yaml` key (with a node parameter and, where the runner needs it,
  a launch argument) whose default reproduces `b5-frozen-2026-10-22`. Unit tests pin the defaults (Section 10).
- After campaign 1 the defaults flip to the chosen configuration and rate in one commit with a roadmap
  comparability-log entry. Campaign 2 runs on it; on pass the annotated tag `b5-frozen-<date>` marks the Phase 5
  base (local; the user is asked before it is pushed).
- The thesis keeps one row named B5, the re-frozen formulation; the 4B-A freeze is cited as its predecessor.
  HPIPM against OSQP and one against two QP iterations are reported as ablation rows.

## 3. Offline replay bench

### 3.1 What it is

`mpc_replay`, a C++ executable in `mobile_manipulator_mpc` (a tool, not a runtime node; it links the `mpc`
library). It reads one run directory (`bag/`, `task.json`), drives `Controller::step` once per recorded cycle with a
variant configuration, and writes one JSON per run and variant. A Python driver in `tools/` runs the variant grid
over a corpus and builds the tables.

### 3.2 Inputs per cycle (all from the recording)

| Input | Source |
|---|---|
| Cycle times | `/wholebody_mpc/cycle` stamps (a 50 Hz bag replays at 50 Hz) |
| State `x` | Stage 0 of the recorded `/wholebody_mpc/plan`: exactly the state the node gave the controller (base pose and the streamed arm reference) |
| Route and goal | `/plan` messages applied at their stamps (`start`, then `update_goal`); goal pose and panel references from `task.json` and the scenario |
| Executed command | `/cmd_vel` (after the collision monitor), passed through `reset_command` as the node does |
| Base at rest, external hold | From the recorded `/odom` and the recorded cycle status (`blocked`) |
| World | The known boxes from the site and scenario |

`/scenario/movers` is not in the 4B-A bags: cells with movers are left out of the replay corpus. The recorder adds
the topic for campaigns 1 and 2 (Section 6, step 1).

### 3.3 What it measures

The replay is open-loop: the robot follows the recorded trajectory, not the variant's plan. The bench therefore
measures computation and plan quality at the recorded state, not closed-loop outcomes (campaign 1 does that).

- Per-cycle time split (linearize, collision, setup, solve, total; prep and feedback with the split on): p50, p95,
  p99, max per run and per cell.
- Solver iterations, solver failures, fallback cycles, slack-gate holds.
- Largest slack; the **true planned clearance**: forward kinematics of the plan's own stages 1 to the slack-gate
  horizon against the boxes (and, with `self_rows`, the panel against the arm links), its minimum per cycle.
- The linearization error: the linearized row distance against the true distance at the planned stage (the error
  the second iteration targets).
- Tracking proxies: planned route progress per cycle, contour error of the plan.

### 3.4 Rate emulation

Subsampling the recorded cycles emulates 20 Hz (warm shift and cycle spacing change). 100 Hz is compute time only.

### 3.5 Fidelity check (gate)

The frozen configuration replayed on the 4B-A campaign-3 bags must reproduce the recorded cycles: the same status
sequence, and in at least 95 % of cycles a stage-1 plan within 1 mm and 1 mrad of the recorded one and the same
iteration count within 10 %. Below that the bench is fixed before any sweep number is used (checkpoint, Section 6).

Amended 2026-10-22 (checkpoint, user decision): the 4B-A bags cannot meet this bar. They record the solved plan's
stage 0, which differs from the controller's input by the solver's tolerance (up to about 2e-6 m); near active
constraints that offset grows to 1-10 mm at stage 1 (a synthetic loop recorded that way: stage 1 0.34, iterations
0.74; recorded with the exact input: 1.00). After one bench fix (a goal the behaviour tree resends without a new
/plan), status agreement is at least 0.98 on all but one run (0.92), and 93.4 % of 98 309 stage-1 states agree
within 1 mm. The sweep proceeds on the 4B-A corpus (variants compared on identical replayed inputs); the node now
records the exact input as stage 0, and campaign 1's bags re-check the bench at the 1 mm / 95 % bar before the
re-freeze (Section 7).

### 3.6 Timing conditions

Single-threaded, container otherwise idle, Unity not in Play, three repeats per variant (median of the three per
statistic). Absolute times carry the host caveat; the sweep compares variants on the same host and corpus.

## 4. Switches

Each switch is one commit with its own unit tests. Defaults are the frozen B5.

### 4.1 `hpipm_vendor`

A new colcon package that builds BLASFEO and HPIPM from pinned release tarballs (URL and SHA256) with
`ExternalProject`. BLASFEO's target is chosen per architecture: `X64_AUTOMATIC` on x86-64, `ARMV8A_ARM_CORTEX_A57`
on arm64, `GENERIC` as a fallback option. Native `linux/amd64` and `linux/arm64` builds; no emulation for normal
development.

### 4.2 `solver.backend: osqp | hpipm` (default `osqp`)

`HpipmSolver` implements `QpSolver` and maps `QpProblem` onto HPIPM's OCP QP one to one: per stage Q, R, S, q, r,
A, B, c; the general rows Cx, Cu with their bounds; the state and control boxes; each slacked row as a soft
constraint on its one finite bound, with the L1 weight in `zl`/`zu` and the quadratic weight in `Zl`/`Zu`, so the
per-row weights (`slack_inside`) carry over exactly. Options: `mode` (speed, balance, robust), `max_iter`,
tolerance, `warm_start`, `partial_condensing_blocks` (0 = none, otherwise N2 blocks through HPIPM's partial
condensing). Keys under `solver.hpipm`.

### 4.3 `solver.warm_start: true | false` (default `true`)

Both backends. `true` is the frozen behaviour (OSQP warm start from the previous solution).

### 4.4 `qp_iterations: 1 | 2` (default 1), with a step check

With 2, after the first solve the controller re-linearizes the model and recomputes the distance rows around the
first solution's plan, then solves again, warm-started from the first solution. The second iterate is kept only if

- its true (nonlinear) worst margin violation within the slack-gate horizon is not larger than the first's, and
- its true cost is not higher than the first's by more than `iter2_cost_tolerance` (relative, default 1 %);

otherwise the first iterate is used and the cycle reports `iter2_rejected`. A deadline guard skips the second
iteration when the first has used more than `iter2_budget` of the period (default 0.5). The slack gate runs on the
iterate that is kept.

### 4.5 `rti_split: false | true` (default `false`)

The QP is already in deviation variables with a fixed `dx0`. With the split on, the preparation (the nominal rolled
out from the previous plan's predicted stage 1, the collision rows, the QP build) runs right after the command is
published, and the next cycle's feedback step sets `dx0 = measured - nominal` and only solves. The state-to-command
latency drops to about the solve time. When the measured state deviates from the prediction by more than
`rti_max_deviation` (base 0.05 m / 0.05 rad, joints 0.05 rad), the cycle falls back to a full preparation in-cycle.
Single-threaded in the node's timer; the controller's API gains `prepare(...)` next to `step(...)`.

### 4.6 `collision.self_rows: false | true` (default `false`)

Panel rows against the arm links: `upper_arm`, `forearm` and the wrist links that are not adjacent to the panel
mount. The exact pair list is fixed from forward kinematics at `vertical_carry`, `vertical_carry_right` and the
qualified poses (a pair that is always within its margin by construction is excluded and listed). The rows have
their own per-stage budget (`collision.self_budget`) so they never displace obstacle rows, use the 0.05 m margin,
and take the `slack_inside` weighting when already inside it. `clearance_3d` scoring adds them so the criteria count
them.

### 4.7 Deadline-aware solve cap: `solver.deadline_cap: false | true` (default `false`)

Each solve gets the time left in the period minus a publish margin (`solver.publish_margin_s`, default 2 ms); a solve
that runs out counts as a solver failure (the shifted-plan fallback, then braking after `fallback_s`) instead of a
missed deadline. `fallback_s` stays 0.3 s.

### 4.8 `MpcCycle` fields

Added, never changed: `iter2_used`, `iter2_rejected`, `prep_s`, `feedback_s`, `self_pairs`, `timer_lateness_s`
(the timer callback's start against its scheduled time) and `cycle_cpu_s` (the cycle's thread CPU time: a cycle whose
wall time far exceeds its CPU time was descheduled, which lateness alone cannot show for a stall inside the cycle;
amended 2026-10-22 while planning). Old bags still parse.

## 5. Offline sweep and shortlist

### 5.1 Corpus

The 4B-A campaign-3 bags (`experiment_runs/20261022T092209Z` to `20261022T105457Z`, 42 runs at 50 Hz) without the
mover cells (`overhead_course_worker_nav`, the V2 worker scenarios) and the two removed terrain scenarios, plus the
low-pass bags of the Section 10A addendum. Runs are tagged by cell; the four clearance-shortfall runs (scripted
constrained runs 4 and 9, store S runs 40 and 42) and the timing-heavy cells (scripted constrained,
`narrow_gate_home_nav`, `v2_gate_right_swing_nav`) are reported separately.

### 5.2 Stage A: one factor at a time around the frozen configuration

| Factor | Levels |
|---|---|
| Solver | OSQP frozen (eps 1e-3, max_iter 500); OSQP tuned (eps, `adaptive_rho_interval`, rho, max_iter); HPIPM without condensing; HPIPM partial condensing N2 in {2, 4, 5}; HPIPM modes speed and balance |
| Horizon at dt 0.1 s | N in {10, 15, 20, 30} |
| Grid at a 2 s horizon | dt in {0.075, 0.1, 0.15} s |
| Warm start | on, off |
| QP iterations | 1, 2 |
| Preparation/feedback split | off, on |
| Deadline cap | off, on |
| Self rows | off, on |
| Collision fidelity | per-stage pair budget, `activation_m`, rows on every stage against every second stage |

### 5.3 Stage B

A factorial of the stage-A winners: solver {best OSQP, best HPIPM} x iterations {1, 2} x N {best two}, with self
rows on.

### 5.4 Shortlist rules (fixed before the sweep)

1. Hard: the fidelity check passed; solver failure rate at most the frozen configuration's; with self rows, no self
   pair below its margin at the poses the frozen B5 used.
2. Clearance: in the four shortfall runs the true planned clearance over the slack-gate horizon is at least 0.045 m
   in at least 99 % of their cycles (frozen: 0.0325-0.044 m minima).
3. Timing at 50 Hz on every cell, scripted constrained included (frozen 59-75 ms p99 there): p99 at most 10 ms and
   max at most 20 ms on the bench.
4. Tracking not worse: median planned progress per cycle at least 95 % of the frozen configuration's; contour
   error at most 10 % higher.
5. Among the configurations that pass: lowest p99, then the fewest switches on.

The best two or three go to campaign 1. If none passes rule 3, the closest are reported with their causes and the
user decides; the target is not relaxed silently.

### 5.5 Reasoning for the two calls left to the designer

- **Two-segment grid (out, with a trigger).** A non-uniform grid touches every stage-indexed part of B5: the
  interpolated warm shift, per-stage dt in the costs, the references, the slack-gate horizon, the side picker's
  k*-1..k*+1 window and the plan message. Its benefit is a long horizon at low cost, which Phase 5's medium layer
  supplies. **Trigger:** if stage A shows N = 30 clearly better than N = 20 on clearance or progress but failing
  rule 3, a `dt_schedule` (first 5 stages at 0.05 s, then 0.15 s) is added as one extra step and swept. Otherwise it
  is a Phase 5/6 note.
- **p99 at most half the period on the bench.** The medium layer will run in its own process and compete for
  cores, not for the MPC's period (B5 used about 35 % of one core of ten). The half-period target absorbs live
  jitter: 4B-A had deadline misses with a p99 of 9 ms. The live bar stays R26 (Section 7).

## 6. Order of work and checkpoints

One commit per change; a reviewer after every task (as in 4B-A). Checkpoints (CP) stop and report to the user.

1. Branch; the recorder adds `/scenario/movers`; `MpcCycle` gains the Section 4.8 fields.
2. `mpc_replay` and the Python driver; the fidelity check on campaign-3 bags. **CP if fidelity is below 95 %.**
3. `solver.warm_start` and the deadline cap.
4. `hpipm_vendor` and `HpipmSolver`.
5. `qp_iterations: 2` with the step check.
6. `collision.self_rows`.
7. `rti_split`.
8. Stage A and B, the grid trigger check, the shortlist report. **CP.**
9. Campaign 1, its report, the rate and configuration decision. **CP.**
10. Flip the defaults; comparability-log entry.
11. Campaign 2 and its exit report. **CP** (the user rules on any failed criterion).
12. Documentation, the F4 brief, the tag.

## 7. Campaign 1: shortlist confirmation

About 45-50 runs, about 3.5 h. Real Unity, clean tree, unscripted unless listed, movers and timer lateness recorded.

| Cells | Runs |
|---|---|
| `constrained_reconfiguration_mission` scripted (`--references scripted`) and unscripted, `v2_store_room_nav`, `overhead_course_nav` (start `home`), `narrow_gate_home_nav`, `v2_gate_right_swing_nav`, `v2_room_r4_nav` | each shortlisted configuration at 50 Hz: 2 per cell (with three configurations: 1 per cell outside the scripted constrained and store S cells); the best configuration at 20 Hz: 1 per cell |

No frozen-B5 reference runs: campaign 3 holds those numbers. The campaign's bags (exact stage 0) are replayed with their own configuration as a fidelity re-check of the
bench at the Section 3.5 bar (amended 2026-10-22); a failure is reported before the defaults are flipped.

### Per-run criteria

- No robot-environment contact.
- The run succeeds; the only accepted failure is the documented crooked-door dead end in `v2_room_r4_nav`,
  listed with its cause.
- 3D clearance at least 0.045 m (obstacles and, now, the panel against the arm links).
- Timing (R26): cycle p99 below the period and deadline misses below 1 % of cycles.
- Arm tracking within 0.10 rad; zero arm-reference sign flips.

### Decisions

- **Configuration:** the shortlisted configuration that passes every criterion; ties go to the lower p99. None
  passes: the failed criteria are reported with their causes first and the user rules.
- **Rate:** 50 Hz if the chosen configuration passes at 50 Hz, else 20 Hz if it passes at 20 Hz, else the user
  rules. Never below 20 Hz (the arm stream's rate bound).
- **Deadline misses:** `cycle_cpu_s` and `timer_lateness_s` split them into compute and scheduling. If scheduling dominates, the
  options (thread priority in the container, timer type) are reported; nothing changes without the user.

## 8. Campaign 2: re-freeze

The defaults flipped to the chosen configuration and rate (Section 6 step 10), real Unity, clean tree, movers
recorded.

| Set | Cells | Runs |
|---|---|---|
| V1 | `overhead_course_nav` (start `home`), `overhead_course_worker_nav`, `constrained_reconfiguration_mission` unscripted and scripted, `narrow_gate_home_nav` | 5 x 3 = 15 |
| V2 | the 19 V2 scenarios | 19 x 1 = 19 |
| V2 | `v2_gate_right_swing_nav` | 3 |
| V2 | `v2_room_r4_nav`, `v2_store_room_nav` extras | 2 x 2 = 4 |

### Exit criteria

- The 4B-A exit criteria with the user's rulings carried over (Phase 4B-A spec Section 8 as amended):
  `v2_worker_late_crossing_nav` and the crooked-door dead end remain accepted, documented limitations; the narrow
  gate passes on either side.
- No run with a 3D clearance below 0.045 m, the panel-against-arm-link pairs included.
- R26 timing in every run (p99 below the period, misses below 1 %); V2 timing at the chosen rate reported.
- Arm tracking within 0.10 rad; zero arm-reference sign flips.

A failed criterion is reported with its cause before the tag; criteria are not relaxed without the user. On pass:
the local annotated tag `b5-frozen-<date>` (the user is asked before it is pushed).

## 9. Documentation and decisions

- `docs/experiments/wholebody-mpc/phase4b-b.md`: the bench and its fidelity, the stage A and B tables, campaigns 1
  and 2.
- ADR 0013: solver backend, QP iterations and the preparation/feedback split (with the B6 deferral).
- Roadmap: "Phase 4B-B outcome"; decision-log resolutions (MPC rate; 3D clearance; B6 deferred to Phase 6,
  optional); the comparability log; the hand-off to Phase 5.
- `docs/running-the-stack.md`: the new switches and the re-frozen recipe.
- **F4 brief** (roadmap or a short doc): what F4 is, what it would show against B5, its cost, and what the
  campaign data say about the arm stream's smoothness (the main B-spline argument). The user decides F4 from it
  before Phase 5.

## 10. Testing

- **Defaults reproduce the frozen B5:** the existing `mobile_manipulator_mpc` suite (235 tests) stays green
  unchanged; a golden test checks that with every new switch at its default the controller output on fixed inputs
  equals the frozen output.
- **Solvers:** the solver tests made backend-parametric; OSQP and HPIPM agree within tolerance, slacked rows and
  per-row slack weights included; partial condensing equals no condensing.
- **Second iteration:** the step check rejects a worse iterate; the deadline guard skips the second iteration;
  under the 1.34 m beam unit loop the clearance stays at least 0.049 m; a scripted-constrained-like case improves.
- **Self rows:** row geometry against an FCL reference; the `vertical_carry_right` all-joint stream case gets rows
  that keep the commanded panel off the arm links.
- **Preparation/feedback split:** a feedback solve with the predicted state equals the full in-cycle solve; a large
  deviation falls back to an in-cycle preparation.
- **Bench:** a test on a small synthetic bag (`tools/`); the real-bag fidelity check is the step 2 gate.
- **Architectures:** native arm64 build; one emulated amd64 build of `hpipm_vendor` as a compile check (AGENTS.md
  forbids emulation for normal development, not a one-off check).

## 11. Risks

| Risk | Mitigation |
|---|---|
| Replay not deterministic (side-picker latch, slack-gate dwell, blocked holds depend on time) | Recorded stamps and statuses drive the bench; the fidelity check measures what remains. |
| HPIPM soft constraints differ from OSQP's slack rows | Backend agreement tests on slacked rows and per-row weights. |
| The second iteration doubles the collision time | Deadline guard (`iter2_budget`); measured on the bench. |
| Self rows reject tight poses B5 used to reach | Intended; campaign 1 shows the outcome cost and the user rules. |
| Bench timing does not predict live timing | Campaign 1 measures live with timer lateness; the bench target keeps half the period in reserve. |
| Operations | The 4B-A rules: caffeinate long jobs; no `scene_capture` during runs; reviewers never use the container or `pkill` during live runs; `MAKEFLAGS=-j2`; the bracket `pkill -f "[c]ontrol_description"` pattern; stop Play after runs; `--new-epoch` when Unity is not in Play. |

## 12. Amendment 2026-10-23: arm motion feedforward before the re-freeze

Campaign 1 (user ruling at its checkpoint, option A): the 0.065 m margin fails at the 1.05 m doors and returns to
0.05 m; the shortfall's tracking half (scripted constrained run: 12 mm, wrist 1 0.019 rad, a dynamic lag) is fixed at
the actuator before campaign 2. Gravity feedforward is already the default.

- **Where:** the Unity arm actuator (`ArmActuatorController`), as a simulated servo drive's internal feedforward
  ([ADR 0014](../../adr/0014-arm-actuator-feedforward.md), amending ADR 0004). Delivered like gravity: the drive's
  target shifts by torque / stiffness, clamped to the torque limit.
- **Model:** τ = M(q) q̈_ref + C(q, q̇_ref) q̇_ref + G(q) by Newton-Euler over the bodies each joint carries (Unity's
  masses and inertia tensors, the payload included), at the measured configuration with the reference rates. The base's
  reference acceleration (linear and yaw, from the base controller's rate-limited command) enters as the root's motion.
  At rest it is the gravity statics exactly.
- **Acceleration channel:** the JTC commands `[position, velocity, acceleration]`; `unity_arm_system` exports the
  acceleration interfaces and sends one `trajectory_msgs/JointTrajectory` point (positions, velocities,
  accelerations) instead of `sensor_msgs/JointState`; Unity validates finite accelerations. A changed verified
  contract: its tests and the arm qualification are re-run.
- **Switch:** `motion_feedforward`, off until qualified (runner `--arm-motion-feedforward`); default on only by the
  user's decision after the qualification, with a comparability-log entry.
- **Tests:** zero rates reduce to the statics; a single pendulum (I α + m g l); a two-link arm against its closed
  form; base acceleration as an equivalent gravity term; acceleration packets validated; the switched-off path
  unchanged.
- **Qualification:** the carry suite with gravity and motion feedforward against gravity only (`gffon2`), two
  repeats, and one recorded trace.
- **Campaign 1b:** HPIPM N 20, arm rows, margin 0.05 m, gravity and motion feedforward, 50 Hz: constrained unscripted
  x2, scripted x3, narrow gate x2, room R4 x3, store S x3, overhead x2 (15 runs). Then the plan resumes at Task 11
  with the chosen configuration (HPIPM, arm rows, 0.05 m, 50 Hz).
