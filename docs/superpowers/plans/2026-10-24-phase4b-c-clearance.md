# Phase 4B-C Clearance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** B5's plan keeps the 0.05 m arm/panel margin between its stages and along the commanded arm path, and arm/panel rows no longer buy references with slack; then the arm stream's lead is re-measured and re-evaluated.

**Architecture:** Mid-stage collision rows are added in `Controller::linearize` as stage-k rows on (dx_k, du_k) through the half-step model Jacobians. The node's arm stream gets dense points from the plan's model for its first 0.5 s. The slack gate gains an arm/panel tolerance, and the formulation an entry slack weight. Everything sits behind YAML keys whose defaults are the current behaviour, and is validated by unit tests, then a bench check, then about 8 live runs.

**Tech Stack:** C++17 (mobile_manipulator_mpc, Eigen, HPIPM/OSQP), Python launch tests, tools/ Python (mpc_sweep, run_nav_scenario), Unity unchanged.

**Spec:** docs/superpowers/specs/2026-10-24-phase4b-c-clearance-design.md

## Global Constraints

- Every new key defaults to the current behaviour: `collision.midstage_rows: false`, `collision.midstage_horizon_s: 1.0`, `stream_dense_s: 0`, `stream_dense_dt_s: 0.02`, `slack_gate_arm_tolerance_m: -1` (the shared `slack_gate_tolerance_m`), `weights.slack_entry: -1` (the plain `slack`). The 4B-C values are `true`, `1.0`, `0.5`, `0.02`, `0.0001`, `3000`.
- The golden test (`test/data/frozen_wholebody_mpc.yaml`) stays green and unchanged.
- Rows stay in stage order (the slack gate requires it), and the per-stage row count is uniform (`rows_per_stage()`).
- Bench timing rule at 50 Hz: p99 <= 10 ms, max <= 20 ms. Live: R26 (p99 < 20 ms, misses < 1 %).
- Commit only when the suite shows `0 failures`. Never commit to main. A fresh read-only reviewer runs after every task (no container use while a live run is going). Stop Unity Play after live runs. Check thermals with the user before timed runs.
- Kill `control_description`, `ros2_control_node` and `move_group` orphans before mock tests.
- Restore point: tag `pre-4b-c-2026-10-24`; branch `phase4b-c-clearance`.

## Review Focus

1. A mid-stage row on the last covered stage (k = K-1) and with a horizon shorter than one stage: no row beyond stage N-1, and no crash with `midstage_horizon_s < dt`.
2. A mid-stage row whose pair is chosen only at the midpoint, not at the stage: the gate's current violation (at x0) for that pair must still be computed.
3. The dense stream when the plan has skipped stages (`first > 1`, latency) or a re-anchor: dense points must start after point 0 and keep times strictly increasing.
4. `slack_entry` on a row exactly at the margin (distance == margin): one rule, documented (entry applies when distance >= margin).
5. Timing with every row doubled in stages 1..9 on OSQP (selectable backend): the bench reports it, and the defaults stay HPIPM.

---

### Task 1: Mid-stage collision rows

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/collision.hpp` (`DistanceRow`: `bool midstage = false; Eigen::Matrix<double, 1, kNu> control_gradient = Zero;`; `CollisionConfig`: `bool midstage_rows = false; double midstage_horizon_s = 1.0;`; `row_budget` doubled when `midstage_rows`)
- Modify: `src/formulation.cpp` (YAML `collision.midstage_rows`, `collision.midstage_horizon_s`, validation `> 0`; `build()`: a mid-stage row sets `Cu` from `control_gradient` and gets no clearance cost)
- Modify: `src/controller.cpp` (`linearize`: mid-stage rows for stages 1..K-1 from `discrete_step(x_k, u_k, dt/2)`, gradients `g A_half` and `g B_half`, merged after each stage's own rows; `linearization_error` skips mid-stage rows)
- Test: `test/test_collision.cpp`, `test/test_controller.cpp`, `test/test_golden.cpp`, `test/test_mpc_config.py`

**Interfaces:**
- Produces: `DistanceRow::midstage`, `DistanceRow::control_gradient`; `CollisionConfig::midstage_rows`, `CollisionConfig::midstage_horizon_s`; `row_budget(c) = (pairs_per_stage + self) * (midstage_rows ? 2 : 1)`.

- [ ] **Step 1: Write the failing tests**
  - `Midstage.RowGradientsMatchFiniteDifferences` (test_controller.cpp): for a nominal near a box, linearize with `midstage_rows` on; for each mid-stage row, compare `distance + gradient*dx + control_gradient*du` against `pair_distance(discrete_step(x+dx, u+du, dt/2).x_next, ...)` for dx, du of 1e-4 in each component (tolerance 1e-6).
  - `Midstage.KeepsTheIntervalAtTheMargin` (test_controller.cpp): the closed-loop beam loop of `closed_loop.hpp` (1.34 m beam). With midstage on, the minimum over 5 sub-steps per interval of the executed plans is >= 0.05 - 0.001 m. The same loop with midstage off dips below that (assert it, documenting the frozen dip).
  - `Midstage.RowsStayInStageOrderWithAUniformBudget`: rows sorted by stage, at most `rows_per_stage()` per stage, none at stage 0 or >= K.
  - `MpcConfig.MidstageKeysDefaultOff` (test_golden.cpp, frozen copy): `midstage_rows` false, horizon 1.0.
- [ ] **Step 2: Build, watch them fail** (compile errors on the new members). Command: `MAKEFLAGS=-j2 colcon build --packages-select mobile_manipulator_mpc` in the container.
- [ ] **Step 3: Implement** as in Files; the mid states use `now + dt/2` for the world's time (movers), with stage index k.
- [ ] **Step 4: Run the MPC suite.** `colcon test --packages-select mobile_manipulator_mpc`. Expected: `0 failures`, golden unchanged.
- [ ] **Step 5: Commit** `Add mid-stage collision rows behind collision.midstage_rows`.

### Task 2: The stream follows the plan

**Files:**
- Modify: `include/mobile_manipulator_mpc/arm_stream.hpp`, `src/arm_stream.cpp`: `struct StreamPoint { double t; Joints q, v; };` and `std::vector<StreamPoint> plan_stream_points(const std::vector<Joints> & q_stage, const std::vector<Joints> & u_stage, double dt, double offset, double min_lead_s, double dense_s, double dense_dt_s)`. These are the points after point 0 up to stage N. With `dense_s` 0 they are exactly the current stage points (positions q_k, velocities the mean of adjacent controls). Within `dense_s`, points come every `dense_dt_s` on q_k + u_k tau with velocity u_k.
- Modify: `formulation.hpp/.cpp` (`MpcConfig::stream_dense_s`, `stream_dense_dt_s`, YAML keys, validation `0 <= dense_s <= horizon`, `0.005 <= dense_dt_s <= dt`)
- Modify: `src/wholebody_mpc_node.cpp` `stream()`: builds its points from `plan_stream_points` (point 0 and the braking tail unchanged)
- Test: `test/test_replay.cpp` (links arm_stream), `test/test_wholebody_mpc_node.py` (point count with the installed default)

**Interfaces:**
- Consumes: the plan's arm states and controls as `stream()` has them.
- Produces: `plan_stream_points`.

- [ ] **Step 1: Failing tests**
  - `ArmStream.WithoutDensePointsItIsTheStagePoints`: equals the current construction (positions, mean velocities, times `k dt - offset`, skipping stages within `min_lead_s`).
  - `ArmStream.DensePointsKeepTheJtcOnThePlan`: build a trajectory (point 0 + points + tail), then `sample_trajectory` every 5 ms over the first 0.5 s. It stays within 1 mm of q_k + u_k tau, where the stage-point stream deviates by more (assert both).
  - `ArmStream.DenseTimesStrictlyIncreaseAfterSkippedStages` (offset 0.06 s, `first` = 2).
- [ ] **Step 2: Watch them fail.**
- [ ] **Step 3: Implement**; the node keeps `stream_dense_s` 0 by default.
- [ ] **Step 4: MPC suite** `0 failures`; the node test's point-count assertion follows the installed default.
- [ ] **Step 5: Commit** `Let the arm stream follow the plan between stages behind stream_dense_s`.

### Task 3: Collision over references

**Files:**
- Modify: `formulation.hpp/.cpp` (`SlackGate::arm_tolerance_m = -1`, key `slack_gate_arm_tolerance_m`; `Weights::slack_entry = -1`, key `weights.slack_entry`; `build()`: a non-base row with `distance >= margin` gets `slack_entry` when >= 0)
- Modify: `src/controller.cpp` `gate_passes`: tolerance `arm_tolerance_m >= 0 && !row.base ? arm_tolerance_m : tolerance_m` for a pair outside the margin
- Test: `test/test_controller.cpp` (`SlackGate.*`), `test/test_formulation.cpp` (`SlackInside.*` pattern)

- [ ] **Step 1: Failing tests**
  - `SlackGate.ArmRowsEnterOnlyByTheArmTolerance`: an arm row outside the margin with 1 mm slack passes at the frozen tolerance and fails with `arm_tolerance_m` 0.0001; a base row with 1 mm passes in both.
  - `SlackEntry.WeighsArmRowsOutsideTheMargin`: slack weights per row are `slack_entry` for non-base rows with distance >= margin, `slack_inside` for those inside, and `slack` for base rows (frozen: -1 gives `slack`).
  - `MpcConfig.EntryKeysDefaultToFrozen`.
- [ ] **Step 2: Watch them fail.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: MPC suite** `0 failures`.
- [ ] **Step 5: Commit** `Let collision win over references for arm and panel rows behind the gate and entry keys`.

### Task 4: Bench check (checkpoint)

**Files:**
- Create: `docs/experiments/wholebody-mpc/phase4b-c/corpus.json` (the scripted constrained, store S and R4 bags of campaigns 1/1b and the 4B-B isolation runs, plus the 4B-A store S / R4 successes for door passability), `variants.yaml` (base: the installed configuration with `arm_stream_lead_s: 0`; variants: `frozen4bb` = no overlay, `mid`, `mid_dense`, `mid_dense_gate` = all 4B-C keys, and `mid05` = midstage horizon 0.5 s)
- Create: `docs/experiments/wholebody-mpc/phase4b-c.md` (bench section)

- [ ] **Step 1:** `python3 tools/mpc_sweep.py run --corpus .../corpus.json --variants .../variants.yaml --out experiment_runs/bench/4bc --base ros2_ws/src/mobile_manipulator_mpc/config/wholebody_mpc.yaml --repeats 1`, idle host under `caffeinate`.
- [ ] **Step 2:** Table: between-stage share >= 0.045 / >= 0.05 on the shortfall runs, holding cycles per run (doors: R4 and store S), p99 and max. Expected: `mid_dense_gate` holds the margin between stages at p99 <= 10 ms, with door holds no more than `frozen4bb`'s.
- [ ] **Step 3: Checkpoint.** Report to the user. If the doors hold on the bench, present the options (0.5 s horizon, door-frame pairs) before any live run.
- [ ] **Step 4: Commit** `Record the 4B-C bench check`.

### Task 5: Targeted live runs (checkpoint)

**Files:** `docs/experiments/wholebody-mpc/phase4b-c/<config>.yaml` (the chosen 4B-C configuration in full, `arm_stream_lead_s: 0`); `phase4b-c.md` (live section)

- [ ] **Step 1:** Thermals Nominal (user). Then, under `caffeinate -is`, with `--mpc-config <config>`: scripted constrained x3, store S x2, R4 x2, unscripted constrained x1, at 50 Hz (about 8 runs). Stop Play after.
- [ ] **Step 2:** The decomposition for every run (extend `shortfall_analysis.py` to take a run list); criteria: no contact, success, planned clearance >= margin at stages and between them within the gate horizon, measured vs 0.045 m, R26.
- [ ] **Step 3: Checkpoint.** Report the table to the user (what remains, by layer). Commit `Record the 4B-C targeted runs`.

### Task 6: The lead, re-measured (checkpoint)

**Files:** `tools/` analysis script for the end-to-end delay (node reference, sampled from the bag's streams by receive time, against Unity's `arm.csv.gz` positions: the lag maximising the correlation per joint); `phase4b-c.md`

- [ ] **Step 1:** Measure the delay on the Task 5 bags (lead 0).
- [ ] **Step 2:** With the user's agreement, the same 8 runs with `arm_stream_lead_s` = the measured delay.
- [ ] **Step 3: Checkpoint.** Report. The user decides the lead's default. Commit `Record the lead re-evaluation`.

### Task 7: Defaults and records

**Files:** `config/wholebody_mpc.yaml` (the chosen 4B-C values; the frozen copy untouched); `test/test_golden.cpp` `InstalledYamlIsTheChosenConfiguration` (extended), `test/test_mpc_config.py`; `docs/roadmap.md` (decision log, comparability log); `docs/experiments/wholebody-mpc/phase4b-b.md` / `phase4b-c.md` problems and resolutions; ADR 0014 (lead) as decided

- [ ] **Step 1:** Extend the installed-configuration test with the chosen values (RED), then set them (GREEN). Run every package suite and `pytest tools`.
- [ ] **Step 2:** Records as listed. Commit `Make the 4B-C configuration B5's default`.
- [ ] **Step 3:** Hand back to the Phase 4B-B plan, Task 12 (campaign 2, reduced as ruled 2026-10-24), on the new defaults.
