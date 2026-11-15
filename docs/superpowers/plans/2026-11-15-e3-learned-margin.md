# E3: Learned Tracking Margin Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Measure B5's tracking error per collision row on the MuJoCo plant, sweep fixed margins on paired tightness
cells, and, after a go at CP1, put a margin learned from the commanded motion into B5's collision rows.

**Architecture:** A row trace in B5 (behind its own switch) records each traced row's distances, gradient and the
plan's commands; a Python tool turns the logs into tracking and linearization errors per motion regime. New scenarios
give the same gap in two approach motions at four tightness steps. After CP1 a `MarginModel` (parameters as data, one
file per plant) sets each row's margin from the commanded motion: first as a constant from the previous plan (A), then
with its slope with respect to the stage's commands in the row (B).

**Tech Stack:** C++17 (`mobile_manipulator_mpc`, `mobile_manipulator_campaign`, `mobile_manipulator_medium`),
gtest/ament, Python 3 (`tools/plant`; standard library on the host's 3.9, numpy and scipy in the container for the
fit), pytest in the container, MuJoCo plant.

**Spec:** `docs/superpowers/specs/2026-11-15-e3-learned-margin-design.md`

## Global Constraints

- Branch `e3-learned-margin`; one commit per task or fix; no commits on main.
- Everything new is off by default: no `--row-trace`, `margin_model: ""` in `wholebody_mpc.yaml`. With both off B5
  reproduces today's runs frame by frame.
- C++ for runtime and library code; `tools/` in Python. `margin_analysis.py` is standard library only and Python
  3.9-compatible; `fit_margin.py` runs in the container, like `identify_tracking.py`.
- Builds and ctest `-DCMAKE_BUILD_TYPE=RelWithDebInfo` with `MAKEFLAGS=-j3 colcon build --parallel-workers 1`.
- pytest in the container needs ROS sourced: `source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash`.
- Ensembles: realistic mode, 20 seeds, stall 45 s, 4 jobs, F2 (`--kinds f2`); never overlap two timed ensembles or
  Unity Play; power and lid open; each long run starts only with the user's go-ahead. Runs go to
  `experiment_runs/e3/<set>` (git-ignored).
- Baselines for M4 and M9: E2's adopted sets (the candidate's runs under `experiment_runs/e2/`; the exact directories
  are in `docs/experiments/execution-fixes/e2.md`, "Acceptance").
- Values from the spec: sweep margins 0.05, 0.04, 0.03, 0.02 m (arm and panel; base stays 0.12 m; the medium
  planner's `margins.arm` equal to B5's); lookaheads 0.1, 0.2, 0.5, 1.0 s (stages 1, 2, 5, 10); safe = zero contacts
  and realised clearance never below 0.02 m; coverage target 95 %; mu 0.6 for the sweep, 0.4 / 0.6 / 0.8 for the fit
  and M3, M4, M8; seeds 1-20 to fit and choose, 21-40 fresh.
- Statistics: Fisher exact one-sided, p < 0.05.
- Motion regimes (from the plan's commands over stages 0..k-1, largest magnitude): **straight** |v| <= 0.15 m/s and
  |omega| <= 0.05 rad/s and every |joint speed| <= 0.05 rad/s; **turning** |omega| > 0.2 rad/s; **arm** any |joint
  speed| > 0.2 rad/s (a record can be both turning and arm); **other** the rest.
- After every task: commit, write a self-contained review prompt, wait for the user's review, write the response
  (the review files stay outside git); only then the next task. Stop at CP1-CP4 for the user's ruling.
- Records as problems are solved: `docs/experiments/execution-fixes/e3.md` (problem / cause / resolution table); the
  roadmap's decision log per ruling. Every attribution is checked against the data before it is written.

Container helpers (recreate in the session scratchpad): `cbuild PKG...`, `ctest_pkg PKG REGEX`, `cpytest FILE`, as in
the E2 plan. Container: `mm-motion-planning-ma-robot-sim-1`.

**Stage 2 is conditional.** Tasks 6-11 are written for the spec's design. CP1 rules go or no-go and fixes the
lookahead, the rows and the feature set; each change is a plan amendment commit before the affected task starts. On a
no-go only Task 12 follows.

## Review Focus

1. **A row whose pair leaves the world or the row set between cycles** (a mover gone, a pair dropped by the budget):
   the trace and the analysis must skip it, not report an infinite or stale distance. Test in Tasks 1 and 2.
2. **Zero and reversed commands in the margin:** at v = omega = 0 and all joint speeds 0 the margin is the floor plus
   the row type's constant, finite, with a zero slope; (-v, -omega) gives the same margin as (v, omega). Test in
   Task 7.
3. **A margin above the row's activation distance or below zero:** the loader refuses `upper` >= `activation_m` and
   `floor` <= 0. Test in Task 7.
4. **A large command change between cycles under approach B:** the gate must hold a plan that passes its linearized
   rows and violates the exact margin at its own commands. Test in Task 9.
5. **Runs that end early** (a stall, a contact, a timeout): their trace is shorter than the lookahead at the end; the
   analysis drops records without a realised state, and the split by run keeps such runs on one side. Test in Tasks 2
   and 6.

---

## Stage 1: measure

### Task 1: Row trace in B5 and the run log

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/controller.hpp` (`ControllerOptions`,
  `CycleOutput`), `ros2_ws/src/mobile_manipulator_mpc/src/controller.cpp`
- Modify: `ros2_ws/src/mobile_manipulator_campaign/src/session.cpp`, `.../include/mobile_manipulator_campaign/session.hpp`,
  `.../src/plant_campaign.cpp`
- Modify: `tools/plant/plant_campaign.py`, `docs/experiments/mujoco-plant/stream-protocol.md`
- Test: `ros2_ws/src/mobile_manipulator_mpc/test/test_controller.cpp`,
  `ros2_ws/src/mobile_manipulator_campaign/test/test_session.cpp`, `tools/plant/test_plant_campaign.py`

**Interfaces:**
- Produces (C++):
  ```cpp
  struct RowTrace {            // one traced row of the solved plan
    std::string pair;          // "<primitive>|<link>|<obstacle>", gate_pair's format
    std::string type;          // "arm" | "panel" | "base" | "self" | "floor"
    int stage;                 // 1, 2, 5 or 10
    double margin;             // the row's margin (m)
    double d_lin;              // row.distance + row.gradient * (x_plan[stage] - x_nominal[stage])
    double d_plan;             // collision_.pair_distance(x_plan[stage], row, world, now)
    Eigen::Matrix<double, 1, kNx> gradient;  // row.gradient
  };
  // ControllerOptions: bool row_trace = false; int row_trace_every = 5;   (cycles; 5 = one stage period)
  // CycleOutput: std::vector<RowTrace> row_trace;           // empty on cycles not traced or without a solve
  //              std::vector<State> row_trace_x;            // the plan's states at stages 1, 2, 5, 10
  //              std::vector<Control> row_trace_u;          // the plan's commands at stages 0..9
  ```
  Type: `base` when `row.base`; `floor` / `self` from the obstacle name; `panel` when `row.link ==
  link_index("panel")`; else `arm`. Mid-stage and contact-point rows are not traced. Rows with a non-finite
  `d_plan` are left out.
- Produces (run log, State header, only on traced cycles with `--row-trace`): `rows` = one
  `[pair, type, stage, margin, d_lin, d_plan, g0..g8]` per `RowTrace`; `rows_x` = 4 x 9 floats; `rows_u` = 10 x 8
  floats. Session Hello gains `row_trace` (the period in cycles, 0 off).
- Produces (CLI): `plant_campaign --row-trace` (C++ and the Python wrapper, passed to every run).

- [ ] **Step 1: Write the failing controller test** `RowTrace.RecordsTracedStagesOfTheSolvedPlan` in
  `test_controller.cpp`: on the existing beam fixture with `row_trace = true, row_trace_every = 1`, after one cycle
  every entry has `stage` in {1, 2, 5, 10}, `margin` equal to the config's margin for its type, `|d_lin - d_plan| <
  0.05`, and `row_trace_x.size() == 4`, `row_trace_u.size() == 10`. A second test `RowTrace.OffChangesNothing`: the
  same two cycles with and without the trace give identical `u0` and `gate_slack`. A third,
  `RowTrace.SkipsAPairThatLeftTheWorld`: a row whose obstacle is removed before the trace is evaluated is absent.
- [ ] **Step 2: Run** `ctest_pkg mobile_manipulator_mpc RowTrace` — expected: FAIL (no member `row_trace`).
- [ ] **Step 3: Implement** the trace in `Controller` after the solve, inside the diagnostics timing bracket (its
  time goes to `timing.diagnostics_s`, not `total_s`), every `row_trace_every`-th cycle.
- [ ] **Step 4: Run** the three tests — expected: PASS; then the whole `mobile_manipulator_mpc` suite — PASS.
- [ ] **Step 5: Write the failing session test** `Session.RowTraceLeavesOtherFramesIdentical` in `test_session.cpp`:
  two 3 s runs of one cell and seed, with and without `row_trace`; after removing `rows`, `rows_x`, `rows_u` from the
  State headers and `row_trace` from Hello, the logs are byte-identical; the traced log has `rows` on every fifth
  State frame.
- [ ] **Step 6: Implement** `SessionConfig::row_trace`, the header fields, `--row-trace` in `plant_campaign.cpp` and
  in `plant_campaign.py` (`runs_of(..., row_trace=False)`, its test in `test_plant_campaign.py` asserting the argument
  reaches the command line). Document the fields in `stream-protocol.md`.
- [ ] **Step 7: Run** `ctest_pkg mobile_manipulator_campaign Session` and `cpytest tools/plant/test_plant_campaign.py`
  — PASS.
- [ ] **Step 8: Commit** `E3 Task 1: a row trace in B5 (traced rows' distances, gradients and the plan's commands) behind --row-trace`.

### Task 2: The analysis tool

**Files:**
- Create: `tools/plant/margin_analysis.py`, `tools/plant/test_margin_analysis.py`

**Interfaces:**
- Consumes: Task 1's State header fields; `model_rollout.records(log_path)`, `model_rollout.rollout(recs, i0, i1,
  inputs, step)`, `model_rollout.container_step(params=...)` (the time convention in that module's docstring).
- Produces:
  ```python
  def trace_records(log_path):
      """One dict per traced row with a realised state: run, t, pair, type, stage, lookahead_s, margin, d_lin,
      d_plan, regime, features (dict: v2, w2, qd2 = the largest squared base speed, turn rate and joint speed of
      rows_u over stages 0..stage-1), e_track, e_lin, e_total (m, positive = closer than predicted).
      Rows whose realised state lies beyond the log's end are dropped."""
  def regime(u_window):            # -> "straight" | "turning" | "arm" | "turning+arm" | "other" (Global Constraints)
  def quantile_table(records, by=("type", "lookahead_s", "regime"), q=(0.5, 0.95, 0.99)):   # -> list of row dicts
  def history_dependence(records): # -> per type: the rank correlation of e_track with the previous window's features
  def main(argv=None):             # report RUN_DIR... --out DIR [--tracking-model FILE]; writes margin_analysis.md
  ```
  Definitions (decisions the tests pin):
  - `e_track = -gradient . (x_true(t + lookahead) - x_model)`, `x_model` = B5's model rolled from the true state at
    the cycle with the commands that followed (`inputs="cmd"`), the plant's tracking model as the run used it;
  - `e_lin = d_lin - d_plan`;
  - `e_total = d_lin - d_true`, `d_true = d_plan - gradient . (x_plan[stage] - x_true(t + lookahead))` (first order;
    the report states it).

- [ ] **Step 1: Write the failing tests** on a synthetic log built in the test (a helper writing State frames with
  `plant_stream`): `test_tracking_error_recovers_a_known_offset` (the true state displaced 0.004 m along a row's
  gradient gives `e_track == pytest.approx(-0.004, abs=1e-6)` with an identity model step stub);
  `test_linearization_error_is_d_lin_minus_d_plan`; `test_regime_boundaries` (v 0.15 -> straight, omega 0.21 ->
  turning, a joint at 0.21 with omega 0.21 -> turning+arm); `test_rows_past_the_end_are_dropped`;
  `test_a_pair_absent_in_later_frames_is_kept_once` (Review Focus 1: no error from a missing pair);
  `test_quantile_table_groups_and_counts`.
- [ ] **Step 2: Run** `cpytest tools/plant/test_margin_analysis.py` — FAIL (module missing).
- [ ] **Step 3: Implement** `margin_analysis.py`.
- [ ] **Step 4: Run** the tests — PASS.
- [ ] **Step 5: Smoke run** one traced run of `v2_low_beam_123_nav` (F2, seed 1) and `report` on it: the table has
  panel rows at all four lookaheads; paste its head into e3.md ("Stage 1, smoke").
- [ ] **Step 6: Commit** `E3 Task 2: margin_analysis.py: tracking and linearization error per traced row, by row type, lookahead and motion regime`.

### Task 3: Tightness cells

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_navigation/config/scenarios_site_v2.yaml` (new scenarios, prefix `e3_`)
- Create: `docs/experiments/execution-fixes/corpus_e3_tight.json`
- Modify: `ros2_ws/src/mobile_manipulator_campaign/test/test_scenario_setup.cpp`
- Record: `docs/experiments/execution-fixes/e3.md` ("Tightness cells")

**Interfaces:**
- Produces: 16 scenarios `e3_<geometry>_<motion>_<step>`, geometry `beam` | `gap`, motion `straight` | `turn`, step
  `1`..`4` (1 easiest); the corpus lists them as `{"cell": NAME, "scenario": NAME, "synthetic": true}`, as
  `v2_door_then_beam_nav` is listed in `corpus_campaign_a.json`.
- Geometry rules:
  - every cell in the open delivery yard, obstacles as scenario `obstacles` entries (the format of
    `v2_low_beam_123_nav`), carry start pose;
  - `beam`: one overhead beam 0.2 x 3.0 x 0.2 m; `gap`: two full-height boxes beside the panel's path, the base's
    0.12 m margin clear on both sides at every step (the base rows must not bind);
  - `straight`: start 5 m before the obstacle on its axis; `turn`: the same obstacle 1.5 m after a 90-degree corner
    of the route (the corner free of other obstacles);
  - a pair differs only in the route: the obstacle's dimensions and the height or width at each step are equal.
- Steps: found by the dry run below; step 1 = the loosest value at which a 0.05 m margin passes 5 of 5 on the straight
  cell with the panel inside 0.02 m of its margin; steps 2-4 tighter by 0.02 m each.

- [ ] **Step 1: Dry run** (short, with the user's go-ahead): the straight beam and the straight gap at five trial
  values around today's tightest (beam z 1.23 m; the gap from the scaffold tunnel's width), margin 0.05 m, 5 seeds,
  F2. Record the table in e3.md and fix step 1-4 values from the rule above.
- [ ] **Step 2: Write the failing test** `ScenarioSetup.E3CellsLoadInPairs`: every `e3_*` scenario loads; for each
  geometry and step the `straight` and `turn` scenarios have obstacle lists equal up to a rigid transform (compare
  sorted sizes and heights); each has a synthetic route.
- [ ] **Step 3: Add** the scenarios and the corpus; run the test — PASS.
- [ ] **Step 4: Check step 1** on the plant: each of the four step-1 cells, margin 0.05 m, 5 seeds: reached 5 of 5,
  zero contacts (spec Section 5, Stage 1). Record in e3.md.
- [ ] **Step 5: Commit** `E3 Task 3: sixteen tightness cells (beam and gap, straight and after a turn, four steps) and their corpus`.

### Task 4: The fixed-margin sweep

**Files:**
- Create: `docs/experiments/execution-fixes/configs/b5_margin_{040,030,020}.yaml` (copies of the adopted
  `wholebody_mpc.yaml` with `collision.margin_m` 0.04 / 0.03 / 0.02),
  `docs/experiments/execution-fixes/configs/medium_margin_{040,030,020}.yaml` (copies of `medium.yaml` with
  `margins.arm` likewise)
- Create: `docs/experiments/execution-fixes/e3_ensembles.sh`
- Create: `tools/plant/margin_sweep.py`, `tools/plant/test_margin_sweep.py`
- Modify: `tools/plant/plant_campaign.py` (`--medium-config`, passed to every run; the C++ runner already has it),
  `tools/plant/test_plant_campaign.py`

**Interfaces:**
- Produces (script): `e3_ensembles.sh sweep` (the 16 cells x 4 margins x seeds 1-20, F2, mu 0.6; tags `m050`,
  `m040`, `m030`, `m020`; `--row-trace` on `m050` and `m030`; out `experiment_runs/e3/sweep`); `e3_ensembles.sh
  variant B5_CONFIG MEDIUM_CONFIG TAG [CORPUS]` for later tasks. Resumable like `e1_ensembles.sh`.
- Produces (Python):
  ```python
  def sweep_table(directory):
      """Per cell and margin tag: n, reached, contacts (runs with any), min realised clearance (m), safe (bool:
      zero contacts and clearance >= 0.02 in every run)."""
  def best_fixed(table):           # -> the tag with the most reached among those safe on every cell, or None
  def pair_trade(table):           # -> per geometry and step: the smallest safe margin of the straight cell and of
                                   #    the turn cell (m, or None), and their difference
  def main(argv=None):             # DIR --out FILE.md
  ```

- [ ] **Step 1: Write the failing tests** on fabricated summaries: `test_safe_needs_zero_contacts_and_clearance`
  (clearance 0.019 -> not safe; 0.020 -> safe; one contact -> not safe), `test_best_fixed_requires_safe_everywhere`,
  `test_pair_trade_reports_the_difference` (straight safe at 0.02, turn only from 0.03 -> difference 0.01),
  `test_medium_config_reaches_the_command_line`.
- [ ] **Step 2: Run** — FAIL. **Step 3: Implement.** **Step 4: Run** — PASS.
- [ ] **Step 5: Verify the configs** differ from their sources only in the margin line (`diff` shows one line each)
  and that B5's and the medium planner's loaders accept them (`plant_campaign --b5-config ... --medium-config ...`
  on one 5 s run per margin).
- [ ] **Step 6: Commit** `E3 Task 4: fixed-margin configs, the sweep script and margin_sweep.py`.
- [ ] **Step 7: Run the sweep** with the user's go-ahead (`caffeinate -i bash .../e3_ensembles.sh sweep`, about
  1,300 runs, close to 2 h). Then `margin_sweep.py experiment_runs/e3/sweep` and `margin_analysis.py report` on the
  traced legs.

### Task 5: CP1 report (go/no-go)

**Files:**
- Modify: `docs/experiments/execution-fixes/e3.md` ("CP1"), `tools/plant/margin_analysis.py` (the first fit),
  `tools/plant/test_margin_analysis.py`

**Interfaces:**
- Produces: `margin_analysis.first_fit(records, features=("v2", "w2", "qd2"), coverage=0.95, seed=0)` -> dict with
  `weights`, `constant`, `held_out_coverage`, `mean_margin`, `constant_margin` (the constant bound with the same
  held-out coverage), `saving` (1 - mean_margin / constant_margin). Split by run, 70 / 30. Standard library only: a
  non-negative least-squares fit of the per-bin 95th percentile on a 5 x 5 x 5 feature grid, then a conformal offset
  on the held-out runs (the full fit is Task 6).

- [ ] **Step 1: Write the failing test** `test_first_fit_beats_the_constant_on_feature_driven_errors`: synthetic
  records with `e_track = 0.002 + 0.02 * w2 + noise` give `saving > 0.3` and `0.92 <= held_out_coverage <= 0.98`;
  `test_first_fit_split_keeps_runs_whole`.
- [ ] **Step 2: Implement; run** — PASS.
- [ ] **Step 3: Write CP1 in e3.md**, each number traced to its command and directory:

  | Criterion | Number to report | Go if |
  |---|---|---|
  | Size | 95th percentile `e_track` in the worst regime, per type and lookahead | >= 5 mm at the lookahead proposed |
  | Dependence | the same in `straight` | worst >= 2 x straight and >= straight + 5 mm |
  | Predictability | `first_fit`'s `saving` and `held_out_coverage` | saving >= 0.30 at coverage within 3 points of 95 % |
  | Trade | `pair_trade`'s difference per geometry and step | >= 5 mm on at least one geometry |

  Also: the lookahead proposed (the shortest at which Size holds, with the reason), the rows that qualify (decision
  3), `history_dependence` (decision 7), the share of `e_lin` in `e_total` (E4), the best fixed margin, the floor
  candidate (the smallest margin safe on every straight cell).
- [ ] **Step 4: Commit** `E3 Task 5: CP1 report`, write the review prompt, **stop for the user's go/no-go**. Record
  the ruling in e3.md and the roadmap's decision log; amend this plan's Stage 2 for the ruled lookahead, rows and
  features.

---

## Stage 2: build (after a go)

### Task 6: Margin model fit and calibration

**Files:**
- Create: `tools/plant/fit_margin.py`, `tools/plant/test_fit_margin.py`
- Create: `ros2_ws/src/mobile_manipulator_mpc/config/tracking_margin_mujoco.yaml` (the fit's output, committed)
- Modify: `docs/experiments/execution-fixes/e3_ensembles.sh` (`fitdata`: the existing grip corpus and the tightness
  cells at the floor candidate's margin, mu 0.4 / 0.6 / 0.8, seeds 1-20, `--row-trace`)

**Interfaces:**
- Consumes: `margin_analysis.trace_records`.
- Produces (file format, the contract with Task 7):
  ```yaml
  version: 1
  lookahead_s: 0.5          # CP1
  coverage: 0.95
  floor_m: 0.03             # CP1's floor candidate
  upper_m: 0.05             # set at CP2
  features: [v2, w2, qd2]   # CP1
  rows:                     # one entry per row type that takes the model; others keep the fixed margin
    panel: {constant: 0.004, weights: [0.0, 0.031, 0.012]}
    arm:   {constant: 0.003, weights: [0.0, 0.020, 0.015]}
  sha256: "<of the file's other lines>"
  ```
  (the numbers above show the format, not results)
- Produces (Python): `fit(records, features, coverage) -> model dict` (quantile regression, pinball loss, weights and
  constant >= 0, `scipy.optimize.linprog`); `calibrate(model, held_out_records) -> model` (adds the split-conformal
  offset to each type's constant); `coverage_report(model, records, by=("regime", "cell", "mu"))`; `split_by_run(records,
  fraction=0.3, seed=0)`; `write_model(model, path)` (computes `sha256`); CLI `fit_margin.py RUN_DIR... --floor F
  --lookahead L --features ... --out FILE --report FILE.md`.

- [ ] **Step 1: Write the failing tests**: `test_fit_recovers_known_weights` (synthetic 95th percentile `0.003 + 0.02
  w2`: weight within 15 %, the others below 0.002); `test_weights_are_never_negative`; `test_split_keeps_each_run_on_one_side`;
  `test_calibration_reaches_coverage_on_held_out` (>= 0.95 on the calibration set by construction; within 3 points on
  a third set); `test_short_runs_stay_on_one_side` (Review Focus 5); `test_written_file_hash_matches`.
- [ ] **Step 2: Run** — FAIL. **Step 3: Implement.** **Step 4: Run** — PASS.
- [ ] **Step 5: Run `fitdata`** (user's go-ahead), fit, write the model file and the coverage report into e3.md.
- [ ] **Step 6: Commit** `E3 Task 6: fit_margin.py and the plant's margin model`.

### Task 7: MarginModel in B5 and its loader

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/margin_model.hpp`,
  `ros2_ws/src/mobile_manipulator_mpc/src/margin_model.cpp`,
  `ros2_ws/src/mobile_manipulator_mpc/test/test_margin_model.cpp`
- Modify: `ros2_ws/src/mobile_manipulator_mpc/CMakeLists.txt`

**Interfaces:**
- Produces:
  ```cpp
  struct MarginTerm { double constant = 0.0; std::vector<double> weights; };
  struct MarginModel {
    double lookahead_s = 0.0, floor_m = 0.0, upper_m = 0.0;
    std::vector<std::string> features;            // "v2" | "w2" | "qd2"
    std::map<std::string, MarginTerm> rows;       // by row type
    std::string sha256;
    bool empty() const;                           // no rows: every row keeps its fixed margin
  };
  struct MarginValue { double margin; Eigen::Matrix<double, 1, kNu> slope; };  // d margin / d u at u
  // The margin of a row of `type` for the stage command u: clamp(floor + constant + sum w_i f_i(u), floor, upper);
  // slope zero where a clamp is active. qd2 is a smooth maximum of the squared joint speeds (log-sum-exp, sharpness
  // 50). A type without an entry: nullopt (the caller keeps the row's fixed margin).
  std::optional<MarginValue> margin_of(const MarginModel & m, const std::string & type, const Control & u);
  // Throws std::runtime_error naming the file and the key: unknown key or feature, version != 1, a negative weight or
  // constant, floor_m <= 0, upper_m < floor_m, upper_m >= activation_m, a sha256 that does not match.
  MarginModel load_margin_model(const std::string & yaml_path, double activation_m);
  ```

- [ ] **Step 1: Write the failing tests**: `MarginModel.ZeroCommandGivesFloorPlusConstant` (finite, zero slope);
  `MarginModel.SignOfTheCommandDoesNotMatter`; `MarginModel.SlopeMatchesFiniteDifferences` (1e-6 step, tolerance
  1e-5, at three commands off the clamps); `MarginModel.ClampsAndZeroesTheSlope`; `MarginModel.TypeWithoutEntryIsNullopt`;
  `MarginModelLoader.Refuses*` (one per refusal above); `MarginModelLoader.ReadsThePlantFile`.
- [ ] **Step 2: Run** `ctest_pkg mobile_manipulator_mpc MarginModel` — FAIL. **Step 3: Implement.** **Step 4:** PASS.
- [ ] **Step 5: Commit** `E3 Task 7: MarginModel, margin_of and the loader`.

### Task 8: Approach A (the margin from the previous plan)

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_mpc/src/formulation.cpp` and `formulation.hpp` (config keys
  `margin_model: ""`, `margin_mode: previous | slope`), `controller.cpp` / `controller.hpp`, `config/wholebody_mpc.yaml`
- Modify: `ros2_ws/src/mobile_manipulator_campaign/src/session.cpp`, `plant_campaign.cpp` (`--margin-model FILE|none`,
  `--margin-mode`), `tools/plant/plant_campaign.py`, `docs/experiments/mujoco-plant/stream-protocol.md`
- Create: `docs/experiments/execution-fixes/configs/b5_margin_a.yaml`, `medium_margin_floor.yaml` (`margins.arm` =
  the model's `floor_m`)
- Test: `test_controller.cpp`, `test_formulation.cpp`, `test_golden.cpp`, `test_session.cpp`

**Interfaces:**
- Consumes: Task 7's `margin_of`, `load_margin_model`; Task 1's row types.
- Produces: with a model and `margin_mode: previous`, each obstacle row's `DistanceRow::margin` is
  `margin_of(model, type, u_prev[stage])->margin`, `u_prev` the previous solution shifted by the elapsed cycles (the
  nominal's commands); stage commands beyond the model's lookahead use the lookahead's value. The slack gate, the
  inside-margin weights and the trace read `row.margin` and so follow. The Session Hello records the model's `sha256`
  and the mode. Without a model nothing changes.

- [ ] **Step 1: Write the failing tests**: `MarginA.RowsTakeTheModelsMarginAtThePreviousCommands` (a turning nominal
  gives panel rows a margin above the floor by the model's value; a base row keeps 0.12 when `base` has no entry);
  `MarginA.FirstCycleWithoutAPreviousPlanUsesZeroCommand`; `Golden.NoMarginModelIsToday` (the golden data unchanged
  with `margin_model: ""`); `Session.HelloRecordsTheMarginModel`.
- [ ] **Step 2: Run** — FAIL. **Step 3: Implement.** **Step 4:** the new tests and the whole MPC and campaign suites
  PASS.
- [ ] **Step 5: M6 on the plant**: three cells (`v2_store_room_nav`, `v2_low_beam_123_nav`, `v2_chicane_nav`), seed
  1, F2, against the E2 baseline's runs with `model_rollout.same_run` — identical.
- [ ] **Step 6: Commit** `E3 Task 8: approach A: each row's margin from the model at the previous plan's commands`.
- [ ] **Step 7: Ensemble** (user's go-ahead): `e3_ensembles.sh variant b5_margin_a.yaml medium_margin_floor.yaml
  mA` on the tightness corpus, seeds 1-20, with `--row-trace`. **CP2 report** in e3.md: A against the best fixed
  margin (`sweep_table`, Fisher per cell), holds and time through the gap, coverage on these runs
  (`coverage_report`), a proposed `upper_m`. **Stop for the user's ruling.**

### Task 9: Approach B (the slope in the rows)

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_mpc/src/formulation.cpp`, `controller.cpp`, `collision.hpp`
  (`DistanceRow::margin_slope`, `Eigen::Matrix<double, 1, kNu>`, zero by default)
- Create: `docs/experiments/execution-fixes/configs/b5_margin_b.yaml`
- Test: `test_formulation.cpp`, `test_controller.cpp`

**Interfaces:**
- Consumes: `MarginValue::slope`.
- Produces: with `margin_mode: slope` the QP row of an obstacle row at stage k is
  `distance + gradient dx_k - margin_slope du_k + slack >= margin`, `du_k = u_k - u_prev[k]`; mid-stage rows add the
  term to their existing `control_gradient` with the opposite sign convention checked by the test. The slack gate
  evaluates each gated row against `margin_of(model, type, u_plan[stage])->margin`, the solved plan's own command.

- [ ] **Step 1: Write the failing tests**: `MarginB.RowCarriesTheSlopeOnTheStagesCommand` (the assembled constraint
  matrix has `-slope` in the stage's control columns; against a finite difference of the row's residual in u);
  `MarginB.SolverSlowsToPassWhereAHolds` (a unit loop: a gap the previous-plan margin rejects at 0.3 m/s; B's plan
  has a lower speed at the gap's stages and zero gate slack; A's has slack above the tolerance);
  `MarginB.GateUsesTheExactMarginAtThePlansCommands` (Review Focus 4: a plan whose commands jump so the linearized
  row passes and the exact margin is violated by more than the tolerance is `gate_unsafe`); `MarginB.PreviousModeIsUnchanged`.
- [ ] **Step 2: Run** — FAIL. **Step 3: Implement.** **Step 4:** PASS, whole suites PASS.
- [ ] **Step 5: Commit** `E3 Task 9: approach B: the margin's slope with respect to the stage's commands in the rows; the gate checks the exact margin`.

### Task 10: Ensembles and the closed-loop coverage check (CP3)

**Files:**
- Modify: `docs/experiments/execution-fixes/e3_ensembles.sh` (`accept TAG`: the tightness corpus, campaign A,
  campaign B and the grip corpus at mu 0.4 / 0.6 / 0.8, twins on campaign A; seeds 1-20; `--row-trace` on the
  tightness and grip sets), `docs/experiments/execution-fixes/e3.md`
- Create: `docs/experiments/execution-fixes/e3_judge.sh` (the M-table's commands in one place, like `e2_judge.sh`)

**Interfaces:**
- Consumes: `margin_sweep.sweep_table`, `best_fixed`; `compare_variants.regressions`, `locks`; `divergence.py`'s flip
  rate; `fit_margin.coverage_report`; the run summaries' B5 cycle p99.

- [ ] **Step 1: Run** `accept mB` with `b5_margin_b.yaml` (user's go-ahead; about the size of E2's ensemble, in two
  sittings if needed).
- [ ] **Step 2: Coverage on these runs** (spec 4.5). Below target: one refit on them (`fit_margin.py`, a new model
  file and hash, a rerun of the tightness and grip sets). A second miss: report, do not iterate.
- [ ] **Step 3: Judge** M1 (B against the best fixed margin per pair, Fisher), M2, M3, M4, M5 (B against `mA`), M7,
  M9 with `e3_judge.sh`; write the table with each number's command and directory.
- [ ] **Step 4: Commit** `E3 Task 10: acceptance ensembles and the coverage check`; **CP3: stop for the user's
  ruling.**

### Task 11: Fresh seeds, acceptance, records (CP4)

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_mpc/config/wholebody_mpc.yaml` and the plant runs' default (only if the
  user adopts: the margin model as the plant runs' default, as ADR 0018 did for the tracking model),
  `ros2_ws/src/mobile_manipulator_medium/config/medium.yaml` (`margins.arm` on adoption),
  `docs/experiments/execution-fixes/e3.md`, `docs/roadmap.md`
- Create: `docs/adr/0019-learned-tracking-margin.md`

- [ ] **Step 1: Run** seeds 21-40 on the tightness corpus for B, A and the best fixed margin (user's go-ahead); judge
  M8.
- [ ] **Step 2: Write** the acceptance table M1-M9 in e3.md, the problems table, ADR 0019 (options: fixed 0.05, the
  best fixed, A, B; the decision; consequences for E4, E5 and the campaign revision).
- [ ] **Step 3: Roadmap:** the E3 outcome under step 3; the line under step 7 that the learned-against-fixed
  comparison is rerun on the irregular passages; the comparability log for every changed default.
- [ ] **Step 4: Commit** `E3 accepted (CP4)` after the user's ruling; then the whole-branch review and the finishing
  choice (merge only when asked).

### Task 12: Closing on a no-go (only if CP1 rules no-go)

**Files:**
- Modify: `docs/experiments/execution-fixes/e3.md`, `docs/roadmap.md`; on the user's separate ruling,
  `wholebody_mpc.yaml` `collision.margin_m` and `medium.yaml` `margins.arm` to the best fixed margin.

- [ ] **Step 1: Write** the result: the measured errors, the sweep, why a learned margin is not built.
- [ ] **Step 2: If the user adopts the best fixed margin:** the existing campaign and grip sets with it (M4's rule),
  the comparability log, the golden data regenerated under the new default.
- [ ] **Step 3: Roadmap** entry and the line under step 7; commit; whole-branch review; the finishing choice.
