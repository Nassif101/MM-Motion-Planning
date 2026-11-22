# The medium-planner diagnosis: implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Find the layer and the decision behind three observed failures of F2 and B5 (E3's gap cells, the R4 door
stall, the low-beam pivot), with log evidence and same-state replays that change one setting.

**Architecture:** Stage A reads run logs that carry a richer seed row; Stage B forks a byte-identical rerun at a time T
with one setting changed (in the session, before the medium cycle); Stage C writes the result. Analysis in Python
(`tools/plant/medium_diag.py`, `tools/plant/fork_replay.py`), runtime in C++.

**Tech Stack:** C++17 + gtest (colcon, ctest) in the dev container; Python 3 + pytest; the MuJoCo plant's
`plant_campaign`; run logs read with `tools/plant/plant_stream.RunLog`.

**Spec:** [docs/superpowers/specs/2026-11-22-medium-diagnosis-design.md](../specs/2026-11-22-medium-diagnosis-design.md)

## Global Constraints

- Default behavior unchanged: with `--seed-detail` and the fork options off, every frame after the Hello is
  byte-identical to what the branch's starting build (main 48c1aa4) writes for the same arguments (checked with
  `medium_diag.log_fingerprint` against Task 1's reference logs). The Hello differs only in `provenance` (the commit)
  and the new `config` keys `seed_detail` and `fork`.
- No fixes, no tuning, no new seeds or cost terms; a replay changes **one setting** against its control.
- Every cause is checked across a cell's failing runs, never one run ([tune-with-ensembles]).
- Runtime and session code in C++; tools in Python under `tools/plant/`.
- Builds: `MAKEFLAGS=-j3 colcon build --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`.
- Host: no Unity Play during timed runs; long runs under `caffeinate -i`, checked against `pmset -g log`; a new output
  directory for every changed cell or setting.
- One commit per task or fix on branch `medium-diagnosis`; never on main. After each task: commit, stop, and hand the
  user a self-contained review prompt (saved under `docs/reviews/medium-diagnosis/`, git-ignored, never committed or
  cited in tracked files).
- Container helpers (recreate in the session scratchpad, not in the repo): `cbuild PKG...` (the build above with
  `--packages-select`), `ctest_pkg PKG REGEX` (`colcon test --packages-select PKG --ctest-args -R REGEX`, then
  `colcon test-result --verbose`), `cpytest FILE` (sourced `python3 -m pytest FILE -q` in
  `/workspaces/mm-motion-planning`). Container: `mm-motion-planning-ma-robot-sim-1`.

**Order against the spec.** The richer seed row (spec B1) moves to Task 1, before the baseline runs, so Stage A's logs
already say which constraint failed and the baseline is not run twice. It changes no plan or command. *(My call.)*

## Review Focus

1. **A fork time that falls on no medium cycle** (between cycles, before the first, after the run ends): the fork
   applies at the first medium cycle at or after T; after the end it is reported as `not_applied`, never silently
   dropped. Test in Task 4 (`ForkAfterTheEndIsReportedNotApplied`).
2. **A fork in a lattice run during a planning hold** (no stepper tracking at T): the settings apply at the next
   medium cycle of any stepper (the idle stepper included). Test in Task 4 (`ForkAppliesInAGeneratedRun`).
3. **A cycle cut by the 10 s wall-clock deadline at a large budget:** flagged in the summary
   (`fork.deadline_cut_cycles`) and excluded from the counts; not a silent non-reproducible result. Test in Task 4
   (`DeadlineCutIsFlagged`, with a 1 ms cap).
4. **A log that ends inside a chunk or has no Plans frames** (a B5-alone run, a crashed run): `medium_diag` reports the
   run as `unreadable` or `no_medium` and keeps going. Test in Task 2.
5. **A seed row without detail mixed with rows with detail** (old and new logs in one report): rows are parsed by
   length; detail fields are `None` when absent. Test in Task 2.

---

### Task 1: Reference logs and the richer seed row (`--seed-detail`)

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_medium/include/mobile_manipulator_medium/problem.hpp` (`Violations`),
  `ros2_ws/src/mobile_manipulator_medium/src/problem.cpp` (near line 518),
  `ros2_ws/src/mobile_manipulator_medium/include/mobile_manipulator_medium/planner.hpp` (`SeedStats`),
  `ros2_ws/src/mobile_manipulator_medium/src/planner.cpp`
- Modify: `ros2_ws/src/mobile_manipulator_campaign/include/mobile_manipulator_campaign/session.hpp`
  (`SessionConfig::seed_detail`), `ros2_ws/src/mobile_manipulator_campaign/src/session.cpp` (`seed_rows`, the Hello's
  `config`), `ros2_ws/src/mobile_manipulator_campaign/src/plant_campaign.cpp`
- Modify: `tools/plant/plant_campaign.py` (`runs_of`, the `run` parser)
- Test: `ros2_ws/src/mobile_manipulator_medium/test/test_problem.cpp`,
  `ros2_ws/src/mobile_manipulator_campaign/test/test_session.cpp`, `tools/plant/test_plant_campaign.py`
- Create: `docs/experiments/medium-diagnosis/reference_logs.json`, `tools/plant/medium_diag.py` (only
  `log_fingerprint` here; Task 2 adds the rest), `tools/plant/test_medium_diag.py`

**Interfaces:**
- Produces: `Violations` gains `std::string deepest_link; std::string deepest_obstacle; double deepest_distance = NaN;
  double deepest_t = NaN;` (the collision row with the largest `margin - distance`; its link's name, the obstacle's
  name, the signed distance, `sample * collision_dt`). `SeedStats` gains `Violations violations; bool interrupted`.
- Produces: with `SessionConfig::seed_detail` (`--seed-detail`), each Plans seed row is
  `[name, merit, feasible, progress_m, merged, slack, collision_m, torque, speed_mps, hull, iterations, converged,
  resumed, homotopy, deepest_link, deepest_obstacle, deepest_distance, deepest_t, interrupted]` (bools as 0/1, NaN as
  null; `interrupted` added by the Task 1 review, I1);
  without it, the five fields of today. The Hello's `config` carries `seed_detail`.
- Produces: `plant_campaign.py run --prediction --seed-detail` passes both flags to every run.
- Produces: `medium_diag.log_fingerprint(path) -> str`: sha256 over the encoded bytes of every frame after the Hello,
  in order (`plant_stream.RunLog`).

- [ ] **Step 1: Write and pass** `test_fingerprint_ignores_the_hello_only` (two synthetic logs with different Hellos
  and equal frames: equal fingerprints; one frame changed: different) with `log_fingerprint`. Run
  `cpytest tools/plant/test_medium_diag.py`: FAIL first (no module), then PASS.
- [ ] **Step 1b: Record the reference logs with the unchanged build.** On a clean build of the branch start
  (48c1aa4), run `plant_campaign` for `e3_gap_turn_2` (realistic, seed 3, the scenario route) and `v2_room_r4_nav`
  (realistic, `--routes lattice`, seed 1, `--sim-time-limit 30`), each with `--prediction`. Write each log's
  `log_fingerprint` and the exact arguments into `reference_logs.json`. Keep the logs in
  `experiment_runs/medium-diag/reference/`.
- [ ] **Step 2: Write the failing tests.**
  - `TEST(Problem, ViolationsNameTheDeepestRow)`: a trajectory driving the panel into a named box; expect
    `deepest_obstacle == "<box name>"`, `deepest_link == "panel"`,
    `collision_m == Approx(arm_margin_m - deepest_distance, 1e-12)`, `deepest_t` a multiple of `collision_dt`.
  - `TEST(Problem, ViolationsWithoutCollisionLeaveTheDeepestEmpty)`: free space; `deepest_obstacle == ""`,
    `std::isnan(deepest_distance)`.
  - `TEST(Session, SeedDetailAppendsToTheSeedRows)`: the same seed with and without `seed_detail`; every Plans frame's
    rows have 19 against 5 fields, and the first five are equal.
  - `TEST(Session, SeedDetailLeavesOtherFramesIdentical)`: all non-Plans frames are byte-equal between the two runs
    (as `RowTraceLeavesOtherFramesIdentical`).
  - pytest `test_prediction_and_seed_detail_are_passed`: `runs_of(..., prediction=True, seed_detail=True)` puts
    `--prediction` and `--seed-detail` in every run's args; neither by default.
- [ ] **Step 3: Run them to see them fail:** `cbuild mobile_manipulator_medium mobile_manipulator_campaign` (build
  error: no `deepest_obstacle`), `cpytest tools/plant/test_plant_campaign.py` (FAIL: unexpected keyword).
- [ ] **Step 4: Implement** the fields and the flag as in Interfaces. The deepest row is taken where `collision_m` is
  maximised (`problem.cpp`, the `row.margin - row.distance` line); ties keep the first.
- [ ] **Step 5: Run** `ctest_pkg mobile_manipulator_medium Problem`, `ctest_pkg mobile_manipulator_campaign Session`,
  `cpytest tools/plant/test_plant_campaign.py`. Expected: PASS, with the existing `SameSeedSameLogBytes`.
- [ ] **Step 6: Check the reference logs:** rerun Step 1b's two commands with the new build; both fingerprints equal
  `reference_logs.json`. Check the viewer by reading: `MediumPlanStore.FromFrame` takes rows with at least five
  fields and reads the first five (`MediumPlanStore.cs:84-87`), so longer rows decode unchanged.
- [ ] **Step 7: Commit** `Medium diagnosis Task 1: reference logs and the richer seed row (--seed-detail)`.

### Task 2: Baseline runs and `medium_diag.py` (spec A1, A2)

**Files:**
- Create: `docs/experiments/medium-diagnosis/corpus_diag.json` (cells `v2_room_r4_nav`, `v2_low_beam_123_nav`, each
  `{"cell": name, "scenario": name}`), `docs/experiments/medium-diagnosis/diag_ensembles.sh`
- Modify: `tools/plant/medium_diag.py`, `tools/plant/test_medium_diag.py` (created in Task 1)

**Interfaces:**
- Consumes: Task 1's seed rows; `plant_stream.RunLog(path).frames()` yielding `(kind, header, blocks)`.
- Produces (`medium_diag.py`):
  - `cycles(log) -> list[dict]`: per Plans frame `{t, status, seed, full_multistart, waiting, seeds: [dict]}`; each
    seed dict has the five base keys and the detail keys (Task 1's names) or `None` for each when the row is short.
  - `states(log) -> list[dict]`: per State frame `{t, x, y, yaw, q, fallback, gate, execution}`.
  - `stall_onset(states, base_m=0.05, yaw_rad=0.1, joint_rad=0.05) -> float | None`: the last time the session's
    progress rule (`session.cpp` `kStallBaseM`, `kStallYawRad`, `kStallJointRad`) saw progress before a run's end
    without reaching; `None` for a reached run.
  - `fork_point(cycles, states, reached) -> dict | None`: `{"kind": "infeasible", "t": t}` for the first infeasible
    cycle after which every cycle up to the stall onset is infeasible; else `{"kind": "stall_onset", "t": onset}`;
    `None` for a reached run.
  - `classify(cycles, states, fork) -> str`: initial classes `infeasible_stall`, `slack_stall` (ok plans with
    collision slack while stalled), `b5_refused` (F2 ok and feasible, B5 holding or on a fallback), `timeout_moving`,
    `unreadable`, `no_medium`, `reached`; refined after reading the logs (the builder's call, recorded in the
    results).
  - CLI `medium_diag.py report --runs DIR --out DIR2`: `DIR2/report.md` (per cell: runs, reached, each class's count,
    the median infeasible share before the fork) and `DIR2/fork_points.json`
    (`[{"run", "cell", "seed", "kind", "t", "class"}]`).
- Produces (`diag_ensembles.sh`): `baseline` stage: the eight `e3_gap_*` cells of
  `docs/experiments/execution-fixes/corpus_e3_tight.json` (`--cells`), `--mode realistic --kinds f2 --seeds 20
  --prediction --seed-detail --tag m050`, and `corpus_diag.json` with `--routes lattice` and the same flags, into
  `experiment_runs/medium-diag/baseline` (`OUT=` overrides).

- [ ] **Step 1: Write the failing tests** (synthetic logs written with `plant_stream.encode` in a tmp dir):
  `test_persistent_infeasibility_is_the_fork` (infeasible from t 12.0 to the stop: kind `infeasible`, t 12.0);
  `test_a_late_recovery_is_not_persistent` (infeasible 12-14 s, ok at 14.4 s, infeasible again from 16.0 s: t 16.0);
  `test_no_infeasibility_forks_at_the_stall_onset` (ok plans, base still from 20.0 s: kind `stall_onset`, t 20.0);
  `test_reached_run_has_no_fork`; `test_short_and_detailed_rows_parse` (Review Focus 5: a 5-field and a 19-field row
  in one frame); `test_unreadable_and_b5_only_logs_are_classified` (Review Focus 4: a truncated file and a log without
  Plans give `unreadable` and `no_medium`, the report goes on); `test_report_counts_per_cell`.
- [ ] **Step 2: Run** `cpytest tools/plant/test_medium_diag.py`. Expected: FAIL (no module).
- [ ] **Step 3: Implement** `medium_diag.py` as in Interfaces, and the corpus and the script.
- [ ] **Step 4: Run** `cpytest tools/plant/test_medium_diag.py`. Expected: PASS.
- [ ] **Step 5: Commit** `Medium diagnosis Task 2: medium_diag.py, the diagnosis corpus and the baseline script`.
- [ ] **Step 6: Run the baseline** (200 runs; estimate the time from the first ten before leaving it):
  `caffeinate -i bash docs/experiments/medium-diagnosis/diag_ensembles.sh baseline`; then `pmset -g log` for sleeps
  in the window (a run that spans one moves to `baseline_slept` and is run again).
- [ ] **Step 7: Report** `python3 tools/plant/medium_diag.py report --runs experiment_runs/medium-diag/baseline
  --out experiment_runs/medium-diag/baseline_report`. Compare the gap cells' reached counts with E3's m050 sweep
  (`experiment_runs/e3/sweep`) in a table in `docs/experiments/medium-diagnosis/diagnosis.md` (new, section
  "Baseline on main"). Commit `Medium diagnosis Task 2: baseline on main`.

### Task 3: The door stall and the pivot from the logs (spec A3, A4); checkpoint A

**Files:**
- Modify: `tools/plant/medium_diag.py`, `tools/plant/test_medium_diag.py`, `docs/experiments/medium-diagnosis/diagnosis.md`
- Create (only if needed): `docs/experiments/medium-diagnosis/goals_r4_from_hallway.yaml` (a goal script)

**Interfaces:**
- Produces: `seed_table(cycles, t0, t1) -> list[dict]` (per cycle: t, winner, and per seed tried: feasible, merit,
  detail; untried seeds absent) and `layer_at(cycles, states, t) -> dict` (`{"medium": status, "plan_feasible",
  "b5_execution", "fallback", "gate"}` at the cycle and State nearest before t). CLI `medium_diag.py seeds --run DIR
  --from T0 --to T1` prints the seed table.

- [ ] **Step 1: Write the failing tests:** `test_seed_table_lists_only_seeds_tried` (a cycle trying `previous` and
  `fold`: `fold_back` absent); `test_layer_at_reports_b5_holding_on_a_feasible_plan`.
- [ ] **Step 2: Run** `cpytest tools/plant/test_medium_diag.py`. Expected: FAIL (no `seed_table`).
- [ ] **Step 3: Implement**, then **Run** the tests. Expected: PASS. Commit
  `Medium diagnosis Task 3: seed table and layer view`.
- [ ] **Step 4: The door.** In the baseline's `v2_room_r4_nav` runs, find stalls within 1.5 m of R4's door (x 7.825,
  the hallway's north wall). For each: `layer_at` over the stall, and whether an ok, feasible F2 plan through the door
  existed while B5 held. If none stalls there, write the goal script (start in the hallway, goal in R4) and run 20
  seeds with `--goals`; if that also passes, stop and ask the user to recreate the stall once in the viewer
  (`plant_demo_server --mode realistic --routes lattice --log FILE`).
- [ ] **Step 5: The pivot.** In the baseline's `v2_low_beam_123_nav` runs: the cycles approaching the beam; how often
  `fold_back` was tried, feasible (and if not, which kind of violation and which row), and its merit against the
  winner's, including the side hysteresis (`side_hysteresis: 0.5`). Say whether the losing seeds' paths (the viewer's
  deferred item) are needed to answer it.
- [ ] **Step 6: Write** `diagnosis.md` section "Stage A" (classes per cell, the door, the pivot, what is answered and
  what needs a fork) and the proposed fork list. Commit `Medium diagnosis Task 3: Stage A results`.
- [ ] **Step 7: Checkpoint A. Stop.** Hand the user the results and the review prompt. The user decides which fork
  questions Stage B runs (and whether the deferred viewer items are wanted). Record the ruling in `diagnosis.md`.

### Task 4: The fork (spec B2)

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/collision.hpp`,
  `ros2_ws/src/mobile_manipulator_mpc/src/collision.cpp` (`CollisionModel::set_margin`),
  `.../mobile_manipulator_mpc/formulation.hpp` + `src/formulation.cpp` (`TightnessFilter::set_margin`),
  `.../mobile_manipulator_mpc/controller.hpp` + `src/controller.cpp` (`Controller::set_collision_margin`)
- Modify: `.../mobile_manipulator_medium/problem.hpp` + `src/problem.cpp` (`MediumProblem::set_arm_margin`),
  `.../planner.hpp` + `src/planner.cpp` (`set_arm_margin`, `set_pose_seeds`, `PlanOutput::deadline_cut`),
  `.../closed_loop.hpp` + `src/closed_loop.cpp` (`ClosedLoopOptions::before_plan`)
- Modify: `.../mobile_manipulator_campaign/session.hpp` + `src/session.cpp` (`SessionConfig::fork`), `src/plant_campaign.cpp`
- Test: `mobile_manipulator_mpc/test/test_collision.cpp`, `test_controller.cpp`;
  `mobile_manipulator_medium/test/test_planner.cpp`; `mobile_manipulator_campaign/test/test_session.cpp`

**Interfaces:**
- Produces: `void CollisionModel::set_margin(double margin_m)` (the arm and panel margin; `base_margin_m` kept; throws
  `std::invalid_argument` unless `0 < margin_m <= base_margin_m`). `void TightnessFilter::set_margin(double)` (keeps
  its state). `void Controller::set_collision_margin(double)` (both). `void MediumProblem::set_arm_margin(double)`
  (`config_.arm_margin_m` and its collision model's margin). `void MediumPlanner::set_arm_margin(double)`;
  `void MediumPlanner::set_pose_seeds(const std::vector<std::string> & names)` (replaces the rotation's pose seeds;
  each must be a config seed other than `previous`, else `std::invalid_argument`; resets the rotation to 0).
  `PlanOutput::deadline_cut` (true when the cycle stopped at the wall-clock deadline, not the budget).
- Produces: `ClosedLoopOptions::before_plan` (`std::function<void(double t)>`, called right before each
  `medium_.plan`).
- Produces: `SessionConfig::fork` = `struct ForkConfig { double at = NaN; std::map<std::string, std::string> set;
  int cycles = 0; }`. Keys: `iteration_budget` (int > 0), `medium_arm_margin` (m), `b5_arm_margin` (m),
  `full_every_cycle` (`1`: `request_full_multistart()` before every cycle from the fork on), `seeds` (comma-separated
  pose seeds). Unknown keys or values throw at construction. The settings apply at the first medium cycle with
  `t >= at`; `cycles > 0` ends the run after that many forked plans (end status `forked`). The Hello's `config`
  carries `fork`; the summary carries `fork: {at, applied_t (null: not_applied), set, cycles, cycle_wall_s: [..],
  deadline_cut_cycles}` (wall times in the summary only, so logs stay deterministic).
- Produces: `plant_campaign --fork-at T --fork-set key=value (repeatable) --fork-cycles N`.

- [ ] **Step 1: Write the failing tests.**
  - `TEST(Collision, SetMarginKeepsTheBaseMargin)` (rows' `margin` is the new value for arm rows, `base_margin_m` for
    base rows) and `TEST(Collision, SetMarginRejectsAboveTheBaseMargin)`.
  - `TEST(Controller, CollisionMarginSetterMatchesAConstructedController)`: built at 0.05 then set to 0.03, against
    built at 0.03; the first cycle's command equal within 1e-12.
  - `TEST(Planner, ArmMarginSetterMatchesAConstructedPlanner)` (same: plan samples equal within 1e-12);
    `TEST(Planner, PoseSeedsSetterLimitsTheRotation)` (`{"fold_back"}`: every non-full cycle tries `previous` and
    `fold_back` only); `TEST(Planner, UnknownPoseSeedIsRejected)`.
  - `TEST(Session, ForkWithNoSettingReproducesTheRunWhole)` (`at` set, `set` empty: log bytes equal the unforked run).
  - `TEST(Session, ForkedRunMatchesUpToTheForkTime)` (`iteration_budget=60` at T: all frames with `t < T` equal the
    unforked run's; some later Plans frame differs).
  - `TEST(Session, OneCycleForkStopsAfterOnePlan)` (exactly one Plans frame with `t >= T`; end status `forked`).
  - `TEST(Session, ForkIsRecordedInHelloAndSummary)`; `TEST(Session, UnknownForkKeyIsRejected)`.
  - Review Focus: `TEST(Session, ForkAfterTheEndIsReportedNotApplied)`, `TEST(Session, ForkAppliesInAGeneratedRun)`
    (a lattice session, T inside a planning hold: applied at the next medium cycle), `TEST(Session,
    DeadlineCutIsFlagged)` (solve cap 1 ms with budget 300: `deadline_cut_cycles > 0`).
- [ ] **Step 2: Run them to see them fail:** `cbuild mobile_manipulator_mpc mobile_manipulator_medium
  mobile_manipulator_campaign`. Expected: build errors (no `set_margin`, no `fork`).
- [ ] **Step 3: Implement** as in Interfaces. `iteration_budget` uses `MediumPlanner::set_iteration_budget`; the
  fixed latency stays the profile's `solve_cap_s` (spec B3).
- [ ] **Step 4: Run** `ctest_pkg mobile_manipulator_mpc "Collision|Controller"`, `ctest_pkg mobile_manipulator_medium
  Planner`, `ctest_pkg mobile_manipulator_campaign Session`. Expected: PASS, the existing tests included.
- [ ] **Step 5: Check the reference logs** (Task 1 Step 6's commands, no fork options): fingerprints unchanged.
- [ ] **Step 6: Commit** `Medium diagnosis Task 4: the fork (--fork-at, --fork-set, --fork-cycles)`.

### Task 5: The replay matrix (spec B3); checkpoint B

**Files:**
- Create: `tools/plant/fork_replay.py`, `tools/plant/test_fork_replay.py`
- Modify: `docs/experiments/medium-diagnosis/diag_ensembles.sh` (stage `forks`), `docs/experiments/medium-diagnosis/diagnosis.md`

**Interfaces:**
- Consumes: Task 2's `fork_points.json`; Task 4's options; `plant_campaign.py`'s `command`/`run_all` for execution.
- Produces: `SETTINGS` = the control `{}` and, each alone: `{"iteration_budget": 60}`, `{"iteration_budget": 120}`,
  `{"iteration_budget": 300}`, `{"medium_arm_margin": 0.04}`, `{"medium_arm_margin": 0.03}`,
  `{"b5_arm_margin": 0.03}`, and the seeds pair compared with each other at budget 300:
  `{"iteration_budget": 300, "full_every_cycle": 1}` against `{"iteration_budget": 300}`. The list is cut to Checkpoint
  A's ruling.
  Every fork run carries the original run's arguments plus `--prediction --seed-detail`, in a new directory per
  setting (`experiment_runs/medium-diag/forks/<setting tag>`).
  `select(fork_points, per_cell=5) -> list` (the failed runs with a fork point, in seed order per cell);
  `one_cycle_runs(points, settings) -> list[run]` (`--fork-cycles 1`); `closed_loop_runs(one_cycle_results) -> list[run]`
  (only point x setting whose forked plan was feasible, plus the control); `report(results) -> str` (per point and
  setting: forked plan feasible, collision_m and the deepest row; closed loop: reached, stalled, min clearance;
  `cycle_wall_s` median against the 0.4 s period; deadline-cut cycles excluded and counted).

- [ ] **Step 1: Write the failing tests:** `test_select_takes_five_per_cell_in_seed_order`;
  `test_each_setting_changes_one_thing` (every setting but the seeds pair differs from the control in one key; the
  pair differs from its partner in `full_every_cycle` alone); `test_closed_loop_only_after_a_feasible_fork`;
  `test_control_must_reproduce_the_original` (a control whose closed-loop summary differs from the original run is
  reported as a reproduction failure); `test_deadline_cut_cycles_are_excluded`.
- [ ] **Step 2: Run** `cpytest tools/plant/test_fork_replay.py`. Expected: FAIL (no module).
- [ ] **Step 3: Implement**, then **Run** the tests. Expected: PASS. Commit
  `Medium diagnosis Task 5: fork_replay.py and the forks stage`.
- [ ] **Step 4: Run the forks:** one-cycle first; give the user the closed-loop run count and a time estimate (from
  the one-cycle wall times) before starting it; then `caffeinate -i bash .../diag_ensembles.sh forks`; `pmset -g log`.
- [ ] **Step 5: Write** `diagnosis.md` section "Stage B" (the matrix per question, the control's reproduction, wall
  times against the period). Commit `Medium diagnosis Task 5: Stage B results`.
- [ ] **Step 6: Checkpoint B. Stop.** Hand over the matrix and the review prompt; record the user's reading.

### Task 6: The write-up (spec Stage C)

**Files:**
- Modify: `docs/experiments/medium-diagnosis/diagnosis.md`, `docs/roadmap.md` (decision log; step 4's 5B notes)

- [ ] **Step 1: Write** per question: the cause (layer and decision), the evidence (run counts, fork results), how
  sure it is, and what 5B should consider, each tied to its evidence. Add the 2026-11-18 data
  (`experiment_runs/e3/diag`, on d633e94) as a preliminary note, and a problems-and-resolutions table.
- [ ] **Step 2: Add** a roadmap decision-log entry (the diagnosis's findings and the 5B candidates) and update the 5B
  notes (the door stall and the pivot: answered, with a link).
- [ ] **Step 3: Check** every number in the write-up against the reports (`report.md`, the forks report).
- [ ] **Step 4: Commit** `Medium diagnosis Task 6: the write-up and the roadmap entry`; hand the final review prompt
  to the user. The `diag-medium-budget` worktree (the user's uncommitted spike) is left alone; ask the user whether
  to remove it.
