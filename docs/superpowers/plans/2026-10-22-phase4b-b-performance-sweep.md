# Phase 4B-B: B5 Performance Sweep and Re-freeze Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Settle B5's solver, horizon, grid and rate with measured headroom, fix the 3D-clearance shortfall and the missing panel-against-arm-link rows, and re-freeze B5 as the Phase 5 base.

**Architecture:** Every candidate change is a switch in `wholebody_mpc.yaml` whose default reproduces `b5-frozen-2026-10-22`; the controller stays ROS-free, so a C++ replay bench (`mpc_replay`) drives `Controller::step` on recorded 4B-A bags and a Python driver sweeps variants and applies the shortlist rules. Two live campaigns confirm the shortlist and re-freeze.

**Tech Stack:** ROS 2 Jazzy C++17 (rclcpp, rosbag2_cpp), Eigen, FCL, OSQP 0.6 (`osqp_vendor`), BLASFEO + HPIPM (new `hpipm_vendor`), CasADi-generated model, gtest, launch_testing, pytest, Python tools in `tools/`, Unity via the Unity CLI.

**Spec:** docs/superpowers/specs/2026-10-22-phase4b-b-performance-sweep-design.md (read it with this plan; "Section N" refers to it).

## Global Constraints

- Branch `phase4b-b-sweep`; one commit per change; never commit to `main`; the user merges; the tag stays local until the user says otherwise.
- Every new key defaults to the frozen B5 (`b5-frozen-2026-10-22`, `ae3f640`); a YAML without the new keys loads with those defaults.
- Spec values: bench target p99 <= 10 ms and max <= 20 ms at 50 Hz; clearance criterion 0.045 m; arm/panel margin 0.05 m, base margin 0.12 m; `iter2_budget` 0.5; `iter2_cost_tolerance` 0.01 (relative); `rti_max_deviation` base 0.05 m, yaw 0.05 rad, joints 0.05 rad; `solver.publish_margin_s` 0.002; `fallback_s` 0.3; fidelity >= 95 % of cycles with stage-1 within 1 mm / 1 mrad and iterations within 10 %; live R26: p99 < period, misses < 1 % of cycles, tracking <= 0.10 rad; never below 20 Hz.
- Runtime nodes and ROS package code in C++ (`mpc_replay` included); launch files, config tests and `tools/` in Python.
- Container helper (recreate in the scratchpad if missing): `c.sh` = `docker exec mm-motion-planning-ma-robot-sim-1 bash -c 'source $ROS_WS/install/setup.bash 2>/dev/null; cd $ROS_WS && '"$1"`. Builds with `MAKEFLAGS=-j2`; tests: `colcon build --packages-select <pkgs> && colcon test --packages-select <pkgs> --event-handlers console_direct+ && colcon test-result --verbose` (`--ctest-args -R <test>` for one gtest). `pkill -f "[c]ontrol_description"` before mock tests.
- Live runs: `python3 tools/run_nav_scenario.py <scenario> --controller wholebody --mpc-rate-hz 50 [--mpc-config FILE]` under `caffeinate -i`; `--new-epoch` when Unity is not in Play; stop Play after runs; never `scene_capture` during runs; reviewers never use the container or `pkill` during live runs.
- No host-absolute paths, credentials, Unity `Library/`, ROS `build/ install/ log/`, or `experiment_runs/` in commits. Native linux/arm64 and linux/amd64 builds; emulation only for the one-off amd64 compile check.
- Failed criteria are reported with causes first; criteria are never relaxed without the user.

### Plan-level decisions not spelled out in the spec

- **Config:** `MpcConfig` gains `SolverConfig solver` (`backend` "osqp"|"hpipm", `warm_start`, `deadline_cap`, `publish_margin_s`, `HpipmOptions hpipm`), `int qp_iterations`, `double iter2_budget`, `double iter2_cost_tolerance`, `bool rti_split`, `RtiDeviation rti_max_deviation`; `CollisionConfig` gains `bool self_rows`, `int self_pairs_per_stage` (2), `std::vector<std::string> self_links`. Existing `solver_eps`/`solver_max_iter` stay. Every new key is optional in the YAML.
- **Controller options:** a new trailing constructor argument `ControllerOptions options = {}` (`qp_iterations`, `iter2_budget`, `iter2_cost_tolerance`, `warm_start`, `deadline_cap`, `period_s` (0 = no deadline logic), `publish_margin_s`, `rti_split`, `rti_max_deviation`), so every existing construction keeps compiling and behaving as frozen.
- **Solver selection:** `std::unique_ptr<QpSolver> make_solver(const MpcConfig &)` in `src/solver_factory.cpp`, used by the node and the bench.
- **Deadline cap:** `QpSolver::set_time_limit(double seconds)` (virtual, default no-op; 0 = none). OSQP maps it to `osqp_update_time_limit`; HPIPM ignores it (its iterations are bounded by `hpipm.max_iter`). A cycle with no time left before the solve skips it and counts a solver failure.
- **Miss split:** besides `timer_lateness_s` the node records `cycle_cpu_s` (`CLOCK_THREAD_CPUTIME_ID` over the cycle); the stats summary counts `misses_compute` (cpu_s > period) and `misses_schedule` (the rest).
- **Variant configs live:** node parameter `config_file` (default the installed `wholebody_mpc.yaml`), launch argument `mpc_config_file`, runner `--mpc-config FILE` (run suffix `-cfg<stem>`, recorded in `summary.json` as `mpc_config_file`).
- **Replay clocks:** message order and the arm stream's offset use bag receive times; the controller's `now` is the recorded cycle's `header.stamp` (the node passes the ROS time). The arm stage-0 previous command is the node's spline sampler (moved to `arm_stream.hpp`) applied to the latest `/arm_controller/joint_trajectory` at the cycle's offset from that message's receipt.
- **Second-iteration cost:** `Formulation::objective(const Inputs &, const Plan &)` = the sum of 0.5 w r^2 over the residuals `build()` linearizes, evaluated at the plan, without slack terms.
- **Corpus manifest:** `docs/experiments/wholebody-mpc/phase4b-b/corpus.json` (run directories relative to the repository, cell tag, flags `shortfall`, `timing_heavy`).

## Review Focus

1. **A YAML without the new keys** (the tag's file, a partial variant overlay): loads with frozen defaults. Test: Task 1 `MpcConfig.FrozenYamlLoadsWithDefaults`.
2. **A mission bag with several FollowPath goals and in-place preemptions:** the bench calls `start`/`update_goal` as the node did. Test: Task 2 `Replay.SecondPlanAfterGoalEndStartsANewGoal` and `Replay.PlanDuringGoalUpdatesInPlace`.
3. **HPIPM on stages with no inequality rows, a terminal with no rows, or rows with both bounds infinite:** solves like OSQP. Test: Task 5 `Backends.HandleStagesWithoutRows`.
4. **The second solve fails numerically, or the first already failed:** the first iterate is kept, no extra failure counted. Test: Task 6 `Controller.Iter2FailureKeepsFirstIterate`.
5. **An RTI feedback cycle with nothing prepared** (first cycle of a goal, after a hold or blocked cycle, after braking): runs the full cycle. Test: Task 8 `Controller.RtiWithoutPreparationRunsFullCycle`.

---

### Task 1: Diagnostics, variant configs, recording, frozen golden

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_interfaces/msg/MpcCycle.msg`; `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/{formulation,controller,collision}.hpp`; `src/formulation.cpp` (`load_mpc_config`); `src/wholebody_mpc_node.cpp`; `config/wholebody_mpc.yaml`; `test/test_mpc_config.py`; `ros2_ws/src/mobile_manipulator_navigation/launch/navigation.launch.py`; `tools/run_nav_scenario.py`, `tools/test_run_nav_scenario.py`
- Create: `test/test_golden.cpp`, `test/data/frozen_golden.json`, `test/data/frozen_wholebody_mpc.yaml`

**Interfaces:**
- Produces: the config structs of the plan-level decisions (fields only; no behaviour yet); `CycleTiming` gains `prep_s`, `feedback_s`; `CycleOutput` gains `bool iter2_used, iter2_rejected`; `CollisionStats` gains `int self_pairs`; `MpcCycle` gains `bool iter2_used, iter2_rejected; float64 prep_s, feedback_s, timer_lateness_s, cycle_cpu_s; int32 self_pairs`; node param `config_file`; launch arg `mpc_config_file`; runner `--mpc-config`; `/scenario/movers` in `WHOLEBODY_BAG_TOPICS`.

- [ ] **Step 1: Generate the golden fixture on the unchanged code.** `test_golden.cpp` runs two fixed closed loops with `make_controller()`-style construction from `test/data/frozen_wholebody_mpc.yaml` (the file as of `ae3f640`, copied in this step) (the 1.34 m beam world of `Guard.LowBeamKeepsComfortAndLowers` and the shelf world of `RightSwing.ShelfLeftOfTheApproachSwingsRight` in `test_controller.cpp`), 60 cycles each, and with `MPC_WRITE_GOLDEN=1` writes every cycle's status, command (8 values) and iterations to `test/data/frozen_golden.json`. Build, run once with the variable set, commit the fixture with this task.
- [ ] **Step 2: Write the failing tests:** `Golden.DefaultsReproduceFrozenOutput` (commands within 1e-9, same statuses and iterations); `MpcConfig.FrozenYamlLoadsWithDefaults` (load `test/data/frozen_wholebody_mpc.yaml`: `solver.backend == "osqp"`, `solver.warm_start`, `!solver.deadline_cap`, `qp_iterations == 1`, `!rti_split`, `!collision.self_rows`); `test_mpc_config.py::test_new_keys_default_to_frozen`; `test_run_nav_scenario.py::test_mpc_config_suffix_and_launch_argument` (`--mpc-config x/fast.yaml` gives suffix `-cfgfast` and ` mpc_config_file:=<abs path in container>`); `test_run_nav_scenario.py::test_wholebody_bag_records_movers`.
- [ ] **Step 3: Run them:** the golden test passes (unchanged code), the config and runner tests fail on missing keys/options.
- [ ] **Step 4: Implement:** the struct fields and optional YAML keys (Section 4 names, defaults as above; write them into `wholebody_mpc.yaml` with comments in the file's style); `MpcCycle` fields; the node fills `timer_lateness_s` (cycle start against the previous scheduled time + period) and `cycle_cpu_s` and adds `misses_compute`/`misses_schedule` to the stats file; `config_file` parameter; launch argument and runner option; movers topic in the bag.
- [ ] **Step 5: Run** `mobile_manipulator_interfaces mobile_manipulator_mpc mobile_manipulator_navigation` package tests and `python3 -m pytest tools -q`: all green, 235 existing mpc tests unchanged.
- [ ] **Step 6: Commit** `Add B5 cycle diagnostics, variant config files, mover recording and a frozen golden test`.

### Task 2: Replay bench and fidelity check (Section 3)

**Files:**
- Create: `include/mobile_manipulator_mpc/arm_stream.hpp`, `src/arm_stream.cpp` (the node's `sample` moved here); `include/mobile_manipulator_mpc/replay.hpp`, `src/replay.cpp`; `src/mpc_replay.cpp`; `test/test_replay.cpp`; `tools/mpc_sweep.py`, `tools/test_mpc_sweep.py`; `docs/experiments/wholebody-mpc/phase4b-b/corpus.json`
- Modify: `src/wholebody_mpc_node.cpp` (use `arm_stream.hpp`), `CMakeLists.txt`

**Interfaces:**
- Consumes: Task 1 config and output fields.
- Produces:
  - `void sample_trajectory(const trajectory_msgs::msg::JointTrajectory &, double t, Joints & q, Joints & v)`.
  - `struct RecordedCycle { double stamp, bag_time; State x; std::string status; int iterations; bool deadline_missed; Eigen::Vector2d executed; Joints arm_v; std::vector<State> plan; }`, `struct RecordedGoal { double bag_time; std::vector<Eigen::Vector2d> points; Pose2 goal; }`, `struct Recording { std::vector<RecordedCycle> cycles; std::vector<RecordedGoal> goals; double period_s; }`; `Recording read_recording(const std::string & bag_uri, const std::array<std::string,6> & joint_names)`.
  - `struct ReplayOptions { double rate_hz = 0; /* 0: recorded */ bool recorded_misses = false; int clearance_stages; }`; `ReplayResult replay(Controller &, const Recording &, const World &, const ReplayOptions &, const CollisionModel &)` with per-cycle vectors `total_s, solve_s, collision_s, prep_s, feedback_s, iterations, status, true_clearance_m, lin_error_m, progress_m, contour_m` and `fidelity { status_match, stage1_match, iterations_match }` (fractions; computed against `Recording` when the rate is the recorded one).
  - `mpc_replay --run DIR --config YAML [--rate-hz R] [--recorded-misses] [--repeats 3] --output FILE` writes `{run, cell, variant, rate_hz, cycles, timing{total_s,solve_s,collision_s,prep_s,feedback_s: {p50,p95,p99,max}}, iterations{p50,p99,max}, statuses{}, solver_failures, true_clearance_min_m, true_clearance_p01_m, lin_error_p95_m, progress_median_m, contour_mean_m, fidelity{}, per_cycle{...}}`; timing statistics are the median over repeats.
  - Python: `merge_overlay(base: dict, overlay: dict) -> dict`; `run_grid(corpus: Path, variants: Path, out: Path, rate_hz: float)`; `shortlist(results: list[dict], frozen: list[dict]) -> list[dict]` implementing Section 5.4 rules 1-5 in order, each candidate annotated with the rules it failed.

- [ ] **Step 1: Write the failing tests.** `test_replay.cpp` writes small synthetic bags with rosbag2_cpp (one `/plan`, cycles with plans and `/cmd_vel`): `Replay.ReadsCyclesAndGoalsInBagOrder`; `Replay.FrozenConfigReproducesItsOwnRecording` (record a closed loop from the test controller into a bag, replay it: `fidelity.stage1_match == 1.0`); `Replay.SecondPlanAfterGoalEndStartsANewGoal`; `Replay.PlanDuringGoalUpdatesInPlace`; `Replay.RateEmulationKeepsEverySecondAndAHalfCycle` (50 Hz recording at 20 Hz: spacing >= 0.05 s - 1e-6). `test_mpc_sweep.py`: `test_merge_overlay_nested`, `test_shortlist_rule_order` (a candidate failing clearance is out even with the best p99; ties go to fewer switches), `test_shortlist_none_passing_returns_closest_with_failed_rules`.
- [ ] **Step 2: Run them** (fail: no `replay.hpp`, no `tools/mpc_sweep.py`).
- [ ] **Step 3: Implement** `arm_stream`, `read_recording` (reuse `clearance_bag.cpp`'s reader), `replay` (per cycle, as the node: `reset_command` with `executed` and `arm_v`, `set_external_hold(status == "blocked")`, `step(stamp, x, true, world, at_rest)`, `note_failure()` when `recorded_misses ? deadline_missed : total_s > period`), the true clearance (min of `CollisionModel::nearest` over stages 1..`clearance_stages` = ceil(slack_gate_horizon_s / dt) of the output plan; with self rows also `nearest_self`, Task 7), the linearization error (row prediction against `pair_distance` at the planned stage), the CLI, and the driver.
- [ ] **Step 4: Run the tests** (pass).
- [ ] **Step 5: Fidelity on real bags:** write `corpus.json` (Section 5.1), then `python3 tools/mpc_sweep.py fidelity --corpus ... --config <installed yaml> --recorded-misses`. Expected: every run `stage1_match >= 0.95`, `iterations_match >= 0.95`, `status_match >= 0.95`. **Checkpoint:** if any run is below, stop and report the runs, the first diverging cycle and the cause to the user.
- [ ] **Step 6: Commit** `Add the B5 replay bench and the sweep driver`.

### Task 3: Warm-start switch and deadline-aware solve cap (Sections 4.3, 4.7)

**Files:** Modify `include/mobile_manipulator_mpc/{qp,osqp_solver,controller}.hpp`, `src/{osqp_solver,controller}.cpp`, `src/wholebody_mpc_node.cpp`, `src/replay.cpp`; create `include/mobile_manipulator_mpc/solver_factory.hpp`, `src/solver_factory.cpp`; test `test/test_controller.cpp`, `test/test_osqp_solver.cpp`.

**Interfaces:**
- Consumes: `ControllerOptions`, `SolverConfig` (Task 1).
- Produces: `virtual void QpSolver::set_time_limit(double seconds)`; `SolveResult.status == "time_limit"` with `ok == false`; `make_solver(const MpcConfig &)` (OSQP only for now; "hpipm" throws `std::invalid_argument` until Task 5).

- [ ] **Step 1: Failing tests:** `Osqp.TimeLimitStopsTheSolve` (a 1e-6 s limit on the slow slack case returns `ok == false`, status `time_limit`); `Controller.WarmStartOffPassesNoWarmSolution` (spy solver records `warm == nullptr` on every cycle); `Controller.DeadlineCapAlreadyExceededFallsBack` (`period_s` 1e-6, so every cycle is past its period before the solve: status Fallback, spy solver not called); `Controller.DeadlineCapOffNeverSetsALimit`.
- [ ] **Step 2: Run** (fail).
- [ ] **Step 3: Implement**; the limit is `period_s - publish_margin_s - (time since step() began)`.
- [ ] **Step 4: Run** the mpc suite (green, golden included).
- [ ] **Step 5: Commit** `Add the warm-start switch and the deadline-aware solve cap`.

### Task 4: `hpipm_vendor` (Section 4.1)

**Files:** Create `ros2_ws/src/hpipm_vendor/{package.xml,CMakeLists.txt,hpipm_vendor-extras.cmake.in,README.md}`.

**Interfaces:** Produces imported targets `hpipm_vendor::hpipm` and `hpipm_vendor::blasfeo` (headers `hpipm_d_ocp_qp*.h`, `hpipm_d_part_cond.h`) via `find_package(hpipm_vendor REQUIRED)`.

- [ ] **Step 1:** `ExternalProject_Add` for BLASFEO then HPIPM from release tarballs of their GitHub releases, each with `URL_HASH SHA256=...`. Take the newest release tags that build together; if they do not, the commits acados's latest release pins. Record tags and hashes in `README.md`. `BLASFEO_TARGET` from `CMAKE_SYSTEM_PROCESSOR`: `x86_64` → `X64_AUTOMATIC`, `aarch64|arm64` → `ARMV8A_ARM_CORTEX_A57`, cache option `HPIPM_VENDOR_GENERIC=OFF` forces `GENERIC`. Static, PIC, `-O2`.
- [ ] **Step 2: Build natively:** `MAKEFLAGS=-j2 colcon build --packages-select hpipm_vendor` → succeeds; a 10-line `try_compile` smoke test in the package's `BUILD_TESTING` solves a 2-stage OCP QP (`ament_add_gtest(test_hpipm_smoke ...)`), green.
- [ ] **Step 3: One-off amd64 compile check:** `docker run --rm --platform linux/amd64 -v <repo>/ros2_ws/src/hpipm_vendor:/src:ro ros:jazzy bash -c 'cd /tmp && cp -r /src v && cd v && cmake -B b && cmake --build b -j2'` → builds. Record the result in the README (no CI change).
- [ ] **Step 4: Commit** `Add hpipm_vendor (BLASFEO and HPIPM, pinned, per-architecture target)`.

### Task 5: `HpipmSolver` and backend selection (Section 4.2)

**Files:** Create `include/mobile_manipulator_mpc/hpipm_solver.hpp`, `src/hpipm_solver.cpp`, `test/test_solver_backends.cpp`; modify `test/test_osqp_solver.cpp` (move the shared cases), `src/solver_factory.cpp`, `package.xml`, `CMakeLists.txt`.

**Interfaces:**
- Consumes: `hpipm_vendor::hpipm`, `QpProblem` (`qp.hpp`).
- Produces: `struct HpipmOptions { std::string mode = "balance"; int max_iter = 50; double tol = 1e-3; int partial_condensing_blocks = 0; bool warm_start = false; }`; `class HpipmSolver : public QpSolver` (`setup`, `solve`, `int setup_count() const`); `make_solver` returns it for backend "hpipm" (tol = `solver.eps`).

- [ ] **Step 1: Failing tests** in `test_solver_backends.cpp`, a typed test over `{OsqpSolver, HpipmSolver, HpipmSolver with partial_condensing_blocks 4}`: the existing LQR/Riccati, box-bound, slack-activation, quadratic-slack, upper-bounded slack and dense-KKT cases (solutions within 1e-4 of the KKT reference); `Backends.PerRowSlackWeightsMatchOsqp`; `Backends.HandleStagesWithoutRows`; `Backends.SamePatternUpdatesWithoutSetup`; `Hpipm.PartialCondensingEqualsNone` (within 1e-6); `Hpipm.NumericalFailureIsReportedNotThrown`.
- [ ] **Step 2: Run** (fail).
- [ ] **Step 3: Implement** the one-to-one mapping (Section 4.2): per stage `nx = kNz`, `nu = kNu`, boxes as `idxbx/idxbu` (stage 0's state box omitted, `dx0` fixed through the first state's bounds), general rows `C = Cx, D = Cu`, each slacked row a soft constraint (`idxs`) with `Zl = Zu = slack_quadratic`, `zl = zu = the row's L1 weight`, the infinite side masked; `setup` creates dims/memory once, `solve` only updates values; partial condensing through `d_part_cond_qp_*` when blocks > 0; warm start from the previous primal/dual when `warm_start`.
- [ ] **Step 4: Run** the mpc suite (green).
- [ ] **Step 5: Commit** `Add the HPIPM backend with partial condensing and backend selection`.

### Task 6: Second QP iteration with a step check (Section 4.4)

**Files:** Modify `include/mobile_manipulator_mpc/{formulation,controller}.hpp`, `src/{formulation,controller}.cpp`; tests `test/test_formulation.cpp`, `test/test_controller.cpp`.

**Interfaces:**
- Produces: `double Formulation::objective(const Inputs &, const Plan &) const`; `CycleOutput::iter2_used`, `iter2_rejected`.

- [ ] **Step 1: Failing tests:** `Formulation.ObjectiveMatchesQpPredictionForSmallSteps` (objective(apply(nominal, s)) - objective(nominal) equals the QP cost of s within 1e-3 relative for a 1e-3-scaled step); `Controller.Iter2RejectsAWorseIterate` (a spy solver whose second solution increases the worst violation: `iter2_rejected`, plan equals the first); `Controller.Iter2BudgetSkipsTheSecondSolve` (`period_s` 1e-6, so the first solve always exceeds `iter2_budget`: `!iter2_used`, exactly one solve); `Controller.Iter2FailureKeepsFirstIterate`; `Controller.Iter2RaisesClearanceUnderTheBeam` (the `Guard.LowBeamKeepsComfortAndLowers` world with `qp_iterations` 2: minimum true clearance >= 0.049 m and not below the one-iteration run); `Controller.Iter2NotWorseBesideTheShelf` (the `RightSwing.ShelfLeftOfTheApproachSwingsRight` world, the spec's constrained-like case: minimum true clearance with 2 iterations >= the one-iteration run's and >= 0.045 m). The live shortfall cells are measured on the bench (Task 9) and in campaign 1.
- [ ] **Step 2: Run** (fail).
- [ ] **Step 3: Implement** in `Controller::step` after a successful first solve (spec Section 4.4): plan1 = apply; rows at plan1 (their distances are plan1's true distances); build and solve around plan1, warm from the first solution with zero primal; plan2 = apply; violation(plan2) from `rows` over stages within `slack_gate.horizon_s`; keep plan2 iff violation(plan2) <= violation(plan1) + 1e-6 and objective(plan2) <= (1 + iter2_cost_tolerance) objective(plan1); the slack gate uses the kept iterate's rows and slack. Timing adds into `collision_s`, `setup_s`, `solve_s`; iterations sum.
- [ ] **Step 4: Run** the mpc suite (green; golden unchanged at `qp_iterations` 1).
- [ ] **Step 5: Commit** `Add a second QP iteration per cycle with a step check`.

### Task 7: Panel-against-arm-link rows (Section 4.6)

**Files:** Modify `include/mobile_manipulator_mpc/collision.hpp`, `src/collision.cpp`, `src/formulation.cpp` (row budget), `src/clearance_3d.cpp`, `src/replay.cpp`, `config/wholebody_mpc.yaml`, `tools/run_nav_scenario.py` (record `clearance_self_m`); tests `test/test_collision.cpp`, `test/test_clearance_3d.cpp`, `test/test_controller.cpp`.

**Interfaces:**
- Produces: rows with `obstacle == "self/<arm link>"`; `CollisionModel::nearest_self(const State &) const -> std::optional<NearestPair>` (panel against `self_links`); `Formulation::rows_per_stage() == pairs_per_stage + (self_rows ? self_pairs_per_stage : 0)`; `clearance_3d` JSON gains `self_min_m`, `self_link`; `summary.json` gains `clearance_self_m`.

- [ ] **Step 1: Fix the link list from FK.** With `carry_pose_search`-style FK over `home`, `carry`, `vertical_carry`, `vertical_carry_right`, `lowered` and the 4B-A campaign-3 plan states (bench), list the panel's minimum distance to each arm link. Links that are always within 0.05 m by construction (the mount) are excluded; write the rest to `collision.self_links` with a comment listing the exclusions and their distances.
- [ ] **Step 2: Failing tests:** `Collision.SelfRowsMatchFclReference` (distance and gradient against `collision_reference.hpp` within 1e-6 / finite differences 1e-5); `Collision.SelfRowsHaveTheirOwnBudget` (8 obstacle rows plus 2 self rows at a crowded stage, none dropped); `Collision.SelfRowsOffAddNoRows`; `Controller.SelfRowsKeepTheRightSwingStreamOffTheArm` (the `vertical_carry_right` all-joint transition from `carry`: planned `nearest_self` >= 0.05 m - 1e-3 on every stage); `Clearance3d.ReportsTheSelfMinimum`.
- [ ] **Step 3: Run** (fail).
- [ ] **Step 4: Implement**; self rows use `margin_m` and the `slack_inside` weighting when inside it; `pair_distance` handles `self/<arm link>`.
- [ ] **Step 5: Run** the mpc suite and `python3 -m pytest tools -q` (green).
- [ ] **Step 6: Commit** `Add panel-against-arm-link collision rows and score them`.

### Task 8: Preparation/feedback split (Section 4.5)

**Files:** Modify `include/mobile_manipulator_mpc/controller.hpp`, `src/controller.cpp`, `src/wholebody_mpc_node.cpp`, `src/replay.cpp`; test `test/test_controller.cpp`.

**Interfaces:**
- Produces: `void Controller::prepare(double t_next, const World & world)`; with `rti_split` the node calls it right after publishing, the bench after each step; `step()` uses the prepared QP when `|now - t_next| <= dt / 2` and the deviation is within `rti_max_deviation`, else runs the full cycle; `timing.prep_s`, `timing.feedback_s`.

- [ ] **Step 1: Failing tests:** `Controller.RtiFeedbackAtPredictedStateEqualsFullSolve` (commands within 1e-6); `Controller.RtiLargeDeviationRunsFullCycle` (0.06 m base offset); `Controller.RtiWithoutPreparationRunsFullCycle` (first cycle, after a hold, after `set_external_hold`, after braking); `Controller.RtiReportsPrepAndFeedbackTimes`.
- [ ] **Step 2: Run** (fail).
- [ ] **Step 3: Implement:** `prepare` advances the nominal to `t_next` from the plan's predicted state, computes rows, tightness, side picker and the QP with `dx0 = 0`; `step` sets `dx0 = (x0, last_command) - nominal stage 0` and solves; everything after the solve is unchanged. The side picker and tightness update once per cycle (in `prepare` when split).
- [ ] **Step 4: Run** the mpc suite and the node launch test (green).
- [ ] **Step 5: Commit** `Add the RTI preparation/feedback split`.

### Task 9: Offline sweep and shortlist (Section 5) — checkpoint

**Files:** Create `docs/experiments/wholebody-mpc/phase4b-b/variants-stage-a.yaml`, `variants-stage-b.yaml`; `docs/experiments/wholebody-mpc/phase4b-b.md` (bench and sweep sections); results JSON under `experiment_runs/bench/` (git-ignored).

- [ ] **Step 1:** Stage A overlays exactly as the Section 5.2 table (one factor per overlay, the rest frozen); OSQP tuned levels: `eps` {1e-3, 5e-3}, `adaptive_rho_interval` {25, 100}, `max_iter` {200, 500}; collision fidelity: `pairs_per_stage` {6, 8, 12}, `activation_m` {0.6, 0.8, 1.0}, row stride {1, 2}. Run at 50 Hz and 20 Hz (emulated), 100 Hz compute only, 3 repeats, Unity not in Play, container otherwise idle, under `caffeinate -i`.
- [ ] **Step 2: Grid trigger:** if N 30 beats N 20 on shortfall-run clearance or median progress by more than 5 % but fails rule 3, stop and add the `dt_schedule` step (report to the user first); else record "trigger not met" with the numbers.
- [ ] **Step 3:** Stage B factorial (Section 5.3) with `self_rows` on.
- [ ] **Step 4:** `shortlist(...)`; write the stage A/B tables (per cell: p50/p99/max, iterations, failures, clearance min and p01 on the four shortfall runs, progress, contour), the shortlist and every rule failure into `phase4b-b.md`.
- [ ] **Step 5: Commit** `Record the Phase 4B-B offline sweep and shortlist`. **Checkpoint:** report the shortlist (or the closest configurations with failed rules and causes) to the user before campaign 1.

### Task 10: Campaign 1 (Section 7) — checkpoint

**Files:** `docs/experiments/wholebody-mpc/phase4b-b/<config>.yaml` (each shortlisted configuration as a full file); `phase4b-b.md` (campaign 1 section); `tools/summarize_nav_runs.py` (+ test): `criteria_4bb(summary: dict) -> dict[str, bool]` (contact, success, `min(clearance_3d_m, clearance_self_m) >= 0.045`, p99 < period, misses < 1 %, tracking <= 0.10 rad, sign flips 0) and the miss split columns.

- [ ] **Step 1:** Failing test `test_criteria_4bb` with a passing and a failing summary per criterion; implement; pass; commit with the configs.
- [ ] **Step 2:** Run the Section 7 table (seven cells; 2 runs per cell per configuration at 50 Hz, 1 outside scripted constrained and store S with three configurations; the best configuration once per cell at 20 Hz) with `--mpc-config`, under `caffeinate -i`, one Unity session; stop Play after.
- [ ] **Step 3:** Per-run criteria table, failures with causes, miss split (compute vs scheduling with `cycle_cpu_s` and `timer_lateness_s`), into `phase4b-b.md`.
- [ ] **Step 4:** Apply the Section 7 decision rules (configuration, then rate). Commit `Record Phase 4B-B campaign 1`. **Checkpoint:** report the chosen configuration and rate (or the failed criteria and causes) to the user; scheduling options only reported.

### Task 11: Flip the defaults

**Files:** `config/wholebody_mpc.yaml`, `launch/navigation.launch.py` / runner default rate if 50 Hz is chosen (`mpc_rate_hz`, `DEFAULT_MPC_RATE_HZ`), `docs/roadmap.md` (comparability log entry), `test/test_golden.cpp`; `test/data/` untouched.

- [ ] **Step 1:** Add `MpcConfig.InstalledYamlIsTheChosenConfiguration` asserting the chosen values (the golden test keeps pinning the frozen copy).
- [ ] **Step 2:** Set the chosen values; run every package suite and `pytest tools` (green).
- [ ] **Step 3:** Comparability-log entry: what changed from `b5-frozen-2026-10-22` and why (campaign 1 numbers). Commit `Make the campaign-1 configuration B5's default`.

### Task 12: Campaign 2 and exit report (Section 8) — checkpoint

- [ ] **Step 1:** Run the Section 8 table on the new defaults (movers recorded), under `caffeinate -i`; stop Play after.
- [ ] **Step 2:** Exit table with `criteria_4bb` plus the carried 4B-A criteria and rulings; V2 timing at the chosen rate; every failure with its cause.
- [ ] **Step 3:** Commit `Record the Phase 4B-B re-freeze campaign`. **Checkpoint:** the user rules on any failed criterion before the tag.

### Task 13: Documentation, F4 brief, tag (Section 9)

**Files:** `docs/adr/0013-mpc-solver-iterations-and-rti.md`; `docs/roadmap.md` ("Phase 4B-B outcome", decision-log resolutions for the MPC rate, the 3D clearance and B6 deferred to Phase 6 optional, "Handed to Phase 5"); `docs/running-the-stack.md` (switches, `--mpc-config`, re-frozen recipe); `docs/experiments/wholebody-mpc/phase4b-b.md` (final); F4 brief as a roadmap subsection under Phase 5 prerequisites.

- [ ] **Step 1:** Write the ADR (options OSQP / HPIPM / acados, decision, consequences, revisit conditions) and the docs; the F4 brief covers what F4 is, what it would show against B5, its cost, and the campaign data on the arm stream's smoothness.
- [ ] **Step 2:** Run every package suite and `pytest tools` once more (green); commit `Document Phase 4B-B and hand B5 to Phase 5`.
- [ ] **Step 3:** `git tag -a b5-frozen-<date> -m "B5 re-frozen after Phase 4B-B (Phase 5 base)"` (local); ask the user before pushing the branch or the tag; update the memory files (phase status, 4B-B planning, a Phase 5 handoff).
