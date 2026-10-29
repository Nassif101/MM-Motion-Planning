# Phase 5A: Medium-Horizon Layer (F2 Bézier) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A medium-horizon optimizer (piecewise Bézier over base SE(2) and four arm joints, multi-start SCP) that runs beside B5 and gives it arc-length references, so B5 approaches doors aligned, picks the swing side by planning, folds under low beams and sequences close sections.

**Architecture:** A new ROS-free C++ library `mobile_manipulator_medium` (Bézier basis, problem, dense-QP backends, SCP, planner) links B5's `mpc` library for FK, collision and scene; `medium_horizon_node` runs it at 5 Hz against the route B5 announces. B5 gains `reference_source: medium`, in which it tracks the medium path, yaw, speed and posture and falls back to the frozen behaviour per cycle. An offline bench (open-loop on bag snapshots, closed-loop with B5's `Controller::step` and a kinematic plant) does the tuning; two live campaigns accept it.

**Tech Stack:** ROS 2 Jazzy C++17 (rclcpp, tf2, rosbag2_cpp), Eigen, FCL, OSQP 0.6 (`osqp_vendor`), BLASFEO + HPIPM (`hpipm_vendor`, dense QP), DAQP (new `daqp_vendor`), gtest, launch_testing, pytest, Python tools in `tools/`, Unity C# (EditMode tests) via the Unity CLI.

**Spec:** docs/superpowers/specs/2026-10-29-phase5a-medium-horizon-design.md (read it with this plan; "Section N" refers to it).

## Global Constraints

- Branch `phase5a-medium`; one commit per change; never commit to `main`; the user merges; tags stay local until the user says otherwise.
- Frozen B5 stays bit-identical by default: `reference_source` defaults to `nav2`, `medium:=none` is the launch default, and `test_golden` must pass unchanged after every task. The installed `nav_operating_envelope.yaml` breakaway values do not change in 5A (the golden test reads them).
- Spec values: medium rate 5 Hz; horizon 8 s, 4 cubic segments, C2; collision samples every 0.1 s; margins base 0.12 m, arm/panel 0.05 m; clearance hinge target 0.25 m; homotopy hysteresis 15 %; solve cap 0.15 s; full multi-start every 2 s, on a new `route_id` and on a draggable change; samples every 0.1 m of arc length; B5 fallback: plan older than 0.5 s, other `route_id`, or more than 0.3 m from the measured base; SCP at most 10 outer iterations; feasible = collision slack < 1 mm with torque and hull bounds held.
- Acceptance (Section 7): campaign A 7 cells × F2/B5 × 3; every F2 run no contact, 3D clearance ≥ 0.045 m, B5 MPC p99 ≤ 10 ms and misses < 1 %, medium p99 ≤ 150 ms, fallback < 1 % of moving time, arm tracking ≤ 0.10 rad; campaign B ≥ 30/34. Kinematic gate: all 7 cells reach the goal, no executed plan with collision slack, medium p99 ≤ 150 ms, no homotopy switch after the first choice in the swing-side cells.
- Runtime nodes and ROS package code in C++; launch files, config tests and `tools/` in Python.
- Container helper (recreate in the scratchpad if missing): `c.sh` = `docker exec mm-motion-planning-ma-robot-sim-1 bash -c 'source $ROS_WS/install/setup.bash 2>/dev/null; cd $ROS_WS && '"$1"`. Builds with `MAKEFLAGS=-j2`; tests: `colcon build --packages-select <pkgs> && colcon test --packages-select <pkgs> --event-handlers console_direct+ && colcon test-result --verbose` (`--ctest-args -R <test>` for one gtest). `pkill -f "[c]ontrol_description"` before mock tests.
- Live runs: `caffeinate -i python3 tools/run_nav_scenario.py <scenario> --controller wholebody --mpc-rate-hz 50 [--medium bezier]`; one timed run at a time, never overlapping Play with other load (fanless host); stop Play after each task (`unity editor_stop`); never `scene_capture` during runs.
- No host-absolute paths, credentials, Unity `Library/`, ROS `build/ install/ log/`, or `experiment_runs/` in commits. Native linux/arm64 and linux/amd64 builds.
- Failed criteria are reported with causes at the next CP; criteria are never relaxed without the user.
- Problems and decisions are recorded as they are solved: the problem/cause/resolution table in `docs/experiments/medium-horizon/phase5a.md` (created in Task 1) and the roadmap decision log.

### Plan-level decisions not spelled out in the spec

- **Variable count (spec 3.1 correction).** C2 cubic piecewise Bézier with 4 segments has 7 degrees of freedom per coordinate; fixing start position and velocity leaves 5, so 35 variables (the spec's "about 77" counted C1). Continuity and start-state equalities are eliminated by an affine map `c = M p + c0` from free parameters `p` to all control points, so every QP has only inequalities and bounds. The bench (Task 9) may raise `segments`.
- **Speed is derived**, not a separate variable: a sample's speed is |d(x, y)/dt| of the base curve.
- **Arc-length samples and turns in place.** A new sample is emitted every 0.1 m of arc length or every 0.1 rad of yaw change, whichever comes first, so a turn on the spot keeps its yaw samples; lookups by arc length return the last sample at that `s`.
- **Torque limits.** `statics.hpp` computes the static gravity torque of every arm joint (links below it plus the panel, the computation `carry_pose_search` already does). Default limit per joint = 1.10 × the largest |torque| over the poses B5 drives in (`carry`, `vertical_carry`, `vertical_carry_right`, `fold`); `level_extension` is a hold pose, not a driving pose. The numbers are printed at the Task 2 CP for a user ruling.
- **Fold pose** = 4B-A's fold limit `[0, 1.236, -2.156, 0, 0.920, 0]` (top 1.20 m, about 22 Nm), qualified with a new `fold` suite in `tools/run_wholebody_arm_qualification.py` modelled on `vertical_right`.
- **Friction feedforward** is a velocity-target offset per wheel: target += sign(command) × `jointFriction` / `damping` when |command| > 0, which cancels the joint's Coulomb friction in the damping drive at steady state. Switch: Unity CLI command `base_friction_feedforward --enabled`, runner `--base-friction-feedforward/--no-base-friction-feedforward` (default on from Task 1, run suffix `-bff` when on).
- **The scenario's obstacles** are per-scenario `obstacles:` entries (as `v2_gate_right_swing_nav`), so no Unity scene edit is needed: a 1.05 m gate at x -1.0 with a 1.23 m beam behind it in the delivery yard.
- **Shared ROS state.** The node's state assembly (TF `map -> base_footprint`, `/joint_states` merged into model order, freshness) moves from `wholebody_mpc_node.cpp` into a library `ros_state` (`include/mobile_manipulator_mpc/ros_state.hpp`), exported with `mpc`, used by both nodes.
- **Kinematic plant** (`Plant` in `test/closed_loop.hpp`) moves to `include/mobile_manipulator_mpc/kinematic_plant.hpp` so the medium bench can use it; the tests include it from there.
- **Route switch in B5.** `Controller::replace_route(const Route &, double goal_s, double s0_hint)` swaps the route in place (mode, warm plan, filters kept) with an explicit progress hint, so a medium path is never projected onto the whole route (dead ends, self-near routes). The node calls it only when a new plan is accepted or the source changes.
- **Plan latency in the kinematic bench.** A medium plan becomes visible to B5 at its start time plus its measured compute time, rounded up to the next B5 cycle.

## Review Focus

1. **A turn on the spot at the goal (store S):** arc length stalls while yaw changes; samples keep the yaw and B5's goal-heading logic is unchanged. Tests: Task 8 `Planner.TurnInPlaceKeepsYawSamples`, Task 10 `MediumReference.InPlaceTurnKeepsGoalHeading`.
2. **A FollowPath preemption while a plan for the old route is in flight:** B5 rejects it (fallback reason `route_id`) until a plan for the new route arrives. Test: Task 10 `MediumMode.PlanForOldRouteFallsBack`.
3. **A remaining route shorter than the horizon:** the plan stops at the goal without overshoot and the splice is empty. Tests: Task 8 `Planner.ShortRouteEndsAtGoal`, Task 10 `MediumReference.SpliceWithNoRemainder`.
4. **No medium node, or one that stops mid-run:** B5 falls back (`no_plan`, then `stale`) every cycle, counts it, and behaves as frozen B5. Test: Task 10 `MediumMode.StalePlanFallsBackToFrozen` (outputs equal to a nav2-mode controller within 1e-9).
5. **A passage blocked for every seed (a draggable dropped into the gate):** the planner publishes status `infeasible` with the lowest-slack solution, keeps planning, and B5's rows and blocked detector decide as today. Test: Task 8 `Planner.BlockedPassagePublishesInfeasible`.

---

### Task 1: Base friction feedforward

**Files:**
- Modify: `motion-planning-sim/Assets/Scripts/Runtime/Control/SkidSteerKinematics.cs`, `SkidSteerBaseController.cs`; `motion-planning-sim/Assets/Scripts/Editor/DemoCommands.cs` (or the editor command file holding base commands); `tools/run_nav_scenario.py`, `tools/test_run_nav_scenario.py`, `tools/summarize_nav_runs.py`; `docs/roadmap.md` (comparability log); `docs/unity-skid-steer-base-controller.md`
- Create: `docs/experiments/medium-horizon/phase5a.md` (results doc with the problem table, started here)
- Test: `motion-planning-sim/Assets/Tests/EditMode/SkidSteerKinematicsTests.cs`

**Interfaces:**
- Produces: `SkidSteerKinematics.FrictionFeedforwardTarget(float commandRadiansPerSecond, float jointFriction, float damping) -> float`; `SkidSteerBaseController.FrictionFeedforward { get; set; }` (default true); CLI `base_friction_feedforward --enabled true|false`; runner option `--base-friction-feedforward` (BooleanOptionalAction, default True), suffix `-bff`, `summary.json` key `base_friction_feedforward`.

- [ ] **Step 1: Write the failing EditMode tests** `FrictionFeedforwardZeroCommandStaysZero` (0 → 0), `FrictionFeedforwardAddsFrictionOverDamping` (command 1.0, friction 0.08, damping D → 1.0 + 0.08 / D; command -1.0 → -(1.0 + 0.08 / D)), and runner tests `test_base_friction_feedforward_suffix_and_command` (on: suffix `-bff`, `unity base_friction_feedforward --enabled true` sent before the run; `--no-base-friction-feedforward`: no suffix, `--enabled false`).
- [ ] **Step 2: Run them** (`unity test editmode --filter SkidSteerKinematicsTests`, `python3 -m pytest tools/test_run_nav_scenario.py -q`): they fail.
- [ ] **Step 3: Implement** the pure function, the switch in `CommandWheelDrives` (offset in rad/s before the degree conversion), the CLI command (returns `{enabled}`), the runner option, the summary column.
- [ ] **Step 4: Run the tests:** pass; full EditMode suite and `python3 -m pytest tools -q` green.
- [ ] **Step 5: Measure** with Play on the open fixture: `ros2 run mobile_manipulator_control base_step_test.py --breakaway-speeds 0.01 0.02 0.03 0.0325 0.035 0.0375 0.04 0.05 --breakaway-yaw-rates 0.02 0.05 0.06 0.07 0.08 0.1 --output-dir docs/experiments/base-controller --prefix lowspeed-bff-<n>` (two runs, on), plus the step-response checks of `docs/unity-skid-steer-base-controller.md` (tracking ratio at 0.3 m/s and 0.4 rad/s must not get worse). Then the B5 smoke: `v2_open_yard_nav`, `v2_door_105_nav`, `v2_chicane_nav`, one run each with the feedforward on. Stop Play.
- [ ] **Step 6: Record:** breakaway table and smoke results in `phase5a.md`; the doc paragraph in `unity-skid-steer-base-controller.md`; a comparability-log row ("Unity base friction feedforward, on by default; changes every baseline; envelope breakaway unchanged in 5A").
- [ ] **Step 7: Commit** `Add base friction feedforward to the Unity skid-steer, on by default`. **CP:** report the new breakaway values and the smoke runs; the user rules whether the envelope's breakaway values change later (that would regenerate the golden fixture in its own commit).

### Task 2: Static torque, fold pose and torque limits

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/statics.hpp`, `src/statics.cpp`, `test/test_statics.cpp`
- Modify: `ros2_ws/src/mobile_manipulator_mpc/CMakeLists.txt` (add to `mpc`, add test); `src/carry_pose_search.cpp` (use `static_torque`); `ros2_ws/src/mobile_manipulator_control/config/qualified_payload.json` (`poses_rad.fold`); `tools/run_wholebody_arm_qualification.py`, `tools/test_run_wholebody_arm_qualification.py` (`fold` suite); `docs/experiments/wholebody-mpc/arm-qualification.json.gz` (suite results); `phase5a.md`

**Interfaces:**
- Produces: `Eigen::Matrix<double, 6, 1> static_torque(const State & x)` (N m, model joint order, gravity of the arm links below each joint plus the payload); `Eigen::Matrix<double, 6, 6> static_torque_jacobian(const State & x)` (d tau / d q, central differences, step 1e-6); `poses_rad.fold = [0, 1.236, -2.156, 0, 0.920, 0]`.

- [ ] **Step 1: Write the failing tests** `Statics.HomeHasNoPanTorque` (|tau(0)| < 1e-9 at every pose), `Statics.MatchesCarryPoseSearch` (carry: shoulder, elbow, wrist_2 equal to `carry_pose_search`'s values within 1e-6), `Statics.FoldShoulderAbout22Nm` (|tau(1)| in [20, 24]), `Statics.JacobianMatchesFiniteDifferences` (random poses, 1e-4), and `test_fold_suite_matches_vertical_right_shape` in the tool test.
- [ ] **Step 2: Run them:** they fail (no `statics.hpp`, no suite).
- [ ] **Step 3: Implement** `statics.cpp` from the lambda in `carry_pose_search.cpp` (which then calls it, same printed output); the `fold` suite (held still and while driving, transitions with `carry` at the `wholebody_arm` limits still and driving, JTC streaming at 50 Hz around it; the acceptance of `carry`).
- [ ] **Step 4: Run the tests:** pass; `test_golden` unchanged.
- [ ] **Step 5: Qualify in Unity** (Play, arm control active, no Nav2, open test area): `python3 tools/run_wholebody_arm_qualification.py --suite fold --prefix fold`, 3 runs per row. Stop Play. Add `fold` to `qualified_payload.json` only if every row passes.
- [ ] **Step 6: Print the torque table** (|tau| per joint for every qualified pose) and the default limits by the plan-level rule into `phase5a.md`.
- [ ] **Step 7: Commit** (two commits: `Add static joint torque to the MPC library` and `Qualify the fold pose and register it`). **CP:** qualification results and the torque-limit table for a user ruling; if the fold fails, the user rules (Section 11 last row).

### Task 3: The sequencing scenario `v2_door_then_beam_nav`

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_navigation/config/scenarios_site_v2.yaml`; its scenario tests (`ros2_ws/src/mobile_manipulator_navigation/test/`, the file that checks V2 scenario names and counts); `docs/roadmap.md` (comparability row: V2 scenario set 20)

**Interfaces:**
- Produces: scenario `v2_door_then_beam_nav`: start `[-6.0, -8.9, 0.0]`, goal `[3.0, -8.9, 0.0]`, `arm_pose: carry`, `footprint_profile: carry`, obstacles `gate_post_left {x -1.0, y -8.275, 0.2 × 0.2 × 2.0}`, `gate_post_right {x -1.0, y -9.525, 0.2 × 0.2 × 2.0}`, `low_beam {x 0.3, y -8.9, size_x 0.2, size_y 3.0, height 0.2, z 1.23}` (beam 1.3 m behind the gate; spacing revisited in Task 12), `timeout_s: 150`, description stating the expected B5 behaviour (vertical carry through the gate, then a stall or contact risk at the beam).

- [ ] **Step 1: Write the failing test** `test_v2_door_then_beam_scenario` (present, site V2, obstacles as above, 20 scenarios in the set).
- [ ] **Step 2: Run it:** fails.
- [ ] **Step 3: Add the scenario** and the comparability row.
- [ ] **Step 4: Run** the navigation package tests and `python3 tools/check_site_plans.py` (if it checks scenario plans offline): pass.
- [ ] **Step 5: Commit** `Add the V2 door-then-beam sequencing scenario`.

### Task 4: Interfaces and B5's route announcement

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_interfaces/msg/{WholeBodyRoute,MediumPlanSample,MediumPlan,MediumCycle}.msg`; `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/ros_state.hpp`, `src/ros_state.cpp`
- Modify: `mobile_manipulator_interfaces/CMakeLists.txt`; `mobile_manipulator_mpc/CMakeLists.txt` (library `ros_state`, exported with `mpc`; `kinematic_plant.hpp` installed); `src/wholebody_mpc_node.cpp` (use `ros_state`; publish the route); `test/closed_loop.hpp` (include the moved plant); `test/test_wholebody_mpc_node.py`; `tools/run_nav_scenario.py` (`WHOLEBODY_BAG_TOPICS` += `/wholebody_mpc/route`)
- Create: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/kinematic_plant.hpp` (moved `Plant`)

**Interfaces:**
- Produces messages:
  - `WholeBodyRoute`: `std_msgs/Header header`, `uint64 route_id`, `nav_msgs/Path path`, `geometry_msgs/Pose2D goal`.
  - `MediumPlanSample`: `float64 s, x, y, yaw, speed`, `float64[6] q`.
  - `MediumPlan`: `std_msgs/Header header`, `uint64 route_id, plan_id`, `string status` (`ok`, `infeasible`, `stale_reused`), `string seed, homotopy`, `float64 cost, solve_s, splice_s`, `MediumPlanSample[] samples`.
  - `MediumCycle`: `std_msgs/Header header`, `uint64 plan_id`, `float64 total_s, linearize_s, qp_s`, `bool full_multistart, capped`, `string[] seeds`, `float64[] seed_costs, seed_slack, seed_solve_s`, `int32[] seed_iterations`, `bool[] seed_converged`, `string homotopy`, `bool homotopy_switched`.
- Produces: topic `/wholebody_mpc/route` (transient local, depth 1), `route_id` +1 on every `start` and `update_goal`; `MeasuredState ros_state::measure(...)` with the node's current fields (moved, not rewritten).

- [ ] **Step 1: Write the failing launch-test assertions** in `test_wholebody_mpc_node.py`: after a FollowPath goal a `WholeBodyRoute` arrives with `route_id` 1 and the goal's path; a second goal gives `route_id` 2.
- [ ] **Step 2: Run it:** fails.
- [ ] **Step 3: Implement** the messages, the `ros_state` extraction (behaviour unchanged), the plant move, the route publisher, the bag topic.
- [ ] **Step 4: Run** `mobile_manipulator_interfaces mobile_manipulator_mpc` tests and the tools tests: green, `test_golden` unchanged.
- [ ] **Step 5: Commit** `Add the medium-layer messages and announce B5's route`.

### Task 5: Package scaffold and the Bézier basis

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_medium/{package.xml,CMakeLists.txt}`, `include/mobile_manipulator_medium/bezier.hpp`, `src/bezier.cpp`, `test/test_bezier.cpp`

**Interfaces:**
- Produces (namespace `mobile_manipulator_medium`):
  - `struct BezierSpec { int degree = 3; int segments = 4; double horizon_s = 8.0; int continuity = 2; };`
  - `class PiecewiseBezier` — `explicit PiecewiseBezier(BezierSpec)`; `int control_points() const`; `int free_parameters() const` (after continuity and the two start equalities); `Eigen::RowVectorXd basis(double t, int derivative) const` over all control points; `Eigen::MatrixXd map() const` and `Eigen::VectorXd offset(double p0, double v0) const` (c = M p + c0); `Eigen::MatrixXd derivative_control_points(int derivative) const` (linear map control points → derivative curve's control points, for hull bounds); `Eigen::MatrixXd acceleration_gram() const` (∫ (c'' )² dt as cᵀ G c).
  - The basis is written over a knot vector so a B-spline basis (F4) can be added as another class with the same interface.

- [ ] **Step 1: Write the failing tests** `Bezier.C2AtJoins` (value, first and second derivative continuous at each join for random p, 1e-9), `Bezier.StartStateHeld` (c(0) = p0, c'(0) = v0 for random p), `Bezier.DerivativesMatchFiniteDifferences` (1e-6), `Bezier.HullBoundsContainSamples` (1000 samples of value and derivatives inside the min/max of their control points), `Bezier.FreeParameterCount` (cubic, 4 segments, C2 → 5), `Bezier.GramMatchesQuadrature` (1e-8).
- [ ] **Step 2: Run them:** fail (package does not exist).
- [ ] **Step 3: Implement** the package (depends on `mobile_manipulator_mpc`, `hpipm_vendor`, `osqp_vendor`, Eigen) and `bezier.cpp`.
- [ ] **Step 4: Run** `colcon build --packages-select mobile_manipulator_medium` and its tests: pass.
- [ ] **Step 5: Commit** `Add the medium-layer package and its piecewise Bézier basis`.

### Task 6: Dense-QP interface and three backends

**Files:**
- Create: `ros2_ws/src/daqp_vendor/{package.xml,CMakeLists.txt,README.md,test/test_daqp_smoke.cpp}` (pinned release tarball with SHA256, built static and position independent like `hpipm_vendor`); `mobile_manipulator_medium/include/mobile_manipulator_medium/dense_qp.hpp`, `src/dense_qp_hpipm.cpp`, `src/dense_qp_osqp.cpp`, `src/dense_qp_daqp.cpp`, `src/dense_qp_factory.cpp`, `test/test_dense_qp.cpp`

**Interfaces:**
- Produces:
  - `struct DenseQp { Eigen::MatrixXd H; Eigen::VectorXd g; Eigen::MatrixXd A; Eigen::VectorXd lA, uA, lb, ub; };` (min ½xᵀHx + gᵀx, lA ≤ Ax ≤ uA, lb ≤ x ≤ ub; ±inf allowed).
  - `struct DenseQpResult { bool ok; std::string status; Eigen::VectorXd x; int iterations; double solve_s; };`
  - `class DenseQpSolver { public: virtual ~DenseQpSolver() = default; virtual DenseQpResult solve(const DenseQp &, const Eigen::VectorXd * warm) = 0; };`
  - `std::unique_ptr<DenseQpSolver> make_dense_solver(const std::string & backend)` for `"hpipm"`, `"osqp"`, `"daqp"`; `std::invalid_argument` otherwise.
- Decision: DAQP is the latest release tag at implementation time; record the tag and SHA256 in the vendor README.

- [ ] **Step 1: Write the failing tests** `DenseQp.BackendsAgreeOnRandomQps` (100 random strictly convex QPs with 35 variables and 300 rows, mixed active and inactive, the three solutions within 1e-5 of each other and of a reference KKT check), `DenseQp.InfiniteBoundsAreFree`, `DenseQp.InfeasibleReportsNotOk`, `DenseQp.WarmStartAccepted` (same answer with and without warm start), `DenseQp.UnknownBackendThrows`; the vendor smoke test (a 2-variable QP to its known answer).
- [ ] **Step 2: Run them:** fail.
- [ ] **Step 3: Implement** the vendor package and the three backends (HPIPM `d_dense_qp` IPM in `balance` mode; OSQP via CSC conversion, eps 1e-6, polish on; DAQP with hot start).
- [ ] **Step 4: Run** the new tests on linux/arm64, and one amd64 compile check of `daqp_vendor`: pass.
- [ ] **Step 5: Commit** (two commits: `Add daqp_vendor` and `Add the dense-QP interface with HPIPM, OSQP and DAQP backends`).

### Task 7: The medium problem (cost, constraints, linearization)

**Files:**
- Create: `mobile_manipulator_medium/include/mobile_manipulator_medium/problem.hpp`, `src/problem.cpp`, `src/medium_config.cpp`, `config/medium.yaml`, `test/test_problem.cpp`

**Interfaces:**
- Consumes: Task 5 `PiecewiseBezier`; Task 6 `DenseQp`; `mpc`: `State`, `link_index`, `point_jacobian`, `CollisionModel::rows`/`nearest`, `World`, `Route`, `Limits`, `load_limits`; Task 2 `static_torque`, `static_torque_jacobian`.
- Produces:
  - `struct MediumConfig` (loaded by `MediumConfig load_medium_config(const std::string & yaml)`; every key below with these defaults): `rate_hz 5`, `bezier {degree 3, segments 4, horizon_s 8.0}`, `arm_joints [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_2_joint]`, `collision_dt 0.1`, `margins {base 0.12, arm 0.05}`, `weights {progress, contour, clearance, comfort, smooth_base, smooth_arm, skid}` (start values in `medium.yaml`, tuned in Task 12), `clearance_target_m 0.25`, `scp {max_iter 10, trust_init, trust_min, step_tol, merit_tol}`, `slack_feasible_m 0.001`, `homotopy_hysteresis 0.15`, `solve_cap_s 0.15`, `full_multistart_period_s 2.0`, `sample_ds 0.1`, `sample_dyaw 0.1`, `torque_limits_nm` (six values from the Task 2 ruling), `seeds [previous, vertical_carry, vertical_carry_right, fold, level_extension]`, `qp_backend` (`hpipm` until Task 9).
  - `struct Trajectory { Eigen::MatrixXd control_points; /* 7 × n, rows x, y, yaw, then arm_joints */ };` with `State state_at(double t, const Eigen::Matrix<double,6,1> & q_rest) const` (unmasked joints from `q_rest`).
  - `class MediumProblem` — `MediumProblem(MediumConfig, Limits, CollisionModel, Eigen::Matrix<double,6,1> q_comfort)`; `DenseQp linearize(const Trajectory & nominal, const Route & route, double goal_s, const World & world, double now, double trust) const`; `double merit(const Trajectory &, const Route &, double goal_s, const World &, double now) const` (cost + slack penalty, exact collision and torque); `double max_slack(...) const`; `Trajectory apply(const Trajectory &, const Eigen::VectorXd & dp) const`.

- [ ] **Step 1: Write the failing tests** `Problem.CollisionRowsMatchFiniteDifferences` (a box near the panel, rows' gradients in p against central differences, 1e-5), `Problem.TorqueRowsMatchFiniteDifferences`, `Problem.SkidResidualZeroForStraightForwardMotion`, `Problem.ForwardOnlyRowRejectsReversing`, `Problem.HullRowsBoundJointLimits` (a QP step that would cross a joint limit at a non-sample time is cut), `Problem.MeritExactAtNominal` (merit equals the QP's model value at dp = 0 up to the constant), `MediumConfig.DefaultsLoad` and `MediumConfig.UnknownKeyThrows`.
- [ ] **Step 2: Run them:** fail.
- [ ] **Step 3: Implement** the cost terms of Section 3.2 and the constraints of Section 3.3, using the mask and `c = M p + c0`.
- [ ] **Step 4: Run** the package tests: pass.
- [ ] **Step 5: Commit** `Add the medium problem: cost, constraints and their linearization`.

### Task 8: SCP and the planner (seeds, selection, homotopy, cap)

**Files:**
- Create: `mobile_manipulator_medium/include/mobile_manipulator_medium/{scp.hpp,planner.hpp}`, `src/scp.cpp`, `src/planner.cpp`, `test/test_planner.cpp`, `test/scenes.hpp` (synthetic scenes)

**Interfaces:**
- Consumes: Task 7 `MediumProblem`, `MediumConfig`, `Trajectory`; Task 6 `make_dense_solver`.
- Produces:
  - `struct ScpResult { Trajectory traj; double cost, slack; int iterations; bool converged, feasible; double solve_s; };` `ScpResult solve_scp(const MediumProblem &, DenseQpSolver &, Trajectory seed, const Route &, double goal_s, const World &, double now, std::chrono::steady_clock::time_point deadline);`
  - `struct PlanSample { double s, x, y, yaw, speed; Eigen::Matrix<double,6,1> q; };`
  - `struct PlanOutput { std::string status, seed, homotopy; double cost, solve_s, splice_s; std::vector<PlanSample> samples; /* + per-seed stats for MediumCycle */ };`
  - `class MediumPlanner` — `MediumPlanner(MediumConfig, MediumProblem, std::unique_ptr<DenseQpSolver>, std::map<std::string, Eigen::Matrix<double,6,1>> seed_poses)`; `void set_route(const Route &, const mobile_manipulator_navigation::Pose2 & goal, uint64_t route_id)`; `void request_full_multistart()`; `const PlanOutput & plan(double now, const State & measured, const Eigen::Matrix<double,6,1> & qd_measured, const World & world)`.
  - `std::string homotopy_label(const Trajectory &, ...)`: `left` / `right` when the pan excursion exceeds ±0.5 rad, `lowered` when the panel top drops more than 0.03 m below `carry`'s, joined with `+`, else `comfort`.
- Decisions: a pose seed ramps the masked joints from the measured joints to the seed pose over the first half of the horizon and puts the base on the route at 0.8 × `v_max`; seeds are merged when their control points differ by < 0.02 (m / rad) max-norm; the incumbent is solved first; hysteresis compares the new best against the re-optimized incumbent.

- [ ] **Step 1: Write the failing tests** on synthetic scenes:
  - `Planner.AngledDoorEntersAligned`: a 1.05 m door right after a 90° route corner (the R4 geometry); the plan's base yaw at the door plane is within 5° of the door axis and its clearance ≥ the margins.
  - `Planner.LowBeamLowersWithinTorque`: a 1.23 m beam on a straight route from `carry`; the homotopy contains `lowered`, panel clearance ≥ 0.05 m, |tau| within limits at every sample.
  - `Planner.SymmetricGateKeepsSide`: a symmetric 1.05 m gate with a 0.1 m lateral bias; side follows the bias and does not switch over 30 warm cycles advanced along the plan.
  - `Planner.SeedsMerge`: an open route; all pose seeds converge to `comfort` and are merged to one.
  - `Planner.ShortRouteEndsAtGoal`: 1.5 m of route left; the last sample is at the goal (≤ 0.02 m) and speeds are non-increasing over the last 0.5 m.
  - `Planner.TurnInPlaceKeepsYawSamples`: a goal yaw 90° from the arrival heading; samples at the goal span the yaw change in ≤ 0.1 rad steps.
  - `Planner.BlockedPassagePublishesInfeasible`: a box filling the gate; status `infeasible`, slack > 1 mm, a plan is still returned and the next cycle runs.
  - `Planner.CapKeepsLastValidPlan`: `solve_cap_s` 1e-4; status `stale_reused`, the previous samples.
- [ ] **Step 2: Run them:** fail.
- [ ] **Step 3: Implement** `solve_scp` (trust region, merit acceptance) and `MediumPlanner` (seeds, merge, feasibility, selection, homotopy, hysteresis, cap, arc-length resampling by the plan-level rule).
- [ ] **Step 4: Run** the package tests: pass; note per-test wall time (a first timing signal for Task 9).
- [ ] **Step 5: Commit** `Add the medium SCP and planner with multi-start seeds and homotopy hysteresis`.

### Task 9: Open-loop bench and solver choice

**Files:**
- Create: `mobile_manipulator_medium/src/medium_bench.cpp` (executable `medium_bench`), `tools/medium_bench.py`, `tools/test_medium_bench.py`, `docs/experiments/medium-horizon/phase5a/corpus.json`

**Interfaces:**
- Consumes: Task 8 `MediumPlanner`; `mpc` `load_scene`, `replay_bag` readers for `/odom`, `/joint_states`, `/tf`, `/plan`.
- Produces: `medium_bench open --bag DIR --scenario NAME --backend B [--segments N --horizon-s T --collision-dt D] --out FILE.json` (per-snapshot cycle records: times split, iterations, converged, seed, homotopy, slack, clearance, torque margin); `tools/medium_bench.py sweep` (corpus × backends × parameter grid → summary tables: mean, median, p90, p95, p99, max, convergence rate).
- Corpus: campaign 2 bags of the seven campaign-A cells where they exist, snapshots every 0.5 s; `v2_door_then_beam_nav` from its scenario start and Nav2 route (no bag).

- [ ] **Step 1: Write the failing tests** `test_corpus_paths_relative_and_present`, `test_summary_statistics` (known records → known p99), `test_choose_backend_rule` (lowest p99 among backends whose convergence rate is within 1 % of the best).
- [ ] **Step 2: Run them:** fail.
- [ ] **Step 3: Implement** the bench and the driver.
- [ ] **Step 4: Run** the sweep: backends × `segments` {4, 6, 8} × `horizon_s` {6, 8, 10} × `collision_dt` {0.1, 0.15}, on the host container with nothing else running. Write the tables into `phase5a.md`; set `qp_backend` and the representation defaults in `medium.yaml` by the rule (cycle p99 ≤ 150 ms required).
- [ ] **Step 5: Commit** `Add the open-loop medium bench and choose the QP backend`. **CP:** solver comparison and chosen parameters; if nothing meets 150 ms, report with the split (linearization vs QP) and the risk table's options.

### Task 10: B5 `reference_source: medium`

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/medium_reference.hpp`, `src/medium_reference.cpp`, `test/test_medium_reference.cpp`
- Modify: `formulation.hpp/.cpp` (`Inputs::medium`), `controller.hpp/.cpp` (`replace_route`, medium inputs, picker and tightness off), `wholebody_mpc.yaml` + `load_mpc_config` (`reference_source`, `medium_stale_s 0.5`, `medium_max_offset_m 0.3`, `weights.medium_posture`), `wholebody_mpc_node.cpp` (subscribe `/medium/plan`, decide per cycle, fill `MpcCycle`), `mobile_manipulator_interfaces/msg/MpcCycle.msg` (`string reference_source`, `float64 medium_plan_age_s`, `uint64 medium_plan_id, medium_route_id`, `string medium_fallback`), `test/test_controller.cpp`, `test/test_mpc_config.py`

**Interfaces:**
- Consumes: Task 4 messages (converted in the node to the ROS-free struct below).
- Produces:
  - `struct MediumSample { double s, x, y, yaw, speed; Eigen::Matrix<double,6,1> q; };`
  - `class MediumReference` — `MediumReference(std::vector<MediumSample>, const Route & nav2_route, double splice_s)`; `const Route & route() const` (samples + Nav2 remainder from `splice_s`); `double goal_s() const`; `double yaw_at(double s) const`, `double speed_at(double s) const`, `Eigen::Matrix<double,6,1> posture_at(double s) const` (beyond the samples: the route tangent, B5's progress speed, the comfort pose); `double start_hint(const Eigen::Vector2d & base) const` (arc length of the nearest sample within the first 1 m).
  - `enum class MediumFallback { None, NoPlan, Stale, RouteId, Offset };` `MediumFallback medium_fallback(double plan_age_s, uint64_t plan_route, uint64_t active_route, double offset_m, const MpcConfig &)`; `const char * to_string(MediumFallback)` (`""`, `no_plan`, `stale`, `route_id`, `offset`).
  - `Inputs::medium` (`const MediumReference *`, null = nav2 behaviour); `Controller::replace_route(const Route &, double goal_s, double s0_hint)`; `Controller::set_medium(const MediumReference *)`.

- [ ] **Step 1: Write the failing tests:**
  - `MediumReference.RouteSplicesNav2Remainder`, `MediumReference.SpliceWithNoRemainder`, `MediumReference.LookupsByArcLength` (yaw, speed, q interpolated between samples, last-sample rule at equal `s`), `MediumReference.InPlaceTurnKeepsGoalHeading` (B5's goal-heading logic unchanged near the goal).
  - `MediumFallback.EachReason` (one case per enum value, thresholds 0.5 s and 0.3 m).
  - `MediumMode.FollowsMediumYawSpeedAndPosture` (closed loop with the kinematic plant: B5's heading, speed and joints move towards the reference within 2 s).
  - `MediumMode.PickerAndTightnessOff` (posture_choice `none`, tightness 0 in medium mode).
  - `MediumMode.PlanForOldRouteFallsBack` (reason `route_id` until a matching plan).
  - `MediumMode.StalePlanFallsBackToFrozen` (outputs equal to a nav2-mode controller within 1e-9 while stale).
  - `test_mpc_config.py::test_reference_source_defaults_to_nav2`.
- [ ] **Step 2: Run them:** fail.
- [ ] **Step 3: Implement** the reference, the formulation inputs (heading target, `progress_speed` from `speed_at` capped at B5's limits, posture target from `posture_at` with `weights.medium_posture`), the controller methods, the node logic (new plan → `MediumReference` → `replace_route` with `start_hint`; per cycle `medium_fallback` → frozen inputs on any reason, swapping back to the Nav2 route with `replace_route` (hint: the Nav2 projection of the measured base) only when the source changes; counts in the stats file), the `MpcCycle` fields.
- [ ] **Step 4: Run** the `mobile_manipulator_mpc` tests: all pass, `test_golden` unchanged.
- [ ] **Step 5: Commit** `Let B5 follow medium-layer references, with a per-cycle fallback to the frozen behaviour`.

### Task 11: `medium_horizon_node`, launch, runner and RViz

**Files:**
- Create: `mobile_manipulator_medium/src/medium_horizon_node.cpp`, `test/test_medium_horizon_node.py` (launch test)
- Modify: `mobile_manipulator_medium/CMakeLists.txt`; `ros2_ws/src/mobile_manipulator_navigation/launch/navigation.launch.py` (`medium:=none|bezier`, `medium_config_file`; with `bezier` start the node and set B5's `reference_source: medium`); `tools/run_nav_scenario.py` (`--medium none|bezier`, suffix `-f2`, bag topics `/medium/plan`, `/medium/cycle`, `summary.json` `medium`), `tools/test_run_nav_scenario.py`, `tools/demo.py` (`drive --medium bezier`), RViz config used by the demo (medium path and panel markers)

**Interfaces:**
- Consumes: Task 4 `ros_state`, messages; Task 8 `MediumPlanner`; `mpc` `load_scene`, `World::update_box`.
- Produces: node `medium_horizon_node` (params `config_file`, `scenario`, `site`, `boxes_file`, `draggables_topic`, `rate_hz`, and `movers_topic` declared but unused in 5A, spec 3.6); topics `/medium/plan`, `/medium/cycle`, `/medium/markers`; full multi-start on a new `route_id`, every `full_multistart_period_s`, and when a draggable moves more than 0.05 m or 0.05 rad.

- [ ] **Step 1: Write the failing tests:** launch test `test_plans_on_announced_route` (publish a `WholeBodyRoute` and states, a `MediumPlan` with that `route_id` arrives within 1 s), `test_new_route_id_triggers_full_multistart`, `test_draggable_move_triggers_full_multistart`; runner tests `test_medium_suffix_launch_and_topics`; launch test with `medium:=none` (no medium node, B5 `reference_source` `nav2`).
- [ ] **Step 2: Run them:** fail.
- [ ] **Step 3: Implement** the node (single-threaded timer; plan in the callback; the cap keeps it below the period), launch, runner, demo, markers.
- [ ] **Step 4: Run** the medium, mpc and navigation tests and `python3 -m pytest tools -q`: green.
- [ ] **Step 5: Commit** `Add the medium-horizon node with launch, runner and RViz support`.

### Task 12: Closed-loop kinematic bench, tuning and gate

**Files:**
- Modify: `mobile_manipulator_medium/src/medium_bench.cpp` (mode `closed`), `tools/medium_bench.py` (`closed` driver and gate check), `tools/test_medium_bench.py`; `config/medium.yaml` and `wholebody_mpc.yaml` (`weights.medium_posture`) with tuned values; possibly `scenarios_site_v2.yaml` (beam spacing, with its test)

**Interfaces:**
- Consumes: Task 10 `Controller` in medium mode, `kinematic_plant.hpp`; Task 8 planner; Task 9 bench I/O.
- Produces: `medium_bench closed --scenario NAME --out FILE.json` (B5 at 50 Hz on the plant, the medium planner at 5 Hz, plan latency by the plan-level rule; records outcome, collision-slack plans executed, fallbacks, homotopy switches, medium timing); `tools/medium_bench.py gate` (the gate of Global Constraints over the seven cells, exit code 0 only if every criterion holds).

- [ ] **Step 1: Write the failing tests** `test_gate_rule` (records violating each criterion fail), and gtest `ClosedLoop.AngledDoorReachesGoal` and `ClosedLoop.LowBeamReachesGoal` (the two scripted integration cells of spec Section 10).
- [ ] **Step 2: Run them:** fail.
- [ ] **Step 3: Implement** the closed mode.
- [ ] **Step 4: Tune** `weights.*`, `clearance_target_m` and `weights.medium_posture` on the seven cells; check the door-then-beam spacing is feasible (if not at 1.3 m, the smallest spacing that is, with the scenario test updated). Record every change and its reason in `phase5a.md`.
- [ ] **Step 5: Run** `python3 tools/medium_bench.py gate`: exit 0.
- [ ] **Step 6: Commit** `Add the closed-loop medium bench and tune F2 on it` (scenario spacing change, if any, as its own commit). **CP:** gate results per cell and the tuned values.

### Task 13: Campaign A (acceptance)

**Files:**
- Create: `docs/experiments/medium-horizon/phase5a/campaign_a.sh`; results section in `phase5a.md`

- [ ] **Step 1: Write the script:** seven cells (Section 7.1) × `--medium bezier` and `--medium none` × 3 runs, `--controller wholebody --mpc-rate-hz 50`, friction feedforward on, under `caffeinate -i`, cells in a fixed interleaved order.
- [ ] **Step 2: Run it** (Docker and Unity up, thermals nominal; about 1.75 h; stop Play at the end).
- [ ] **Step 3: Score** with `tools/summarize_nav_runs.py` plus the medium columns (medium p99, fallback share, homotopy switches, torque peak), and check each criterion of Section 7.1.
- [ ] **Step 4: Commit** `Record Phase 5A campaign A`. **CP:** per-criterion results; any failure with its cause for a user ruling.

### Task 14: Campaign B (no regression)

**Files:**
- Create: `docs/experiments/medium-horizon/phase5a/campaign_b.sh`; results section in `phase5a.md`

- [ ] **Step 1: Write the script:** the campaign 2 exit set (the 34 runs of `docs/experiments/wholebody-mpc/phase4b-b.md`), `--medium bezier`.
- [ ] **Step 2: Run it** (about 3 h; stop Play at the end).
- [ ] **Step 3: Score** against campaign 2 (≥ 30/34, no new failure mode, contacts ≤ campaign 2's), stating the base change; worker-cell regressions are recorded for 5C.
- [ ] **Step 4: Commit** `Record Phase 5A campaign B`. **CP:** results and rulings.

### Task 15: Documentation and freeze

**Files:**
- Create: `docs/adr/0015-medium-horizon-layer.md`
- Modify: `docs/experiments/medium-horizon/phase5a.md` (final); `docs/roadmap.md` (Phase 5A outcome under Phase 5; decision-log entries: formulation 1b, F4 optional, friction feedforward timing, QP backend; the F4 open decision resolved as optional; the baseline matrix F2 row; section 12 step 7 status); `docs/running-the-stack.md` (`medium:=bezier`, `--medium`)

- [ ] **Step 1: Write** ADR 0015 (context, decision: interface, ownership, arc-length references, fallback, formulation 1b and the backend; consequences; revisit conditions: 5B/5C), the roadmap entries and the run instructions.
- [ ] **Step 2: Verify** links resolve (`grep -o '](.*)' ` on the changed docs, every target exists) and the full test suites are green one last time.
- [ ] **Step 3: Commit** `Document Phase 5A and freeze F2`; create the local annotated tag `f2-frozen-<date>` (ask before pushing).
