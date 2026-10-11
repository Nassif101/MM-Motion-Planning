# Phase 4 Whole-Body MPC (B5) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add baseline B5, a local whole-body MPC that drives the base and moves the arm and panel together as the `FollowPath` server under Nav2's route, with free drive from RViz, new interaction scenarios, an overhead-obstacle course, and the timing instrumentation Phase 4B builds on.

**Architecture:** A new C++ package `mobile_manipulator_mpc` holds a CasADi-generated kinematic model, an FCL-based distance library over known boxes and predicted movers, a stage-wise LTV QP formulation with an OSQP backend, a controller loop (fallback, braking, breakaway output) and `wholebody_mpc_node`, which serves `FollowPath` in its own process. `navigation.launch.py controller:=wholebody` replaces `controller_server` and `velocity_smoother` with it; `dynamic_footprint_node` gains the B4 grow-only rule and a `wholebody` mode whose slowdown zone is the hull swept by the MPC's predicted configurations.

**Tech Stack:** ROS 2 Jazzy, Nav2 1.3.13 (Smac Lattice, collision monitor, BT navigator), ros2_control JTC, C++17 + gtest, Eigen, FCL 0.7, OSQP (`osqp_vendor`), CasADi (Python, build-time code generation), pytest/launch_testing, Python tooling in `tools/`, Unity 6 Editor via Unity CLI.

**Spec:** `docs/superpowers/specs/2026-10-11-phase4-wholebody-mpc-design.md`

## Global Constraints

- Branch `wholebody-mpc`; one commit per task (the measurement tasks may add one results commit); never commit to `main`.
- Runtime nodes and ROS package code in C++; launch files, pytest config checks, build-time code generation and `tools/` in Python.
- B1-B4 behaviour unchanged except the B4 grow-only rule (Task 1); their tests pass unchanged.
- State order (x, y, yaw, shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint); control order (v, omega, the six joint velocities).
- Bring-up grid dt = 0.1 s, N = 20; loop 20 Hz, target 50 Hz.
- Base limits: 0 <= v <= 0.3 m/s, |omega| <= 0.4 rad/s; rate limits 0.45 m/s² accelerating, 0.5 m/s² braking, 0.8 rad/s² yaw accelerating, 1.0 rad/s² yaw braking, applied to the last command sent. Breakaway 0.0375 m/s, 0.07 rad/s (`nav_operating_envelope.yaml`; read from the file, never duplicated).
- Collision margin 0.05 m, activation distance 0.8 m, 8 pairs per stage; raised floor top 0.15 m.
- Goal tolerance 0.15 m and 0.15 rad.
- B5 route costmap and collision monitor observe points from 0.05 m to 0.60 m (base box top 0.32 m, arm mount top 0.50 m); B1-B4 keep 0.05-2.0 m.
- Exactly one publisher upstream of the collision monitor on its `cmd_vel` input (ADR 0006); one owner of footprint and zone topics per mode (ADR 0010).
- No host-absolute paths, credentials, Unity `Library/`, or ROS `build/ install/ log/` in commits; shell scripts LF.
- Long Unity batches under `caffeinate -i` (the runner holds it); stop Play (`unity command editor_stop`) when a task that used it finishes; `pkill -f control_description` before mock tests.
- Do not commit the paper PDFs.

Container commands use `C='docker exec mm-motion-planning-ma-robot-sim-1 bash -ic'`; `cb` is the project build function and `$ROS_WS` the workspace. Package tests: `$C 'cd $ROS_WS && colcon test --packages-select <pkgs> --event-handlers console_direct+ && colcon test-result --verbose'`.

### Plan-level decisions not spelled out in the spec

- **Robot shapes are the URDF primitives themselves**, not capsules: `mobile_manipulator_geometry::load_primitives` already gives every collision box and cylinder plus the payload box, and FCL has exact `Box` and `Cylinder` shapes. This removes the capsule approximation; the panel stays a 1.20 x 1.20 x 0.04 m box. Spec Sections 2 and 4 are amended in Task 6's commit.
- **Known obstacles and panel references are loaded by the MPC node from the scenario** (parameters `scenario`, `references: scripted|none`) through the shared C++ loaders, instead of latched topics published by the runner: the runner then needs no long-lived publisher, and free drive simply has no scenario. Spec Sections 2, 4 and 6 are amended in Task 10's commit.
- **The box loader moves** from `mobile_manipulator_manipulation/scene_diff` to `mobile_manipulator_navigation/known_world` so the MPC does not depend on MoveIt; the manipulation package keeps using it through the navigation package.
- **B5 gets its own behaviour tree** (`navigate_to_pose_wholebody.xml`): there is no local costmap without `controller_server`, so the local-costmap clears are removed and `FollowPath` gets `server_name="wholebody_mpc/follow_path"`.
- **Named arm states** (`home`, `vertical_carry`) resolve to joints through `qualified_payload.json` `poses_rad` and `joint_order`, and to panel poses through `FootprintProjector::panel_pose`.
- **"Base moving"** for the grow-only rule is `/odom` with |v| > 0.01 m/s or |omega| > 0.02 rad/s within the last 0.5 s.
- **Generated code is built, not committed:** CMake runs `scripts/generate_model.py` at build time; CasADi is installed by pip in the Dockerfile and, until the container is rebuilt, in the running container.

## Review Focus

1. **A goal behind the robot from rest** (the RPP stall): B5 must start the in-place turn. Tests in Task 8 (`Formulation.RateLimitRelativeToLastCommand`) and Task 9 (`Controller.TurnsInPlaceFromRestToGoalBehind`, `BreakawayOutput.LiftsSubBreakawayCommandAtRest`).
2. **Arm joint states missing, partial or stale while driving:** the MPC must brake and hold the arm, never command from zeros. Tests in Task 10 (`test_stale_joint_states_brake_and_fail`) and Task 9 (`Controller.BrakesWhenStateIsStale`).
3. **QP infeasible next to an obstacle** (robot already inside the margin after a disturbance): slack keeps the QP solvable and the plan moves away; repeated failure brakes. Tests in Task 8 (`Formulation.SlackKeepsQpFeasibleInsideMargin`) and Task 9 (`Controller.BrakesAfterKFailedCycles`).
4. **A new goal or cancel while the arm is mid-reorientation:** the action must preempt cleanly and the arm must hold where it is (a final streamed point with zero velocity), not snap back. Test in Task 10 (`test_cancel_holds_arm_in_place`).
5. **A mover that appears, jumps or disappears** (`/scenario/movers` gaps, teleports between runs): velocity estimation must not produce huge predictions. Test in Task 6 (`World.MoverJumpResetsVelocity`, `World.StaleMoverDropped`).

---

### Task 1: B4 grow-only footprint while the base moves, and the B4 rerun

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_navigation/include/mobile_manipulator_navigation/dynamic_footprint.hpp`, `src/dynamic_footprint.cpp`, `src/dynamic_footprint_node.cpp`
- Test: `ros2_ws/src/mobile_manipulator_navigation/test/test_dynamic_footprint.cpp`
- Modify: `docs/experiments/dynamic-footprint/README.md`

**Interfaces:**
- Produces: `void DynamicFootprint::base_motion(double receive_time, double linear_mps, double yaw_radps)`; `bool DynamicFootprint::moving(double now) const`; `tick(now)` applies the grow-only rule. Constants `kMovingLinearMps = 0.01`, `kMovingYawRadps = 0.02`, `kMotionHoldS = 0.5`.

- [ ] **Step 1: Write the failing tests** in `test_dynamic_footprint.cpp`:
  - `DynamicFootprint.GrowOnlyIgnoresSwayWhileMoving`: publish `home`, report motion, then alternate `wrist_1_joint` by ±0.019 rad for 40 ticks; expect no update after the first.
  - `DynamicFootprint.GrowsWhileMovingWhenHullExtends`: while moving, change to a pose whose hull leaves the current hull grown by 0.01 m; expect one update whose polygon contains both hulls' points.
  - `DynamicFootprint.ShrinksAtStandstill`: after the motion hold expires (0.5 s without motion), a smaller hull is published.
  - `DynamicFootprint.NeverSmallerThanArmWhileMoving`: for every sway sample, the published polygon contains the unpadded hull of that sample.
- [ ] **Step 2: Run** `$C 'cd $ROS_WS && colcon build --packages-select mobile_manipulator_navigation && colcon test --packages-select mobile_manipulator_navigation --ctest-args -R test_dynamic_footprint'`. Expected: the four tests fail.
- [ ] **Step 3: Implement** the rule: while `moving(now)`, publish only when the new padded hull is not contained in `offset_outward(current, change_threshold_m)`, and then publish the hull of the union of current and new; at standstill keep the existing Hausdorff rule. The node feeds `base_motion` from its existing `/odom` subscription.
- [ ] **Step 4: Run the package tests.** Expected: all pass, including the existing B4 tests.
- [ ] **Step 5: Commit** "Grow the dynamic footprint only while the base moves".
- [ ] **Step 6: Rerun B4** in Unity: the 9-run B4 regression (`tools/run_nav_scenario.py narrow_gate_mission wide_gate_mission constrained_reconfiguration_mission --footprint-mode dynamic --runs 3`) and the 20 dynamic-strategy drives as recorded in `docs/experiments/dynamic-footprint/README.md`. Expected: publishes per drive in `home` drop from 7-8 to at most 1; record results against the 2026-10-04 rows; stop Play.
- [ ] **Step 7: Commit** the README update "Record the B4 rerun with the grow-only footprint".

---

### Task 2: Arm qualification for B5: CPU headroom, combined motion, JTC streaming

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_control/src/arm_experiment.cpp`
- Create: `docs/experiments/wholebody-mpc/README.md` (sections "CPU headroom", "Combined arm and base motion", "JTC streaming")
- Modify: `ros2_ws/src/mobile_manipulator_navigation/config/nav_operating_envelope.yaml` (new `wholebody_arm` block)
- Test: `ros2_ws/src/mobile_manipulator_navigation/test/test_operating_envelope.py`

**Interfaces:**
- Produces: `arm_experiment` flags `--cpu-load-threads N` (busy threads pinned to no core, for the run's duration), `--drive-during-motion` (runs the existing bounded base sequence while the trajectory executes, not only during the hold), `--stream-rate HZ --stream-duration S` (sends overlapping 1 s trajectory segments of a sinusoid on `/arm_controller/joint_trajectory` at HZ, each starting at the current time). The JSON report adds `max_path_error_rad` per joint, `position_jumps` (count of command discontinuities > 0.01 rad between consecutive controller-state samples) and `cpu_load_threads`.
- Produces: `nav_operating_envelope.yaml` `wholebody_arm: {max_joint_velocity_radps: [6 values], max_joint_accel_radps2: [6 values], evidence: <README section>}` used by Task 8.

- [ ] **Step 1: Extend `test_operating_envelope.py`** with `test_wholebody_arm_limits_within_urdf`: the `wholebody_arm` velocities are positive and at most the URDF velocity limits, accelerations positive. Expected: fails (block missing).
- [ ] **Step 2: Implement the three flags** in `arm_experiment.cpp`; build.
- [ ] **Step 3: CPU headroom.** With Unity in Play and the arm in `home`, run the qualified reference transition with `--cpu-load-threads 0,1,2,4,6` (3 runs each). Record the load at which `max_path_error_rad` exceeds 0.10 rad (two thirds of the 0.15 rad tolerance) as the CPU budget, with `nproc` and Docker CPU limits.
- [ ] **Step 4: Combined motion.** Run the transitions `home -> vertical_carry` and back with `--drive-during-motion` at joint speed scales 1.0, 0.75, 0.5 (3 runs each). Choose the largest scale with `max_path_error_rad` <= 0.10 rad and no watchdog or tilt flag; write its per-joint velocity and acceleration as `wholebody_arm`.
- [ ] **Step 5: JTC streaming.** Run `--stream-rate 20` and `--stream-rate 50` for 20 s each, and once stopping the stream mid-motion. Expected: `position_jumps` = 0 and the arm settles at the last segment's end when the stream stops. If jumps occur, record them and the JTC setting tried (`interpolate_from_desired_state`); if they cannot be removed, stop and report to the user (fallback decision, spec Section 12).
- [ ] **Step 6: Run the envelope test.** Expected: passes. Stop Play.
- [ ] **Step 7: Commit** "Qualify the arm for whole-body motion: CPU headroom, combined motion, JTC streaming".

---

### Task 3: Known world with elevated boxes, shared by MoveIt and the MPC

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_navigation/include/mobile_manipulator_navigation/known_world.hpp`, `src/known_world.cpp` (in the `scenario_tools` library)
- Modify: `ros2_ws/src/mobile_manipulator_manipulation/include/mobile_manipulator_manipulation/scene_diff.hpp`, `src/scene_diff.cpp`, `src/planning_scene_loader.cpp` (use the moved functions)
- Modify: `ros2_ws/src/mobile_manipulator_navigation/src/scenario_spec.cpp` (validate `z`), `config/scenarios.yaml` (header comment)
- Modify: `motion-planning-sim/Assets/Scripts/Editor/ScenarioCommands.cs` (`scenario_obstacle --z`)
- Modify: `tools/run_nav_scenario.py` (pass `z`)
- Test: `ros2_ws/src/mobile_manipulator_navigation/test/test_known_world.cpp`, `test/test_scenario_spec.cpp`

**Interfaces:**
- Produces (namespace `mobile_manipulator_navigation`): `struct Box { std::string name; Eigen::Vector3d center; Eigen::Vector3d size; double yaw = 0.0; };` `Box scenario_box(const Json & obstacle);` (bottom at `obstacle.value("z", 0.0)`, so `center.z = z + height / 2`), `std::vector<Box> load_boxes(const std::string & yaml_path);`, `std::vector<Box> known_boxes(const std::string & boxes_file, const Json * scenario);` (map boxes plus the scenario's `obstacles`; movers excluded).
- `mobile_manipulator_manipulation::Box` becomes an alias of the navigation type.

- [ ] **Step 1: Write the failing tests:** `KnownWorld.ScenarioBoxStandsOnGroundByDefault` (z absent: center.z = height/2), `KnownWorld.ScenarioBoxFloatsAtZ` (z 1.1, height 0.2: center.z = 1.2), `KnownWorld.KnownBoxesIncludesMapAndScenario` (28 map boxes + scenario obstacles), and in `test_scenario_spec.cpp` `ScenarioSpec.ElevatedObstacleSkipsGroundFreeCheckAboveBaseBand` (an obstacle with z >= 0.60 m over the route is accepted; with z < 0.60 m it must lie in free map space as today) and `ScenarioSpec.RejectsNegativeZ`.
- [ ] **Step 2: Run** the navigation tests. Expected: new tests fail.
- [ ] **Step 3: Implement** `known_world`, move the loader out of `scene_diff` (keep `build_scene_diff` and its tests unchanged), add the `z` rule to `obstacle_problems`, document `z` in the `scenarios.yaml` header, add `--z` (default 0, range [0, 2.5]) to `scenario_obstacle` in Unity (box centre at `z + height/2`) and pass it from the runner.
- [ ] **Step 4: Run** navigation and manipulation tests (`pkill -f control_description` first). Expected: all pass, B3 scene tests unchanged.
- [ ] **Step 5: Unity check.** In Play: `unity command scenario_obstacle --name beam --x 12 --y 0 --size_x 0.3 --size_y 2.0 --height 0.2 --z 1.1`; confirm the box floats (screenshot or bounds query) and the lidar returns it at 1.1-1.3 m; clear it; stop Play.
- [ ] **Step 6: Commit** "Share the known world with elevated scenario boxes between MoveIt and later the MPC".

---

### Task 4: `mobile_manipulator_mpc` package and the generated whole-body model

**Files:**
- Modify: `.devcontainer/Dockerfile` (add `casadi` to the pip install line)
- Create: `ros2_ws/src/mobile_manipulator_mpc/{package.xml,CMakeLists.txt}`
- Create: `ros2_ws/src/mobile_manipulator_mpc/scripts/generate_model.py`
- Create: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/model.hpp`, `src/model.cpp`
- Test: `ros2_ws/src/mobile_manipulator_mpc/test/test_model.cpp`

**Interfaces:**
- Produces (namespace `mobile_manipulator_mpc`):
  - `constexpr int kNx = 9, kNu = 8;` `using State = Eigen::Matrix<double, kNx, 1>; using Control = Eigen::Matrix<double, kNu, 1>;`
  - `struct Step { State x_next; Eigen::Matrix<double, kNx, kNx> A; Eigen::Matrix<double, kNx, kNu> B; };` `Step discrete_step(const State & x, const Control & u, double dt);` (RK4)
  - `int link_index(const std::string & link);` (links with collision primitives plus `panel`; throws for unknown)
  - `Eigen::Isometry3d link_pose(int link, const State & x);` (map frame; base pose from x)
  - `struct PointJacobian { Eigen::Vector3d p; Eigen::Matrix<double, 3, kNx> J; };` `PointJacobian point_jacobian(int link, const State & x, const Eigen::Vector3d & p_link);`
  - `struct PanelError { Eigen::Vector4d e; Eigen::Matrix<double, 4, kNx> J; };` `PanelError panel_error(const State & x, const Eigen::Isometry3d & panel_in_base);` with `e = (p - p_ref, 0.5 * trace(I - R_ref^T R))` in `base_footprint`.
  - `double manipulability(const State & x);` (sqrt(det(Jt Jt^T)) of tool0's translational Jacobian over the six joints)
- Generator: `generate_model.py --urdf <path> --payload <qualified_payload.json> --out <dir>` parses the URDF chain `base_footprint -> tool0` with `urdf_parser_py`, attaches the payload link, and writes `wholebody_model.c/.h` with CasADi `CodeGenerator`; CMake runs it with `add_custom_command` from the installed description and control shares.

- [ ] **Step 1: Install CasADi** in the running container (`$C 'python3 -m pip install --break-system-packages casadi'`) and add `casadi` to the Dockerfile pip line.
- [ ] **Step 2: Write the failing tests:** `Model.LinkPoseMatchesGeometryFk` (for 20 random joint states and base poses, `link_pose` of every primitive link equals `FootprintProjector::link_pose` composed with the base pose within 1e-9), `Model.PanelPoseMatchesProjector`, `Model.StepJacobiansMatchFiniteDifferences` (1e-6), `Model.PointJacobianMatchesFiniteDifferences`, `Model.PanelErrorZeroAtReference`, `Model.UnicycleHasNoLateralMotion` (v = 0.3, omega = 0: displacement along heading only), `Model.ManipulabilityPositiveAtHomeAndDropsWhenStretched`.
- [ ] **Step 3: Run** `$C 'cd $ROS_WS && colcon build --packages-select mobile_manipulator_mpc && colcon test --packages-select mobile_manipulator_mpc'`. Expected: build fails (no sources) or tests fail.
- [ ] **Step 4: Implement** the generator, CMake wiring and the thin C++ wrappers over the generated functions.
- [ ] **Step 5: Run the tests.** Expected: all pass on this machine (arm64); note in the commit that amd64 is covered by the same pip wheel.
- [ ] **Step 6: Commit** "Add the MPC package with a CasADi-generated whole-body model".

---

### Task 5: Route and panel references

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/route.hpp`, `src/route.cpp`, `include/.../references.hpp`, `src/references.cpp`
- Test: `ros2_ws/src/mobile_manipulator_mpc/test/test_route.cpp`, `test/test_references.cpp`

**Interfaces:**
- Produces:
  - `class Route { public: explicit Route(std::vector<Eigen::Vector2d> points); double length() const; struct Projection { double s; Eigen::Vector2d point; double heading; double lateral; }; Projection project(const Eigen::Vector2d & p, double s_hint) const; Eigen::Vector2d at(double s) const; double heading_at(double s) const; };` (polyline by arc length; `project` searches ±1 m around `s_hint` to stay on the right branch of a route that passes near itself)
  - `struct PanelReference { Eigen::Isometry3d panel_in_base; double start_s, end_s, ramp_m; };` `double reference_weight(const PanelReference & r, double s);` (0 before `start_s - ramp_m`, linear to 1 at `start_s`, 1 until `end_s`, linear to 0 at `end_s + ramp_m`)
  - `struct AnchoredReference { Eigen::Isometry3d panel_in_base; Eigen::Vector2d start, end; };` `std::vector<AnchoredReference> mission_references(const Json & scenario, const mobile_manipulator_geometry::FootprintProjector & projector, const Json & qualified_payload);` and `std::vector<PanelReference> anchor_on_route(const std::vector<AnchoredReference> & refs, const Route & route, double ramp_m);`
  - `Eigen::Isometry3d nominal_panel(const std::string & arm_pose, ...)` for the unscripted pull.

- [ ] **Step 1: Write the failing tests:** `Route.ProjectsOntoNearestSegment`, `Route.ProjectStaysOnBranchNearHint` (a U-shaped route), `Route.HeadingAtCorners`; `References.WeightRampsByRouteProgress` (values at start - ramp, start - ramp/2, start, end, end + ramp: 0, 0.5, 1, 1, 0), `References.NarrowGateMissionGivesVerticalThenHome` (anchors (-4.2, -7.725) -> (-9.0, -7.725) vertical carry with the mission's panel pose, then `home` from (-9.0, -7.725) to the goal), `References.NamedStateUsesQualifiedPose`, `References.AnchorOutsideRouteThrows`.
- [ ] **Step 2: Run** the MPC tests. Expected: new tests fail.
- [ ] **Step 3: Implement** `Route` and the reference functions; ramp length default 1.5 m.
- [ ] **Step 4: Run the tests.** Expected: pass.
- [ ] **Step 5: Commit** "Add route projection and route-anchored panel references".

---

### Task 6: Collision world and linearized distance rows

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/world.hpp`, `src/world.cpp`, `include/.../collision.hpp`, `src/collision.cpp`
- Modify: `docs/superpowers/specs/2026-10-11-phase4-wholebody-mpc-design.md` (Sections 2 and 4: URDF primitives instead of capsules)
- Test: `ros2_ws/src/mobile_manipulator_mpc/test/test_world.cpp`, `test/test_collision.cpp`

**Interfaces:**
- Produces:
  - `struct ObstacleBox { std::string name; Eigen::Isometry3d pose; Eigen::Vector3d size; };` (map frame, centre pose)
  - `class World { public: void set_static(const std::vector<mobile_manipulator_navigation::Box> & boxes); void update_mover(const std::string & name, const Eigen::Vector2d & position, const Eigen::Vector3d & size, double stamp); std::vector<ObstacleBox> at(double t_ahead, double now) const; };` velocity from the last two positions; a jump > 1.0 m between samples resets velocity to zero; movers older than 1.0 s are dropped.
  - `struct CollisionConfig { double margin_m = 0.05; double activation_m = 0.8; int pairs_per_stage = 8; double floor_top_m = 0.15; };`
  - `struct DistanceRow { int stage; int link; std::string obstacle; double distance; Eigen::Matrix<double, 1, kNx> gradient; };` (gradient = nᵀ J(p))
  - `struct CollisionStats { int active_pairs = 0; int dropped_pairs = 0; double compute_s = 0.0; };`
  - `class CollisionModel { public: CollisionModel(const std::vector<mobile_manipulator_geometry::Primitive> & primitives, CollisionConfig config); std::vector<DistanceRow> rows(const std::vector<State> & nominal, double dt, double now, const World & world, CollisionStats * stats) const; };` Rows also cover panel versus base primitives and panel/wrist versus the floor plane at `floor_top_m`.

- [ ] **Step 1: Write the failing tests:** `World.MoverPredictedAtConstantVelocity`, `World.MoverJumpResetsVelocity`, `World.StaleMoverDropped`; `Collision.DistanceMatchesAnalyticBoxBox` (axis-aligned panel above a box: distance = gap), `Collision.GradientMatchesFiniteDifferences` (random states near a gate post, 1e-4), `Collision.PanelThicknessNotInflated` (vertical-carry panel centred in a 1.05 m gate: panel rows report about 0.5 m per side, no row below the margin), `Collision.OnlyObstaclesWithinActivation`, `Collision.PairBudgetKeepsClosestAndCountsDropped`, `Collision.BeamAboveBaseOnlyConstrainsArmAndPanel` (a box at z 1.1 m produces no base rows), `Collision.FloorRowForLoweredPanel`.
- [ ] **Step 2: Run** the MPC tests. Expected: new tests fail.
- [ ] **Step 3: Implement** with FCL (`fcl::Boxd`, `fcl::Cylinderd`, `fcl::distance` with `enable_nearest_points`), broad phase by axis-aligned bounds of each primitive over the nominal; amend the spec as listed.
- [ ] **Step 4: Run the tests, then a timing check:** a gtest `Collision.TwentyStagesUnderOneMillisecond` (median over 200 calls near the narrow gate, Release build) prints the time; fails above 1 ms (spec fallback then applies; report).
- [ ] **Step 5: Commit** "Add the collision world and linearized distance rows over the URDF primitives".

---

### Task 7: Stage-wise QP and the OSQP backend

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/qp.hpp`, `include/.../osqp_solver.hpp`, `src/osqp_solver.cpp`
- Test: `ros2_ws/src/mobile_manipulator_mpc/test/test_osqp_solver.cpp`

**Interfaces:**
- Produces:
  - `struct StageQp { Eigen::MatrixXd Q, R, S; Eigen::VectorXd q, r; Eigen::MatrixXd A, B; Eigen::VectorXd c; Eigen::MatrixXd Cx, Cu; Eigen::VectorXd lower, upper; Eigen::VectorXd x_lower, x_upper, u_lower, u_upper; };` per stage k: variables dx_k, du_k; dynamics dx_{k+1} = A dx_k + B du_k + c; inequalities lower <= Cx dx_k + Cu du_k <= upper.
  - `struct QpProblem { std::vector<StageQp> stages; StageQp terminal; Eigen::VectorXd dx0; int slack_count = 0; double slack_weight = 1e4; std::vector<int> slack_rows; };` (each slacked row gets its own nonnegative slack with L1 weight)
  - `struct QpSolution { std::vector<Eigen::VectorXd> dx, du; Eigen::VectorXd slack; Eigen::VectorXd primal, dual; };`
  - `struct SolveResult { bool ok; std::string status; int iterations; double solve_s; QpSolution solution; };`
  - `class QpSolver { public: virtual ~QpSolver() = default; virtual void setup(const QpProblem & shape) = 0; virtual SolveResult solve(const QpProblem & problem, const QpSolution * warm) = 0; };` and `class OsqpSolver : public QpSolver` (sparsity pattern fixed at `setup`, values updated per cycle, warm start from `warm`).

- [ ] **Step 1: Write the failing tests:** `Osqp.SolvesSmallLqrToRiccatiSolution` (double integrator, N 10, compare with a discrete Riccati recursion, 1e-4), `Osqp.RespectsBoxBounds`, `Osqp.SlackActivatesOnlyWhenInfeasible`, `Osqp.WarmStartReducesIterations`, `Osqp.SamePatternUpdatesWithoutSetup` (second solve with changed values does not reallocate: check `setup` count).
- [ ] **Step 2: Run.** Expected: fail.
- [ ] **Step 3: Implement** the stacking into OSQP's CSC form; the formulation never sees OSQP types.
- [ ] **Step 4: Run.** Expected: pass.
- [ ] **Step 5: Commit** "Add a stage-wise QP interface with an OSQP backend".

---

### Task 8: The B5 formulation

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/formulation.hpp`, `src/formulation.cpp`, `config/wholebody_mpc.yaml`
- Test: `ros2_ws/src/mobile_manipulator_mpc/test/test_formulation.cpp`

**Interfaces:**
- Consumes: Tasks 4-7.
- Produces:
  - `struct Plan { std::vector<State> x; std::vector<Control> u; };` (x has N+1 entries, u has N)
  - `struct Limits { double v_max, omega_max, accel, decel, yaw_accel, yaw_decel; Eigen::Matrix<double, 6, 1> q_min, q_max, qd_max, qdd_max; Breakaway breakaway; };` (`Breakaway` as in Task 9's Interfaces, declared in `formulation.hpp`) loaded by `Limits load_limits(const std::string & envelope_yaml, const std::string & urdf_text);` (base speeds, rates and breakaway from `nav_operating_envelope.yaml`, `wholebody_arm` from Task 2, joint position limits from the URDF)
  - `struct Weights { double contour, lag, progress, heading, goal_heading, panel_position, panel_orientation, posture, effort_base, effort_arm, smooth_base, smooth_arm; };` from `wholebody_mpc.yaml`
  - `struct Inputs { State x0; Control last_command; const Route * route; double goal_s; double goal_yaw; std::vector<PanelReference> references; Eigen::Matrix<double, 6, 1> nominal_joints; };`
  - `class Formulation { public: Formulation(int horizon, double dt, Limits, Weights); QpProblem build(const Inputs & in, const Plan & nominal, const std::vector<DistanceRow> & rows) const; Plan apply(const Plan & nominal, const QpSolution & s) const; Plan shift(const Plan & plan, const State & x0) const; Plan hold(const State & x0) const; };`

- [ ] **Step 1: Write the failing tests** (closed loop with the generated model as the plant, no ROS):
  - `Formulation.TracksStraightRouteAtProgressSpeed` (10 m straight route, reaches 0.3 m/s, lateral error < 0.02 m).
  - `Formulation.RateLimitRelativeToLastCommand` (x0 at rest, last command 0.04 rad/s: first-stage omega bound is 0.04 ± 0.8·dt, independent of x0's measured rates).
  - `Formulation.TurnsTowardGoalBehindWithoutReversing` (route behind; v >= 0 at every stage, omega saturates towards the route).
  - `Formulation.PanelReferenceRampsInBeforeAnchor` (vertical-carry reference 3 m ahead: panel error at the robot's arrival < 0.05 m, 0.05 rad; arm starts moving before the anchor).
  - `Formulation.AvoidsBoxOnRoute` (box on the straight route: minimum distance over the closed loop >= margin - 0.01 m).
  - `Formulation.SlackKeepsQpFeasibleInsideMargin` (start 0.02 m from a box: QP solves, slack > 0, next plan increases distance).
  - `Formulation.RespectsJointAndRateLimits` (every stage within limits over a 30 s closed loop with references).
  - `Formulation.GoalHeadingAtEnd` (final heading error < 0.15 rad).
- [ ] **Step 2: Run.** Expected: fail.
- [ ] **Step 3: Implement** cost and constraint assembly per spec Section 3: contouring and lag linearized at each stage's projection on the route; progress speed tapering linearly over the last 1.0 m; heading to path tangent, goal heading on stages within 0.5 m of the goal; panel error weighted by `reference_weight` at the stage's route progress; posture pull to `nominal_joints` when no reference weight is active; effort and smoothness; the rate limit of stage 0 relative to `last_command`; collision rows as slacked inequalities.
- [ ] **Step 4: Run.** Expected: pass. Tune initial weights in `wholebody_mpc.yaml` until they do; record the values.
- [ ] **Step 5: Commit** "Add the B5 whole-body formulation".

---

### Task 9: Controller loop: breakaway output, fallback, braking, goal

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/controller.hpp`, `src/controller.cpp`
- Test: `ros2_ws/src/mobile_manipulator_mpc/test/test_controller.cpp`

**Interfaces:**
- Consumes: Tasks 4-8.
- Produces:
  - `struct Breakaway` (declared in `formulation.hpp`, Task 8) `{ double linear_mps, yaw_radps, rest_linear_mps = 0.01, rest_yaw_radps = 0.02, dead_linear_mps = 0.005, dead_yaw_radps = 0.01; };` `Control breakaway_output(const Control & planned, bool base_at_rest, const Breakaway & b);`
  - `enum class CycleStatus { Ok, Fallback, Braking, Failed, GoalReached, StaleState };`
  - `struct CycleTiming { double linearize_s, collision_s, setup_s, solve_s, total_s; };`
  - `struct CycleOutput { CycleStatus status; Control command; Plan plan; CycleTiming timing; CollisionStats collision; int iterations; double max_slack; double manipulability; };`
  - `class Controller { public: Controller(Formulation, CollisionModel, std::unique_ptr<QpSolver>, Breakaway, int fallback_cycles, double goal_xy_m, double goal_yaw_rad); void start(const Route & route, const mobile_manipulator_navigation::Pose2 & goal, std::vector<PanelReference> refs, Eigen::Matrix<double,6,1> nominal); CycleOutput step(double now, const State & x, bool state_fresh, const World & world, bool base_at_rest); };`

- [ ] **Step 1: Write the failing tests:** `BreakawayOutput.LiftsSubBreakawayCommandAtRest` (0.03 rad/s at rest -> 0.07), `BreakawayOutput.LeavesMovingBaseUnchanged`, `BreakawayOutput.ZeroBelowDeadZone`; `Controller.TurnsInPlaceFromRestToGoalBehind` (plant with a 0.07 rad/s yaw breakaway and 0.0375 m/s linear breakaway: goal 3 m behind reached within 30 s); `Controller.FallbackUsesShiftedPlan` (solver stub fails once: status Fallback, command = shifted plan's first control); `Controller.BrakesAfterKFailedCycles` (K = 3 failures: Braking, base decelerates at <= 0.5 m/s², arm velocities zero; after stop: Failed); `Controller.BrakesWhenStateIsStale`; `Controller.ReportsGoalReachedWithinTolerance`.
- [ ] **Step 2: Run.** Expected: fail.
- [ ] **Step 3: Implement**; K default 3 cycles.
- [ ] **Step 4: Run.** Expected: pass.
- [ ] **Step 5: Commit** "Add the B5 controller loop with breakaway output and failure handling".

---

### Task 10: `wholebody_mpc_node` and its messages

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_interfaces/msg/WholeBodyPlan.msg`, `msg/MpcCycle.msg` (and register in its CMakeLists)
- Create: `ros2_ws/src/mobile_manipulator_mpc/src/wholebody_mpc_node.cpp`
- Modify: `docs/superpowers/specs/2026-10-11-phase4-wholebody-mpc-design.md` (Sections 2, 4, 6: obstacles and references from the scenario parameter)
- Test: `ros2_ws/src/mobile_manipulator_mpc/test/test_wholebody_mpc_node.py` (launch_testing with mock TF, joint states and odometry)

**Interfaces:**
- Produces:
  - `WholeBodyPlan.msg`: `std_msgs/Header header` (map frame, stamp = cycle time), `string[] joint_names`, `float64[] time_from_start`, `geometry_msgs/Pose2D[] base`, `float64[] positions` (row-major stage x joint).
  - `MpcCycle.msg`: `std_msgs/Header header`, `string status`, `float64 linearize_s, collision_s, setup_s, solve_s, total_s, state_to_command_s`, `int32 iterations, active_pairs, dropped_pairs`, `float64 max_slack, manipulability`, `bool deadline_missed`.
  - Node `wholebody_mpc` (executable `wholebody_mpc_node`): action server `wholebody_mpc/follow_path` (`nav2_msgs/action/FollowPath`); parameters `rate_hz` (20), `scenario` (""), `references` (`scripted|none`), `boxes_file`, `envelope_file`, `stats_file`; subscribes `/joint_states`, `/odom`, `/scenario/movers`; reads TF `map -> base_footprint`; publishes `cmd_vel_wholebody` (remapped by launch to the collision monitor input), `/arm_controller/joint_trajectory` (predicted joints with positions and velocities, `header.stamp` 0 = start now), `wholebody_mpc/plan` (`WholeBodyPlan`), `wholebody_mpc/cycle` (`MpcCycle`). Timer in its own `MutuallyExclusive` callback group; QP and buffers allocated at activation. On shutdown writes the timing summary (mean, median, p90, p95, p99, max, misses per field) to `stats_file`.

- [ ] **Step 1: Write the failing launch tests:** `test_follow_path_succeeds_on_straight_route` (mock plant integrates `cmd_vel_wholebody` and the streamed joints), `test_publishes_plan_cycle_and_trajectory`, `test_cancel_holds_arm_in_place` (cancel mid-reorientation: last trajectory has one point at the current joints with zero velocity, `cmd_vel` zero), `test_new_goal_preempts`, `test_stale_joint_states_brake_and_fail`, `test_scenario_references_loaded` (`scenario:=narrow_gate_mission references:=scripted` logs two references), `test_stats_file_written_on_shutdown`.
- [ ] **Step 2: Run** (`pkill -f control_description` first). Expected: fail.
- [ ] **Step 3: Implement** the node over `Controller`; known boxes via `known_boxes(boxes_file, scenario)`; amend the spec as listed.
- [ ] **Step 4: Run.** Expected: pass.
- [ ] **Step 5: Commit** "Add the whole-body MPC node as a FollowPath server".

---

### Task 11: `controller:=wholebody` launch, behaviour tree, footprint and safety modes

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_navigation/behavior_trees/navigate_to_pose_wholebody.xml`
- Modify: `ros2_ws/src/mobile_manipulator_navigation/launch/navigation.launch.py`, `launch/global_planning.launch.py`, `config/nav2_global_planning.yaml` (a `base_band` obstacle variant, 0.05-0.60 m), `config/nav2_navigation.yaml` (comment), `src/dynamic_footprint.cpp`, `src/dynamic_footprint_node.cpp`, `src/check_footprint_ownership.cpp`, `src/check_cmd_vel_ownership.cpp`
- Test: `test/test_navigation_config.py`, `test/test_footprint_ownership.cpp`, `test/test_dynamic_footprint.cpp`

**Interfaces:**
- Produces: `footprint_mode:=wholebody`; `Polygon swept_hull(const mobile_manipulator_geometry::FootprintModel & model, const std::vector<std::pair<Pose2, JointMap>> & predicted_in_base, double padding_m);`; `DynamicFootprint` publishes in `wholebody` mode the base-only footprint to both costmaps and the stop zone, and the swept hull (+ slowdown margin) to the slowdown zone on each `wholebody_mpc/plan`.
- Launch `controller:=wholebody`: no `controller_server`, `velocity_smoother`; behaviour tree `navigate_to_pose_wholebody.xml`; `wholebody_mpc_node` with `cmd_vel_wholebody -> cmd_vel_smoothed` (the collision monitor's input); collision monitor source `max_height: 0.60`; global costmap `base_band`; `footprint_mode` forced to `wholebody`; `behavior_server` kept for `Wait`.

- [ ] **Step 1: Write the failing tests:** `test_wholebody_launch_has_no_controller_server_or_smoother`, `test_wholebody_tree_has_no_local_costmap_clear_and_points_follow_path_at_mpc`, `test_base_band_heights` (0.05/0.60 in the global costmap variant and the collision monitor override); `FootprintOwnership.WholebodyModeHasFootprintNodeAsOnlyOwner`; `DynamicFootprint.SweptHullContainsEveryPredictedConfiguration`, `DynamicFootprint.WholebodyCostmapFootprintIsBaseOnly`.
- [ ] **Step 2: Run.** Expected: fail.
- [ ] **Step 3: Implement.** `check_cmd_vel_ownership` accepts `wholebody_mpc` as the one upstream publisher.
- [ ] **Step 4: Run** all navigation tests. Expected: pass; B1-B4 launch tests unchanged.
- [ ] **Step 5: Commit** "Launch B5 with its behaviour tree, base-band costmap and swept-hull safety zone".

---

### Task 12: Runner, metrics and summaries for B5

**Files:**
- Modify: `tools/run_nav_scenario.py`, `tools/summarize_nav_runs.py`, `tools/test_summarize_nav_runs.py`
- Create: `ros2_ws/src/mobile_manipulator_mpc/src/clearance_3d.cpp` (executable `clearance_3d`: minimum distance between the robot primitives and the known boxes over a bag's `/tf` and `/joint_states`)
- Test: `ros2_ws/src/mobile_manipulator_mpc/test/test_clearance_3d.cpp`, `tools/test_summarize_nav_runs.py`

**Interfaces:**
- Produces: `CONTROLLERS = ("rpp", "dwb", "mppi", "wholebody")`; `--references {scripted,none}` (default `scripted`; run directory suffix `-noref` for `none`); for `wholebody` a `mission` scenario runs `navigate_scenario_task` from start to goal with the arm recorder and the MPC node given `scenario:=<name> references:=<...>`; `summary.json` gains `mpc` (the stats file) and `clearance_3d_m`; bag topics add `wholebody_mpc/cycle` and `wholebody_mpc/plan`. `summarize_nav_runs` adds columns `MPC p99 ms`, `misses`, `near-misses`, `3D clearance m` when present.

- [ ] **Step 1: Write the failing tests:** `Clearance3d.MatchesCollisionModelDistance` (synthetic bag-free input: same states, same value as `CollisionModel` without margin); `test_summarize_wholebody_columns` (fixture summary with `mpc` block).
- [ ] **Step 2: Run.** Expected: fail.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** the tests and `python3 tools/run_nav_scenario.py --help` (shows `wholebody` and `--references`). Expected: pass.
- [ ] **Step 5: Commit** "Run and score B5 scenarios with timing and 3D clearance".

---

### Task 13: B5 bring-up in Unity

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_mpc/config/wholebody_mpc.yaml` (tuned weights)
- Modify: `docs/experiments/wholebody-mpc/README.md` (section "Bring-up")

- [ ] **Step 1: Open space.** `tools/run_nav_scenario.py open_space_nav --controller wholebody --new-epoch`. Expected: success, no contact, p99 total < 50 ms at 20 Hz.
- [ ] **Step 2: Goal behind the robot.** Repeat the 2026-10-06 probe (start (12.01, 0) facing -pi, goal (15, 0) facing 0) via `tools/demo.py drive --controller wholebody` and a `ros2 action send_goal /navigate_to_pose`. Expected: turns in place and arrives within tolerance.
- [ ] **Step 3: Gates and a mission.** `wide_gate_vertical_carry_nav`, `narrow_gate_vertical_carry_nav`, `wide_gate_mission` (scripted). Expected: success or a recorded failure reason; the arm reorients while driving in the mission (no stop at the waypoints).
- [ ] **Step 4: Tune** only `wholebody_mpc.yaml` weights if a step fails for a tuning reason; record each change and its reason. Stop and report if a failure needs a design change.
- [ ] **Step 5: 50 Hz check** on `wide_gate_mission`: `rate_hz:=50`. Record p99 and misses; choose the campaign rate (50 Hz if p99 < 20 ms and misses < 1 %, else 20 Hz).
- [ ] **Step 6: Stop Play; commit** "Bring up B5 in Unity and record the tuning".

---

### Task 14: Worker interaction scenarios and the overhead course

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_navigation/config/scenarios.yaml`
- Test: `ros2_ws/src/mobile_manipulator_navigation/test/test_scenario_spec.cpp` (all scenarios valid), `test/test_mission.cpp`

**Interfaces:**
- Produces scenarios: `worker_late_crossing_nav`, `worker_stops_in_lane_nav` (arm `home`, open band, `navigate_to_pose`); `overhead_course_nav` (drive version, arm `vertical_carry`) and `overhead_course_mission` (start `home`, reconfigure to vertical carry before the gate, to `home` before the beam, a 90° turn after it). The beam is a scenario obstacle with `z` set so vertical carry collides and `home` clears it by >= 0.10 m.

- [ ] **Step 1: Write the failing test:** `ScenarioSpec.NewPhase4ScenariosValid` (the four names resolve with no problems) and `Mission.OverheadCourseMissionSteps` (two reconfigure steps, panel poses as above).
- [ ] **Step 2: Choose the layout:** place the course in free map space with a 1.05 m-gate-equivalent pair of posts (scenario obstacles), the beam 3-4 m after the gate, the turn 3 m after the beam; check with `scenario_spec` and a plan-only run that Nav2 (B5 `base_band`) routes under the beam and B1-B4 (0.05-2.0 m band) do not.
- [ ] **Step 3: Tune the worker timing** (speed, trigger distance) with a `rpp` run each until the worker reaches the lane within 0.5 m of the robot's front in the late crossing, and stops on the route 2-3 m ahead in the stop scenario. Record the values.
- [ ] **Step 4: Run** the tests. Expected: pass. Stop Play.
- [ ] **Step 5: Commit** "Add the worker interaction scenarios and the overhead course".

---

### Task 15: Free drive with B5

**Files:**
- Modify: `tools/demo.py`, `docs/running-the-stack.md`

- [ ] **Step 1: Implement** `drive --controller wholebody` (launches `controller:=wholebody` with `references:=none` and no MoveIt) and the menu entry; `arm` refuses with a message under B5 (the MPC owns the arm).
- [ ] **Step 2: Verify in Unity:** three RViz goals (ahead, behind, beside a gate post). Expected: arrival without contact; RViz shows the predicted plan and the swept slowdown zone. Stop Play.
- [ ] **Step 3: Commit** "Add free drive with the whole-body MPC to the demo launcher".

---

### Task 16: Phase 4 campaign

**Files:**
- Modify: `docs/experiments/wholebody-mpc/README.md` (sections per scenario group, timing, findings)

- [ ] **Step 1: Run the matrix** of spec Section 9 at the rate chosen in Task 13 (3 runs per cell): B5 scripted on the 8 Phase 1 driving scenarios and the 3 B3 missions; B5 unscripted on the gate missions and the overhead course; the two worker scenarios with RPP, DWB, MPPI, B4 (RPP, `--footprint-mode dynamic`) and B5; the overhead course with B1/B2 (drive version), B3, B4 and B5 (mission version). Use `caffeinate`; resume from the last finished cell after interruptions.
- [ ] **Step 2: Timing runs** at 20 and 50 Hz on `wide_gate_mission` and `overhead_course_mission` (3 each).
- [ ] **Step 3: Summarize** with `tools/summarize_nav_runs.py`; write results and findings (including contact, near-misses, `off_goal`, unscripted failures as the Phase 5 motivation) to the README. Stop Play.
- [ ] **Step 4: Commit** "Record the Phase 4 campaign".

---

### Task 17: ADR, roadmap and final documentation

**Files:**
- Create: `docs/adr/0011-wholebody-mpc-command-ownership.md`
- Modify: `docs/adr/0006-base-command-ownership.md` (revisit note), `docs/adr/0010-dynamic-footprint-ownership.md` (wholebody mode), `docs/roadmap.md`, `docs/running-the-stack.md`

- [ ] **Step 1: Write ADR 0011** (FollowPath server in its own process, `cmd_vel` path through the collision monitor, JTC streaming, fallbacks, safety layer) and the ADR 0006/0010 notes.
- [ ] **Step 2: Update the roadmap:** Phase 4 implemented-B5 section; baseline matrix row B5; resolved open decisions (B4 in-drive footprint: grow-only; B6 optional; F4 before Phase 5); new entries from spec Section 11 (CasADi codegen, Bézier local MPC as a Phase 6 ablation, flat-output yaw and axis-angle orientation for Phase 5, moving obstacles in the Phase 5 triggers, perception baseline with the 3D safety check, the 50 Hz actuation note for 4B); decision-log entries for this phase.
- [ ] **Step 3: Check links** (`grep -o '](.*\.md)' docs/roadmap.md docs/adr/0011-*.md` targets exist).
- [ ] **Step 4: Commit** "Document B5: ADR 0011, roadmap and experiment results".
