# Phase 4B-A: B5 Behaviour Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix B5's door deadlock, left-only swing, slowdown confound, stage-0 command mismatch and (if the evidence supports it) its margin shortfall; rerun B5 on the cells that show them; freeze the formulation for 4B-B; then demonstrate low passes.

**Architecture:** The safety layer changes only in `footprint_mode wholebody` (StopZone 0.02 m, SlowdownZone off), and the MPC gets a base-only margin coupled to it plus a blocked detector fed by the executed `/cmd_vel` and `/collision_monitor_state`. The posture blend gets a pure side picker over two mirrored tight poses with a clearance guard. Every new decision is a small pure class in `mobile_manipulator_mpc` with its own gtest; the node only wires ROS topics into them.

**Tech Stack:** ROS 2 Jazzy C++17 (rclcpp, rclcpp_action, Nav2 `FollowPath`, `nav2_msgs/CollisionMonitorState`), Eigen, FCL, OSQP, gtest, launch_testing (Python), pytest, Python tools in `tools/`, Unity Editor via the Unity CLI (`com.unity.pipeline`).

**Spec:** docs/superpowers/specs/2026-10-18-phase4b-a-b5-behaviour-fixes-design.md (read it with this plan; "Section N" below refers to it).

## Global Constraints

- Branch `phase4b-fixes`; one commit per fix; never commit to `main`; the user merges.
- B1-B4 behaviour and configuration unchanged (frozen at tag `baselines-2026-10-13`); every change is gated on `footprint_mode wholebody` / `controller:=wholebody` or lives in `mobile_manipulator_mpc`.
- Values fixed by the spec: StopZone margin in wholebody mode 0.02 m; arm/panel margin 0.05 m; `base_margin_m = padding_m (0.02) + 0.02 + q` with q the measured lidar intrusion rounded up to the centimetre (stop if q > 0.08 m; measured q = 0.08, base margin 0.12 m — amended during Task 1); blocked: own |v| >= 0.05 m/s or |ω| >= 0.1 rad/s, executed |v| < 0.01 m/s and |ω| < 0.02 rad/s, `detect_s` 1.0, `fail_s` 3.0, `clear_s` 0.5, `progress_m` 0.05, executed fresh for 0.5 s (kExecutedStaleS; amended after the Task 3 review: two periods fired on monitor jitter), pass-through match 1e-3 against the last five sent commands; side picker α_on 0.05, α_off 0.02, comfort distance 0.30 m (`kTightComfortM`), score window k*-1..k*+1, guard and choice difference 0.02 m, tie-break |pan| > 0.1 rad; lag compensation clamp 0.02 m; decision rule reference clearance >= 0.049 m and >= 80 % of the shortfall.
- Runtime nodes and ROS package code in C++; launch files, configuration tests and `tools/` in Python.
- Container: `C='docker exec mm-motion-planning-ma-robot-sim-1 bash -ic'`; package tests: `$C 'cd $ROS_WS && colcon build --packages-select <pkgs> && colcon test --packages-select <pkgs> --event-handlers console_direct+ && colcon test-result --verbose'` (add `--ctest-args -R <test>` for one gtest). `pkill -f control_description` before mock tests.
- Unity only through the Unity CLI and pipeline commands (unity-cli skill), never Unity MCP; stop Play (`editor_stop`) when a task that used it finishes; long runs under `caffeinate -i`.
- No host-absolute paths, credentials, Unity `Library/`, or ROS `build/ install/ log/` in commits.
- Live runs: `python3 tools/run_nav_scenario.py <scenario> --controller wholebody --mpc-rate-hz 50` (current defaults: carry comfort pose, slack gate on); run directories under `experiment_runs/` stay git-ignored.

### Plan-level decisions not spelled out in the spec

- **Row margins:** `DistanceRow` gains `double margin`, set by `CollisionModel::rows` (`base_margin_m` for base bodies, `margin_m` otherwise; floor and self rows `margin_m`). `Formulation::build` and the slack gate use `row.margin` instead of the config margin; lag compensation (Task 10) raises `row.margin`. `CollisionConfig` gains `base_margin_m` (default = `margin_m`, so existing tests are unchanged).
- **The StopZone's wholebody margin** lives next to its margin in `nav2_navigation.yaml` (`StopZone.wholebody_margin_m: 0.02`); `MonitorZone` gains `wholebody_margin_m` (defaults to `margin_m` when absent); `dynamic_footprint_node` uses it in wholebody mode. `wholebody_mpc.yaml` records `collision.lidar_intrusion_m` (q) and `collision.base_margin_m`; a config test asserts `base_margin_m == dynamic_footprint.yaml padding_m + StopZone.wholebody_margin_m + lidar_intrusion_m` (amended during Task 1).
- **Previous command:** the node composes it each cycle (base from `ExecutedCommand`, arm from the streamed spline's velocity now) and passes it with the existing `Controller::reset_command(const Control &)` before `step()`; the controller is not changed for it.
- **Blocked hold:** `Controller::set_external_hold(bool)`; while set, a solved plan is not executed and the cycle holds exactly like a slack-gate hold but reports the new `CycleStatus::Blocked`. The decision is `BlockedDetector` (pure); the node feeds it and ends the goal.
- **Stop ramp after a goal ends** (cancel, abort): the node keeps publishing `brake_base(from, elapsed, limits, breakaway)` from its timer until zero; a new goal ends the ramp and calls `reset_command` with the ramped base command. `brake_base` is the base half of `Controller::brake`, moved to a free function.
- **Mirror pose solve:** `carry_pose_search --mirror-of vertical_carry` (damped least squares from the sign-flipped seed, matching panel centre and panel axes up to sign, joints within the search's limits).
- **`posture.tight_poses`** replaces the single tight pose; `PostureTargets::tight` becomes `std::vector<Eigen::Matrix<double,6,1>> tight` (one or two entries); `posture_targets(payload, comfort, mode, tight_names)` resolves names from `poses_rad`.

## Review Focus

1. **The monitor's output lagging the MPC by a message** while accelerating: expected unchanged acceleration in open space, not a stage-0 band stuck on the previous command. Test: Task 3 `ExecutedCommand.PassThroughUsesTheLatestSent`.
2. **No `/cmd_vel` message yet at a goal's start** (or a monitor that publishes only after its first input): expected no `blocked` in the first second of a normal start. Test: Task 5 `BlockedDetector.FirstMessageWithinDetectTimeNeverBlocks`.
3. **The behaviour tree's 1 Hz in-place FollowPath preemptions while blocked:** expected the accumulated blocked time to survive `update_goal`, so the 3 s failure still fires. Test: Task 5 `test_blocked_survives_preemption` (node launch test).
4. **A goal that starts with the arm already swung right** (a previous run ended in `vertical_carry_right`): expected the picker to keep right on a tie, not swing back across. Test: Task 8 `SidePicker.TieKeepsTheCurrentPanSide`.
5. **Adding a pose to `poses_rad`** breaks consumers that enumerate it (MoveIt SRDF named states, footprint profiles): expected every package test green after registration. Test: Task 7 Step 5 runs those packages' suites.

---

### Task 1: Measure the lidar intrusion q (Section 2, evidence)

**Files:**
- Create: `tools/measure_lidar_intrusion.py`, `tools/test_measure_lidar_intrusion.py`
- Modify: `docs/experiments/wholebody-mpc/README.md` (new section "Lidar intrusion at the StopZone (2026-10-xx)")

**Interfaces:**
- Produces: `q` (m, rounded up to 0.01) recorded in the README; Task 4 copies it into `wholebody_mpc.yaml` `collision.lidar_intrusion_m`.

- [ ] **Step 1: Write the failing tests** for the pure statistics in `measure_lidar_intrusion.py`:

```python
def test_kth_deepest_intrusion_per_scan():
    # face at x = 1.0, robot on the -x side: intrusion = 1.0 - x for points with x < 1.0
    scan = [(0.97, 0.0), (0.99, 0.1), (0.95, -0.1), (1.02, 0.0), (0.98, 0.2)]
    assert kth_deepest_intrusion(scan, face_x=1.0, k=4) == pytest.approx(0.01)
def test_scan_with_fewer_than_k_points_counts_zero():
    assert kth_deepest_intrusion([(0.9, 0.0)], face_x=1.0, k=4) == 0.0
def test_q_is_the_999th_percentile_rounded_up_to_a_centimetre():
    assert q_from(per_scan=[0.01] * 999 + [0.043]) == pytest.approx(0.05)
```

- [ ] **Step 2: Run** `python3 -m pytest tools/test_measure_lidar_intrusion.py -v`. Expected: FAIL (module missing).
- [ ] **Step 3: Implement** `kth_deepest_intrusion(points_xy, face_x, k) -> float` (points in the wall's frame, intrusion measured along the wall normal towards the robot, points behind the face ignored) and `q_from(per_scan) -> float` (numpy 99.9th percentile, `math.ceil(v * 100) / 100`). The orchestration reuses `run_nav_scenario.Runner`: for each obstacle (a tall wall box 2.0 x 0.2 x 2.0 m and a door jamb 0.1 x 0.2 x 2.0 m, placed with `scenario_obstacle` in the V1 open test area) and each base-hull distance 0.05, 0.08, 0.11, 0.14, 0.20 m, `scenario_place` the robot (arm `carry`) facing and beside it, then run an in-container `python3` rclpy script for 60 s that transforms `/livox/points_filtered` into the map frame with tf2, keeps points inside the StopZone polygon (base hull + 0.02 m) and the band 0.05-0.60 m, and prints per-scan intrusion as JSON. Prints q and the per-position table.
- [ ] **Step 4: Run** the tests. Expected: PASS.
- [ ] **Step 5: Run the measurement** (Unity in Play on the V1 scene, no Nav2): `caffeinate -i python3 tools/measure_lidar_intrusion.py --output experiment_runs/lidar-intrusion.json`. Expected: q printed, 20 positions x ~600 scans. **If q > 0.08 m: stop and report to the user** (Section 2 stop rule). Stop Play.
- [ ] **Step 6: Record** q, the table and the method in the README section; commit tool, tests and README: `git commit -m "Measure the lidar intrusion the StopZone sees"`.

### Task 2: Offline lag check (Section 5, step 1)

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/clearance.hpp`, `src/clearance_bag.cpp`, `include/.../clearance_bag.hpp`, `src/clearance_3d.cpp`
- Test: `ros2_ws/src/mobile_manipulator_mpc/test/test_clearance_3d.cpp`
- Modify: `docs/experiments/wholebody-mpc/README.md` (section "Commanded versus measured arm at the closest approach")

**Interfaces:**
- Produces: `ClearanceSample::reference` (`std::optional<Eigen::Matrix<double,6,1>>`, from `/arm_controller/controller_state` `reference.positions` by joint name, latest at the sample time); `ReferenceComparison compare_reference(const CollisionModel &, const std::vector<ClearanceSample> &, const std::vector<ObstacleBox> &, double margin_m)` returning `{measured_min, reference_at_measured_min, samples_below_margin, explained (bool), worst_shortfall_m, covered_fraction}`; `clearance_3d --compare-reference` adds them to its JSON.

- [ ] **Step 1: Write the failing tests** in `test_clearance_3d.cpp`: `CompareReference.LagExplainsShortfall` (a box 0.045 m from the measured panel, the reference state 0.006 m farther: `explained == true`, `covered_fraction >= 0.8`); `CompareReference.NoLagDoesNotExplain` (reference = measured: `explained == false`); `ClearanceBag.ReadsControllerStateReference` (a small bag written in the test with one `controller_state` message: the sample's `reference` equals its positions in model order).
- [ ] **Step 2: Run** `$C '... colcon test --packages-select mobile_manipulator_mpc --ctest-args -R test_clearance_3d'`. Expected: FAIL to compile.
- [ ] **Step 3: Implement** the reader (subscribe the bag reader to `control_msgs/msg/JointTrajectoryControllerState`), `compare_reference` (decision rule of Section 5: every sample with measured clearance < margin has reference clearance >= margin - 0.001 and the difference covers >= 80 % of `margin - measured`), and the CLI flag.
- [ ] **Step 4: Run** the tests. Expected: PASS.
- [ ] **Step 5: Run on the 17 runs:** every `experiment_runs/2026101[23]T*-wholebody-*` whose `clearance_3d.json` distance is in [0, 0.05): `clearance_3d --bag <run>/bag --scenario <scenario> --compare-reference`. Tabulate per run: measured min, reference at that time, difference, per-joint lag, `explained`.
- [ ] **Step 6: Record** the table and the verdict in the README; commit: `git commit -m "Compare commanded and measured arm clearance on the below-margin runs"`. The verdict selects Task 10's branch (2a if every run is explained, otherwise the 2b checkpoint).

### Task 3: Executed-command feedback, arm stage-0, stop ramp, SlowdownZone default (Section 4)

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/executed_command.hpp`, `src/executed_command.cpp`, `test/test_executed_command.cpp` (+ CMake: add to the controller library and `ament_add_gtest`)
- Modify: `include/.../controller.hpp`, `src/controller.cpp` (extract `brake_base`), `src/wholebody_mpc_node.cpp`, `test/test_wholebody_mpc_node.py`
- Modify: `ros2_ws/src/mobile_manipulator_navigation/launch/navigation.launch.py` (`slowdown_zone` default `""`: wholebody false, others true)
- Create: `ros2_ws/src/mobile_manipulator_navigation/test/test_navigation_launch_zones.py`

**Interfaces:**
- Produces:
  - `class ExecutedCommand { void on_sent(double t, const Eigen::Vector2d & vw); void on_executed(double t, const Eigen::Vector2d & vw); Eigen::Vector2d previous(double now, double max_age_s) const; Eigen::Vector2d executed(double now, double max_age_s) const; }` — `executed()` is the raw latest executed command or zero when older than `max_age_s` (never received counts as old); `previous()` applies the pass-through rule (match within 1e-3 of one of the last five `on_sent` values → the latest sent; else `executed()`).
  - `Eigen::Vector2d brake_base(const Eigen::Vector2d & from, double elapsed_s, const Limits & l, const Breakaway & b)` in `controller.hpp` (what `Controller::brake` does for the base; `brake` calls it).
  - The node subscribes to `/cmd_vel` (`geometry_msgs/Twist`, depth 10) and calls `on_sent` after each publish.

- [ ] **Step 1: Write the failing tests:**

```cpp
TEST(ExecutedCommand, PassThroughUsesTheLatestSent) {
  ExecutedCommand e; e.on_sent(0.00, {0.10, 0.0}); e.on_sent(0.02, {0.12, 0.0});
  e.on_executed(0.021, {0.10, 0.0});                       // the monitor forwarded the older one
  EXPECT_TRUE(e.previous(0.03, 0.04).isApprox(Eigen::Vector2d(0.12, 0.0)));
}
TEST(ExecutedCommand, ScaledCommandIsUsedAsIs) {   // FootprintApproach scaled 0.12 -> 0.06
  ExecutedCommand e; e.on_sent(0.0, {0.12, 0.0}); e.on_executed(0.001, {0.06, 0.0});
  EXPECT_TRUE(e.previous(0.01, 0.04).isApprox(Eigen::Vector2d(0.06, 0.0)));
}
TEST(ExecutedCommand, SilenceIsZero) {
  ExecutedCommand e; e.on_sent(0.0, {0.3, 0.2}); e.on_executed(0.0, {0.3, 0.2});
  EXPECT_TRUE(e.executed(0.05, 0.04).isZero()); EXPECT_TRUE(e.previous(0.05, 0.04).isZero());
}
TEST(ExecutedCommand, NeverReceivedIsZero) { ExecutedCommand e; EXPECT_TRUE(e.previous(1.0, 0.04).isZero()); }
TEST(BrakeBase, RampsAtTheEnvelopeDecel) { /* from (0.3, 0.4), elapsed 0.1: v drops by l.decel*0.1, omega by l.yaw_decel*0.1, never past zero */ }
```

  Launch zone tests (`test_navigation_launch_zones.py`, importing `navigation.launch.py` with `importlib`): `test_wholebody_disables_the_slowdown_zone_by_default` (`slowdown_parameters` for controller wholebody and value `""` gives `{"SlowdownZone.enabled": False}`), `test_wholebody_slowdown_zone_true_reenables`, `test_other_controllers_keep_the_zone_by_default` (rpp, `""` → `{}`). Node launch test: `test_cancel_ramps_the_base_down` (after a cancel at 0.3 m/s, `cmd_vel_wholebody` shows at least two decreasing non-zero values within the envelope decel before zero), `test_previous_command_follows_the_executed_cmd_vel` (the test publishes a scaled `/cmd_vel`; the next cycle's command stays within one rate step of the scaled value).

- [ ] **Step 2: Run** the mpc gtest and the two Python tests. Expected: FAIL.
- [ ] **Step 3: Implement** `ExecutedCommand`, `brake_base`, the node wiring (each cycle before `step()`: `reset_command` with base `previous(now, 2 * period_s_)` and arm = the streamed spline's velocity at now (the `v0` that `stream()` samples; zero when nothing was streamed or the arm is held)), the stop ramp (on every `end_goal` except `Succeeded`: ramp from the last sent base command with `brake_base` each timer tick until zero; `take_goal` ends it and calls `reset_command` with the ramped command), and the launch default (`slowdown_parameters(value, wholebody)`; launch argument default `""`; its description updated).
- [ ] **Step 4: Run** `colcon test --packages-select mobile_manipulator_mpc mobile_manipulator_navigation`. Expected: PASS, existing tests unchanged.
- [ ] **Step 5: Live check:** `narrow_gate_home_nav --arm-pose carry` and `overhead_course_nav --arm-pose home`, one run each. Expected: success, no new holds, times near the no-zone follow-ups (19.7 s and 50.3 s without the clearance cost, so slower is plausible; report the numbers). Stop Play.
- [ ] **Step 6: Commit:** `git commit -m "Feed the executed command back to B5 and ramp down on stop"` (split the launch default into its own commit: `"Disable the slowdown zone for B5 by default"`).

### Task 4: Base margin coupled to the StopZone (Section 2)

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_mpc/include/.../collision.hpp`, `src/collision.cpp`, `include/.../formulation.hpp` + `src/formulation.cpp` (row margin in `build`; `MpcConfig::collision` read by `load_mpc_config` from the yaml's `collision:` block), `src/controller.cpp` (`gate_passes` uses `row.margin`), `src/wholebody_mpc_node.cpp` (passes `config.collision` to both `Formulation` and `CollisionModel` instead of `CollisionConfig{}` at line 230), `config/wholebody_mpc.yaml`
- Modify: `ros2_ws/src/mobile_manipulator_navigation/config/nav2_navigation.yaml` (`StopZone.wholebody_margin_m: 0.02`), `include/.../mission.hpp` + `src/mission.cpp` (`MonitorZone::wholebody_margin_m`), `src/dynamic_footprint_node.cpp` (wholebody: use it)
- Test: `test/test_collision.cpp`, `test/test_controller.cpp`, `ros2_ws/src/mobile_manipulator_navigation/test/test_mission.cpp` (or the file that tests `monitor_zones`), `ros2_ws/src/mobile_manipulator_mpc/test/test_mpc_config.py` (new)

**Interfaces:**
- Consumes: q from Task 1.
- Produces: `CollisionConfig::base_margin_m`; `DistanceRow::margin`; `MonitorZone::wholebody_margin_m`; yaml keys `collision.base_margin_m`, `collision.lidar_intrusion_m`.

- [ ] **Step 1: Write the failing tests:** `Collision.BaseRowsCarryTheBaseMargin` (config base 0.07, arm 0.05: every row of a base body has `margin == 0.07`, panel and wrist rows 0.05, floor and self rows 0.05); `Controller.BaseStopsAtTheBaseMargin` (closed loop driving the base at a wall: closest base distance >= 0.069 m and < 0.08 m); `SlackGate.UsesTheRowMargin` (a base pair at 0.06 m with base margin 0.07 counts a 0.01 m violation); `MonitorZones.WholebodyMarginDefaultsToMargin` and `...ReadsWholebodyMargin` (StopZone 0.02, SlowdownZone falls back to its margin); `test_base_margin_is_stop_margin_plus_intrusion` (`wholebody_mpc.yaml` `base_margin_m == nav2_navigation.yaml StopZone.wholebody_margin_m + lidar_intrusion_m` within 1e-9).
- [ ] **Step 2: Run** the tests. Expected: FAIL.
- [ ] **Step 3: Implement** the margins as in the plan-level decisions; yaml `collision: {base_margin_m: <0.02 + q>, lidar_intrusion_m: <q>}` with a comment pointing at Task 1's README section.
- [ ] **Step 4: Run** `colcon test --packages-select mobile_manipulator_mpc mobile_manipulator_navigation`. Expected: PASS.
- [ ] **Step 5: Commit:** `git commit -m "Give B5's base rows a margin outside the StopZone"` and `git commit -m "Tighten the StopZone to 0.02 m in wholebody mode"` (two commits: MPC, then navigation).

### Task 5: Blocked detection and the FollowPath failure (Section 2)

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_mpc/include/.../blocked_detector.hpp`, `src/blocked_detector.cpp`, `test/test_blocked_detector.cpp`
- Modify: `include/.../controller.hpp`, `src/controller.cpp` (`CycleStatus::Blocked`, `set_external_hold`), `src/wholebody_mpc_node.cpp`, `config/wholebody_mpc.yaml` (`blocked:` block), `ros2_ws/src/mobile_manipulator_interfaces/msg/MpcCycle.msg` (status comment lists `blocked`), `package.xml`/CMake (`nav2_msgs` for `CollisionMonitorState` if not yet a dependency), `test/test_wholebody_mpc_node.py`

**Interfaces:**
- Consumes: `ExecutedCommand::executed()` (Task 3).
- Produces:
  - `struct BlockedConfig { double own_v = 0.05, own_w = 0.1, exec_v = 0.01, exec_w = 0.02, detect_s = 1.0, fail_s = 3.0, clear_s = 0.5, progress_m = 0.05; };`
  - `class BlockedDetector { explicit BlockedDetector(BlockedConfig); enum class State { Clear, Blocked, Failed }; State update(double now, const Eigen::Vector2d & own, const Eigen::Vector2d & executed, bool monitor_stop, const Eigen::Vector2d & base_xy); void restart(); };` — `restart()` on a new goal only (not on preemption).
  - `void Controller::set_external_hold(bool on)`; `CycleStatus::Blocked` (`to_string` "blocked").

- [ ] **Step 1: Write the failing tests:** `BlockedDetector.DetectsAfterOneSecond` (own 0.3, executed 0 from t=0: Clear at 0.99, Blocked at 1.0); `...ScaledIsNotBlocked` (executed 0.06); `...FirstMessageWithinDetectTimeNeverBlocks` (executed absent (zero) until 0.5 s, then follows); `...ClearsAfterMonitorNotStopForClearS` (Blocked, then `monitor_stop` false from t: Clear at t + 0.5); `...AccumulatesAcrossHoldsAndFailsAtThreeSeconds` (alternating 1.5 s blocked / 0.4 s resumed with no base motion: Failed when the blocked time sums to 3.0); `...BaseMotionRestartsTheAccount` (0.06 m of motion between blocks: no Failed); `Controller.ExternalHoldHoldsLikeTheGate` (with `set_external_hold(true)` a solved cycle reports `Blocked`, the base brakes at the envelope limits and the plan is not executed; releasing resumes `Ok`). Node launch tests: `test_blocked_fails_follow_path_with_105` (test publishes `/collision_monitor_state` STOP and a zero `/cmd_vel` while the MPC commands: the goal aborts with error 105 and `error_msg` "blocked by the collision monitor" within 4.5 s; cycle status `blocked` seen); `test_blocked_survives_preemption` (resending the same path every 1 s while blocked still aborts by 4.5 s).
- [ ] **Step 2: Run** the tests. Expected: FAIL.
- [ ] **Step 3: Implement** the detector (states as tested; "own" is the controller's sent base command of the previous cycle), the controller hold path (in the tracking branch, after a successful solve, `holding = holding || external_hold_` with status `Blocked` when the gate alone would pass), and the node (subscribe `/collision_monitor_state` `nav2_msgs/msg/CollisionMonitorState`, `monitor_stop = action_type == STOP`, latest value; feed the detector every cycle; `set_external_hold(state == Blocked)`; on `Failed` call `end_goal(End::Aborted, FAILED_TO_MAKE_PROGRESS, "blocked by the collision monitor")`; `restart()` in `start_goal` when not preempting); yaml `blocked:` with the values above, read into `BlockedConfig`.
- [ ] **Step 4: Run** `colcon test --packages-select mobile_manipulator_interfaces mobile_manipulator_mpc`. Expected: PASS.
- [ ] **Step 5: Live check** (after Tasks 3-5): `v2_room_r4_nav`, `v2_store_room_nav`, `v2_door_105_nav`, `narrow_gate_home_nav --arm-pose carry`, one run each. Expected: R4 and store S succeed (or, if one fails, it ends with 105 "blocked" instead of freezing; report the StopZone events and the closest base distance); the other two succeed as before. Stop Play.
- [ ] **Step 6: Commit:** `git commit -m "Detect a blocked base and fail FollowPath instead of freezing"`.

### Task 6: Find `vertical_carry_right` by forward kinematics (Section 3) — user checkpoint

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_mpc/src/carry_pose_search.cpp` (`--mirror-of NAME`)

**Interfaces:**
- Produces: the joint vector `vertical_carry_right` (6 values, 1 mrad) with its residuals; used by Task 7.

- [ ] **Step 1: Implement** `--mirror-of NAME`: target panel centre = the named pose's centre with y negated, target panel axes = its axes mirrored across the base x-z plane (compared up to sign, the box is symmetric); damped least squares (λ 0.05, at most 200 iterations) from each seed `[-q1, q2, q3, q4, ±q5, ±q6]`; keep solutions inside the search's joint limits (incl. `wrist_2` 1.77 rad) that pass `check()` with the footprint test relaxed to the mirrored pose's own footprint; print the best with position residual, axis residual (rad) and the usual metrics table.
- [ ] **Step 2: Run** `$C '... ros2 run mobile_manipulator_mpc carry_pose_search --mirror-of vertical_carry'`. Expected: one solution, position residual < 1 mm, axis residual < 1 mrad, panel-base and arm clearances at least `vertical_carry`'s minus 0.005 m. If none: stop and report (Section 11 risk: B5 keeps one tight pose).
- [ ] **Step 3: Checkpoint:** in Unity Play (V1 open test area), `scenario_place` the robot and move the arm to the candidate (`arm_experiment --positions ... --duration 8`), take a screenshot; then the same for `vertical_carry`; show both to the user and **wait for approval** before Task 7. Stop Play.
- [ ] **Step 4: Commit** the tool change: `git commit -m "Solve a mirrored pose in carry_pose_search"`.

### Task 7: Qualify and register `vertical_carry_right`

**Files:**
- Modify: `tools/run_wholebody_arm_qualification.py` (suite `vertical_right`), `ros2_ws/src/mobile_manipulator_control/config/qualified_payload.json` (`poses_rad.vertical_carry_right`), `ros2_ws/src/mobile_manipulator_moveit_config/config/mobile_manipulator.srdf` (group_state of the same name), `ros2_ws/src/mobile_manipulator_navigation/config/footprint_profiles.yaml` only if its test requires a profile per pose, `docs/experiments/wholebody-mpc/README.md` (section "vertical_carry_right qualification"), `docs/experiments/wholebody-mpc/arm-qualification.json.gz`

**Interfaces:**
- Consumes: the joint vector from Task 6.
- Produces: `poses_rad.vertical_carry_right`.

- [ ] **Step 1: Add the suite** `vertical_right` (mirrors `carry`): hold (8 s cubic from `carry`, held 20 s) still, `compact` and `extended` drive; transitions `carry <-> vertical_carry_right` at the `wholebody_arm` limits (`limit_duration`), still and `compact`; JTC streaming at 50 Hz about the pose; 3 repeats each; acceptance as `carry` (path error <= 0.10 rad, no watchdog, tilt < 3 degrees, panel bottom >= 0.15 m, no panel penetration, hold error <= 0.06 rad).
- [ ] **Step 2: Run** `caffeinate -i python3 tools/run_wholebody_arm_qualification.py vertical_right --prefix vcr1` then `--table`. Expected: every row `pass`. If a row fails: stop and report (Section 11 risk). Stop Play.
- [ ] **Step 3: Register** the pose in `qualified_payload.json` and the SRDF (and a footprint profile if `test_footprint_profiles.py` requires one per pose; generate it with the existing profile generator, never by hand).
- [ ] **Step 4: Record** the table in the README.
- [ ] **Step 5: Run** `colcon test --packages-select mobile_manipulator_control mobile_manipulator_moveit_config mobile_manipulator_navigation mobile_manipulator_mpc mobile_manipulator_geometry mobile_manipulator_manipulation`. Expected: PASS.
- [ ] **Step 6: Commit:** `git commit -m "Qualify and register vertical_carry_right"`.

### Task 8: Side picker with the clearance guard (Section 3)

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_mpc/include/.../formulation.hpp`, `src/formulation.cpp` (`SidePicker`; posture target from the picked pose), `include/.../references.hpp`, `src/references.cpp` (`PostureTargets::tight` vector, `posture_targets(..., tight_names)`), `include/.../collision.hpp`, `src/collision.cpp` (`clearance_per_stage`), `src/controller.cpp` (pick each cycle; `CycleOutput::posture_choice`, `probe_scores`), `src/wholebody_mpc_node.cpp` (parameter `tight_poses`; cycle fields; `arm_trace` sample gains `choice`), `config/wholebody_mpc.yaml` (`posture.tight_poses: [vertical_carry, vertical_carry_right]`), `ros2_ws/src/mobile_manipulator_interfaces/msg/MpcCycle.msg` (`string posture_choice`, `float64[] probe_scores`), `ros2_ws/src/mobile_manipulator_navigation/launch/navigation.launch.py` (`tight_poses` in `MPC_OVERRIDES`), `tools/arm_trace.py` (+ `tools/test_arm_trace.py`)
- Test: `test/test_formulation.cpp`, `test/test_references.cpp`, `test/test_controller.cpp`

**Interfaces:**
- Produces:
  - `std::vector<double> CollisionModel::clearance_per_stage(const std::vector<State> & nominal, const Eigen::Matrix<double,6,1> & joints, double dt, double now, const World & world) const` (per stage, capped at `activation_m`; `clearance_with_joints` = its minimum).
  - `enum class PostureChoice { None, Comfort, Tight0, Tight1 };` (`to_string`: "none", "comfort", "left", "right" for the configured order `[vertical_carry, vertical_carry_right]`).
  - `class SidePicker { PostureChoice update(double alpha, const std::vector<double> & comfort, const std::vector<std::vector<double>> & tight, double shoulder_pan, double route_turn_sign); bool needs_probe(double alpha) const; PostureChoice choice() const; };` with α_on 0.05, α_off 0.02, comfort 0.30 m, guard/choice 0.02 m, pan 0.1 rad.
  - `Inputs::posture_choice` used by `Formulation::build`: target `(1-α) comfort + α tight[i]` for `Tight0/1`, the comfort pose for `Comfort` or `None`.

- [ ] **Step 1: Write the failing tests:** `SidePicker.NoProbeBelowAlphaOn`; `...LargerScoreWins` (tight scores 0.20 / 0.12 at k*, comfort 0.05: Tight0); `...GuardKeepsComfortUnderABeam` (comfort 0.10, both tight 0.06: Comfort); `...ScoresTheFirstConflictWindowOnly` (Tight0 better at k*-1..k*+1, worse later: Tight0); `...TieKeepsTheCurrentPanSide` (equal scores, pan -1.2 rad: Tight1); `...TieFollowsTheRouteTurn` (equal, pan 0, turn sign -1: Tight1); `...TieDefaultsLeft`; `...LatchesUntilAlphaOff` (scores swap while α 0.5: unchanged; α 0.019: None); `References.TightPosesResolveInOrder` and `...UnknownTightPoseThrowsByName`; closed loops in `test_controller.cpp`: `RightSwing.TallBoxLeftOfTheGateSwingsRight` (1.05 m gate, tall box 0.15 m left of the gate's approach side, start `carry`: pan < -1.2 rad at the gate, passes, closest >= 0.049 m), `RightSwing.SymmetricGateStillSwingsLeft` (pan > 1.2 rad), `Guard.LowBeamKeepsComfortAndLowers` (beam bottom 1.27 m, nothing beside it: choice Comfort, passes with closest >= 0.049 m, panel top under the beam < 1.27 - 0.049), `Guard.GateThenBeamStillPicksVerticalForTheGate` (gate, beam 1.5 m after it at 1.70 m: Tight0 at the gate).
- [ ] **Step 2: Run** the mpc tests. Expected: FAIL.
- [ ] **Step 3: Implement** as in the Interfaces block and Section 3: k* = first stage with comfort clearance < 0.30 m (none: keep the choice `None` and α's blend target is the comfort pose); score = min over stages k*-1..k*+1 clamped to the horizon; one-entry `tight_poses` reproduces the left-only B5 (picker eligible set of one, same guard).
- [ ] **Step 4: Run** `colcon test --packages-select mobile_manipulator_interfaces mobile_manipulator_mpc mobile_manipulator_navigation` and `python3 -m pytest tools/test_arm_trace.py`. Expected: PASS; existing controller tests unchanged.
- [ ] **Step 5: Commit:** `git commit -m "Pick the swing side by clearance and keep carry under low obstacles"`.

### Task 9: Scenario `v2_gate_right_swing_nav` (Section 6)

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_navigation/config/scenarios_site_v2.yaml`, `docs/experiments/site-v2/plan-expectations.yaml`, `docs/experiments/site-v2/README.md` (scenario list)

- [ ] **Step 1: Place** in the delivery yard a 1.05 m gate of two scenario boxes (0.2 x 0.2 x 2.0 m posts) on a straight 8 m route and a tall box (2.0 x 0.3 x 2.0 m) along the left of the approach; choose its lateral offset with `carry_pose_search --eval` of both vertical poses at the approach base poses so the probed clearance differs by at least 0.10 m in favour of the right swing. `arm_pose: carry`, `footprint_profile: carry`, `timeout_s: 120`, description stating "Expected - B5 swings the panel right (pan -90 degrees); B1-B4 fail to plan (carry wider than the gate)".
- [ ] **Step 2: Run** the plan-only check `python3 tools/check_site_plans.py --scenario v2_gate_right_swing_nav` after adding the expectation (carry false, vertical_carry true). Expected: matches.
- [ ] **Step 3: Live:** one B5 run. Expected: succeeds, `posture_choice` right at the gate (from `wholebody_mpc/cycle` in the bag), no contact. Stop Play.
- [ ] **Step 4: Commit:** `git commit -m "Add a gate that needs the right swing"`.

### Task 10: Lag compensation (Section 5, step 2a) or checkpoint (2b)

**If Task 2's verdict is "not explained": stop, report Task 2's table to the user with the next candidates (Section 5 step 2b), and skip to Task 11 only on the user's decision.**

**Files (2a):**
- Modify: `include/.../controller.hpp`, `src/controller.cpp`, `src/wholebody_mpc_node.cpp`
- Test: `test/test_controller.cpp`

**Interfaces:**
- Produces: `void Controller::set_measured_arm(const Eigen::Matrix<double,6,1> & q)` (called by the node before `step()` with the measured joints; the state passed to `step()` stays the reference). In `step()`, after `rows()`: for each distinct (primitive, link, obstacle) of a non-base, non-floor, non-self row, `offset = clamp(pair_distance(x_ref) - pair_distance(x_meas), 0, 0.02)`, and every row of that pair gets `margin += offset`.

- [ ] **Step 1: Write the failing test** `LagCompensation.KeepsTheMarginWithALaggingArm` (the closed-loop plant applies the commanded arm with a 0.012 rad lag on lift and elbow past the 1.70 m beam; with compensation closest >= 0.049 m; with `set_measured_arm` never called (compensation off) closest < 0.049 m).
- [ ] **Step 2: Run** it. Expected: FAIL.
- [ ] **Step 3: Implement** as in the Interfaces block; the node passes `state.x.tail<6>()` (measured) each cycle.
- [ ] **Step 4: Run** `colcon test --packages-select mobile_manipulator_mpc`. Expected: PASS.
- [ ] **Step 5: Live:** `overhead_course_nav --arm-pose home` and `constrained_reconfiguration_mission`, one run each; `clearance_3d` on both. Expected: >= 0.049 m, times within a few percent of Task 3's. Stop Play.
- [ ] **Step 6: Commit:** `git commit -m "Compensate the arm lag in B5's arm and panel rows"`.

### Task 11: Exit campaign (Section 8)

**Files:**
- Create: `docs/experiments/wholebody-mpc/phase4b-a.md` (results; linked from the README)

- [ ] **Step 1: Run** under `caffeinate -i`, clean tree, 50 Hz, defaults: V1 `overhead_course_nav --arm-pose home`, `overhead_course_worker_nav`, `constrained_reconfiguration_mission`, `constrained_reconfiguration_mission --references scripted`, `narrow_gate_home_nav --arm-pose carry`, 3 each (15); V2 all 20 scenarios once; `v2_gate_right_swing_nav` 3 times. Stop Play at the end. *(As run, 2026-10-22: the final campaign also ran R4 and store S twice more, the two cells that failed earlier, on the fix-round code `ae3f640`; see docs/experiments/wholebody-mpc/phase4b-a.md.)*
- [ ] **Step 2: Score** each run (`summarize_nav_runs.py`, `clearance_3d`, `arm_trace.py`, cycle statuses): outcome, time, contact, 3D clearance, posture choice per tight section, blocked events, MPC p99/misses/CPU.
- [ ] **Step 3: Check the exit criteria** of Section 8 one by one in the results file: no contact; V2 19/20 with only `v2_worker_late_crossing_nav` failing and R4 + store S succeeding; each V1 cell at least the recorded B5's success; >= 0.045 m (amended: Task 10 ran 2b; slack_inside x30); right swing 3/3 and the symmetric gate left 3/3; no `blocked` in successful runs. **Any failed criterion: find the cause and report it to the user before Task 12; criteria are not relaxed without the user.**
- [ ] **Step 4: Commit:** `git commit -m "Record the Phase 4B-A exit campaign"`.

### Task 12: Documentation, roadmap and freeze (Section 10)

**Files:**
- Modify: `docs/roadmap.md`, `docs/adr/0011-wholebody-mpc-command-ownership.md`, `docs/adr/0010-dynamic-footprint-ownership.md`, `docs/running-the-stack.md`, `docs/experiments/wholebody-mpc/README.md`

- [ ] **Step 1: Write** the roadmap Phase 4B-A outcome section; resolve the open decisions "The slowdown zone and the B4/B5 comparison" and "3D clearance below the 0.05 m margin" (with Task 2/10's outcome); move "B5 does not yield to a crossing worker" to Phase 5 as a documented limitation; add the comparability-log rows (StopZone 0.02 m and SlowdownZone off in wholebody mode, base margin, executed-command feedback, stop ramp, side picker and guard, lag compensation if any); add Phase 5 notes (a)-(d) of Section 10.
- [ ] **Step 2: Write** the ADR 0011 amendment and the ADR 0010 note (Section 2 "ADR notes").
- [ ] **Step 3: Update** `running-the-stack.md`: `tight_poses`, the `blocked` parameters, the `slowdown_zone` default for wholebody, and the recorded-B5 reproduction (`--comfort-pose home --no-slack-gate --mpc-rate-hz 50`, `slowdown_zone:=true`, `tight_poses:=[vertical_carry]`, and the StopZone margin note).
- [ ] **Step 4: Commit** `git commit -m "Document Phase 4B-A and freeze B5"`, then tag the commit the campaign ran on: `git tag b5-frozen-<YYYY-MM-DD> <campaign commit>` (do not push; tell the user).

### Task 13: Low-pass addendum (Section 10A, after the freeze)

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_mpc/test/test_controller.cpp` (a disabled-by-default sweep test `LowPass.Sweep`, run with `--gtest_also_run_disabled_tests`), `ros2_ws/src/mobile_manipulator_navigation/config/scenarios_site_v2.yaml`, `docs/experiments/site-v2/plan-expectations.yaml`, `docs/experiments/wholebody-mpc/phase4b-a.md` (low-pass section)

- [ ] **Step 1: Offline sweep:** `DISABLED_LowPass.Sweep` closed loop (generated model plant, a 3 m wide beam box across an 8 m straight route, start `carry`, frozen defaults) for beam bottoms 1.30 down to 0.80 m in 0.01 m steps; print per beam: passed, minimum clearance, the arm configuration and panel top under the beam, shoulder and elbow gravity torque (the search tool's torque model) and whether it exceeds the qualified carry/level_extension envelope. Record the lowest pass of each regime (folded: panel centre within the home footprint; extended: centre forward of it). No B5 code changes in this task.
- [ ] **Step 2: Add** `v2_low_beam_fold_nav` (beam bottom 1.30 m) and `v2_low_beam_extend_nav` (the extended regime's lowest pass + 0.03 m) in an open V2 area (scenario boxes with `z`, start `carry`), plan-only expectations (the 2D band does not see the beam: plans succeed).
- [ ] **Step 3: Live:** 3 runs each; Unity screenshots (or a recording) of each pass at the beam; report outcome, clearance, arm configuration, and, for the extended pass, joint torques against the qualified envelope and the speed at which it would stay inside (if exceeded). Stop Play.
- [ ] **Step 4: Show** the screenshots and numbers to the user; commit: `git commit -m "Demonstrate B5 low passes folded and extended"`.
