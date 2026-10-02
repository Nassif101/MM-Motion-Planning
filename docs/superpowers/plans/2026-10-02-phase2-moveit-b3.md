# Phase 2 MoveIt B3 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add baseline B3: the base stops, MoveIt plans and executes a collision-free panel reconfiguration through a `ReconfigurePanel` action, Nav2's footprint switches between named profiles at standstill, and scripted mission scenarios measure it.

**Architecture:** A MoveIt config package plans the 6-DOF `arm` group against a known collision world (exported Unity boxes, scenario obstacles, raised floor) with the panel attached to `tool0`. A C++ action server wraps plan → verify footprint containment → execute → verify → publish the footprint. A C++ mission task sequences `NavigateToPose` and `ReconfigurePanel` steps from `scenarios.yaml`; the existing Python runner launches everything.

**Tech Stack:** ROS 2 Jazzy, MoveIt 2.12.4 (OMPL RRTConnect, Ruckig, KDL), ros2_control JTC and `mock_components/GenericSystem`, Nav2, C++17 + gtest, pytest/launch_testing, Unity 6 Editor C# (Pipeline `CliCommand`, EditMode tests).

**Spec:** `docs/superpowers/specs/2026-10-02-phase2-moveit-b3-design.md`

## Global Constraints

- Branch `moveit-arm`; one commit per task; never commit to `main`.
- Runtime nodes and ROS tasks in C++; launch files, tests and `tools/` may be Python.
- Only Nav2 (via the collision monitor) publishes `/cmd_vel`; no new node publishes base commands or sends Nav2 goals except the scenario runner/mission task.
- Arm execution only through `/arm_controller/follow_joint_trajectory`; joint order `shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint`.
- Acceptance limits: path error 0.15 rad, hold error 0.06 rad, panel ground clearance 0.15 m, base tilt 3 deg, no panel penetration.
- Base stopped: `/odom` linear speed < 0.01 m/s and yaw rate < 0.02 rad/s for 0.5 s.
- Velocity and acceleration scaling 0.5 with the panel attached; acceleration/jerk limits from the design-ledger "Arm motion-limit contract" table.
- Floor collision object top at z = 0.15 m in `map`; floor may touch `base_link`, the four `*_wheel_link`s and `arm_mount_link` only.
- Panel: 1.20 × 1.20 × 0.04 m box, centre and size from `qualified_payload.json` (`com_tool_ros_m`, `dimensions_tool_ros_m`), attached to `tool0`, touch links `tool0` and `wrist_3_link` only, subframe `panel` at its centre.
- No host-absolute paths, Unity `Library/`, or ROS `build/ install/ log/` in commits; shell scripts LF.
- Stop Unity Play mode when a task that used it finishes.

Container commands below use `C='docker exec mm-motion-planning-ma-robot-sim-1 bash -ic'`; `$ROS_WS` is the workspace inside the container and `cb` is the project build function.

### Plan-level decisions not spelled out in the spec

- The SRDF self-collision block is generated headlessly with the Setup Assistant's own library (`moveit_setup::srdf_setup::computeDefaultCollisions`) rather than through its GUI.
- IK fallback is TRAC-IK (already in the container) instead of pick_ik (not installed).
- Scenario obstacles gain an optional per-obstacle `clearance_m` (default 0.3) so the constrained-reconfiguration box can stand beside the start pose.
- The mock stack places `base_footprint` with a static `odom -> base_footprint` from a `base_pose` argument, so offline checks run at scenario poses.

## Review Focus

1. **Simulation epoch or teleport during a server's lifetime** (`/clock` goes backwards): the base-stopped window must reset and report "not stopped" until 0.5 s of new samples — test in Task 4 (`BaseMotionWindow.ResetsWhenTimeGoesBackwards`).
2. **Unity stream pause right after execution:** the measured-state containment check must use a `/joint_states` sample stamped after the trajectory end, not a cached pre-pause one — test in Task 4 (`FreshState.RejectsSamplesBeforeTrajectoryEnd`).
3. **move_group missing or stalled:** the server must return `PLANNING_FAILED` within its deadline, never hang the mission — test in Task 4 mock test `test_planning_deadline_without_move_group`.
4. **Costmap misses the footprint update** (late subscription, Nav2 relaunch): the mission task must confirm both `published_footprint` topics match the requested polygon before the next drive — test in Task 6 (`FootprintMatch.PaddingAndOrderTolerant`, `FootprintMatch.RejectsOtherProfile`).
5. **Cancel during planning (before any motion):** returns `CANCELED`, joints unchanged, footprint unchanged — test in Task 4 mock test `test_cancel_during_planning`.

---

### Task 1: MoveIt configuration and mock arm stack

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_moveit_config/{package.xml,CMakeLists.txt}`
- Create: `ros2_ws/src/mobile_manipulator_moveit_config/config/{mobile_manipulator.srdf,kinematics.yaml,joint_limits.yaml,ompl_planning.yaml,moveit_controllers.yaml}`
- Create: `ros2_ws/src/mobile_manipulator_moveit_config/launch/{move_group.launch.py,mock_stack.launch.py}`
- Create: `ros2_ws/src/mobile_manipulator_moveit_config/src/generate_collision_matrix.cpp`
- Create: `ros2_ws/src/mobile_manipulator_moveit_config/test/test_moveit_config.py`
- Modify: `ros2_ws/src/mobile_manipulator_control/launch/arm_control.launch.py` (add `use_mock_hardware`)

**Interfaces:**
- Produces: `ros2 launch mobile_manipulator_moveit_config move_group.launch.py use_sim_time:=<bool>`; `ros2 launch mobile_manipulator_moveit_config mock_stack.launch.py base_pose:="x,y,yaw"` (robot_state_publisher, static `map->odom`, static `odom->base_footprint` at `base_pose`, arm control on mock hardware publishing `/joint_states`, `use_sim_time:=false`); planning group `arm`; named states `home`, `vertical_carry`, `level_extension`; virtual joint `planar` `map -> base_footprint`; move_group capabilities include `move_group/ClearOctomapService`.

- [ ] **Step 1: Write the failing config tests** in `test/test_moveit_config.py` (pytest, registered with `ament_add_pytest_test`):
  - `test_named_states_match_qualified_payload`: each SRDF `group_state` of group `arm` equals `qualified_payload.json["poses_rad"][name]` (1e-9) in `joint_order`, for `home`, `vertical_carry`, `level_extension`.
  - `test_group_joints_match_controller_order`: `arm` chain base `arm_mount_link`, tip `tool0`; joint list equals `controllers.yaml` `arm_controller.joints`.
  - `test_virtual_joint_is_planar_map_to_base_footprint`.
  - `test_joint_limits_match_design_ledger`: parse the "Arm motion-limit contract" markdown table in `mobile_manipulator_description/docs/design-ledger.md`; for each joint `max_acceleration` and `max_jerk` equal the table, `has_acceleration_limits` and `has_jerk_limits` true; `default_velocity_scaling_factor == 0.5`, `default_acceleration_scaling_factor == 0.5`.
  - `test_disabled_collisions_only_name_real_links`: every `disable_collisions` pair names URDF links with collision geometry; no pair is `(upper_arm_link|forearm_link|wrist_*_link, base_link)` unless the reason is `Never` from the generator.
  - `test_pipeline`: `ompl_planning.yaml` default planner config for `arm` is `RRTConnectkConfigDefault`; response adapters include Ruckig smoothing (`default_planning_response_adapters/AddRuckigTrajectorySmoothing`); `moveit_controllers.yaml` maps `arm_controller` to `FollowJointTrajectory` action ns `follow_joint_trajectory` with the six joints.

- [ ] **Step 2: Run, expect failures** — `$C 'cd $ROS_WS && cb --packages-select mobile_manipulator_moveit_config && colcon test --packages-select mobile_manipulator_moveit_config && colcon test-result --verbose'` → FAIL (files missing).

- [ ] **Step 3: Write `generate_collision_matrix.cpp`** — executable `generate_collision_matrix --urdf FILE --srdf FILE --trials 10000` that builds a `planning_scene::PlanningScene` from URDF + SRDF (without disable entries), calls `moveit_setup::srdf_setup::computeDefaultCollisions(scene, nullptr, true, trials, 0.95, false)` and prints `<disable_collisions link1=… link2=… reason=…/>` lines. Link against `moveit_setup_srdf_plugins`, `moveit_core`.

- [ ] **Step 4: Write the config files.** SRDF: group, named states, virtual joint, end effector none; paste the generator output as the `disable_collisions` block. `kinematics.yaml`: `kdl_kinematics_plugin/KDLKinematicsPlugin`, timeout 0.05 s. `joint_limits.yaml`: values from the ledger table, scaling 0.5. `move_group.launch.py`: build config with `moveit_configs_utils.MoveItConfigsBuilder("mobile_manipulator", package_name="mobile_manipulator_moveit_config")`, robot description from `mobile_manipulator_description/urdf/mobile_manipulator.urdf`, `publish_robot_description_semantic=True`, capabilities param includes `move_group/ClearOctomapService`. `arm_control.launch.py`: `use_mock_hardware` (default false) switches the hardware plugin to `mock_components/GenericSystem` (initial positions = `home`), removes the Unity params, overrides `use_sim_time` false and `arm_joint_state_broadcaster.use_local_topics` false.

- [ ] **Step 5: Run the config tests** — same command → all PASS.

- [ ] **Step 6: Smoke-check move_group on mock hardware** — start `mock_stack.launch.py base_pose:="12.0,0.0,3.141593"` and `move_group.launch.py use_sim_time:=false`, then `ros2 service call /check_state_validity moveit_msgs/srv/GetStateValidity "{group_name: arm, robot_state: {is_diff: true}}"` → `valid: true`. Stop both.

- [ ] **Step 7: Commit** — `git add ros2_ws/src/mobile_manipulator_moveit_config ros2_ws/src/mobile_manipulator_control/launch/arm_control.launch.py && git commit -m "Add the MoveIt configuration for the arm and a mock arm stack"`

---

### Task 2: Footprint projection library

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_navigation/include/mobile_manipulator_navigation/footprint_projection.hpp`
- Create: `ros2_ws/src/mobile_manipulator_navigation/src/footprint_projection.cpp`, `src/panel_pose_main.cpp`
- Create: `ros2_ws/src/mobile_manipulator_navigation/test/test_footprint_projection.cpp`
- Modify: `ros2_ws/src/mobile_manipulator_navigation/CMakeLists.txt` (library `footprint_projection` linking `lidar_robot_filter`, executable `panel_pose`, gtest)

**Interfaces:**
- Consumes: `Primitive`, `Payload`, `load_primitives(urdf_text, payload)` from `lidar_robot_filter.hpp`; `Polygon` from `scenario_spec.hpp`.
- Produces (namespace `mobile_manipulator_navigation`):
  - `using JointMap = std::map<std::string, double>;`
  - `Payload payload_from_json(const nlohmann::json & qualified_payload);` (link `tool0`)
  - `class FootprintProjector { public: FootprintProjector(const std::string & urdf_text, const Payload & payload); Eigen::Isometry3d link_pose(const std::string & link, const JointMap & joints) const; Eigen::Isometry3d panel_pose(const JointMap & joints) const; std::vector<std::array<double, 2>> projected_points(const JointMap & joints) const; };` — poses are `base_footprint -> X`; unlisted joints at zero; boxes contribute 8 corners, cylinders a circumscribed 64-gon at both ends (same as `test_footprint_profiles.py`).
  - `struct Containment { bool inside; double margin_m; };` and `Containment contains(const Polygon & convex_polygon, const std::vector<std::array<double, 2>> & points);` — `margin_m` is the minimum signed distance from any point to the polygon edges, positive inside.
  - Executable `panel_pose --pose NAME` printing `{"xyz":[…],"rpy":[…]}` of `panel_pose` for a qualified pose (used to write mission goals).

- [ ] **Step 1: Write failing gtests** in `test_footprint_projection.cpp`:
  - `Projection.NamedProfilesContainTheirPoseWithTheAllowance`: for `home` and `vertical_carry`, `contains(profile polygon, projected_points(pose))` is inside with `0.019 <= margin_m <= 0.026`.
  - `Projection.LevelExtensionDoesNotFitHome`: `inside == false`, `margin_m < 0`.
  - `Projection.PanelOrientation`: at `home` `|panel_pose.linear()(2,2)| > 0.999`; at `vertical_carry` `|panel_pose.linear()(2,2)| < 1e-6`.
  - `Containment.UnitSquare`: square `[[1,1],[1,-1],[-1,-1],[-1,1]]`; points `{0,0}` → inside, margin 1.0; `{1.5,0}` → outside, margin -0.5.

- [ ] **Step 2: Run, expect compile failure** — `$C 'cd $ROS_WS && cb --packages-select mobile_manipulator_navigation && colcon test --packages-select mobile_manipulator_navigation --ctest-args -R test_footprint_projection && colcon test-result --verbose'`.

- [ ] **Step 3: Implement** `footprint_projection.cpp` (URDF joint chain walk with `urdf::Model`, revolute/continuous rotation about the joint axis) and `panel_pose_main.cpp`.

- [ ] **Step 4: Run the tests** — same command → PASS; also run the full package test suite → PASS.

- [ ] **Step 5: Commit** — `git commit -m "Add the footprint projection and containment library"`

---

### Task 3: Known collision world — Unity box export and planning-scene loader

**Files:**
- Create: `motion-planning-sim/Assets/Scripts/Runtime/Environment/PlanningBoxGeometry.cs`
- Create: `motion-planning-sim/Assets/Scripts/Editor/PlanningBoxExporter.cs`
- Create: `motion-planning-sim/Assets/Tests/EditMode/PlanningBoxGeometryTests.cs`
- Create (generated): `ros2_ws/src/mobile_manipulator_navigation/maps/construction_site.boxes.yaml`
- Create: `ros2_ws/src/mobile_manipulator_navigation/test/test_planning_boxes.py`
- Create: `ros2_ws/src/mobile_manipulator_manipulation/{package.xml,CMakeLists.txt}`
- Create: `ros2_ws/src/mobile_manipulator_manipulation/include/mobile_manipulator_manipulation/scene_diff.hpp`, `src/scene_diff.cpp`, `src/planning_scene_loader.cpp`
- Create: `ros2_ws/src/mobile_manipulator_manipulation/test/test_scene_diff.cpp`
- Modify: `ros2_ws/src/mobile_manipulator_navigation/src/scenario_spec.cpp` and `test/test_scenario_spec.cpp` (optional per-obstacle `clearance_m`)

**Interfaces:**
- Produces: `PlanningBoxGeometry.UnityBoundsToRos(Bounds unityWorldBounds) -> (Vector3 centreRos, Vector3 sizeRos)` with ROS x = Unity z, y = −Unity x, z = Unity y (sizes permuted the same way, unsigned).
- Produces: Unity commands `export_planning_boxes` and `validate_planning_boxes` (same source hierarchy and enabled/non-trigger filter as `Nav2MapExporter`; non-box colliders use world bounds). File format:
  ```yaml
  frame_id: map
  source_scene: Assets/Scenes/ConstructionSiteV1.unity
  boxes:
    - {name: <hierarchy path>, center: [x, y, z], size: [sx, sy, sz]}
  ```
- Produces (namespace `mobile_manipulator_manipulation`): `struct SceneInputs { std::vector<Box> static_boxes; std::vector<Box> scenario_boxes; double floor_top_m = 0.15; double floor_size_m = 40.0; Payload panel; };` with `struct Box { std::string name; Eigen::Vector3d center, size; };` and `moveit_msgs::msg::PlanningScene build_scene_diff(const SceneInputs &);` — world objects in `map`, floor id `floor`, scenario boxes from `{x, y, size_x, size_y, height}` standing on z = 0, attached object id `panel` on `tool0` with `touch_links {tool0, wrist_3_link}` and subframe `panel` at identity, ACM entries allowing `floor` with exactly `base_link, front_left_wheel_link, front_right_wheel_link, rear_left_wheel_link, rear_right_wheel_link, arm_mount_link`.
- Produces: node `planning_scene_loader` (params `boxes_file`, `scenario` (empty = none), `include_scenario_obstacles` (default true), `payload_file`) that calls `/apply_planning_scene` once at startup and exits successfully on `success: true`.

- [ ] **Step 1: Write the failing EditMode test** `PlanningBoxGeometryTests.UnityBoundsMapToRosAxes`: bounds centre (1, 2, 3), size (0.5, 1.0, 2.0) → centreRos (3, −1, 2), sizeRos (2.0, 0.5, 1.0).
- [ ] **Step 2: Run EditMode tests via the Unity CLI** (unity-cli skill), filter `PlanningBoxGeometryTests` → FAIL; implement `PlanningBoxGeometry`; rerun → PASS.
- [ ] **Step 3: Implement the export/validate commands; run** `unity command export_planning_boxes --project-path ./motion-planning-sim` then `unity command validate_planning_boxes --project-path ./motion-planning-sim` → validation reports the committed file matches.
- [ ] **Step 4: Write `test_planning_boxes.py`**: every box centre's map cell in `construction_site.pgm` is occupied, and every box has positive size and z-centre ≥ 0 → PASS against the generated file.
- [ ] **Step 5: Write failing gtests** `test_scene_diff.cpp`:
  - `SceneDiff.FloorTopAtClearanceHeight`: floor box pose z + size_z/2 == 0.15.
  - `SceneDiff.PanelAttachedWithSubframe`: one attached object, link `tool0`, touch links exactly `{tool0, wrist_3_link}`, primitive size (1.2, 1.2, 0.04), subframe names `{panel}`.
  - `SceneDiff.FloorAllowedOnlyForBaseLinks`: ACM `floor` entries true exactly for the six links.
  - `SceneDiff.ScenarioBoxesStandOnTheGround`: a `{x:1,y:2,size_x:0.6,size_y:0.6,height:0.8}` box → centre z 0.4.
  - `ScenarioSpec.ObstacleClearanceOverride` (in `test_scenario_spec.cpp`): an obstacle 0.1 m from the start footprint is rejected with default clearance and accepted with `clearance_m: 0.05`.
- [ ] **Step 6: Run, expect failures; implement `scene_diff.cpp`, `planning_scene_loader.cpp`, the `clearance_m` field; run** `$C 'cd $ROS_WS && cb --packages-up-to mobile_manipulator_manipulation && colcon test --packages-select mobile_manipulator_manipulation mobile_manipulator_navigation && colcon test-result --verbose'` → PASS.
- [ ] **Step 7: Verify on mock** — mock stack at `base_pose:="12.0,0.0,3.141593"`, move_group, loader; `/check_state_validity` for `home` → `valid: true`; for joints `[0, 1.745, 0, 0, 0, 0]` → `valid: false` with a contact between `panel` and `floor`.
- [ ] **Step 8: Commit** — `git commit -m "Export static collision boxes and load the known MoveIt planning scene"`

---

### Task 4: `ReconfigurePanel` action server

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_interfaces/{package.xml,CMakeLists.txt,action/ReconfigurePanel.action}`
- Create: `ros2_ws/src/mobile_manipulator_manipulation/include/mobile_manipulator_manipulation/reconfigure_logic.hpp`, `src/reconfigure_logic.cpp`
- Create: `ros2_ws/src/mobile_manipulator_manipulation/src/reconfigure_panel_server.cpp`, `src/reconfigure_panel_cli.cpp`
- Create: `ros2_ws/src/mobile_manipulator_manipulation/launch/manipulation.launch.py`
- Create: `ros2_ws/src/mobile_manipulator_manipulation/test/test_reconfigure_logic.cpp`, `test/test_reconfigure_mock.py`

**Interfaces:**
- Consumes: `FootprintProjector`, `contains`, `payload_from_json` (Task 2); `build_scene_diff` loader (Task 3); `move_group.launch.py`, `mock_stack.launch.py` (Task 1).
- Produces `ReconfigurePanel.action`:
  ```
  uint8 PANEL_POSE=0
  uint8 NAMED_STATE=1
  uint8 target_type
  geometry_msgs/PoseStamped panel_pose
  geometry_msgs/Vector3 position_tolerance
  geometry_msgs/Vector3 orientation_tolerance
  string named_state
  string footprint_profile
  float64 planning_time_s 5.0
  int32 planning_attempts 5
  ---
  uint8 SUCCESS=0
  uint8 BASE_NOT_STOPPED=1
  uint8 ARM_NOT_ACTIVE=2
  uint8 UNKNOWN_PROFILE=3
  uint8 NO_IK=4
  uint8 PLANNING_FAILED=5
  uint8 PROFILE_TOO_SMALL=6
  uint8 EXECUTION_FAILED=7
  uint8 ARM_FAULT=8
  uint8 PROFILE_VIOLATED_AFTER_EXECUTION=9
  uint8 CANCELED=10
  uint8 error_code
  string message
  float64[] reached_joint_positions
  geometry_msgs/Pose reached_panel_pose
  string applied_footprint_profile
  bool profile_violated
  float64 planned_containment_margin_m
  float64 measured_containment_margin_m
  float64 planning_time_s
  float64 execution_time_s
  float64 footprint_switch_time_s
  float64 trajectory_duration_s
  float64 joint_path_length_rad
  float64 min_planned_clearance_m
  float64 max_path_error_rad
  float64 hold_error_rad
  ---
  uint8 CHECKING=0
  uint8 PLANNING=1
  uint8 VERIFYING_PLAN=2
  uint8 EXECUTING=3
  uint8 VERIFYING_STATE=4
  uint8 SWITCHING_FOOTPRINT=5
  uint8 phase
  ```
- Produces (namespace `mobile_manipulator_manipulation`):
  - `moveit_msgs::msg::Constraints panel_goal_constraints(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Vector3 & position_tolerance, const geometry_msgs::msg::Vector3 & orientation_tolerance);` — link name `panel/panel`, box region of half extents `position_tolerance`, `OrientationConstraint` with `parameterization = XYZ_EULER_ANGLES` and the per-axis tolerances, weights 1.0.
  - `class BaseMotionWindow { public: void add(double t, double linear, double angular); bool stopped(double now, double window_s = 0.5, double v_max = 0.01, double w_max = 0.02) const; };` — clears itself when `t` is earlier than the last sample.
  - `bool fresh_after(const builtin_interfaces::msg::Time & sample, const builtin_interfaces::msg::Time & trajectory_end);`
- Produces: action server node `reconfigure_panel_server` on `/reconfigure_panel` (params `use_sim_time`, `payload_file`, `profiles_file`, `scene_source` default `known`, `octomap_settle_s` default 1.0). Flow as spec §4: preconditions (`BaseMotionWindow` from `/odom`; `arm_controller` active via `/controller_manager/list_controllers` and hardware component active via `list_hardware_components`); plan via `/move_action` with `plan_only: true` (named states → joint goal constraints from the SRDF state via `moveit::core::RobotModel`); `NO_IK` when MoveIt returns `NO_IK_SOLUTION` / `GOAL_CONSTRAINTS_VIOLATED` sampling failure, else `PLANNING_FAILED`; containment on the plan's last point; execute via `/execute_trajectory`; containment on the first `/joint_states` sample with `fresh_after`; publish `geometry_msgs/msg/Polygon` on `/global_costmap/footprint` and `/local_costmap/footprint` (reliable, transient local). Metrics: `min_planned_clearance_m` = minimum `distanceToCollision` over the planned trajectory points, against the scene fetched from `/get_planning_scene` (all components); `max_path_error_rad` = max absolute error on `/arm_controller/controller_state` during execution; `hold_error_rad` = max |measured − planned final| over 1.0 s after execution. Deadlines: planning `planning_time_s + 5 s`, execution `trajectory duration + 3 s + 5 s`. `ARM_FAULT` when execution aborts and the arm hardware is no longer active.
- Produces: CLI `reconfigure_panel` (`--named STATE | --panel-pose X Y Z R P Y --position-tolerance DX DY DZ --orientation-tolerance RX RY RZ`, `--profile NAME`, `--output FILE`), exit 0 on `SUCCESS`, writes the result as JSON.
- Produces: `manipulation.launch.py` (args `scenario`, `scene_source` = `known|octomap`, `use_sim_time` default true) → `move_group`, `planning_scene_loader`, `reconfigure_panel_server`.

- [ ] **Step 1: Write failing gtests** `test_reconfigure_logic.cpp`:
  - `PanelGoal.ConstraintsUseThePanelSubframe`: link name `panel/panel`, region box dimensions = 2 × tolerance, orientation tolerances copied, parameterization `XYZ_EULER_ANGLES`.
  - `BaseMotionWindow.StoppedAfterHalfSecondBelowThresholds`: samples 0.0–0.6 s at 0.005 m/s → stopped at 0.6; with one 0.02 m/s sample at 0.3 → not stopped at 0.6.
  - `BaseMotionWindow.YawRateCounts`: 0.03 rad/s → not stopped.
  - `BaseMotionWindow.ResetsWhenTimeGoesBackwards` (Review Focus 1): samples to t = 10 s, then a sample at t = 1 s → not stopped at 1.0 s; stopped at 1.5 s after continuous samples.
  - `FreshState.RejectsSamplesBeforeTrajectoryEnd` (Review Focus 2): sample stamp < end → false; ≥ end → true.
- [ ] **Step 2: Run, expect failures; implement `reconfigure_logic.cpp`; rerun** → PASS.
- [ ] **Step 3: Write the mock integration tests** `test_reconfigure_mock.py` (launch_testing; launches `mock_stack.launch.py base_pose:="12.0,0.0,3.141593"`, `manipulation.launch.py use_sim_time:=false`, a 20 Hz zero `/odom` publisher in the test, subscribers on both footprint topics):
  - `test_named_vertical_carry_succeeds`: `NAMED_STATE vertical_carry`, profile `vertical_carry` → `SUCCESS`; both footprint topics received the `vertical_carry` polygon; `measured_containment_margin_m >= 0`; `trajectory_duration_s > 0`, `joint_path_length_rad > 0`, `min_planned_clearance_m > 0`.
  - `test_panel_pose_goal_succeeds`: `PANEL_POSE` = `panel_pose --pose vertical_carry` output, tolerances (0.05, 0.05, 0.05) m and (0.1, 0.1, 0.1) rad, from `home` → `SUCCESS`; reached panel pose within tolerance.
  - `test_profile_too_small_does_not_move`: from `home`, `NAMED_STATE level_extension`, profile `vertical_carry` → `PROFILE_TOO_SMALL`; joint states unchanged (1e-3); no footprint message.
  - `test_unknown_profile`, `test_base_not_stopped` (publish 0.1 m/s), `test_arm_not_active` (`ros2 control set_controller_state arm_controller inactive`) → matching codes, no motion.
  - `test_cancel_during_planning` (Review Focus 5): cancel as soon as feedback phase is `PLANNING` → `CANCELED`, joints unchanged, no footprint message.
  - `test_cancel_during_execution`: cancel at phase `EXECUTING` → `CANCELED`; `profile_violated` equals `not contains(previous profile, measured state)`.
  - `test_planning_deadline_without_move_group` (Review Focus 3): kill move_group, send a goal with `planning_time_s` 1.0 → `PLANNING_FAILED` within 7 s wall time.
- [ ] **Step 4: Run, expect failures; implement server, CLI, launch; run** `$C 'cd $ROS_WS && cb --packages-up-to mobile_manipulator_manipulation && colcon test --packages-select mobile_manipulator_interfaces mobile_manipulator_manipulation && colcon test-result --verbose'` → PASS.
- [ ] **Step 5: Commit** — `git commit -m "Add the ReconfigurePanel action server"`

---

### Task 5: Re-qualify MoveIt-planned transitions in Unity

**Files:**
- Modify: `tools/run_arm_qualification.py` (suite `moveit`), `tools/analyze_arm_qualification.py` only if new case names need handling
- Create: `docs/experiments/moveit-arm/README.md` (section "Transition qualification")
- Modify (only if a transition fails): `mobile_manipulator_moveit_config/config/joint_limits.yaml` scaling factors and the matching expectation in `test_moveit_config.py`

**Interfaces:**
- Consumes: CLI `reconfigure_panel` (Task 4), `manipulation.launch.py` with `use_sim_time:=true`, Unity commands `arm_test_place_open`, `arm_test_record`, `arm_test_end`, `arm_test_snapshot`.

- [ ] **Step 1: Add suite `moveit`** to `run_arm_qualification.py`: place at the open test area, start arm control and `manipulation.launch.py`, then for each case record with `arm_test_record moveit-<case>-<n>` and run `reconfigure_panel` via `docker exec`: `home→vertical_carry` (named), `vertical_carry→home` (named), `home→vertical panel pose` (Task 4 panel goal), 5 repeats each; analyze with `analyze_arm_qualification.analyze`.
- [ ] **Step 2: Run** `python3 tools/run_arm_qualification.py --container mm-motion-planning-ma-robot-sim-1 --suite moveit --prefix moveit` with the scene in Play → every case within path error 0.15 rad, hold error 0.06 rad, base tilt 3 deg, panel ground clearance ≥ 0.15 m, no penetration. If any case fails, lower both scaling factors to 0.3, update the config test, rerun, and record both attempts.
- [ ] **Step 3: Write the qualification section** in `docs/experiments/moveit-arm/README.md` (table per case: max path error, hold error, tilt, clearance, planning time, trajectory duration; scaling used). Stop Play mode.
- [ ] **Step 4: Commit** — `git commit -m "Qualify MoveIt-planned panel transitions in Unity"`

---

### Task 6: Mission task and B3 scenarios

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_navigation/include/mobile_manipulator_navigation/navigate_run.hpp`, `src/navigate_run.cpp` (drive-and-measure logic extracted from `navigate_scenario_task.cpp`)
- Modify: `ros2_ws/src/mobile_manipulator_navigation/src/navigate_scenario_task.cpp` (use `navigate_run`)
- Create: `ros2_ws/src/mobile_manipulator_navigation/include/mobile_manipulator_navigation/mission.hpp`, `src/mission.cpp`, `src/mission_scenario_task.cpp`
- Create: `ros2_ws/src/mobile_manipulator_manipulation/src/transition_validity_check.cpp`
- Modify: `ros2_ws/src/mobile_manipulator_navigation/{CMakeLists.txt,package.xml}` (depend on `mobile_manipulator_interfaces`), `src/scenario_spec.cpp`, `test/test_scenario_spec.cpp`, `config/scenarios.yaml`
- Create: `ros2_ws/src/mobile_manipulator_navigation/test/test_mission.cpp`
- Modify: `tools/run_nav_scenario.py`, `tools/summarize_nav_runs.py`

**Interfaces:**
- Consumes: `/reconfigure_panel` action (Task 4), `panel_pose` CLI (Task 2), `manipulation.launch.py` (Task 4).
- Produces:
  - `Recorder` (the node class now inside `navigate_scenario_task.cpp`) moves to `navigate_run.hpp` unchanged.
  - `Json run_navigate(Recorder & node, const Pose2 & goal, const Polygon & footprint, double timeout_s, const Json & obstacles, const Json & movers);` returning the existing `task.json` drive fields; `navigate_scenario_task` output is byte-for-byte the same field set as before.
  - `struct MissionStep { enum class Kind { Navigate, Reconfigure } kind; Pose2 pose; Json reconfigure; };` `std::vector<MissionStep> parse_mission(const Json & scenario);` `std::vector<std::string> mission_problems(const StaticMap & map, const Json & scenario, const Json & profiles);` — every navigate pose free for the profile active there; the pose where each `reconfigure` happens free for both the profile before and after; every `footprint_profile` known; each reconfigure has exactly one of `named_state` / `panel_pose`.
  - `bool footprint_matches(const Polygon & published, const Polygon & expected, double padding, double tolerance = 0.005);` — vertex-order independent, compares against `expected` grown by `padding`.
  - Executable `mission_scenario_task --scenario NAME --output FILE` → JSON `{status, steps: [...], total_time_s, reconfigure_time_s, cpu_percent_of_core, ...}` (each reconfigure step carries the full action result; CPU/memory sampling includes `move_group`); after each reconfigure it waits ≤ 5 s for both `published_footprint` topics to satisfy `footprint_matches` and fails the mission otherwise.
  - Executable `transition_validity_check --from STATE --to STATE --samples 50` → samples the straight joint-space path (the qualified transition shape, via `home`) and calls `/check_state_validity`; prints the first invalid sample and contacts; exit 3 if any sample is invalid.
  - `scenarios.yaml` `task: mission` with `steps` (spec §5 format) and `timeout_s` per drive.

- [ ] **Step 1: Write failing gtests** `test_mission.cpp`:
  - `Mission.ParsesStepsInOrder` (navigate, reconfigure panel_pose, reconfigure named).
  - `Mission.RejectsReconfigureWithBothTargets` and `...WithUnknownProfile`.
  - `Mission.RejectsReconfigurePoseBlockedForTheWiderProfile`: a reconfigure pose free for `vertical_carry` but blocked for `home` (inside the 1.05 m gate throat, `[-7.2, -7.725, π]`) with a `vertical_carry → home` step → problem reported.
  - `FootprintMatch.PaddingAndOrderTolerant` (Review Focus 4): expected profile + 0.01 padding, vertices rotated → true.
  - `FootprintMatch.RejectsOtherProfile`: `home` vs `vertical_carry` → false.
  - Extend `ScenarioSpec.EveryScenarioIsConsistentAndStartsInFreeSpace` to accept `task == "mission"` and assert `mission_problems` is empty.
- [ ] **Step 2: Run, expect failures; extract `navigate_run`, implement `mission.cpp`, `mission_scenario_task.cpp`, `transition_validity_check.cpp`; rerun the navigation test suite** → PASS, including the unchanged navigate-task tests.
- [ ] **Step 3: Add the three scenarios to `scenarios.yaml`**, panel goals from `panel_pose --pose vertical_carry`, tolerances (0.05, 0.05, 0.05) m / (0.1, 0.1, 0.1) rad:
  - `narrow_gate_mission` (roadmap 5): start `[-2.2, -7.725, π]` in `home`; navigate `[-4.2, -7.725, π]`; reconfigure → `vertical_carry`; navigate `[-9.0, -7.725, π]`; reconfigure named `home` → `home`; navigate `[-11.0, -7.725, π]`.
  - `wide_gate_mission`: same pattern across `WideGate_1p30m` (line x = −3, centre y = −16.0): start `[2.0, -16.0, π]`, staging `[0.0, -16.0, π]`, after `[-4.8, -16.0, π]`, goal `[-6.8, -16.0, π]`.
  - `constrained_reconfiguration_mission`: start `[12.0, 0.0, π]` in `home`; reconfigure → `vertical_carry`; navigate `[9.0, 0.0, π]`; reconfigure named `home` → `home`. One tall box beside the start with `clearance_m` as needed.
  Where `mission_problems` reports a blocked pose, shift it along the route in 0.5 m steps and keep the 3.0 m approach / 1.8 m exit where possible.
- [ ] **Step 4: Place the constrained box** — on the mock stack at `base_pose:="12.0,0.0,3.141593"` with `scenario:=constrained_reconfiguration_mission`, adjust the box until `transition_validity_check --from home --to vertical_carry` exits 3 (naive path collides) while `home` and `vertical_carry` are valid and `reconfigure_panel --named vertical_carry --profile vertical_carry` succeeds in ≥ 9 of 10 attempts. Record the box and the check output.
- [ ] **Step 5: Extend the runner** — `run_nav_scenario.py`: `task: mission` launches `navigation.launch.py` with the first step's profile plus `manipulation.launch.py scenario:=<name>`, waits for `/reconfigure_panel`, runs `mission_scenario_task`, adds `/planning_scene` and `/joint_states` to the bag, records the run with `arm_test_record mission-<run>` / `arm_test_end` and analyzes base tilt and panel ground clearance with `analyze_arm_qualification.analyze`, stops MoveIt processes between runs; `summarize_nav_runs.py` reports per-mission totals and per-reconfiguration metrics (spec §5).
- [ ] **Step 6: Live check** — one `narrow_gate_mission` run with `--controller rpp` in Unity → mission `succeeded`; stop Play mode.
- [ ] **Step 7: Commit** — `git commit -m "Add the mission task and the B3 reconfiguration scenarios"`

---

### Task 7: B3 runs and documentation

**Files:**
- Modify: `docs/experiments/moveit-arm/README.md`
- Create: `docs/adr/0009-moveit-panel-reconfiguration-baseline.md`
- Modify: `docs/roadmap.md` (Phase 2 section, decision-log entry, open decision "CHOMP post-processing"), `ros2_ws/src/mobile_manipulator_description/docs/design-ledger.md` (SRDF and collision matrix exist; MoveIt perception input; correct the self-return note to the URDF/payload self-filter), `docs/running-the-stack.md` (MoveIt launch, `reconfigure_panel`, mission runs)

- [ ] **Step 1: Run** each of the three missions × `rpp`, `dwb`, `mppi` × 3 runs with `run_nav_scenario.py`; collect with `summarize_nav_runs.py`.
- [ ] **Step 2: Write the results** in `docs/experiments/moveit-arm/README.md`: per-mission success, total time, reconfiguration time; per-reconfiguration metrics; `narrow_gate_mission` time vs Phase 1 `narrow_gate_home_nav` detour time; the constrained-scenario validity-check evidence; failures with causes.
- [ ] **Step 3: Write ADR 0009 and the roadmap/ledger/running-the-stack updates.** The roadmap decision-log entry uses the Section 11 template; the CHOMP open decision names it as a B3 variant (OMPL + CHOMP post-processing) to compare on the same missions.
- [ ] **Step 4: Verify** links resolve (`grep -o '](.*)' ` targets exist) and the full test suite passes: `$C 'cd $ROS_WS && colcon test --packages-select mobile_manipulator_moveit_config mobile_manipulator_manipulation mobile_manipulator_navigation && colcon test-result --verbose'`. Stop Play mode.
- [ ] **Step 5: Commit** — `git commit -m "Record the B3 mission results and the Phase 2 design decisions"`

---

### Task 8: Octomap scene source

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_moveit_config/config/sensors_3d.yaml`
- Modify: `mobile_manipulator_moveit_config/launch/move_group.launch.py` (`scene_source`), `mobile_manipulator_manipulation/launch/manipulation.launch.py`, `src/reconfigure_panel_server.cpp`, `test/test_reconfigure_mock.py`, `test/test_moveit_config.py`
- Modify: `config/scenarios.yaml`, `tools/run_nav_scenario.py` (`--scene-source`), `docs/experiments/moveit-arm/README.md`, design ledger, roadmap log

**Interfaces:**
- Produces: `scene_source:=octomap` → `PointCloudOctomapUpdater` on `/livox/points_filtered`, `octomap_frame: map`, `octomap_resolution: 0.05`, `max_range: 5.0`; loader runs with `include_scenario_obstacles:=false`; the server calls `/clear_octomap` (`std_srvs/srv/Empty`) and waits `octomap_settle_s` (1.0 s, simulation time) at standstill before planning.

- [ ] **Step 1: Write failing tests** — `test_moveit_config.py::test_octomap_sensor` (topic, frame, resolution, range above); `test_reconfigure_mock.py::test_octomap_wall_blocks_goal`: in `scene_source:=octomap`, the test publishes a synthetic `PointCloud2` wall in `map` covering the `vertical_carry` panel volume → `NAMED_STATE vertical_carry` returns `PLANNING_FAILED` (or `NO_IK`); after the cloud stops and a new goal clears the Octomap, the same goal → `SUCCESS`.
- [ ] **Step 2: Run, expect failures; implement; rerun** → PASS.
- [ ] **Step 3: Add scenarios** `octomap_unmapped_box_mission` (an unmapped box beside the staging pose that the planned motion must avoid; it is not given to MoveIt) and `octomap_blind_zone_mission` (a low box inside the documented near-field blind zone, below 0.2 m within 1.5 m), both run only with `--scene-source octomap`.
- [ ] **Step 4: Run** both × `rpp` × 3 in Unity; record results, including whether the blind-zone box was perceived, in the experiment README; update the ledger and roadmap log. Stop Play mode.
- [ ] **Step 5: Commit** — `git commit -m "Add the lidar Octomap scene source for MoveIt"`
