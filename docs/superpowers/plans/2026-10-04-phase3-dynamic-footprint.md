# Phase 3 Dynamic Footprint (B4) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add baseline B4: a configuration-dependent convex-hull Nav2 footprint (Sagar et al., CoDIT 2026) published by a standalone node from a ROS-free geometry library, selectable with `footprint_mode:=dynamic`, regression-tested on the B3 missions, and compared against static footprints as in the paper's Tables II and III.

**Architecture:** A new package `mobile_manipulator_geometry` holds the robot model, ground projection, convex hull, exact outward offset and two footprint models (mesh, disc). `dynamic_footprint_node` (navigation package) computes the padded hull from `/joint_states` at 20 Hz and is the only publisher of both costmap footprints and the collision monitor zones in dynamic mode. `ReconfigurePanel` gains a dynamic mode that publishes nothing and checks the planned and measured hull against the global costmap.

**Tech Stack:** ROS 2 Jazzy, Nav2 1.3.13 (Smac Lattice, RPP, collision monitor), MoveIt 2, C++17 + gtest, pytest/launch_testing, Python tooling in `tools/`, Unity 6 Editor via Unity CLI.

**Spec:** `docs/superpowers/specs/2026-10-04-phase3-dynamic-footprint-design.md`

## Global Constraints

- Branch `dynamic-footprint`; one commit per task; never commit to `main`.
- Runtime nodes and ROS package code in C++; launch files, pytest config checks and `tools/` stay Python.
- Exactly one publisher of `/global_costmap/footprint`, `/local_costmap/footprint`, `/collision_monitor/stop_zone_in`, `/collision_monitor/slowdown_zone_in` per run: none (`static`), `reconfigure_panel_server` (`profiles`), `dynamic_footprint_node` (`dynamic`).
- Only Nav2 (via the collision monitor) publishes `/cmd_vel`; nothing new commands the base.
- Footprint padding `delta` = 0.02 m (B3 perimeter allowance); Nav2 `footprint_padding` 0.01 m stays on top.
- Node rate 20 Hz; publish threshold 0.01 m (symmetric Hausdorff); joint-state staleness 0.5 s.
- Mesh model cylinders: circumscribed 16-gon for footprints; the lidar filter keeps 64.
- Footprint polygons are in `base_footprint`, REP-103, metres, counter-clockwise.
- Arm joint order `shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint`.
- B3 behaviour (`footprint_mode` `profiles`, the default for missions) must not change; B3 tests pass unchanged.
- No host-absolute paths, credentials, Unity `Library/`, or ROS `build/ install/ log/` in commits; shell scripts LF.
- Long unattended Unity batches run under `caffeinate -i` (the runner holds it); stop Unity Play mode (`unity command editor_stop`) when a task that used it finishes.
- Do not commit the Sagar et al. PDF (IEEE licence).

Container commands use `C='docker exec mm-motion-planning-ma-robot-sim-1 bash -ic'`; `cb` is the project build function and `$ROS_WS` the workspace. Package tests: `$C 'cd $ROS_WS && colcon test --packages-select <pkgs> --event-handlers console_direct+ && colcon test-result --verbose'`.

### Plan-level decisions not spelled out in the spec

- **`enlarged` strategy = the existing `home` profile.** The envelope of all three qualified poses (with `level_extension`, x up to 1.22 m) has a 1.39 m circumscribed radius, past the 1.0 m inflation radius every baseline uses and `test_global_costmap_uses_default_profile_and_sufficient_inflation` enforces. The envelope of the two transport poses (`home` ∪ `vertical_carry`) is exactly `home`. So only `base_only` is a new profile. The spec's Section 6 is amended in Task 2's commit.
- **Disc radii come from the URDF, not hand calibration.** For link i the radius is the largest 3D distance from its collision points to the segment from its origin to the next selected link's origin (the last link: to its own origin), rounded up to 0.005 m. Joint i+1's origin is fixed in link i's frame, so this capsule encloses the link in every pose and its projection encloses the link's projection. Config can override a radius. The containment test still verifies it.
- **Run metrics keep the scenario's true profile.** `navigate_scenario_task --footprint-profile` stays the scenario's own profile (the real robot) in every footprint strategy. Only Nav2's footprint changes, so clearances are comparable across strategies.
- **The hull check takes the base pose from TF** (`map -> base_footprint`), not `/odom`; missing TF fails closed.
- **`FootprintDriftGuard` moves** from `mobile_manipulator_manipulation` to `mobile_manipulator_navigation` (`mission.hpp`), so the node and the server share it without a package cycle.
- **`FootprintWatch` moves** from `mission_scenario_task.cpp` into the `navigate_run` library, so a new `footprint_wait` tool can confirm a static override or a dynamic footprint before plan-only and navigate tasks.
- **"Each mode gives exactly one owner"** is tested as a pure function (`ownership_problems`) plus the live preflight, not by introspecting launch files.
- **Contact cancel applies to every run.** At the first Unity contact the runner cancels all `navigate_to_pose` goals. B1-B3 recorded no contact, so their results do not change.

## Review Focus

1. **Partial `/joint_states` messages** (a message without the arm joints, or with only some): the node must merge by name and never treat a missing arm joint as zero. Test in Task 4 (`DynamicFootprint.MergesPartialJointStates`, `DynamicFootprint.HoldsWhenAnArmJointIsMissing`).
2. **Arm hold jitter while driving** (a few mrad at a 1.3-1.5 m lever moves the panel corner by a few mm): it must not republish and trigger costmap re-inflation mid-drive. Test in Task 4 (`DynamicFootprint.IgnoresSubThresholdJitter`); Task 7 checks zero publishes during drives.
3. **Nav2 relaunched with the arm still** (the new costmap starts with the launch profile): the node must put its hull back. Test in Task 4 (`FootprintRepair.RepublishesAfterTwoSecondsAtStandstill`).
4. **No `map -> base_footprint` TF or no costmap at the hull check:** `HULL_IN_COLLISION` with a message, arm unmoved. Tests in Task 5 (`HullClearance.*` and mock `test_a_no_costmap_fails_closed`).
5. **A hull whose circumscribed radius exceeds the 1.0 m inflation radius** (an arbitrary arm pose, e.g. near `level_extension`): Smac's collision shortcut then misjudges. The node must warn and count it in the stats. Test in Task 4 (`DynamicFootprint.FlagsHullBeyondInflationRadius`).

---

### Task 1: Verify the Nav2 behaviours the design relies on

**Files:**
- Create: `docs/experiments/dynamic-footprint/README.md` (section "Nav2 prerequisites")

**Interfaces:**
- Produces: a recorded yes/no with source citations for the four behaviours; Tasks 4-8 build on them.

- [ ] **Step 1: Read the Nav2 1.3.13 source** (GitHub `ros-navigation/navigation2`, tag `1.3.13`, matching `dpkg -s ros-jazzy-nav2-costmap-2d`) and record file and line for each:
  1. `nav2_smac_planner/src/smac_planner_lattice.cpp` `createPlan` calls `_collision_checker.setFootprint(_costmap_ros->getRobotFootprint(), ...)` on every plan.
  2. `nav2_costmap_2d/plugins/inflation_layer.cpp` `onFootprintChanged` recomputes the inscribed and circumscribed radii and sets `need_reinflation_`.
  3. `nav2_collision_monitor/src/polygon.cpp`: the `polygon_sub_topic` callback accepts any polygon with at least 3 points (no rectangle assumption).
  4. `nav2_costmap_2d/src/footprint.cpp` `padFootprint`: each vertex gets `sign(x) * padding`, `sign(y) * padding`.
- [ ] **Step 2: Live probe of 1 and 2.** Enter Play, place the robot with `wide_gate_vertical_carry` (runner `--new-epoch`, plan-only), keep `global_planning.launch.py footprint_profile:=vertical_carry` running, then:
  - publish a hexagon 1.5 m wide on `/global_costmap/footprint` (`ros2 topic pub --once --qos-durability transient_local`);
  - confirm `/global_costmap/published_footprint` shows it padded within one costmap cycle;
  - send a `ComputePathToPose` through the 1.30 m gate (`gate_planning_check --label probe --planners Lattice`). Expected: the route no longer passes the gate.
  - Republish the `vertical_carry` polygon. Expected: the route passes the gate again.
- [ ] **Step 3: Check 4 numerically.** Pad the mesh hull of `home` (8-20 vertices, from Task 3's library or by hand) with the `padFootprint` rule and confirm it contains the unpadded hull. Record the result.
- [ ] **Step 4: Gate.** If any of 1-4 fails, stop and report to the user with the evidence before any further task.
- [ ] **Step 5: Stop Play** (`unity command editor_stop`) **and commit** `docs/experiments/dynamic-footprint/README.md`: "Record the Nav2 footprint behaviours B4 relies on".

---

### Task 2: Move the robot model into `mobile_manipulator_geometry` (no behaviour change)

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_geometry/{package.xml,CMakeLists.txt}`
- Create: `ros2_ws/src/mobile_manipulator_geometry/include/mobile_manipulator_geometry/{polygon.hpp,robot_model.hpp,footprint_projection.hpp}`
- Create: `ros2_ws/src/mobile_manipulator_geometry/src/{robot_model.cpp,footprint_projection.cpp}`
- Delete: `ros2_ws/src/mobile_manipulator_navigation/{src/footprint_projection.cpp,include/mobile_manipulator_navigation/footprint_projection.hpp}`
- Modify: `ros2_ws/src/mobile_manipulator_navigation/{CMakeLists.txt,package.xml}`, `include/mobile_manipulator_navigation/{lidar_robot_filter.hpp,scenario_spec.hpp}`, `src/{lidar_robot_filter.cpp,livox_robot_filter_node.cpp,panel_pose_main.cpp}`, `test/{test_footprint_projection.cpp,test_lidar_robot_filter.cpp}`
- Modify: `ros2_ws/src/mobile_manipulator_manipulation/{CMakeLists.txt,package.xml}`, `src/{reconfigure_logic.cpp,scene_diff.cpp,planning_scene_loader.cpp,reconfigure_panel_server.cpp}`
- Modify: `docs/superpowers/specs/2026-10-04-phase3-dynamic-footprint-design.md` Section 6 (the `enlarged` decision above)

**Interfaces:**
- Produces (namespace `mobile_manipulator_geometry`, alias `mmg`; CMake target `mobile_manipulator_geometry::geometry`, a static library linking Eigen3, urdf, nlohmann_json):
  - `polygon.hpp`: `using Point2 = std::array<double, 2>; using Polygon = std::vector<Point2>; struct Containment { bool inside; double margin_m; }; Containment contains(const Polygon &, const std::vector<Point2> &);`
  - `robot_model.hpp`: `enum class Shape`, `struct Primitive`, `struct Payload`, `std::vector<Primitive> load_primitives(const std::string & urdf_text, const std::optional<Payload> &)`, unchanged signatures.
  - `footprint_projection.hpp`: `using JointMap = std::map<std::string, double>; Payload payload_from_json(const nlohmann::json &); class FootprintProjector` (as today) plus `struct ProjectionOptions { int cylinder_sides = 64; std::set<std::string> links; /* empty = all */ bool payload = true; }` and the overload `std::vector<Point2> projected_points(const JointMap &, const ProjectionOptions &) const` (the existing one-argument overload forwards with defaults); `std::array<double, 3> rpy_of(const Eigen::Matrix3d &)`; `const std::vector<Primitive> & primitives() const`.
- `mobile_manipulator_navigation`: `scenario_spec.hpp` keeps `Polygon` as `using Polygon = mmg::Polygon;`; `lidar_robot_filter.hpp` keeps its ray functions and includes `robot_model.hpp`.

- [ ] **Step 1: Create the package and move the code verbatim**, changing only the namespace and includes. Call sites use `mmg::`. `footprint_projection` and the robot-model part of `lidar_robot_filter` link `mobile_manipulator_geometry::geometry`; remove the `footprint_projection` target from navigation's install/export and export the geometry dependency instead.
- [ ] **Step 2: Add the `ProjectionOptions` overload** (link filter, payload switch, cylinder sides) in `footprint_projection.cpp`; the default reproduces today's points exactly.
- [ ] **Step 3: Build and run every affected test:** `$C 'cb && cd $ROS_WS && colcon test --packages-select mobile_manipulator_geometry mobile_manipulator_navigation mobile_manipulator_manipulation --event-handlers console_direct+ && colcon test-result --verbose'`. Expected: all pass, including `test_footprint_projection`, `test_lidar_robot_filter`, `validate_footprint_profiles` and `test_reconfigure_mock.py`.
- [ ] **Step 4: Amend spec Section 6** ("`enlarged` = the `home` profile" with the 1.39 m vs 1.0 m reason) **and commit:** "Move the robot model and footprint projection into mobile_manipulator_geometry".

---

### Task 3: Convex hull, outward offset and the mesh and disc footprint models

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_geometry/include/mobile_manipulator_geometry/{polygon_ops.hpp,footprint_model.hpp}`, `src/{polygon_ops.cpp,footprint_model.cpp}`
- Create: `ros2_ws/src/mobile_manipulator_geometry/test/test_polygon_ops.cpp`
- Create: `ros2_ws/src/mobile_manipulator_navigation/config/dynamic_footprint.yaml`
- Create: `ros2_ws/src/mobile_manipulator_navigation/test/test_footprint_models.cpp`
- Create: `ros2_ws/src/mobile_manipulator_navigation/src/footprint_benchmark.cpp`
- Modify: both packages' `CMakeLists.txt`

**Interfaces:**
- Consumes: Task 2's `FootprintProjector`, `ProjectionOptions`, `Polygon`.
- Produces (`mmg`):
  - `Polygon convex_hull(const std::vector<Point2> & points);`: counter-clockwise, no collinear vertices; throws `std::invalid_argument` with fewer than 3 non-collinear points.
  - `Polygon offset_outward(const Polygon & convex_ccw, double delta);`
  - `double hausdorff(const Polygon & a, const Polygon & b);`: symmetric Hausdorff distance between the two boundaries.
  - `double area(const Polygon &); double circumscribed_radius(const Polygon &);` (max vertex distance from the origin).
  - `class FootprintModel { public: virtual ~FootprintModel() = default; virtual std::vector<Point2> points(const JointMap &) const = 0; Polygon footprint(const JointMap & joints, double delta) const; /* offset_outward(convex_hull(points(joints)), delta) */ };`
  - `class MeshHullModel : public FootprintModel { public: MeshHullModel(std::shared_ptr<const FootprintProjector>, int cylinder_sides = 16); };`
  - `struct DiscLink { std::string link; std::optional<double> radius_m; }; struct DiscModelConfig { std::vector<DiscLink> links; int disc_samples = 16; int segment_samples = 3; std::set<std::string> base_links; };`
  - `class DiscHullModel : public FootprintModel { public: DiscHullModel(std::shared_ptr<const FootprintProjector>, DiscModelConfig); const std::vector<double> & radii() const; };`
- `dynamic_footprint.yaml` (node parameters, used again in Task 4): `model: mesh`, `padding_m: 0.02`, `change_threshold_m: 0.01`, `rate_hz: 20.0`, `stale_after_s: 0.5`, `mesh.cylinder_sides: 16`, `disc.links: [shoulder_pan_link, upper_arm_link, forearm_link, wrist_1_link, wrist_2_link, wrist_3_link, tool0]`, `disc.disc_samples: 16`, `disc.segment_samples: 3`, `disc.base_links: [base_link, front_left_wheel_link, front_right_wheel_link, rear_left_wheel_link, rear_right_wheel_link, arm_mount_link]`.

- [ ] **Step 1: Write the failing pure tests** in `test_polygon_ops.cpp`:
  - `Hull.ContainsEveryPointConvexCcw`: 200 random points in [-1, 1]²; `contains(hull, points).inside`; every consecutive vertex triple turns left (cross > 1e-12).
  - `Hull.DropsCollinearAndDuplicates`: the unit square's corners plus edge midpoints and duplicates give exactly 4 vertices.
  - `Hull.ThrowsOnDegenerate`: 2 points, or 5 collinear points, throws `std::invalid_argument`.
  - `Offset.EveryEdgeAtDelta`: for a 1.24 × 0.04 m rectangle and for a random convex hexagon, each offset edge's supporting line lies `delta` (1e-9) from the matching original edge, and `contains(offset, original).margin_m >= delta - 1e-9`.
  - `Offset.BevelsSharpCorners`: a triangle with a 20° vertex; no offset vertex lies more than `2 * delta + 1e-9` from the original polygon.
  - `Offset.BeatsCentroidPushOnThinHull`: for the 1.24 × 0.04 m rectangle, the paper's centroid push (their Eq. 8) leaves a long-side clearance below `0.1 * delta`, while `offset_outward` gives `delta`.
  - `Hausdorff.ShiftedSquare`: a unit square against itself shifted 0.03 m in x gives 0.03 (1e-3); identical polygons give 0.
- [ ] **Step 2: Run them to see them fail:** `colcon test --packages-select mobile_manipulator_geometry`. Expected: build failure (undefined `convex_hull`).
- [ ] **Step 3: Implement `polygon_ops.cpp`.** The hull uses Andrew's monotone chain. For the offset, shift each edge's line outward by `delta`, intersect consecutive shifted lines; where the intersection lies more than `2 * delta` from the original vertex, emit instead the two points `v + delta * n_prev` and `v + delta * n_next`, offset along the two edge normals. `hausdorff` samples both boundaries at ≤ 0.005 m spacing and takes the larger directed maximum.
- [ ] **Step 4: Run the pure tests.** Expected: PASS.
- [ ] **Step 5: Write the failing model tests** in navigation's `test_footprint_models.cpp`, using the URDF, `qualified_payload.json` and `dynamic_footprint.yaml`. The pose sweep is the three qualified poses plus 21 joint-space samples on each of home→vertical_carry and home→level_extension.
  - `MeshModel.HullInsideMatchingProfile`: for `home` and `vertical_carry`, `contains(profile, mesh.footprint(pose, 0.0)).inside` (the B3 profile includes the 0.02 allowance).
  - `MeshModel.MatchesProjectorBounds`: the axis-aligned bounds of `mesh.footprint(pose, 0.0)` are within 0.006 m of the profile generator's `projected_bounds` (home: x -0.68..0.52, y ±0.60; vertical_carry: x -0.68..0.52, y ±0.365).
  - `DiscModel.ContainsMeshOverSweep`: for every sweep pose, `contains(disc.footprint(pose, 0.0), mesh.points(pose)).inside`.
  - `DiscModel.RadiiFromUrdf`: every computed radius is in (0, 0.25] m and a multiple of 0.005.
  - `Models.FiniteOutputOverSweep`: both models give 3-40 vertices and a positive area for every sweep pose.
- [ ] **Step 6: Run them to see them fail**, then implement `footprint_model.cpp`:
  - Mesh: `projector->projected_points(joints, {cylinder_sides, {}, true})`.
  - Disc: base points `projected_points(zero joints, {16, base_links, false})`, computed once; for each listed link, a circumscribed `disc_samples`-gon (radius `r / cos(pi / disc_samples)`) plus the centre at the projected origin; the same discs at `segment_samples` interior points of each consecutive-origin segment; the 8 projected panel-box corners; radii from the URDF rule in the plan-level decisions unless configured.
- [ ] **Step 7: Run the model tests.** Expected: PASS.
- [ ] **Step 8: Add `footprint_benchmark`** (`--output FILE`, `--evaluations 5000`): for each model and each of the three poses, time `footprint(pose, 0.02)` and write JSON `{model: {pose: {mean_us, min_us, max_us, p99_us, points, vertices, area_m2}}, disc_to_mesh_area: {pose: ratio}}`. Smoke-run it: `$C 'ros2 run mobile_manipulator_navigation footprint_benchmark --evaluations 200 --output /tmp/fb.json && cat /tmp/fb.json'`. Expected: valid JSON with both models.
- [ ] **Step 9: Commit:** "Add the convex hull, exact outward offset and mesh and disc footprint models".

---

### Task 4: `dynamic_footprint_node`, the `footprint_mode` switch and the ownership preflight

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_navigation/include/mobile_manipulator_navigation/dynamic_footprint.hpp`, `src/dynamic_footprint.cpp`, `src/dynamic_footprint_node.cpp`, `src/check_footprint_ownership.cpp`, `launch/dynamic_footprint.launch.py`
- Create: `ros2_ws/src/mobile_manipulator_navigation/test/{test_dynamic_footprint.cpp,test_footprint_ownership.cpp}`
- Modify: `include/mobile_manipulator_navigation/mission.hpp` and `src/mission.cpp` (receive `FootprintDriftGuard`; add `ownership_problems`)
- Modify: `ros2_ws/src/mobile_manipulator_manipulation/{include/mobile_manipulator_manipulation/reconfigure_logic.hpp,src/reconfigure_logic.cpp,src/reconfigure_panel_server.cpp,test/test_reconfigure_logic.cpp}` (use the moved guard)
- Modify: `ros2_ws/src/mobile_manipulator_navigation/launch/{navigation.launch.py,global_planning.launch.py}`, `src/mission_scenario_task.cpp` (error message), `tools/run_nav_scenario.py` (pass `footprint_mode:=profiles` instead of `dynamic_monitor_zones:=true`)

**Interfaces:**
- Consumes: Task 3's `FootprintModel`, `offset_outward`, `hausdorff`, `circumscribed_radius`; `mission.hpp` `Stillness`, `footprint_matches`, `to_base_frame`, `monitor_zones`.
- Produces:
  - `struct FootprintUpdate { Polygon footprint; std::vector<Polygon> zones; };` `struct TickStats { double compute_s; size_t points; size_t vertices; bool beyond_inflation; };`
  - `class DynamicFootprint { public: DynamicFootprint(std::shared_ptr<const mmg::FootprintModel>, std::vector<std::string> arm_joints, std::vector<double> zone_margins_m, double padding_m, double change_threshold_m, double stale_after_s, double inflation_radius_m); void joints(double receive_time, const std::vector<std::string> & names, const std::vector<double> & positions); std::optional<FootprintUpdate> tick(double now); const std::optional<Polygon> & current() const; enum class Health { NoData, Fresh, Stale, MissingJoint }; Health health(double now) const; const TickStats & last_stats() const; };` Times are steady-clock seconds (receive times, never message stamps).
  - `class FootprintDriftGuard` moved unchanged to `mobile_manipulator_navigation` (`mission.hpp`).
  - `std::vector<std::string> ownership_problems(const std::string & mode, const std::map<std::string, std::vector<std::string>> & publishers_by_topic);`
  - Launch argument `footprint_mode` (`static` default, `profiles`, `dynamic`) and `footprint_model` (`mesh` default, `disc`) on `navigation.launch.py` and `global_planning.launch.py`. Collision monitor zones are dynamic for `profiles` and `dynamic`; the `dynamic_monitor_zones` argument is removed. `dynamic` includes `dynamic_footprint.launch.py`.
  - Node topics: publishes `/global_costmap/footprint`, `/local_costmap/footprint` (`geometry_msgs/Polygon`), each zone's `dynamic_polygon_topic` (`PolygonStamped`, `base_footprint`), `/dynamic_footprint/footprint` (`PolygonStamped`, `base_footprint`), all reliable + transient local, depth 1. Node parameters from `dynamic_footprint.yaml` plus `stats_file` (default `""`).
  - `ros2 run mobile_manipulator_navigation check_footprint_ownership --mode static|profiles|dynamic` exits 0 when the graph matches.

- [ ] **Step 1: Write the failing logic tests** in `test_dynamic_footprint.cpp` with a fake model (a box whose half-width is `0.3 + shoulder_pan_joint` m):
  - `DynamicFootprint.SilentBeforeFirstJointState`: `tick` returns nothing; `health` is `NoData`.
  - `DynamicFootprint.PublishesFirstAndOnChange`: the first complete state gives an update whose footprint equals `offset_outward(hull, 0.02)` and whose zones equal `offset_outward(footprint, margin)`; a 0.02 m change publishes; a second identical tick does not.
  - `DynamicFootprint.IgnoresSubThresholdJitter`: 100 ticks with ±0.004 m jitter produce no update.
  - `DynamicFootprint.MergesPartialJointStates`: a message with only `shoulder_pan_joint`, after a complete one, keeps the other joints' values.
  - `DynamicFootprint.HoldsWhenAnArmJointIsMissing`: no complete state yet gives `MissingJoint` and no update.
  - `DynamicFootprint.HoldsWhenStale`: a state 0.6 s old gives `Stale`, no update, and `current()` unchanged (never shrinks).
  - `DynamicFootprint.FlagsHullBeyondInflationRadius`: with `inflation_radius_m` 0.5 and a 0.6 m half-width, `last_stats().beyond_inflation` is true.
  - `FootprintRepair.RepublishesAfterTwoSecondsAtStandstill`: the moved `FootprintDriftGuard` tests (rename from `test_reconfigure_logic.cpp`).
- [ ] **Step 2: Write the failing ownership tests** in `test_footprint_ownership.cpp`:
  - `Ownership.StaticNeedsNoPublisher`: any publisher on the four topics is a problem.
  - `Ownership.ProfilesNeedsTheServerOnly`: exactly `reconfigure_panel_server` on all four.
  - `Ownership.DynamicNeedsTheNodeOnly`: exactly `dynamic_footprint_node`; a second publisher, or the server, is named in the problem.
  - `Ownership.UnknownMode`: one problem naming the mode.
- [ ] **Step 3: Run both to see them fail**, then **implement** `dynamic_footprint.cpp`, `ownership_problems`, and move `FootprintDriftGuard`. Expected after: PASS, and `test_reconfigure_logic` still passes.
- [ ] **Step 4: Implement `dynamic_footprint_node.cpp`.** A 20 Hz wall timer calls `tick`. It subscribes `/joint_states` and `/odom` (for `Stillness`: pose and 0.5 s at rest), plus both costmaps' `published_footprint`. The self-repair is `FootprintDriftGuard` with `footprint_matches(to_base_frame(seen, pose), current, 0.01)`, judged only when the base is still and the last publish is ≥ 2 s old. It warns on `Stale`, `MissingJoint` and `beyond_inflation` at most once per second. On shutdown it writes `stats_file`: `{"model", "ticks", "publishes", "publish_times_s", "compute_us": {"mean", "min", "max", "p99"}, "points", "vertices": {"min", "max"}, "beyond_inflation_ticks"}`. `inflation_radius_m` is read from `nav2_global_planning.yaml`.
- [ ] **Step 5: Implement `check_footprint_ownership.cpp`**, modelled on `check_cmd_vel_ownership.cpp` (discovery wait, then `ownership_problems`).
- [ ] **Step 6: Wire the launch files and the runner line.** `profiles` and `dynamic` make the collision monitor follow its `dynamic_polygon_topic`s. `dynamic` includes `dynamic_footprint.launch.py` (parameters `dynamic_footprint.yaml`, `model:=<footprint_model>`, `stats_file:=/tmp/mm_dynamic_footprint_stats.json`, `use_sim_time`). The runner passes `footprint_mode:=profiles` for missions.
- [ ] **Step 7: Build and run the package tests** (`mobile_manipulator_navigation mobile_manipulator_manipulation`). Expected: all pass, including `test_reconfigure_mock.py` (B3 unchanged).
- [ ] **Step 8: Commit:** "Add the dynamic footprint node and the footprint_mode switch".

---

### Task 5: `ReconfigurePanel` dynamic mode and the hull check

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_interfaces/action/ReconfigurePanel.action`, `include/mobile_manipulator_interfaces/reconfigure_panel_codes.hpp`
- Modify: `ros2_ws/src/mobile_manipulator_navigation/{include/mobile_manipulator_navigation/mission.hpp,src/mission.cpp,test/test_mission.cpp}` (`hull_clearance`)
- Modify: `ros2_ws/src/mobile_manipulator_manipulation/{src/reconfigure_panel_server.cpp,launch/manipulation.launch.py,CMakeLists.txt}`
- Create: `ros2_ws/src/mobile_manipulator_manipulation/test/test_reconfigure_mock_dynamic.py`

**Interfaces:**
- Consumes: Task 3's `MeshHullModel::footprint`; Task 4's `dynamic_footprint.launch.py`; `StaticMap` and `Pose2` from `scenario_spec.hpp`.
- Produces:
  - Action (append-only): result constant `uint8 HULL_IN_COLLISION=11`; result fields `float64 planned_hull_clearance_m`, `float64 measured_hull_clearance_m` (NaN in `profiles` mode; the containment-margin fields are NaN in `dynamic` mode). Add the code to `reconfigure_panel_codes.hpp`.
  - `struct HullClearance { bool collision; double clearance_m; }; HullClearance hull_clearance(const StaticMap & costmap, const Polygon & footprint_base, const Pose2 & base_in_map, int lethal = 100, double search_radius_m = 1.0);`. `collision` is true when any lethal cell centre lies inside the posed polygon or within half a cell of its boundary. `clearance_m` is the distance from the polygon to the nearest lethal cell centre minus half a cell (≥ 0), or `search_radius_m` when none is within it.
  - Server parameter `footprint_mode` (`profiles` default, `dynamic`); `manipulation.launch.py` argument `footprint_mode` passes it.

- [ ] **Step 1: Write the failing `hull_clearance` tests** in `test_mission.cpp`, on a 0.05 m grid:
  - `HullClearance.FreeGridReportsSearchRadius`.
  - `HullClearance.LethalInsideIsCollision`: one lethal cell under the polygon centre.
  - `HullClearance.ClearanceToNearbyCell`: a lethal cell centred 0.30 m beyond an edge gives 0.275 (1e-6).
  - `HullClearance.UsesBasePose`: the same cell is inside for base yaw 0 and outside for yaw π/2 with an elongated polygon.
  - `HullClearance.IgnoresInscribedCost`: cost 99 cells are not collisions.
- [ ] **Step 2: Run to see failure; implement `hull_clearance`; run to pass.**
- [ ] **Step 3: Write the failing mock test** `test_reconfigure_mock_dynamic.py`. It launches `mock_stack.launch.py base_pose:=12.0,0.0,3.141593`, `manipulation.launch.py footprint_mode:=dynamic use_sim_time:=false`, and `dynamic_footprint.launch.py use_sim_time:=false`. The test publishes zero `/odom` like `test_reconfigure_mock.py`, and a latched `/global_costmap/costmap` (`map` frame, 0.05 m, 6 × 6 m centred on the base) only after `test_a`:
  - `test_a_no_costmap_fails_closed`: `vertical_carry` gives `HULL_IN_COLLISION`, the message mentions the costmap, and the arm is unchanged.
  - `test_b_reconfiguration_follows_hull`: free costmap; `vertical_carry` gives `SUCCESS`, `applied_footprint_profile == "dynamic"`, both clearances finite and > 0; `/dynamic_footprint/footprint` y-extent within ±0.386 m (`vertical_carry` ±0.365 m + 0.02 padding, 1 mm); the publishers on `/global_costmap/footprint` are exactly `[dynamic_footprint_node]`.
  - `test_c_hull_in_collision_does_not_move`: from `vertical_carry`, a lethal 0.1 m block at map (12.1, -0.5) (inside the `home` hull, outside the `vertical_carry` hull at yaw π); `home` gives `HULL_IN_COLLISION`, `planned_hull_clearance_m == 0`, and the arm is unchanged (1e-3).
  - `test_d_restore_home`: clear the block; `home` gives `SUCCESS` (leaves the mock arm at home).
- [ ] **Step 4: Run it to see failures**, then **implement dynamic mode** in the server. Parameter `footprint_mode`; in `dynamic` it creates no footprint, zone or repair publishers or timers. It keeps a latched `/global_costmap/costmap` subscription (converted to `StaticMap`) and a tf2 buffer. Before execution it runs `hull_clearance` on the planned final state's `MeshHullModel::footprint(reached, 0.02)` at TF `map -> base_footprint`. If there is no costmap, no TF, or a collision, it ends with `HULL_IN_COLLISION` before motion. After execution the same check runs on the measured state. It sets `applied_footprint_profile = "dynamic"`, `profile_violated = false`, and NaN containment margins. `profiles` mode stays byte-for-byte the B3 path.
- [ ] **Step 5: Run both mock tests and the package tests.** Expected: `test_reconfigure_mock.py` and `test_reconfigure_mock_dynamic.py` pass.
- [ ] **Step 6: Commit:** "Check the planned hull against the global costmap in ReconfigurePanel's dynamic mode".

---

### Task 6: Mission task, runner, summarizer and the `base_only` profile

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_navigation/config/footprint_profiles.yaml`, `test/test_footprint_profiles.py`
- Create: `ros2_ws/src/mobile_manipulator_navigation/{include/mobile_manipulator_navigation/footprint_watch.hpp,src/footprint_watch.cpp,src/footprint_wait.cpp}`
- Modify: `ros2_ws/src/mobile_manipulator_navigation/{include/mobile_manipulator_navigation/mission.hpp,src/mission.cpp,src/mission_scenario_task.cpp,test/test_mission.cpp,CMakeLists.txt}`
- Modify: `tools/run_nav_scenario.py`, `tools/summarize_nav_runs.py`

**Interfaces:**
- Consumes: Task 4's topics and `check_footprint_ownership`; Task 5's result fields; Task 3's `offset_outward`.
- Produces:
  - Profile `base_only: {base_only: true, polygon: [[0.46, 0.385], [0.46, -0.385], [-0.46, -0.385], [-0.46, 0.385]]}` (non-arm links without the panel: x ±0.44, y ±0.365, plus 0.02).
  - `bool footprints_applied(const std::map<std::string, PublishedPolygon> &, const std::vector<std::string> & costmap_topics, const std::vector<MonitorZone> &, const Polygon & footprint, const std::vector<Polygon> & zone_polygons, const Pose2 & robot, double since, double padding);` The existing overload builds `padded_rectangle` zone polygons and forwards.
  - `class FootprintWatch` (moved, unchanged behaviour) in `footprint_watch.hpp`, plus `std::optional<Polygon> dynamic_footprint() const` (latest `/dynamic_footprint/footprint`).
  - `ros2 run mobile_manipulator_navigation footprint_wait --mode static|dynamic [--profile NAME] [--zones] --timeout 30`: exit 0 once both costmaps show the expected footprint after a full refresh (`FootprintRefresh`), 1 on timeout.
  - `mission_scenario_task --footprint-mode profiles|dynamic` (default `profiles`).
  - Runner options `--footprint-mode static|profiles|dynamic` (default: `profiles` for missions, `static` otherwise; `profiles` is rejected for non-mission tasks), `--footprint-model mesh|disc` (dynamic only), `--footprint-profile NAME` (static only; the Nav2 launch footprint). The run directory gets the suffix `-dyn`, `-dyndisc` or `-fp<profile>`. Each run summary gets `"footprint": {"mode", "model", "nav2_profile"}` and, in dynamic mode, `"footprint_stats"` (the node's stats file).

- [ ] **Step 1: Write the failing tests:**
  - `test_footprint_profiles.py::test_base_only_bounds_the_base_without_arm_or_panel`: the non-arm links' bounds plus `perimeter_allowance_m` match `base_only` (0.006 m); the existing pose test skips `base_only` entries.
  - `test_mission.cpp` `FootprintsApplied.DynamicZonesUseOffsetPolygons`: hull zones built with `offset_outward` match and padded rectangles do not.
- [ ] **Step 2: Run to see failures, then implement** the profile, the overload, the `FootprintWatch` move, `footprint_wait`, and the mission task's dynamic mode. In dynamic mode the starting and after-step expected footprint is `watch->dynamic_footprint()`; zones are `offset_outward(footprint, margin)`. The preflight additionally runs `check_footprint_ownership --mode <mode>` logic in-process.
- [ ] **Step 3: Runner.**
  - Pass `footprint_mode`, `footprint_model` and the Nav2 `footprint_profile` (the override, or the scenario's) to the launches, and `footprint_mode` to `manipulation.launch.py`.
  - Run `check_footprint_ownership --mode <mode>` after the launch.
  - For plan-only and navigate tasks, run `footprint_wait` (`--mode dynamic --zones` or `--mode static --profile <nav2_profile>`) before the task.
  - In dynamic mode, copy `/tmp/mm_dynamic_footprint_stats.json` into the run directory after stopping Nav2.
  - Run navigate and mission tasks through `run_watching_contacts(runner, script, timeout)`. It starts the `docker exec` with `subprocess.Popen` and polls `runner.unity("scenario_contacts")` every 1.0 s. At the first contact it calls `ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '{}'` once and records `"contact_cancel_s"` (time since start) in the summary.
- [ ] **Step 4: Summarizer.**
  - Group rows by scenario, controller and footprint (`dyn`, `dyndisc`, `fp:<profile>`, or `default` for older runs).
  - Add a `collision` failure mode (drive not reached and contact true).
  - Add columns `Fp updates` (publishes during drives, from `publish_times_s` against the drive windows), `Fp µs mean/max` and `Hull refusals` (`HULL_IN_COLLISION` steps).
  - Check that older summaries still load: `python3 tools/summarize_nav_runs.py docs/experiments/moveit-arm/runs/*.json` prints the same success counts as the 2026-10-04 table (21/27).
- [ ] **Step 5: Run the package tests.** Expected: all pass.
- [ ] **Step 6: Commit:** "Run missions and scenarios with a selectable footprint strategy".

---

### Task 7: Live smoke test and the B4 regression

**Files:**
- Create: `docs/experiments/dynamic-footprint/runs/` (summaries), extend `docs/experiments/dynamic-footprint/README.md`

**Interfaces:**
- Consumes: Tasks 4-6 end to end.

- [ ] **Step 1: Build in the container** (`$C cb`) and enter Play with a new epoch through the runner.
- [ ] **Step 2: Smoke runs:**
  - `python3 tools/run_nav_scenario.py wide_gate_vertical_carry --footprint-mode dynamic --new-epoch`. Expected: ownership check passes, `footprint_wait` passes, the Lattice route passes the gate.
  - `python3 tools/run_nav_scenario.py wide_gate_mission --footprint-mode dynamic`. Expected: mission success, no contact, hull refusals 0, `Fp updates` during drives 0.

  If either fails, debug before Step 3 (superpowers:systematic-debugging).
- [ ] **Step 3: Regression** under the runner's caffeinate: `python3 tools/run_nav_scenario.py narrow_gate_mission wide_gate_mission constrained_reconfiguration_mission --footprint-mode dynamic --controller rpp --runs 3`. Expected: 9/9 as B3 RPP, no contact, `Fp updates` during drives 0.
- [ ] **Step 4: Record** the summary table (against B3 RPP at `849f19b`) and the node's live compute times in the README; copy the run summaries into `runs/`. Any deviation from B3 is reported, not tuned away.
- [ ] **Step 5: Stop Play and commit:** "Record the B4 regression on the B3 missions".

---

### Task 8: Reproduce the paper's Table III on this robot

**Files:**
- Modify: `docs/experiments/dynamic-footprint/README.md`, add summaries under `runs/`

- [ ] **Step 1: Plan-only matrix** (one run each; repeats as configured): scenarios `narrow_gate_home narrow_gate_vertical_carry wide_gate_home wide_gate_vertical_carry` × strategies `--footprint-profile base_only`, `--footprint-profile home` (enlarged), `--footprint-mode dynamic`, `--footprint-mode dynamic --footprint-model disc`. Expected pattern:
  - `base_only` routes through both gates in both poses (unsafe with `home` at the 1.05 m gate).
  - `home` (enlarged) detours at the 1.05 m gate in both poses (over-conservative for vertical carry) and is marginal at the 1.30 m gate (0.03 m per side).
  - `dynamic` detours with `home` at the 1.05 m gate and routes vertical carry through both gates.
- [ ] **Step 2: Drive matrix** under caffeinate: `narrow_gate_home_nav narrow_gate_vertical_carry_nav wide_gate_home_nav wide_gate_vertical_carry_nav` × strategies `base_only`, `home`, `dynamic` × `--runs 5 --controller rpp` (60 drives).
- [ ] **Step 3: Record a Table III analogue** (success, dominant failure mode, contacts, path length, minimum clearance, time through the gate) beside the paper's numbers. State honestly where our pattern differs from theirs and why (e.g. B3's precise profiles, our gates' geometry).
- [ ] **Step 4: Stop Play and commit:** "Record the footprint-strategy comparison (paper Table III)".

---

### Task 9: Table II, disc vs mesh, and documentation

**Files:**
- Modify: `docs/experiments/dynamic-footprint/README.md`
- Create: `docs/adr/0010-dynamic-footprint-ownership.md`
- Modify: `docs/roadmap.md` (Phase 3 section, baseline matrix B4 status, decision-log entry), `docs/adr/0009-moveit-panel-reconfiguration-baseline.md` (answer "Revisit when Phase 3 replaces profile switching"), `docs/running-the-stack.md` (new launch and runner options)

- [ ] **Step 1: Run the benchmark:** `$C 'ros2 run mobile_manipulator_navigation footprint_benchmark --evaluations 5000 --output $ROS_WS/../docs/experiments/dynamic-footprint/benchmark.json'`. Record a Table II analogue (mean, min, max, p99 µs, points, vertices) for both models beside the paper's 689-857 µs, the disc-to-mesh area ratios, and the live node numbers from Tasks 7-8.
- [ ] **Step 2: Write ADR 0010.** Context: Phase 3, the paper. Options: the three architectures, and centroid padding vs exact offset. Decision: the library plus a single-owner node, `footprint_mode`, the hull check, `enlarged = home`. Consequences, validation, revisit-when (arm motion while driving, footprint prediction in Phase 5).
- [ ] **Step 3: Update the roadmap** (decision-log entry in the template format with the Task 7-9 numbers; B4 implemented; Phase 3 comparisons answered), ADR 0009, and `running-the-stack.md`.
- [ ] **Step 4: Commit:** "Record the footprint update cost and document B4".
