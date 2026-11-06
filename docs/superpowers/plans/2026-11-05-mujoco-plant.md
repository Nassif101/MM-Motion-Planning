# MuJoCo Campaign Plant Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A ROS-free, deterministic MuJoCo plant with a Mid-360 lidar model that runs B5 and F2 in-process, calibrated against Unity and accepted by the spec's gate.

**Architecture:** `mobile_manipulator_plant` is a ROS-free C++ library (MuJoCo, Eigen, yaml-cpp) behind one step API. `mobile_manipulator_campaign` glues it to the framework (adapters, `Session`, campaign runner, demo server, replay). `run_closed_loop` gains a `ClosedLoopPlant` interface so the kinematic plant and the MuJoCo plant are interchangeable. Python tools generate the models, calibrate and view.

**Tech Stack:** C++17, MuJoCo (pinned prebuilt release via `mujoco_vendor`), Eigen, yaml-cpp, nlohmann JSON (already used via `mobile_manipulator_navigation/yaml_json.hpp`), gtest, Python 3 (pytest, numpy, PyYAML, rosbags in the container, `mujoco` on the host).

**Spec:** `docs/superpowers/specs/2026-11-05-mujoco-plant-design.md` (approved 2026-11-05). Executors read both.

## Global Constraints

- Branch `mujoco-plant`; one commit per task (more only where a step says so); never commit to `main`.
- C++ for runtime code and ROS package code; `tools/` and launch files stay Python.
- `mobile_manipulator_plant` depends on `mujoco_vendor`, Eigen, yaml-cpp, nlohmann_json and zlib only (system packages, all in the container): no rclcpp, no ROS messages, no `mobile_manipulator_*` package.
- Never commit host-absolute paths, `build/`, `install/`, `log/`, `experiment_runs/`, or the 25 MB `mid360.csv`.
- Physics step 0.002 s, integrator `implicitfast`, single thread; one seed drives every random source; same seed + same binary = bit-identical run and run log.
- Lidar: 10 Hz, 20,000 points per scan, min range 0.1 m, max range 70 m, σ = 0.02 m, dropout 0 by default; pattern from UnitySensors commit `bc8b1d6445e1` checked by SHA-256.
- Panel payload 3.0 kg (from `qualified_payload.json`); torque limits and qualified poses referenced, not copied.
- MuJoCo results carry `plant_profile` `mujoco-ideal` or `mujoco-realistic` and are never pooled with Unity results.
- Never overlap Unity Play with timed (P) measurements; stop Play after any Unity step.
- Shell scripts LF; paths in tools derived from the repository root (`ROOT = Path(__file__).resolve().parents[2]` in `tools/plant/`).

**Commands used throughout** (the container is `mm-motion-planning-ma-robot-sim-1`, workspace `$ROS_WS` = repository root):

```bash
# build PKG and its dependencies, run its gtests matching REGEX
CT() { docker exec mm-motion-planning-ma-robot-sim-1 bash -c "source /opt/ros/jazzy/setup.bash && cd \$ROS_WS && colcon build --packages-up-to $1 && source install/setup.bash && colcon test --packages-select $1 --ctest-args -R '${2:-.}' && colcon test-result --verbose --test-result-base build/$1"; }
# run a command in the sourced container workspace
CX() { docker exec mm-motion-planning-ma-robot-sim-1 bash -c "source \$ROS_WS/install/setup.bash && cd \$ROS_WS && $1"; }
# host Python tests
PT() { python3 -m pytest "$@" -q; }
```

**Deviations from the spec found while planning (for the user's review):**
1. Spec 3.1 generates one MJCF include per scenario. The plan adds scenario boxes and movers at `reset` from a `ScenarioSetup` built by the campaign glue with `mmm::load_scene`, the function B5 itself uses, so the plant collides with exactly the boxes B5 knows. The run log header stores the scenario boxes so `view_run.py` can draw them. The 604-box site stays a generated, committed MJCF.
2. Spec 7.2 fits the base from `/cmd_vel_smoothed` → `/odom` and copies "the velocity-smoother limits". In the whole-body stack B5 publishes straight to `cmd_vel_smoothed` (no smoother), then the collision monitor, then `/cmd_vel`. The base limits are therefore Unity's `SkidSteerBaseController` limits (`docs/unity-skid-steer-base-controller.md`), the fit uses `/cmd_vel` → `/odom`, and windows where `/cmd_vel` differs from `/cmd_vel_smoothed` (collision-monitor action) are excluded and counted.
3. The plant has no collision monitor (a lidar consumer; it belongs with spec 2). R reports any Unity run where the monitor acted.

## Review Focus

- **A mover blocked by the robot must wait** (Unity `ScenarioMover`: holds while the robot is within 0.1 m of its next box), or the late-crossing deadlock silently disappears → Task 4 `MoverMotion.HoldsWhileRobotWithinStandOff`.
- **A stale or ended command:** the base command older than 0.5 s ramps to zero (Unity watchdog), and the arm holds at the stream's last point → Task 6 `BaseDrive.StaleCommandRampsToZero`, `ArmServo.HoldsAtLastStreamPoint`.
- **References beyond joint limits or efforts** clamp without NaN or explosion → Task 6 `ArmServo.ClampsToEffortAndLimits`.
- **Rays starting inside a geom or hitting below the minimum range** are dropped, never reported as 0 m points → Task 5 `Lidar.DropsBelowMinRange`, `Lidar.RayFromInsideGeomIsNotAZeroHit`.
- **Reset mid-run, another scenario, or a client disconnect mid-stream** leaves no old movers, boxes or state behind and does not stop the server → Task 8 `Session.ResetToOtherScenarioLeavesNothingBehind`, Task 9 `DemoServer.SurvivesClientDisconnect`.

---

### Task 1: `mujoco_vendor`

**Files:**
- Create: `ros2_ws/src/mujoco_vendor/{CMakeLists.txt,package.xml,mujoco_vendor-extras.cmake.in,test/test_mujoco_smoke.cpp}`

**Interfaces:**
- Produces: CMake target `mujoco::mujoco` (imported shared library `libmujoco.so.<ver>` + headers `mujoco/mujoco.h`), installed licence `share/mujoco_vendor/LICENSE.mujoco`, and a CMake variable `MUJOCO_VERSION`.

- [ ] **Step 1: Pick the release.** The newest MuJoCo release on GitHub at least two weeks old that has `mjSpec` (`mj_parseXMLString`, `mjs_attach`, `mj_compile`), `mj_geomDistance` and `mj_multiRay`. Record version and SHA-256 of `mujoco-<ver>-linux-aarch64.tar.gz` and `mujoco-<ver>-linux-x86_64.tar.gz`.
- [ ] **Step 2: Write the smoke test** `test_mujoco_smoke.cpp`:
  - `TEST(MujocoSmoke, FallingBoxLandsOnPlane)`: an XML string with a plane and a 0.1 m free box at z 1.0, loaded via `mj_loadXML` from a VFS; after 2 s of `mj_step` at 0.002 s, `d->qpos[2]` within 0.002 of 0.1.
  - `TEST(MujocoSmoke, VersionMatchesPin)`: `std::string(mj_versionString()) == MUJOCO_VERSION` (passed as a compile definition).
- [ ] **Step 3: Implement the vendor package** following `daqp_vendor/CMakeLists.txt`: `ExternalProject_Add` (download only, no build) with the arch-selected URL and hash (`CMAKE_SYSTEM_PROCESSOR` `aarch64` / `x86_64`, else `FATAL_ERROR`); install `include/mujoco`, `lib/libmujoco.so*` and the licence; the extras file defines the imported target `mujoco::mujoco`.
- [ ] **Step 4: Run** `CT mujoco_vendor MujocoSmoke` on the Mac's container (arm64). Expected: 2 tests pass. Note in the commit message that amd64 is configured but tested only on the user's PC.
- [ ] **Step 5: Host viewer dependency.** Ask the user before running `python3 -m pip install --user mujoco==<ver>` on the Mac; then `python3 -c "import mujoco; print(mujoco.__version__)"` prints `<ver>`.
- [ ] **Step 6: Commit** `Vendor MuJoCo <ver> (pinned prebuilt release) with a smoke test`.

### Task 2: Site generator (G1, G2, G5)

**Files:**
- Create: `tools/plant/generate_site_model.py`, `tools/plant/test_generate_site_model.py`, `tools/plant/__init__.py`
- Create (generated, committed): `ros2_ws/src/mobile_manipulator_plant/models/site_v2.xml`
- Create: `ros2_ws/src/mobile_manipulator_plant/{CMakeLists.txt,package.xml}` (library skeleton, `models/` installed to `share`), `test/test_site_model.cpp`

**Interfaces:**
- Produces: `generate_site(boxes_yaml: Path, metadata_json: Path) -> str` (MJCF text); CLI `python3 tools/plant/generate_site_model.py [--check]` writes or checks `models/site_v2.xml`. Every box geom is named `site/<export name>`, class `site`, `contype 1 conaffinity 1`, group 0; ground `plane` named `ground`, group 0; `<custom><text name="site_checksum" data="<checksum>"/><text name="generator" data="generate_site_model 1"/></custom>`.

- [ ] **Step 1: Write failing pytest tests:**
  - `test_every_box_becomes_a_geom`: parse the output with ElementTree; 604 `geom type="box"`; for each export box: `pos == center`, `size == size/2`, `quat == (cos(yaw/2), 0, 0, sin(yaw/2))`, all within 1e-6 (G1 at generator level).
  - `test_checksum_and_generator_embedded`: `site_checksum` equals the metadata's checksum field.
  - `test_committed_model_is_current`: `generate_site(...) == models/site_v2.xml` text.
  - `test_rasterization_matches_exported_map` (G2): rasterize the geoms at 0.05 m in `PlanningBoxExporter`'s height band (read it from `PlanningBoxExporter.cs` constants and record the value in the test) over the map's grid; compare with `construction_site_v2.pgm` cell by cell; every mismatch must have its cell centre within 1e-6 of a box edge; the test prints the list.
  - `test_fixtures_match_scenarios` (G5): the gate and door openings (1.05 m) and beam undersides (1.23 / 1.30 m) computed from the generated geoms plus the scenario obstacles equal the values the scenario descriptions state, within 1e-6. Fixture list as a table in the test.
- [ ] **Step 2: Run** `PT tools/plant/test_generate_site_model.py` → FAIL (module missing).
- [ ] **Step 3: Implement** `generate_site` and the CLI (`--check` exits 1 on a difference); generate `site_v2.xml`.
- [ ] **Step 4: Run** the pytest file → PASS.
- [ ] **Step 5: C++ read-back test** `TEST(SiteModel, CompiledGeomsMatchExport)` (G1 after MuJoCo's compiler): load `share/mobile_manipulator_plant/models/site_v2.xml`, compare all 604 compiled `geom_pos`, `geom_size`, `geom_quat` to the boxes YAML (yaml-cpp) within 1e-6; read `site_checksum` from `custom`. Run `CT mobile_manipulator_plant SiteModel` → PASS.
- [ ] **Step 6: Commit** `Generate the MuJoCo site model from the V2 box export (G1, G2, G5)`.

### Task 3: Robot profile and robot generator (consistency, G3)

**Files:**
- Create: `tools/plant/generate_robot_model.py`, `tools/plant/test_generate_robot_model.py`
- Create: `ros2_ws/src/mobile_manipulator_plant/robots/mobile_manipulator/profile.yaml`, generated `.../robot.xml`
- Create: `ros2_ws/src/mobile_manipulator_plant/include/mobile_manipulator_plant/{types.hpp,robot_profile.hpp}`, `src/robot_profile.cpp`, `test/test_robot_profile.cpp`
- Create: `ros2_ws/src/mobile_manipulator_campaign/{CMakeLists.txt,package.xml}`, `test/test_profile_consistency.cpp`, `test/test_distance_agreement.cpp`

**Interfaces:**
- `profile.yaml` keys: `urdf` (package-relative `mobile_manipulator_description/urdf/mobile_manipulator.urdf`), `payload_json` (`mobile_manipulator_control/config/qualified_payload.json`), `arm_joints` (6 names, UR order), `torque_limits_nm` (reference to the medium config key, plus the values for the consistency test), `base: {height_m, mass_kg, limits: {v_max_fwd 0.8, v_max_rev 0.5, w_max 0.8, accel 0.5, decel 0.8, yaw_accel 0.8, yaw_decel 1.2, stale_s 0.5}, breakaway: {linear_mps 0.0375, yaw_radps 0.07}, traction_mu}`, `arm: {kp [2000,6000,3500,500,650,300], kd [240,360,180,40,35,28], armature [...], damping [...], gravity_ff true, motion_ff true}`, `lidar: {site livox, rate_hz 10, points 20000, min_m 0.1, max_m 70, sigma_m 0.02, dropout 0.0}`, `ideal: {breakaway false}`, `realistic: {command_delay_s, state_delay_s, base_lag_s [tau_v, tau_w], base_slip [k_v, k_w]}` (Unity placeholders 0.04 / 0 / measured later; Task 7 overwrites with provenance).
- `types.hpp` (namespace `mobile_manipulator_plant`, alias `mmp` in tests): `using Vec6 = Eigen::Matrix<double,6,1>; struct Pose2 {double x, y, yaw;}; enum class PlantMode {Ideal, Realistic}; struct StreamPoint {double t; Vec6 q, v;}; struct ArmReference {double start_time = 0; std::vector<StreamPoint> points; std::optional<Vec6> joint_velocity;}; struct PlantCommand {double stamp; double v, omega; ArmReference arm;}; struct PlantState {double t; Pose2 base; double v, omega; Vec6 q, qd; Eigen::Isometry3d panel;}; struct SceneBox {std::string name; Eigen::Vector3d center, size; double yaw;}; struct MoverSpec {std::string name; Eigen::Vector2d start, end; Eigen::Vector3d size; double speed_mps, trigger_distance_m; int crossings;}; struct ScenarioSetup {std::string name; std::vector<SceneBox> boxes; std::vector<MoverSpec> movers;};`
- `RobotProfile load_profile(const std::string & profile_path, const std::function<std::string(const std::string&)> & resolve_package)` (the resolver maps a package name to its share directory; the campaign passes `ament_index_cpp`, plant tests pass a lambda over `$ROS_WS/install`).
- Robot MJCF names: bodies `base_footprint`, URDF link names; joints `base_x`, `base_y`, `base_yaw`, the six URDF arm joints; geoms `robot/<urdf collision name>`, class `robot`, group 1; panel body `panel` child of `tool0`, geom `robot/panel`, group 2; site `livox`; motors `motor/<joint>` with `ctrlrange` = URDF effort.

- [ ] **Step 1: Failing pytest tests:** `test_planar_base_joints` (slide x, slide y, hinge z on `base_footprint`), `test_arm_matches_urdf` (axes, limits, masses, inertias, origins within 1e-9), `test_collision_geoms_are_urdf_primitives` (one geom per URDF `<collision>`, sizes = box half sizes / cylinder radius and half length), `test_panel_from_payload` (size, centre, 3.0 kg), `test_wheels_have_no_contact` (`contype 0 conaffinity 0`), `test_contact_exclusions` (arm self-pairs off except panel vs base_link/chassis), `test_committed_robot_is_current`.
- [ ] **Step 2: Run** `PT tools/plant/test_generate_robot_model.py` → FAIL. **Step 3:** implement the generator (ElementTree over the URDF; masses/inertias in MuJoCo `inertial` with `fullinertia`). **Step 4:** run → PASS.
- [ ] **Step 5: C++ profile test** `TEST(RobotProfile, LoadsAndResolvesReferences)`: joint order equals `kArmJoints`' strings, the payload mass 3.0, lidar values as in Global Constraints. Run `CT mobile_manipulator_plant RobotProfile` → PASS.
- [ ] **Step 6: Consistency test** (campaign package, depends on plant + mpc + medium + geometry): `TEST(ProfileConsistency, MatchesB5AndF2Configs)`: joint order = `mmm::kArmJoints`; joint limits = `mmm::load_limits(...)` values; `torque_limits_nm` = medium config's; collision primitives from `load_primitives(urdf, payload)` = the compiled robot geoms (same count; pose and dims within 1e-9).
- [ ] **Step 7: G3** `TEST(DistanceAgreement, MujocoMatchesCollisionModel)`: compose robot + site (Task 4's builder is not ready yet, so this test loads `robot.xml` and `site_v2.xml` via an MJCF `<include>` wrapper string); sample 10,000 states (seeded `std::mt19937 42`) within 0.5 m of the gate, door and beam fixtures (base poses along the scenario routes, joints uniform within ±0.3 rad of the qualified poses); for each primitive–box pair with B5 distance < 0.3 m, `|mj_geomDistance − CollisionModel distance| <= 0.001`. Report the max difference. Run `CT mobile_manipulator_campaign 'ProfileConsistency|DistanceAgreement'` → PASS.
- [ ] **Step 8: Commit** `Add the robot profile and generated robot model; check them against B5's configs (G3)`.

### Task 4: Plant core in ideal mode, `ClosedLoopPlant`, movers (E1, E2) — CP1

**Files:**
- Create (plant): `include/mobile_manipulator_plant/{model_builder.hpp,plant.hpp,movers.hpp}`, `src/{model_builder.cpp,plant.cpp,movers.cpp}`, `test/{test_movers.cpp,test_plant_ideal.cpp}`
- Create (mpc): `include/mobile_manipulator_mpc/closed_loop_plant.hpp`; modify `kinematic_plant.hpp` (implements it)
- Modify (medium): `include/mobile_manipulator_medium/closed_loop.hpp`, `src/closed_loop.cpp`
- Create (campaign): `include/mobile_manipulator_campaign/{adapters.hpp,mujoco_plant.hpp,scenario_setup.hpp}`, `src/{adapters.cpp,mujoco_plant.cpp,scenario_setup.cpp}`, `test/test_equivalence.cpp`

**Interfaces:**
- `mjModel * build_model(const RobotProfile &, const std::string & site_mjcf, const ScenarioSetup &, std::string & error)` via `mjSpec`: parse site and robot, attach robot to the world, add scenario boxes as static geoms `scenario/<name>` (group 3) and one mocap body per mover `mover/<name>` with a box geom (group 4), compile. Caller frees with `mj_deleteModel`.
- `class MoverMotion` (exact port of `ScenarioMoverMotion.cs`, same validation messages): `MoverMotion(const MoverSpec &)`, `Eigen::Vector2d next(double t, Eigen::Vector2d robot)`, `void commit(double t, Eigen::Vector2d p)`, `void hold(double t)`, `Eigen::Vector2d position_at(double distance) const`, accessors `walked()`, `blocked_s()`, `finished()`, `position()`. Stand-off 0.1 m (`ScenarioMover.StandOff`); the plant holds a mover when its box at the next position, grown by 0.1 m, overlaps any robot geom (`mj_geomDistance < 0`).
- `class Plant`: `Plant(RobotProfile, std::string site_mjcf, PlantMode)`; `void reset(const ScenarioSetup &, const Pose2 & start, const Vec6 & q0, uint64_t seed)`; `const PlantState & step(const PlantCommand &, double dt)` (dt a multiple of 0.002 within 1e-9, else `std::invalid_argument`); `GroundTruth ground_truth() const` (`struct Contact {std::string robot_geom, other_geom; double depth; Eigen::Vector3d pos;}; struct GroundTruth {double min_clearance_m; std::string robot_geom, other_geom; Eigen::Vector3d witness_robot, witness_other; std::vector<Contact> contacts;}` in `types.hpp`); `std::vector<std::pair<std::string, Eigen::Vector2d>> mover_positions() const`; `void move_body(const std::string & name, const Eigen::Vector3d & center, double yaw)`; `void set_ideal_breakaway(bool)`; `const mjModel * model() const`.
- Ideal step: base by the exact unicycle arc for (v, ω) over each 0.002 s substep, joints `q += qdot·dt` (stream references sampled at the substep end set q, qd directly), clamped to joint limits; `mj_forward` after each substep for contacts; ideal breakaway as `KinematicPlant` (from rest, |v| < 0.0375 and |ω| < 0.07 → base stays).
- `mobile_manipulator_mpc::ClosedLoopPlant` (pure interface): `virtual const State & state() const`, `virtual const Control & executed() const`, `virtual bool at_rest() const`, `virtual void step(const Control & command, const CycleOutput & cycle, double t, double dt)`, `virtual void update_world(World & world, double t)` (default: nothing), `virtual std::optional<double> true_min_clearance() const` (default `std::nullopt`), `virtual int contacts() const` (default 0).
- `run_closed_loop` overload taking `mmm::ClosedLoopPlant & plant` and `mmm::World world` (by value; the plant updates its movers each cycle before B5's step); the existing signature constructs a `KinematicPlant` and forwards. `ClosedLoopResult` gains `double plant_min_clearance_m = inf; int plant_contacts = 0;`.
- Campaign: `ScenarioSetup scenario_setup(const mmm::Scene &)`; `mmp::PlantCommand to_plant_command(const mmm::Control &, const mmm::CycleOutput &, double t)` (ideal: joint velocities; realistic, Task 6: stream points via `mmm::plan_stream_points` with the node's defaults); `mmm::State to_state(const mmp::PlantState &)`; `class MujocoPlant : public mmm::ClosedLoopPlant` (owns a `mmp::Plant`; `update_world` calls `World::update_mover` for each mover position with stamp t).

- [ ] **Step 1: Mover tests** `test_movers.cpp`, mirroring the C# class: `WaitsUntilTriggered`, `WalksAtSpeedAfterTrigger` (0.8 m/s, 1 s → 0.8 m), `AlternatesCrossings`, `StaysAtLastEnd`, `RejectsInvalidSpecs` (speed 0, 2.1; trigger 0, 21; crossings 0, 11; length < 0.1), `HoldsWhileRobotWithinStandOff` (plant-level: a robot base placed on the mover's path; after 2 s the mover has `walked()` 0 past the blocking point and `blocked_s() > 1.9`). Run → FAIL; implement `movers.cpp`; run → PASS.
- [ ] **Step 2: Plant ideal tests** `test_plant_ideal.cpp`: `StepRejectsNonMultipleDt`; `ArcIntegrationIsExact` (v 0.3, ω 0.4, 2 s, against the closed-form arc within 1e-9); `IdealBreakawayHoldsBaseFromRest`; `ContactsLoggedNotPushed` (drive into a scenario box; `contacts` non-empty, base pose still the integrated one); `ResetClearsState` (reset twice with different scenarios; geoms and movers of the first are gone, t = 0); `Deterministic` (two plants, same seed and commands, `PlantState` bytes equal after 10 s).
- [ ] **Step 3: Run** `CT mobile_manipulator_plant 'Movers|PlantIdeal'` → FAIL; implement `model_builder.cpp`, `plant.cpp`; run → PASS.
- [ ] **Step 4: Interface refactor.** Add `closed_loop_plant.hpp`; `KinematicPlant` implements it (behaviour unchanged); add the `run_closed_loop` overload. Run `CT mobile_manipulator_medium ClosedLoop` and `CT mobile_manipulator_mpc` → the existing tests pass unchanged.
- [ ] **Step 5: Equivalence test (E1)** `test_equivalence.cpp`, `TEST(Equivalence, IdealPlantMatchesKinematicBench)`, parameterized over the 7 corpus cells (`docs/experiments/medium-horizon/phase5a/corpus.json`; cells with a bag read their first goal's route and start as `medium_bench closed` does, skipped with a message when the bag is absent; `v2_door_then_beam_nav` synthetic): run `run_closed_loop` with `KinematicPlant` and with `MujocoPlant` (ideal, breakaway on), both with `fixed_latency_s` = profile cap and iteration budget 15 (the bench's reproducible setting). Assert: same `reached`, same `end_status`, `|time_s| diff <= 0.02`, max base position deviation along the run `<= 1e-3` m (sampled every cycle via `on_plan` or a per-cycle hook). **E2:** `TEST(Equivalence, SameSeedSameRun)`: one cell twice with `MujocoPlant`, every per-cycle state equal bit for bit.
  - To share the bench setup, move `Bench` (config, URDF, payload, primitives, limits, poses, scene, world, planner factory) out of `medium_bench.cpp` into `include/mobile_manipulator_medium/bench_setup.hpp` + `src/bench_setup.cpp` unchanged in behaviour; `medium_bench` uses it.
- [ ] **Step 6: Run** `CT mobile_manipulator_campaign Equivalence` → PASS (long; RUN_SERIAL, timeout 1800).
- [ ] **Step 7: Commit** in two commits: `Add the ClosedLoopPlant interface and move the bench setup into the medium library`; `Add the MuJoCo plant in ideal mode with scenario movers (E1, E2)`.
- [ ] **CP1:** report the 7-cell equivalence table (outcome, time, max deviation) to the user; stop until they continue.

### Task 5: Lidar (G4, L statistics) — CP2

**Files:**
- Create (plant): `cmake/fetch_scan_pattern.cmake`, `include/mobile_manipulator_plant/{lidar.hpp,lidar_noise.hpp,static_bvh.hpp}`, `src/{lidar.cpp,lidar_noise.cpp,static_bvh.cpp}`, `test/{test_lidar_noise.cpp,test_lidar.cpp}`, `bench/lidar_bench.cpp`

**Interfaces:**
- Build: download `Runtime/RawData/LivoxScanPattern/mid360.csv` from the UnitySensors repository at commit `bc8b1d6445e1` (`file(DOWNLOAD ... EXPECTED_HASH SHA256=<hash of the file in the Unity package cache>)`), convert at build time (a small C++ tool) to `share/mobile_manipulator_plant/lidar/mid360.bin` (float32 azimuth, zenith in radians, little-endian, 800,000 records) and install the Livox licence beside it.
- `double lidar_range_noise(uint64_t seed, int64_t scan, int index, double sigma)`: port of `LidarRangeNoise.Sample` (same hash and Box–Muller as the C# source, so identical inputs give identical values).
- `struct ScanPattern {std::vector<float> azimuth, zenith;}; ScanPattern load_pattern(const std::string & bin_path);`
- `enum class HitClass : uint8_t {Site, Scenario, Mover, Ground, Robot, Panel};` `struct PlantScan {double t_start, t_end; std::vector<Eigen::Vector3f> points; std::vector<float> dt; std::vector<int> geom; std::vector<HitClass> cls;}` (sensor frame).
- `class Lidar`: `Lidar(const LidarParams &, ScanPattern)`, `void reset(uint64_t seed)`, `void cast_slice(const mjModel *, mjData *, double t, double dt)` (casts `points·rate·dt` rays from the current `livox` site pose; window index advances through the pattern by `points` per scan, wrapping), `std::optional<PlantScan> take_scan()`, `void set_enabled(bool)`. Geom class from the geom name prefix (`site/`, `scenario/`, `mover/`, `ground`, `robot/panel`, `robot/`).
- `class StaticBvh` (only if CP2 rules it in): built from the compiled model's group-0 and group-3 geoms; `double ray(const Eigen::Vector3d & o, const Eigen::Vector3d & d, double cutoff, int & geom) const`.
- `Plant` gains `std::optional<PlantScan> scan()` and calls `cast_slice` every substep.

- [ ] **Step 1: Noise test** `TEST(LidarNoise, MatchesCSharpReference)`: five `(seed, scan, index, sigma)` tuples with values computed by running the C# `LidarRangeNoise.Sample` once (record the values in the test as constants; obtain them with `unity` CLI or a one-off `dotnet` script — note the command in the test comment); `TEST(LidarNoise, IsStandardNormal)`: 200,000 draws, mean within 0.01σ·5, std within 1 %, KS p > 0.01.
- [ ] **Step 2: Lidar tests** `test_lidar.cpp` on a small fixture model (plane + three boxes + the robot): `ExactRangesWithoutNoise` (G4: every ray's range equals an analytic ray–box/plane intersection within 1e-6 and the same geom), `WindowAdvancesPerScan`, `MotionDistortion` (moving base: a slice's origin differs from the previous slice's by v·dt), `LabelsByClass`, `DropsBelowMinRange`, `RayFromInsideGeomIsNotAZeroHit` (sensor inside a box: no point with range < 0.1), `NoiseResidualsNormal` (KS against N(0, 0.02), p > 0.01), `DisabledCastsNothing`.
- [ ] **Step 3: Run** `CT mobile_manipulator_plant 'Lidar'` → FAIL; implement; run → PASS.
- [ ] **Step 4: Benchmark** `lidar_bench`: the full site model, robot in carry at five site poses, 50 scans each with `mj_multiRay`; print mean and p99 ms per scan.
- [ ] **Step 5: Commit** `Add the Mid-360 lidar model with seeded noise and labels (G4)`.
- [ ] **CP2:** report the benchmark. If p99 > 25 ms (25 % of 100 ms), propose the BVH; on the user's go-ahead implement `StaticBvh` (static geoms; MuJoCo rays for groups 1, 2, 4, nearest hit wins), extend `ExactRangesWithoutNoise` to both paths, rerun the benchmark, commit `Cast static geometry through a BVH`.

### Task 6: Realistic mode (`BaseDrive`, `ArmServo`, delays, `plant_replay`)

**Files:**
- Create (plant): `include/mobile_manipulator_plant/{base_drive.hpp,arm_servo.hpp,stream_sampler.hpp,delay_line.hpp}`, `src/{base_drive.cpp,arm_servo.cpp,stream_sampler.cpp}`, `test/{test_base_drive.cpp,test_arm_servo.cpp,test_plant_realistic.cpp}`
- Create (campaign): `src/plant_replay.cpp`, `test/test_stream_sampler_consistency.cpp`; modify `adapters.cpp` (realistic: stream points)

**Interfaces:**
- `template <class T> class DelayLine {public: explicit DelayLine(int steps); T push(const T & in);}` (returns the value `steps` pushes ago; the first `steps` calls return the first value).
- `struct BaseLimits {double v_fwd, v_rev, w, accel, decel, yaw_accel, yaw_decel, stale_s;}`; `struct BaseDriveParams {BaseLimits limits; double breakaway_linear, breakaway_yaw, tau_v, tau_w, k_v, k_w, traction_mu, mass_kg;}`. `class BaseDrive {public: explicit BaseDrive(BaseDriveParams); void reset(); Eigen::Vector2d target(Eigen::Vector2d command, double command_age_s, Eigen::Vector2d measured, double dt);}`: clamp (forward/reverse speed, yaw rate), stale command (age > 0.5 s) → command zero, chassis rate limits with braking-to-zero before reversal (Unity order), breakaway from rest, first-order lag, slip. `Plant` applies the target with a velocity servo on `base_x`, `base_y`, `base_yaw` in the body frame (lateral target 0), force limited to μ·m·g (torque about z limited to μ·m·g·0.32 m, half the 0.64 m track).
- `void sample_stream(const ArmReference &, const Vec6 & q_hold, double t, Vec6 & q, Vec6 & v, Vec6 & a)`: JTC semantics (cubic Hermite between points, the first point before it, at rest at the last point after it; `a` by the Hermite second derivative).
- `struct ArmServoParams {Vec6 kp, kd, effort; bool gravity_ff, motion_ff;}`; `Vec6 servo_torque(const mjModel *, mjData *, const ArmServoParams &, const Vec6 & q_r, const Vec6 & v_r, const Vec6 & a_r)`: τ = Kp(q_r − q) + Kd(v_r − q̇) + gravity FF + motion FF via `mj_rne` on a scratch `mjData` at the reference state, clamped to ±effort.
- `plant_replay --bag DIR --profile FILE --out FILE.json [--param key=value ...]`: replays the recorded `/cmd_vel` and arm stream (from `/arm_controller/joint_trajectory`) open-loop through the realistic plant from the bag's first state; writes time series of achieved base twist and joint positions next to the recorded ones.

- [ ] **Step 1: Tests:** `DelayLine.ReturnsValueStepsAgo`; `BaseDrive.ClampsSpeeds`, `BaseDrive.RateLimitsLikeUnity` (0 → 0.3 m/s takes 0.6 s at 0.5 m/s²), `BaseDrive.BrakesBeforeReversal`, `BaseDrive.BreakawayFromRest`, `BaseDrive.StaleCommandRampsToZero`, `BaseDrive.LagReachesSixtyThreePercentAtTau`; `ArmServo.GravityFeedforwardHoldsCarry` (carry pose, payload, zero reference motion, 2 s: |q − q_r| < 1e-3 rad per joint with Kp only 10 % of profile), `ArmServo.HoldsAtLastStreamPoint`, `ArmServo.ClampsToEffortAndLimits` (reference 1 rad beyond the shoulder limit: torque within ±160 N m, no NaN, q stays within limit + 0.05), `PlantRealistic.CommandDelayShiftsResponse` (a step command: base starts moving `command_delay_s` later, within one physics step), `PlantRealistic.StateDelayShiftsReportedState` (the reported pose lags the true pose by `state_delay_s`), `PlantRealistic.FeedforwardSwitchesOff` (with both off, the carry hold error grows above the Kp-only sag), `PlantRealistic.SideslipUnderArmSwing` (fast pan swing at rest: lateral displacement bounded, < 0.02 m); campaign `StreamSampler.MatchesJtcSampling` (`sample_stream` vs `mmm::sample_trajectory` on 100 random streams within 1e-12).
- [ ] **Step 2: Run** `CT mobile_manipulator_plant 'DelayLine|BaseDrive|ArmServo|PlantRealistic'` and `CT mobile_manipulator_campaign StreamSampler` → FAIL; implement; run → PASS.
- [ ] **Step 3: `plant_replay`** on one recorded run (`CX "install/mobile_manipulator_campaign/lib/mobile_manipulator_campaign/plant_replay --bag experiment_runs/20261102T074616Z-v2_door_105_nav-wholebody-50hz-gff-mff-bff-f2/bag --profile ... --out /tmp/replay.json"`): exits 0 and writes both series with equal lengths.
- [ ] **Step 4: Commit** `Add the realistic plant: Unity-like base drive, servo arm with MuJoCo feedforward, delays, replay`.

### Task 7: Calibration (C) — CP3

**Files:**
- Create: `tools/plant/calibrate_plant.py`, `tools/plant/test_calibrate_plant.py`, `docs/experiments/mujoco-plant/calibration.md`
- Modify: `ros2_ws/src/mobile_manipulator_plant/robots/mobile_manipulator/profile.yaml` (`realistic` section with `provenance`)

**Interfaces:**
- `select_runs(experiment_runs: Path) -> list[Path]`: whole-body runs whose directory suffix contains `gff-mff` (today's actuator configuration), with `summary.json` and a bag.
- `split_by_cell(runs, held_out=0.2, seed=0) -> tuple[list, list]`: whole cells (scenario names) go to one side.
- `cross_correlation_delay(cmd_t, cmd, out_t, out, max_s=0.2, step=0.002) -> float`.
- `fit_first_order(cmd_t, cmd, out_t, out, delay) -> tuple[float, float]` (τ, k by least squares on the discretized lag).
- `monitor_windows(cmd_vel, cmd_vel_smoothed, tol=1e-6) -> list[tuple[float, float]]` (excluded windows).
- `validate(held_out, profile) -> dict` (per Section 7.3 metrics; runs `plant_replay` per run).
- CLI: `calibrate_plant.py base|arm|validate|all --out docs/experiments/mujoco-plant/ [--write-profile]`. Runs in the container (rosbags, numpy): `CX "python3 tools/plant/calibrate_plant.py all --out docs/experiments/mujoco-plant/"`.

- [ ] **Step 1: Unit tests on synthetic signals:** `test_cross_correlation_recovers_delay` (0.04 s within 0.002), `test_first_order_fit_recovers_tau_and_gain` (τ 0.15, k 0.93 within 2 %), `test_split_keeps_cells_whole`, `test_monitor_windows_found`, `test_select_runs_filters_configuration` (fixture directory names).
- [ ] **Step 2: Run** `PT tools/plant/test_calibrate_plant.py` → FAIL; implement; run → PASS.
- [ ] **Step 3: Fit** base (delay, τ_v, τ_ω, k_v, k_ω; breakaway cross-check against turn-from-rest events) and arm (delay; Kp/Kd refinement by coordinate search over ×{0.5, 0.7, 1, 1.4, 2} per joint with `plant_replay`, objective the per-joint p95 error difference from Unity's `controller_state` error).
- [ ] **Step 4: Validate** on held-out runs against Section 7.3 (arm p95 within ±30 % of Unity's or ≤ 2 mrad; base speed RMSE ≤ 0.02 m/s; yaw-rate RMSE ≤ 0.03 rad/s; 2 s drift ≤ 0.03 m; delays within one physics step). Write `calibration.md` (runs used, split, fitted values, residual table, excluded monitor windows count, gaps found, e.g. missing turn-from-rest excitation).
- [ ] **Step 5: Commit** (profile values with `--write-profile` only after CP3) `Calibrate the realistic plant against the Unity recordings`.
- [ ] **CP3:** show the user the fit and residual table; on approval run with `--write-profile`, commit `Freeze the calibrated realistic profile`.

### Task 7b: URDF-based realistic mode (amendment 2026-11-06) — CP3b

**Spec:** `docs/superpowers/specs/2026-11-06-mujoco-plant-amendment-urdf-realistic.md` (approved 2026-11-06).

**Files:**
- Modify: `tools/plant/generate_robot_model.py` (+ test): also writes `robot_wheels.xml` (free base `base_free`, wheel
  hinges with the URDF's axis, damping, friction; wheel geoms touching the ground; `wheel/<joint>` velocity actuators,
  kv `base.wheel_kv`, forcerange the URDF effort). `robot.xml` (planar) unchanged in structure.
- Modify (plant): `robot_profile` (`realistic.base_model: wheels|planar`, `model_wheels`, `base.wheel_kv`,
  `realistic.contact: {mu, impratio}`, optional `arm.gain_rule: {bandwidth_hz, zeta, pose}`), `model_builder`
  (picks the model, sets the elliptic cone, impratio, ground and wheel friction, wheel kv; `resolve_arm_gains`),
  `base_drive` (`wheel_speeds`), `plant` (free-base pose and twist helpers, wheel motors, wheel–ground contacts not
  logged as collisions).
- Create: `robots/mobile_manipulator/profile_unity.yaml` (the Task 7 calibrated planar profile, reproducible);
  `profile.yaml` becomes the URDF-based default (assumptions section with sources).
- Modify (campaign): `replay.cpp` overrides `effective_track_m`, `wheel_speed_max`, `wheel_kv`, `contact_mu`,
  `gain_bandwidth_hz`; `test_profile_consistency` (the planners' envelope against the plant's reachable envelope).
- Create: `tools/plant/commission_plant.py` (+ tests), `docs/experiments/mujoco-plant/commissioning.md`.

**Interfaces:**
- `Eigen::Vector2d wheel_speeds(const Eigen::Vector2d & u, double effective_track_m, double wheel_radius_m)` → (left,
  right) rad/s (u already saturated by `BaseDrive`).
- `void resolve_arm_gains(const mjModel *, RobotProfile &)`: with a gain rule, kp = I ω², kd = 2 ζ I ω, I the diagonal
  of the joint-space inertia (armature included) at the rule's pose, ω = 2π bandwidth.

- [ ] **Step 1: Tests (RED):** generator `test_wheels_model_has_free_base_and_driven_wheels`,
  `test_wheels_model_uses_urdf_wheel_dynamics`, `test_default_profile_arm_terms_are_the_urdf_s`; plant
  `RobotProfile.ParsesBaseModelContactAndGainRule`, `RobotProfile.RejectsBadBaseModel`, `BaseDrive.WheelSpeedsFromTrack`,
  `PlantWheels.StandsStill` (5 s, drift < 1 mm), `PlantWheels.DrivesStraight` (0.3 m/s within 3 %),
  `PlantWheels.TurnsInPlace` (0.4 rad/s command, achieved > 0.25), `PlantWheels.StopsWithinDecelLimit`,
  `PlantWheels.WheelGroundContactIsNotACollision`, `PlantWheels.ReportsPoseAndTwistFromTheFreeBase`,
  `PlantWheels.IdealModeStaysPlanar`, `ArmGains.FromTheBandwidthRule`, `PlantWheels.ArmHoldsCarryWithUrdfFriction`;
  campaign `ProfileConsistency.PlannerEnvelopeIsReachable`; python `test_commission_*` on synthetic replay outputs.
- [ ] **Step 2:** implement; `ct.sh mobile_manipulator_plant`, `ct.sh mobile_manipulator_campaign`, `pt.sh tools/plant` → PASS.
- [ ] **Step 3: Commission** (`commission_plant.py all`): steady grid, effective track, breakaway, envelope, drift,
  μ 0.4–0.8 and armature ±50 % sensitivity, arm steps and recorded streams, bandwidth choice; write the profile values;
  rerun `calibrate_plant.py validate` (Unity comparison, reported). **CP3b:** the user reviews commissioning.md.
- [ ] **Step 4: Commit** per fix; task-done.

### Task 8: Run log, `Session`, campaign runner

**Files:**
- Create (plant): `include/mobile_manipulator_plant/{frames.hpp,run_log.hpp}`, `src/{frames.cpp,run_log.cpp}`, `test/test_frames.cpp`
- Create (campaign): `include/mobile_manipulator_campaign/session.hpp`, `src/{session.cpp,plant_campaign.cpp}`, `test/test_session.cpp`
- Create: `tools/plant/plant_campaign.py`, `tools/plant/test_plant_campaign.py`

**Interfaces:**
- Frame types (`enum class FrameType : uint16_t {Hello = 1, Control = 2, Event = 3, State = 10, Plans = 11, Scan = 12}`), `std::string encode(FrameType, const nlohmann::json & header, const std::vector<std::vector<float>> & blocks)`; `bool decode(std::string_view, FrameType &, nlohmann::json &, std::vector<std::vector<float>> &, size_t & consumed)`. Wire: `[u32 length][u16 type][u32 json_len][json][u32 n_blocks]{[u32 n_floats][float32...]}`, little-endian. `kSchemaVersion = 1`. JSON via the system `nlohmann-json3-dev` (`find_package(nlohmann_json)`), zlib via `find_package(ZLIB)`.
- `class RunLogWriter {RunLogWriter(path, const nlohmann::json & header); void write(const std::string & frame); void close();}` zlib chunks of 1 MiB; `class RunLogReader {explicit RunLogReader(path); const nlohmann::json & header() const; bool next(std::string & frame);}`.
- `struct SessionConfig {std::string profile; mmp::PlantMode mode; bool medium; std::string medium_config; bool lidar; int scan_decimation; double timeout_s; uint64_t seed; std::string log_path;}`; `class Session {explicit Session(SessionConfig); std::vector<std::string> list_scenarios() const; void reset(const std::string & scenario, std::optional<mmp::Pose2> start, uint64_t seed); void set_goal(const mmp::Pose2 &); void set_route(const mmm::Route &); bool step_cycle(); void move_body(const std::string &, const Eigen::Vector3d &, double yaw); std::vector<std::string> take_frames(); std::vector<nlohmann::json> take_events(); nlohmann::json summary() const;}`. Events `goal_reached`, `failed`, `contact`, `timeout`, `fallback`. Routes in this spec: recorded (from a bag, as `medium_bench closed`) or a straight start–goal line (synthetic cells, click goals).
- `plant_campaign --scenario NAME [--bag DIR] --mode ideal|realistic [--b5-only] [--medium-config FILE] [--seed N] [--lidar] --out DIR`: one run; writes `DIR/summary.json` (the `medium_bench closed` record fields plus `plant_profile`, `host`, `site_checksum`, `seed`, `plant_min_clearance_m`, `plant_contacts`, `real_time_factor`) and `DIR/run.mmlog`. Missions: every recorded goal in order, plant state carried over.
- `tools/plant/plant_campaign.py run --corpus FILE --mode ideal|realistic --seeds N --jobs J --out DIR [--b5-only]`: parallel `docker exec` runs (J processes); `table` subcommand reuses `medium_bench.closed_table` columns plus the plant fields.

- [ ] **Step 1: Tests:** `Frames.RoundTrip` (random JSON + blocks), `Frames.RejectsTruncated`, `RunLog.RoundTripAndHeader`, `RunLog.SameSeedSameBytes` (E2 for logs); `Session.ResetToOtherScenarioLeavesNothingBehind`, `Session.EmitsGoalReached` (open-space straight route), `Session.SummaryHasProfileFields`; pytest `test_jobs_partition_runs`, `test_table_has_plant_fields`.
- [ ] **Step 2: Run** `CT mobile_manipulator_plant 'Frames|RunLog'`, `CT mobile_manipulator_campaign Session`, `PT tools/plant/test_plant_campaign.py` → FAIL; implement; run → PASS.
- [ ] **Step 3: Smoke:** `python3 tools/plant/plant_campaign.py run --corpus docs/experiments/medium-horizon/phase5a/corpus.json --mode ideal --seeds 1 --jobs 4 --out experiment_runs/plant-smoke` → 7 summaries, each with `plant_profile` `mujoco-ideal`.
- [ ] **Step 4: Commit** `Add the run log, the session and the parallel campaign runner`.

### Task 9: Demo server and `view_run.py`

**Files:**
- Create (campaign): `src/plant_demo_server.cpp`, `src/stream_server.cpp`, `include/mobile_manipulator_campaign/stream_server.hpp`, `test/test_demo_server.cpp`
- Create: `tools/plant/plant_stream.py` (Python frame codec + client), `tools/plant/view_run.py`, `tools/plant/test_plant_stream.py`, `docs/experiments/mujoco-plant/stream-protocol.md`

**Interfaces:**
- `plant_demo_server --port 7700 --profile FILE --mode ideal|realistic`: one TCP listener (POSIX sockets, one client at a time, non-blocking send with a 4 MiB backlog cap: frames dropped and counted when a client is slow), Hello frame `{schema, site_checksum, scenarios}` at connect; Control JSON commands `{"cmd": "reset"|"set_goal"|"run"|"pause"|"step"|"move_body"|"list_scenarios", ...}`; `run` takes `speed` (1.0 paced; 0 unpaced).
- Python: `encode(frame_type, header, blocks) -> bytes`, `decode(buffer) -> (frame_type, header, blocks, consumed) | None` (same wire as Task 8); `view_run.py LOG` replays a run log in `mujoco.viewer.launch_passive` (robot from the header's model hashes + the committed MJCF, scenario boxes from the header, panel, movers; plans and scans as viewer `user_scn` geoms); `view_run.py --live HOST:PORT --scenario NAME [--goal x y yaw]` drives a session.
- `stream-protocol.md`: the wire format, frame types, every JSON field and float block layout, version 1.

- [ ] **Step 1: Tests:** `DemoServer.HelloOnConnect`, `DemoServer.ResetAndRunStreamsState`, `DemoServer.SurvivesClientDisconnect` (client closes mid-stream; a second client connects and receives Hello), `DemoServer.RejectsWrongSchema`; pytest `test_codec_matches_cpp` (bytes from a C++ fixture file written by a gtest equal Python's encoding), `test_decode_partial_buffer_returns_none`.
- [ ] **Step 2: Run** `CT mobile_manipulator_campaign DemoServer`, `PT tools/plant/test_plant_stream.py` → FAIL; implement; run → PASS.
- [ ] **Step 3: Manual check** with the container port published (`docker exec` cannot publish ports: if `7700` is not forwarded in `.devcontainer`, add it to the devcontainer's `forwardPorts`/compose ports and tell the user a container restart is needed): `python3 tools/plant/view_run.py --live localhost:7700 --scenario v2_door_105_nav` shows the robot driving through the door at 1×; replay of a Task 8 log shows the same run.
- [ ] **Step 4: Commit** `Add the demo server, its stream protocol and the host viewer`.

### Task 10: Targeted Unity lidar recording and L against Unity

**Files:**
- Create: `tools/plant/lidar_fixture_poses.yaml` (five site poses: open yard, beside the 1.05 m gate, inside door S, under the 1.23 m beam, by the scaffold; arm in carry), `tools/plant/compare_lidar.py`, `tools/plant/test_compare_lidar.py`, `docs/experiments/mujoco-plant/lidar.md`

**Interfaces:**
- `class_fractions(points_labelled) -> dict[str, float]`; Unity points get labels by nearest-geom lookup against the site boxes and the robot primitives at the recorded pose (`label_unity_points(points, boxes, robot_geoms, tol=0.03)`).
- `compare(unity_scans, plant_scans) -> dict`: per pose, class fractions (pass: within ±5 percentage points), self-hit region overlap (robot/panel points binned on a 5 cm voxel grid in the base frame; pass: Jaccard ≥ 0.7, a threshold to confirm at CP4), min-range and blind-zone violations (pass: 0).

- [ ] **Step 1: Tests** on synthetic point sets: `test_class_fractions`, `test_label_unity_points`, `test_compare_flags_fraction_gap`.
- [ ] **Step 2: Run** `PT tools/plant/test_compare_lidar.py` → FAIL; implement; run → PASS.
- [ ] **Step 3: Record Unity** (user present; Unity editor up, container running, no timed job): for each fixture pose, `scenario_place` with `--arm_pose carry` via the existing runner, then record 20 `/livox/points` scans (raw, before the self-filter) with `ros2 bag record` for 3 s; stop Play afterwards. Store under `experiment_runs/lidar-fixtures-<date>/`.
- [ ] **Step 4: Plant scans** at the same poses (Session, 20 scans, seed 1) and run `compare_lidar.py`; write `lidar.md` with the table.
- [ ] **Step 5: Commit** `Compare the plant lidar against Unity scans at fixture poses (L)`.

### Task 11: Campaign replicas (R) — CP4

**Files:**
- Create: `docs/experiments/mujoco-plant/corpus_campaign_b.json` (the 34 campaign-B runs' bags, from `campaign_b.sh`'s list and the Unity run directories of campaign B), `tools/plant/compare_campaigns.py`, `tools/plant/test_compare_campaigns.py`, `docs/experiments/mujoco-plant/replicas.md`, `docs/experiments/mujoco-plant/replicas.sh`

**Interfaces:**
- `unity_records(run_dirs) -> list[dict]` (via `medium_bench.campaign_runs` / `run_stats`, the windows of `campaign_a_table.md` and `campaign_b_table.md`).
- `agreement(unity, plant) -> dict` with the R pass conditions: per-cell ranking F2 ≥ B5 as on Unity; per-cell outcome match in ≥ 6 of 7 cells (campaign A); tight-passage min clearance within ±0.010 m of Unity's per cell (median over seeds and runs); no plant contact in a cell where Unity had none; plus the list of Unity runs where the collision monitor acted.
- `replicas.sh`: campaign A (7 cells × F2/B5 × 3 seeds) and campaign B (34 F2 runs), ideal and realistic, `--jobs` from the host's performance cores minus one.

- [ ] **Step 1: Tests:** `test_agreement_passes_identical`, `test_agreement_flags_ranking_flip`, `test_agreement_clearance_tolerance`, `test_agreement_contact_rule`.
- [ ] **Step 2: Run** `PT tools/plant/test_compare_campaigns.py` → FAIL; implement; run → PASS.
- [ ] **Step 3: Run** `caffeinate -i bash docs/experiments/mujoco-plant/replicas.sh`; write `replicas.md` (tables per mode, agreement verdicts, every disagreement with its investigation).
- [ ] **Step 4: Commit** `Replicate campaigns A and B on the MuJoCo plant (R)`.
- [ ] **CP4:** present the agreement table and each disagreement; fixes only with a physical reason, each as its own commit with a rerun of the affected cells.

- [ ] **Grip ensemble (CP3b, 2026-11-06):** on a subset of cells (the tight ones: door_105, low_beam_123, chicane,
  store_room), F2 and B5 at `contact_mu` 0.4 and 0.8 (effective track kept), >= 20 perturbed runs per cell and variant;
  report whether the per-cell outcome distributions and the F2/B5 ranking move with grip (commissioning_notes.md: the
  effective track's indirect effects).

### Task 12: P timings, visual check (V), docs and the tag — CP5

**Files:**
- Create: `docs/adr/0016-mujoco-campaign-plant.md`, `docs/experiments/mujoco-plant/acceptance.md`
- Modify: `docs/roadmap.md` (MuJoCo plant outcome, comparability log entry, decision log), `docs/running-the-stack.md` (a "MuJoCo plant" section: build, campaign runner, demo server, viewer)

- [ ] **Step 1: P** (cool host, no Unity Play; ask the user to check `sudo powermetrics --samplers thermal -n 1`): `plant_campaign` on three cells with `--lidar` off and on, with a `--profile-timing` flag that reports per-step plant cost and per-scan cost; pass: plant step ≤ 10 % of real time, scan p99 ≤ 25 ms. Record in `acceptance.md` with host and thermal state.
- [ ] **Step 2: V:** `view_run.py` on three logs (door 1.05, low beam 1.23, worker crossing) beside Unity screenshots at the same poses; the user signs off in chat.
- [ ] **Step 3: Docs:** ADR 0016 (context, options from the spec's decision table, decision, consequences: MuJoCo is a test plant and owns nothing in the live ROS stack yet; Unity ownership unchanged until spec 4's ADR); `acceptance.md` collects G, E, C, L, R, P, V verdicts with links; roadmap and running-the-stack updates.
- [ ] **Step 4: Verify** all suites: `CT mujoco_vendor`, `CT mobile_manipulator_plant`, `CT mobile_manipulator_campaign`, `CT mobile_manipulator_medium`, `CT mobile_manipulator_mpc`, `PT tools/plant tools/test_medium_bench.py` → all pass.
- [ ] **Step 5: Commit** `Accept the MuJoCo campaign plant: acceptance record, ADR 0016, roadmap and running-the-stack`; then `git tag mujoco-plant-accepted-<date>` (local).
- [ ] **CP5:** the user's sign-off; then the finishing-a-development-branch choice.
