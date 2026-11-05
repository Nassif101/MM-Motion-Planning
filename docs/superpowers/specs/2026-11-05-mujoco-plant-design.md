# MuJoCo campaign plant design: a ROS-free plant with lidar (spec 1 of 4)

Date 2026-11-05. Branch `mujoco-plant` off `main` (692a9c4: Phase 5A merged, F2 frozen at the local tag
`f2-frozen-2026-11-02`).

## 1. Purpose and scope

The roadmap's step 2 after Phase 5A ("Priorities after Phase 5A and the simulation test bed"; "MuJoCo campaign plant
(plan)") is a fast, deterministic, CPU-only test plant. It runs the framework (B5, F2) in-process, headless and without
ROS 2. It is where the execution fixes (step 3) are developed and where later campaigns run beside Unity.

The work is split into four specs on one phase branch, built in order (user decision 2026-11-05):

1. **Plant + lidar (this spec):** generated model, robot profile, ideal and realistic modes, calibration, the lidar
   model, the session API, the stream and run log, and the acceptance gate.
2. **Costmap + route planner:** our own ROS-free route source (an SE(2), footprint-aware lattice or hybrid A* on our own
   costmap: known map, lidar marks with persistence, inflation, replans), including the self-filter port. It replaces
   recorded Nav2 routes in ROS-free runs.
3. **ROS bridge:** real Nav2 and MoveIt running against MuJoCo on sim time.
4. **Unity viewer executable:** a standalone player build (macOS, Windows) as a pure client of the demo server, with
   scenario selection, reset, start pose, click-to-goal and overlays (lidar, route, plans, panel sweeps, clearance).
   It needs an ADR, since it changes AGENTS.md ownership for that mode.

### Decisions taken in the design discussion (2026-11-05)

| Topic | Decision |
|---|---|
| Scope of this spec | Stage 1 (headless in-process plant) **with the lidar model now**: the live stack's route already depends on lidar (`/livox/points_filtered` feeds the global costmap's persistent layer, the local costmap, the collision monitor and MoveIt's Octomap) |
| Final baselines ROS-free | Your idea: B1-B4 as our own configs on the plant. Risk raised: re-implementations are "Nav2-like", so reviewers could call the baselines weakened. Decided at the final baseline run. Requirement now: the plant's stepping, sensors and clock must not assume their driver |
| ROS bridge | Its own spec (3) once the plant is accepted |
| Unity | Renderer and demo client only, discussed after this plant is tested (spec 4). This spec carries the session API, the stream schema and the run log so the viewer can attach live or replay |
| Base model | Option C: planar base with a calibrated drive model now. A wheel-contact skid-steer model may be added later behind the same interface, selected by the profile |
| Route source | Own planner (spec 2). Recorded Nav2 routes stay as an **acceptance-only** input: the cross-simulator check needs identical routes |
| Calibration source | Option A: fitted from existing Unity recordings, validated on held-out runs; datasheets or short targeted runs only for gaps |
| Architecture | Approach A: a plant core library behind one step API, with thin drivers (campaign runner, demo server, later the bridge) |
| Lidar pattern | The same Livox Mid-360 pattern file and noise function as Unity |
| Robot as data | URDF + robot profile; built for the custom robot now, the Husky A300 + UR5e later as its own mini-phase |

### Non-goals

- The route planner, costmap and self-filter (spec 2); the ROS bridge (spec 3); the Unity viewer (spec 4).
- Moving B5 and F2 onto the robot profile (Husky mini-phase). They keep their configs; a consistency test ties them to
  the profile.
- Odometry noise and wheel-contact physics (added only if calibration or the execution fixes show they matter).
- Pooling MuJoCo with Unity results: never. MuJoCo results are their own profile.

## 2. Packages and interfaces

### 2.1 New packages

- **`mujoco_vendor`** (ament CMake, like `daqp_vendor`): one official MuJoCo release pinned by version and SHA-256,
  for linux arm64 and amd64. Exports `mujoco::mujoco`. The version is the newest release at least two weeks old when
  Task 1 runs; it must include the `mjSpec` API and `mj_geomDistance`.
- **`mobile_manipulator_plant`** (C++17, ROS-free: depends on `mujoco_vendor`, Eigen and yaml-cpp only). It knows nothing
  about B5, F2 or ROS messages.
  - `RobotProfile`: loads the profile YAML and the files it references.
  - `Plant`:
    - `reset(scenario, start_pose, seed)`
    - `step(const PlantCommand &, double dt) -> PlantState`
    - `scan() -> std::optional<PlantScan>` (when a scan completed)
    - `ground_truth() -> GroundTruth` (contacts, true minimum clearance per body with witness points)
    - `move_body(id, pose)` (mocap movers and appearing obstacles)
  - Internal units: `BaseDrive`, `ArmServo`, `Lidar`, `RunLog`.
  - `PlantCommand`: the base twist (v, ω) and an arm reference, either streamed points (t, q, q̇) as the JTC receives
    them or joint velocities.
  - `PlantState`: time, base pose and twist, q and q̇, panel pose.
- **`mobile_manipulator_campaign`**: the only package that depends on both the plant and the framework.
  - Adapters between `mmm::State` / `Control` / `CycleOutput` and `PlantCommand` / `PlantState`. The arm stream points
    are built with the existing `arm_stream` logic.
  - `Session` (Section 6.1), used by both drivers.
  - `plant_campaign`: the campaign runner executable (unpaced, one run per process, parallel across cores).
  - `plant_demo_server`: the demo server executable (Section 6.3).
  - `plant_replay`: recorded commands in, achieved trajectory out (calibration, Section 7).

### 2.2 Changes to existing code

- `run_closed_loop` (`mobile_manipulator_medium`) takes a `ClosedLoopPlant` interface declared in
  `mobile_manipulator_mpc`: `state()`, `executed()`, `at_rest()`, `step(command, cycle_output, dt)`.
  `KinematicPlant` implements it, and the existing gtests and bench results stay unchanged. The MuJoCo adapter in the
  campaign package implements it as well.

### 2.3 Python tools (`tools/plant/`)

- `generate_site_model.py`: boxes export + scenario → site MJCF and scenario includes.
- `generate_robot_model.py`: URDF + profile → robot MJCF.
- `calibrate_plant.py`: `base` and `arm` fits from the Unity recordings.
- `view_run.py`: the host viewer through the Python `mujoco` package; replays a run log, or `--live host:port`.

Generated MJCF files are committed, following the existing `mobile_manipulator.py` → `.urdf` pattern. A test
regenerates them and fails on any difference.

## 3. Model generation and the robot profile

### 3.1 Site model

- Every entry of `construction_site_v2.boxes.yaml` (604 boxes: name, center, size, yaw) becomes a static box geom
  with the export's name, center, half-size and yaw quaternion. The ground plane at z = 0 is added for lidar ground
  hits. Nothing is placed by hand.
- The export's checksum (map metadata) and the generator version go into the MJCF as custom text. The plant reports
  both in every run summary.
- Each scenario gets a generated include. Its static boxes come from `scenarios_site_v2.yaml`. Movers and obstacles that
  appear mid-run are mocap bodies reserved in the model and moved by `move_body`.

### 3.2 Robot model

The generator reads the URDF (the generated `mobile_manipulator.urdf`) and the profile and writes:

- **Base:** a chassis body on planar joints (slide x, slide y, hinge yaw) at its fixed height, with the URDF's masses
  and inertias. The wheels are visual only (no contact) in this base model.
- **Arm:** the URDF's six revolute joints with their axes, limits, masses and inertias. Armature and damping come from
  the profile. The actuators are torque motors with force ranges from the URDF efforts; `ArmServo` computes the torque.
- **Collision geoms:** the URDF's collision primitives (chassis box, wheel cylinders, link boxes and cylinders), the
  set B5's primitives come from. The **panel** is a box welded to `tool0` with the payload's size and mass (3 kg).
- **Visuals:** the description's STLs, used in the viewer only.
- **Lidar site:** at the URDF's `livox` frame.
- **Contacts:** robot–world and panel–world on, with a physical response, and every contact logged as ground truth.
  Arm self-contacts off, except panel against base and chassis; MuJoCo's default parent–child exclusion applies.

At `reset`, the plant composes robot + site + scenario in C++ through `mjSpec` (attach, compile). There is one robot
file and one site file, not one file per scenario.

### 3.3 Robot profile

`mobile_manipulator_plant/robots/mobile_manipulator/profile.yaml` holds the joint order and limits, the drive model
parameters, the servo gains, the delays, the torque limits and the lidar parameters, with `ideal` and `realistic`
sections. It **references** existing sources instead of copying them: the URDF, `qualified_payload.json` (qualified
poses, payload) and the torque limits. A **consistency test** checks the profile against B5's and F2's configs: joint
order, limits, collision primitives and payload.

## 4. Plant modes and the low-level layer

### 4.1 Timing and determinism

- Physics runs at a fixed 2 ms step with the `implicitfast` integrator, single-threaded. `BaseDrive` and `ArmServo`
  update every physics step, the lidar at 10 Hz, and the framework at its own rates (B5 50 Hz; F2 2.5 or 5 Hz).
- One seed drives every random source (lidar noise, dropouts, any process noise). The same seed on the same binary
  gives a bit-identical run, run log included.
- Latency is configurable in both directions: a command delay (framework → actuators) and a state delay (sensors →
  framework), as FIFOs quantized to the physics step.

### 4.2 Ideal mode

- The base and arm follow the command exactly, with no delay. The planar and arm joints are set kinematically every
  step from the integrated command. Collision detection still runs, so contacts and penetration are logged but don't
  push the robot.
- An optional breakaway switch reproduces `KinematicPlant`'s static friction. Ideal mode with breakaway is the
  equivalence reference (E1, Section 8).

### 4.3 Realistic mode: `BaseDrive`

- The command (v, ω) passes through:
  1. the smoother limits (velocity, acceleration);
  2. breakaway (from rest, the base doesn't move until |v| or |ω| exceeds its threshold);
  3. a first-order lag (τ_v, τ_ω);
  4. slip (achieved = k · target, separately for linear and yaw).
- A stiff, force-limited velocity servo applies the target twist on the planar joints in the body frame, with the
  lateral velocity driven to zero. The force limit is a traction limit μ·m·g, so arm reactions and obstacle contacts
  produce small physical sideslip and pushback.

### 4.4 Realistic mode: `ArmServo`

- Streamed points are sampled every step as the JTC samples them (cubic Hermite), giving q_r, q̇_r and q̈_r.
  Joint-velocity commands are integrated into q_r instead.
- τ = Kp(q_r − q) + Kd(q̇_r − q̇) + gravity feedforward + motion feedforward. Both feedforward terms come from
  MuJoCo's inverse dynamics (`mj_rne`, payload included). τ is clamped to the URDF efforts.
- The gains start from Unity's ArticulationBody drive stiffness and damping; calibration refines them. Each
  feedforward term can be switched off, for the tracking-aware B5 work in step 3.

### 4.5 State reported to the framework

The true MuJoCo state (base pose and twist, q, q̇), passed through the state delay. No odometry noise in this spec.

## 5. Lidar

### 5.1 Pattern and parameters

- Livox's Mid-360 pattern, `mid360.csv` (800,000 points of time, azimuth and zenith; MIT licence, Livox). It is 25 MB
  and is not committed. The build fetches it from the UnitySensors commit the Unity project pins (`bc8b1d6445e1`),
  checks its SHA-256 and converts it to a compact binary.
- The profile carries today's Unity values: 10 Hz, 20,000 points per scan, minimum range 0.1 m, maximum range 70 m,
  Gaussian range noise σ = 0.02 m. Each scan advances a 20,000-point window through the pattern, as UnitySensors does.
- The noise is a port of `LidarRangeNoise.Sample(seed, scan, index, σ)` (`IidNoiseRaycastLiDARSensor`): i.i.d. per point
  and scan, and seeded.

### 5.2 Casting

- Each physics step casts its slice of the window (400 rays per 2 ms step) from that step's sensor pose, so motion
  distortion within a scan appears as in Unity. A scan is emitted at the end of its window, in the sensor frame, with a
  per-point time offset.
- Rays hit everything in the model: site and scenario boxes, movers, the ground, the robot's links and the panel.
  Self-hits are real; the plant doesn't filter them (the self-filter is spec 2).
- Dropouts: a per-point probability, 0 by default (Unity has none). A datasheet-based `realistic+` value is an option
  for the perception step and is never used in the cross-simulator checks.
- Each point carries a ground-truth label: the geom hit and its class (site, scenario, mover, ground, robot, panel).
- `lidar: off` skips casting for campaigns that don't consume it (today's F2/B5 runs on recorded routes).

### 5.3 Performance decision

That is 200,000 rays per simulated second against about 650 geoms. Task 5 benchmarks `mj_multiRay`. If a scan costs more
than 25 % of its 100 ms period, the static geoms get a BVH ray caster built from the **compiled MuJoCo model's geoms**
(one geometry source), and MuJoCo's rays still handle the robot, panel and movers. G4 tests both paths.

### 5.4 Output

`PlantScan`: start and end stamps, points (x, y, z in the sensor frame, time offset), labels. It goes to the framework
and to the run log at a configurable decimation.

## 6. Session API, stream and run log

### 6.1 Session API (`mobile_manipulator_campaign`)

- `open(profile, mode, config)`, `list_scenarios()`, `reset(scenario, start_pose | default, seed)`, `set_goal(pose)`
- `run(speed)` paced at a factor of real time, or unpaced
- `pause()`, `step(n)`, `move_body(id, pose)`
- Events: goal reached, failed, contact, timeout, fallback.

### 6.2 Frames (one schema for the stream and the log)

- **State**, every B5 cycle (50 Hz): time, base pose and twist, q and q̇, panel pose, mover poses, minimum true
  clearance with witness points, active contacts, B5 status.
- **Plans**, at the planners' rates: the route, B5's horizon (base poses and q per stage), the medium plan's samples,
  and the panel pose along both plans (forward kinematics on the server, so clients draw panel sweeps without the
  robot's kinematics).
- **Scans**: points and labels, decimated (every scan, every Nth scan, or off).

### 6.3 Stream protocol (`plant_demo_server`)

- TCP, length-prefixed frames `[u32 length][u16 type][payload]`; the schema version is checked at connect.
- Control and event messages are JSON. Bulk data (points, plan arrays) are little-endian float32 blocks with a
  documented layout. No dependency on either end (C++, C#, Python).
- `view_run.py --live` is the reference client: it drives a session and shows it in MuJoCo's passive viewer on the
  Mac, proving the protocol before Unity connects (spec 4).

### 6.4 Run log and run summary

- **Run log:** a header (schema version, git commit, profile and generated-model hashes, site checksum, scenario, seed,
  mode, full config) followed by the same frames, zlib-compressed in chunks. `view_run.py` replays it.
- **Run summary:** a separate small JSON in the shape `medium_bench.py` and the summarize tools already read, plus the
  plant profile (`mujoco-ideal` / `mujoco-realistic`), the host, the site checksum and the seed.

## 7. Calibration

### 7.1 Data

- Recent whole-body runs log `/cmd_vel`, `/cmd_vel_smoothed`, `/odom`, `/joint_states`, the arm stream
  (`/arm_controller/joint_trajectory`) and `/arm_controller/controller_state` (reference, feedback, error), all on
  `/clock`. No lidar clouds are recorded.
- The training set is the runs with today's Unity actuator configuration only (`gff-mff` arm feedforward and the
  current base controller, Phase 4B-C and 5A). It is split by cell: about 80 % for fitting, 20 % held out.

### 7.2 Fits

- **Base (`calibrate_plant.py base`):**
  - smoother limits copied from the velocity-smoother config, not fitted;
  - breakaway from the Task 1 (Phase 5A) and ADR 0003 measurements, cross-checked against turn-from-rest events;
  - command delay by cross-correlation, then lag (τ_v, τ_ω) and slip (k_v, k_ω) by least squares on
    `/cmd_vel_smoothed` → `/odom` twist.
- **Arm (`calibrate_plant.py arm`):**
  - delay by cross-correlating reference and feedback in `controller_state`;
  - PD gains starting from Unity's drive values, refined by replaying the recorded streams through `plant_replay`
    and minimizing the per-joint error difference from Unity.

### 7.3 Validation on held-out runs (open-loop replay of the recorded commands)

| Metric | Target |
|---|---|
| Arm tracking error p95 per joint | within ±30 % of Unity's, or within 2 mrad, whichever is looser |
| Base speed RMSE | ≤ 0.02 m/s |
| Base yaw-rate RMSE | ≤ 0.03 rad/s |
| Pose drift per 2 s window | ≤ 0.03 m |
| Delays | match the cross-correlation estimates within one physics step |

CP3 shows you the fit and residuals before anything is frozen; the targets may be revised there with reasons.

### 7.4 Gaps

- Lidar acceptance against Unity needs Unity scans at a few fixed poses: one short targeted recording (Task 10).
- A base turn-from-rest sweep, only if the bags lack the excitation.

### 7.5 Output

The calibrated values go into the profile's `realistic` section with provenance (runs used, date, residuals). The
report goes to `docs/experiments/mujoco-plant/calibration.md`.

## 8. Acceptance and the gate

A plant that disagrees is not accepted.

**G: geometry**
- **G1 Read-back:** the compiled model's 604 box geoms match the export (center, size, yaw within 1e-6), and the
  checksum matches; the same for every scenario's boxes.
- **G2 Map:** the MuJoCo geoms rasterized at 5 cm in the exporter's height band match the Unity-exported `.pgm` cell
  by cell. Mismatches are allowed only where a cell center lies on a box edge, each one listed.
- **G3 Distances:** `mj_geomDistance` and B5's FCL distances agree within 1 mm over about 10,000 sampled robot states
  near obstacles (primitives and panel against boxes).
- **G4 Rays:** with noise off, every ray (MuJoCo and BVH paths) agrees with an exact analytic ray–box intersection
  (within 1e-6 m, same geom hit).
- **G5 Fixtures:** the 1.05 m gate and doors and the 1.23 / 1.30 m beams in the compiled model match the scenario
  definitions.

**E: equivalence and determinism**
- **E1:** ideal mode with breakaway reproduces the kinematic bench's closed loop on its 7 cells: same outcomes, and
  trajectories within numerical tolerance.
- **E2:** the same seed gives bit-identical runs and run logs.

**C: calibration.** Section 7.3's targets on held-out runs.

**L: lidar**
- Noise-free ranges are exact (G4).
- With noise on, the range residuals match N(0, 0.02) (KS test).
- At the targeted Unity poses, the hit fractions per class (site, ground, robot, panel) match Unity's within ±5
  percentage points, and the self-hit regions on the arm and panel match.
- The minimum range and the blind zone are respected.

**R: cross-simulator agreement** (realistic mode, the recorded routes, the planners' latency fixed at the 2.5 Hz
profile for determinism)
- Replicas of campaign A (7 cells × F2/B5 × 3 seeds) and campaign B (34 F2 runs), in ideal and realistic modes.
- Pass conditions (realistic mode):
  - the same ranking as Unity (F2 ≥ B5 per cell; on Unity, F2 20/21 vs B5 12/21);
  - per-cell outcomes match in at least 6 of 7 cells;
  - the smallest clearances at the tight passages are within ±10 mm of Unity's;
  - no contacts where Unity had none.
- Disagreements are investigated and recorded. They are fixed only with a physical reason (model or calibration),
  never by tuning until the numbers agree.

**P: performance**
- Plant step (physics, drive, servo) ≤ 10 % of real time.
- A lidar scan ≤ 25 % of its period (else the BVH path, Section 5.3).
- Campaigns run unpaced and in parallel; the plant's real-time factor is reported per run. P timings are taken on a
  cool host with no Unity Play running.

**V: visual.** You check a few runs in MuJoCo's viewer against Unity screenshots at the same poses.

**The gate:** G, E, C, L, R and P pass, plus your visual sign-off. Then the plant is accepted, tagged
`mujoco-plant-accepted-<date>` (local), and recorded in the roadmap (outcome, comparability log) and an ADR.

## 9. Staging and checkpoints

One commit per task on `mujoco-plant`.

1. `mujoco_vendor` and a load-a-model smoke gtest in the container. On the Mac host, `pip install mujoco` for the
   viewer (with your OK).
2. Site generator; G1, G2, G5.
3. Robot profile and robot generator; the consistency test; G3.
4. Plant core in ideal mode; the `ClosedLoopPlant` interface with `KinematicPlant` behind it; the MuJoCo adapter; E1,
   E2. **CP1:** the ideal plant equals the kinematic bench.
5. Lidar: pattern fetch, casting, noise, labels; G4 and the lidar statistics. **CP2:** the lidar benchmark and the BVH
   decision.
6. Realistic mode: `BaseDrive`, `ArmServo`, delays, `plant_replay`.
7. Calibration: `calibrate_plant.py`, the fit, held-out validation, the report. **CP3:** you review the fit.
8. Run log, `Session`, `plant_campaign` (parallel), summary fields.
9. `plant_demo_server` and `view_run.py` (replay and live).
10. Targeted Unity lidar recording (a few minutes; Play stopped afterwards) and the L comparison against Unity.
11. Campaign A and B replicas (R), ideal and realistic. **CP4:** R results and your rulings on any disagreement.
12. Visual check (V); docs: ADR, roadmap outcome and comparability-log entry, running-the-stack section; the
    acceptance tag. **CP5:** your sign-off.

Effort: about two weeks of sessions; the checkpoints are natural stopping points. The execution style is chosen when
the plan is approved.

## 10. Risks

- **Fidelity:** MuJoCo's drive and contact models are not PhysX's. Mitigation: calibration (Section 7), the ranking and
  clearance checks (R), and separate result profiles.
- **Lidar cost:** mitigated by the benchmark and the BVH path (Section 5.3).
- **The pattern source** (an external file at a pinned commit) may move. Mitigation: SHA-256 check; the fetch fails
  loudly; the Unity project's package cache holds the same file as a fallback source.
- **Calibration data coverage:** today's actuator configuration has fewer runs than older ones. If the held-out
  validation is thin, Task 7 records it and CP3 decides on a targeted Unity session.
- **Host load:** parallel campaigns on the fanless Mac. Results are unaffected (fixed latency, deterministic); only P
  timings need a cool host.
