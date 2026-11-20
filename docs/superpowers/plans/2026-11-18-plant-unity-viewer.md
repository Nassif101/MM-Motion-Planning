# Unity plant viewer implementation roadmap (spec 4)

Date: 2026-11-18. Branch: `plant-routing-unity-viewer`, from `e3-learned-margin` at `4d9a16f`.
Status: viewer requirements approved by the user on 2026-11-18. Spec 2 is complete (`968628d`, 2026-11-20).
The detailed viewer technical design and numeric performance workload are still to be completed in Task 0.

**This document is a staged roadmap, not an executable plan** (review, 2026-11-20). Task 0 produces the design.
Tasks 1–8 then become three compact implementation plans written to the superpowers writing-plans standard
(header, Global Constraints, Review Focus, file map, per-task interfaces, test-first steps, one commit per task).
Each plan delivers software that works and can be tested on its own (user, 2026-11-20):

- **4A, first demo:** the schema 4 essentials, the C# client and state store, the display rig, cameras, minimap
  goals and session controls (Tasks 1–4, cut to the data these need).
- **4B, inspection:** telemetry, the console and the overlays (Tasks 5–6); each overlay adds its own schema fields.
- **4C, qualification:** replay, performance measurement and the native players (Tasks 7–8).

Keep the plans compact: no data, settings or features that no approved requirement needs (user, 2026-11-20).

**Goal:** an interactive native-host Unity viewer of the ROS-free MuJoCo/F2/B5 backend, with destination
selection, inspection overlays and session controls. Unity remains optional for all backend operation.

**Inputs:** the approved [phase brief](../specs/2026-11-18-plant-routing-unity-viewer-brief.md),
[spec 2 design](../specs/2026-11-18-plant-route-planner-design.md), and
[spec 2 implementation plan](2026-11-18-plant-route-planner.md).
This roadmap depends on the spec 2 destination/route/session contracts; it is not a completed detailed spec 4 design.
Spec 3, new worker prediction and full 3D perception remain separate phases.

## Execution boundaries

- Run Unity natively through Unity CLI and `com.unity.pipeline`; never configure Unity MCP.
- Retain the existing HDRP project. Do not add navigation, physics authority or route generation to Unity.
- Backend state, accepted goals/routes, predictions, margins and outcome reasons are authoritative.
- New destination requests use the same backend API as headless campaigns; Unity does not own their lifecycle.
- Rendering and overlay settings cannot alter physics, observations, planning or research-run outcomes.
- Preserve the existing Unity/ROS simulator as a separately configured mode; viewer startup must not activate its publishers, sensors or actuators.
- Keep generated builds, captures and measurements outside committed source; never commit `Library/`, caches or host paths.
- Complete each task's checks before dependent work; record observed results separately from intended behavior.

## Execution process

- One commit per task on `plant-routing-unity-viewer`; never commit to `main`.
- After each task: commit, stop, and give the user one self-contained review prompt (the established scheme since E2);
  fix Critical/Important findings test-first before the next task.
- Record problems (problem/cause/resolution) in `docs/experiments/plant-viewer/`, and decisions in the roadmap's
  decision log (plus an ADR where one is warranted).
- Unity tests run through Unity CLI. Only an EditMode test assembly exists (`Assets/Tests/EditMode`); the design
  decides whether display-rig and lifecycle checks need a PlayMode assembly.
- Stop Play mode when a task finishes. The MacBook Air is fanless: never overlap Unity Play with timed runs, and use
  `caffeinate` for long jobs.

## Task 0: finish the viewer technical design and workload

**Read:** [viewer-handoff.md](../../experiments/plant-routing/viewer-handoff.md) (the backend contracts, measured
costs and open gaps), [stream-protocol.md](../../experiments/mujoco-plant/stream-protocol.md) (schema 3),
`mobile_manipulator_campaign` `goal_manager.hpp` and `goal_records.hpp`, ADR 0019, and the existing
`Assets/Scripts/Runtime/Control/ThirdPersonRobotCamera.cs`, `Runtime/ROS/RosMapPose.cs`,
`Runtime/Visualization/TelemetryWindow.cs` and `Editor/MobileManipulatorRosSetup.cs`.
Paths under `Assets/` in this plan are relative to `motion-planning-sim/`.

- [ ] Agree the design with the user (brainstorming, one decision at a time), then write
  `docs/superpowers/specs/2026-11-20-plant-unity-viewer-design.md`, preserving the approved requirements.
- [ ] Settle the brief's open points with the user: the brief's "MCP" read as MPC; whether the session controls include
  stop/cancel (no `cancel_goal` exists; the goal manager has a `canceled` outcome) and step.
- [ ] Decide the minimap's map source: the costmap is not streamed. Either stream the map, or read the same map file
  and check it against the session's checksum.
- [ ] Fix the reference hardware and rate profile (F2 at 2.5 Hz on the Mac, 5 Hz on the user's PC). Fix how the
  measurements run Unity Play and the container demo server together on the fanless Mac.
- [ ] Define the display rig, dedicated viewer scene/bootstrap, transport/replay adapters, lifecycle, reset/reconnect behavior and source of each overlay.
- [ ] Select JSON/codec facilities from the project's actual pinned dependencies; isolate parsing from Unity scene code and avoid unnecessary packages.
- [ ] Define reference hardware, resolution, HDRP settings, target/frame-time percentiles, warm-up and bounded measurement duration.
- [ ] Freeze workloads for scene-only, normal demo and maximum supported overlays: cloud count, ghost count, history lengths and log/event rate.
- [ ] Define the minimum live simulation speed and maximum displayed-state age separately from FPS; a smooth stale picture must not pass.
- [ ] Specify default view, minimap interaction, console behavior, replay controls and how users recognize stale/approximate data.
- [ ] Complete this technical design before Tasks 1–8 implementation; existing requirements approval remains in force.
- [ ] After the user approves the design, write plan 4A per writing-plans. Write 4B and 4C when their stage starts, so
  they build on what 4A measured.

**Exit:** a concrete design, protocol requirements and numeric workload exist. Any new scope or changed user behavior is identified explicitly; this task does not invent validation results.

## Task 1: extend backend display data and protocol compatibility

**Surfaces:** `mobile_manipulator_plant/{include,src}/frames.*`, `plant.hpp`, `plant.cpp`,
`mobile_manipulator_campaign/src/session.cpp`, `demo_server.cpp`, `tools/plant/plant_stream.py`,
`view_run.py`, their tests and `docs/experiments/mujoco-plant/stream-protocol.md`.

- [ ] Extend spec 2's schema 3 (Goal type 20, Route type 21) to schema 4; specify frame types, block layouts, units, limits and handshake before coding. Stage 4A adds only the fields 4A needs; each 4B overlay adds its own.
- [ ] Provide full chassis translation/orientation and actual wheel joint positions for the free-base model, with six arm joints and stable joint/link identity.
- [ ] Provide sufficient sensor acquisition transforms/timestamps for truthful point-cloud placement; do not transform a moving sweep solely with its final pose.
- [ ] Distinguish requested goal, raw global search result, accepted hierarchy route, selected F2 prediction, B5/MPC prediction and actual state.
- [ ] Include session epoch, goal/route/plan IDs, simulation timestamps, source/mode and data-validity flags in the applicable records.
- [ ] Supply backend-derived future arm/payload poses, active margin information and available rejected candidates/reasons; bound optional diagnostic detail.
- [ ] Define reconnect snapshot semantics without implicitly resetting a running headless session; preserve command acknowledgements and terminal events.
- [ ] Retain schema 1/2 readers under their original meanings. New clients reject incompatible live schemas clearly; do not reinterpret old recordings as full-pose data.

**Checks:** C++/Python golden codec records; fragmented/malformed/oversized input; schema mismatch; reset during delayed delivery; acquisition-pose fixtures; old log reading; no visual decimation affecting backend input.

## Task 2: C# transport, replay input and bounded state store

**Proposed files:** `Assets/Scripts/Runtime/PlantViewer/{PlantFrameCodec,PlantStreamClient,
PlantRunLogReader,PlantViewerState}.cs` and focused tests under `Assets/Tests/EditMode/`.
Use the existing runtime/test assemblies initially unless the technical design establishes a useful separate assembly boundary.

- [ ] Implement one shared decoder/state-application contract for TCP and replay sources.
- [ ] Keep network/decompression work away from Unity object mutation; apply validated values on the main thread through explicit references.
- [ ] Bound input sizes, queues and histories. Coalesce replaceable state/scan data without dropping required acknowledgements or reset/outcome ordering.
- [ ] Handle connect, disconnect, cancellation, scene reload and application shutdown with deterministic cleanup.
- [ ] Discard records from obsolete epochs/routes; retain measured snapshots separately from interpolated display values.
- [ ] Define overflow, stale-state and missing-prediction states and expose their counters for inspection.

**Checks:** fragmented frames, burst/slow-reader loads, reconnect, epoch changes, queued old predictions and lifecycle cleanup; no unbounded memory growth or Unity calls from worker callbacks.

## Task 3: renderer mode, display rig and ownership record

**Existing surfaces:** `Assets/Scenes/ConstructionSiteV2.unity`, existing robot/environment assets,
`Editor/MobileManipulatorRosSetup.cs`, `Runtime/Environment/ScenarioMover.cs`, camera and coordinate helpers.
**Proposed surfaces:** `Runtime/PlantViewer/{PlantViewerBootstrap,PlantRobotView,PlantEnvironmentView}.cs`,
an Editor setup command and a dedicated viewer scene/prefab constructed through Unity CLI/Pipeline.

- [ ] Build the viewer composition from existing V2 art and robot meshes while keeping the legacy simulator scene configuration available.
- [ ] Create a display rig whose transforms follow backend poses/joints without simulated articulation/rigidbody motion or independently advancing movers.
- [ ] Prevent Unity clock, odometry, TF, joint-state, arm transport, base controller, lidar and scenario publisher startup in this mode.
- [ ] Retain query-only geometry for camera/picking only if the detailed design needs it; it must not generate robot sensor/control feedback.
- [ ] Validate site checksum, robot/joint contract and map origin before displaying an active session; fail visibly on mismatch.
- [ ] Apply FLU-to-Unity conversion, complete base pose and wheel/arm/payload transforms against backend pose fixtures.
- [ ] Add a short ownership ADR and a mode-specific `AGENTS.md` amendment alongside this implementation: MuJoCo physics/lidar, framework planning/control, Unity rendering and explicit user requests.

**Checks:** static and moving pose comparisons; paused backend with freely moving camera; no Unity-generated state or commands; reset/reload; the legacy setup retains its own existing contract checks.

## Task 4: cameras, minimap destinations and session controls

**Reuse:** `ThirdPersonRobotCamera` and `RosMapPose`, adapting explicit targets for the display rig.
**Proposed files:** `Runtime/PlantViewer/{PlantViewerControls,PlantMinimap,PlantGoalPresenter}.cs`.

- [ ] Add the camera-perspective button and support existing orbit, chase and payload views where the rig permits them.
- [ ] Draw the minimap from backend map metadata/data, with robot pose, accepted goal and route; avoid deriving passability from rendered pixels.
- [ ] Map click/drag to position/final heading according to Task 0's interaction, then submit through the common backend goal interface.
- [ ] Show pending, rejected, accepted, superseded, reached and failed outcomes; never display a submitted destination as already accepted.
- [ ] Implement pause, step if retained by the design, speed and same-scenario/seed reset through backend controls.
- [ ] Keep camera/UI/inspection responsive while simulation is paused. Display requested and achieved speed; speed does not change physics timestep.
- [ ] Handle goal replacement during motion, pause, pending search and immediately after completion without stale markers or ghost plans.

**Checks:** pixel/world round trips, map-edge clicks, heading convention, invalid/replaced goals and paused controls; independently confirm backend accepted goal/state rather than relying on local UI state.

## Task 5: optional telemetry and diagnostic console

**Existing surface:** `Runtime/Visualization/TelemetryWindow.cs` mixes ROS/local simulator inputs and cannot simply be rebound as plant truth.
**Proposed files:** `Runtime/PlantViewer/{PlantTelemetryPresenter,PlantDiagnosticConsole}.cs`.

- [ ] Present backend planning state, goal progress, fallback/hold/failure reason and timing with explicit unavailable values.
- [ ] Add the togglable console with severity/subsystem filters, search, pause-autoscroll and copy/export as defined in Task 0.
- [ ] Distinguish backend simulation events from local transport/render messages; retain simulation and receipt times where relevant.
- [ ] Bound retained messages and pending event processing; report truncation/drop counts and avoid recursive log capture.
- [ ] Display FPS/frame time, stream age and achieved simulation speed without treating them as planner or experiment metrics.

**Checks:** malformed/missing telemetry, high-rate repeated errors, paused inspection and reset; sustained log load remains within Task 0 memory/frame budgets.

## Task 6: path, lidar, prediction and safety overlays

**Proposed files:** `Runtime/PlantViewer/{PlantPathOverlay,PlantCloudOverlay,PlantPredictionOverlay,
PlantMarginOverlay}.cs`; extend the state store rather than creating independent transport subscriptions.

- [ ] Render separately togglable global route, selected medium path, MPC prediction and actual travelled path, with clear legends and bounded history.
- [ ] Render lidar using acquisition-time coordinates, controllable density and bounded reusable buffers; retain the backend scan unchanged.
- [ ] Pool ghost robot/arm/payload render instances and color/sample future time explicitly. Superseded/stale predictions disappear or are visibly identified.
- [ ] Begin future-motion inspection with sampled ghost poses. A continuous swept-volume layer requires a defined construction/error bound; sampled ghosts are not labelled a complete collision envelope.
- [ ] Display active safety margins from backend geometry/configuration; distinguish predicted clearance, measured/ground-truth diagnostics and required margin.
- [ ] Show available failed/rejected candidates with their reasons and IDs. A search with no candidate yields a failure event rather than an invented path.
- [ ] Add optional collision-geometry inspection if retained by Task 0; acknowledge the known visual-art/exported-box mismatch in inspection data.

**Checks:** point-cloud pose fixtures, reset/goal-change invalidation, future pose versus backend samples, margin-source identity and missing candidates; all supported overlay combinations stay within frozen workload limits.

## Task 7: replay and performance qualification

- [ ] Run recorded inputs through the same state store and render presenters, with replay pacing/pause/reset independent of live backend control.
- [ ] Define legacy planar-log presentation explicitly; identify reconstructed/absent pose information instead of inventing measured chassis/wheel states.
- [ ] Measure the frozen scene-only, normal and maximum-supported overlay workloads after warm-up on recorded reference hardware/settings.
- [ ] Profile allocations, CPU/GPU frame time, queue age, render memory and receive/decode cost; optimize evidenced bottlenecks.
- [ ] Use timestamp-aware interpolation and bounded buffering for display; record raw values unchanged and avoid extrapolating unobserved robot motion as fact.
- [ ] Adjust visual density/ghost/history budgets through declared settings; rendering may discard replaceable display data, but required events retain their ordering.
- [ ] Repeat identical headless inputs with no viewer, normal viewer and stressed viewer to verify planning/observation independence under the declared deterministic mode.

**Exit:** measured frame-time, freshness, memory and backend-independence limits pass. If a requested workload fails, report it and make an explicit workload/implementation decision before claiming stable FPS.

## Task 8: native players and final integration evidence

- [ ] Build native macOS Apple Silicon and Windows x86-64 standalone players using installed host tooling/modules; derive all paths from the repository/build root.
- [ ] Validate connection configuration, scenario/reset flow, camera controls, minimap goals, pause/speed, diagnostics and replay in the player rather than only the Editor.
- [ ] Exercise full-hierarchy destinations through open space, detours, doors, overhead passages and unreachable goals with the spec 2 backend.
- [ ] Record precise platform/toolchain/settings and results in `docs/experiments/plant-viewer/`; unavailable platform runs remain explicitly unverified.
- [ ] Add user operating instructions and bounded qualification commands to `docs/running-the-stack.md`; update the roadmap with measured outcome and outstanding limitations.
- [ ] Keep player binaries and transient captures/logs outside committed source, with a reproducible build manifest and artifact locations recorded.

**Completion:** all approved viewer functions have implementation and evidence, the ownership record matches actual runtime behavior, and spec 2 remains fully usable without a viewer. A successful build alone is not cross-platform runtime validation.
