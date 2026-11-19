# Plant route planner implementation plan (spec 2)

Date: 2026-11-18. Branch: `plant-routing-unity-viewer`, from `e3-learned-margin` at `4d9a16f`.
Status: the user approved the [spec 2 design](../specs/2026-11-18-plant-route-planner-design.md) and
[viewer requirements](../specs/2026-11-18-plant-routing-unity-viewer-brief.md) on 2026-11-18.
This plan organizes execution; no runtime implementation or performance validation is claimed here.

**Goal:** our own ROS-free state-lattice planner supplies generated routes to F2/B5 on MuJoCo, accepts
replacement goals from Unity or headless clients, and supports the self-filter/persistent costmap milestone.
Spec 4 follows the [viewer roadmap](2026-11-18-plant-unity-viewer.md). Spec 3 remains deferred.

## Execution rules and task order

- Keep the routing core portable C++17. No ROS graph, Nav2 costmap, TF, ROS messages, Unity or bag dependency.
- Resolve robot geometry/configuration in adapters and pass plain data into the core. Existing workspace packaging is not proof of standalone portability.
- Work through the tasks in order; independent pure components may be delegated after their shared contracts are fixed.
- Run focused behavioral checks before dependent integration. Keep old recorded-route behavior explicit and covered.
- Use the development container for ROS/workspace builds and MuJoCo integration; use native-host Unity CLI/Pipeline for spec 4.
- Preserve E3's ruling and the separate diagnosis/E5 obligations. Review relevant results before fixing the final integration corpus; do not take over the other worktree's work.
- Keep build, benchmark, run and capture artifacts in ignored directories; shell scripts use LF and repository-relative path resolution.
- Record task outcomes and material decisions in the experiment report. Checkboxes mean verified completion, not files merely written.

There are three backend milestones: **A**, known-map search; **B**, live/headless destination execution;
**C**, acquisition-correct lidar filtering, persistence and base execution guard. All three complete spec 2.
The first Unity destination demo depends on B and Task 9's goal/route protocol; the branch completes C
before claiming spec 2 completion.

## Fixed starting decisions and source data

Read-only preflight inspected the installed Nav2 data on 2026-11-18, without running a simulation:

| Item | Observed value or planned policy |
|---|---|
| Reference package | `ros-jazzy-nav2-smac-planner`, Debian version `1.3.13-1noble.20260921.071143` |
| Primitive relative path | `sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json` |
| Primitive SHA-256 | `105b37351d15e340cc2921fa8878c8ea727a65cd7e89c39c977be0654be3b2a9` |
| File and metadata | 71,540 bytes; format version 1.0; 0.05 m grid; `diff`; turning radius 0.5 m |
| Heading/motion data | 16 nonuniform heading angles; 112 trajectories, seven per start heading; 32 pure rotations |
| Rotation cost input | Pure-rotation `trajectory_length` is zero; our search must add a positive angular cost |
| Licence metadata | Installed package declares Apache-2.0; retain the full applicable upstream licence and provenance when provisioning the asset |
| Initial goal policy | Preserve requested position/yaw; controller completion tolerances stay as configured (currently 0.15 m / 0.15 rad), without an extra 0.20 m approximate search endpoint |
| Amendment 1 (2026-11-19) | The search becomes a port of Smac Lattice 1.3.13 ([tasks S0-S6](#amendment-1-tasks-2026-11-19-port-smac-lattice)). Smac's 0.20 m search tolerance and its costs replace the rows above where they differ; the per-tick cell-work cap is removed |
| Search mode | Resumable deterministic work, stable successor/tie ordering, no reverse expansion |
| Starting search bounds | 256 expansions per 20 ms simulation tick; 1,000,000 total expansions; 256 MiB of tracked routing-owned storage |
| Work within expansions | Cap collision-cell work per slice as well; carry unfinished primitive/connector validation into the next slice rather than allowing one expansion unbounded work |
| Route checking | Periodic check at 1 Hz simulation time, plus new-goal/map-change triggers; input cancellation is handled at the next control boundary |

The search bounds are initial engineering limits, not measured throughput or a promise that 256 expansions
fit within 20 ms wall time. Task 0 freezes the cell-work limit and storage accounting before coding search;
Task 14 reports measured costs. Bound all queues and transient connector work as well as visited nodes.
Do not replace the heading array with uniform `2*pi/N` bins: this file's headings are not uniform.

## Task 0: freeze contracts, fixtures and evidence records

**Files:** create `docs/experiments/plant-routing/implementation.md`, a comparison-case manifest and the
configuration/asset manifest under the proposed routing package. Read current map, footprint, primitive,
controller, stepper, arm-command and session code before fixing interfaces.

- [ ] Record the exact primitive source/package version, hash and licence; provision the existing bytes without altering them. A fetch fallback must be pinned and hash checked, with an offline path.
- [ ] Freeze plain-value contracts for `Pose2`, `GridSnapshot`, polygon, `PrimitiveSet`, `SearchRequest`, `SearchResult` and revision IDs. Keep raw lattice poses separate from hierarchy routes.
- [ ] Define explicit outcomes, cancellation points and persistent search/checker cursors. Charge map, masks, nodes, frontier and owned buffers to memory limits; reject before exceeding them.
- [ ] Define translational cost in metres, non-straight multiplier 1.2 and normalized inflation-cost weight 1.0 as initial settings; pure rotation costs `0.5 m/rad * 5.0 * accumulated_turn_radians`. Accumulate angular travel from the unwrapped sample sequence, including the exact initial pose, rather than subtracting wrapped endpoint yaws. Do not import Smac's history-dependent change penalty without augmenting the search state.
- [ ] Use Euclidean distance as the initial lower bound; each translational edge cost must be at least its metric displacement. Declare any later heuristic weighting as a changed search policy.
- [ ] Fix the conservative primitive-sweep construction and an interpolation/dilation bound for arbitrary connectors. Set collision work per slice from bounded fixture work, independently of wall-clock outcomes.
- [ ] Define exact-pose start/goal connectors using valid forward/rotate-in-place motions to nearby lattice states. Use deterministic candidate order and a bounded neighbourhood; no lateral teleportation or unchecked snapping.
- [ ] Record the connector-policy identity with outcomes: `no_path` means exhaustion of that configured lattice/connector graph, not proof of continuous-space unreachability.
- [ ] Freeze a pose-only stationary-route contract for same-position heading goals, including zero-length projection/splice behavior, before touching F2/B5 consumers.
- [ ] Fix goal state transitions, hold/rearm behavior and generated-mode termination, including pause and future scheduled goals.
- [ ] Specify schema 3 additions: `Goal=20`, `Route=21`; existing State blocks retain their documented layouts. Goal/route records carry epoch, IDs, revisions, timestamps, requested pose, raw/accepted path identity and outcome. Add explicit capabilities for optional fields.
- [ ] Preserve schema 1/2 reading and the existing `frame_v1.bin`. A future incompatible viewer layout gets an explicit further schema/capability decision; do not silently mutate a frozen layout.
- [ ] Freeze paired correctness cases and route-quality/timing decision limits before running comparisons. Include open space, detours, narrow doors, overhead passages, off-grid goals and disconnected maps; save start/goal/footprint and asset hashes.

**Exit:** the record contains these choices and numeric limits with no unresolved runtime semantic.
Primitive ingestion can begin from the observed data above; a different installed version is not silently substituted.

## Task 1: standalone package and portable dependency boundary

**Create:** `ros2_ws/src/mobile_manipulator_routing/{CMakeLists.txt,package.xml}`, `include/mobile_manipulator_routing/types.hpp`,
`src/`, `test/`, `config/`, `data/` and a short package README. The core uses ordinary C++/Eigen and the
existing YAML/JSON libraries as needed; ROS packaging is an optional wrapper.

- [ ] Export a routing library with a standalone CMake configuration that does not require ament or ROS environment variables.
- [ ] Add the pinned primitive data/provenance/licence, a tiny synthetic primitive fixture and an effective configuration with explicit limits.
- [ ] Establish portable tests and installation paths; receive resolved asset paths rather than discovering ROS shares in the core.
- [ ] Verify both standalone and workspace configuration. Inspect linked dependencies rather than inferring portability from a successful workspace build.

**Checks:** standalone configure/build/CTest in an environment without sourced ROS; valid installation and asset lookup; unsupported/missing primitive version produces a clear startup failure.

## Task 2: strict map loading and layered grid

**Create:** routing `include/.../{grid,map_io}.hpp`, `src/{grid,map_io}.cpp`, `test/test_map_io.cpp` and `test/test_grid.cpp`.

- [ ] Support the V2 P5/8-bit trinary map contract, threshold/negate semantics, row inversion, negative origin and metre/cell conversion. Explicitly reject unsupported image modes/origin rotation until implemented.
- [ ] Validate dimensions and multiplication/length bounds before allocation. Preserve a first pixel whose byte is whitespace; the binary payload is not a whitespace-separated token stream.
- [ ] Keep known occupancy, observed occupancy and inflated cost separate. Unknown/outside is nontraversable; observation expiry cannot remove static occupancy.
- [ ] Produce immutable revisioned snapshots and dirty regions. Known scenario additions are explicit inputs, not an enumeration of MuJoCo bodies.
- [ ] Implement a documented inflation encoding/distance function with the reference radius/scaling; keep footprint validity distinct from soft cost.

**Checks:** asymmetric 3x2 image, exact threshold boundaries, edges/flooring, rotated-origin rejection, truncated/extra payload, integer overflow, 16-bit rejection, unknown cells and layer removal. Cross-check V2 pixel/world probes against the source map independently.

## Task 3: primitive parsing and complete footprint/motion checks

**Create:** routing `include/.../{primitives,footprint}.hpp`, `src/{primitives,footprint}.cpp`, corresponding tests and adapter footprint extraction.

- [ ] Validate heading array, IDs, index consistency, finite/ordered poses and metric lattice endpoints. Index the actual heading array, including wraparound; prepend/check the declared initial state when a first sample is not the origin. Do not subtract the first sample and shift the trajectory.
- [ ] Build the base polygon from the current projected base-link contract; apply geometric and costmap allowances once each. Keep the pure core independent of URDF resolution.
- [ ] Check polygon/cell intersection over the footprint interior, edges and boundaries; vertex-only and cell-centre-only checks are insufficient.
- [ ] Build conservative swept masks per primitive and use the same checked motion model for off-grid connectors. Make checker work resumable within the slice limit.
- [ ] Validate requested start/goal footprints before accepting search, including pure rotation and map boundaries.

**Checks:** occupied cell wholly inside a footprint, corner/edge contact, clear endpoints with a blocked intermediate sweep, rotating-corner collision, malformed primitive metadata and inconsistent map resolution. Compare sweep validity to an independent geometric fixture oracle.

## Task 4: deterministic resumable lattice A* and route output

**Create:** routing `include/.../search.hpp`, `src/search.cpp`, `test/test_search.cpp` and a small standalone route-check executable.

- [ ] Implement immutable-input search, fixed successor order, stable tie handling, valid decrease/reopen logic, positive rotation costs and bounded node/frontier storage.
- [ ] Implement `advance(work_budget)` with pending/found/invalid/no-path/budget/canceled/stale outcomes. Cancellation/revision checks occur during long collision work, not only after whole searches.
- [ ] Connect exact input poses to the lattice and reconstruct the complete raw pose sequence; preserve requested/resolved endpoint identity and unwrapped angular motion for sweep/cost checks.
- [ ] Return a tagged translating or stationary pose-path result. Keep pure rotations as pose-only results until Task 5 supplies the hierarchy's stationary-route representation; no artificial second XY point. Ordinary XY conversion retains final yaw and raw intermediate rotations in diagnostics.
- [ ] Add conservative optional simplification only after its validity tests; preserve the raw validated path if processing fails. Do not implement unneeded Smac features merely to copy parameter names.

**Checks:** Dijkstra comparison on small weighted graphs; known detour/disconnection; deterministic ties; cost-versus-length choice; off-grid connectors; rotation-only search; limit/cancel/revision outcomes; repeated searches with different slice sizes agree under equal total work.

**Milestone A:** standalone known-map destinations produce independently valid routes with no ROS/Unity/bag. This is not yet full hierarchy execution.

## Task 5: stationary-route and full-hierarchy heading support

**Modify:** MPC `route.{hpp,cpp}`, its formulation/reference consumers, medium planner/problem/splice consumers and focused tests.

- [ ] Add explicit `Route::stationary(anchor_xy, target_yaw)` or the Task 0 equivalent, retaining the ordinary polyline constructor's current rejection of degenerate input.
- [ ] Define zero-length `at`, projection and heading without segment indexing/division; audit every route-length/tangent/splice consumer.
- [ ] Make F2 yaw seeds/references and B5 final-heading control work on the stationary representation; do not insert an artificial translation or disable the full hierarchy.
- [ ] Preserve ordinary-route calculations and existing golden controller behavior.

**Checks:** finite costs/Jacobians, yaw wrapping, nonzero initial yaw rate, reached-A-then-heading-B, blocked rotation and successful yaw alignment within configured XY/yaw tolerances. Run existing route/formulation/medium-reference tests after this change.

## Task 6: shared goal manager and deterministic event journal

**Create:** campaign `include/.../goal_manager.hpp`, `src/goal_manager.cpp`, `test/test_goal_manager.cpp`.
**Modify:** `SessionConfig` with explicit route mode, goal source and simulation termination policy.

- [ ] Assign backend epoch/goal/request/route IDs and stable input sequence numbers. Validate malformed requests before preemption; accepted replacements supersede the old goal even if their search later fails.
- [ ] In generated mode, assign each accepted goal its acceptance timestamp and deadline, including time spent planning; retain the recorded mode's existing timing semantics.
- [ ] Implement duplicate request acknowledgement, reset invalidation, snapshot revision checks and current-state route revalidation before adoption.
- [ ] Archive per-goal outcomes, including superseded, without clearing prior history or treating preemption as a contact/failure.
- [ ] Keep goal sources and adapters separate from search/controller ownership; no viewer connection is required.

**Checks:** rapid A/B/C replacements, equal-time inputs, reset during search, duplicate messages, stale completions, changed map/footprint and invalid input leaving an existing goal intact.

## Task 7: persistent generated-goal execution and controlled hold

**Modify:** medium `closed_loop.{hpp,cpp}`, campaign `session.{hpp,cpp}`, `mujoco_plant.{hpp,cpp}`, and arm-command integration/tests.

- [ ] Add explicit generated-mode session advance/tick, separating running/paused/stopped from navigation status. Preserve legacy `step_cycle`/`active` terminal semantics for recorded-route callers.
- [ ] Keep plant/controller/executor objects persistent. Update an active route in place, rearm after terminal/idle state, and deliberately reset goal-local progress while retaining physical/command/tracking histories.
- [ ] Reactivate the executor's latched terminal state and reset the new goal's timeout/deadline, stall/progress window, result interval and applicable medium schedule against the one continuing session clock. An old goal's origin/cycle count must not determine the next goal's elapsed time.
- [ ] Clear superseded medium deliveries/RTI preparation, preserve valid same-goal carry-over, and start fresh F2 planning for a replacement destination.
- [ ] Advance physics/sensors/goal events while running-idle or awaiting search, using the controlled braking/arm hold path. A goal scheduled after earlier completion must still arrive.
- [ ] Implement a single command-boundary hold hook for both planning holds and the later observation guard. Copy the aliased controller result before changing it; a zeroed base Control with an `Ok`/`Fallback` CycleOutput still streams the old arm plan and is incorrect.
- [ ] Supply Holding/Blocked semantics to `ArmCommandPath`, cover ideal arm-velocity and realistic arm-stream modes, and feed actual sent commands back through `Controller::reset_command` and the existing reference/tracking path.

**Checks:** same-goal updates, reached/failed A then B, pending goal while idle, A completes then idles beyond its old timeout before B receives its own full interval, pause/step/reset, command continuity, arm hold actually sent, fallback cannot bypass hold, no stale plan execution. Existing recorded-mode numerical traces and same-build determinism checks remain applicable.

## Task 8: headless goal sources and common live controls

Revised 2026-11-19 after the Smac port (scope unchanged, design made concrete).

**Modify:** `plant_campaign.cpp`, `demo_server.{hpp,cpp}`, `plant_demo_server.cpp`, `tools/plant/plant_campaign.py`.
**Create:** `goal_script.{hpp,cpp}` (a bounded JSON goal-source contract and the headless driver) and tests.

- [ ] Goal script: JSON with `goals`, a list in which each goal has:
  - an `id`;
  - a `target` (x, y, yaw);
  - an optional `timeout_s`;
  - either `at_s` (simulation time, applied through the goal manager's schedule) or `after: "previous"` (submitted
    at the first control boundary after the previous script goal ends).

  It may also set `time_limit_s` and `end` (`idle`: stop when every goal has been submitted and none is active;
  `time_limit`: run to the limit). Unknown keys, nonfinite values and over-limit sizes are rejected.
- [ ] `plant_campaign --routes recorded|lattice` (default `recorded`; `lattice` refuses `--bag`) with
  `--goals FILE` and `--sim-time-limit S`. Without a script, lattice mode submits the scenario's destination.
  Generated mode steps with `Session::advance()`. The run completes by the script's policy, independent of
  intermediate goal outcomes. The summary records the route mode and the script's SHA-256.
- [ ] Demo server: `submit_goal`, `schedule_goal` and `request_replan` commands through the same goal manager, applied
  at control boundaries; `--routes lattice` steps generated sessions, which stay alive while idle. On disconnect
  the run pauses, as now; that is the demo's policy, separate from campaigns.
- [ ] Paced runs keep the simulation timestep; when the server falls behind it reports the achieved speed (an event)
  instead of changing the step.
- [ ] `tools/plant/plant_campaign.py run --routes lattice` passes lattice mode (no bag) to every run.

**Checks:**
- a scheduled first goal at t=5 while idle;
- successive `after: "previous"` goals;
- a replacement while moving and while planning;
- an unreachable replacement (the run still ends by its policy);
- the time limit;
- malformed scripts;
- demo commands without and with a connected client;
- paced and unpaced runs reaching the same decisions.

**Milestone B:** full hierarchy accepts/executes new and replacement destinations headlessly; known-map integration evidence and logs are available for the viewer.

## Task 9: schema 3 goal/route records and resume integrity

**Modify:** plant `frames.{hpp,cpp}`, `run_log.{hpp,cpp}`, session/server logging, `plant_stream.py`, `view_run.py`,
campaign resume checks and `docs/experiments/mujoco-plant/stream-protocol.md`.

- [ ] Implement Task 0's Goal/Route layouts, IDs/capabilities and schema handshake; make the reference Python tools understand the same records.
- [ ] Keep schema 1/2 readers and the byte-identical existing frame fixture. Schema/provenance additions do not imply whole old/new run-log byte identity; compare historical decision meaning separately.
- [ ] Include the routing inputs in run fingerprints: map, primitives, the footprint, the `Lattice` block and costmap
  settings, and the goal script. These close the gap that generated routes open; resume compares content hashes,
  not paths alone.
- [ ] Revised 2026-11-19, deferred: fingerprints of every other effective configuration (plant profile bytes, MJCF,
  B5/tracking/medium defaults, build identity), and log-write failure before the summary. They predate this
  branch, the viewer does not need them, and they are listed in the implementation report's open items.

**Checks:** C++/Python golden goal/route records, fragmented/malformed input and limits, old logs, deterministic new logs, changed file bytes at identical paths, changed installed defaults, truncated files and simulated log-write failure.

**Viewer handoff:** Milestone B plus this task provides the typed goal/route stream, compatible readers and
complete evidence records required for the first Unity destination integration.

## Task 10: acquisition-time observations and one scan owner

**Modify:** plant `lidar.{hpp,cpp}`, acquisition state capture in `plant.cpp`, session scan draining;
**Create:** plain observation-batch contracts and the campaign observation adapter.

- [ ] Capture sensor pose and measured robot/self-model state per physics cast batch; associate returns through time/batch indices without copying full transforms per point.
- [ ] Declare/localize frame and timestamp conventions; preserve the simulator ground-truth localization assumption explicitly where used.
- [ ] Drain scans once, distribute every observation to filtering/mapping, then apply independent optional log/stream decimation.
- [ ] Supply production filtering with a stripped type that does not expose `geom`/`cls`; retain those labels only in validation/diagnostic data.
- [ ] Bound acquisition history, count missing history and apply a defined out-of-order policy. Do not substitute end-of-scan/latest poses.

**Checks:** translated/rotated sensor and moving arm/payload fixtures, return-to-batch identity, missing history, no point loss caused by display decimation, and unchanged mapping inputs with slow/disconnected viewers.

## Task 11: shared geometric self-filter and persistent observed layer

**Modify/extract:** existing geometric filter/persistence helpers into a pure shared target; preserve ROS wrapper behavior.
**Create:** routing observation integration and focused tests.

- [ ] Pose the robot/payload self-model from acquisition-time measured state, reusing the existing ray/primitive semantics rather than oracle hit classes.
- [ ] Apply the base band 0.05–0.60 m and 5 m range, with initial confirmation/gap/decay of 2/1/10 s and costmap updates at 2 Hz simulation time.
- [ ] Validate persistence parameters/monotonic observation times and use stable cell ordering. Missing/no-return/shadowed rays are unobserved, not cleared cells.
- [ ] Update revisions and invalidate affected routes on confirmation/expiry. Keep known static occupancy and declared scenario/world inputs separate.

**Checks:** noisy self surfaces, an obstacle before a self surface retained, moving payload, transient crossing versus stopped blockage, expiry, out-of-order scans and simulation-speed invariance. Confirm labels cannot affect production results.

## Task 12: observed-base command guard and stopping qualification

**Create:** routing `base_motion_guard.{hpp,cpp}`, pure trajectory/footprint tests and campaign integration with Task 7's hold hook.

- [ ] Freeze guard horizon from command age/delay plus a conservative stopping envelope and declared uncertainty. Derive initial bounds from the active profile and qualify them on bounded MuJoCo braking/turn fixtures before enabling the lidar execution milestone.
- [ ] Check present footprint, proposed near-term base motion and braking sweep against confirmed observed occupancy. A failure/unavailable check requests hold and replanning; current intrusion or insufficient stopping room records an incident.
- [ ] Apply to every command/fallback path and retain the actual sent-command/arm-hold feedback contract. No oracle box injection into the planner world.
- [ ] Record proposed/sent command, guard reason, stopping bound, relevant cells and snapshot ID; preserve full 3D perception as later work.
- [ ] Use physics-rate contact/clearance observations in qualification fixtures. The current 50 Hz endpoint snapshots alone cannot certify absence of transient contact; use an independent substep recorder or an audited interval-contact implementation.

**Checks:** route detours while F2/MPC cuts toward the box, rotating corners, delay, fallback, intrusion, check failure and insufficient stopping distance. A two-second confirmation layer is not advertised as immediate transient-hazard protection.

**Milestone C:** all mapping/guard tests and bounded full-hierarchy persistent-blockage fixtures pass; limits are recorded. Spec 2's lidar scope is base-band routing/execution, not new 3D arm/payload perception.

## Task 13: experiment condition separation (revised 2026-11-19)

The route-level Nav2 comparison is done by Amendment 1's S5: the port is equal to the installed SmacPlannerLattice on
the qualification fixtures. What remains is keeping the experiment conditions apart:

- [ ] Run the V2 corpus once with generated (lattice) routes and once with recorded Nav2 routes, on the same plant
  profile and seeds. Report whole-hierarchy outcomes side by side, labelled by route source.
- [ ] Record in the comparability log that generated-route campaigns are a new condition. The recorded bags'
  routes came from the same planner with Nav2's costmap timing (map updates, TF and start-pose quantization);
  differences are reported, not tuned away.

**Exit:** both conditions are run and labelled; no claim beyond A1.5's equivalence.

## Task 14: final backend checks, documentation and viewer handoff

- [ ] Run focused routing, MPC/medium route-consumer, plant and campaign tests plus affected Python tools; use native arm64/amd64 container targets as available without forced emulation.
- [ ] Run the frozen headless multi-goal/reset/pause/replacement suite in deterministic mode, including runs with no viewer and stressed transport.
- [ ] Measure search, costmap and guard overhead per tick and paced throughput on the recorded host, with Unity Play
  closed during timed runs.
- [ ] Complete the standalone install/run instructions, effective configuration schema, source/licence manifest and run fingerprint documentation.
- [ ] Update `docs/running-the-stack.md`, roadmap outcomes and the implementation report with observed results, outstanding limitations and diagnostic/E5 dependencies.
- [ ] Hand accepted goal/route/observation/protocol contracts and fixtures to spec 4's technical-design task. The viewer must not create a second backend state machine.

**Completion:** milestones A/B/C have implementation and evidence; the portable core and headless goal path
work without ROS/Unity; historical formats/recorded mode remain explicit; spec 4 has concrete integration inputs.
No performance or planner-equivalence claim is established by writing this plan.

## Amendment 1 tasks (2026-11-19): port Smac Lattice

[Spec 2 Amendment 1](../specs/2026-11-18-plant-route-planner-design.md#amendment-1-2026-11-19-the-route-search-is-a-port-of-nav2s-smac-lattice-planner)
and [ADR 0019](../../adr/0019-route-search-ports-smac-lattice.md) replace the search internals of Tasks 2-4. These
tasks run after Task 7 and before Task 8, one commit each.

Sources:
- `navigation2` tag `1.3.13`: `nav2_smac_planner` and `nav2_costmap_2d`;
- OMPL 1.7.0's `DubinsStateSpace`.

The container has `ros-jazzy-nav2-smac-planner` and `ros-jazzy-nav2-costmap-2d` 1.3.13 and `ros-jazzy-ompl` 1.7.0.
Ported files keep their upstream copyright headers and carry a "ported from" line naming the upstream file.

The equivalence checks need Nav2, so they live in a new container-only test package,
`mobile_manipulator_routing_nav2_check`. The routing core stays portable: it has no Nav2, OMPL or ROS dependency.

### S0: provenance and licences

- Record the upstream commit, file list and SHA-256 of every source the port follows in `implementation.md`.
- Add `third_party/NOTICE.md` to the routing package, with the Apache-2.0 and BSD-3 texts.

### S1: Nav2 costmap semantics

Port these behaviors into the core as an 8-bit cost layer built from the grid snapshot:
- the static layer's map interpretation;
- `padFootprint`;
- the inscribed and circumscribed radii;
- the inflation layer, including its cell-distance cache and the order in which costs spread.

The check package compares the port with `nav2_costmap_2d`'s static and inflation layers byte for byte. The cases are
the V2 map and a seeded box map, each with the `home` footprint and with the V2 base footprint.

### S2: collision checker

Port `FootprintCollisionChecker` (edge line costs) and Smac's `GridCollisionChecker` (72 bins, the centre-cost
shortcut). The check package compares collision outcomes and costs with the originals at seeded random poses on the
V2 map.

### S3: heuristics and Dubins curves

Port these:
- OMPL's Dubins distance and interpolation;
- `NodeLattice::precomputeDistanceHeuristic` and `getDistanceHeuristic`;
- `NodeHybrid::resetObstacleHeuristic` and `getObstacleHeuristic`, including Lattice's defaults: no downsampling and a
  linear penalty.

The check package compares Dubins distances with OMPL and obstacle-heuristic values with Smac's. The lattice file is
parsed into Smac's float structures with nlohmann/json, as Smac does. The existing loader's validation stays.

### S4: planner and integration

Port these:
- `NodeLattice`: validity and its cache, traversal cost, neighbors, backtrace;
- `AnalyticExpansion`;
- `AStarAlgorithm::createPath`, as a loop that can resume between iterations;
- `createPlan`'s conversion, the same-cell case and duplicate removal;
- the `Smoother`.

Put them behind the existing `Search` interface with typed outcomes and the validation pass of A1.2. Then:
- Wire the session to it.
- Remove `FreeSpaceHeuristic`, the old A* internals and our soft-cost field.
- Mirror the `Lattice` block in `config/routing.yaml`.

Portable tests:
- every outcome type;
- any slicing of the expansion budget gives the identical result;
- the same-cell and heading-only cases;
- a validation rejection that falls back to the raw path.

### S5: end-to-end equivalence

The check package runs `nav2_smac_planner::SmacPlannerLattice` and the port on the same costmap. The fixtures are:
- every V2 scenario start/destination pair;
- 200 seeded random free pairs;
- unreachable and occupied goals;
- goals reached within the tolerance.

Pass: in every case, the same outcome and iteration count, and raw and smoothed poses within 1e-4 m and 1e-4 rad.
Report each mismatch with its cause; the qualification fails unless A1.2 explains the mismatch.

### S6: plant measurements and hand-over

- Re-measure the V2 scenario destinations in a generated plant session: simulation-time planning hold, iterations and
  wall time.
- Review the 256-iteration per-tick budget against those measurements, and record any change as a decision.
- Update `implementation.md` and the handoff, and write the S1-S6 review prompt.
