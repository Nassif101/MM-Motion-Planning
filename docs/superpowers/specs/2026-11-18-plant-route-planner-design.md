# Plant route planner, costmap and lidar self-filter (spec 2)

Date: 2026-11-18. Branch: `plant-routing-unity-viewer`, from `e3-learned-margin`
at `4d9a16f8da23a04185c1fe75858ced7831647b36`. Status: design approved by the user on 2026-11-18;
implementation and validation have not started. Read with the
[phase brief](2026-11-18-plant-routing-unity-viewer-brief.md), which records the user's decisions and the
approved spec 4 viewer requirements. Execution is organized by the
[spec 2 implementation plan](../plans/2026-11-18-plant-route-planner.md).

## 1. Purpose and agreed scope

Give the MuJoCo plant its own C++ state-lattice route planner, costmap and geometric lidar self-filter,
operating without ROS 2 or Unity. A destination supplied during execution replaces the current destination
and triggers planning from the current robot state. The first interactive application is the full hierarchy:
global route provider, F2 medium-horizon planner, B5/MPC, then MuJoCo execution.

Unity is one optional goal client. Headless campaigns, scenario goal sources and command-line clients use
the same backend contract. A generated-route campaign needs destinations rather than recorded Nav2 paths;
goals can also arrive during a run. Recorded routes remain an explicit mode for reproduction and matched comparisons.

The branch covers spec 2, followed by spec 4. Destination planning through the known site is the first
spec 2 milestone; geometric self-filtering and the persistent lidar costmap complete this spec. The ROS
bridge (spec 3), full 3D perception, worker-prediction changes and solver research remain their separate phases.
The actual Nav2/MoveIt baselines still require spec 3; our planner is not labelled as a Smac implementation.

The user approved the architecture and behavior below. The explicitly identified engineering details
are resolved in the implementation plan and its preflight tasks. Values copied from the existing Nav2
configuration remain reference settings, not newly measured performance or proof of equivalent behavior.

## 2. Existing workflow to preserve

The [full-hierarchy Nav2 launch](../../../ros2_ws/src/mobile_manipulator_navigation/launch/navigation.launch.py)
uses Smac State Lattice to find a base route. Its [behavior tree](../../../ros2_ws/src/mobile_manipulator_navigation/behavior_trees/navigate_to_pose_wholebody.xml)
checks whether a new route is needed at 1 Hz: a changed destination or an invalid path triggers search.
It does not unconditionally search every second. F2 and B5 use the route as a navigation reference and
choose the coordinated motion through passages; the global path is not a fixed whole-body trajectory.

B5 accepts the path, extracts its XY polyline and final goal yaw, assigns a new `route_id`, and announces
the accepted route. F2 consumes that route rather than the raw planner output. Same-destination route
updates carry the medium plan onto the new route where supported; changed destinations require fresh
medium planning. See [ADR 0015](../../adr/0015-medium-horizon-layer.md) and
[WholeBodyRoute](../../../ros2_ws/src/mobile_manipulator_interfaces/msg/WholeBodyRoute.msg).

The [current MuJoCo session](../../../ros2_ws/src/mobile_manipulator_campaign/src/session.cpp) differs:
`set_goal` makes a straight route, and `set_route` recreates the closed-loop stepper. Its constructor calls
`Controller::start`. The stepper's existing periodic route update only trims the old route; it does not
search around obstacles. Spec 2 must add an in-place route-adoption path and preserve the recorded-route mode.

## 3. Ownership and component boundaries

| Component | Responsibility and boundary |
|---|---|
| Routing core, `mobile_manipulator_routing` | Own map grid, footprint checks, lattice search, route validation and resumable search state; plain C++ API |
| Observation helpers | Geometric self-filter and persistence logic shared with existing ROS wrappers where practical |
| Campaign/session adapter | Resolve configuration and assets, arbitrate goals, schedule search, adopt routes, advance simulation and log outcomes |
| Base observation guard | Check near-term base motion and braking against confirmed observed occupancy before commands are released |
| Existing F2/B5 libraries | Coordinate base, arm and payload motion; retain command ownership, tracking model and existing safety logic |
| MuJoCo plant | Dynamics, simulated lidar and separately exposed validation ground truth |
| Unity and other clients | Submit goal/session requests and inspect published state; contain no route search or control policy |

The routing core takes ordinary values: map data, footprint polygons, motion primitives, poses, goal IDs,
configuration and simulation timestamps. It has no `rclcpp`, TF, Nav2 costmap or ROS message dependency.
Provide a portable CMake target; workspace packaging may wrap it without making a ROS graph necessary.
The planner must not require Unity assets, an Editor, a display, a bag or a connected TCP client at runtime.

The existing [geometry target](../../../ros2_ws/src/mobile_manipulator_geometry/CMakeLists.txt),
[geometric self-filter](../../../ros2_ws/src/mobile_manipulator_navigation/src/lidar_robot_filter.cpp) and
[PersistenceGrid](../../../ros2_ws/src/mobile_manipulator_navigation/include/mobile_manipulator_navigation/persistence_grid.hpp)
contain reusable logic. Their containing packages have build dependencies that must be assessed explicitly.
Pass resolved polygons/shapes into the core; export or move the necessary pure helpers into a shared target,
preserving the ROS wrappers' behavior and tests. Do not make the standalone target depend on the entire navigation package.

## 4. Map, footprint and motion data

### 4.1 Known map

Use the same [V2 occupancy map](../../../ros2_ws/src/mobile_manipulator_navigation/maps/construction_site_v2.yaml)
and exported site identity as the plant. Its current resolution is 0.05 m; its image is 1000 by 800 cells,
with origin (-25, -20) m. The export's height band is 0.02 to 0.60 m. Preserve the image row-to-map
conversion, origin and metric units. Validate dimensions, resolution, finite metadata, occupancy thresholds
and image length. Support the fields declared by the loader or reject them explicitly; silently ignoring
origin rotation or occupancy interpretation is unacceptable. Out-of-map and unknown cells are not traversable.

Known scenario geometry may be added through an explicit known-world input and rasterized in the base
height band. Record which obstacles were declared known. Do not populate the costmap from every MuJoCo
body: reserved/unmapped obstacles must become route obstacles only through the declared observation path.
Keep scenario movers' existing experimental input separate from the lidar costmap, and log any oracle mode.

Maintain distinct static occupancy, observed occupancy and inflated traversal-cost layers. Observation
expiry can remove an observation; it cannot erase a wall from the known map. Increment `map_revision`
when routing-relevant occupancy/costs change, with dirty regions available for route checks.

### 4.2 Footprint and inflation

Match the full hierarchy's base geometry: chassis, wheels and arm mount, excluding the arm and payload.
The [current wholebody footprint](../../../ros2_ws/src/mobile_manipulator_navigation/src/dynamic_footprint.cpp)
uses projected URDF base links with 0.02 m geometric padding and a separate 0.01 m Nav2 costmap allowance.
Represent both allowances explicitly and apply each once. Do not substitute the startup `home` polygon,
B4's changing whole-body footprint, or B5's independently configured collision margin.

The [reference inflation configuration](../../../ros2_ws/src/mobile_manipulator_navigation/config/nav2_global_planning.yaml)
uses radius 1.0 m and cost scaling 5.0. Inflation defines a traversal-cost field, not a mandatory metre
of clearance. Define our cost encoding and distance-to-cost function explicitly before adopting those
numbers as equivalent settings. Full footprint collision checks remain authoritative for route validity.

Base-route feasibility does not establish arm/payload feasibility. Overhead obstacles stay available to
F2/B5's 3D world rather than blocking all base routes through a height-collapsed whole-body map.
An infeasible whole-body result is reported distinctly from a global route-search failure.

### 4.3 Primitive data and collision checking

Initially use the differential-drive primitive data selected by the existing
[global-planning launch](../../../ros2_ws/src/mobile_manipulator_navigation/launch/global_planning.launch.py):
`sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json` in Nav2's installed data.
This is a data source, not a runtime dependency. Before implementation, pin its source revision and SHA-256,
inspect its schema and heading set, and retain the applicable licence/attribution in a repository-owned asset
or reproducible fetch process. Offline operation must be possible after assets are provisioned.

Validate the primitive model, declared resolution, headings, start/end states, sample order and finite
values. Derive heading bins and successor tables from the file; do not invent their count. Reject
primitive/map mismatches. Reverse expansion remains off initially, matching the forward-only hierarchy.

Check the oriented footprint over the complete motion of every primitive, including rotations in place,
not merely its endpoint. Conservative swept masks may be precomputed for lattice transitions. Arbitrary
start/goal connectors must obey the same kinematics and footprint checks, with a conservative bound covering
motion between samples. Define the bound and its conservatism before choosing sample spacing.
An occupied cell intersecting the padded footprint, including its interior, invalidates the motion.

## 5. Search and route representation

Search state is map position and primitive heading. Use A* with deterministic successor order, stable
priority-queue tie breaking and a bounded state store. Begin with an admissible distance lower bound under
the declared nonnegative edge costs. Rotation has a positive cost even when translation is zero.
Clearance costs, non-straight penalties and rotation penalties follow documented edge-cost definitions.
Nav2's reference weights are comparison inputs; identical parameter names do not establish identical costs.

Return a structured outcome: `found`, `invalid_start`, `invalid_goal`, `no_path`, `budget_exhausted`,
`canceled` or `stale`. A search limit is not proof that no route exists. Record expansions, frontier/memory
high-water marks, route length/cost and validation outcome. Wall-clock timings are separate diagnostics.

Preserve the raw lattice pose sequence for diagnostics. Produce the hierarchy's XY route separately,
with the requested final yaw explicit. Connect off-lattice starts and goals without teleporting or
rounding them into occupied cells. Any simplification/smoothing is subject to the same collision and
motion checks; retain the validated raw path if processing fails. Smac's analytic-expansion and smoothing
behavior are not automatically inherited by loading its primitive file.

Initial destination policy: preserve the requested goal pose, with the existing controller's
completion tolerances; do not silently substitute a nearby destination. Nav2's 0.20 m search tolerance
is a reference behavior requiring an explicit approximation policy if adopted. Report requested and
resolved endpoints separately whenever they differ; search tolerance and controller tolerance must not
silently accumulate into a larger success region.

Heading-only goals need a defined handoff. The existing `Route` requires two distinct XY points and drops
duplicates, while lattice rotations may repeat position. Define a heading-only reference understood by
the hierarchy instead of fabricating a small translation or loop. Preserve intermediate rotation poses
in diagnostic output even when the XY route omits them. This interface is an explicit design-review item.

## 6. Goals, route adoption and failure behavior

Each run has a `session_epoch`; each accepted destination a `goal_id`; each search a `request_id`;
each adopted route a `route_id`. Search inputs also carry map, footprint and configuration revisions.
IDs are assigned by the backend, not by a viewer. Client request IDs support acknowledgements and duplicate
detection. Simulation-time inputs receive a stable sequence number for equal timestamps.

| Situation | Backend behavior |
|---|---|
| Malformed or immediately invalid request | Reject with a reason before replacing the active destination |
| Valid replacement destination | Acknowledge acceptance, supersede the old goal, cancel its searches and invalidate its future references |
| Waiting for the replacement route | Brake the base and use the existing collision-checked arm hold; keep advancing an unpaused simulation |
| Same-destination replan | Retain the old route only while it remains valid; adopt a validated replacement in place |
| Destination/route search failure | Report its precise outcome and remain in a controlled hold; never silently resume a superseded destination |
| New request/reset during search | Cancel work; results with obsolete epoch/goal/request IDs cannot be adopted |
| Map/footprint change during search | Revalidate the complete result against current data, or cancel and restart; never adopt solely on an old snapshot |
| Whole-body planning failure | Preserve existing F2/B5 failure policy and report it separately from route-search status |
| Confirmed observation blocks proposed base motion | Enter the existing braking/hold path, report `base_observation_blocked`, and request route replanning |

Braking/hold is an executable control state, not an
instantaneous zero-velocity state; use the existing command limits and arm path rather than resetting
MuJoCo or teleporting joints. Route installation uses the then-current state, so an old search start cannot
introduce a discontinuous connector after braking motion.

The campaign/session adapter owns accepted-route arbitration. Add an in-place update to the persistent
closed-loop executor using the existing controller and medium-switch mechanics. An active goal's route
update uses `Controller::update_goal`, retaining its mode, failure history and warm state. Activating a
new goal after idle, failure or completion requires an explicit rearm (`Controller::start` or an equivalent
activation contract): `update_goal` deliberately leaves terminal controller modes latched. Define activation
after a planning hold as well; simply changing route pointers is insufficient. Discard stale RTI preparation
and queued plans from superseded routes. Same-goal carry-over and changed-goal medium reset must follow
the existing distinction. Both F2 and B5 see the same accepted route ID.

Preserve plant state, executed-command history, arm-reference state and applicable tracking adaptation
across route updates. Do not recreate the plant/controller/executor objects merely to replace a route.
Rearming a goal may reset goal-local solver state while retaining the last executed command and physical
histories. Progress/failure accounting is retained across same-goal replans; goal-scoped accounting is reset
deliberately for an accepted replacement, while the old goal's history is archived as `superseded`, not
erased or scored as a collision.

## 7. Simulation scheduling and headless operation

Separate session execution (`paused`, `running`, `stopped`) from navigation state (`idle`, `planning`,
`tracking`, `holding`, `reached`, `failed`). An unpaused session can advance physics, sensor acquisition,
observation expiry and scheduled goal events without an active route. Otherwise a goal scheduled at a
future simulation time can never arrive. Terminal navigation outcomes do not implicitly terminate a
campaign with future scheduled goals; the runner declares its end condition and simulation-time limit.

Apply inputs and route adoptions at control-cycle boundaries. Check a new accepted goal on the next
boundary; the 1 Hz route-validity check is not a one-second delay for user cancellation. Recheck the
remaining route after relevant map changes. Bound search work and memory so a pending search cannot
block B5 stepping or client control indefinitely. No search worker may mutate live MuJoCo data or controller state.

Campaign mode uses deterministic resumable search: a fixed expansion quota per simulation tick,
fixed total expansion/memory limits and fixed tie order. Completion/adoption occurs at declared simulation
boundaries, independently of host speed. Hash and log those settings. Preserve the medium planner's
existing deterministic budget/latency policy in recorded-route reproductions. A wall-clock watchdog
termination is a separately recorded failure, not an ordinary deterministic search result.

A paced live driver can use the same mode initially. If a worker/deadline mode is later needed, log
actual adoption times and replay those events; do not claim bit-identical timing from asynchronous arrivals.
Requested speed controls pacing, not physics step size or costmap persistence intervals. Pause freezes
simulation-time events and plan adoption; a paused destination can be acknowledged/queued for the next
step, with its state shown explicitly. Reset invalidates all work and observations from the previous epoch.

The direct C++ interface, headless runner and TCP adapter must use this same session machinery. A scripted
goal source may provide scenario goals, simulation-time events or logged state-triggered decisions.
An external live source records when the backend applied each request. A headless campaign does not
depend on viewer connection/disconnection semantics or the demo server's current single-client policy.

## 8. Lidar self-filter and persistent costmap milestone

One session-owned observation path drains completed plant scans and distributes them to filtering,
mapping and optional logging/visualization. Mapping receives every scan before viewer decimation or TCP
frame dropping; toggling an overlay or disconnecting a viewer cannot alter planner input.

The current [PlantScan](../../../ros2_ws/src/mobile_manipulator_plant/include/mobile_manipulator_plant/lidar.hpp)
contains per-return time offsets but lacks acquisition transforms/configuration history. Sweep mode
casts at multiple physics times. Extend the internal observation contract with the sensor pose and
robot geometry state at acquisition, or provide a bounded timestamped history that resolves them.
Declare the localization source: simulator ground-truth localization may remain the existing experimental
assumption, but obstacle labels and obstacle truth are not production perception inputs.

Transform points and pose the geometric self-model at their acquisition time, including the measured
arm and rigid payload. Reuse the existing ray/primitive self-filter semantics; never filter by MuJoCo
`geom` or `cls`. Those fields are validation labels only. Missing history/transforms drop the affected
observation with diagnostics instead of substituting a latest pose. Keep acquisition, receipt and
application times distinct. Update/reject out-of-order observations by a defined policy.

Apply the reference base marking band (0.05 to 0.60 m) and 5 m observation range after self-filtering.
Use the existing persistence semantics as the initial reference: confirmation after 2 s, maximum gap
1 s, expiry 10 s after last observation. All are simulation-time durations. Update at the reference
2 Hz costmap rate initially; observations between updates still contribute their acquisition timestamps.
Use stable cell ordering so unordered-container iteration cannot perturb search outcomes.

No return, missing scan or payload shadow means no observation, not free space. Match the current
persistence/expiry policy first; new ray-clearing behavior requires separate semantics and validation.
Expose counts for self-filter removals, missing history, observation confirmation/expiry and dropped scans.
This base costmap does not supply full 3D arm/payload perception or new worker tracking.

Confirmed observed obstacles must constrain execution as well as global routing: F2/B5 can depart from
the route, so a route-only costmap connection is insufficient. The minimum integration is a base-only
guard at the command boundary, checking the proposed near-term base sweep and braking envelope against
confirmed observed occupancy. It uses the current oriented/padded footprint, command latency and declared
stopping bounds; validate these with the plant rather than assuming ideal stopping. If the motion cannot
be cleared, take the existing braking/whole-body hold path and request replanning. An already occupied
footprint or insufficient stopping distance is reported as an execution incident, not hidden by a later replan.

All command paths, including medium fallback, pass this guard. A failed or unavailable check cannot be
treated as clear. The exact guard horizon and conservative stopping model are design-review items and
must be fixed before the lidar milestone's execution acceptance. No oracle boxes may be injected to make
F2/B5 see an unmapped obstacle. This adds base-band execution protection; it does not establish protection
for the arm/payload against new 3D obstacles, nor replace the later Nav2 collision-monitor integration.

## 9. Logs, protocol and provenance

Record submitted/accepted/superseded goals and search/route outcomes with simulation times, IDs and input
revisions. Include raw global paths, accepted hierarchy routes and requested final heading, so spec 4 can
distinguish predictions, failed searches and actual trajectory. Bound optional search-expansion and rejected
candidate traces; availability is explicit. Do not invent a failed path when search produced no candidate.

Extend run provenance and resume validation together: source/build identity, route mode, effective routing
configuration, primitive source/hash, map YAML/image/metadata hashes, footprint geometry/padding, goal-source
definition/events, seed and observation/timing policy. Generated-route runs must not resume against stale
map, goals, defaults or primitives. Keep per-goal histories; current setters' `records_.clear()` is not
appropriate for an interactive session audit trail.

The current stream/log schema is 2. New goal/route frames or changed layouts require an explicit schema
revision and client handling, including existing Python viewers/readers. Existing schema 1/2 recordings
remain readable under their original meaning. The implementation plan must specify new frame layouts,
handshake behavior and historical-reader checks before changing the wire format. Spec 4's full chassis,
wheel, sensor-pose and prediction display needs remain in its own detailed stream design.

Record source choices independently of viewer settings. Research ground truth remains separate from
what the costmap and planners observe. A new route source changes a campaign's experimental condition;
do not pool generated-route runs with earlier recorded-route ensembles without the declared comparison.

## 10. Acceptance and remaining design-review items

These are required acceptance areas, not completed tests or frozen empirical thresholds. The implementation
plan will name fixtures, bounded workloads, numeric budgets and evidence paths before running evaluations.

| Area | Required evidence |
|---|---|
| Standalone boundary | Build/run the routing core without ROS/Unity; destination submission works in a headless plant session |
| Map contract | Correct pixel/world transforms and occupancy semantics; reject malformed and unsupported metadata |
| Motion validity | Independent footprint/sweep checks for narrow doors, turns, primitive interiors, off-grid endpoints and heading-only goals |
| Search outcomes | Reachable, disconnected, occupied/out-of-map goal, expansion/memory limit and cancellation cases remain distinct |
| Goal replacement | Rapid A/B/C replacements, reset during search and stale-map results never activate obsolete routes or references |
| Control continuity | No object recreation on route adoption; bounded command transition, medium carry-over/reset, and successful goal activation after idle/reached/failed/hold states |
| Headless scheduling | Future goals arrive while idle/planning; paused sessions do not advance; generated and recorded modes have explicit termination |
| Reproducibility | Same complete inputs and deterministic settings reproduce goal/route decisions; resume invalidates changed inputs |
| Observation integrity | Moving sensor/arm fixtures, self-hit retention/removal checks and no oracle labels in filtering/planning |
| Persistence | Transient crossing versus persistent blockage, simulation-speed invariance, expiry, and no free-space inference from missing returns |
| Observed base obstacles | A global detour with F2/MPC cutting toward the observed box triggers the base guard; medium fallback cannot bypass it; insufficient stopping distance is reported |
| Viewer independence | Connection state, overlay settings, decimation and slow reads do not change headless mapping/planning inputs |
| Nav2 comparison | Same map/primitives/footprint and paired start/goals: reachability, route validity, length, clearance and search effort; exact paths need not match |

The comparison corpus must include open space, detours, narrow doors, overhead passages and unreachable
goals. An independently checked route-validity failure blocks acceptance; solver/search failure is reported
without claiming unreachability. Set paired-case quality and timing limits before observing comparison
results. Record complete hierarchy outcomes alongside route metrics so global success cannot hide a
whole-body planning failure. Broad claims of Nav2 equivalence require this evidence.

Implementation preflight resolves the primitive schema/provisioning, conservative sweep method,
heading-only handoff, search quotas/memory bounds, hold activation details, base observation guard
horizon/stopping model, standalone helper extraction and protocol transition. The approved initial goal
policy preserves the requested destination. The implementation plan records concrete engineering choices
and freezes qualification limits before measurements; design approval does not establish runtime validation.

The [medium-planner diagnosis](../../experiments/execution-fixes/e3.md) and E5/re-freeze follow-ups remain
separate recorded obligations. This branch's design does not claim to solve the existing passage stalls,
change E3's no-go ruling or validate historical plant evidence. Incorporate relevant diagnosis results
before freezing integration acceptance cases. New route-source performance must be distinguished from
any independent planner or plant fixes adopted from that work.

Spec 4 follows the accepted backend contracts and receives its own design and plan. Its mode-specific
ownership ADR and `AGENTS.md` amendment accompany implementation of the renderer mode; this design changes
no current runtime ownership. The branch brief retains all viewer requirements while that design is pending.
