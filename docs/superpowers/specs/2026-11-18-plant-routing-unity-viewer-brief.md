# Plant routing and Unity viewer: phase brief

Date: 2026-11-18. Branch: `plant-routing-unity-viewer`, created from `e3-learned-margin`
at `4d9a16f8da23a04185c1fe75858ced7831647b36`. Status: the user approved the spec 2 design and this
spec 4 requirements brief on 2026-11-18. 2026-11-20: spec 2 is complete (`968628d`) under its revised plan;
spec 4 begins with the viewer design (plan Task 0), staged as plans 4A, 4B and 4C (user, 2026-11-20).

This branch covers spec 2 (the plant's route planner, costmap and lidar self-filter), followed by spec 4
(the Unity viewer). The first deliverable is the [spec 2 design](2026-11-18-plant-route-planner-design.md).
The [spec 2 implementation plan](../plans/2026-11-18-plant-route-planner.md) follows that approval.
The [spec 4 staged plan](../plans/2026-11-18-plant-unity-viewer.md) begins by completing the viewer's
detailed technical design and performance workload before viewer implementation.

## 1. User decisions from this discussion

| Topic | Decision |
|---|---|
| Primary use | Interactive live demonstrations, beginning with destination selection through the known site |
| Branch scope and order | Spec 2 first, then spec 4, on one new branch from `e3-learned-margin` |
| Planning backend | Our own C++ state-lattice planner, costmap and self-filter, operating without ROS 2 (2026-11-19: the search is a direct C++ port of Nav2 Smac Lattice 1.3.13, spec 2 Amendment 1 and ADR 0019; the self-filter moved to the perception phase, Amendment 2) |
| Reference workflow | Preserve the established Nav2 lattice workflow and hierarchy handoff where applicable |
| Initial hierarchy | Global route planner, F2 medium-horizon planner, B5/MPC and MuJoCo; the first viewer runs the full hierarchy |
| Goal replacement | A newly accepted destination replaces the current destination and triggers replanning |
| Headless operation | Goals can originate from a headless campaign or another backend client, including goals supplied during execution |
| Viewer independence | Unity is optional; complete planning, goal replacement and MuJoCo execution work without it |
| Physics | MuJoCo is the physics backend; Unity displays the scene, robot and optional telemetry |
| Development sequence | Discuss, write and review the design, create the implementation plan, then execute |

Recorded routes remain available for reproducibility and matched comparisons. New scenario-defined or
programmatically supplied goals can use the lattice planner instead of relying on those recordings.
Headless goal submission does not imply wall-clock pacing: a campaign can operate on simulation time.

## 2. Spec 2 scope

The route provider is a ROS-free C++ library with ordinary map, footprint, pose, goal and time inputs.
Unity, command-line tools and headless campaigns submit goals through the same backend contract.
The planner searches position and heading, checks the base footprint along motion primitives, and supplies
a route to the existing F2/B5 hierarchy. Arm and payload motion remain the whole-body hierarchy's responsibility.

The agreed destination-first staging is known-map planning and goal replacement, followed by the lidar
self-filter and persistent obstacle costmap needed to complete spec 2. (2026-11-19: the self-filter, the persistent
costmap and the base guard moved to the perception phase; spec 2 keeps the acquisition-time observation path. See the
spec's Amendment 2.) Interactive obstacle placement is a
later demo capability; it is not the first acceptance scenario.

The existing Nav2 configuration supplies reference behavior, primitive data and comparison cases.
Our planner is identified as our state-lattice planner. Reusing primitive data does not make it the Nav2
Smac implementation, and equivalence to Nav2 remains a validation question. (Superseded 2026-11-19 by spec 2
Amendment 1: the search is a direct port of Smac Lattice 1.3.13, equal to the installed Smac on 342 test cases.)

## 3. Spec 4 viewer requirements

These features were requested by the user. Exact layout, representations, budgets and release staging
belong to the spec 4 design.

| Area | Requested behavior |
|---|---|
| Main view | Render the site and robot from MuJoCo state |
| Cameras | Change camera perspective with a button |
| Goals | Open a minimap and place destinations in a Nav2-like interaction |
| Telemetry | Show optional telemetry |
| Lidar | Toggle point-cloud overlays |
| Paths | Toggle the planned path and actual travelled path |
| Controller predictions | Optionally display MPC planned paths; the discussion interpreted the user's “MCP” as MPC |
| Future motion | Show ghosted future arm and payload motion, with swept-volume visualization to be designed |
| Safety and failures | Display safety margins and failed paths |
| Session controls | Pause, change speed and reset |
| Performance | Maintain stable FPS, including when overlays are enabled |
| Diagnostics | Provide a togglable diagnostic log console |

Live viewing is the priority. Recorded-run replay and standalone macOS/Windows players are existing
commitments in the [plant specification](2026-11-05-mujoco-plant-design.md); their detailed staging belongs
to the spec 4 design. Backend goal submission and logging must remain usable with no viewer connected.

## 4. Approved design direction

The following began as assistant recommendations and are included in the approved design direction.
Exact representations, limits and the FPS workload remain implementation-design details:

- Click for goal position and drag for final heading; show goal acceptance and planning outcome on the minimap.
- Distinguish the global route, selected medium plan, MPC prediction, actual trajectory and rejected candidates.
- Key predictions and ghosts to the backend's accepted goal/route/plan identifiers so obsolete plans disappear.
- Begin future-motion visualization with sampled translucent poses; define the meaning and accuracy of any
  continuous swept volume before presenting it as a collision envelope.
- Obtain active margins, failure reasons and prediction data from the backend.
- Provide optional collision-geometry inspection, planner status, stream freshness, FPS and achieved simulation speed.
- Keep cameras and inspection usable while simulation is paused; reset the same scenario and seed by default.
- Decouple rendering from simulation and planning, with bounded overlay histories and adjustable visual density.
- Give the console severity/subsystem filters, simulation timestamps, bounded history, search and copy/export.

The FPS target, reference hardware, overlay load and measurement protocol remain to be set in spec 4.

## 5. Ownership and compatibility

For the new renderer mode, MuJoCo owns physics and lidar, and the ROS-free framework owns planning and
control. Unity renders received state and sends explicit user goal/session requests. The roadmap's
“nothing it does feeds back” is interpreted as no Unity-generated physical or sensor feedback in this mode;
goal and session controls are intentional backend inputs.

The viewer uses the existing HDRP project and native-host Unity CLI/Pipeline workflow. The spec 4 ownership
ADR and a mode-specific `AGENTS.md` amendment must accompany the implementation of this mode. This brief
does not change the existing Unity/ROS simulation's runtime ownership or assert that the renderer exists.

Spec 3, the MuJoCo ROS bridge, is outside this branch's scope. It remains required later for the actual
Nav2/MoveIt baselines and the live ROS end-to-end check; the new lattice planner is not a substitute for those baselines.
Full 3D lidar perception and improved worker prediction remain their documented later phases.

## 6. Relationship to current project work

This session refines the [2026-11-18 roadmap ordering](../../roadmap.md): destination-first interaction
places spec 2 before spec 4 on this branch. The separate medium-planner diagnosis, E5 follow-up and B5
re-freeze obligations remain recorded; this branch does not claim they are complete or take over their work.
The E3 no-go ruling, existing measurements and scientific claims are preserved.

The detailed designs must identify dependencies on that diagnosis and the plant's current correctness,
without treating this requirements brief as validation evidence. Implementation completion and performance
claims require their own checks and recorded results.
