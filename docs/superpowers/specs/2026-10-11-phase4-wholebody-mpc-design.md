# Phase 4 design: local whole-body MPC (baseline B5)

**Date:** 2026-10-11
**Branch:** `wholebody-mpc` (from `demo-launcher` at `5dd342f`)
**Roadmap:** Phase 4, baseline B5 (docs/roadmap.md Sections 2, 3 and 8)

## 1. Purpose and scope

Phase 4 replaces the Nav2 local controller and MoveIt as the robot's command authority with
a local whole-body MPC that drives the base and moves the arm and the attached panel at the
same time. B1-B4 either keep the arm still or reconfigure it at standstill; B5 is the first
baseline with continuous, coordinated base and arm motion. Nav2 stays the source of the
global route (planner server, behaviour tree, recoveries).

Phase 4 delivers B5 on the existing scenarios and on new ones that force interaction
(worker scenarios, an overhead-obstacle course), free drive with the MPC from RViz goals,
and the timing instrumentation that Phase 4B builds on. It does not add foresight beyond the
MPC horizon (Phase 5).

### Decisions taken in the design discussion

| Topic | Decision |
|---|---|
| Baseline scope | B6 (acados SQP-RTI) optional; kept cheap by a formulation separate from the solver. F4 (B-spline) decided before Phase 5. |
| Local MPC representation | Stage-wise discretized LTV QP. A Bézier-parameterized local MPC (Du et al.) is a Phase 6 ablation, discussed again once B5 has numbers. |
| Optimization split | Phase 4: per-stage timing, warm start, preallocation, solver-agnostic formulation, own process, minimal staged collision checking, arm CPU headroom. 4B: horizon, rate, grid, QP solver, condensing. Phase 5: conditional computation. Phase 6: scheduling and real-time measures, RTI split, AS-RTI, threading (Section 12). |
| Model generation | CasADi symbolic model with generated C code, shared by B5's linearization and a later B6. |
| Obstacle model | Known geometry: the exported map boxes and the scenario boxes (option A), as B3's MoveIt world. A perceived distance field is a separate later baseline. |
| Moving obstacles | Unity ground truth on `/scenario/movers` with a constant-velocity prediction over the horizon; results labelled "known movers". |
| Worker scenarios | `worker_crossing_nav` unchanged; add a late crossing and a worker that stops in the lane. |
| Overhead course | Built in Phase 4 and run by B1-B5. |
| B4 in-drive footprint | Grow-only while the base moves; B4 regression and dynamic-strategy drives rerun. |
| Safety layer | Collision monitor kept for the base (base-only zones, points up to base height); slowdown zone = hull swept by the MPC's predicted configurations. A 3D safety check goes with the perception baseline. |
| Arm command path | Stream the predicted joint trajectory to JTC every cycle. A forward command controller or a custom ros2_control controller are fallbacks if timing or tracking requires them. |
| Panel references | One interface for optional panel references along the route. Scripted rows (references from the mission steps, as B3/B4 get) and unscripted rows (no references) are both reported. Free drive is unscripted. |
| Architecture | `wholebody_mpc_node` serves the `FollowPath` action in its own process; the behaviour tree's `FollowPath` node points at it. |
| Base motion | Forward-only plus turns in place, as the Nav2 baselines. |

### Out of scope

- Foresight beyond the MPC horizon: deciding early where the panel must change (Phase 5).
- Perceived obstacles (distance field from the lidar) and the 3D safety check (perception
  baseline, unscheduled); the MoveIt Octomap fault stays open.
- B6 (acados SQP-RTI); the formulation keeps it possible.
- Reversing, a manipulability cost term, link-to-link self-collision constraints.
- Real-time scheduling, CPU pinning and solver threading (Phase 6).
- Changes to B1-B4 other than the B4 grow-only rule.

### Paper facts this design relies on

- Wang, Chen, Zhao (ICRA 2025): state (x, y, θ, q), velocity controls, unicycle base,
  RK discretization with Δt = 0.1 s and N = 20, collision circles queried against an ESDF
  projected to the ground plane, one SQP-RTI iteration per cycle with acados and a CasADi
  model, 4.3 ms mean on an i7-1370PE, MPC at 100 Hz, joint PD at 1000 Hz. Task weights that
  vary along the horizon introduce a new task gradually.
- Du et al. (JFR 2026): a 0.2 s whole-body horizon made the base oscillate near obstacles;
  2-3 s was smooth. Separate safety constraints for base and end-effectors instead of one
  large sphere. Timings are CasADi/KNITRO from Python on an i9.
- Li et al. (arXiv 2608.07005): independent per-link collision gradients stall the
  optimizer when one link's escape pushes an upstream link into collision; gradients must
  be consistent along the kinematic chain. Bounding-box prefilter of obstacle points.
- Sagar et al. (CoDIT 2026) future work: planner-aware footprint prediction from future arm
  poses over a short horizon. B5 covers the controller half in 3D (Section 5); the planner
  half is Phase 5.

## 2. Architecture

```text
RViz "2D Goal Pose" / tools/run_nav_scenario.py
        | NavigateToPose (unchanged)
        v
bt_navigator (project tree) ----> planner_server (Smac Lattice; B5: base-height costmap)
        | FollowPath(path), BT node server_name -> wholebody_mpc
        v
wholebody_mpc_node (own process; replaces controller_server and velocity_smoother in B5)
  in:  TF (map -> base_footprint), arm joint states, /scenario/known_obstacles (latched),
       /scenario/movers, panel references (latched; empty = unscripted)
  out: cmd_vel ----------------> collision_monitor input --> /cmd_vel --> Unity base
       arm joint trajectory ---> arm_controller (JTC) --------------------> Unity arm
       predicted trajectory ---> dynamic_footprint_node (wholebody mode), RViz
       timing and solver diagnostics --> nav_telemetry, run summary
```

- **`mobile_manipulator_mpc`** (new, C++): the formulation (cost, constraints, linearization
  through the generated model), a solver interface with an OSQP backend, the world-distance
  library (Section 4) and `wholebody_mpc_node`. The formulation knows nothing about OSQP;
  the solver interface receives a stage-wise QP (per-stage matrices and bounds) so an HPIPM
  or acados backend can be added without touching the formulation.
- **Model generation:** a build-time Python script builds the kinematic model in CasADi
  from the robot description (unicycle base, the six arm joints, tool and grasp transforms,
  the collision primitives' frames) and generates C functions for the discrete dynamics,
  the panel pose, primitive poses and their Jacobians. The runtime is C++ only. CasADi is
  added to the container for both `linux/amd64` and `linux/arm64`.
- **`mobile_manipulator_geometry`** gains 3D collision primitives (capsules and oriented
  boxes for base, links and panel) next to the existing ground projection, so the hull and
  the MPC use one robot model.
- **`wholebody_mpc_node`** runs its control loop on its own timer in a dedicated callback
  group, separate from the action server and subscriptions. It preallocates the QP and the
  distance buffers at start-up.
- **Launch:** `controller:=wholebody` starts the planner server, the behaviour tree (with
  `FollowPath` pointing at the MPC), the collision monitor, `dynamic_footprint_node` in
  `wholebody` mode and the MPC node; it does not start `controller_server` or
  `velocity_smoother`. One `cmd_vel` publisher upstream of the collision monitor remains
  (ADR 0006 single-publisher check).
- **B1-B4** launch files, costmaps, controllers and missions are unchanged.

## 3. MPC formulation

### Model

- State x = (x, y, θ, q1..q6), control u = (v, ω, q̇1..q̇6).
- Base: unicycle kinematics (no lateral motion), commanded through `cmd_vel`. Arm: joint
  velocity integrators. Discretized in CasADi (RK4 or exact unicycle integration over Δt).
- Prediction grid for bring-up: Δt = 0.1 s, N = 20 (2 s). Nonuniform grids are a 4B option.

### Per-cycle problem

1. Read the measured state (base pose from TF, joint positions from the arm state).
2. Shift the previous solution by one cycle as the nominal trajectory (cold start: hold
   the current state).
3. Linearize the dynamics and the collision distances around the nominal (generated
   Jacobians, FCL distances).
4. Solve one QP (OSQP, warm-started from the shifted primal and dual solution).
5. Send the first base command and the predicted arm trajectory; store the solution.

### Constraints

- Joint position limits, joint velocity limits; arm velocity and acceleration limits from
  the combined-motion qualification (Section 8).
- Base: 0 <= v <= 0.3 m/s, |ω| <= 0.4 rad/s (operating envelope); forward-only.
- Rate limits (operating envelope): linear 0.45 m/s² accelerating and 0.5 m/s² braking, yaw
  0.8 rad/s² accelerating and 1.0 rad/s² braking. The first stage's limit is relative to the
  last command sent, not to the measured velocity (the RPP rotate-from-rest stall,
  docs/experiments/nav2-navigation).
- Collision: one linearized distance constraint per active robot-obstacle pair and stage,
  d + nᵀ J(p) Δx >= margin - s, with slack s >= 0 under a large L1 weight (Section 4).
- Self and floor: panel versus base, panel and wrist above the raised floor (0.15 m, as in
  B3's MoveIt world).

### Cost

- **Route:** the Nav2 path as a reference: contouring and lag error of each predicted stage
  against its reference point, and a progress speed along the path that tapers near the
  goal.
- **Heading:** alignment with the path tangent; the goal heading at the terminal stage near
  the goal.
- **Panel reference** (when given): panel position error plus the orientation error
  tr(I - R_refᵀ R), weighted by a factor that depends on the predicted stage's progress along
  the route (Section 6).
- **Nominal posture:** a small pull towards the scenario's starting arm pose (`home` in free
  drive) when no panel reference is active.
- **Effort and smoothness:** control magnitude and control changes.

### Breakaway and command output

The base does not start below 0.07 rad/s or 0.0375 m/s from rest (operating envelope). The
QP plans continuous commands; an output stage replaces a non-zero planned command below the
breakaway value by the breakaway value while the base is measured at rest, until motion is
measured. Below a dead zone the command is zero.

### Goal and failures

- Goal reached: 0.15 m and 0.15 rad, as the baselines' goal checker; the MPC reports
  `FollowPath` success and holds.
- QP infeasible, solver error or a missed deadline: use the shifted previous plan for up to
  K cycles (parameter); JTC keeps following the last streamed trajectory meanwhile. After K
  cycles: brake at the envelope's braking limit, hold the arm, and fail `FollowPath`, so the
  tree's recoveries (clear costmap, wait, replan) run.
- No progress for the progress checker's window: fail `FollowPath` as Nav2's controller
  server would.

### Rate and timing

Bring-up at 20 Hz; 50 Hz once the CPU headroom and timing allow it. The controller manager
and Unity physics run at 50 Hz, so an MPC above 50 Hz only shortens the state-to-command
latency (a 4B measurement). Each cycle records linearization, collision, QP setup, solve and
total time, the state-to-command latency, solver status and iterations, deadline misses, and
the arm's translational manipulability sqrt(det(J Jᵀ)) as a diagnostic (a cost term is added
only if low values coincide with tracking or reference failures);
summaries report mean, median, p90, p95, p99, max and the miss rate (roadmap Section 7).

## 4. Collision and world model

### Robot

- Arm links: capsules. Base: an oriented box including the wheels. Panel: an oriented box
  of 1.20 x 1.20 x 0.04 m at the grasp transform.
- Spheres are not used for the panel: in vertical carry its 0.04 m thickness is what passes
  the 1.05 m gate, and a sphere cover of the plate would add about 0.2 m per side.

### Distances and linearization

- FCL (0.7, installed with MoveIt) computes the distance and nearest points between each
  robot primitive and each obstacle box (GJK for convex pairs).
- For a pair with distance d, normal n and nearest robot point p, the constraint is
  linearized through the full chain: d + nᵀ J(p) Δx >= margin, where J(p) is the Jacobian of
  p with respect to base pose and all joints. This keeps the escape directions consistent
  along the arm (Li et al.).
- Margin 0.05 m (parameter).

### Fixed problem size

- Broad phase: the bounding box of each robot primitive swept over the nominal trajectory
  against obstacle bounding boxes; only obstacles within an activation distance (about
  0.8 m, parameter) produce pairs.
- A fixed budget of the closest pairs per stage (initially 8) keeps the QP size constant.
  Pairs beyond the budget are counted and logged.

### World

- One loader reads the exported map boxes (`construction_site.boxes.yaml`) and the
  scenario's boxes, now with an optional height offset (`z`, default 0) so boxes can float.
  The runner publishes the result on a latched `/scenario/known_obstacles` topic; the loader
  that fills B3/B4's MoveIt scene reads the same source.
- Movers: boxes from `/scenario/movers`; velocity from consecutive positions; pose at stage
  k predicted at constant velocity.
- The distance query sits behind an interface so a distance-field backend (perception
  baseline) can replace it.

### Cost

About 20 stages x 8 pairs of GJK queries per cycle is expected to stay well below 1 ms; the
timing instrumentation measures it. Fallbacks: closed-form capsule-box distances, or
collision constraints on fewer stages.

## 5. Safety layer and footprint modes

`dynamic_footprint_node` remains the only owner of the costmap footprints and the
collision monitor zones (ADR 0010) and gains two changes.

- **Grow-only rule (B4, `dynamic` mode):** while the base moves, it publishes a hull only
  if the hull is not contained in the current one grown by the 0.01 m threshold; it shrinks
  the footprint only at standstill. This removes the sway republishes and never publishes
  a hull smaller than the swinging panel.
- **`wholebody` mode (B5):**

| Output | Polygon | Observation points |
|---|---|---|
| Costmap footprints (route) | base only | obstacles up to base height |
| Stop zone | base only + margin | up to base height |
| Slowdown zone | hull of the MPC's predicted configurations over its horizon (each at its predicted base pose, expressed in the current base frame), + margin | up to base height |

Overhead obstacles have no points up to base height, so neither zone stops the robot under
a beam; the MPC keeps arm and panel clear of them. A low obstacle under the panel can slow
the robot (never stop it). The swept hull is the paper's footprint prediction in 2D form
for the safety layer; the MPC's own 3D constraints are the primary protection.

The B5 route uses a costmap whose obstacle layers mark points up to base height only; B1-B4
keep their 0.05-2.0 m band.

## 6. Panel references and free drive

- **Interface:** a latched message with a list of route-anchored references: a panel pose in
  `base_footprint`, a start point and an end point on the route, and a ramp length.
- **Scripted (missions):** the runner converts each `reconfigure` step into a reference
  that starts at the waypoint of the `navigate` step before it and stays active until the
  waypoint of the next `reconfigure` step (in `narrow_gate_mission`: vertical carry from
  (-4.2, -7.725) to (-9.0, -7.725), then `home`). The robot does not stop at the waypoints;
  the `navigate` steps become one route to the final goal.
- **Weighting:** the panel cost weight of a predicted stage ramps from 0 to full over the
  ramp length before the start point and back down after the end point, measured as route
  progress, so a reference enters the far end of the horizon first.
- **Unscripted:** no references; the nominal-posture cost (the starting arm pose) applies and
  the arm deviates only when constraints force it.
- **Free drive:** `tools/demo.py drive --controller wholebody` launches B5 unscripted;
  RViz goals work unchanged, including goals behind the robot and new headings.

## 7. Scenarios and runner

- **Worker scenarios** (arm `home`, existing mover features):
  - `worker_late_crossing_nav`: a slower worker timed to reach the lane as the robot
    arrives; without adjustment they would collide.
  - `worker_stops_in_lane_nav`: the worker steps onto the route and stays.
  - Speeds and triggers are tuned in the plan so that a robot that does not react would
    collide.
- **Overhead course:** a narrow gate needing vertical carry, then a low beam needing the
  panel lowered, then a 90° turn, placed in free map space (location and dimensions in the
  plan). Scenario boxes gain the optional `z` offset; Unity places elevated boxes. A drive
  version for B1/B2 and a mission version (with reconfigure steps) for B3/B4/B5.
- **Runner:** `--controller wholebody` (B5), `--references none` (unscripted). Scoring is
  unchanged (success, `off_goal`, contact from Unity). Added for B5: per-stage timing
  statistics, deadline misses, near-misses (slack used), arm tracking error, and the
  minimum 3D clearance between the robot primitives and the known boxes, computed from the
  recorded states.

## 8. Prerequisites and qualification

1. **B4 grow-only rule**, then rerun the B4 regression (9 runs) and the 20 dynamic-strategy
   drives.
2. **Arm CPU headroom:** run a synthetic CPU load beside a qualified arm transition and find
   the load at which the 0.15 rad path tolerance is approached; this sets the MPC's CPU
   budget and starting rate (pre-Phase-4 checklist item 2).
3. **Combined arm and base motion:** the operating envelope was qualified with the arm
   holding while the base moves. Measure JTC tracking with the panel while arm and base
   move together, and derive the arm velocity and acceleration limits the MPC uses.
4. **JTC streaming:** verify trajectory replacement at 20 and 50 Hz (where each new
   trajectory starts, continuity of position and velocity, behaviour when updates stop).

## 9. Evaluation and exit criteria

Three runs per cell:

| Scenarios | B1/B2 (RPP, DWB, MPPI) | B3 | B4 | B5 scripted | B5 unscripted |
|---|---|---|---|---|---|
| 8 Phase 1 driving scenarios | recorded (2026-10-04) | - | - | yes | same as scripted (no references) |
| 3 B3 missions | - | recorded | recorded | yes | gate missions |
| 2 new worker scenarios | yes | - | yes (RPP, dynamic) | yes | - |
| Overhead course | drive version | yes | yes | yes | yes |

Timing runs at 20 and 50 Hz on a subset; the full sweep is Phase 4B.

Phase 4 is complete when:

- B5 drives free-drive goals from RViz, including goals behind the robot and new headings;
- every cell above is recorded, with no contact in the scripted B5 missions or the failures
  reported as such;
- the p99 cycle time stays within the period at the chosen rate (at least 20 Hz, target
  50 Hz);
- the documentation in Section 11 is written.

## 10. Testing

- Unit tests (no ROS graph): generated model against numerical differentiation; panel pose
  against the robot model's forward kinematics; FCL distances and the linearized constraint
  against finite differences; QP assembly sizes and stage structure; the breakaway output
  stage (a goal behind the robot from rest produces a command at or above breakaway); the
  rate limit uses the last command, not the measured velocity; the shifted-plan fallback and
  the brake after K failures; route-progress weighting of panel references.
- Node tests with mock inputs: `FollowPath` success, failure and cancel; latched obstacles
  and references; diagnostics published.
- Footprint node: grow-only rule (sway does not republish, standstill shrinks); `wholebody`
  swept hull from a predicted trajectory.
- Integration: the existing mock-arm tests plus a B5 mission against the mock arm; the
  scenario runs in Unity are the acceptance tests.

## 11. Documentation and roadmap

- ADR 0011: whole-body MPC command ownership (FollowPath server, `cmd_vel` path, JTC
  streaming, fallbacks), revisiting ADR 0006's Phase 4 note.
- `docs/experiments/wholebody-mpc/README.md`: prerequisites, qualification, runs, timing.
- Roadmap: Phase 4 decisions; the resolved open decisions (B4 in-drive footprint, B6
  optional, F4 deferred to before Phase 5); the paper items (CasADi codegen in Phase 4,
  Bézier local MPC as a Phase 6 ablation, Li's flat-output yaw and Du's axis-angle
  orientation for Phase 5, moving obstacles in the Phase 5 triggers); the perception
  baseline with the 3D safety check; the 50 Hz actuation note for 4B.
- `docs/running-the-stack.md`: B5 launch and free drive.

## 12. Open items after this design

- **Phase 4B:** horizon, rate (20/50/100 Hz), prediction grid, OSQP versus HPIPM, partial
  condensing, warm start ablation, collision-model detail, B6 go/no-go.
- **Before Phase 5:** F4 core or optional.
- **Phase 5:** medium-layer representation (F2 first, F3, F1), conditional computation
  triggers including moving obstacles, flat-output yaw, axis-angle panel orientation,
  asynchronous medium planner.
- **Phase 6:** Bézier local MPC ablation; CPU affinity, SCHED_FIFO, PREEMPT_RT, mlockall,
  threading; RTI preparation/feedback split, AS-RTI; bit-packed voxel kernels, swept-volume
  collision, geometry fidelity.
- **Fallbacks:** forward command or custom ros2_control arm controller; closed-form
  capsule-box distances.
- **Unscheduled:** perception baseline with the 3D safety check; MoveIt Octomap fault;
  camera noise (only with a camera); CHOMP for B3.
- **Off in B5:** reversing, manipulability cost, link-to-link self-collision.

## 13. Risks

- **Arm tracking under combined motion:** the cantilevered panel may not track arm motion
  during base acceleration within 0.15 rad; the qualification in Section 8 sets the limits,
  and tighter limits slow B5.
- **Linearization quality:** one QP per cycle around the shifted plan can lag fast
  configuration changes; the slack log and the near-miss count show it, and B6 is the
  remedy if needed.
- **JTC replacement behaviour:** frequent replacement may cause hiccups; fallbacks exist.
- **CPU load:** MPC, Nav2 planner, Unity and the endpoint share the machine; the headroom
  measurement and the timing data decide the rate.
- **Reactive limitation:** unscripted B5 sees only 2 s ahead and may stop or fail at the
  gate and the beam; this is reported as the Phase 5 motivation, not tuned away.
