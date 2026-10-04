# MM Motion Planning: Phase Plan, Optimization Roadmap, and Design Decision Log

**Project:** Mobile manipulator carrying a bulky panel through construction-like environments  
**Purpose:** Living reference for the user and Codex. Preserve the agreed thesis architecture, baselines, optimization variants, implementation order, computational strategy, and literature-derived design choices.  
**Status:** Living document. Update it as phases are implemented, benchmarks are collected, or design choices change.

---

## 1. Thesis architecture in one picture

The final intended architecture is hierarchical:

```text
                         NAV2
                long-range base route
                         |
                         v
          MEDIUM-HORIZON OPTIMIZER
    reduced task space + compact trajectory
     base / payload foresight, several seconds
                         |
                         v
          LOCAL WHOLE-BODY MPC / QP
      short receding horizon, high update rate
        base + arm collision-aware correction
                /                   \
               v                     v
          base command            arm reference
           /cmd_vel                 |
               |                    v
               v               ros2_control
         Unity skid-steer       arm controller
```

The final system is **not** one giant long-horizon optimizer running at control rate. Computational feasibility comes from separating long-range routing, medium-range foresight, and short-range reactive whole-body control.

### Non-negotiable architectural principle

Exactly one component may have effective authority over `/cmd_vel` at a time.

- **Phases 1-3:** Nav2 Controller Server owns `/cmd_vel`.
- **Phases 4-6:** custom whole-body MPC owns `/cmd_vel`.
- MoveIt does not own `/cmd_vel` in the main architecture.

The Unity/base actuator interface remains `/cmd_vel -> skid-steer actuator`; the planner changes above it.

---

# 2. Development phases

## Phase 0: Low-level simulation/control foundation

### Goal
Freeze a believable simulator/controller foundation so the thesis can focus on planning rather than repeatedly reopening actuator-control work.

### Existing/accepted decisions

- Mobile base: skid-steer / differential-drive-style platform.
- Manipulator: 6-DOF arm.
- Unity remains the initial simulator, but ROS 2 interfaces should remain simulator-agnostic.
- `ros2_control` is used for the arm-side control interface.
- Arm gravity should be enabled in Unity rather than permanently disabled.
- Arm must hold its commanded configuration while the base moves, as a real mobile manipulator would.
- Gravity feedforward is a possible later enhancement, not a hard requirement for initial controller qualification.
- Freeze the low-level arm controller once it is sufficiently tested. The thesis research contribution is the planner, not endless joint-controller tuning.

### Performance rule
A future real robot may use a ~1000 Hz low-level servo/controller while the optimizer runs more slowly. In Unity, however, a nominal 1000 Hz control calculation only has physical meaning if physics integration/actuator updates support it. Do not confuse controller-thread frequency with actual simulated plant update frequency.

---

## Phase 1: Plain Nav2 base navigation, fixed arm, fixed footprint

### Research purpose
Establish a conventional mobile-base navigation baseline before manipulation changes the geometry.

### Robot state

- Arm remains fixed in a known transport pose.
- Payload is absent or represented by the fixed conservative footprint appropriate to the experiment.
- Current configured footprint in the `nav2-setup` work is approximately 1.2 m x 1.2 m and should be preserved as an explicit baseline until changed intentionally.

### Current repository status before full Phase 1
The current Nav2 work already has static map export, Map Server, Planner Server, NavFn/A*, global costmap, StaticLayer, and InflationLayer. It is a planner-only smoke-test rather than complete autonomous navigation.

### Add for full navigation

- `/odom` as `nav_msgs/Odometry`, including pose and twist.
- Controller Server.
- Local rolling costmap.
- Filtered Livox point cloud input.
- VoxelLayer or appropriate live obstacle representation plus InflationLayer.
- Self/payload/height filtering so the lidar does not treat the robot itself as an obstacle.
- BT Navigator and Behavior Server for ordinary `NavigateToPose` behavior/recovery.
- Complete lifecycle integration.

### Nav2 responsibilities

- Long-range global route from the known static map.
- Local path following and obstacle response.
- Replanning/recovery through its navigation stack.
- Initially keep the exported global map static while live Livox observations primarily feed the rolling local costmap.
- Treat transient workers/carts as local reactive obstacles. If a blockage persists and invalidates the route itself, later provide global-planner awareness through an appropriate global obstacle/custom layer or tracked blockage representation; otherwise repeated global replans against an unchanged static map may simply return the same blocked route.
  - *2026-09-30:* confirmed: with the static-only global costmap, all three controllers failed both the static-obstacle detour (scenario 2) and the persistent blockage (scenario 7), 0/18. An opt-in STVL obstacle layer in the global costmap (`global_obstacles:=true`) made both succeed, 18/18, without regressing open space or the 1.30 m gate (docs/experiments/nav2-navigation). It is the navigation default from 2026-09-30; `global_obstacles:=false` keeps the Phase 1 baseline.
  - *2026-10-01:* the live layer made a crossing worker (scenario 3) a global obstacle and cost 4 of 10 runs; the global costmap now marks only obstacles observed for 2 s (`PersistentObstacleLayer`, `global_obstacles:=persistent`), which passed scenarios 2, 3 and 7 in all 27 runs (docs/experiments/nav2-navigation). Transient workers stay local obstacles; blockages still change the route.

### Controller baselines worth preserving

1. **DWB** as a conventional local dynamic-window baseline.
2. **Nav2 MPPI** as a stronger predictive base-only controller baseline.

Regulated Pure Pursuit may be used for bring-up/path-following qualification, but DWB and Nav2 MPPI are more interesting final comparisons for obstacle-rich navigation.

### `/cmd_vel` owner
**Nav2 Controller Server.**

### Phase 1 measurements

- global planning success/failure,
- path length,
- planning latency,
- controller tracking error,
- static obstacle clearance,
- live obstacle avoidance,
- recoveries/replans,
- traversal time,
- CPU and memory usage,
- local costmap update latency.

---

## Phase 2: Nav2 base navigation + MoveIt arm planning

### Research purpose
Create a decoupled mobile-manipulation baseline: Nav2 handles base navigation, MoveIt handles arm collision-aware planning.

### Architecture

```text
Nav2 global/local navigation -> /cmd_vel -> base
MoveIt MoveGroup planning -> arm trajectory -> ros2_control/JTC -> arm
```

### Accepted design choices

- Prefer standard MoveGroup planning for the baseline.
- MoveIt Servo is optional as a comparison or experiment, not the main architecture.
- MoveIt provides FK/IK, joint limits, PlanningScene, self-collision checks, environment collision checking, and trajectory planning.
- The carried panel should be represented as an attached collision object once grasped.
- Keep MoveIt collision geometry and the Nav2 2D footprint as distinct abstractions.

### Limitation deliberately exposed by this phase
MoveIt may know the arm/panel is safe in 3D while Nav2 still sees only a fixed footprint. This mismatch motivates Phase 3.

### Implemented B3 baseline (2026-10-03)
Decided in the Phase 2 design discussion ([spec](superpowers/specs/2026-10-02-phase2-moveit-b3-design.md), [ADR 0009](adr/0009-moveit-panel-reconfiguration-baseline.md)):

- The base stops and MoveIt plans a collision-free reconfiguration (no arm motion while driving; Phases 4-5 cover that).
- Goals are computed panel poses (panel centre in `base_footprint`, with tolerances) or named SRDF states, given as scripted steps of a `task: mission` scenario.
- Nav2 keeps a fixed footprint while driving; `ReconfigurePanel` switches between the named profiles at standstill after checking that the planned and the measured robot + panel projection fit, and also resizes the collision monitor's stop and slowdown zones.
- MoveIt's world is the known geometry (exported Unity boxes inflated by 0.05 m, scenario boxes, a floor raised to the 0.15 m panel clearance). A lidar Octomap scene source exists (`--scene-source octomap`) but is not usable yet: MoveIt's Octomap misrepresents a box the lidar sees fully (open decision below).
- Missions: `narrow_gate_mission` (scenario 5 at 1.05 m), `wide_gate_mission` (1.30 m), `constrained_reconfiguration_mission` (scenario 6: a box blocks the qualified straight transition, so MoveIt must find another path). Results: [docs/experiments/moveit-arm](experiments/moveit-arm/README.md).

### `/cmd_vel` owner
**Nav2 Controller Server.**

---

## Phase 3: Nav2 + MoveIt + dynamic configuration-dependent footprint

### Research purpose
Implement and reproduce the dynamic-footprint idea from:

**K. Sagar, P. Long, C. G. Santiago, “Convex-Hull Based Dynamic Footprint of Mobile Manipulators for Collision-Safe Navigation in Nav2,” CoDIT 2026, pp. 2872-2878, DOI: 10.1109/CODIT70676.2026.11630999.**

### Paper idea
At each update:

1. Obtain current arm configuration.
2. Project mobile base + manipulator geometry onto the ground plane.
3. Include the grasped payload projection for this thesis.
4. Compute a convex hull of the projected geometry.
5. Publish the changing polygon to Nav2's costmap footprint topic.

The paper describes both lightweight geometric/link approximations and higher-fidelity URDF collision-geometry projection. The reported concept is designed to update Nav2 without rewriting the navigation stack.

### Project adaptation
For this thesis, compute the current 2D projection of:

```text
base + arm links + rigidly attached panel
```

and update the Nav2 footprint.

### Critical limitation
This solves:

> “What shape does the robot occupy now?”

It does **not** solve:

> “What arm/panel configuration should the robot adopt several metres in the future so a route becomes feasible?”

A current dynamic footprint can still cause Nav2 to reject a corridor that would become feasible after future panel reorientation. Future configuration-dependent feasibility is the motivation for whole-body and medium-horizon planning.

### Global planner note
NavFn is useful for initial baseline qualification but has limitations for orientation-dependent non-circular geometry. Smac State Lattice is a later candidate for a skid-steer/non-circular footprint baseline because it reasons in SE(2) with kinematic structure. This still does not add arm joint configuration to the global state.

### `/cmd_vel` owner
**Nav2 Controller Server.**

### Design constraints for later phases (2026-10-04)
- Build the footprint projection (base + arm + panel from the joint state) as a library independent of Nav2: the Phase 4 MPC needs the same geometry as continuous collision constraints, evaluated far more often.
- Keep one source of world geometry (the exported Unity boxes plus scenario obstacles, including overhead ones) read by Nav2's map, MoveIt and later the MPC, so every phase sees the same course.
- Keep the mission task, runner and scoring controller-agnostic so B4-B6 and arbitrary demo goals plug in without rework.

### Phase 3 comparisons

- fixed base-only footprint,
- fixed conservative base+payload footprint,
- dynamic convex-hull footprint,
- optional geometric approximation versus URDF/collision-geometry projection.

Record collision rate, false infeasibility, path length, clearance, footprint-update cost, and controller behavior near narrow passages.

### Implemented B4 baseline (2026-10-04)
Decided in the Phase 3 design discussion ([spec](superpowers/specs/2026-10-04-phase3-dynamic-footprint-design.md), [ADR 0010](adr/0010-dynamic-footprint-ownership.md)):

- Phase 3 is an implementation and qualification phase: B4 is `--footprint-mode dynamic` on any scenario (a short regression on the B3 missions, no B3 rerun campaign, no new B4 missions); the later phases run B4 on their own scenarios. The arm still reconfigures only at standstill.
- `mobile_manipulator_geometry` (no ROS graph) holds the robot model, ground projection, convex hull, exact outward offset (0.02 m; not the paper's centroid push) and the mesh and disc footprint models; `dynamic_footprint_node` publishes the padded hull at 20 Hz on a change above 0.01 m and is the only owner of both costmap footprints and the collision monitor zones (`footprint_mode` static / profiles / dynamic, checked before every run). In dynamic mode `ReconfigurePanel` checks the planned and measured hull against the global costmap (`HULL_IN_COLLISION`).
- Results ([docs/experiments/dynamic-footprint](experiments/dynamic-footprint/README.md)): regression 8/9 B3 missions with RPP, no contact (the failure is the 1.05 m throat stop); footprint strategies (paper Table III analogue, 60 drives): base-only 12/20 with 5 collisions, enlarged (`home`) 20/20 with 11 m detours where vertical carry fits, dynamic 17/20 with no collision (its 3 failures are throat aborts at base-only's rate); update cost 11 µs per evaluation, 43-61 µs per node tick (paper 689-857 µs); the disc model gives the same hull as the mesh model with this panel.
- Found on the way: in `home` the cantilevered panel sways `wrist_1` by up to 19 mrad while driving, so the hull republishes 7-8 times per drive (open decision below); NavFn checks only an inscribed circle and can open the 1.05 m gate for a hull a few millimetres narrower than the profile; Nav2's footprint padding leaves exact-zero coordinates unpadded.

---

## Phase 4: Replace MoveIt as the main motion planner with local whole-body MPC/QP

### Research purpose
Introduce continuous coordinated base-arm motion rather than two separate planners.

### Core state and control concept
A first kinematic formulation may use approximately:

```text
state x = [x_base, y_base, yaw_base, q1 ... q6]
control u = [v, omega, qdot1 ... qdot6]
```

The rigid panel pose is derived from base pose + arm forward kinematics + fixed grasp transform. Do not create independent payload DOFs unless the grasp itself is modeled as flexible.

### Environmental input after MPC enters

Measured limits of the Nav2 stack that the MPC should not inherit (Phases 1-2, 2026-10-04):

- **Costmap quantization.** At 5 cm cells, obstacle cells reach 0.05-0.10 m into a passage (cell quantization plus lidar noise); in the 1.05 m gate that leaves a centred vertical-carry robot 0.02-0.04 m per side. Tight clearances should come from the MPC's own representation (ESDF or exact known geometry), not from the Nav2 costmap; Nav2's resolution stays unchanged for the baselines.
- **Global plan shape.** The Smac Lattice plan S-bends through narrow passages depending on the start pose (17 % of plans through the 1.05 m gate, up to 0.10 m; open decision resolved 2026-10-03). Treat the Nav2 path as a route or corridor and let the medium-horizon layer and the MPC choose the line through tight spots; tracking it tightly would inherit the bend.
- **Compute.** Unity and the full stack run at a real-time factor of 1.00-1.04 on the development machine; the Nav2 controllers use about 6 % (RPP), 30 % (DWB) and 35-60 % (MPPI) of a core, `move_group` 3-16 % per reconfiguration, and the Octomap updater degraded arm tracking (arm-tracking CPU headroom, open decision). Measure the MPC budget against this when the solver is chosen.

Do not discard the useful Nav2/perception infrastructure when replacing the Nav2 local controller. The local MPC may consume an appropriate local costmap representation or, preferably for 3D arm/payload constraints, an ESDF/SDF or other obstacle representation produced from the same filtered perception pipeline. Nav2 remains infrastructure and long-range routing; the custom MPC replaces only local command authority.

### Main local MPC objective terms

- follow Nav2 global/reference path,
- track medium/reference state when Phase 5 exists,
- maintain obstacle clearance for base, arm, and panel,
- obey nonholonomic/skid-steer base constraints,
- obey joint position/velocity/acceleration limits as modeled,
- smooth control and state changes,
- maintain favorable arm configurations/manipulability if useful,
- avoid unnecessary arm motion and return toward a default transport posture where appropriate.

### Initial formulation
Start with a **linearized/LTV QP MPC**, not a complicated nonlinear formulation immediately.

At each cycle:

1. Warm start from the previous predicted trajectory.
2. Linearize dynamics/kinematics and collision-distance constraints around the nominal trajectory.
3. Solve one convex QP.
4. Apply the first control.
5. Shift the previous solution and repeat.

This is conceptually close to one-step SQP / sequential convexification / real-time iteration, depending on exact formulation.

### Why this is related to nonlinear SQP-RTI
SQP handles nonlinear problems by repeatedly linearizing/quadratizing and solving QP subproblems. SQP-RTI deliberately performs one real-time SQP iteration per control instant instead of fully converging each sample. Therefore the project's linearized QP MPC is not philosophically opposite to SQP-RTI; it is a simpler point on the same successive-linearization family.

### Key literature baseline

**Y. Wang, R. Chen, M. Zhao, “Whole-Body Model Predictive Control for Mobile Manipulation with Task Priority Transition,” ICRA 2025, DOI: 10.1109/ICRA55743.2025.11127515.**

Relevant lessons:

- whole-body MPC on a nonholonomic mobile manipulator is practical,
- average reported computation cost is about 4.3 ms,
- the reported architecture runs the optimizer at 100 Hz while lower-level control runs at 1000 Hz,
- SQP with Real-Time Iteration is used to keep nonlinear MPC reactive.

Do not treat these numbers as guaranteed for this project; use them as evidence that the target architecture is computationally credible.

### MPC frequency targets

- Bring-up: 20 Hz if needed for debugging.
- Main target: 50 Hz.
- Stretch/final design target: **100 Hz** if p99 timing and deadline behavior permit.

A 100 Hz optimizer has a 10 ms cycle budget. Use p95/p99 and deadline misses, not just mean time.

Prediction discretization does not need to equal optimizer frequency. A 100 Hz solver can predict with coarser time intervals over a 1-3 s horizon.

### `/cmd_vel` owner
**Custom whole-body MPC.**

Disable/bypass Nav2 local controller output as command authority. Nav2 remains a source of the global/reference path.

### Arm command path
MPC outputs joint references/velocities appropriate to the selected `ros2_control` interface. Retain the tested JTC/arm-control pipeline unless measurements show that a streaming controller is necessary.

---

## Phase 4B: MPC performance qualification before medium horizon

Do not stack Phase 5 on an unmeasured Phase 4.

Benchmark:

- N = 10/15/20/30 or other justified horizons,
- 20/50/100 Hz optimizer rates,
- collision-model fidelity,
- warm start on/off,
- QP solver choice,
- prediction-grid spacing,
- solver status/failure handling.

If MPC p99 is already near its deadline, optimize it before adding medium-horizon computation.

---

## Phase 5: Medium-horizon foresight + local whole-body MPC

### Research purpose
Give the planner enough foresight to start reorienting the panel before a future narrow passage while preserving a fast reactive local controller. The intended medium layer remains a continuous trajectory-optimization layer in the TrajOpt/SCP family conceptually, regardless of which compact trajectory representation is ultimately selected.

### Final hierarchy

```text
Nav2 global route
      |
      v
Medium-horizon optimizer
      |
      v
Local whole-body MPC
      |
      +--> /cmd_vel
      +--> arm references
```

### Key design choice: reduced task space
Do not automatically optimize dense `[x, y, yaw, q1..q6]` samples far into the future.

The medium layer should primarily decide what must be known early, for example:

- progress along Nav2 route,
- lateral base offset,
- base yaw,
- desired payload/end-effector position,
- desired payload/end-effector orientation,
- timing of panel reorientation,
- possibly a compact arm posture descriptor if task-space information alone is insufficient.

The local MPC should resolve the exact joint configuration whenever feasible.

This is a major computational design choice: long horizon = lower-dimensional task-space foresight; short horizon = high-dimensional whole-body feasibility.

### Key 2026 hierarchical-MPC paper

**W. Du, R. Long, J. Moura, J. Wang, S. Samadi, S. Vijayakumar, “Efficient Whole-Body Model Predictive Control for Online Compliant Dual-Arm Mobile Manipulation,” Journal of Field Robotics, first published 17 Aug 2026, DOI: 10.1002/rob.70320.**  
Preprint: arXiv:2410.22910, originally titled “An Efficient Representation of Whole-body Model Predictive Control for Online Compliant Dual-arm Mobile Manipulation.”

Relevant lessons:

- Bilevel/hierarchical MPC separates long-horizon task-space planning from shorter-horizon whole-body motion generation.
- Bézier parameterization uses a small set of control points as decision variables.
- Position, velocity, and acceleration are analytically related to the same control points, removing many transition/equality variables.
- Bézier convex-hull properties can simplify bounds on state/velocity/acceleration control points.
- Their comparison reports task-level MPC decision variables reduced from 224 under direct discretization to 96 in their Bézier formulation, with average compute reduced from 60.8 ms to roughly 6.2 ms in that experiment.
- Their whole-body comparison with a 5 s horizon reports 936 variables / 60.1 ms for dense discretization versus 108 variables / 12.5 ms for the Bézier representation in the dense-knot comparison.
- In a narrow-space experiment, the long layer uses up to a 15 s horizon while the whole-body layer uses 3 s; average compute is reported around 5.5 ms and 13.2 ms respectively, with a 20 ms control-loop period.

These results motivate compact continuous parameterization and dimensional separation. They do not require copying the paper's dual-arm formulation.

### Medium-horizon trajectory parameterization candidates

#### A. Piecewise Bézier
**Current first candidate to investigate.**

Advantages:

- very few optimization variables,
- analytic derivatives,
- convex-hull bounds,
- strong evidence from the Du et al. hierarchical MPC paper,
- piecewise segments reduce the global-coupling problem of one high-order Bézier curve.

Questions to benchmark:

- number of control points,
- segment count,
- continuity order,
- orientation representation for the panel,
- collision constraints between control points / continuous-time safety,
- sensitivity to local replanning.

#### B. MINCO
Strong second candidate because it is demonstrated in a 2026 payload-carrying mobile-manipulator planner very close to this thesis.

Advantages:

- piecewise polynomial trajectory,
- smoothness/continuity handled structurally,
- intermediate waypoints and segment durations can serve as compact optimization variables,
- supports time allocation optimization.

#### C. B-spline
Keep as a candidate when local support is valuable.

Advantages:

- changing a control point affects only a local trajectory region,
- useful for local medium-horizon replanning and incremental shape changes.

#### D. Dense direct discretization
Keep as a baseline for scientific comparison and debugging, not as the preferred final architecture unless benchmarks unexpectedly favor it.

### Important terminology

- Bézier / B-spline / MINCO = trajectory representation.
- SCP / SQP / SQP-RTI = optimization strategy.

They are not mutually exclusive. Example: a B-spline or piecewise-Bézier trajectory may be optimized by SCP with linearized collision constraints.

---

## Phase 6: System optimization and ablation, not architectural rescue

### Purpose
The full planner already works. Phase 6 determines which optimizations actually matter and produces the computational evaluation chapter.

### Candidate ablations

- local QP vs acados SQP-RTI,
- OSQP vs HPIPM or other structured QP choices,
- full vs partial condensing,
- preparation/feedback RTI split,
- AS-RTI if justified,
- uniform vs nonuniform prediction grid,
- 50 Hz vs 100 Hz MPC,
- medium-horizon dense vs Bézier vs MINCO vs optional B-spline,
- control-point count and segment count,
- medium planner always-on vs conditional activation,
- collision stages on/off,
- simple geometry vs higher-fidelity geometry,
- one solver thread vs multiple threads,
- default Linux scheduling vs CPU affinity / real-time scheduling,
- sequential vs parallel/asynchronous medium-planner execution.

The goal is to identify causal improvements, not merely report an opaque “optimized version.”

---

# 3. Computational optimization strategy

## 3.1 acados and solver escalation

acados is an optimal-control solver framework rather than merely one QP solver. It supports structured OCP-NLP/QP formulations, code generation, RTI, advanced-step RTI, nonuniform grids, warm starts, and partial condensing. Its documented default structured QP backend is `PARTIAL_CONDENSING_HPIPM`.

Reference: https://docs.acados.org/

### Recommended ladder

1. Basic linearized QP MPC.
2. Warm start from shifted previous trajectory.
3. Benchmark a simple QP backend.
4. Evaluate structure-aware HPIPM / partial condensing.
5. If nonlinear treatment materially improves robustness, evaluate acados SQP-RTI.
6. Split RTI into preparation and feedback phases to reduce control latency.
7. Use nonuniform prediction grids if distant horizon states can be coarser.
8. Evaluate AS-RTI only if ordinary SQP-RTI still leaves a useful performance/robustness gap.

### Why SQP-RTI is relevant
A nonlinear problem can be approximated locally as a QP using model Jacobians and linearized collision constraints. Classical SQP may solve several QPs per sample. RTI performs one major SQP step per control instant and relies on the next measurement/control cycle to continue the process.

### Partial condensing
MPC QPs have chain structure across shooting stages. Partial condensing groups stages to trade sparse horizon structure against denser smaller problems. This can outperform a generic dense treatment, but the optimal condensed horizon/block structure must be benchmarked rather than assumed.

---

## 3.2 Conditional computation

### Principle learned from recent whole-body payload planning
Do cheap work first and invoke expensive coupled/refinement computation only when geometry requires it.

Possible final behavior:

```text
open/easy region
    -> Nav2 reference + default panel posture + local MPC

predicted narrow/confined region or current solution becoming infeasible
    -> activate medium-horizon whole-body/task-space optimization

candidate trajectory comfortably far from obstacles
    -> cheap collision validation

candidate near critical geometry / cheap validation fails
    -> high-fidelity collision refinement
```

### Implementation order
Design the interfaces for conditional operation early, but initially run the medium planner deterministically/always-on during validation. Add condition triggering only after the full hierarchy is correct, otherwise failures become difficult to diagnose.

### What may trigger higher computation
Possible signals to research/benchmark:

- predicted footprint/path clearance below threshold,
- local MPC slack variables rising,
- repeated local MPC infeasibility,
- curvature / corridor-width changes ahead,
- current panel orientation predicted to collide with future geometry,
- distance-field gradients indicating narrowing free space,
- Nav2 route entering tagged manipulation/narrow-passage regions,
- failure of a cheap swept-volume validator.

Do not hard-code a final trigger before experiments establish useful thresholds.

---

## 3.3 Staged collision checking

Collision checking is expected to become one of the largest compute consumers.

### Preferred architecture

```text
all map geometry
      |
      v
broad-phase spatial filter
      |
      v
nearby obstacle candidates
      |
      v
cheap collision / distance approximation
      |
      v
critical link-obstacle pairs only
      |
      v
high-fidelity gradient / swept-volume evaluation
```

### Candidate techniques

- axis-aligned bounding box broad phase,
- spatial hash / voxel neighborhood filtering,
- primitive approximations (capsules, spheres, boxes),
- ESDF/SDF distance and gradient queries,
- MoveIt/FCL for baseline exact/discrete checks,
- bit-packed voxel kernels / early exit,
- later swept-volume or kinematically coupled SDF methods if justified.

### Payload representation
Treat the grasped panel as a rigid link attached to the end effector. Preserve its thin, extended geometry. A single huge bounding sphere may create false infeasibility in narrow passages.

---

# 4. Lessons from the 2026 payload-carrying whole-body planner

**Y. Li, L. Yin, T. Zhang, R. Xue, H. Zhu, N. Chen, S. Liang, Y. Liu, F. Zhang, “Real-time Whole-Body Motion Planning for Mobile Manipulators Carrying Arbitrarily Shaped Payloads via Kinematically-Coupled SVSDF,” arXiv:2608.07005v1, 7 Aug 2026.**

Reference: https://arxiv.org/abs/2608.07005

This paper is much larger than the intended master's scope. It should be mined for computational/architectural lessons, not reproduced wholesale.

## 4.1 Their planning hierarchy

1. **Front end:** Hybrid A* for the mobile base + Multilayer Constrained RRT*-Connect for manipulator configurations.
2. If decoupled search fails, invoke whole-body RRT* fallback.
3. **Mid end:** convert discrete waypoints to a smooth/kinematically feasible continuous trajectory.
4. If that trajectory is already collision free, execute/bypass expensive back-end refinement.
5. **Back end:** use KC-SVSDF trajectory optimization for continuous high-fidelity collision safety.

### Lesson for this thesis
Use cheap/low-dimensional computation where it is sufficient and spend coupled whole-body compute only in difficult geometry. The thesis remains optimization/MPC based; there is no requirement to copy their A*/RRT* stack.

## 4.2 Their collision strategy

- Chain-decomposed kernel per mobile base/manipulator link.
- Payload treated as an additional rigid link attached to the end effector.
- Precomputed voxelized kernels and bit-level queries.
- Forward kinematics resolves current link transforms.
- Early exit on collision.
- KC-SVSDF back end propagates collision-avoidance gradients consistently along the kinematic chain.

### Lesson for this thesis
Collision geometry should be hierarchical and link-aware. Do not make the local MPC query every raw point-cloud point against every mesh triangle at every horizon node.

## 4.3 MINCO trajectory parameterization
Their unified variable is based on base planar translation plus arm configuration and is parameterized with MINCO piecewise polynomials. Intermediate waypoint parameters and time allocations determine polynomial coefficients while boundary/continuity conditions are satisfied structurally.

### Lesson for this thesis
MINCO is a serious medium-horizon candidate, particularly if smoothness and optimized segment timing are valuable.

## 4.4 Important difference from this thesis
Their reported complete planner may require seconds in difficult scenarios. Therefore it is a reference for medium/global planning architecture, initialization, collision optimization, and conditional refinement, **not** the timing model for a 50-100 Hz local MPC.

---

# 5. Lessons from the 2026 hierarchical MPC paper

**Du et al., Journal of Field Robotics 2026, DOI 10.1002/rob.70320.**

Reference: https://doi.org/10.1002/rob.70320

### Their high-level idea

```text
MPC-T: long horizon, lower-dimensional task-space trajectory
                     |
                     v
MPC-W: short horizon, whole-body trajectory
```

This closely supports the thesis decision to use a reduced medium-horizon representation feeding a short whole-body MPC.

### Bézier-specific lessons

- Only control points need to be optimization variables.
- Derivative curves are analytic functions of the same control points.
- Dense evaluation/time knots do not require equally dense decision-variable sets.
- Convex-hull bounds can provide efficient sufficient conditions for state/velocity/acceleration limits.
- Compact representation makes long horizons computationally possible.

### Why piecewise Bézier is now a primary candidate
A single Bézier segment has global support, which may be awkward for local replanning. Piecewise Bézier can preserve compact analytic representation while localizing modifications through multiple segments and continuity constraints.

### Why MINCO remains competitive
MINCO naturally handles piecewise smooth polynomial trajectories and segment time allocation and is validated in the 2026 bulky-payload planner.

### Why B-spline remains a candidate
B-spline control points have local support, attractive for incremental re-planning near a changing obstacle. Benchmark rather than assume.

---

# 6. Multicore, scheduling, and asynchronous execution

## 6.1 Conceptual CPU separation
Exact core assignments depend on the machine, but the software should permit a structure such as:

```text
Core/group A: low-level control
Core/group B: local whole-body MPC
Core/group C: medium-horizon optimizer
Core/group D: perception/filtering/SDF
Core/group E: Nav2/costmaps
Other cores: ROS/DDS/OS/Unity as available
GPU: Unity rendering and any GPU-accelerated perception if later used
```

### Important
Wall-clock compute of the stack is not the sum of all component runtimes because many modules run concurrently.

## 6.2 Asynchronous medium planner
Never block the fast MPC while the medium horizon is solving.

```text
medium planner computes candidate trajectory
                 |
                 v
          timestamped reference buffer
                 |
                 v
       local MPC consumes latest valid reference
```

While a new medium plan is being computed, the local MPC continues using the last valid reference.

## 6.3 Linux optimization ladder
Initially:

- ordinary scheduling,
- sensible ROS 2 callback groups,
- separate processes where isolation is useful,
- preallocated solver structures where easy.

Later, only if timing data justifies:

- dedicated CPU affinity,
- `SCHED_FIFO` / real-time priorities,
- PREEMPT_RT kernel,
- `mlockall()` / page-fault avoidance,
- eliminate dynamic allocation from hot loops,
- reduce logging/I/O in control threads,
- explicit executor/thread-count tuning.

## 6.4 Multithreading warning
Do not assume a QP/NLP solver should consume all CPU cores. Small structured MPC problems can lose time to synchronization overhead. Benchmark one solver thread on a dedicated core versus internal multithreading.

---

# 7. Computational evaluation plan

Computational performance is a thesis result, not an implementation footnote.

## 7.1 Per-component timing
Instrument at minimum:

- input message timestamp,
- compute-start timestamp,
- linearization duration,
- collision-processing duration,
- solver duration,
- compute-end timestamp,
- command/reference publish timestamp,
- medium-planner duration,
- perception/map/SDF update duration,
- end-to-end state-to-command latency.

## 7.2 Statistical reporting
For every real-time-relevant component report:

- mean,
- median,
- p90,
- p95,
- p99,
- max,
- deadline miss count/rate.

Do not evaluate real-time performance using mean latency alone.

## 7.3 Resource reporting

- CPU utilization overall and by major process/thread if possible,
- memory usage,
- solver iterations/status,
- warm-start failures,
- fallback/recovery counts,
- local map update rate,
- planner update rate.

## 7.4 Task-performance metrics

- navigation success rate,
- minimum base/arm/payload clearance,
- collision count,
- false infeasibility / rejected feasible corridor rate,
- path length,
- executed trajectory length,
- traversal time,
- base tracking error,
- arm/end-effector/payload tracking error,
- control smoothness / acceleration / jerk where relevant,
- replanning count,
- time spent in constrained/expensive planning modes,
- percentage of medium-horizon calls avoided by conditional computation.

## 7.5 Fixed benchmark scenarios
Build and keep deterministic scenes such as:

1. Open-space navigation.
2. Static obstacle detour.
3. Worker/cart crossing local path.
4. Narrow gate rejected by conservative footprint.
5. Passage feasible only after payload reorientation.
6. Passage requiring coordinated base yaw + arm motion.
7. Persistent blockage requiring route-level replan.
8. Dense clutter stressing collision evaluation.
9. Infeasible environment.
10. Rapidly changing obstacle near the current MPC horizon.
11. Obstacle course with overhead obstacles that the arm and panel must avoid while the base drives through (for the MPC and medium-horizon phases; the decoupled baselines can only stop and reconfigure or detour). Needs 3D obstacle knowledge for the arm while driving: known geometry, or the lidar once the Octomap insertion fault is fixed or replaced by the MPC's ESDF; the Livox does not see below 0.2 m within 1.5 m.

Use identical scenarios to compare variants.

Live demonstration (MPC and medium-horizon phases): an arbitrary goal anywhere on the map, set interactively (RViz "2D Goal Pose" or equivalent), reached with the full stack. This needs arm-configuration decisions without scripted mission steps, handling of unreachable goals and goals in tight spots, and RViz reaching the container from the macOS host.

---

# 8. Baseline matrix to preserve

| ID | Base global planner | Base local controller | Arm planner/controller | Footprint / geometry | Medium horizon |
|---|---|---|---|---|---|
| B1 | Nav2 | DWB or RPP bring-up | fixed arm | fixed | none |
| B2 | Nav2 | Nav2 MPPI | fixed arm | fixed | none |
| B3 | Nav2 | Nav2 controller | MoveIt | fixed | none |
| B4 | Nav2 | Nav2 controller | MoveIt | dynamic convex-hull (`footprint_mode:=dynamic`, 2026-10-04) | none |
| B5 | Nav2 | custom linearized QP MPC | MPC + ros2_control | whole-body collision model | none |
| B6 | Nav2 | acados SQP-RTI MPC variant | MPC + ros2_control | whole-body collision model | none |
| F1 | Nav2 | custom MPC | custom MPC | whole-body predicted geometry | dense medium baseline |
| F2 | Nav2 | custom MPC | custom MPC | whole-body predicted geometry | piecewise Bézier |
| F3 | Nav2 | custom MPC | custom MPC | whole-body predicted geometry | MINCO |
| F4 | Nav2 | custom MPC | custom MPC | whole-body predicted geometry | B-spline if implemented |

Additional switches for ablation:

- conditional medium planner on/off,
- staged collision checking on/off,
- low/high fidelity payload geometry,
- solver warm-start on/off,
- 50/100 Hz local MPC,
- default/pinned CPU scheduling.

---

# 9. Key design decisions already accepted

1. **Continuous final planner:** no fundamentally discrete mode planner.
2. **Hierarchical horizons:** Nav2 long range, medium-horizon continuous optimization, short-horizon MPC/QP.
3. **Nav2 retained:** it remains the long-range base-routing tool in the final architecture.
4. **MoveIt is a baseline/tool, not the final local planner.**
5. **Dynamic footprint is a baseline and current-shape safety mechanism, not future configuration planning.**
6. **Current footprint changes should use Nav2's native footprint update mechanism rather than inventing a custom costmap layer solely for current geometry.**
7. **Custom costmap/planning costs remain useful for future state-dependent feasibility, manipulation zones, predicted swept-volume penalties, or route-level information that native current footprints cannot encode.**
8. **Local whole-body MPC eventually owns `/cmd_vel`.**
9. **Medium-horizon planner provides a reference; local MPC is allowed to correct it reactively.**
10. **Default payload posture is desirable:** carry the panel in a favorable transport configuration when possible and deviate only when geometry requires, then return when practical.
11. **Payload is modeled as rigidly attached geometry.**
12. **Collision computation must be hierarchical/staged if it becomes a bottleneck.**
13. **Compact medium-horizon trajectory parameterization should be present from first real Phase 5 implementation.**
14. **Piecewise Bézier is the current first candidate to test; MINCO is the key comparison; B-spline is retained as a local-support candidate.**
15. **SCP/SQP is compatible with any of these trajectory representations.**
16. **Local MPC starts simple:** linearized QP first, acados/SQP-RTI later if justified.
17. **100 Hz local MPC is a design target, not a guarantee; 50 Hz is an acceptable fallback.**
18. **Low-level control may target ~1000 Hz on real hardware, but simulator physics rate limits what is meaningful in Unity.**
19. **Medium planning is asynchronous and must not block the local MPC.**
20. **Conditional computation is designed in early but enabled only after deterministic full-hierarchy validation.**
21. **Structural optimizations early; micro-optimizations late.**
22. **Phase 6 is an optimization/ablation phase, not a rewrite.**
23. **Computational evaluation is mandatory and uses tail latency/deadline statistics.**
24. **Do not let multiple modules publish competing base commands.**
25. **Preserve baselines rather than deleting old implementations after improvements.**

---

# 10. Literature and technical references

## Dynamic footprint baseline
K. Sagar, P. Long, C. G. Santiago, “Convex-Hull Based Dynamic Footprint of Mobile Manipulators for Collision-Safe Navigation in Nav2,” 12th International Conference on Control, Decision and Information Technologies (CoDIT), 2026, pp. 2872-2878. DOI: **10.1109/CODIT70676.2026.11630999**.

## Whole-body SQP-RTI MPC
Y. Wang, R. Chen, M. Zhao, “Whole-Body Model Predictive Control for Mobile Manipulation with Task Priority Transition,” IEEE International Conference on Robotics and Automation (ICRA), 2025, pp. 13356-13362. DOI: **10.1109/ICRA55743.2025.11127515**.  
Project page: https://wbmpc.github.io/

## Payload-carrying whole-body planner
Y. Li, L. Yin, T. Zhang, R. Xue, H. Zhu, N. Chen, S. Liang, Y. Liu, F. Zhang, “Real-time Whole-Body Motion Planning for Mobile Manipulators Carrying Arbitrarily Shaped Payloads via Kinematically-Coupled SVSDF,” arXiv:2608.07005v1, 7 Aug 2026.  
https://arxiv.org/abs/2608.07005

## Hierarchical / bilevel MPC with Bézier representation
W. Du, R. Long, J. Moura, J. Wang, S. Samadi, S. Vijayakumar, “Efficient Whole-Body Model Predictive Control for Online Compliant Dual-Arm Mobile Manipulation,” *Journal of Field Robotics*, first published 17 Aug 2026. DOI: **10.1002/rob.70320**.  
Earlier preprint: arXiv:2410.22910.  
https://doi.org/10.1002/rob.70320

## acados
Official documentation: https://docs.acados.org/  
Relevant features: SQP, SQP-RTI, preparation/feedback split, Advanced-Step RTI, nonuniform grids, partial condensing, HPIPM, code generation.

## Nav2 footprint support
Nav2 footprint setup/tuning documentation: https://docs.nav2.org/  
Nav2 supports polygon footprints and runtime footprint updates through the costmap footprint topic for state-dependent robot shape changes.

---

# Open decisions

Decisions deliberately deferred; resolve them explicitly and record the outcome in the log below.

- **2026-09-28 - Baseline-matrix scope (Section 8).** Decide which baselines are core thesis results and which are optional. Candidate optional items: B6 (acados SQP-RTI) and F4 (B-spline). Until decided, treat B3-F4 as provisional.
  - *Resolved for Phase 1 (2026-09-28):* Regulated Pure Pursuit is the bring-up controller; both DWB (B1) and Nav2 MPPI (B2) are kept as Phase 1 controller baselines. Initial recoveries are limited to wait and clear-costmap until spin/backup are qualified against the payload footprint.

- **2026-09-28 - Local-costmap ghost clearing.** Removed obstacles leave 50-65 % of their voxels marked in open directions because the simulated Livox reports misses as directionless zero points (docs/experiments/local-costmap). Options: Spatio-Temporal Voxel Layer with time decay (new container dependency), or Unity encoding misses as max-range points at fixed message size for clearing-only rays. Needed before dynamic-obstacle scenarios (3, 10).
  - *Resolved (2026-09-28):* Spatio-Temporal Voxel Layer with 10 s linear decay and a +/-7.2 deg, 1.5-6.0 m clearing frustum; all removed obstacles cleared within 11 s.

- **2026-09-28 - 1.05 m gate with pre-rotated panel.** Standard Nav2 (RPP, STVL, Lattice) stops at the throat because lidar range noise narrows the local-costmap opening to 0.85 m (0.14 m nominal margin per side).
  - *Resolved (2026-09-28):* kept as a Phase 1 baseline finding; a 1.30 m comparison gate (`WideGate_1p30m`, 0.265 m per side in vertical carry) was added and passes.
  - *Update (2026-09-28, controller baselines):* the throat is marginal rather than impassable: 5 of 9 runs passed (RPP 2/3, DWB 2/3, MPPI 1/3), and the failures stop under the collision monitor without contact (docs/experiments/nav2-navigation).

- **2026-09-28 - Arm feedback timeout vs. Unity stream pauses.** Unity's `/clock` and `/arm/state` reach ROS with pauses of 0.25-0.4 s about once a minute and occasionally 0.6-1.0 s, clustered around Nav2 launches and shutdowns. The ROS arm hardware's 0.5 s feedback timeout then latches a fault until arm control restarts; this cost 3 of the 45 batch runs (the runner now restarts arm control between runs). Options: find and remove the pause source (Unity main thread or ROS-TCP endpoint), or revisit the 0.5 s timeout in the arm safety contract. Decide before long unattended runs or arm motion during navigation (Phase 2).
  - *Resolved (2026-09-29):* the pauses come from the ROS-TCP endpoint, whose single thread forwards every Unity stream and blocks inside Fast DDS publish for about 0.3 s whenever ROS processes start or die uncleanly (docs/experiments/ros-tcp-stalls). The 0.5 s timeout is kept; stale feedback now deactivates the arm hardware and a supervisor re-activates it (ADR 0005 amendment). Nav2 already shuts down cleanly on SIGINT, and an orderly lifecycle shutdown did not reduce the pauses; a middleware comparison (2026-09-30) found no pauses with Cyclone DDS, and the ROS side now uses it (ADR 0008); the recovery supervisor stays as a safety net.

- **2026-09-28 - MPPI baseline (B2) tuning.** Stock Nav2 MPPI, forward-only, cuts corners on the home-footprint detours (cross-track p95 0.16-0.33 m, lowest clearance 0.075 m) and ends 0.18-0.50 m from the goal when the final heading change is large, because it turns on a forward arc after the stateful goal checker has latched the position. Keep it stock as the B2 baseline, or tune it (for example allow reversing within the envelope, weight the goal critics, or use a non-stateful goal checker) and keep stock MPPI as a variant.
  - *Resolved (2026-09-29):* keep stock MPPI as the B2 baseline with no further tuning; its corner cutting, goal overshoot, and CPU cost are reported as baseline results. RPP and DWB are the primary Phase 1 baselines.
  - *Update (2026-10-04):* the overshoot is now scored as a failure (`off_goal`, decision log 2026-10-04) instead of a success with a large final error; with the stateful goal checker kept, stock MPPI reaches 12/24 in the Phase 1 rerun and 5/9 in B3.

- **2026-10-01 - Global costmap and transient obstacles (scenario 3 vs. 7).** With live lidar obstacles in the global costmap (the navigation default since 2026-09-30) a crossing worker's walked line stays in the global costmap for the 10 s decay, and the replan-if-invalid tree immediately detours around its far end, where the worker is going: 6/10 successes with 13-20 m paths, against 9/9 on a near-straight 7.9 m route with the static global costmap (docs/experiments/nav2-navigation). The same layer is what makes the persistent blockage (scenario 7) solvable (9/9 against 0/9). Options: (a) keep the default and report both scenarios as Phase 1 baseline behaviour; (b) make the static global costmap the default again and accept scenario 7 failing; (c) keep the global layer but have the tree wait before rerouting around a newly blocked path (for example, replan only after the path has stayed invalid for a few seconds), so transient obstacles are handled locally and persistent ones still reroute; (d) a shorter decay or marking threshold in the global layer only. (c) and (d) change the Nav2 baseline and would need the full scenario set rerun.
  - *Resolved (2026-10-01):* a variant of (c)/(d) at the costmap level: `PersistentObstacleLayer` marks a global-costmap cell only after 2 s of observations (gaps up to 1 s) and clears it 10 s after the last one. Default for navigation from commit `1d04eb4`; 33/33 runs succeeded across scenarios 2, 3, 7, open space and the 1.30 m gate with RPP, DWB and MPPI. Decided before Phase 2 because every later phase follows the Nav2 global route, so the baselines are recorded with this layer from here on; `global_obstacles:=live` and `static` remain for comparison.

---

- **2026-10-03 - MoveIt Octomap insertion fault (Phase 2 Octomap scene source).** With `--scene-source octomap` the filtered Livox cloud contains the whole tall box beside the robot, but MoveIt's Octomap holds only 9-16 voxels on it and a ~5 m^2 artefact layer at 1.0-1.25 m; MoveIt planned through the box and the panel touched it in all 5 runs (docs/experiments/moveit-arm). Find the cause (frame or stamp handling in the point-cloud updater, the Livox cloud layout, MoveIt's self-filter) before any experiment uses perceived obstacles; the B3 baseline uses known geometry. Rechecked after the lidar noise fix (2026-10-03): unchanged (10 voxels on the box, panel contact), so the repeating noise was not the cause.
- **2026-10-03 - CHOMP post-processing for B3.** The B3 pipeline is OMPL RRTConnect with time-optimal parameterization and Ruckig smoothing. CHOMP after OMPL would add obstacle clearance and smoothness (planned clearance to the inflated boxes in `constrained_reconfiguration_mission` was often below 0.02 m). Decide whether to add it as a B3 variant measured on the same missions. Smoothing can also push a grazing OMPL path into the box, which MoveIt then rejects (3 of 12 constrained reconfigurations on 2026-10-03); `ReconfigurePanel` now re-plans up to 3 times within the planning deadline, which CHOMP would make less necessary.
  - *Resolved (2026-10-03):* deferred. With the re-plan B3 needs no smoothing aid, and the B3 baseline stays OMPL + TOTG + Ruckig; CHOMP can be added later as a separately labelled variant on the same missions.
- **2026-10-03 - Arm-tracking CPU headroom (before Phase 4).** With the Octomap scene source, move_group's extra ~35 % of a core pushed the arm controller's path error past the 0.15 rad acceptance in Unity (docs/experiments/moveit-arm). A 50-100 Hz whole-body MPC will load the same container. Measure how much CPU the arm control path can lose before tracking degrades (for example with a synthetic load beside a qualified transition) and decide core pinning or a CPU budget before Phase 4.
- **2026-10-03 - Phase 1 rerun with the persistent global layer.** The B1/B2 controller baselines were recorded before `PersistentObstacleLayer` became the navigation default; only scenarios 2, 3, 7, open space and the 1.30 m gate were rerun with it. Rerun the full Phase 1 scenario set so B1/B2 sit on the same footing as B3 and later baselines. Phase 1 was also recorded with the repeating lidar noise and the 4 sigma self-filter band (decision log 2026-10-03), so every Phase 1 scenario needs the rerun, not only the gates.
  - *Resolved (2026-10-04):* all eight driving scenarios x RPP, DWB, MPPI x 3 and the five plan-only scenarios rerun at `849f19b` with the persistent layer, the corrected lidar noise and the stricter drive scoring (decision log 2026-10-04): RPP 22/24, DWB 22/24, MPPI 12/24 (11 of its failures `off_goal`), no contact; the 1.05 m vertical-carry throat 8/9, the 1.30 m gate 9/9 (docs/experiments/nav2-navigation, `runs-2026-10-04`). These are the B1/B2 baselines from here on.
- **2026-10-03 - The 1.05 m gate under realistic lidar noise.** With independent 0.02 m range noise the gate posts' returns reach about 0.09 m into the throat, which leaves 0.14 m per side in vertical carry; 3 of 9 B3 throat drives failed (RPP 1/3, DWB 2/3, MPPI 3/3) and DWB needed up to 140 s. The local and global voxel layers mark single returns (`mark_threshold: 0`, `voxel_min_points: 0`). Options: keep the gate as the deliberately marginal scenario and report it; require a minimum number of returns per voxel (changes every navigation baseline, so with the Phase 1 rerun); or rely on the 1.30 m gate for comparisons. Decide before Phase 3, whose B4 runs the same missions.
  - *Resolved (2026-10-03):* keep the gate and the stock planner unchanged; the gate is the deliberately tight stress test and the 1.30 m gate the main comparison. Two mechanisms decide a throat drive. (1) The Smac Lattice plan: in the narrow gate it often S-bends (about 0.10 m to one side, then 0.04-0.07 m off centre at the posts) depending on the start pose to the millimetre, with the gate centreline on a costmap cell boundary; 334 of 1925 plans on a grid of starts (-1 to -5.5 m) and goals (-8.4 to -12 m) bent, 26 of 55 on the mission route, none at the wide gate or in open space. It is already in the raw lattice path (primitives: 0.15 m straight steps, smallest lateral move 0.10 m; analytic expansion only within 3 m of the goal), so the smoother is not the cause. `non_straight_penalty` 2.0 with `analytic_expansion_max_length` 6.0 cut it to 1.4 % (0 on the scenario routes) but brought obstacle detours 0.07-0.19 m closer to the box in a live A/B pilot, and no expansion cost limit separates the two (gate cells cost more than the detour shortcuts); `non_straight_penalty` 2.0 alone left 5.5 %. The planner therefore stays stock and the S-bend is a property of the B1-B3 baselines. (2) The effective opening: post cells reach 0.05-0.10 m into the throat (cell quantization plus noise), leaving a centred robot 0.02-0.04 m per side, so a robot reaching the posts a few centimetres off centre or a few degrees turned is stopped (RPP), creeps (DWB) or hesitates (MPPI); this is the fixed-footprint limitation the later phases address. A footprint-switch race found on the way (the mission sent the drive goal before the global costmap had re-inflated for the new footprint) is fixed, but it did not cause the bends: plans made after the fix bend the same way.

- **2026-10-04 - Dynamic footprint updates while driving in `home` (B4).** With the panel horizontal (`home`), `wrist_1` sways by up to 18.9 mrad under base acceleration and turning (vertical carry at most 3.2 mrad), which moves the hull by more than the 0.01 m publish threshold: 35-40 publishes over 5 drives at each gate, each re-inflating both costmaps; no drive failed from it (docs/experiments/dynamic-footprint). Options: keep it (the footprint reflects the measured arm); a larger threshold; an asymmetric one (grow at once, shrink only at rest); or footprints from the arm controller's reference instead of the measured joints. Decide before the later phases compare against B4 on drives in `home`.

# 11. Decision/benchmark log template

Append entries chronologically rather than silently overwriting history.

```markdown
## YYYY-MM-DD - Decision title

Phase:
Component:
Problem / hypothesis:
Variants considered:
Decision:
Reason:
Benchmark scenario(s):
Metrics before:
Metrics after:
Trade-offs / regressions:
Keep old variant as baseline? yes/no
Follow-up:
References:
```

## 2026-10-03 - B3: MoveIt panel reconfiguration at standstill

Phase: 2
Component: `mobile_manipulator_moveit_config`, `mobile_manipulator_manipulation` (ReconfigurePanel), mission scenarios
Problem / hypothesis: A decoupled Nav2 + MoveIt baseline can pass a gate the home footprint cannot by reconfiguring the panel at standstill, while exposing the fixed-footprint limitation.
Variants considered: arm motion while driving (dropped; Phases 4-5), named poses only vs computed panel-pose goals, scripted vs route- or failure-triggered reconfiguration, fixed vs switched vs computed Nav2 footprint, known geometry vs Octomap ([spec](superpowers/specs/2026-10-02-phase2-moveit-b3-design.md)).
Decision: Reconfigure at standstill to computed panel poses in scripted mission steps; switch named footprint profiles (costmaps and collision-monitor zones) after checking the planned and measured projection; known geometry first ([ADR 0009](adr/0009-moveit-panel-reconfiguration-baseline.md)).
Reason: Cleanest decoupled baseline inside the qualified arm envelope; leaves Phase 3 (dynamic footprint) one seam to replace.
Benchmark scenario(s): `narrow_gate_mission` (5), `wide_gate_mission`, `constrained_reconfiguration_mission` (6); RPP, DWB, MPPI x 3.
Metrics before: Phase 1 home footprint detours around the gate in 41-46 s (11 m path); panel pre-rotated crosses in 26-34 s.
Metrics after (rerun 2026-10-04 at `849f19b`, with the lidar noise fix, the re-plan, the footprint refresh wait and `off_goal` scoring): 21/27 missions (RPP 9/9, DWB 7/9, MPPI 5/9), 49/49 reconfigurations (no re-plan needed), no contact, every Unity arm check passes; reconfigure + cross 33-41 s at the 1.30 m gate against 43-45 s for the Phase 1 home detour and 27 s with the panel already vertical; the 1.05 m throat drive passed 7 of 9 (DWB failed twice); three MPPI drives ended `off_goal`; planning 0.02-0.72 s, motion 2.0-5.6 s, `move_group` 3-16 % of a core and 70-71 MB per reconfiguration; MoveIt transitions 19/20 within arm acceptance (the 20th a Unity feedback stall) ([results](experiments/moveit-arm/README.md)). Earlier series are superseded: 2026-10-02 (25/27, repeating lidar noise) and 2026-10-03 at `e4dbc1d` (23/27 as reported, 21/27 rescored).
Trade-offs / regressions: Fixed profiles leave a vertical-carry panel goal +/-0.01 m and +/-0.01 rad; KDL gives a different arm configuration every run; the collision monitor zones must be dynamic for missions; the 1.05 m throat is marginal, and its Smac Lattice plan S-bends depending on the start pose (open decision resolved 2026-10-03: kept as the stress test).
Keep old variant as baseline? yes (Phase 1 B1/B2 unchanged; `dynamic_monitor_zones` defaults to false)
Follow-up: fix the Octomap insertion fault before using perceived obstacles; CHOMP post-processing (deferred 2026-10-03); Phase 3 B4 on the same missions.
References: docs/experiments/moveit-arm, ADR 0009.

## 2026-10-03 - Independent lidar range noise and a 6 sigma self-filter band

Phase: all (simulation sensor contract)
Component: Unity lidar (`IidNoiseRaycastLiDARSensor`), `livox_robot_filter`
Problem / hypothesis: B3 runs depended on the Play session: in some sessions the robot's own cell was marked in the global costmap at every stop ("Start occupied"; MPPI failed the narrow gate 0/3). UnitySensors' noise job restarted one random sequence every scan, so the "Gaussian" range noise was a fixed per-session pattern (about 3,200 distinct values per 20,000-point scan, never beyond about 3.5 sigma); a large draw on the arm mount leaked through the self-filter in the same spot every pattern cycle.
Variants considered: widen the filter only (keeps the wrong noise); embed and patch the package (500 MB of scan patterns in the repository); a project sensor reusing the package's public jobs with its own noise.
Decision: The robot uses `IidNoiseRaycastLiDARSensor` (noise a pure function of session seed, scan and point index; installed by `configure_mobile_manipulator`), and the self-filter band is 6 sigma (0.12 m) because about 21,000 self-returns a second made 4 sigma leak about 16 a minute (8 in 30 s) into the 10 s voxel memory.
Reason: Every consumer of the lidar (costmaps, Octomap, a later ESDF for the MPC) needs the documented sensor model; the filter must hold with genuine Gaussian tails.
Benchmark scenario(s): B3 missions (27 runs).
Metrics before: 3,177 distinct noise values per scan, identical scan to scan; self-marked frames in whole sessions (old RPP batch, interim MPPI batch).
Metrics after: 19,998 distinct values per scan, sigma 0.0200, 3.3e-5 beyond 4 sigma (Gaussian 3.2e-5); no self-return leak in 600 scans; no self-marked costmap frame in 27 B3 runs.
Trade-offs / regressions: Obstacles within 0.12 m in front of the robot's own surfaces along rays that hit the robot are dropped (inside the robot envelope); realistic noise tails make the 1.05 m throat harder (open decision). The package's depth and RGB-D cameras share the noise bug; any camera added later needs the same treatment.
Keep old variant as baseline? no (the old noise was a defect); Phase 1 must be rerun on the new sensor.
Follow-up: Phase 1 rerun; the 1.05 m gate decision (both done 2026-10-04 and 2026-10-03).
References: docs/experiments/moveit-arm ("Lidar noise and the self-filter").

## 2026-10-04 - A drive succeeds only if the base ends at the goal

Phase: all (benchmark scoring)
Component: `navigate_run` (`drive_status`), mission and navigate tasks, `tools/summarize_nav_runs.py`
Problem / hypothesis: Nav2's stateful goal checker stops checking the position once the base has passed within 0.15 m; stock MPPI then sometimes kept driving on a forward arc while turning to the goal heading, and Nav2 reported success up to 3.8 m from the goal. Missions continued from the wrong place, and success rates counted these drives.
Variants considered: (a) keep the stateful checker and score the final pose; (b) a non-stateful checker (changes the controller behaviour of every baseline).
Decision: (a): a drive is reached only if Nav2 succeeded and the base ended within the goal checker's tolerances plus 0.02 (0.17 m, 0.17 rad, for settling after the result); otherwise `off_goal`, which fails the navigate task and stops a mission. Nav2's own result stays in the report as `nav2_status`; the summarizer rescores older summaries the same way.
Reason: RPP and DWB never ended beyond 0.149 m in 312 successful drives, so the margin separates settling from wandering; the baselines stay stock and the scoring stops overstating them.
Benchmark scenario(s): all Phase 1 driving scenarios and B3 missions.
Metrics before: 25 of 121 MPPI successes (21 %) ended beyond 0.17 m (0.18-3.78 m).
Metrics after: MPPI 12/24 in the Phase 1 rerun (11 failures `off_goal`), 5/9 in B3; RPP and DWB unaffected.
Trade-offs / regressions: Earlier MPPI success counts in the experiment notes are too high (18 Phase 1 summaries and 2 B3 summaries at `e4dbc1d` rescore to `off_goal`).
Keep old variant as baseline? no (scoring fix; Nav2 behaviour unchanged)
Follow-up: none.
References: docs/experiments/nav2-navigation (2026-10-04 full rerun), docs/experiments/moveit-arm.

## 2026-10-04 - B4: dynamic convex-hull footprint

Phase: 3
Component: `mobile_manipulator_geometry`, `dynamic_footprint_node`, `ReconfigurePanel` (dynamic mode), `footprint_mode` launch switch, runner and summarizer
Problem / hypothesis: A configuration-dependent footprint (Sagar et al., CoDIT 2026) makes Nav2 collision-safe for any arm or panel pose without named profiles, without the over-conservatism of an enlarged static footprint, and without Nav2 modification.
Variants considered: hull in a standalone node / in `ReconfigurePanel` / in a costmap layer; centroid padding / exact offset; mesh / disc model; no 2D check / hull check in `ReconfigurePanel` ([spec](superpowers/specs/2026-10-04-phase3-dynamic-footprint-design.md)).
Decision: standalone node on a ROS-free geometry library, single footprint owner chosen by `footprint_mode`, exact 0.02 m outward offset, mesh model by default, hull check against the global costmap ([ADR 0010](adr/0010-dynamic-footprint-ownership.md)).
Reason: the node serves every later phase (demo, MPC comparison) without MoveIt; Nav2 stays unmodified; the exact offset keeps the panel's long edges at the full margin.
Benchmark scenario(s): B3 missions (regression); 1.05 m and 1.30 m gate scenarios with the arm in `home` or `vertical_carry` (strategy comparison); qualified-pose benchmark.
Metrics before: B3 RPP 9/9 missions (2026-10-04 at `849f19b`); static strategies as the paper's baselines.
Metrics after: B4 regression 8/9 with RPP, no contact, no hull refusal; strategies over 60 RPP drives: base-only 12/20 (5 collisions), enlarged 20/20 (two cells detour 11 m instead of 4.65 m), dynamic 17/20 (no collision; 3 throat aborts, base-only's rate in that cell); plan-only with Smac Lattice: dynamic decides 4/4 cells correctly with both models, base-only and enlarged 2/4 each; 11 µs per footprint evaluation, 43-61 µs per node tick, at most 6.2 ms (docs/experiments/dynamic-footprint).
Trade-offs / regressions: in `home` the hull republishes 7-8 times per drive from arm sway (open decision 2026-10-04); the padding makes B4 reject `home` at the 1.30 m gate although the panel passes with 0.03 m per side; the disc model brings no benefit with this panel; B3 missions gain a footprint-ownership preflight.
Keep old variant as baseline? yes (B3 with `footprint_mode:=profiles`, B1/B2 with `static`)
Follow-up: the open decision on in-drive updates; B4 on the Phase 4-5 scenarios (overhead course, demo goals); footprint prediction from future arm poses in Phase 5 (the paper's future work).
References: Sagar et al. (Section 10), docs/experiments/dynamic-footprint, ADR 0010.

---

# 12. Immediate next development order

1. Complete and qualify Phase 1 Nav2 navigation with the fixed arm and current footprint.
2. Preserve detailed compute/logging hooks while doing so.
3. Add MoveIt arm baseline and 3D collision checking.
4. Implement dynamic footprint paper baseline including the panel projection (done 2026-10-04, B4).
5. Build the simplest correct local whole-body linearized QP MPC.
6. Qualify MPC computation before adding medium horizon.
7. Add reduced-task-space medium horizon with compact parameterization, beginning with piecewise Bézier and comparing MINCO.
8. Add conditional computation and staged collision refinement after full hierarchy works deterministically.
9. Run Phase 6 solver/concurrency/real-time optimization and ablation.
10. Use the resulting benchmark history as direct material for the thesis computational-evaluation chapter.
