# ADR 0015: a medium-horizon layer plans the whole body above B5, and B5 follows it by arc length

Date: 2026-11-02. Status: accepted (Phase 5A); F2 frozen at the tag `f2-frozen-2026-11-02`. Builds on
[ADR 0011](0011-wholebody-mpc-command-ownership.md) (B5 owns the commands) and [ADR 0013](0013-mpc-solver-iterations-and-rti.md).
Design: [Phase 5A spec](../superpowers/specs/2026-10-29-phase5a-medium-horizon-design.md); results:
[phase5a.md](../experiments/medium-horizon/phase5a.md).

## Decision

A medium-horizon layer, `medium_horizon_node` (`mobile_manipulator_medium`, C++, its own process), sits between Nav2
and B5. Nav2 → medium layer → B5 is the hierarchy of the roadmap.

- **Ownership.** Nav2 still sends FollowPath goals to B5. B5 alone publishes `/cmd_vel` and the arm stream and keeps
  its collision rows, margins, slack gate and blocked detection: the final say on safety. The medium layer publishes
  only a reference, `/medium/plan` (`MediumPlan`), and its timing, `/medium/cycle` (`MediumCycle`).
- **Route announcement.** On every FollowPath goal or path update B5 publishes its active route with an increasing
  `route_id` on `/wholebody_mpc/route` (transient local). The medium layer plans on that route, not on `/plan`, and
  every plan carries its `route_id`, so a behaviour-tree replan cannot desynchronize the layers.
- **Arc-length references.** A plan is sampled every 0.1 m of arc length (or 0.1 rad of yaw) with x, y, yaw, speed
  and a six-joint posture. In `reference_source: medium` B5 rebuilds its route from the samples, splices the rest of
  the Nav2 route beyond the last one, and takes heading, speed and posture targets by its stages' projections. Its
  posture blend, tightness filter and side picker are off.
- **Fallback.** No plan, a plan older than `medium_stale_s`, a different `route_id` or a plan more than 0.3 m from the
  measured base: B5 runs as the frozen B5 for that cycle and reports the reason in `/wholebody_mpc/cycle`. A
  same-goal replan (a new `route_id`) does not wait for a solve: the medium layer republishes its plan under the new
  id at once, spliced on the new route (final review). `reference_source: nav2` (the default) is the frozen B5,
  pinned by the golden test.
- **Formulation 1b (F2).** Base x, y, yaw and all six arm joints as cubic piecewise Bézier curves, 6 segments over
  8 s, C2 at the joins, start fixed to the measured state. Costs: progress, a weak contour, a clearance hinge to
  0.25 m, a weak comfort pull (stronger where the comfort pose is clear, released before a conflict), smoothness and
  a skid-steer residual. Constraints: signed distance at least B5's margins (base 0.12 m, arm and panel 0.05 m) every
  0.16 s with L1 slack, joint limits and velocity and acceleration bounds on the control points (whole curve by the
  convex hull), static torque within 60 % of the rated efforts, a forward-only base. Qualified poses are only
  multi-start seeds and the comfort target: the arm moves continuously, and the one discrete choice is the cheapest of
  several continuous solutions (homotopy classes: swing side, lowered).
- **Solver.** Sequential convex programming, one dense QP per iteration solved by **DAQP** (1e-6), at most 15
  iterations per seed, trust region on the control points and an exact merit.
- **Selection.** The incumbent and one pose seed per cycle in turn (every seed on a new route); a seed the cap cuts
  short resumes from its last iterate. A plan that waits widens the next cycle to every seed; a feasible plan that
  reaches 0.25 m further wins over a slowing one; homotopy hysteresis 15 %, a swing-side switch 50 %.
- **Rate profiles.** `medium.yaml` plans at 2.5 Hz with a 0.28 s cap (the fanless MacBook Air), `medium_5hz.yaml` at
  5 Hz with 0.14 s (a faster host); the launch derives `medium_stale_s` (period + cap + 0.16 s). Timing criteria are
  75 % of the period, and runs of different profiles or hosts are never pooled.

## Context and constraints

- Phase 5A had to fix the four B5 limitations handed to Phase 5: the crooked-door dead end, the swing-side heuristic,
  the 72° side pivot under low beams (71-79 Nm at the shoulder) and close-section sequencing.
- B5's 2 s horizon sees a door or beam too late to align, slow or reconfigure; it must keep 50 Hz with its p99 well
  inside 20 ms on the same host.
- The user rejected blending among a finite list of qualified poses: it would make the arm a scheduled mode switch
  (accepted decision 1, no discrete mode planner).
- The fanless host gives live medium cycles about half the compute the bench measured once it is hot.

## Options considered

- **Frenet coordinates (s, d, ψ).** Rejected: singular at Nav2's 90° corners right before the crooked doors.
- **References as a lateral offset on the Nav2 route.** Rejected: projections around a 90° corner are ambiguous
  there; B5 follows the medium path itself, by arc length.
- **The medium layer commands the robot.** Rejected: it would split command ownership (ADR 0011) and lose B5's 50 Hz
  safety rows.
- **Pose blending (formulation 1a).** Rejected (above).
- **Four arm joints.** Kept as `medium_4joint.yaml`, the comparison option: at 15 SCP iterations six joints converge
  more often.
- **HPIPM dense or OSQP.** Benchmarked on the same QPs: DAQP 0.50 ms median within 7e-8 of its 1e-8 objective, HPIPM
  3.0 ms, OSQP 257 ms and 7 of 177 unsolved; cold multi-start p99 1.18 s (DAQP) against 1.36 s (HPIPM).
- **5 Hz on the Mac with cheaper iterations.** Failed the closed-loop gate at the hot host's iteration budget; 2.5 Hz
  passed it.

## Consequences and risks

- **Campaign A** (7 cells x 3, spec 7.1): F2 20/21 against B5 alone 12/21. Doors room_r4 3/3 (B5 1/3), the low beams
  6/6 (B5 0/6) at a peak static shoulder torque within the limit, door_then_beam 3/3.
- **Campaign B** (campaign 2's 34-run exit set): 33/34 against campaign 2's 30/34, no contact against one. The one
  failure, a wedge aligned in store_room's door S throat, is accepted with campaign A's store_room failure (5C and the
  execution work). It followed fallbacks after Nav2's replans; with the plan carried on to a replan's route (final
  review), a recheck ran store_room 3/3.
- **Clearance is the price.** F2 threads the panel at B5's 0.05 m margin wherever a passage needs the arm, so the
  closest panel clearance at the tight passages fell from campaign 2's 0.10-0.15 m (B5 alone) to about 0.045 m, and
  the execution shortfall (Phase 4B-C: the state below the margin before each plan, the arm's lag, the dips between
  collision samples) takes 7 of 34 runs below 0.045 m (lowest 0.0407 m; campaign 2's shared runs: 1, 0.0427 m). F2
  buys success at the passages with the clearance margin until the execution work closes the shortfall (user ruling
  2026-11-02).
- A forward-only base wedged inside the margin has no plan out: the medium layer's give-way and widening act on
  feasible plans only.
- The fallback costs 3-6 cycles at a new goal, and about one cycle per same-goal replan (up to 8 when the replan
  arrives during a solve: the node's executor is single-threaded).
- The swing-side count is over the whole run, so separate passages or the start pose's unwind can count as a switch.
- Two processes share the host: B5's p99 stayed at 2.9-8.0 ms at 50 Hz, the medium cycle p99 at most 0.285 s of the
  0.30 s limit at 2.5 Hz. On the Mac the 2.5 Hz profile is the measured operating point, not the design rate.

## Validation

Unit and gtests of the basis, problem, SCP, planner (selection, resume, waiting, give-way, hysteresis, side labels),
B5's medium mode and splice, and launch tests of the node (a route before B5 tracks). The open-loop bench (278 cold
snapshots, 2442 warm cycles) chose the solver and representation; the closed-loop kinematic bench (B5's real
controller at 50 Hz on ideal actuators) gates live runs, also under SCP iteration budgets for the hot host. Live:
campaigns A and B on Unity.

## Revisit

- **5B:** F1 dense and F3 MINCO (F4 B-spline optional) on the same decision variables and interface; MINCO's cheaper
  iterations may bring 5 Hz back on the Mac.
- **5C:** movers in the medium layer (stop and reverse prediction, the late-crossing deadlock, conditional triggers),
  through the declared tracked-object input; the stopping worker came closer under F2 (0.025 m against 0.06 m).
- **The execution work** (after the MuJoCo campaign plant, roadmap "Priorities after Phase 5A"): delay-aware MPC,
  base tracking, continuous-time collision; then the clearance at the tight passages and the door S wedge again.
- On a faster host: `medium_5hz.yaml`, scored separately.
- If B5's margins or collision model change, the medium layer's must change with them (one library, one geometry).
