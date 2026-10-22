# ADR 0011: Whole-body MPC command ownership (baseline B5)

Date: 2026-10-13. Status: accepted for Phase 4; revisit at Phase 4B and Phase 5.

## Context

Phase 4 replaces the Nav2 local controller and MoveIt as command authority with a local
whole-body MPC (a linearized time-varying QP, solved with OSQP) that commands the base and the
arm at the same time. ADR 0006 gave `/cmd_vel` one owner, the collision monitor, with the
Nav2 chain feeding it, and flagged this phase as its revisit point. ADR 0004 and ADR 0005
describe the arm path (`JointTrajectoryController` in `controllers.yaml`: 0.15 rad path
tolerance, `allow_nonzero_velocity_at_trajectory_end: false`, no interpolation from the desired
state).
ADR 0010 gave `dynamic_footprint_node` the footprint and zone inputs. B5 has to fit these
without changing B1-B4. Design: `docs/superpowers/specs/2026-10-11-phase4-wholebody-mpc-design.md`;
measurements: `docs/experiments/wholebody-mpc/README.md`.

## Options considered

- Where the MPC lives: (1) a Nav2 controller plugin in `controller_server`; (2) its own node
  serving the `FollowPath` action, which the behaviour tree's `FollowPath` node names; (3) a
  node outside the behaviour tree that follows `/plan` and ignores the navigator.
- Where the base command goes: straight to `/cmd_vel` (bypassing the collision monitor), or to
  the collision monitor's input so that it stays the only `/cmd_vel` publisher.
- How the arm is commanded: (1) stream the predicted joint trajectory to the existing JTC every
  cycle; (2) a forward command controller on position or velocity interfaces; (3) a custom
  `ros2_control` controller.
- How the collision monitor is configured for a robot whose footprint changes in 3D: keep the
  B4 hull for every zone, or a base-only footprint for the stop and a swept hull for the
  slowdown.

## Decision

- **Option 2: `wholebody_mpc_node` serves `FollowPath` in its own process** with its control
  loop on a dedicated timer and callback group. `bt_navigator`, `planner_server`, the behaviour
  tree (`navigate_to_pose_wholebody.xml`, same recoveries and replanning) and the lifecycle
  manager are kept; B5 starts neither `controller_server` nor `velocity_smoother`
  (`navigation.launch.py controller:=wholebody`). Option 1 would put a 50 Hz loop with its own
  collision library and arm publisher inside a process built around a local costmap; option 3
  would lose Nav2's goal handling, recoveries and result reporting.
- **Base command into the collision monitor.** The node publishes `cmd_vel_smoothed` (the
  monitor's input); the collision monitor stays the only `/cmd_vel` publisher and
  `check_cmd_vel_ownership --expect-upstream wholebody_mpc` is the gate (ADR 0006 unchanged in
  structure; the upstream publisher is the MPC instead of the velocity smoother). Output stage:
  a non-zero planned command below the base's breakaway (0.07 rad/s, 0.0375 m/s) is raised to it
  while the base is measured at rest (the RPP rotate-from-rest stall, open decision 2026-10-06);
  the rate limits of the operating envelope apply relative to the last command sent.
- **Arm streaming to JTC, unchanged controller settings** (B3 and B4 keep their recorded
  controller). Every cycle the node sends one `JointTrajectory` with header stamp 0 and:
  point 0 at `time_from_start` of about 0 equal to the current commanded reference (the last
  trajectory sent, sampled now: positions and velocities), the plan's stages 1..N, and a braking
  tail of at least 0.5 s that ends at rest within the arm's acceleration limit (JTC rejects a
  trajectory that ends moving, and with the flag set to `true` it would keep the last velocity if
  the MPC died). Streaming only stages 1..N re-anchors the reference on the measured joints
  every cycle and a gravity-loaded joint then creeps (bring-up: `wrist_2` 0.12 rad off at 20 Hz,
  stalled at 50 Hz); starting at the reference removed it (JTC error at most 0.067 rad, ruling
  R24).
- **Arm state of the MPC.** x0 for the arm is the commanded reference while every joint is
  within 0.15 rad (the JTC path tolerance) of the measured joints, otherwise the measured joints
  (a re-anchor, counted in `MpcCycle` and the timing summary; 0 in the campaign). The arm limits
  are the combined-motion qualification (`wholebody_arm`: 0.60 rad/s, 0.61 rad/s^2) and
  `wholebody_arm.joint_bounds` (`wrist_2` at most 1.77 rad).
- **Stops.** Every stop (cancel, goal, braking, stale state, an exception, node shutdown) sends
  zero base command and a braking hold for the arm: point 0 on the reference (q, v), then rest at
  q + v T / 2 with T = max(0.5 s, max |v_j| / qdd_max_j), a constant deceleration inside the arm's
  acceleration limit. "Hold at the measured joints" is not physically possible for a moving arm.
- **Failures.** A solver failure or a missed deadline rides on the shifted previous plan for
  `fallback_s` = 0.3 s (K = ceil(0.3 s x rate): 6 cycles at 20 Hz, 15 at 50 Hz; host stalls of
  0.3-0.4 s were measured), then the MPC brakes (base at the envelope's braking limit, arm
  hold) and fails `FollowPath` so the tree's recoveries run. No progress (less than 0.3 m of base
  motion and less than 0.5 rad of turning in 10 s) also fails it. A new path preempts the active
  goal in place and keeps the command, failure count, warm plan and tightness.
- **Safety layer, `footprint_mode: wholebody`.** `dynamic_footprint_node` stays the only owner of
  the footprints and zones (ADR 0010). B5 publishes the base-only footprint to the costmap
  footprint topics (only the global costmap exists in B5) and the stop zone (base polygon plus
  margin), and the **swept hull of the MPC's predicted configurations** (each at its predicted
  base pose in the current base frame, plus margin) as the slowdown zone. The costmap obstacle layers, the stop and slowdown zones use lidar points from
  0.05 to 0.60 m (base height); B1-B4 keep their 0.05-2.0 m band. Overhead obstacles therefore
  never stop the robot under a beam; the MPC's 3D constraints against the known geometry are the
  primary protection there. A low obstacle under the panel can slow the robot, never stop it.
- **Known geometry.** The MPC avoids the exported map boxes and the scenario's boxes (the world
  B3's MoveIt uses, now with an optional height offset) through exact FCL distances on the URDF
  primitives and the payload box, linearized through the full kinematic chain; movers come from
  Unity ground truth on `/scenario/movers` with a constant-velocity prediction. Results are
  labelled "known geometry, known movers". A perceived distance field is a separate, unscheduled
  baseline.
- **Fallbacks kept in reserve, not built:** a forward command controller, or a custom
  `ros2_control` controller, for the arm if streaming to JTC had not tracked; closed-form
  capsule-box distances if the FCL queries had not fitted the budget.

## Consequences

- Command authority stays structural: one `ros2 topic info -v` query shows the single publisher
  on `/cmd_vel`, and the collision monitor remains the last layer for B5 as for B1-B4. B1-B4
  launch files, costmaps, controllers and missions are untouched.
- The MPC is a separate process on the same machine as Unity, the endpoint and Nav2. It used
  27-44 % of a core at 50 Hz in the campaign (the most on the overhead course; the 43-47 %
  reported for the R26 rate probe came from the fix-round bring-up runs, not the campaign); the
  CPU headroom measurement
  put the arm path's budget at the whole 10-core VM, with sporadic Unity stalls (0.3-0.65 s) as
  the failure mode, so the node must brake on stale feedback.
- **Slowdown-zone confound.** The collision monitor scales the command to 0.5 in its slowdown
  zone, which is the MPC's own swept hull; the MPC's model assumes the unscaled command. In the
  campaign the zone halved the command for 79-93 % of the moving time in the gate drives (the
  base moved at 0.46-0.47 of the speed the plan assumed), which accounts for part of B5's 8-12 s
  deficit to B4 in the vertical-carry gate drives. B5 times against B4 are not like for like until
  this is resolved (roadmap open decision).
- **Clearance below the margin from tracking error.** The MPC enforces 0.05 m at the commanded
  reference, while the measured arm lags it (JTC error 0.004-0.018 rad at the minimum-clearance
  instants sampled, of the order of the 0.015 rad hold sag). 17 of 60 B5 runs had a 3D clearance of
  0.030 to 0.046 m (no contact; in the unscripted runs the collision slack was at most 0.05 mm),
  0.038-0.043 m at the overhead beam.
  The collision rows need the reference-to-measured offset, or 1.5-2 cm more margin, before
  Phase 5. *(Phase 4B-A, 2026-10-22: the offset does not explain it, 24 of 24 runs; the cause is slack on pairs
  already inside the margin plus the linearization; see the roadmap resolution and the amendment below.)*
- **Stop zone against the MPC.** The 2D stop zone can hold the robot where the MPC's 3D model
  sees clearance (one narrow-gate run, 90 s timeout); the MPC cannot tell it is being held.
- B5 does not yield to a crossing worker (`worker_late_crossing_nav` 0/3: a deadlock with the
  worker inside the lane, no contact); moving-obstacle handling is a Phase 5 trigger.
- **If the MPC process dies**, nothing sends a stop: the base stops through the Unity base
  watchdog (no command for 0.5 s), but the JTC keeps executing the last streamed trajectory, up
  to about 2 s of arm plan (the horizon) plus its braking tail. That residual arm motion is
  unsupervised, a residual risk carried to Phase 4B (an arm-side watchdog or a shorter streamed
  horizon would bound it).
- Rate: 50 Hz was chosen after fix round 1 (ruling R26), but its conditions were not all met in
  the full campaign (p99 and misses in two cells, clearance below the margin). It stays an
  open decision for Phase 4B.

**Note (2026-10-13): follow-up switches.** For the confounds above and the open posture question,
the code now has switches whose defaults keep this decision and the recorded campaign unchanged:
`slowdown_zone:=false` disables the collision monitor's slowdown zone for any controller (the stop
zone and `FootprintApproach` stay, and `dynamic_footprint_node` keeps publishing the zone input, so
ownership is unchanged); `posture_mode` (`blend` default, `comfort_only`, `none`) and `comfort_pose`
(`home` at first; `carry` once qualified, see below) change the posture preference; `clearance_weight` (default 0)
adds a cost for obstacle distance below 0.25 m. The MPC also records its arm choices (`arm_trace`).
Runs with non-default switches are not like for like with the recorded baselines (roadmap comparability
log); first results are in the [results](../experiments/wholebody-mpc/README.md) ("Follow-up switches").

**Note (2026-10-13): `carry` is the default comfort pose.** The bent-arm level carry
`[0, 0.968, -1.735, 0, 0.767, 0]` (panel top 1.32 m, the home footprint) was qualified like the other
B5 arm motion ([results](../experiments/wholebody-mpc/README.md), "carry pose qualification") and is
now `comfort_pose` in `wholebody_mpc.yaml`; `home` stays the B1-B4 pose and the recorded B5's comfort
pose (`comfort_pose:=home slack_gate:=false` reproduced the recorded B5 on the 2026-10-13 code, see the slack gate note; since Phase 4B-A the tag `baselines-2026-10-13` does). Command ownership is unchanged. B5 runs with `carry` are not
like for like with the recorded B5 (roadmap comparability log).

**Note (2026-10-13): slack gate.** The collision rows stay slacked (the QP must always have a
solution), but a plan whose slack within its first 1.0 s exceeds its allowance is no longer executed:
the allowance is 5 mm for a pair outside the margin and the pair's current violation (max(0, margin -
its distance at the measured state)) for a pair inside it, so a robot inside the margin may stay or
plan its way out but never plan deeper (amended after review: 5 mm on top of the violation let it
creep in 5 mm per cycle). Such a plan is not executed:
the cycle is `holding`, the base brakes at the envelope's limits through the collision monitor as any
command, the arm gets the stop's braking hold, and the MPC plans again every cycle; it resumes when
the plans pass over the whole 2 s plan for 0.2 s. The held plan is never executed: a solver failure
while holding keeps holding (it does not fall back to the held plan) and restarts the 0.2 s. A held
solve never fails the goal by itself; the no-progress check does after 10 s, and the tree's
recoveries take over. Solver failures and missed deadlines while holding still count as failures:
more than K in a row brake and fail `FollowPath` with error 106 (no valid control), as the recorded
`comfort_only` gated runs ended each `FollowPath`. Command ownership is
unchanged (`slack_gate:=false` executes every plan; with `comfort_pose:=home` it reproduces the recorded
B5, runner `--comfort-pose home --no-slack-gate`, plus `--mpc-rate-hz 50` for the campaign rate; inside the margin a plan may be at most 0.1 mm (solver noise) deeper than the measured violation; spec Section 3, "Slack gate").
Residual risks: the gate judges the linearized plan, not the motion, so (a) the hold itself moves
the robot (base braking distance, the arm's braking hold coasts v T / 2 with T >= 0.5 s; in the unit
closed loop this took the panel up to 1 cm closer than the plan before the resume hysteresis);
(b) the breakaway output stage lifts a resuming command above the plan; (c) slack beyond the first
second is still allowed and the plan is executed until the penetration reaches the first second; (d)
tracking error between the commanded reference and the measured arm (above) is not covered; (e) a
mover predicted at constant velocity can make the robot hold where waiting does not help (a worker
that waits for the robot: the `worker_late_crossing_nav` deadlock is not resolved by holding in the
unit model when the robot stops within 0.1 m of the worker's path); (f) the gate cannot tell slack
caused by a mover from slack caused by the robot: an oncoming mover keeps the robot braked even where
moving away would help; (g) resuming checks 2 s of plan against the 1 s entry check, so a hold may
last until the no-progress check ends the goal although a 1 s plan would pass; (h) the slack-to-row
mapping assumes `CollisionModel::rows` emits rows in stage order (the gate checks it and throws
otherwise; a collision test pins it). The gate is a rule on top of the MPC, not a safety layer: the
collision monitor stays the last layer.

## Amendment: Phase 4B-A safety envelope (2026-10-22)

Phase 4B-A ([design](../superpowers/specs/2026-10-18-phase4b-a-b5-behaviour-fixes-design.md),
[results](../experiments/wholebody-mpc/phase4b-a.md)) reshaped B5's safety layer. This is in
`footprint_mode` `wholebody` only; B1-B4 are unchanged.

**Why.** At the obliquely entered 1.05 m doors the 0.05 m StopZone stopped the base against noisy lidar
while the MPC, planning 0.05 m from the true boxes, kept commanding into it. The SlowdownZone halved
B5's command (the B4/B5 confound).

What changed:

- **StopZone:** the base hull + `padding_m` 0.02 + `StopZone.wholebody_margin_m` 0.02 m (was 0.05).
- **SlowdownZone:** off by default (`slowdown_zone:=""` is false for `controller:=wholebody`). The swept-hull input
  is still published and the zone stays declared. `slowdown_zone:=true` is the ablation.
- **FootprintApproach:** kept.
- **Base margin (coupled to the StopZone).** The MPC's base rows keep `collision.base_margin_m` 0.12 m: padding
  0.02 + StopZone 0.02 + the measured lidar intrusion q = 0.08 m (the fourth-nearest return's reach in from a
  surface, p99.9). The base therefore stays outside what the StopZone sees. Arm and panel rows keep 0.05 m.
  Changing the StopZone or the lidar noise means re-measuring q and the base margin.
- **Executed-command feedback.** The MPC's base previous control is the executed `/cmd_vel` after the monitor
  (stale after 0.5 s; off in the slowdown ablation).
- **Blocked detection.** When B5 commands at least the breakaway while the base stays at rest and the executed
  command stays below the breakaway, it holds (status `blocked`) after 1.0 s. It fails FollowPath after 3 s of
  blocked time without 0.05 m of motion (error 105), so the behaviour tree's recoveries and replanning run
  instead of a silent standstill.
- **Arm control as a state input.** Once received, `/arm_controller/controller_state` older than 0.5 s is
  stale state: brake and error 106. In the exit campaign's first run, a rejected arm target had deactivated arm
  control while B5 kept driving the base.

Consequences:

- B5 times are no longer halved by the SlowdownZone. The recorded B4/B5 comparison stays provisional (roadmap
  comparability log).
- The crooked-door dead end remains. The forward-only base cannot back out of a 1.05 m throat entered 27-40°
  off axis. This is deferred to Phase 5.

## Validation

- Unit tests of the generated model, FCL distances, QP assembly, breakaway stage, rate limits,
  fallback and braking; the node in a launch test against a Python plant (it integrates the base
  command and moves the arm along the streamed trajectories); the footprint node's grow-only and
  `wholebody` hull tests. There is no automated test against the mock-hardware JTC: the
  streaming contract (point 0 at time 0 on the commanded reference, the braking tail,
  `interpolate_from_desired_state: false`) was verified live in Unity in the bring-up (Task 13).
  An automated JTC streaming test is a gap recorded for Phase 4B.
- `check_cmd_vel_ownership --expect-upstream wholebody_mpc`, `footprint_wait` and the costmap
  preflight pass in every B5 run.
- Campaign (143 runs, 45 cells, 2026-10-12/13, `5458e0f`): no robot-environment contact; B5 23/24
  on the Phase 1 driving scenarios, 9/9 unscripted and 9/9 scripted missions, 3/3 on both overhead
  courses, `worker_stops_in_lane_nav` 3/3, `worker_late_crossing_nav` 0/3; p99 cycle time 10.6-14.9
  ms at 50 Hz in the cell medians; free drive 4/4 goals including one behind the robot
  ([results](../experiments/wholebody-mpc/README.md)).

## Revisit when

- Phase 4B changes the rate, the horizon or the solver, or measures an MPC above 50 Hz (the
  controller manager and Unity physics run at 50 Hz, so a faster loop only shortens
  state-to-command latency).
- The slowdown-zone confound (resolved in Phase 4B-A by switching B5's zone off, amendment above) returns
  if a B5 variant needs the zone again: then model its scaling in the MPC; B4/B5 times are rerun in
  the final baseline run.
- The StopZone, the lidar noise or the sensor changes: re-measure the lidar intrusion and the base margin
  (amendment above).
- Arm tracking, not the plan, limits clearance and streaming to JTC cannot keep it within the
  margin: build the forward command or custom `ros2_control` controller.
- A perceived obstacle representation (distance field, 3D safety check) replaces known geometry,
  or the collision monitor is replaced by a 3D safety layer.
- Phase 5 adds the medium-horizon layer above the MPC, which changes what feeds the MPC's
  reference and the swept hull.
- Operator teleop must pre-empt autonomy (ADR 0006: `twist_mux`).
