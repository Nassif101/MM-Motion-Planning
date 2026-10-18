# Phase 4B-A design: B5 behaviour fixes before the performance sweep

**Date:** 2026-10-18
**Branch:** `phase4b-fixes` (from `main` at `fd1ff0e`)
**Roadmap:** Phase 4B (docs/roadmap.md), open decisions of 2026-10-13 and 2026-10-17

## 1. Purpose and scope

Phase 4B qualifies B5 before Phase 5 builds on it. It is split in two specs, in order:

- **4B-A (this spec):** fix the B5 behaviours found in the Phase 4 campaign, the follow-up runs and
  the V2 usability check, rerun B5 on the cells that show them, and freeze the formulation.
- **4B-B (later spec):** the performance sweep on the frozen formulation (horizon, rate 20/50/100 Hz,
  collision-model fidelity, warm start, OSQP against HPIPM / partial condensing, prediction grid,
  failure handling) and the decisions it feeds: the MPC rate, B6 go/no-go, F4 before Phase 5.

The sweep only means something on a formulation that has stopped changing, hence the order.

### Decisions taken in the design discussion

| Item | Finding | Decision |
|---|---|---|
| (1) Door deadlock | At V2 interior 1.05 m doors entered 22-29 degrees off-axis (R4, store S) the collision monitor's StopZone (base hull + 0.05 m, against noisy lidar) stops the base while the MPC plans 0.05 m from the boxes and keeps commanding; nothing recovers. | Reshape the safety layer in `footprint_mode wholebody` only (tight StopZone, SlowdownZone off, FootprintApproach kept); a base margin in the MPC that sits outside the StopZone, measured against the lidar; blocked detection that fails FollowPath (Section 2). |
| (2) Left-only swing | The posture blend's tight attractor is the fixed `vertical_carry` (shoulder pan +90 degrees). | Mirrored pair: add a qualified `vertical_carry_right`, choose the side by probed clearance at the first conflict, only when it beats the comfort pose (so a low beam keeps `carry` and the panel is lowered), latch the choice while tight (Section 3). |
| (3) Late-crossing deadlock | `worker_late_crossing_nav` B5 0/3 (V1), `v2_worker_late_crossing_nav` fails: B5 holds at its 0.05 m margin inside the worker's 0.1 m wait distance. | **Deferred to Phase 5** (prediction-aware movers are on its trigger list); stays a documented B5 limitation. |
| (4) Slowdown confound | The swept-hull SlowdownZone halves B5's command for most of the gate drives while the MPC assumes the unscaled command. | SlowdownZone off by default in `wholebody` mode; the MPC takes the executed command as its stage-0 previous base command (Section 4). |
| (5) Clearance below the margin | 17 of 60 B5 runs at 0.030-0.046 m (margin 0.05 m) without contact; candidate cause the commanded-to-measured arm lag. | Offline evidence first; lag compensation only if the evidence supports it, otherwise a checkpoint (Section 5). |
| (6) Small items | A cancel steps the base command to zero in one cycle; the stage-0 previous command does not match what was executed. | Ramp down at the envelope's deceleration; stage-0 previous commands from the executed base command and the streamed arm spline (Section 4). |

### Out of scope

- The performance sweep and the MPC rate decision (4B-B).
- The late-crossing deadlock and any mover-specific margin or yield rule (Phase 5).
- Medium-horizon foresight; panel references from a medium layer (Phase 5).
- A pose below `carry` and the V2 H2 member (open item from the site V2 design).
- Changes to B1-B4 or their configurations (frozen at `baselines-2026-10-13`).

### Why `comfort_only` and `none` cannot be the fix for (2)

Recorded in the discussion because it shapes Sections 3 and 9. The MPC solves one QP per cycle
linearized at the current configuration, so it sees first-order effects only. At a gate narrower
than the panel, a small shoulder-pan rotation moves the panel towards one post and away from the
other (the two post rows pull in opposite directions), and the width gain of a rotation is
second order (about W cos θ, zero slope at θ = 0). Pitching the panel up is the one direction that
improves both post rows at first order (it pulls the front edge back), which is what `comfort_only`
did in the follow-up runs, and it does not narrow the panel. The side of a swing is a discrete
choice that a local optimizer cannot make; it has to be made outside the QP. Phase 5's medium layer
fixes the timing (earlier decisions, iterated solutions) but is still local, so it needs the same
side choice as candidate initializations (Section 9).

## 2. Safety layer and blocked detection (`wholebody` mode only)

### Monitor zones

Set by `wholebody_monitor_parameters` in `navigation.launch.py`; the B1-B4 zone parameters are
unchanged.

- **StopZone:** margin 0.02 m around the base hull (was 0.05 m). It stays the last resort; `min_points` 4
  unchanged.
- **SlowdownZone:** `enabled: false` by default (still declared, so the footprint ownership and zone
  checks see the same graph). `slowdown_zone:=true` re-enables it for the ablation.
- **FootprintApproach:** unchanged (the base-only footprint from `/global_costmap/published_footprint`,
  `time_before_collision` 1.2 s). It scales a command only when that command's projected motion
  would reach a point, so turning or moving away from a wall still passes.

### MPC base margin

Base-to-box collision rows get their own margin, `collision.base_margin_m`; arm and panel rows keep
`collision.margin_m` 0.05 m. Floor and self rows are unchanged.

`base_margin_m = 0.02 + q`, where q is the measured lidar intrusion: how far into a wall the points
reach that the StopZone counts. Measurement (no B5 change; a small tool under `tools/`): the robot is
parked in Unity facing and beside a wall and beside a door jamb at known distances (0.05-0.20 m from
the base hull, 5 positions per obstacle, 60 s each); from the filtered Livox cloud in the base frame
take, per scan, the 4th-deepest point inside the true obstacle surface (StopZone `min_points` 4) and
set q to its 99.9th percentile over all scans, rounded up to the centimetre. With q about 0.05 m the
base margin is about 0.07 m, which leaves about 7 cm per side for the 0.77 m base hull in a 1.05 m
door. If q comes out above 0.08 m (base margin above 0.10 m, under 4 cm per side), stop and report
before continuing: the door would become marginal for the base.

The tightness probe keeps using `margin_m` (it measures the arm and panel's comfort).

### Blocked detection

`wholebody_mpc_node` subscribes to the executed `/cmd_vel` (the collision monitor's output).

- **Executed command:** the latest `/cmd_vel` if it is at most two MPC periods old, otherwise zero (the
  monitor stops publishing `stop_pub_timeout` 2 s after a stop, so silence means stopped).
- **Blocked:** for 1.0 s in a row the MPC's own base command has |v| >= 0.05 m/s or |ω| >= 0.1 rad/s
  while the executed command has |v| < 0.01 m/s and |ω| < 0.02 rad/s. A command scaled down by
  FootprintApproach but not zero is not blocked.
- **While blocked:** the MPC holds as the slack gate's hold does (base braking at the envelope's limits,
  the arm's braking hold), plans every cycle, and reports status `blocked` in `MpcCycle`. It resumes
  when the executed command follows its command again.
- **Fail:** blocked for 3.0 s in a row ends FollowPath as aborted with `FAILED_TO_MAKE_PROGRESS` (105)
  and the reason `blocked by the collision monitor` in the log; `MpcCycle` carries the status. The
  existing B5 tree then clears the global costmap, retries, and its recoveries wait and replan from the
  current pose. (Nav2's FollowPath result has no free text field in Jazzy, so the cause is identified by
  the code, the cycle status and the log rather than a new code.)

Parameters (`wholebody_mpc.yaml`, `blocked:` block): `detect_s` 1.0, `fail_s` 3.0, the command
thresholds above. The detector is a pure class (input: own command, executed command and its age,
time; output: state), unit-tested on its own.

### ADR notes

ADR 0011 amendment: the B5 safety envelope (StopZone 0.02 m, no SlowdownZone by default,
FootprintApproach kept), the base margin and why it is coupled to the StopZone, blocked detection.
ADR 0010 note: in `wholebody` mode `dynamic_footprint_node` remains the footprint owner (base-only
footprint for both costmaps and the StopZone); its swept-hull SlowdownZone input is still computed but
feeds a disabled zone by default (RViz view, the ablation, a possible Phase 5 route check).

## 3. Swing side: mirrored tight poses

### The pose

`vertical_carry_right` mirrors `vertical_carry` (`[π/2, 0, 0, 0, π/2, 0]`) across the base's vertical
x-z plane. Its joint vector is found by forward kinematics, not by flipping signs (the UR shoulder has a
lateral offset): the joint vector, within the joint limits and `wrist_2` within 1.77 rad, whose panel pose
is the mirror of `vertical_carry`'s, with the residual reported. **Checkpoint:** a Unity screenshot of the
candidate beside `vertical_carry` is shown to the user before qualification. It is then qualified as
`carry` was (docs/experiments/wholebody-mpc, "carry pose qualification": held pose and transitions with
the base driving, same acceptance), and added to `qualified_payload.json` `poses_rad`. The node refuses
to start if a configured tight pose is missing there.

### Configuration

`posture.tight_poses: [vertical_carry, vertical_carry_right]` replaces the single tight pose. One entry
reproduces the left-only B5 (`[vertical_carry]`); the node parameter / launch argument `tight_poses`
overrides.

### Side picker

A pure class beside `TightnessFilter` in `formulation`:

- Below the release level α_off = 0.02 no side is chosen and the tight poses are not probed.
- When the filtered tightness α rises above α_on = 0.05 with no side chosen, find the **first conflict
  stage** k*: the first stage of the nominal horizon where the comfort pose's clearance is below the
  tightness comfort distance (0.30 m). Probe the comfort pose and each tight pose with
  `CollisionModel::clearance_with_joints` (the query the tightness already uses for the comfort pose),
  per stage, and score each pose by its minimum clearance over stages k*-1..k*+1. Conflicts after that
  window are left to the collision rows (and to Phase 5): comparing whole-horizon minima would let an
  obstacle further ahead (a beam 1.5 m past a gate) veto the pose the robot needs now.
- **Clearance guard:** a tight pose is only eligible if its score exceeds the comfort pose's score by more
  than 0.02 m. If none is eligible (a low beam, where both vertical poses are taller than `carry`), the
  posture target stays at the comfort pose and the collision rows lower the panel from there.
- Among eligible poses choose the larger score if it is larger by more than 0.02 m. On a tie: the side
  the arm already pans towards (|shoulder pan| > 0.1 rad), else the side the route turns towards within
  the horizon (sign of the route heading change over the horizon's progress), else left.
- **Latch:** the choice (left, right or comfort) holds until α falls below α_off. No switch while tight; a
  choice that turns bad later is handled by the collision rows and the slack gate, as now.
- The posture target becomes `(1 - α) comfort + α tight[side]`, or the comfort pose when the guard
  chose it, as in [formulation.cpp](../../../ros2_ws/src/mobile_manipulator_mpc/src/formulation.cpp)'s
  blend today. `comfort_only` and `none` are unchanged.

Cost: three per-stage clearance probes (comfort and two tight poses) in the cycle that chooses, none
while latched or below α_on. The prefilter keeps a probe near 0.1 ms on V2.

Physical limits, recorded so they are not mistaken for picker faults: a vertical panel (1.2 m tall, bottom
at least 0.15 m above the floor and beside the base) cannot get its top below about 1.4 m, and a square
panel is not narrowed by turning it flat, so a space both narrower than the panel and lower than that
cannot be passed in any configuration (B5 holds and fails FollowPath). A narrow section followed closely
by a low one needs the vertical-to-carry change (about 4.5 s at the arm limits, about 1.3 m at 0.3 m/s)
to start before the low obstacle enters the 2 s horizon; when it does not, B5 holds and reconfigures or
stalls. Sequencing such sections is Phase 5's (Section 10).

### Logging

`MpcCycle` gains the choice (none / comfort / left / right) and the three probe scores of the choosing
cycle; `tools/arm_trace.py` reports the side per tight section.

## 4. Executed-command feedback and small items

- **Stage-0 base previous command:** the executed command from Section 2 replaces the command the
  controller last sent. The base's acceleration (rate) and smoothness rows of stage 0 then start from
  what the base was actually told, including any FootprintApproach scaling.
- **Stage-0 arm previous command:** the velocity of the streamed spline at the current time (the
  reference JTC is following), replacing the clamped command (roadmap Phase 4B item b).
- **Cancel or planner failure:** the base ramps down at the envelope's deceleration limits using the
  hold's braking profile instead of one zero command; the arm gets the hold's braking stop (roadmap
  Phase 4B item a).
- **SlowdownZone off by default** in `wholebody` mode (Section 2).

## 5. Clearance below the margin

### Step 1: offline evidence (no B5 change)

Extend the bag clearance tool (`clearance_bag.cpp`) to compute, per sample and at the same base pose
(from `/tf`), the minimum 3D clearance of the arm and panel twice: with the measured joints
(`/joint_states`) and with the commanded reference (`/arm_controller/controller_state` reference).
Run it on the 17 recorded B5 runs below the margin (overhead beam, constrained mission unscripted and
scripted, blocked gate; `experiment_runs/2026101[23]T*-wholebody-*`). Report per run, at the closest
approach: both clearances, their difference, the per-joint lag.

**Decision rule:** the lag explains the shortfall if, wherever the measured clearance is below 0.05 m,
the reference clearance is at least 0.049 m and the difference covers at least 80 % of the shortfall
(0.05 m minus the measured clearance).

### Step 2a (the lag explains it): lag compensation

For each arm or panel collision pair active in the cycle, compute its distance at the measured joints and
at the reference (one extra query per active pair) and tighten that pair's rows in every stage by
`clamp(d_ref - d_meas, 0, 0.02 m)`. Base rows are not compensated (the base pose is measured). Unit
test: the closed loop with an artificially lagging arm plant past the beam keeps at least 0.049 m with
the compensation and dips below it without.

### Step 2b (it does not): checkpoint

Stop and report to the user before choosing a fix. Next candidates: the URDF/FCL primitives against
Unity's colliders, the MPC's base pose source against the ground truth, penetration between the 0.1 s
prediction stages.

## 6. Scenario: right swing

New `v2_gate_right_swing_nav` in `config/scenarios_site_v2.yaml`, in the open delivery yard: a 1.05 m gate
of scenario boxes on a straight route, with a tall box along the left side of the approach placed so that
the left-swung panel would have less clearance than the right-swung one (positions fixed in the plan from
the two poses' forward kinematics with at least 0.10 m difference in probed clearance, and a plan-only
check like the other V2 scenarios). Starts in `carry`. Expectation: B5 swings right. The symmetric V1
`narrow_gate_home_nav` is the left-swing control.

## 7. Order of work and checkpoints

1. Evidence, no B5 change: the lidar intrusion measurement (q; stop if q > 0.08 m) and the offline lag
   check (Section 5 step 1).
2. Section 4: executed-command feedback, arm stage-0 command, cancel ramp, SlowdownZone off.
3. Section 2: base margin, StopZone 0.02 m, blocked detection and the FollowPath failure.
4. Section 3: `vertical_carry_right` by forward kinematics, **checkpoint: Unity screenshot for the
   user**, qualification, side picker, `v2_gate_right_swing_nav`.
5. Section 5 step 2a, or the step 2b **checkpoint**.
6. Exit campaign (Section 8).
7. Documentation and freeze (Section 10).
8. Low-pass addendum (Section 10A), on the frozen formulation.

One commit per fix. Unity Play mode is stopped after every task that used it; long campaigns run under
`caffeinate`.

## 8. Evaluation and exit criteria

### Exit campaign

B5 with the new defaults at 50 Hz, real Unity, clean tree, unscripted unless listed.

| Set | Cells | Runs |
|---|---|---|
| V1 | `overhead_course_nav` (start `home`), `overhead_course_worker_nav`, `constrained_reconfiguration_mission` unscripted, the same scripted (`--references scripted`), `narrow_gate_home_nav` | 5 x 3 = 15 |
| V2 | the 20 V2 scenarios | 20 x 1 = 20 |
| V2 | `v2_gate_right_swing_nav` | 3 |

V1 is limited to the cells that test what V2 cannot: lowering the panel in transit under a beam (V2 has
no forced-lowering member: every V2 scenario starts in `carry`, the lowest qualified pose, and H2 was
dropped), the tall-box constrained mission, the symmetric gate, and the cells where the recorded runs
went below the margin. They are compared cell by cell with the recorded B5 campaign (2026-10-12/13).

### Exit criteria

- No robot-environment contact in any run.
- V2: 19 of 20, the only failure `v2_worker_late_crossing_nav` (deferred); `v2_room_r4_nav` and
  `v2_store_room_nav` succeed.
- V1: on each of the five cells, success at least the recorded B5's.
- If Section 5 step 2a ran: no run with a 3D clearance below 0.049 m.
- `v2_gate_right_swing_nav` swings right 3/3; `narrow_gate_home_nav` still swings left 3/3.
- No `blocked` status in a successful run; every blocked FollowPath failure is listed with its cause.
- Timing (cycle p99, misses, CPU) is reported, not gated (4B-B).

If a criterion fails, the cause is found and reported before the freeze; criteria are not relaxed
without the user.

## 9. Testing

- Unit (`mobile_manipulator_mpc` gtest): blocked detector (executed stream fresh, scaled, zeroed, silent;
  detect and fail times; resume); base margin applied to base rows only; side picker (larger clearance
  wins, tie-break order, latch through changing clearances, release at α_off, no probe below α_on);
  stage-0 previous-command source (fresh, stale, silent, scaled) and the rate rows built from it; cancel
  ramp within the deceleration limits; lag compensation (if 2a).
- Unit closed loop (`test_controller`): the 1.05 m gate with a tall box left of the mouth swings right and
  passes; the symmetric gate swings left as now; a beam just below `carry`'s top with nothing beside it
  keeps the comfort target (guard) and passes lowered; a gate followed 1.5 m later by a beam still picks
  vertical for the gate (first-conflict window); the existing controller tests unchanged.
- Launch tests: `wholebody` monitor parameters (StopZone margin 0.02, SlowdownZone disabled, `slowdown_zone:=true`
  re-enables); B1-B4 parameters unchanged.
- Live during development: `v2_room_r4_nav`, `v2_store_room_nav`, `v2_door_105_nav`, `narrow_gate_home_nav`;
  then the exit campaign.

## 10. Documentation, roadmap and freeze

- Tag `b5-frozen-<date>` on the commit the exit campaign ran; 4B-B sweeps exactly this formulation.
- Roadmap: a Phase 4B-A outcome section; resolve the open decisions "The slowdown zone and the B4/B5
  comparison" and "3D clearance below the 0.05 m margin"; "B5 does not yield to a crossing worker"
  moved to Phase 5 as a documented limitation; comparability-log rows for the B5 changes.
- Roadmap Phase 5 additions: (a) the medium layer must be initialized from candidate configurations, at
  least the mirrored tight poses, because a local optimizer cannot choose the swing side; (b) open:
  whether the comfort-pose preference stays as a weak posture term once panel references come from the
  medium layer (the tight attractor goes away); (c) open: a configuration-dependent or swept footprint to
  check medium-layer routes in 2D (B5's route uses the base-only footprint, so Nav2 can keep routing B5
  into a space no configuration passes); (d) sequencing of closely spaced narrow and low sections (start
  the reconfiguration and slow the base before the 2 s horizon sees the second section).
- ADR 0011 amendment and ADR 0010 note (Section 2).
- `docs/running-the-stack.md`: `tight_poses`, `blocked` parameters, the new `slowdown_zone` default for
  `wholebody`, how to reproduce the recorded B5 (`--comfort-pose home --no-slack-gate --mpc-rate-hz 50`
  plus the old safety envelope and single tight pose).
- `docs/experiments/wholebody-mpc`: the lidar intrusion measurement, the lag check, the pose
  qualification, the exit campaign.

## 10A. Addendum: low passes (after the freeze)

The user wants to see B5 pass under low obstacles by lowering the panel itself, folded towards the fold
limit or with the arm extended forward and down. This is a capability demonstration, run **after** the
freeze and the exit campaign on the frozen formulation; it is not an exit criterion and changes no B5
code. No new qualified pose is involved: the MPC lowers the panel through its collision rows (a first-order
clearance gain, which the single linearized QP finds), within the joint bounds.

Facts from `carry_pose_search --top-min 0.70 --top-max 1.30` (2026-10-18): with the panel level and inside
`home`'s footprint, the lowest pose has its panel top at 1.20 m (`[0, 1.236, -2.156, 0, 0.920, 0]`; joint
margin 0.20 rad and wrist_1-wrist_3 clearance 0.052 m at their search limits); lower needs the panel moved
forward (`level_extension`: top 0.865 m, footprint 1.70 m long, shoulder gravity torque 95.7 Nm against
22 Nm, qualified only to 0.15 m/s and 0.2 rad/s).

1. **Offline:** the unit closed loop (generated model plant, beam boxes across a straight route, start in
   `carry`) finds the lowest beam bottom B5 passes, in 1 cm steps from 1.30 m down, and reports per beam
   the arm configuration under the beam, the minimum clearance, the shoulder and elbow gravity torque
   against the qualified envelope, and whether the shoulder, upper arm and base also fit. Two regimes are
   expected: folded (down to about 1.25 m) and extended (lower, if the joint bounds and the floor rule
   allow it).
2. **Live:** two V2 scenarios in an open area, `v2_low_beam_fold_nav` (beam bottom just under `carry`'s
   1.32 m top) and `v2_low_beam_extend_nav` (below the fold limit, from step 1 with a 0.03 m allowance),
   3 runs each, with Unity screenshots or a recording of each pass for the user.
3. If the extended pass exceeds the arm's qualified torque or speed envelope, the result is reported as such
   (with the speed at which it would stay inside) rather than capped silently; pose-dependent base limits
   would then be a 4B-B or Phase 5 item.

## 11. Risks

- **q larger than expected** makes the base margin eat the door width: stop rule in Section 2.
- **The tight StopZone with noise** could still trip in a 1.05 m door if the MPC plans exactly at the base
  margin: blocked detection turns that into a FollowPath failure and a replan instead of a silent freeze;
  if it recurs, the clearance cost is the next lever (reported, not switched on silently).
- **The mirror pose fails qualification** (gravity or wrist limits): report; B5 keeps the single tight pose
  and (2) moves to Phase 5 with the candidate initialization.
- **FootprintApproach scales B5 often** in tight places: the executed-command feedback makes it visible to
  the MPC; the campaign reports the scaled share as before.
- **Recorded B5 rows stop being reproducible by defaults:** the reproduction recipe is kept and documented.

## 12. Open items handed to 4B-B

The MPC rate (open decision 2026-10-13), horizon, grid, solver, warm start, collision fidelity, failure
handling, B6 go/no-go, F4 before Phase 5, and V2 timing at 50 Hz on the frozen formulation.
