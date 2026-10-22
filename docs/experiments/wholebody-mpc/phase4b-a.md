# Phase 4B-A exit campaign: B5 behaviour fixes (2026-10-18 to 2026-10-22)

The exit campaign of the Phase 4B-A spec
([design](../../superpowers/specs/2026-10-18-phase4b-a-b5-behaviour-fixes-design.md), Section 8): B5 with the
4B-A defaults at 50 Hz on real Unity, before the formulation is frozen for the 4B-B performance sweep. The first
campaign (one pass, then a rerun of the V2 runs that had failed in setup) found four defects. A fix round repaired
them, and the final campaign ran on the code that is frozen (`ae3f640`, plus the terrain-scenario removal `3723821`,
which changes no controller code).

## What the campaign tests

B5 as Phase 4B-A leaves it:

- **Safety layer (wholebody mode only).** A 0.02 m StopZone with no SlowdownZone. The MPC base margin is 0.12 m
  (padding 0.02 + StopZone 0.02 + measured lidar intrusion 0.08). A blocked detector fails FollowPath after 3 s
  of commanding without moving.
- **Executed-command feedback.** The base's previous control is the executed `/cmd_vel`. A cancel ramps the
  command down.
- **Mirrored tight poses.** `vertical_carry` (left) and `vertical_carry_right`, with a side picker and a
  clearance guard.
- **Collision-checked arm braking hold.**
- **Slack weights.** The L1 slack weight is 30 times higher (`weights.slack_inside` 3000) on arm and panel rows
  already inside their margin.

Cells (spec Section 8):

- **V1:** `overhead_course_nav` from home, `overhead_course_worker_nav`, `constrained_reconfiguration_mission`
  unscripted and scripted, `narrow_gate_home_nav`; three runs each.
- **V2:** the V2 set once, plus `v2_gate_right_swing_nav` three times.
- **Extra:** two more runs each of `v2_room_r4_nav` and `v2_store_room_nav`, the two cells with failures in
  earlier campaigns.

## Campaign history

### First campaign (2026-10-18/19, `f762693`)

| Finding | Cause | Fix |
|---|---|---|
| `v2_gate_right_swing_nav` run 2: panel contact with the shelf (0.0024 m) | Since Task 4 the QP's arm starting velocity came from a reading of the streamed spline at the start of the cycle, while the stream's first point used a second reading at publish time. On a 60-90 ms first segment the two can differ in sign. The spline's first segment then overshoots, and the next cycle's plan goes the other way. `shoulder_lift`'s first-point velocity grew 0.46, 0.80, 1.01, 1.47, then 2.13 rad/s, the hardware rejected the target (URDF limit 1.6 rad/s), arm control deactivated, and B5 kept driving the base with the arm frozen. The same oscillation, smaller, appeared in R4 and two scripted constrained runs. | `a424f0c`, `2be2397`: one reference sample per cycle, used by both the QP and the stream's first point, clamped to the arm's 0.6 rad/s box. Arm control that stops publishing its state for 0.5 s makes the state stale (brake, goal fails, error 106). |
| `v2_room_r4_nav` aborted (167.6 s) | Diagnosed then as the side choice: the picker's window scores tied at the hallway's right turn (left 0.655, right 0.641 m; the margin is 0.02 m), the tie went to the turn side (right) and stayed latched through the left turn into R4. **Not confirmed by the final campaign:** this abort ended in the R4 throat about 24° off the door axis (7.92, 13.23), the same crooked pose as the final campaign's left-swung abort (run 22, 7.78, 13.30), and a right swing succeeded there (run 39). The crooked-door dead end (below) explains both aborts. | `d4f701e`, `94f8994`, `ae3f640`: with both tight poses eligible, each pose is probed rigidly along the route up to 6 m ahead. Ties are broken in turn by the segment minimum, the window, the segment mean, then the old pan, turn and left rules. Details in [the picker](#side-picker-after-the-fix-round). Its benefit at R4 is unproven live. |
| `v2_store_room_nav` succeeded 1 of 3 | The base stalled crooked in the 1.05 m S door, dithering ±0.07 rad/s. A live A/B with `slack_inside` 100 on every row succeeded 3 of 3, but with panel clearances of 0.039, 0.067 and 0.018 m (two of three below 0.045 m). | `b2c87ae` (user decision 2026-10-19): the inside weight applies to arm and panel rows only. Base rows keep 100. |
| Every V2 run after R4 failed in setup | R4 stopped in the door, and the runner then tried to move the arm to carry with MoveIt in a place where carry does not fit. | `b38c439`: the runner starts a new Play epoch when that MoveIt change fails. |

### Fix round evidence

- **Arm stream.**
  - The final campaign has zero sign flips of the arm reference velocity in every run. The streams' first-point
    velocity peaks at the 0.6 rad/s clamp; the first campaign's contact run peaked at 2.13 rad/s.
  - In the contact run's bag, `/arm_controller/controller_state` stops right after the 2.13 rad/s point while arm
    trajectories continue for 4.3 s: the arm-control guard would have braked about 0.5 s after it stopped (code
    review of `a424f0c`).
  - The node tests check this as a property only: the mock's latency is too short to reproduce the oscillation.
- **Side picker.** `RouteProbe.R4ChoosesTheSideThatClearsTheWholeSegment` runs a closed loop on the V2 map along
  the failed run's global path. It chooses right without the route probe, as the live run did, and left with it.
  It shows that the choice changes, not that the choice caused the abort: the unit loop also enters R4 with right.
- **Store S.** No unit test could reproduce the stall.
  - A closed loop along the failed run's path passes at every weight, with no slack used.
  - Started from the live stall pose, B5 stays stuck at every weight. The forward-only base cannot back out of a
    throat. See the [limitations](#limitations-carried-forward).

## Final campaign (2026-10-22, `ae3f640`)

The V2 rows include the two Tier 2 terrain scenarios, `v2_spine_rough_nav` and `v2_scaffold_tunnel_rough_nav`.
They were removed from the V2 set after this campaign (user decision 2026-10-22), so the V2 set now has 19
scenarios.

| # | Run | Outcome | Time s | 3D clearance m (pair) | Swing | Holds | MPC p99 ms / misses | CPU % |
|---|---|---|---|---|---|---|---|---|
| 1 | overhead_course_nav | succeeded | 49.78 | 0.1133 (panel / PrimaryTransportRoute) | left, right | 0 | 4.5 / 0 | 18.2 |
| 2 | overhead_course_worker_nav | succeeded | 50.12 | 0.1516 (front_left_wheel / gate_post_north) | left, comfort, right | 0 | 9.2 / 8 | 31.1 |
| 3 | constrained_reconfiguration_mission | succeeded | 13.18 | 0.064 (panel / tall_box) | right | 0 | 7.1 / 0 | 25.7 |
| 4 | constrained_reconfiguration_mission (scripted) | succeeded | 15.04 | 0.0446 (panel / tall_box) | right | 29 | 59.4 / 47 | 43.9 |
| 5 | narrow_gate_home_nav | succeeded | 19.52 | 0.1284 (rear_left_wheel / ManipulationRequiredGate_1p05m) | right | 0 | 23.6 / 14 | 33.6 |
| 6 | overhead_course_nav | succeeded | 49.46 | 0.151 (rear_left_wheel / gate_post_north) | left, right | 0 | 8.6 / 1 | 33.1 |
| 7 | overhead_course_worker_nav | succeeded | 50.76 | 0.1546 (front_right_wheel / gate_post_south) | left, comfort, right | 0 | 14.6 / 9 | 36.9 |
| 8 | constrained_reconfiguration_mission | succeeded | 13.16 | 0.0577 (panel / tall_box) | right | 0 | 10.4 / 4 | 36.8 |
| 9 | constrained_reconfiguration_mission (scripted) | succeeded | 15.56 | 0.0325 (panel / tall_box) | right | 25 | 64.9 / 56 | 47.2 |
| 10 | narrow_gate_home_nav | succeeded | 19.72 | 0.1539 (rear_left_wheel / ManipulationRequiredGate_1p05m) | left | 0 | 11.7 / 1 | 37.8 |
| 11 | overhead_course_nav | succeeded | 49.68 | 0.1345 (panel / PrimaryTransportRoute) | left, right | 0 | 9.8 / 4 | 38.3 |
| 12 | overhead_course_worker_nav | succeeded | 51.46 | 0.1521 (rear_left_wheel / gate_post_north) | left, comfort | 11 | 21.9 / 29 | 41.6 |
| 13 | constrained_reconfiguration_mission | succeeded | 13.18 | 0.0586 (panel / tall_box) | right | 0 | 9.3 / 1 | 38.1 |
| 14 | constrained_reconfiguration_mission (scripted) | succeeded | 14.7 | 0.0716 (panel / tall_box) | right | 0 | 74.9 / 38 | 49.0 |
| 15 | narrow_gate_home_nav | succeeded | 19.52 | 0.129 (rear_left_wheel / ManipulationRequiredGate_1p05m) | right | 0 | 25.1 / 13 | 39.5 |
| 16 | v2_open_yard_nav | succeeded | 39.66 | 1.7222 (panel / Row1_04) | - | 0 | 9.7 / 3 | 34.8 |
| 17 | v2_door_130_nav | succeeded | 51.34 | 0.0934 (panel / C1West_1) | left | 0 | 9.1 / 3 | 36.5 |
| 18 | v2_door_105_nav | succeeded | 45.56 | 0.1227 (panel / LaneDoor105_West) | left | 0 | 9.2 / 1 | 36.6 |
| 19 | v2_door_105_mission | succeeded | 45.58 | 0.1184 (panel / LaneDoor105_West) | left | 0 | 9.2 / 2 | 36.8 |
| 20 | v2_through_building_mission | succeeded | 20.42 | 2.4362 (panel / LaneDoor130_West) | - | 0 | 8.0 / 2 | 33.5 |
| 21 | v2_room_r1_nav | succeeded | 49.5 | 0.1039 (panel / C1West_1) | left | 0 | 12.3 / 7 | 38.3 |
| 22 | v2_room_r4_nav | aborted | 166.14 | 0.0469 (panel / DoorR4Frame) | left | 0 | 12.8 / 33 | 36.5 |
| 23 | v2_store_room_nav | succeeded | 61.82 | 0.0956 (panel / StoreWest_1) | left | 9 | 15.3 / 20 | 37.7 |
| 24 | v2_yard_store_nav | succeeded | 98.54 | 0.0893 (panel / Row1_01) | right, left | 0 | 8.5 / 5 | 35.7 |
| 25 | v2_scaffold_tunnel_nav | succeeded | 76.84 | 0.0984 (panel / H1Ledger) | comfort | 0 | 10.7 / 3 | 36.5 |
| 26 | v2_scaffold_tunnel_mission | succeeded | 76.16 | 0.0986 (panel / H1Ledger) | comfort, right | 0 | 10.1 / 1 | 35.8 |
| 27 | v2_chicane_nav | succeeded | 97.08 | 0.1041 (panel / Chicane4_1) | right, left, right, left | 0 | 8.8 / 2 | 36.4 |
| 28 | v2_services_deadend_nav | succeeded | 75.2 | 0.228 (panel / RackSupport_1) | right | 0 | 10.9 / 2 | 34.3 |
| 29 | v2_goal_near_skip_nav | succeeded | 36.0 | 1.4081 (panel / Row1_01) | - | 0 | 8.5 / 2 | 34.2 |
| 30 | v2_worker_crossing_nav | succeeded | 31.24 | 3.092 (panel / Chicane1_6) | - | 0 | 9.7 / 2 | 34.7 |
| 31 | v2_worker_late_crossing_nav | aborted | 71.1 | 2.385 (panel / LaneDoor130_West) | right | 598 | 28.3 / 84 | 39.8 |
| 32 | v2_worker_stops_in_lane_nav | succeeded | 32.26 | 2.9812 (panel / PanelsEast_1) | right | 0 | 9.0 / 1 | 35.1 |
| 33 | v2_spine_nav | succeeded | 128.62 | 2.4454 (panel / LaneDoor130_West) | - | 0 | 9.7 / 5 | 34.1 |
| 34 | v2_spine_rough_nav (removed from the set) | succeeded | 129.16 | 2.4375 (panel / LaneDoor130_East) | - | 0 | 8.5 / 3 | 33.2 |
| 35 | v2_scaffold_tunnel_rough_nav (removed from the set) | aborted | 184.78 | 0.0986 (panel / H1Ledger) | comfort | 0 | 9.1 / 7 | 33.1 |
| 36 | v2_gate_right_swing_nav | succeeded | 31.3 | 0.0995 (panel / approach_shelf) | right | 0 | 21.0 / 18 | 37.6 |
| 37 | v2_gate_right_swing_nav | succeeded | 31.36 | 0.1113 (panel / approach_shelf) | right | 0 | 25.1 / 22 | 37.8 |
| 38 | v2_gate_right_swing_nav | succeeded | 31.32 | 0.0765 (panel / approach_shelf) | right | 0 | 21.2 / 20 | 36.6 |
| 39 | v2_room_r4_nav | succeeded | 62.9 | 0.0471 (panel / C1East_1) | left, right | 0 | 19.9 / 31 | 38.3 |
| 40 | v2_store_room_nav | succeeded | 63.82 | 0.0422 (panel / DoorSFrame) | left | 44 | 21.0 / 37 | 40.9 |
| 41 | v2_room_r4_nav | succeeded | 63.0 | 0.1213 (panel / C1West_1) | left | 0 | 10.0 / 4 | 37.3 |
| 42 | v2_store_room_nav | succeeded | 88.86 | 0.034 (panel / StoreWest_1) | left | 12 | 23.1 / 59 | 39.0 |

Notes on the table:
- "Swing" lists the side picker's choices in order (comfort: no tight pose beat carry).
- "Holds" counts slack-gate holding cycles.
- The runs are in `experiment_runs/20261022T092209Z` to `20261022T105457Z` (the per-run `summary.json`, the bag,
  `clearance_3d.json`); earlier `20261022T` directories are discarded starts.
- CPU is the MPC node's share of one core.
- Store S run 40 also had one controller failure, with braking, that the behaviour tree retried. It succeeded.

## Exit criteria

The user ruled on every criterion that failed (2026-10-22). Nothing was relaxed before that.

| Criterion (spec Section 8) | Result | Ruling |
|---|---|---|
| No robot-environment contact | **Pass**: 0 of 42 runs | - |
| V2 19 of 20, only `v2_worker_late_crossing_nav` failing; R4 and store S succeed | **Fail**: 17 of 20. R4 aborted in its campaign run (a crooked-door dead end, below), and `v2_scaffold_tunnel_rough_nav` aborted (a goal stop on gravel, below). R4 2 of 3 (the campaign run failed, both extra runs passed), store S 3 of 3. | Amended: R4 and store S succeed in most runs, with every failure listed and its cause; the crooked-door dead end is a documented B5 limitation deferred to Phase 5. Terrain scenarios removed from V2 (19 scenarios now); the gravel goal stop is a documented limitation. |
| V1: each cell at least the recorded B5's success | **Pass**: 15 of 15 | - |
| No run with a 3D clearance below 0.045 m | **Fail**: 4 runs. Scripted constrained 0.0446 and 0.0325 m (the third 0.0716). Store S 0.0422 m (panel to the door frame) and 0.0340 m (panel to the store's west wall, turning on the spot in the dead end). | Kept at 0.045 m. The shortfall is reported and the fix goes to 4B-B (the second QP iteration per cycle, spec Section 8 and the 4B-B hand-off). The freeze goes ahead. |
| `v2_gate_right_swing_nav` swings right 3 of 3; `narrow_gate_home_nav` still swings left 3 of 3 | Right swing **pass** (3 of 3). Narrow gate **fail**: left 1 of 3. | Amended: the symmetric gate passes, either side. |
| No `blocked` status in a successful run; every blocked failure listed with its cause | **Pass**. `blocked` appears only in `v2_worker_late_crossing_nav` (673 cycles; the late-crossing deadlock deferred to Phase 5). | - |
| Timing reported, not gated | MPC cycle p99 4.5-28 ms in the unscripted runs, 59-75 ms in the scripted constrained runs; MPC node CPU 18-49 % of one core. | 4B-B. |

### Why the narrow gate swung right

- The window scores favoured left by far: left 0.80 m, capped at the activation distance, against right 0.60 m.
- Along the route past the gate, the right pose stays slightly farther from things: 0.304 m against 0.249 m.
- The segment minimum outranks the window, so right won by 0.055 m. In the run that went left, both minimums
  were 0.28 m and the window decided.
- Both values are well clear of the 0.05 m margin, and right passed the gate with 0.128-0.129 m. The user
  accepted either side as valid.

## Side picker after the fix round

The picker first scores both tight poses over the first-conflict window, as before.

When both poses are eligible, it also probes them along the route:
- **Probe:** the base on the route and its heading along the route, every 0.1 m, up to 6 m ahead, with the
  obstacles where they are now.
- **Tight segment:** from the first sample where carry is within 0.30 m to the last, bridging clear gaps of up
  to 0.6 m (the horizon's travel, over which the choice stays latched).
- **Skipped samples:** samples where carry beats both tight poses, as under a beam both would pass only by
  reconfiguring, are left to the collision rows.
- **Scores:** each pose's window score is capped by its segment minimum.
- **Decision order:** the capped scores, then the window scores, then the segment means, then the old pan, turn
  and left rules. A step decides only when its difference exceeds 0.02 m.

Live at R4 (bags of runs 22, 39 and 41):
- **First choice (door A).** Everything tied in all three runs, and the old rules gave left.
- **Second choice (hallway and R4 door):**
  - run 22: the capped scores decided left (0.18 against 0.114);
  - run 39: everything tied (capped 0.18 / 0.18, window 0.649 / 0.644, means 0.401 / 0.389), and the old turn rule gave right; the run succeeded;
  - run 41: the window decided left (0.669 against 0.639).

The segment mean never decided live. The offline rigid probe of the first campaign's path had predicted it would
(both 0.18 m at the door, the means favouring left). `MpcCycle.probe_scores` logs the comfort score, the two window scores, then the two capped
scores and the two means when the route was used.

## Limitations carried forward

- **Crooked-door dead end (Phase 5).**
  - The base sometimes enters a 1.05 m door about 24-40° off the door axis, right after a 90° turn: R4 in the first
    campaign (right swing, about 24°) and in run 22 (left swing, about 27°); store S in the first campaign (about 40°).
  - B5 then cannot get out. The forward-only base cannot reverse out of the throat, and the 22-43 Nav2 recoveries
    do not free it.
  - From the stall pose a unit loop stays stuck at every slack weight. The slack weight changes how often the
    robot arrives crooked, not the dead end itself.
  - A medium horizon that approaches doors aligned is the fix the user chose (Phase 5).
- **Late-crossing worker deadlock (Phase 5).** Deferred in the spec; it fails as `blocked`, as designed.
- **Goal stop on rough ground.**
  - `v2_scaffold_tunnel_rough_nav` stopped on the gravel patch 0.154 m from the goal (tolerance 0.15 m) with its
    heading aligned.
  - It then sent only the turn breakaway (0.07 rad/s), which does not move the base there. Earlier runs stopped at
    0.141-0.147 m, at the edge of the tolerance.
  - Terrain scenarios are removed for now (user decision 2026-10-22). The Tier 2 pieces stay inactive in the
    scene, and the `disturbances` field and the runner's switch remain.
- **Clearance below 0.045 m.**
  - The scripted control row and store S's turn on the spot go below it, with no contact.
  - The fix goes to 4B-B (a second QP iteration per cycle), together with the panel-against-arm-link collision
    rows (deferred from Task 10) and the timing work.
- **Rate bound of the arm stream.** With one reference sample per cycle the reference converges at 50 Hz and rings
  (damped) at 20 Hz for latencies up to 60 ms. Below about 15 Hz it would alternate, capped by the clamp
  (review of `a424f0c`). The 4B-B rate sweep must stay at or above 20 Hz, or re-check this.

## Low passes (spec Section 10A, after the freeze, 2026-10-22)

A capability demonstration on the frozen formulation, not an exit criterion; no B5 code changed. It answers how low
B5 brings the panel under an overhead beam.

### Offline sweep

`LowPass.DISABLED_Sweep` (`test_controller`, run with `--gtest_also_run_disabled_tests`) is a closed loop on the
generated model with the frozen defaults. A 3 m wide, 0.2 m deep beam crosses a straight 8 m route; its bottom
goes from 1.30 m down to 0.80 m in 0.01 m steps, starting in carry (panel top 1.32 m).

| Beam bottom | Result |
|---|---|
| 1.30-1.20 m | Passes, at 0.049-0.050 m, with almost no holds (1.22 m: 0.030 m, 23 holding cycles). |
| 1.19 m and 1.16 m | Pass marginally: 0.011 m and 0.044 m, with 32 and 95 holding cycles. |
| 1.18 m and below | Stops before the beam, except 1.16 m. |

**Mechanism.** B5 passes neither folded (panel inside the `home` footprint) nor extended forward. Carry collides
with the beam rigidly, so the side picker switches to a tight pose. The posture blend then pulls the panel towards
vertical carry while the collision rows hold it under the beam. The panel ends about 72° to the side, tilted
0.28 rad, and lowered beside the base. The side varies between runs.

**`carry_pose_search --eval` of the arm near the beam.** Compared with carry:
- **Torque:** 71-79 Nm of shoulder gravity torque, against 19-22 Nm for the qualified carry. This is outside the
  qualified envelope; `level_extension`, at 95.7 Nm, is qualified only to 0.15 m/s and 0.2 rad/s.
- **Footprint:** 1.55 x 1.75 m, with the panel about 1.3 m out to the side.

**The unit loop's own penetrations are a harness artefact.** Some stopped runs penetrate the beam (down to -0.094 m
at 1.17 m) during slack-gate holds. In the unit loop the plant follows the controller's internal arm braking hold,
which is not collision-checked. The node replaces it with the checked hold (`safe_arm_hold`, Task 10).

### Live

Two V2 scenarios in the open delivery yard: `v2_low_beam_130_nav` and `v2_low_beam_123_nav`. Each is a straight 9 m
route with a 3 m beam at x 0, its bottom at 1.30 m and at 1.23 m (the sweep's lowest reliable pass, 1.20 m, plus
0.03 m). They are named by height, not "fold" and "extend" as the spec had it, because neither regime appeared.
The 2D band does not see the beam, so every planner plans.

| Run | Outcome | Panel to beam m | Swing | MPC p99 ms / misses |
|---|---|---|---|---|
| 1.30 m, 1 (`20261022T112854Z`) | succeeded 37.2 s | 0.0147 | right | 14.5 / 14 |
| 1.30 m, 2 (`113041Z`) | succeeded 37.8 s | 0.0467 | right | 22.3 / 25 |
| 1.30 m, 3 (`113216Z`) | succeeded 35.2 s | 0.0062 | left | 51.7 / 87 |
| 1.23 m, 1 (`113357Z`) | succeeded 37.6 s | 0.0450 | right | 40.8 / 99 |
| 1.23 m, 2 (`113539Z`) | succeeded 35.2 s | 0.0401 | left | 46.6 / 69 |
| 1.23 m, 3 (`113720Z`) | succeeded 42.4 s | 0.0048 | right | 55.0 / 144 |
| 1.23 m, watched by the user (`114412Z`) | succeeded 38.8 s | 0.0366 | right | 30.2 / 33 |
| 1.23 m, `--posture-mode comfort_only` (`115608Z`) | **aborted** 84.1 s, about 0.3 m before the beam | 0.0144 | - | 41.9 / 1146 |
| 1.30 m, with screenshot captures (`113936Z`, disturbed) | **contact** (panel and beam, 0.0016 m, 9 ticks), canceled | -0.0242 | - | 81.7 / 131 |


- **No contact in the seven undisturbed default runs.** But three of them came within 15 mm of the beam: the low
  pass has no margin to spare.
- **Without the vertical-carry pull (`comfort_only`), B5 does not get under 1.23 m.**
  - It folds the arm (`shoulder_lift` 0.97 to 0.32 rad, elbow to -2.15 rad) and pitches the panel towards vertical
    (up to 1.47 rad).
  - Its plans keep pushing into the beam: 1826 plans failed the gate, with 2269 holding cycles in 27 holds.
  - This is the one-shot linearization limit, as with the pan pivot: the side swing is not a local move.
- **The disturbed run: screenshots made contact.**
  - `scene_capture`, two 1280x720 HDRP renders per second at the beam, stalled the simulation. That gave a 0.52 s
    lidar gap, the MPC's deadlines missed, and the arm 0.147 rad behind its reference (8 re-anchors).
  - The panel touched the beam.
  - The run is not a measurement. It shows that at these clearances a timing disturbance becomes contact.
  - `scene_capture` also renders without the runtime scenario obstacles, so the frames show the posture but not the
    beam.
- **A second watched default run under 1.23 m** (`20261022T*-v2_low_beam_123_nav`, after the camera fix): it
  succeeded in 35.0 s with a left swing, closest 0.0414 m, no contact.
- **Viewing.** The robot camera pulled in to the robot under the beam, because its focus point (1.25 m up) was
  inside it. Since `5b67ba7` it ignores the scenario obstacles.
- **Open items.**
  - The torque outside the qualified envelope at speed.
  - The 5-15 mm approaches.
  - The comfort-only stall.
  - These go with the 4B-B clearance work, and with Phase 5's medium layer, which can plan the lowered
    configuration ahead (spec Section 10A, item 3).

### Why the local MPC does not fold the arm (comfort-only sweep)

`LowPass.DISABLED_SweepComfortOnly` runs the same loop with `comfort_only` posture: the posture term pulls to carry,
with no tight attractor and no side swing. Beams 1.32 m down to 1.18 m.

| Beam bottom | Result |
|---|---|
| 1.32 m | Passes with a slight fold (`shoulder_lift` 1.01, elbow -1.93 rad). |
| 1.27 m | Passes (`shoulder_lift` 1.03, `wrist_2` 1.00 rad). |
| 1.31-1.28 m, and 1.26 m down | Stops before the beam: about 700 slack-gate holding cycles, the arm in pitched configurations (`shoulder_lift` 0.1-0.2 rad). |

Folding reaches a level panel top of 1.20 m inside the base footprint (fold limit
`[0, 1.236, -2.156, 0, 0.920, 0]`, shoulder gravity torque about 22 Nm), so geometry does not stop it above
about 1.25 m. Three local reasons do:

1. **The gradient.** The closest pair as the panel approaches is its top-front edge against the beam's
   bottom-front edge. The cheapest way to increase that distance is back and down, so pulling back, slowing the
   base or pitching the panel satisfies the row more cheaply than the coordinated fold (shoulder up, elbow tucked,
   wrist compensating).
2. **The posture term.** It resists every step of a fold away from carry, while stopping costs only progress. The
   robot stalls with its plans pressing into the beam, and the gate holds.
3. **One linearized QP per cycle.** It sees first-order local moves. Folding is a coordinated multi-joint change,
   and extending forward or backward is farther still: about 6 s of arm motion to `level_extension`, beyond the
   2 s horizon.

The pivot works only because the vertical-carry attractor supplies a distant target that the collision rows bend
under the beam.

**User decisions 2026-10-22.**
- The pivot stays in the frozen B5 as a documented limitation: it needs about 1.3 m beside the robot, is not
  guaranteed indoors and loads the shoulder outside the qualified envelope.
- Lowering under overhead obstacles goes to Phase 5. The medium layer chooses and times the lowered configuration:
  fold, extend forward or extend backward, and slowing the base for an extended arm.
- A fold attractor in B5 was considered and not taken.

