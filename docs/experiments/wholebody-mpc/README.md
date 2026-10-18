# Arm qualification for the whole-body MPC (B5)

Measured 2026-10-11 in the native Unity 6000.5.2f1 Editor (normal frame settings, Play, construction-site
scene, open stability-test area from `arm_test_place_open`) and the Linux Jazzy container, with the
3 kg 1.20 x 1.20 x 0.04 m panel on `tool0` and JointTrajectoryController 4.40.1 as configured in
`mobile_manipulator_control/config/controllers.yaml` (50 Hz, path tolerance 0.15 rad, position and velocity
interfaces). All motion is simulation only; the base sequences are the bounded `--disturbance` sequences
of `arm_experiment` in a cleared area, with no Nav2 running (`/cmd_vel` has one publisher).

Every run is one `arm_experiment` call recorded by the Unity arm recorder. The per-tick recording is
reduced to the physical checks (watchdog, tilt, floor clearance, panel penetration, saturation, wall gaps)
and discarded; the reduced results of every run are in
[arm-qualification.json.gz](arm-qualification.json.gz). Reproduce with
`python3 tools/run_wholebody_arm_qualification.py {cpu,combined,stream,poses} --prefix NEW`
(the runner refuses to overwrite a prefix that exists in the JSON).

Definitions used throughout:

- **Path error**: largest absolute value of the JTC controller-state error
  (`/arm_controller/controller_state`, `error.positions`) per joint while the trajectory ran
  (`max_path_error_rad` in the tool report; a run's value is the maximum over the six joints).
  The acceptance bound for the MPC is 0.10 rad, two thirds of the 0.15 rad tolerance.
- **Position jump**: a controller-state sample whose commanded (reference) position differs from the
  previous sample by more than 0.01 rad beyond what the commanded velocity explains
  (`|dq - 0.5 (v_prev + v) dt| > 0.01` on any joint, `dt` from the message stamps).
- **Watchdog flag**: any recorded arm state other than HOLD or EXTERNAL_CONTROL. **Tilt flag**: base tilt
  of 3 degrees or more.

## Tool changes

`arm_experiment` gains `--cpu-load-threads N` (N busy threads without core affinity for the run),
`--drive-during-motion` (starts the `--disturbance` sequence when the goal is sent, so the arm moves while
the base drives; the sequence continues into the hold), and `--stream-rate HZ --stream-duration S`
(with `--stream-amplitude`, `--stream-frequency`, `--stream-stop-after`) for JTC streaming. The report adds
`max_path_error_rad`, `max_reference_velocity_radps`, `position_jumps`, `max_position_jump_rad`,
`controller_state_samples` and `cpu_load_threads`.

## CPU headroom

Host: Apple M4, 10 cores (4 performance, 6 efficiency), 16 GB. Docker Desktop VM: `NCPU=10`, 7.75 GiB,
aarch64. The container has no CPU limit (`NanoCpus=0`, `CpuQuota=0`, `CpuShares=0`, no cpuset), `nproc`
reports 10. Unity runs on the host and competes with the VM for the same ten cores; the container's idle
stack (endpoint, controller manager, TF) uses about 0.3 of one core when idle.

Reference transition home <-> vertical_carry (8 s synchronized cubic, 2 s hold), 3 round trips
(6 recorded transitions) per load, run with the arm starting in `home`. Loads of 0 and 4 were repeated
to 9 and 9 round trips to separate load from chance stalls; loads above the requested 0, 1, 2, 4, 6 were
added because nothing exceeded 0.10 rad up to 6 threads.

| Busy threads | Runs | Failed | Median run path error, rad | Largest path error, rad | Watchdog flags |
|---:|---:|---:|---:|---:|---:|
| 0 | 18 | 0 | 0.0172 | 0.0309 | 0 |
| 1 | 6 | 0 | 0.0183 | 0.0298 | 0 |
| 2 | 6 | 0 | 0.0186 | 0.0255 | 0 |
| 4 | 18 | 1 | 0.0247 | 0.0280 | 0 |
| 6 | 6 | 0 | 0.0213 | 0.0315 | 0 |
| 8 | 6 | 0 | 0.0293 | 0.0448 | 0 |
| 10 | 6 | 0 | 0.0379 | 0.0651 | 0 |
| 12 | 6 | 1 | 0.0411 | **0.1517** | 1 |
| 14 | 6 | 0 | 0.0390 | 0.0451 | 0 |
| 16 | 6 | 0 | 0.0500 | 0.0642 | 0 |
| 24 | 6 | 0 | 0.0486 | 0.0945 | 0 |
| 32 | 6 | 0 | 0.0503 | 0.0631 | 0 |

Findings:

- The path error grows slowly with load (0.017 rad idle, 0.04 to 0.05 rad median from 10 threads, which
  fully occupy the ten VM cores), and the return transition (vertical_carry -> home) is always the worse
  one (largest values 0.031 idle, 0.065 at 10, 0.095 at 24 threads); `shoulder_pan` and `wrist_2` carry the
  error.
- The first load at which the path error exceeded 0.10 rad is **12 threads**: one of six runs, a Unity-side
  stall (0.647 s wall gap between physics samples, command age 0.90 s, `WATCHDOG_HOLD`, action aborted with
  0.152 rad). It did not recur at 12 threads (5 of 6 ok), nor at 14, 16, 24 or 32 threads, so it is a
  sporadic stall that heavier load makes more likely, not a clean threshold.
- The **CPU budget** is 10 busy threads, the whole VM: the highest requested-or-lower load below the first
  exceedance (12) at which no run exceeded 0.10 rad or stalled. It is a rule-based figure, not a cliff, since
  heavier loads (14 to 32 threads) passed. Plan the B5 node and everything else in the container to stay within
  about 10 cores in total, and well under that for the MPC, which runs in a single control thread.
- Two sporadic failures are not monotonic in load. At 4 threads one run ended with the tool's own
  "lost feedback" check (no `/arm/state` for 0.5 s of wall time) while Unity's recording showed healthy
  physics (largest wall gap 0.097 s, command age 0.13 s); 17 other runs at 4 threads and 18 at 0 did not.
  In the combined-motion batch below, an unloaded run at scale 1.5 hit `WATCHDOG_HOLD` (0.404 s wall gap,
  command age 0.54 s). These are the stalls documented for low frame rates in the arm qualification;
  they are why the MPC needs its own stale-feedback braking.

## Combined arm and base motion

`home -> vertical_carry` and back with `--disturbance compact --drive-during-motion` (starts at the goal:
forward 0.3 m/s for 3 s, 1 s stop, reverse, yaw +-0.4 rad/s, curves at 0.15/0.2, then stop; 23 s in all, the
rest of it during the hold). The speed scale divides the reference duration: scale s runs the same
synchronized cubic in 8/s seconds. 3 round trips (6 transitions) per scale; the base was re-placed in the
open area before each run.

| Scale | Duration, s | Peak joint speed (pan, wrist_2), rad/s | Largest path error, rad | Largest hold error, rad | Watchdog flags | Largest tilt, deg | Min panel bottom, m |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.0 | 8.000 | 0.30 | 0.0251 | 0.0240 | 0 | 0.13 | 0.709 |
| 0.75 | 10.667 | 0.22 | 0.0225 | 0.0232 | 0 | 0.13 | 0.709 |
| 0.5 | 16.000 | 0.15 | 0.0198 | 0.0174 | 0 | 0.16 | 0.709 |
| 1.5 (extra) | 5.333 | 0.45 | 0.0318 | 0.0259 | 1 (stall, see above) | 0.14 | 0.709 |
| 2.0 (extra) | 4.000 | 0.61 | 0.0364 | 0.0247 | 0 | 0.13 | 0.709 |

All runs of the five scales passed with the base driving at 0.30 m/s and 0.4 rad/s (measured peaks 0.294 to
0.300 m/s): path error at most 0.036 rad (limit 0.10), hold error at most 0.026 rad, no tilt flag (tilt at most
0.16 degrees against 3), no panel penetration, no saturation, floor clearance 0.709 m. The single watchdog
flag (scale 1.5) is an unloaded Unity stall (0.404 s wall gap, command age 0.54 s, path error 0.030 rad at
the time), the same kind of host stall seen at 12 busy threads and not related to the scale; it is treated as
a stall of the run, not as a failure of the scale, and the other five scale-1.5 runs passed.

**Decision rule**: `wholebody_arm` uses the largest tested scale for which every run passed all checks
(path error <= 0.10 rad, no watchdog or tilt flag). The brief asked for scales 1.0, 0.75 and 0.5; 1.5 and 2.0
were added runs, and scale **2.0** (4 s cubic, 3.9 s segment) is the largest tested. All six of its runs passed.
Its segment (1.5708 rad on `shoulder_pan` and `wrist_2`) peaks at 1.5 x 1.5708 / 3.9 = 0.604 rad/s and
6 x 1.5708 / 3.9^2 = 0.620 rad/s^2 (measured reference peak 0.605 to 0.610 rad/s); the `wholebody_arm` block in
`nav_operating_envelope.yaml` uses 0.60 rad/s and 0.61 rad/s^2, rounded down. The block applies these values
to all six joints, but the transitions exercise only `shoulder_pan` and `wrist_2`; the streamed sinusoids below
exercise all six joints only at 0.31 rad/s and 0.49 rad/s^2 (within 0.09 rad). The other four joints are
therefore untested at the chosen limits. Scale 2.0 is still below the tool's own guard (half the URDF velocity
limit, 0.5 x the acceleration envelope), which would allow about 2.5; faster scales were not run.

`position_jumps` is 1 on 10 of the 15 vertical_carry -> home transitions and 0 on the other five and on all
15 home -> vertical_carry transitions (each jump 0.0147-0.0149 rad, the `wrist_2` hold sag of 1.5856 against
1.5708 rad): with `interpolate_from_desired_state: false` JTC starts an action trajectory at
the measured state, so the first reference sample steps by the holding error. It is not a streaming
discontinuity.

### Held poses beyond the named ones

Each pose is built from `vertical_carry` (`lowered` from `home`), reached with the 8 s cubic and held
for 20 s, without and with `--drive-during-motion` (compact sequence; the lowered pose uses the extended
sequence, the 0.15 m/s and 0.2 rad/s that `level_extension` is qualified for). None was rejected by the
tool's URDF limit and acceleration-envelope checks. Hold error is the largest deviation from the target
over the six joints during the hold; the panel bottom is measured by Unity.

| Pose (change from vertical_carry) | Target | Hold error, still / driving, rad | Min panel bottom, m | Result |
|---|---|---:|---:|---|
| vertical left (`shoulder_pan` +0.5) | 2.0708 | 0.0148 / 0.0155 | 0.709 | holds |
| vertical right (`shoulder_pan` -0.5) | 1.0708 | 0.0149 / 0.0153 | 0.709 | holds |
| tilted `wrist_2` -0.4 | 1.1708 | 0.0138 / 0.0147 | 0.709 | holds |
| tilted `wrist_3` +0.4 | 0.4 | 0.0148 / 0.0155 | 0.528 | holds |
| tilted `wrist_3` -0.4 | -0.4 | 0.0151 / 0.0154 | 0.528 | holds |
| lowered (home, `shoulder_lift` 1.29) | 1.29 | 0.0266 / 0.0272 | 0.252 / 0.250 | holds |
| tilted `wrist_2` +0.4 | 1.9708 | not held: aborted at 1.806 rad | 0.678 | **fails** |
| probe `wrist_2` +0.1 | 1.6708 | 0.0147 / 0.0153 | 0.694 | holds |
| probe `wrist_2` +0.2 | 1.7708 | 0.0146 / 0.0152 | 0.680 | holds |

The `wrist_2` +0.4 pose failed in both modes: the joint stalled at 1.806 rad (path error 0.150 rad, JTC
status 6 "path tolerance violation"), the panel overlapped another collider by 0.5 mm and a wrist drive sat
at its torque limit for 24 % of the samples. The cause was not identified: contact with another body or the
wrist torque limit (or both). Probes at +0.1 and +0.2 rad held within 0.016 rad with no overlap, so
`wholebody_arm.joint_bounds` limits `wrist_2` to at most **1.77 rad** and leaves every other joint on its
URDF range. This was measured at the vertical_carry pan only, with the other joints fixed; the bound is a
conservative per-joint limit, and the MPC's collision model owns the geometry. The
lowered pose uses `shoulder_lift` 1.29 rad: the planar formula of the arm-controller README gives 0.295 m
nominal, the measured panel bottom is 0.252 m (0.043 m of payload sag), meeting the 0.25 m target.

## JTC streaming

`arm_experiment --stream-rate R --stream-duration S` publishes, at R Hz on simulation time, overlapping
segments of `q0 + 0.2 sin(2 pi 0.25 t)` on all six joints (peak 0.314 rad/s, 0.49 rad/s^2) on
`/arm_controller/joint_trajectory`: header stamp 0 (start now), points at 0, 0.25, 0.5, 0.75 and 1.0 s with
positions and velocities sampled from the sinusoid at the publish time, then a braking-tail point at 1.5 s
at rest. Each segment is therefore **1.5 s** (1 s of motion plus a 0.5 s braking tail), because JTC rejects a
segment that ends moving (findings below). The arm starts in `home`. `--stream-stop-after T` stops the stream
at T; the tool then watches the arm settle for 2.5 s after the last segment's braking-tail end (the rest point,
`braking_tail_end` in the report).

Final design (1.5 s segments: 1 s of motion plus a 0.5 s braking tail that ends at rest):

| Run | Segments | Achieved rate, Hz | Position jumps | Largest jump, rad | Path error, largest joint (`wrist_1`), rad | Error against the sinusoid, rad | Settle error against the braking-tail end (rest point), rad |
|---|---:|---:|---:|---:|---:|---:|---:|
| 20 Hz, 20 s | 401 | 20.1 | 0 | 0.0061 | 0.0636 (others <= 0.023) | 0.070 | 0.011 |
| 50 Hz, 20 s | 1001 | 50.0 | 0 | 0.0058 | 0.0749 (others <= 0.027) | 0.081 | 0.011 |
| 20 Hz, stopped at 7.3 s (mid-motion) | 147 | 20.1 | 0 | 0.0034 | 0.0736 (others <= 0.026) | 0.080 | 0.009 |

The simulation clock ticks at 50 Hz, so the tool schedules each segment on the average period rather
than resetting the timer. Settle errors of 0.009 to 0.011 rad are the usual `shoulder_lift` hold sag; after
the stop-mid-motion run the arm rests at the braking-tail end (the 1 s sinusoid point plus 0.25 s times its velocity), and the JTC
reference is constant there. In the committed results the key `braking_tail_end` (runs `tail-*`) holds this rest
point; the pre-tail runs `jtc0-*` and `jtc1-*` had no tail, and their `last_segment_end` key is the 1 s sinusoid point.

JTC setting findings (all on the unchanged `controllers.yaml`; no setting had to be changed):

- With `allow_nonzero_velocity_at_trajectory_end: false` JTC **rejects any segment whose last point still
  moves**. Streaming plain 1 s sinusoid segments (before the braking tail was added) kept only the few
  segments that happened to end near a velocity zero: 18 position jumps of up to 0.32 rad, path error 0.33 rad,
  0.53 rad from the sinusoid, and 0.29 to 0.33 rad error after a stop (runs `jtc0-*`).
- Setting the flag to `true` accepted the segments (0 jumps, path error 0.063 to 0.069 rad) but, when the
  stream stopped, the arm did not stop at the 1 s segment end: it settled up to 0.034 rad past it
  (`settle_error_rad` of `jtc1-stop-mid-motion` in the committed results; same tool version as `jtc0`). A
  controller-state log of that run, which is not in the committed results, showed the JTC reference velocity
  still at about 0.28 rad/s (the last point's velocity) after the stream ended; how long it stayed there was
  not measured. A trajectory allowed to end moving is a hazard if the MPC dies mid-motion, so the flag stays
  `false`.
- A **braking tail** (a last point 0.5 s after the 1 s segment end, at rest, 0.25 s times the end velocity
  beyond it, i.e. constant deceleration) satisfies the validator, never executes while newer segments keep arriving
  (every segment starts at the current time with the sinusoid's own position and velocity, so consecutive
  segments are continuous: 0 jumps), and brings the arm to rest at the tail end if the stream stops. The
  Task 10 node must stream this way, or the tail's equivalent (hold at the current joints, zero velocity)
  when it stops.
- `interpolate_from_desired_state` is read-only in this JTC and was not needed: with the first point at time 0
  each segment starts on the commanded trajectory, not on the measured state.

Limits of the evidence: one 20 s run per rate and one stop run, at a single amplitude and frequency
(0.2 rad, 0.25 Hz, inside the tool's guard); no streaming was run while the base drove.

## Bring-up

First live drives of B5 (2026-10-12): native Unity 6000.5.2f1 Editor in Play, construction-site scene, the
Linux Jazzy container, `tools/run_nav_scenario.py <scenario> --controller wholebody` (the first run with
`--new-epoch`, which restarts the endpoint, Play and arm control). Unscripted (`references:=none`) and 20 Hz
unless noted, one run per row. Contact is Unity's robot-environment contact report; the 3D clearance is
`clearance_3d` (the MPC's collision model at the recorded base pose and joints against the known boxes,
no margin); p99 and misses are the node's whole-cycle time against the period (`mpc.total_s` in
`summary.json`); a near miss is a cycle above 0.8 of the period. This first bring-up streamed the arm as
Task 10 built it; [Fix round 1](#fix-round-1) below changes the streaming and the final approach, and its
results supersede the tuning, arm tracking, stop and rate paragraphs here.

| Scenario | Refs | Rate | Outcome | Time, s | Contact | p99, ms | Misses / cycles | Near misses | 3D clearance, m |
|---|---|---:|---|---:|---|---:|---:|---:|---|
| open_space_nav | none | 20 | succeeded | 14.8 | none | 6.6 | 0 / 295 | 0 | 2.498 (panel, CrossLaneFenceNorth) |
| goal behind the robot (below) | none | 20 | succeeded | 15.6 | not measured | 5.2 | 0 / 312 | 0 | not measured |
| wide_gate_vertical_carry_nav | none | 20 | succeeded | 31.2 | none | 8.9 | 0 / 623 | 0 | 0.187 (panel, ApproachWallRight) |
| narrow_gate_vertical_carry_nav | none | 20 | succeeded | 32.5 | none | 7.8 | 0 / 650 | 0 | 0.099 (panel, GatePostRight) |
| wide_gate_mission | none | 20 | succeeded | 45.6 | none | 16.0 | 1 / 911 | 1 | 0.185 (panel, ApproachWallRight) |
| wide_gate_mission | scripted | 20 | succeeded | 40.8 | none | 25.0 | 2 / 807 | 2 | 0.278 (front left wheel, GatePostRight) |
| narrow_gate_mission | none | 20 | succeeded | 46.9 | none | 14.5 | 0 / 937 | 0 | 0.113 (panel, GatePostRight) |
| wide_gate_mission | none | 50 | succeeded | 45.4 | none | 13.3 | 2 / 2269 | 5 | 0.156 (panel, GatePostRight) |
| narrow_gate_vertical_carry_nav | none | 50 | succeeded after 1 recovery | 33.3 | none | 16.7 | 10 / 1648 | 21 | **0.014** (panel, GatePostRight) |
| narrow_gate_vertical_carry_nav | none | 50 | succeeded | 32.6 | none | 18.8 | 15 / 1594 | 26 | **0.030** (panel, GatePostRight) |

Every run reached the goal within the 0.15 m / 0.15 rad tolerance (final errors 0.10 to 0.15 m, at most
0.12 rad) with no contact. Unscripted, B5 passed the 1.05 m gate (`narrow_gate_vertical_carry_nav`,
`narrow_gate_mission`) without stopping. The MPC process used 9 to 12 % of a core on the navigate scenarios
and about 20 % on the missions at 20 Hz, 48 to 50 % at 50 Hz. Cycles were 25 OSQP iterations at the median
(at most 225, in `narrow_gate_mission`); no cycle needed the fallback plan, and the collision slack stayed at zero.

**Tuning.** No weight in `wholebody_mpc.yaml` was changed in the first bring-up: no run failed for a tuning
reason (fix round 1 changed `lag` and `progress`).

**Goal behind the robot** (2026-10-06 probe; `navigation.launch.py controller:=wholebody` started where the
robot stood after `scenario_place --x 12.01 --y 0 --yaw -3.14159 --arm_pose home`, goal (15, 0, 0) in `map`
with `ros2 action send_goal /navigate_to_pose`). The MPC started the in-place turn from rest at the
breakaway (command 0.08 rad/s, then 0.40 rad/s; measured up to 0.45 rad/s), blended into the forward drive
and stopped at (14.897, -0.019, 0.117 rad), 0.105 m and 0.117 rad from the goal, 15.6 s after the goal was
accepted. The commands stayed inside the envelope (0.30 m/s, 0.40 rad/s); the measured base overshot them
briefly in the turn-to-drive blend (0.37 m/s, 0.71 rad/s), the Unity skid-steer response to combined
commands.

**Tightness and arm adjustment.** The posture preference rose before each gate, held 1 through it and
returned to 0 after it; the arm moved while the base drove, with no stop at the mission's reconfiguration
waypoints (the base slowed to about 0.14 m/s near the gates, where the collision monitor's swept-hull
slowdown zone halved the MPC's 0.30 m/s command; the MPC itself never slowed for the arm).

| Run | Gate line x, m | Tightness above 0 from x | 1.0 through | Back to 0 by x (goal) |
|---|---:|---:|---|---|
| wide_gate_mission | -3.0 | -0.92 | -2.2 to -3.9 | -6.07 (-6.8): arm back in home |
| narrow_gate_mission | -7.2 | -4.89 | the gate | -10.69 (-11.0): arm back in home |
| wide_gate_vertical_carry_nav | -3.0 | -0.93 | -2.2 to -3.9 | 0.19 at the goal, 1.8 m after the gate |
| narrow_gate_vertical_carry_nav | -7.2 | -4.88 | -6.3 to -8.0 | 0.64 at the goal, 1.8 m after the gate |

The vertical-carry scenarios start in vertical carry in open space, so the arm first returns to home
(tightness 0) and then rises to vertical carry again before the gate. In the scripted run the arm reached
the two panel references (vertical panel within 15 mm and 0.055 rad, home panel within 13 mm and 0.014 rad
by forward kinematics) with other joint configurations than the named poses (`shoulder_pan` and `wrist_3`
counter-rotated), since the posture preference is off while a reference is active; the arm therefore ends
outside a qualified pose and the runner restores one with MoveIt before the next run.

**Arm tracking.** The JTC path error stayed at most 0.031 rad at 20 Hz and 0.056 rad at 50 Hz (the
0.15 rad tolerance), and Unity's arm
checks passed on the unscripted missions. This error says little while streaming: every streamed trajectory
starts its first point 0.1 s ahead, so the JTC starts each one from the measured state, and the reference is
re-anchored on the measured joints every cycle. Measured against the plan at 20 Hz, the joints were within
0.034 rad (p99) of the plan's stage 1 one stage later, but the gravity-loaded `wrist_2` did not converge: in vertical
carry it held 1.69 to 1.70 rad while every plan ran it to 1.571 rad (0.12 rad of panel tilt at 20 Hz), and
at 50 Hz it stalled at 1.75 to 1.76 rad (measured velocity 5 to 27 % of the planned one, against 45 to 74 %
at 20 Hz). The two 50 Hz narrow-gate runs passed the gate post at 0.014 and 0.030 m, inside the 0.05 m
margin, where the 20 Hz runs (n = 1 per scenario) kept 0.099 m (`narrow_gate_vertical_carry_nav`) and
0.113 m (`narrow_gate_mission`, a different route into the gate). The plans respected the qualified arm limits
(at most 0.60 rad/s, 0.61 rad/s²); the JTC reference peaked at 0.67 to 0.73 rad/s, and the measured
accelerations had p99 0.5 to 0.9 rad/s² with short peaks up to 1.75 rad/s² (`wrist_2`, `shoulder_pan`) at
20 Hz, and p99 1.4 rad/s² with peaks of 3.5 rad/s² on `wrist_2` at 50 Hz on the narrow gate.
The qualification's streaming put the first point at time 0 on the commanded trajectory; the node's stream
(stages 1..N) does not, which is the likely cause.

**Stops.** A cancel at tightness 0.5 with the arm moving (`shoulder_pan` 0.25 rad/s, `wrist_2` 0.28 rad/s
measured; 0.43 rad/s streamed) gave a braking hold of T = 0.71 s; the arm came to rest within 0.009 rad of the
hold target, 0.15 rad past the cancel point, with peak measured decelerations of 0.89 rad/s² (`shoulder_pan`)
and 0.73 rad/s² (`wrist_2`), above the 0.61 rad/s² limit: the hold target uses the streamed velocity, but the
JTC starts the hold from the measured, slower one and must catch up. Goal ends with the arm still moving
peaked at 0.85 and 1.24 rad/s² (20 Hz) and the 50 Hz stops at 1.5 to 1.9 rad/s² (where streamed and
measured velocities differ most); the arm settled within 0.014 rad of every hold target. The base stops at
the goal tolerance from 0.15 to 0.20 m/s, not from the breakaway speed (likely because the lag reference
stays one full-speed step ahead until 0.03 m before the goal and outweighs the progress taper); Unity's base
takes about 0.35 s to stop.

**Timing and the campaign rate.** On `wide_gate_mission` at 50 Hz, p99 was 13.3 ms (< 20 ms) and 2 of 2269
cycles missed (0.09 %), which meets the timing rule for 50 Hz. On the narrow gate at 50 Hz, p99 was 16.7 and
18.8 ms with 0.6 and 0.9 % misses; one host stall (four consecutive cycles of 26 to 127 ms, every stage
slow) exceeded the three fallback cycles, which at 50 Hz cover only 60 ms, so the MPC braked, `FollowPath`
failed and the behaviour tree's recovery replanned. Because of the `wrist_2` stall and the margin violations
above, the first bring-up chose 20 Hz until the arm streaming tracked at 50 Hz (at 20 Hz the largest p99 was
16.0 ms unscripted and 25.0 ms scripted, period 50 ms); fix round 1 below chose **50 Hz**.

**Host stalls.** In the scripted run a container-wide stall (0.33 to 0.40 s gaps on `/clock`,
`/joint_states` and the JTC state; MPC cycles of 95 and 101 ms) left Unity's arm without a fresh command
for 0.62 s: one `WATCHDOG_HOLD` of 0.18 s, the run's only failed arm check. It is the sporadic stall seen in
the qualification, not a B5 fault.

**Runner and telemetry checks.** In wholebody mode the runner's `check_cmd_vel_ownership --expect-upstream
wholebody_mpc` (only `wholebody_mpc` publishes `/cmd_vel_smoothed`), `footprint_wait --mode dynamic
--global-only` and the preflight on `/global_costmap/costmap` passed in every run; the MPC wrote its timing
summary on shutdown into `summary.json` (`mpc`); the bags hold `/wholebody_mpc/cycle` and `/wholebody_mpc/plan`
and, since this bring-up, `/arm_controller/controller_state` and `/arm_controller/joint_trajectory`
(the MPC moves the arm in every B5 run); `clearance_3d_m` was filled. `/mm/telemetry` carried controller
`WHOLEBODY`, behaviour tree `wholebody`, goal status, distance remaining and the path, which the Unity window
prints as received. `navigation.launch.py mpc_rate_hz:=50` (runner `--mpc-rate-hz 50`, run directory suffix
`-50hz`) sets the MPC rate.

### Fix round 1

Two changes after the first bring-up (rulings R24 and R25), then the bring-up set again at 20 and 50 Hz
(2026-10-12, same setup; the first run with `--new-epoch`).

- **Streaming from the commanded reference (R24).** Each streamed trajectory now starts with point 0 at
  time 0 on the reference the JTC commands at that moment (the last trajectory sent, sampled as the JTC's
  cubic spline: positions and velocities), followed by the plan's stages 1..N and the braking tail. With
  `interpolate_from_desired_state: false` (unchanged, B3 keeps its recorded controller settings) the JTC
  splines from the measured state only towards a first point in the future; a first point at time 0 is
  where it continues. The MPC's arm state is that reference while every joint is within 0.15 rad of the
  measured joints, otherwise the measured joints (a re-anchor, counted as `arm_reanchors` in the timing
  summary and flagged in `MpcCycle` with `arm_reference_offset`). Braking holds start from the reference
  (q, v) at time 0 and stop at q + v T / 2: a constant deceleration v / T.
- **Final approach (R25).** The lag reference is one step at the tapered progress speed ahead of each
  stage (it was one full-speed step), so the base slows along the taper; `lag` 20 -> 30 and `progress`
  10 -> 20 in `wholebody_mpc.yaml` keep a goal heading perpendicular to the route's end within the goal
  tolerance (with 20/10 the new reference parked 0.153 m away in the closed-loop test).

| Scenario | Refs | Rate | Outcome | Time, s | Contact | p99, ms | Misses / cycles | Re-anchors | 3D clearance, m | Largest JTC error, rad (joint) | Arrival, m/s |
|---|---|---:|---|---:|---|---:|---:|---:|---|---|---:|
| open_space_nav | none | 20 | succeeded | 16.6 | none | 6.2 | 0 / 331 | 0 | 2.515 | 0.003 (lift) | 0.065 |
| goal behind the robot | none | 20 | succeeded, 0.143 m / 0.047 rad | 18.7 | none | 7.0 | 1 / 351 | 0 | 4.184 | 0.019 (wrist_1) | 0.076 |
| narrow_gate_vertical_carry_nav | none | 20 | **failed**: held at the gate by the stop zone (below) | 90 (timeout) | none | 13.0 | 0 / 1688 | 0 | 0.103 (wheel, GatePostRight) | 0.044 (wrist_2) | — |
| narrow_gate_vertical_carry_nav (repeat) | none | 20 | succeeded | 37.1 | none | 15.8 | 0 / 741 | 0 | 0.080 (panel, ApproachWallRight) | 0.040 (wrist_2) | 0.024 |
| wide_gate_mission | none | 20 | succeeded, arm checks pass | 46.5 | none | 11.1 | 0 / 929 | 0 | 0.155 (panel, ApproachWallRight) | 0.029 (wrist_2) | 0.059 |
| wide_gate_mission | scripted | 20 | succeeded, arm checks pass | 42.7 | none | 15.8 | 0 / 852 | 0 | 0.278 (wheel, GatePostLeft) | 0.037 (wrist_2) | 0.044 |
| open_space_nav | none | 50 | succeeded | 16.7 | none | 13.3 | 2 / 835 | 0 | 2.518 | 0.003 (lift) | 0.058 |
| goal behind the robot | none | 50 | succeeded, 0.142 m / 0.047 rad | 19.8 | none | 19.9 | 8 / 858 | 0 | 4.190 | 0.018 (wrist_1) | 0.062 |
| narrow_gate_vertical_carry_nav | none | 50 | succeeded | 37.3 | none | 12.6 | 4 / 1864 | 0 | 0.100 (panel, ApproachWallRight) | 0.038 (wrist_2) | 0.045 |
| wide_gate_mission | none | 50 | succeeded, arm checks pass | 46.8 | none | 11.3 | 3 / 2340 | 0 | 0.151 (panel, ApproachWallRight) | 0.067 (wrist_2) | 0.065 |

Arrival is the measured base speed when the goal was reached; the goal-behind rows are the probe above
(`navigation.launch.py` started directly, contact from Unity, clearance against the map boxes). Every final
error was 0.142 to 0.149 m and at most 0.047 rad. No run re-anchored, needed the fallback plan or used
collision slack; OSQP needed at most 175 iterations in these runs (the first bring-up's 225 was in
`narrow_gate_mission`, which the fix round did not rerun).

- **Tracking.** The JTC error is meaningful again (the reference now runs continuously): at most 0.067 rad
  over all runs and joints, p99 at most 0.029 rad, against the 0.15 rad tolerance and the 0.10 rad acceptance
  bound. In vertical carry `wrist_2` held 1.5856 rad against the reference's 1.5708 rad (the 0.015 rad hold sag
  of the qualification) at both rates, where it had stalled 0.12 rad (20 Hz) and 0.19 rad (50 Hz) above it.
  The joints were within 0.042 rad (p99) of the plan's stage 1 one stage later and within 0.042 rad of its
  stage 5 half a second later (0.14 to 0.21 rad before). The JTC reference stayed within 0.61 rad/s; the
  measured accelerations had p99 0.52 to 0.77 rad/s², with isolated peaks up to 3.4 rad/s² (`wrist_2`,
  50 Hz wide mission). The narrow gate's 3D clearance at 50 Hz is now 0.0997 m (was 0.014 and 0.030 m).
- **Stops.** A cancel at tightness 0.5 with the arm moving (`wrist_2` reference 0.377 rad/s) started the hold
  exactly on the reference (no step in the JTC reference), which decelerated at 0.61 rad/s² for T = 0.62 s at
  both rates; the measured velocity followed it with a peak deceleration of 0.78 to 0.84 rad/s², and the arm
  came to rest 0.009 rad past the target (the hold sag). Goal ends now happen with the arm almost at rest.
- **Final approach.** The base reached the goal tolerance at 0.024 to 0.076 m/s (0.15 to 0.20 m/s before);
  GoalReached's step to zero is now about the breakaway speed.
- **The failed narrow-gate run.** The first global plan of that run bent 0.2 m to the side and back; the
  replanned route then ran into the 1.05 m gate 0.05 m off its centre line. The collision monitor's stop
  zone (base polygon plus margin, lidar points up to base height) saw the gate posts and zeroed the MPC's
  0.30 m/s command repeatedly 0.5 m before the gate line; the behaviour tree's recoveries (32) kept the same
  route until the 90 s timeout. The MPC kept commanding 0.30 m/s with zero collision slack; it is a route-centring and
  safety-zone interaction, not an arm or rate effect, and the repeat at 20 Hz and the 50 Hz run passed.
- **Rate (ruling R26).** 50 Hz needs p99 < 20 ms, misses < 1 %, no run inside the 0.05 m margin and joint
  tracking within 0.10 rad. At 50 Hz: p99 11.3 to 19.9 ms, misses 0.13 to 0.93 % (the goal-behind probe is the
  tightest: 19.9 ms and 0.93 %, in a cancel test 0.42 %), clearance at least 0.0997 m, largest JTC error
  0.067 rad. All four hold, so **the campaign runs at 50 Hz**. The margin on timing is small (the probe); the
  MPC process uses 43 to 47 % of a core at 50 Hz.
- **Fallback budget (ruling R27).** These runs used three fallback cycles, which cover only 60 ms at 50 Hz
  (one host stall of 26 to 127 ms cycles ended a `FollowPath` in the first bring-up). The budget is now time:
  `fallback_s: 0.3` in `wholebody_mpc.yaml`, K = ceil(0.3 s x rate) = 6 cycles at 20 Hz and 15 at 50 Hz,
  covering the 0.3 to 0.4 s host stalls of the arm qualification at either rate (logged at start-up and in
  the timing summary as `fallback_cycles`). No run above used the fallback plan, so their results stand.

## Phase 4 campaign (2026-10-12/13, `5458e0f`)

The spec's Section 9 matrix, as run under rulings R10 and R29, with B5 at 50 Hz (fix round 1, ruling R26).
R10: the B4 regression missions and strategy drives are rerun inside the campaign with the final `/cmd_vel`
motion source. R29: B3 and B4 run the overhead courses with RPP only and B1/B2 with RPP, DWB and MPPI; B5
runs unscripted at 50 Hz everywhere, scripted only on the three B3 missions; plus the timing runs.
**143 of 143 runs recorded in 45 cells** (3 runs per cell; 5 for the four B4 base-only drives), commit
`5458e0f` with a clean tree in every run, 2026-10-12 18:34Z to 2026-10-13 00:04Z (3.7 h of runs in 5.5 h),
real-time factor 0.981-1.036, no host sleep and no container OOM kill in any recorded run. A local,
resumable driver started one runner invocation per run (`--runs 1`; `--new-epoch` for the first run and
after an infrastructure failure); its manifest and logs stay in the git-ignored
`experiment_runs/`. **No run had robot-environment contact.** Tables come from
`python3 tools/summarize_nav_runs.py <run directories>` per group, plus the run summaries and the bags
(`/cmd_vel_smoothed` against `/cmd_vel`, `/wholebody_mpc/cycle`, `/wholebody_mpc/plan`, `/plan`,
`/arm_controller/controller_state`).

| Group | Cells | Runs |
|---|---|---|
| B5 on the 8 Phase 1 driving scenarios | 8 | 24 |
| B5 on the 3 B3 missions (route start to goal, one navigate task) | 3 | 9 |
| B5 `--references scripted` on the 3 B3 missions (control row) | 3 | 9 |
| B5 on `worker_late_crossing_nav`, `worker_stops_in_lane_nav` | 2 | 6 |
| B5 on `overhead_course_nav`, `overhead_course_worker_nav` (`--arm-pose home`) | 2 | 6 |
| RPP, DWB, MPPI, B4 (RPP + dynamic footprint) on the two worker scenarios | 8 | 24 |
| RPP, DWB, MPPI on the two overhead drive versions; B3 (RPP) and B4 on the two overhead mission versions | 10 | 30 |
| Timing at 20 Hz: `wide_gate_mission`, `overhead_course_nav` home (the 50 Hz cells are B5 rows above) | 2 | 6 |
| B4 rerun with the final footprint code: 3 regression missions, 4 base-only strategy cells | 7 | 29 |

Definitions used below. **3D clearance** is the minimum distance between the robot's collision primitives
(wheels, chassis, arm links, panel) and the known boxes over the run, from the Unity bag (`clearance_3d`);
the MPC enforces 0.05 m on these primitives. Its uncertainty is about 1 cm at most: each joint-state
sample is paired with the latest base transform (not one interpolated to its stamp), the samples are
decimated (`--period`), and the base is taken as level (the model's planar base pose, no tilt).
**Misses** are cycles over the period (20 ms), **near-misses**
cycles over 80 % of it. **Slowed share** is the fraction of the 50 Hz command samples with
|`/cmd_vel_smoothed`| above 0.05 whose `/cmd_vel` (after the collision monitor) was scaled to 0.4-0.6 of it
(the slowdown zone's factor is 0.5; no other scale occurred). Cells show the median, with the range in
parentheses when there are several runs; time and final error use successful runs. The cited baseline rows
were not rerun: Phase 1 B1/B2 are the [2026-10-04 full rerun](../nav2-navigation/README.md), B3 the
[B3 missions](../moveit-arm/README.md), B4 the [B4 sections](../dynamic-footprint/README.md).

### Non-successes

Every run that did not end `succeeded` (B5 rows first) and what caused it. The only infrastructure failure
of the campaign is the first attempt of `b5-open_space_nav`; every other non-success is a result.

| Run | Outcome | Cause | Class |
|---|---|---|---|
| B5 `narrow_gate_vertical_carry_nav`, `185040Z` | canceled at the 90 s timeout, 2.44 m from the goal, 10 recoveries | The first global plan S-bent (0.20 m to the side at x = -5 m, 2.2 m before the gate line, back across the centre line); the replans and the path kept the base 0.066 m off centre. The collision monitor's stop zone saw the gate posts 0.6 m before the gate line (robot at x = -6.57 m) and zeroed the command (11 stop activations) while the MPC kept asking 0.30 m/s or turning at 0.4 rad/s; the base reached x = -6.57 m at about 15 s of the 90 s run and stayed there (shifts of at most 0.015 m) for the remaining 75 s. The MPC's collision slack stayed 0 (3D clearance 0.097 m) | result |
| B5 `worker_late_crossing_nav` x3 | canceled at the 90 s timeout, 3.8 m from the goal, 23-30 recoveries | Deadlock with the worker (below): the robot stopped with its footprint touching the worker's box, the worker was blocked in the lane for 83-85 s (walked 2.83-2.93 of 5 m) | result |
| RPP/DWB/MPPI `overhead_course_nav` x9, and the worker version x8 (MPPI 1/3 passed) | aborted (error 105; one DWB run 103) after 11 recoveries, 70-128 s | The beam is outside the lidar's 5 m range at the start, so the first plan runs straight through the gate and under it (RPP bag: 248 poses). At x = 11 m the lidar sees a wall across the lane in the 0.05-2.0 m band, the replan is a 526-pose route back out of the lane and round the south fence, and the robot stays at x = 11.1-12.1 m for about 100 s (the one MPPI success excepted); in the first nav run of each controller its yaw stayed within 0.17 rad, so the turn the detour needs never started (for RPP consistent with its known in-place-turn deadlock; DWB and MPPI not diagnosed). One MPPI run drove the detour (32.4 m, 177 s) | result |
| B3 `overhead_course_mission` failed 2 of 3, `overhead_course_worker_mission` 3 of 3; B4 `overhead_course_mission` 2 of 3 | 5 aborted on the last leg (home footprint, past the beam; error 105, 12-13 recoveries, 1.1-1.3 m of path, 85-105 s); 1 B3 run aborted on the gate leg (vertical carry, error 104, 11 recoveries, 2.6 m from the waypoint); 1 B3 worker run reached the goal position (0.142 m) and aborted with error 104 at 0.182 rad of heading error | The B1-B4 costmap band sees the beam as a wall across the lane although the home panel passes 0.135 m under it. The five missions that passed (B3 1, B4 1 and 3 of the worker version) took 6-12 recoveries on that leg. Why the leg passes in some runs was not investigated; the gate-leg abort is the known throat, the heading abort the goal-heading limitation | result |
| DWB `worker_late_crossing_nav` x3 | aborted (error 103) after 10-11 recoveries, 36-37 s, 4.0 m from the goal | Stopped at x = 11.95 m while the worker crossed (it finished, 0.14-0.20 m clearance) and gave up | result |
| MPPI `worker_stops_in_lane_nav` 1 | `off_goal` (0.72 m) | Stock MPPI passes the goal inside the tolerance (Phase 1) | result |
| B4 `narrow_gate_mission` 1 | failed in step 3 (the vertical-carry drive to the throat), error 104, 13 recoveries, 2.4 m from the waypoint | The known 1.05 m throat abort (also 2/3 on 2026-10-04) | result |
| B5 `open_space_nav`, attempt 1 (`183439Z`) | no summary | `lifecycle_manager_global_planning` did not bring up its nodes after two relaunches; the second attempt's log ends mid-launch when the driver was restarted; the three kept runs started later | infrastructure |

Six runs (two `wide_gate_vertical_carry_nav`, one `static_obstacle_detour_nav`, two
`worker_late_crossing_nav`, one `worker_stops_in_lane_nav`) began with the runner's own "arm deflected near
home/vertical_carry; starting a new Play epoch" recovery of the arm left by the previous run
(`recovered_by_new_epoch` in the summary); none had contact and their results are in the tables.

### Driving scenarios, B5 against B1/B2

Eight Phase 1 scenarios, B5 unscripted at 50 Hz, three runs each; B1 (DWB) and B2 (MPPI) and the RPP
bring-up are the 2026-10-04 rows. B5 **23/24, no contact**, against RPP 22/24, DWB 22/24 and MPPI 12/24 (no
contact in any).

| Scenario | B5 success | B5 time s | B5 MPC p99 ms | B5 misses / cycles | B5 3D clearance m | RPP | DWB | MPPI |
|---|---|---|---:|---:|---|---|---|---|
| open_space_nav | 3/3 | 16.7 (16.7-16.8) | 11.0 | 3 / 2506 | 2.517 | 3/3, 15.6 | 3/3, 15.7 | 3/3, 14.4 |
| narrow_gate_home_nav | 3/3 | 37.7 (37.1-37.7) | 12.4 | 20 / 5593 | 0.105 (0.085-0.109) | 3/3, 44.4 (detour) | 3/3, 45.2 | 0/3 |
| narrow_gate_vertical_carry_nav | 2/3 (canceled 1) | 38.0 (37.4-38.6) | 11.7 | 36 / 7948 | 0.104 (0.097-0.128) | 2/3, 26.7 | 3/3, 29.1 | 3/3, 27.0 |
| wide_gate_home_nav | 3/3 | 32.5 (32.5-32.8) | 11.0 | 2 / 4876 | 0.132 (0.129-0.138) | 3/3, 44.8 (detour) | 1/3, 43.1 | 0/3 |
| wide_gate_vertical_carry_nav | 3/3 | 32.5 (32.3-32.7) | 10.9 | 6 / 4865 | 0.133 (0.132-0.136) | 3/3, 26.6 | 3/3, 27.3 | 3/3, 27.1 |
| static_obstacle_detour_nav | 3/3 | 43.5 (43.5-43.6) | 11.9 | 11 / 6511 | 0.429 | 2/3, 33.5 | 3/3, 34.6 | 2/3, 31.7 |
| worker_crossing_nav | 3/3 | 32.2 (32.1-32.3) | 10.9 | 5 / 4817 | 2.514 | 3/3, 30.3 | 3/3, 39.5 | 1/3, 38.4 |
| persistent_blockage_nav | 3/3 | 50.8 (48.2-51.4) | 10.6 | 16 / 7496 | 0.045 (0.042-0.104) | 3/3, 41.2 | 3/3, 40.6 | 0/3 |

- **B5 chooses the arm itself and takes the gates the baselines cannot.** In `narrow_gate_home_nav` and
  `wide_gate_home_nav` (home start) B5 raised the panel to vertical carry on the way (shoulder pan 1.57 rad,
  `wrist_2` 1.59 rad in every run), drove the 4.65 m straight line through the gate and finished in 37.7 and
  32.5 s, against the 11 m detours of RPP and DWB (44-45 s) and MPPI's `off_goal` 0/3. Where the panel is
  already in vertical carry, RPP, DWB and MPPI are 5-11 s faster than B5 (below). In `open_space_nav`,
  `static_obstacle_detour_nav`, `worker_crossing_nav` the arm stayed in home (all six joints within 0.02 rad
  of 0).
- **The blocked gate** (`persistent_blockage_nav`): B5 drove the 10.75 m detour in 50.8 s (RPP 41.2 s, MPPI
  0/3) and passed the approach wall with the panel at 0.042-0.045 m in two runs, below the margin (finding 2).
- **Final errors** are 0.098-0.149 m (finding 5), cross-track p95 0.004-0.074 m, no controller error or loop
  miss; the controller process used 27-44 % of a core (RPP 6-8 %, DWB 19-22 %, MPPI 20-45 %).
- **Timing** is in a later section; the collision monitor's slowdown zone halved B5's command for 79-93 %
  of the moving time in the gate drives (finding 1), which accounts for part of the time difference to the
  baselines.

### Missions and the scripted control row

B5 drives the three B3 missions as one navigate task from the mission's start to its goal, with the arm
moving while the base drives (no stop at the reconfiguration waypoints); B3 and B4 stop at each waypoint,
reconfigure with MoveIt and drive again, so times are not like for like. The control row gives B5 the
scripted panel references of the B3 poses (`--references scripted`).

| Mission | B5 unscripted | B5 scripted | B3 RPP / DWB / MPPI (10-04) | B4 (10-04) | B4 (10-11) | B4 (this campaign) |
|---|---|---|---|---|---|---|
| narrow_gate_mission | 3/3, 48.6 s (48.6-48.7) | 3/3, 46.3 s (46.0-46.4) | 3/3 63.0 / 1/3 111.1 / 2/3 57.8 | 2/3, 57.1 | 3/3, 58.0 | 2/3, 60.2 (58.7-61.8) |
| wide_gate_mission | 3/3, 46.8 s (46.5-46.9) | 3/3, 42.8 s (42.7-43.0) | 3/3 57.2 / 3/3 71.5 / 2/3 59.9 | 3/3, 56.4 | 3/3, 57.2 | 3/3, 57.1 (56.4-62.2) |
| constrained_reconfiguration_mission | 3/3, 17.7 s (17.6-17.8) | 3/3, 14.7 s (14.4-15.1) | 3/3 28.4 / 3/3 28.8 / 1/3 35.2 | 3/3, 27.4 | 3/3, 29.5 | 3/3, 28.8 (27.5-29.0) |

| Mission | Row | MPC p99 ms | Misses / cycles | 3D clearance m | Cross-track p95 m | Largest JTC error rad | Unity arm checks |
|---|---|---:|---:|---|---|---:|---|
| narrow_gate | unscripted | 11.6 | 12 / 7286 | 0.109 (0.108-0.113) panel, ApproachWallRight | 0.005 | 0.028 | 3/3 pass |
| narrow_gate | scripted | 13.5 | 11 / 6919 | 0.154 (0.153-0.155) wheels, gate posts | 0.005 | 0.034 | 3/3 |
| wide_gate | unscripted | 10.8 | 14 / 7003 | 0.151 (0.147-0.154) | 0.004 | 0.050 | 3/3 |
| wide_gate | scripted | 14.9 | 20 / 6405 | 0.277 (0.277-0.283) | 0.006 | 0.037 | 3/3 |
| constrained | unscripted | 12.8 | 7 / 2647 | **0.043** (0.042-0.045) panel, tall_box | 0.017 | 0.023 | 3/3 |
| constrained | scripted | **28.4** (20.1-36.7) | 81 / 2180 | **0.033** (0.030-0.034) panel, tall_box | 0.086 | 0.033 | 3/3 |

- **9/9 unscripted and 9/9 scripted missions succeeded, no contact**, all Unity arm checks passed (panel
  bottom at least 0.60 m, base tilt at most 0.14 degrees, JTC path error at most 0.050 rad against the
  0.15 rad tolerance).
- **Unscripted B5 against the scripted references:** the same success; unscripted took 2.3-4.0 s longer on
  every mission, kept less clearance at the gates (0.109/0.151 against 0.154/0.277 m) and chose
  intermediate arm configurations (the scripted row moves `elbow` and `wrist_3` through poses of its own:
  `elbow` -0.55 to 0.34 rad). On the constrained mission it passed the tall box at 0.043 m with a partial
  rotation of the panel (shoulder pan 1.12-1.15 rad, `wrist_2` 1.2 rad, `wrist_3` -0.23 rad), the scripted
  row at 0.030-0.034 m with collision slack up to 0.19 m: the scripted references push the panel into the
  constraint, the unscripted MPC reached the box with another configuration.
- **Against B3/B4** (cited): B5 completes each mission 8.5-11.6 s (narrow), 9.6-10.4 s (wide) and
  9.7-11.8 s (constrained) faster than B4 in its three series (2026-10-04, 2026-10-11 and this campaign), and
  14.4, 10.4 and 10.7 s faster than B3 RPP, without the standstills of the staged reconfigurations and with
  the same zero contact.

### Worker scenarios

Both new scenarios on the open worker_crossing_nav route (start (16, 0), goal (8, 0)); the baselines are
RPP, DWB, MPPI and B4 (RPP with the dynamic footprint), three runs each. "Mover clearance" is the smallest
distance between the home footprint polygon and the worker's box over the run.

| Scenario | Controller | Success | Time s | Mover clearance m | Recoveries | Contact |
|---|---|---|---|---|---|---|
| worker_late_crossing_nav | RPP | 3/3 | 37.5 (37.3-37.6) | 0.19 (0.18-0.19) | 6 | 0 |
| | B4 (RPP + dynamic) | 3/3 | 37.6 (37.4-37.7) | 0.20 (0.20-0.20) | 6 | 0 |
| | DWB | 0/3 (aborted, error 103) | - | 0.18 (0.14-0.20) | 11 | 0 |
| | MPPI | 3/3 | 39.8 (39.4-40.6) | 0.15 (0.12-0.15) | 1 (1-3) | 0 |
| | **B5** | **0/3** (canceled at 90 s) | - | **0.00** | 28 (23-30) | 0 |
| worker_stops_in_lane_nav | RPP | 3/3 | 45.5 (44.7-49.5) | 0.18 (0.15-0.19) | 1 (1-5) | 0 |
| | B4 (RPP + dynamic) | 3/3 | 34.2 (33.6-45.9) | 0.36 (0.26-0.42) | 1 (0-1) | 0 |
| | DWB | 3/3 | 45.6 (35.3-54.3) | 0.21 (0.11-0.33) | 0 (0-1) | 0 |
| | MPPI | 2/3 (off_goal 1) | 38.7 (37.0-40.5) | 0.28 (0.27-0.36) | 0 | 0 |
| | **B5** | 3/3 | 43.3 (42.7-43.4) | 0.14 (0.07-0.15) | 0 | 0 |

- **`worker_late_crossing_nav` (B5 0/3): a deadlock with the worker, not a collision.** The scenario is
  timed so that a robot that does not react would collide (the worker reaches the lane with the robot's
  front 0.29 m from it). All five controllers were at the same place at the same time after the worker's
  trigger: x = 12.97-13.03 m at +0 s, 12.41-12.48 m at +2 s, 11.97-12.05 m at +4 s (so the slowdown zone
  is not what distinguishes B5). RPP, DWB, MPPI and B4 then held at x = 11.94-12.00 m (0.12-0.20 m from the
  worker's box) until the worker had passed. B5 did not hold: it kept creeping at about 0.1 m/s to
  x = 11.81-11.86 m by +6 s, into the worker's path, while the worker crossed. The worker walked 2.83-2.93
  of 5 m (about +5.8 s), stopped at y = -0.33 to -0.43 m inside the lane and was blocked there for 83-85 s
  by the worker's wait rule, while the robot's home footprint polygon touched the worker's box (clearance
  0.00 m; no physics contact was recorded). Nothing moved for the rest of the run: the MPC asked
  0.05-0.10 m/s and up to 0.4 rad/s and the stop zone zeroed it (stop-zone share of the moving command
  0.11, 0.21 and 0.12). The cause of the advance was not isolated; the candidates are the MPC's
  prediction of the worker (a constant-velocity or static box) and the route-progress term pulling it
  forward. The three runs differ in how the MPC was left: runs 1 and 3 had sustained collision slack
  (up to 0.14 and 0.11 m over 406 and 370 cycles; run 1 also 22 braking cycles, one fallback plan and two
  failed cycles; p99 26.4 and 25.5 ms, up to 500 OSQP iterations, misses 3.1 % and 9.4 %), while run 2 had
  slack 0.33 m in only 2 cycles, all 4215 cycles `ok`, p99 17.9 ms and 0.6 % misses: there the stop zone
  held the robot, not the MPC. Moving obstacles are a Phase 5 trigger (spec Sections 11 and 12).
- **`worker_stops_in_lane_nav`** passed 3/3: B5 detoured round the standing worker (path 8.31-8.38 m, 43.3 s)
  with the arm between home and vertical carry (shoulder pan up to 0.92-1.31 rad) and 0.07-0.15 m from it,
  closer than RPP and DWB (0.15-0.33 m); MPPI 2/3.
- **B4 in these scenarios:** the same success as RPP (3/3 and 3/3), no footprint publish in the late-crossing
  drives and 2-3 grows in the stops-in-lane drives (B4 rerun below).

### Overhead course

Drive versions (vertical carry start for B1/B2; B5 starts in home): 1.05 m gate at x = 12 m, a beam with its
bottom at z = 1.70 m at x = 15.5 m across the lane, a 90 degree turn round the fence end; the worker version
adds a worker stepping onto the route after the beam. B3/B4 drive the mission versions (vertical carry for
the gate, home for the beam).

| Scenario | RPP | B1 (DWB) | B2 (MPPI) | B3 (RPP) | B4 (RPP + dynamic) | B5 |
|---|---|---|---|---|---|---|
| overhead_course (drive / mission) | 0/3 | 0/3 | 0/3 | 1/3 (81.0 s) | 1/3 (80.4 s) | **3/3**, 74.9 s (74.3-76.0) |
| overhead_course_worker | 0/3 | 0/3 | 1/3 (177 s, 32.4 m detour) | 0/3 | 3/3, 85.1 s (83.9-94.8) | **3/3**, 75.7 s (74.1-75.8) |

B5 detail: path 13.02-13.21 m, final error 0.148-0.149 m, cross-track p95 0.041 (0.040-0.043) and 0.054
(0.047-0.070) m, MPC p99 13.7 and 12.8 ms (22 / 11228 and 18 / 11261 misses), no recoveries, controller
process 39.5-44.3 % of a core, worker clearance 0.66 (0.66-0.69) m (B4 0.46-0.47 m). **3D clearance to the beam
0.039 m (0.038-0.043) and 0.041 m (0.040-0.042), below the 0.05 m margin in all six runs** (finding 2).

- **How B5 passed.** It raised the panel to vertical carry for the gate (shoulder pan 1.57 rad), lowered it
  after the gate (shoulder lift down to -0.31 rad and elbow to -0.27 rad, a configuration that is neither
  named pose) and passed under the beam while the panel was still coming down: at the minimum clearance
  instants of the two runs sampled the shoulder pan was 1.06 and 1.24 rad and `wrist_2` 0.86 and 0.99 rad
  (home is 0 rad), so the beam was passed in transit, not in home (the scenario comment gives 0.135 m under
  the beam for a panel in home). The mean speed was 0.17 m/s over the run; the slowdown zone was active for
  66-70 % of the moving time.
- **B1/B2/B3/B4 fail by construction** (Non-successes): the course has one way for them, under the beam,
  which their costmap band forbids, and the only alternative is a 32 m detour round the whole lane.
- The worker (version 2) crossed 0.66 m from B5 and cost it nothing: 75.7 s against 74.9 s.

### Timing at 20 and 50 Hz

`wide_gate_mission` and `overhead_course_nav` (home), three runs at each rate; everything but the rate is
the same. The cycle is the node's whole cycle.

| Cell | Rate | Period ms | p99 ms | Median ms | Misses / cycles | Near-misses | MPC CPU % of a core | Time s | 3D clearance m |
|---|---|---:|---|---:|---:|---:|---:|---|---|
| wide_gate_mission | 20 Hz | 50 | 13.7 (10.3-15.3) | 7.6 | 0 / 2795 | 0 | 18 | 46.7 (46.4-46.7) | 0.157 (0.155-0.161) |
| wide_gate_mission | 50 Hz | 20 | 10.8 (10.6-11.9) | 5.2 | 14 / 7003 | 23 | 34 | 46.8 (46.5-46.9) | 0.151 (0.147-0.154) |
| overhead_course_nav | 20 Hz | 50 | 24.4 (20.7-26.3) | 8.8 | 15 / 4475 | 19 | 22 | 75.0 (74.0-75.3) | 0.044 (0.043-0.046) |
| overhead_course_nav | 50 Hz | 20 | 13.7 (13.1-14.6) | 8.1 | 22 / 11228 | 53 | 44 | 74.9 (74.3-76.0) | 0.039 (0.038-0.043) |

- **The outcome does not depend on the rate**: the same success, time (within 0.2 s) and path; 3D clearance
  differs by 0.004-0.006 m (20 Hz keeps slightly more).
- **Cycles are not shorter at 20 Hz** (median 7.6 against 5.2 ms and 8.8 against 8.1 ms; p99 13.7 against
  10.8 ms and 24.4 against 13.7 ms), so the slower rate buys CPU share (18-22 % against 34-44 % of a core)
  and period (p99 27-49 % of the period at 20 Hz, 54-69 % at 50 Hz), not cycle time. The host may be
  clocking down between sparse cycles; I did not measure it.
- **p99 against the period at 50 Hz over the whole campaign:** 49 of 54 runs have p99 under 20 ms (cell
  medians 10.6-14.9 ms; the slowest run outside the two cells below, 19.7 ms, is the canceled
  `narrow_gate_vertical_carry_nav`). The five that exceed it are two of the three
  `worker_late_crossing_nav` runs (26.4 and 25.5 ms; the third 17.9 ms) and the three scripted
  `constrained_reconfiguration_mission` runs (20.1, 28.4, 36.7 ms, although all three succeeded): runs with
  sustained collision slack (up to 0.14 m and 0.19 m) and 400-500 OSQP iterations. Per-run miss rates in the
  other 48 runs (the 54 at 50 Hz less the six of these two cells) were at most 0.82 % (0.81 % in a
  `narrow_gate_home_nav` run, 0.82 % in the canceled
  narrow-gate run, 0.52 % in a `worker_stops_in_lane_nav` run and in a scripted `wide_gate_mission` run); in
  the two cells they were 0.6-9.4 % (late crossing) and 1.1-6.2 % (scripted constrained). No run
  re-anchored the arm (`arm_reanchors` 0 throughout) or needed more than 250 OSQP iterations outside those
  two cells.
- **The R26 conditions for 50 Hz were not all met in the campaign.** Ruling R26 chose 50 Hz if p99 < 20 ms,
  misses < 1 %, no run inside the 0.05 m margin and joint tracking within 0.10 rad. In the campaign p99
  and misses failed in the two cells above, 17 of 60 B5 runs were inside the 0.05 m margin (finding 2), and
  20 Hz does not repair the margin (the three 20 Hz `overhead_course_nav` runs were at 0.043-0.046 m).
  Tracking held (largest JTC error 0.050 rad). Whether to keep 50 Hz, change the rate or change the
  margin handling is left to the controller and user; it is an open point for Phase 4B.

### Findings

1. **The collision monitor's slowdown zone halves B5's command near obstacles, which the MPC's model does not
   know.** The command after the monitor (`/cmd_vel`) was scaled by 0.5 (never another factor) for the
   slowed share of the moving time below; in those cycles the base moved at 0.46-0.47 of the speed the MPC
   predicted for its next stage (median over cycles with a plan speed of at least 0.1 m/s; 0.88-0.93 when
   unscaled), so the plan ran about twice as far ahead of the base as it assumed.

   | Cell | B5 slowed share | B4 slowed share (same drive) |
   |---|---:|---:|
   | open_space_nav | 0 | - |
   | narrow_gate_home_nav (B5 through the gate, B4 detours) | 0.93 | 0.06-0.08 |
   | narrow_gate_vertical_carry_nav | 0.92-0.93 | 0.58 |
   | wide_gate_home_nav / wide_gate_vertical_carry_nav | 0.79-0.80 / 0.79 | 0.01-0.05 / 0.52 |
   | static_obstacle_detour_nav / persistent_blockage_nav | 0.50 / 0.30-0.42 | - |
   | worker_crossing_nav / worker_stops_in_lane_nav | 0.06 / 0.48-0.52 | - |
   | narrow / wide / constrained mission (unscripted) | 0.60 / 0.54-0.55 / 0.53 | 0.19-0.42 / 0.36 / 0.41-0.44 |
   | overhead_course_nav / worker (home) | 0.66-0.70 / 0.66-0.70 | - |

   The effect is on time and on the B4/B5 comparison: in the vertical-carry gate drives B5 was 8-12 s
   slower than B4 (38.0 against 26.0 s narrow, 32.5 against 24.9 s wide), with 79-93 % of its moving time
   slowed against 52-58 % for B4, whose zone follows the footprint. B5's slowdown zone is the swept hull of
   its own plan, and the MPC does not see it. Stop activations (zeroing the
   command) happened in B5 only in the runs that failed (finding 6 and the late-crossing deadlock): 11, and
   3, 23 and 4 per run. Not measured: how the mismatch changes the plan's tracking or margins (a run with
   the zone off would).
2. **3D clearance fell below the 0.05 m margin in 17 of 60 B5 runs, without contact.** Unscripted 50 Hz: 11 of
   45 (`persistent_blockage_nav` 0.045 and 0.042 m to ApproachWallLeft; `constrained_reconfiguration_mission`
   0.043, 0.045, 0.042 m to the tall box; `overhead_course_nav` 0.043, 0.039, 0.038 m and
   `overhead_course_worker_nav` 0.041, 0.042, 0.040 m to the beam); scripted constrained 0.033, 0.034,
   0.030 m (the campaign minimum); 20 Hz overhead 0.046, 0.044, 0.043 m. No unscripted run was below
   0.038 m. The MPC's collision rows hold at the commanded reference: the largest slack was 0.05 mm in these
   runs (the reference was 0.0499 m or more from the box), while the arm sags below its reference. At the
   minimum-clearance instants of four sampled runs (two overhead, one tall box, one blocked gate) the JTC
   error was 0.016-0.018 rad on `shoulder_lift` and 0.008-0.009 rad on `elbow` under the beam, and
   0.004-0.009 rad on three to four joints at the tall box and the blocked gate: of the order of the 1.5 cm
   hold sag of the qualification (0.015 rad on `wrist_2`), and the size of the 0.005-0.012 m shortfall
   against the margin. I did not compute the panel displacement with forward kinematics. A margin that holds
   for the measured arm needs the reference-to-measured offset in the rows (or 1.5-2 cm more margin) before
   Phase 5.
3. **B1/B2 on the overhead course start with the beam outside the lidar's range and plan under it first**:
   the RPP bag's first plan is a straight 248-pose route under the beam, and 17 of the 18 B1/B2 runs
   never got beyond x = 12.1 m (the gate posts occupy x = 11.8-12.2 m), where the replan (526 poses, round the south
   fence) needed a turn in the 2.4 m lane that 17 of the 18 runs did not complete (the one MPPI success
   drove it; one DWB run turned and aborted at x = 5 m). For the baselines the course tests the reaction to
   a late-discovered obstacle in a lane;
   B5 never needed the lidar for the beam (the scenario boxes are in its known world, commit `3001b86`), so
   B1/B2 against B5 here compares knowledge as much as control. The B3/B4 mission versions, which know the
   beam only from the same lidar band, behave alike (5 of 12 passed).
4. **The Nav2 route enters the 1.05 m gate off centre** (Smac Lattice S-bend, Phase 1 "Throat plans"): the
   first plan of two of the three `narrow_gate_vertical_carry_nav` runs was 0.20 m to one side at x = -5 m
   (2.2 m before the gate line) and 0.04-0.05 m to the other side at the posts; the base was 0.126 and
   0.106 m off centre at x = -5 m and 0.066 and 0.027 m at the gate line. The run that stayed 0.066 m off
   centre is the one the stop zone held; the one that corrected to 0.027 m passed (the fix round's failing
   run was 0.05 m off). The other narrow-gate B5 paths (`narrow_gate_home_nav`, `narrow_gate_mission`,
   scripted; nine runs) stayed within 0.007 m of the centre, as did B4's five vertical-carry drives. The
   centring is a planner property that depends on the start pose to the millimetre, not an MPC one, and the
   MPC cannot correct a route that the stop zone refuses.
5. **Final errors sit just inside the tolerance**: 51 of the 56 successful B5 runs finished 0.145-0.150 m from
   the goal (median 0.148 m, range 0.098-0.150 m; yaw at most 0.137 rad), so the base stops as soon as the
   goal checker's 0.15 m tolerance is reached (the tapered approach of fix round 1 arrives at 0.02-0.08
   m/s). The accuracy is the tolerance, not better; the three runs below 0.13 m (0.098-0.129 m) are
   `persistent_blockage_nav`.
6. **Unscripted B5 stopped at the narrow gate once in the campaign** (`narrow_gate_vertical_carry_nav`, 1 of
   9 unscripted narrow-gate runs; 0 of 3 scripted; the fix round's 20 Hz failure had the same cause): not
   by the MPC (its constraint held, 3D clearance 0.097 m, slack 0) but by the collision monitor's stop zone
   (a 2D check of the base polygon plus margin against lidar points) on an off-centre route (finding 4). It
   is a Phase 5 input: the safety layer and the MPC disagree on what is free, and the MPC cannot tell it is
   being held (it kept commanding 0.30 m/s, or turning, for the remaining 75 s). B5 used configurations other than home and vertical
   carry in 16 of the 45 unscripted 50 Hz runs (9 stayed in home, 20 used vertical carry): the late
   crossing (shoulder pan up to 1.64-1.69 rad, past vertical carry, with tightness 1.0 at the stop), the
   stops-in-lane runs (0.92-1.31 rad), the constrained mission (1.12-1.15 rad), the six overhead runs (a
   lowered pose, shoulder lift -0.31 rad and elbow -0.27 rad, and the mid-descent under the beam, finding 2)
   and one blocked-gate run (shoulder lift -0.10 rad). The gate, detour and open runs used exactly the two
   named poses. The choices that mattered were all near obstacles (the beam transit at 0.038-0.043 m, the
   tall-box pass at 0.042-0.045 m, the worker deadlock): the cases for a longer horizon and for predicting
   moving obstacles in Phase 5.

### Exit criteria (spec Section 9)

| Criterion | Met |
|---|---|
| B5 drives free-drive goals from RViz, including goals behind the robot and new headings | **Met**, verified in Task 15 (4/4 free-drive goals, including one behind the robot); not repeated in the campaign |
| Every cell recorded, with contact and failures reported as such | **Met**: 45 cells, 143 of 143 runs (one infrastructure attempt repeated), zero contact, all non-successes listed and classified above; unscripted failures are reported (B5 `worker_late_crossing_nav` 0/3, one gate run) and not tuned away |
| p99 cycle time within the period at the chosen rate (target 50 Hz) | **Not met in 2 of 45 cells**: `worker_late_crossing_nav` (p99 25.5-26.4 ms in two runs) and the scripted `constrained_reconfiguration_mission` control row (p99 20.1-36.7 ms, though it completes 3/3); in the other 50 Hz B5 cells every run has p99 under 20 ms (cell medians 10.6-14.9 ms; 49 of 54 runs overall). The R26 rate conditions as a whole were not met either (see Timing) |
| Documentation in spec Section 11 | **Met**: this README (prerequisites, qualification, runs, timing), [ADR 0011](../../adr/0011-wholebody-mpc-command-ownership.md) (with notes in ADR 0006 and 0010), the roadmap's Phase 4 section, decision log and open decisions, and the `running-the-stack.md` B5 launch and free drive |

## Follow-up switches (2026-10-13)

First live runs of the follow-up switches (commits `b6dc8f1` to `60245fe`; spec Sections 3 and 5 amended
2026-10-13): `--no-slowdown-zone`, `--posture-mode`, `--comfort-pose` (only `home` exists; `carry` is not
qualified yet) and `--clearance-weight`. B5 at 50 Hz, unscripted, one run per row, clean tree, no host sleep,
real Unity. These runs are not like for like with the recorded baselines (roadmap comparability log).

The clearance weight was chosen at unit level (`Clearance.KeepsFartherFromABoxBesideTheRoute`): in closed
loop past a tall box 0.12 m beside a straight route the default B5 ends at the 0.049 m margin (the
tightness swings the panel towards the box), and weights 5, 10, 20, 50 and 100 keep 0.112, 0.141, 0.171,
0.202 and 0.220 m, with at most 1.6 cm of lateral deviation and no change in time. **W = 20** (the contour
weight) was used: noticeable, and it does not take over the route tracking.

| Run | Settings | Outcome | Time s | 3D clearance m | MPC p99 ms (misses), CPU | Slowed share | Arm (tools/arm_trace.py) |
|---|---|---|---:|---|---|---:|---|
| `narrow_gate_home_nav`, `080054Z` | recorded defaults | succeeded, 16 recoveries | 69.4 | 0.116 (wheel to gate post) | 7.0 (1), 26 % | 0.57 | home 5.0 s, vertical carry 46.2 s from t = 13 s, partly lowered at the end |
| `narrow_gate_home_nav`, `080556Z` | `comfort_only`, W 20, no slowdown zone | **contact** (panel with the right gate post, 2.6 mm), canceled | 17.0 | -0.015 | 30.2 (62), 50 % | 0.00 | home 7.6 s, then moving: shoulder lift -0.61 to 0.64, elbow to -0.51, `wrist_2` to -1.72 rad, shoulder pan within +-0.15 rad |
| `overhead_course_nav --arm-pose home`, `080752Z` | recorded defaults | succeeded, 0 recoveries | 76.2 | 0.037 (panel to the lane fence); beam 0.042 | 11.6 (6), 34 % | 0.70 | home 15.6 s, vertical carry 13.4 s (gate), pan 1.04 / `wrist_2` 1.05 rad 8.4 s, lowered (lift -0.29, elbow -0.25) 10.0 s, pan 0.74 rad 4.2 s |
| `overhead_course_nav --arm-pose home`, `081058Z` | `comfort_only`, W 20, no slowdown zone | **contact** (panel with `gate_post_north` at the 1.05 m gate, 1.8 mm), canceled | 24.7 | -0.082 | 32.9 (55), 48 % | 0.00 | home 14.6 s, then moving: lift -0.68 to 0.55, elbow to -0.51, `wrist_2` to -1.69 rad, pan within +-0.14 rad |
| extra: `narrow_gate_home_nav`, `081420Z` | `blend`, W 20, no slowdown zone | succeeded, 0 recoveries | **19.7** | **0.140** (wheel to gate post) | 17.9 (7), 50 % | 0.00 | home 4.2 s, vertical carry 5.4 s from t = 10 s |
| extra: `overhead_course_nav --arm-pose home`, `081547Z` | `blend`, W 20, no slowdown zone | succeeded, 0 recoveries | **50.3** | **0.121 (panel to beam)** | 14.8 (12), 47 % | 0.00 | home 22.0 s, vertical carry 4.6 s (gate), then transit; at the beam pan 1.22, lift -0.30, elbow -0.28, `wrist_2` 0.82 rad |

Campaign reference (same scenarios, recorded defaults): narrow gate 37.7 s (37.1-37.7), 3D clearance
0.085-0.109 m; overhead course 74.9 s (74.3-76.0), beam 0.038-0.043 m, slowdown zone active for 66-70 % of
the moving time. "Slowed share" as in the campaign (command samples scaled to 0.4-0.6 by the monitor). The
beam clearance of the overhead runs comes from `clearance_3d` with a beam-only boxes file; the
3D clearance column is the minimum over all known boxes.

- **The defaults reproduce the recorded B5** (the defaults then, before `fb1bc0d` made `carry` the comfort
  pose and the slack gate came on; now `--comfort-pose home --no-slack-gate`). The overhead run matches the campaign in time (76.2 s), arm
  (home, vertical carry for the gate, the lowered lift/elbow pose, under the beam mid-descent at pan 1.08,
  `wrist_2` 0.89 rad) and beam clearance (0.042 m, again below the margin). The narrow-gate run made the
  same arm choice and raised the panel at the same time as the campaign runs (`wrist_2` past 1.5 rad at
  20.7 s against 20.4-21.1 s), but took 69.4 s: its first global plan S-bent 0.20 m off the centre line
  2.2 m before the gate (the three campaign plans were centred), and the collision monitor's stop zone held
  the base at the gate from 15 to 51 s until a replan straightened the route. This is the campaign's known
  stop-zone failure mode (finding 4), upstream of the changed code.
- **`comfort_only` cannot pass a 1.05 m gate.** Without the vertical-carry attractor the MPC tried another
  reconfiguration: it pitched the panel upright with the shoulder lift and `wrist_2` (panel pitch to -1.55
  rad), which keeps its 1.2 m width across the lane; only the shoulder-pan rotation of vertical carry
  narrows it. The slacked collision rows then let the plan into the posts (slack up to 0.51 and 0.53 m in
  225 and 242 cycles; the default runs had none) and the panel touched them at both gates; the QP was also
  slow (p99 30-33 ms, 55-62 misses). The unit closed loop through the gate predicted it (-0.15 m with
  `comfort_only`, 0.16 m with `blend`). `comfort_only` is usable only where the comfort pose fits.
- **The slowdown zone and the clearance cost (extra runs, `blend`).** Without the slowdown zone and with
  W = 20 B5 took 19.7 s through the narrow gate (campaign 37.7 s) and 50.3 s on the overhead course (74.9 s),
  with no recoveries and no monitor activations, which is consistent with the zone halving the MPC's command
  for most of the drive (ADR 0011). The clearance cost kept 0.140 m at the gate posts (campaign
  0.085-0.109 m) and 0.121 m under the beam, where every recorded run was below the 0.05 m margin
  (0.038-0.043 m): at the beam the arm was bent slightly more (lift -0.30, elbow -0.28 rad against -0.25 and
  -0.22) and the panel passed lower. Both switches changed together here, so the time gain cannot be
  attributed between them (the zone is the likely cause, the cost the likely cause of the clearance); the
  MPC used more CPU (47-50 % of a core against 26-34 %) and its p99 stayed under the 20 ms period.
- Two attempts of the extra runs failed before driving (infrastructure): after the second contact the arm
  was left deflected outside a qualified pose and MoveIt could not plan back; both were repeated after a new
  Play epoch.

## carry pose qualification (2026-10-13)

The bent-arm level carry `carry` = `[0, 0.968, -1.735, 0, 0.767, 0]` rad (found by `carry_pose_search`,
commit `3a694ea`; registered in `c77f4fb`): `wrist_2 = -(shoulder_lift + elbow)` keeps the panel level,
panel top 1.320 m (home 1.565 m), panel centre over the shoulder axis, the home footprint (1.24 x 1.24 m),
manipulability 0.023 m³ (home 0). Qualified as the poses and combined-motion sections above: same Editor,
scene, open test area, JTC configuration and tool, `--prefix carry`, 3 runs per row, 36 runs in all, every
one recorded by the Unity arm recorder (results in [arm-qualification.json.gz](arm-qualification.json.gz),
suite `carry`). Reproduce with
`python3 tools/run_wholebody_arm_qualification.py carry --prefix NEW [--parts hold transitions stream]`
(`--table` prints the per-run judgement of a prefix).

Acceptance as in Task 2: path error <= 0.10 rad, no watchdog flag, base tilt below 3 degrees, panel bottom
at least 0.15 m above the floor, no panel penetration, and (`qualified_payload.json`) hold error <= 0.06 rad.

- **Hold**: home -> carry with the 8 s reference cubic, then held 20 s: still, and with
  `--disturbance compact|extended --drive-during-motion` (0.3 m/s and 0.4 rad/s; 0.15 m/s and 0.2 rad/s).
- **Transitions at the `wholebody_arm` limits**: carry <-> vertical_carry and carry <-> home as one
  synchronized cubic of `--duration` 4.47 s (segment 4.37 s), 2 s hold, still and with the compact
  sequence started at the goal. The largest move in all four legs is the elbow's 1.735 rad, which peaks at
  1.5 x 1.735 / 4.37 = 0.596 rad/s and 6 x 1.735 / 4.37² = 0.545 rad/s² (the duration allows 0.01 rad of
  hold sag at the start so the measured peak stays at 0.60 rad/s; measured reference peak 0.595-0.598
  rad/s). The other joints move less in the same time (pan and `wrist_2` 1.571 and 0.804 rad to
  vertical_carry, shoulder lift 0.968 rad), so only the elbow runs at the velocity limit; no leg reaches
  the 0.61 rad/s² acceleration limit (that needs a 1.57 rad move in 3.9 s, as Task 2's pan and `wrist_2`).
- **JTC streaming**: the Task 2 stream (overlapping 1.5 s segments of `q0 + 0.2 sin(2 pi 0.25 t)` on all six
  joints with the braking tail, 50 Hz, 20 s) about carry. All six joints in phase tilt the panel by up to
  0.6 rad; the offline geometry check of both extremes (`carry_pose_search --eval`) found no self-contact
  (arm-arm >= 0.051 m, arm-base >= 0.065 m, panel-base >= 0.53 m).

| Row (3 runs each) | Path error, rad (joint) | Hold error, rad | Ref. peak, rad/s | Jumps | Watchdog | Tilt max, deg | Panel bottom min, m | Base speed max, m/s | Result |
|---|---:|---:|---:|---|---:|---:|---:|---:|---|
| hold still 20 s | 0.0073-0.0077 (elbow) | 0.0075 | 0.329 | 0, 0, 0 | 0 | 0.03 | 1.274 | 0.02 | pass |
| hold, compact drive | 0.0074-0.0105 (elbow) | 0.0087 | 0.329 | 0, 0, 0 | 0 | 0.11 | 1.272 | 0.293 | pass |
| hold, extended drive | 0.0082-0.0118 (elbow) | 0.0087 | 0.329 | 0, 0, 0 | 0 | 0.10 | 1.273 | 0.159 | pass |
| carry -> vertical_carry, still | 0.0238-0.0242 (elbow) | 0.0151 | 0.598 | 0, 0, 0 | 0 | 0.04 | 0.714 | 0.02 | pass |
| vertical_carry -> carry, still | 0.0237-0.0278 (`wrist_2`, elbow) | 0.0075 | 0.596 | 1, 1, 1 | 0 | 0.05 | 0.709 | 0.02 | pass |
| carry -> home, still | 0.0231-0.0294 (elbow) | 0.0005 | 0.598 | 0, 0, 0 | 0 | 0.02 | 1.270 | 0.01 | pass |
| home -> carry, still | 0.0173-0.0213 (elbow) | 0.0076 | 0.596 | 0, 0, 0 | 0 | 0.00 | 1.274 | 0.00 | pass |
| carry -> vertical_carry, driving | 0.0228-0.0349 (elbow) | 0.0155 | 0.598 | 0, 0, 0 | 0 | 0.14 | 0.713 | 0.293 | pass |
| vertical_carry -> carry, driving | 0.0206-0.0261 (`wrist_2`, pan) | 0.0088 | 0.596 | 0, 1, 1 | 0 | 0.14 | 0.709 | 0.294 | pass |
| carry -> home, driving | 0.0244-0.0279 (elbow) | 0.0243 | 0.598 | 0, 0, 0 | 0 | 0.11 | 1.269 | 0.294 | pass |
| home -> carry, driving | 0.0143-0.0215 (elbow) | 0.0088 | 0.596 | 0, 0, 0 | 0 | 0.13 | 1.273 | 0.295 | pass |
| JTC stream 50 Hz about carry | 0.0236-0.0256 (`wrist_3`, elbow) | - | 0.314 | 1, 0, 1 | 0 | 0.06 | 0.665 | 0.02 | pass |

All 36 runs passed: largest path error 0.035 rad (limit 0.10), largest hold error 0.024 rad (limit 0.06;
it is home's, held while driving, as in Task 2; carry itself held within 0.009 rad), no watchdog flag, tilt
at most 0.14 degrees, no panel penetration, no drive saturation, largest wall gap between physics samples
0.106 s (command age at most 0.13 s). Every action ended with status 4 (succeeded).

- **Sag**: held in carry the elbow settles 7.5-8.5 mrad further bent and the shoulder lift 3-5 mrad forward
  (final `[0.001, 0.971, -1.743, 0.001, 0.767, 0.000]`), a net panel pitch of about -0.004 rad; Unity's panel
  bottom is 1.272-1.274 m against the nominal 1.280 m. The elbow carries the largest static torque of the arm
  in this pose (about 27 of its 90 N m, `carry_pose_search`) and has the largest error in most rows.
- **Position jumps** are the start-of-trajectory steps documented above, not streaming discontinuities: every
  vertical_carry -> carry jump is 0.0148-0.0150 rad, the `wrist_2` hold sag in vertical_carry (JTC starts an
  action at the measured state; one of the six runs had none). The two stream jumps (0.0107 and 0.0108 rad, just over the 0.01 rad threshold; the
  third run's largest step was 0.0063 rad) are most likely the first segment starting at the measured, sagged
  carry (elbow 8 mrad from the held reference) while the sinusoid already moves; the tool does not time-stamp
  jumps, so this was not confirmed. No jump occurred after a stream ended.
- **Streaming** about carry: 1001 segments at 50.05 Hz in every run, error against the sinusoid at most
  0.030 rad, settle error against the braking-tail end 0.009 rad, panel bottom at least 0.665 m (the 0.6 rad
  panel pitch). Task 2's stream about home had a larger path error (0.075 rad on `wrist_1`); here every
  joint stayed within 0.026 rad.
- Panel ground clearance: in carry 1.27 m; the transitions to vertical_carry pass 0.709 m (vertical_carry's
  own bottom, as in Task 2).

**Verdict: `carry` is qualified** for whole-body motion under the same envelope as `home` and
`vertical_carry` (base 0.3 m/s and 0.4 rad/s; `base_commands_tested.carry` in `qualified_payload.json`),
with transitions to and from vertical_carry and home at the `wholebody_arm` velocity limit. Not covered:
MoveIt (`reconfigure_panel --named carry`) transitions, which plan their own paths and were not run;
transitions to other poses than home and vertical_carry; and the acceleration limit on the elbow and
shoulder lift (exercised to 0.55 rad/s² and 0.30 rad/s²).

### Live B5 smoke runs with the carry comfort pose

After `fb1bc0d` (carry the default `comfort_pose`): B5 at 50 Hz, unscripted, `blend`, slowdown zone on,
no clearance cost (the defaults), each scenario started in carry with `--arm-pose carry`, one run each,
real Unity, clean tree. The node recorded `comfort_pose: carry` in `mpc.options`. Not like for like with the
recorded B5, which used straight `home` (roadmap comparability log).

| Run | Outcome | Time s | 3D clearance m | Beam clearance m | MPC p99 ms (misses), CPU | Slowed share | Arm (tools/arm_trace.py) |
|---|---|---:|---|---:|---|---:|---|
| `narrow_gate_home_nav --arm-pose carry`, `092132Z` | succeeded, 0 recoveries, no contact | 37.2 | 0.083 (panel to the gate's right approach wall) | - | 6.8 (2), 22 % | 0.93 | carry 4.0 s from t = 0, then raised (pan, `wrist_2`) to vertical carry by t = 14 s, held 13.8 s through the gate, then partly lowered (pan 1.12, lift 0.28, elbow -0.50, `wrist_2` 1.35 rad, tightness 0.55-0.72) to the goal |
| `overhead_course_nav --arm-pose carry`, `092322Z` | succeeded, 0 recoveries, no contact | 69.3 | 0.047 (panel to the lane fence `EastLaneFenceWest`, in the turn) | **0.134** | 13.9 (13), 40 % | 0.58 | carry to t = 13 s, vertical carry t = 18-31 s (gate), back in carry t = 38-45 s, under the beam at t = 46.8 s mid-way towards the turn's configuration (pan 0.32, lift 0.77, elbow -1.39, `wrist_2` 0.94 rad, panel pitch 0.32 rad, tightness 0.28), pan up to 1.05 rad in the turn, carry again at the goal |

Campaign reference (recorded defaults, straight home): narrow gate 37.7 s (37.1-37.7), 3D clearance
0.085-0.109 m, slowed share 0.57 in the follow-up rerun; overhead course 74.9 s (74.3-76.0), beam
0.038-0.043 m, slowed share 0.66-0.70, overall 3D clearance 0.037 m (panel to the lane fence) in the follow-up
rerun. Beam clearance from `clearance_3d` with a beam-only boxes file; slowed share as defined above.

- **The beam**: the course's beam (bottom 1.70 m) was sized for straight home, whose panel top (1.565 m)
  clears it by 0.135 m when level. With carry the level panel top is 1.320 m (0.38 m below the beam), but the
  MPC did not pass under it level: it had already started turning the panel (pitch 0.32 rad) for the fence
  ahead, which raises one edge. It still kept 0.134 m, against 0.038-0.043 m for the recorded B5 (which
  passed mid-descent from vertical carry, lift and elbow lowered), so the beam is no longer the binding
  obstacle; the closest approach is now the lane fence in the turn, 0.047 m (just below the 0.05 m margin,
  as the recorded B5's 0.037 m there).
- **Arm choice**: with the carry comfort pose the MPC made the same reconfiguration as with home (vertical
  carry for the 1.05 m gate, from carry via pan and `wrist_2` while the shoulder lift and elbow straighten)
  and came back to carry between the obstacles and at the goal. On the overhead course it returned to the
  comfort pose between the gate and the beam (t = 38-45 s); the recorded B5 passed the beam in a lowered,
  tilted configuration (shoulder lift -0.25 to -0.31 rad, elbow -0.22 to -0.27 rad).
- **Time**: the overhead course took 69.3 s (campaign 74.9 s) with a lower slowed share (0.58 against
  0.66-0.70); the narrow gate took 37.2 s, as the campaign, with the base slowed for 93 % of its moving
  samples. One run each, so neither difference is established; the slowdown zone (the swept hull of the
  predicted configurations) is the likely link between the arm pose and the time.

## Slack gate (2026-10-13)

The collision rows are slacked (`d + g Δx >= margin - s`, L1 100 + quadratic 1000) so the QP always has
a solution, also with the robot already inside the margin. With a finite penalty the QP also bought
progress or posture with slack: `worker_late_crossing_nav` (slack 0.11-0.14 m sustained, deadlock)
and `posture_mode comfort_only` at the 1.05 m gate (slack up to 0.51 m, contact with the posts) above.
The slack gate (spec Section 3 "Slack gate", ADR 0011 note; commits `2791866`, `8049b33`, `5158211`)
holds instead: a solved plan whose slack, on any row of a stage within its first 1.0 s, exceeds the
current violation of that row's pair (`max(0, 0.05 m - d₀)`, `d₀` the same primitive and obstacle at the
measured state, the obstacle where it is now) by more than 5 mm is not executed (review follow-up: for a
pair already inside the margin, by any amount beyond 0.1 mm of solver noise; with 5 mm on top of the
violation a unit closed loop with a tenth of the slack weights crept from 0.026 m into contact, test
`SlackGate.InsideTheMarginNeverGetsDeeper`); the base brakes, the
arm gets the stop's braking hold, the MPC plans again every cycle and resumes once its plans are clean
over the whole 2 s plan for 0.2 s. `--comfort-pose home --no-slack-gate` (with `--mpc-rate-hz 50`) reproduces
the recorded B5; `--no-slack-gate` alone still runs the `carry` default.

**Resume hysteresis (added during implementation).** With the first-second rule for resuming as well,
the unit closed loop through the gate (`comfort_only`, home, 50 Hz) alternated hold and track every few
cycles (91 holds in 60 s): from rest a plan's push-in lies beyond the first second, so it passes; one
step later the next linearization pulls it into the first second again. Each resumed step plus the
arm's braking hold (which coasts `v T / 2`, `T >= 0.5 s`) took the panel a few millimetres closer:
closest 0.030 m (margin 0.05 m). Requiring clean plans over the whole 2 s for 0.2 s left 2 holds and
0.049 m. `slack_gate_resume_horizon_s: 1.0` and `slack_gate_resume_dwell_s: 0` restore the
first-passing-plan rule.

Unit closed loops (`test_controller`, the generated model as plant with static friction, the map boxes):

| Case | Without the gate | With the gate |
|---|---|---|
| `comfort_only` (home) through the 1.05 m gate, 50 Hz, 60 s | passes the gate at -0.145 m (in the posts), slack 0.29 m within the first second | drives to the posts at the margin (panel pitched), holds at x = -7.63 m, resumes once (0.16 m), then holds for the rest; closest 0.049 m; holding 1959 of 3000 cycles in 2 holds |
| late worker (0.8 m/s, triggered at 1.0 m, Unity's wait rule), 50 Hz | reaches the goal, 0.049 m, slack 0.25 m | holds 92 cycles in 3 holds while the worker crosses, then reaches the goal; 0.049 m |
| robot 0.02 m beside a crate (inside the margin) | - | not gated (slack 0.027 m against a 0.03 m violation); drives away |
| default `blend` through the 1.05 m gate; the existing controller tests | - | no holding cycle; same results |
| funnel: two low walls closing in 1 cm/m per side, robot starting 0.026 m from them (inside the margin), a tenth of the slack weights, 50 Hz, 15 s (review follow-up) | drives in to -0.019 m (contact) | 5 mm on top of the violation: never holds, -0.019 m as without; violation-only rule: holds from the first cycle at 0.026 m |
| solver failure in a hold (comfort_only at the posts; late worker within the resume dwell; review follow-up) | - | was: `fallback` on the held plan (executed and streamed) and the dwell lost; now: keeps holding, K failures then braking, the dwell restarts |

Mover sweep (12 cells: worker speed 0.5/0.8/1.2 m/s, trigger 2.0/1.6/1.3/1.0 m, 50 Hz, 60 s, each with
and without the gate): with the gate the MPC held in 9 cells; it reached the goal in 7 cells (6 without),
removed the one contact (1.2 m/s at 1.3 m: -0.040 m without, 0.050 m with) and otherwise changed the
closest distance by at most a few millimetres. In 5 cells the robot and the worker deadlock with or
without the gate (6 without): the robot stops at the margin within the worker's 0.1 m wait distance and
the worker waits for it, so holding alone does not resolve that deadlock mechanism.

### Live runs with the slack gate

B5 at 50 Hz, unscripted, defaults (`blend`, `comfort_pose` carry, slowdown zone on, no clearance cost,
slack gate on) unless listed, one run each, real Unity, commit `7ba36d2`. These runs predate the review
follow-ups (no live rerun yet): the violation-only rule for pairs inside the margin, holding through a
solver failure (the `comfort_only` run went holding, fallback, holding at cycle 1557: the fallback
executed the held plan) and no pair distances with the gate off.

| Run | Settings | Outcome | Time s | Contact | 3D clearance m | Holding cycles (holds) | MPC p99 ms (misses), CPU |
|---|---|---|---:|---|---|---|---|
| `narrow_gate_home_nav --arm-pose carry`, `123057Z` | defaults | succeeded, 0 recoveries | 37.4 | none | 0.084 (panel to the gate's right approach wall) | 0 (0); largest slack beyond the current violation 5e-8 m | 9.8 (5), 27 % |
| `narrow_gate_home_nav --arm-pose carry`, `123255Z` | `--posture-mode comfort_only` | aborted after 9 recoveries | 49.0 | **none** | **0.044** (panel to the right gate post) | 236 (16); 152 plans failed the gate, up to 0.54 m beyond | 87.1 (463), 71 % |
| `worker_late_crossing_nav`, `123748Z` | defaults (starts in home, carry comfort pose) | canceled at the 90 s timeout, 31 recoveries | 90.0 | none | movers 0.004 m (footprint) | 0 (0); largest slack 7e-5 m | 64.2 (433), 70 % |

- **Defaults unchanged.** The narrow gate drive matches the carry smoke run above (37.2 s, 0.083 m, same
  arm choice: carry, vertical carry through the gate, partly lowered after it); the gate never triggered.
- **`comfort_only`: no contact, but not a clean hold.** The recorded `comfort_only` run touched the post
  (-0.015 m, 2.6 mm contact) after slack up to 0.51 m; with the gate the closest approach was 0.044 m, the
  arm stayed near carry (pitched to lift 0.28, elbow -2.09, `wrist_2` 0.29 rad for 19 s) and the MPC
  held 16 times before the posts. The goal did not end by no progress as designed: every FollowPath
  ended with "no valid control" (error 106) after 15 missed deadlines in a row, because the QP of these
  conflicting cycles was slow (solve p99 79 ms, total p99 87 ms; the recorded `comfort_only` run had
  p99 30 ms and 62 misses), and the tree aborted after 9 recoveries. The held plan being the next
  linearization point keeps deep, slacked rows in the QP, which is a likely cause of the slower solves
  (not isolated). After this run the arm was left outside a qualified pose and the next run needed a new
  Play epoch (as in the follow-up runs).
- **`worker_late_crossing_nav`: the deadlock is not resolved, and the gate was not involved.** The
  worker was triggered at 65.8 s, walked 2.82 m and then waited 88 s for the robot (its wait rule), the
  base footprint 0.004 m from it, held by the collision monitor's stop zone (185 activations) while the
  arm sat in vertical carry for 62 s. The MPC's slack stayed at 7e-5 m, so no cycle was gated: as in the
  recorded run 2, the stop zone held the robot, not the MPC. One run cannot say whether the recorded
  runs 1 and 3 (sustained slack) would now hold; the unit sweep says the deadlock is the wait rule's.

## Commanded versus measured arm at the closest approach (Phase 4B-A, 2026-10-18)

Phase 4B-A spec Section 5, step 1: does the lag between the JTC's commanded reference and the measured arm
explain the B5 runs whose 3D clearance fell below the 0.05 m margin? `clearance_3d --compare-reference`
computes, for every sample that has the reference (`/arm_controller/controller_state`), the minimum clearance
with the measured joints and with the reference joints at the same base pose. Decision rule: the lag
explains the shortfall if, wherever the measured clearance is below 0.05 m, the reference clearance is at least
0.049 m and the difference covers at least 80 % of the shortfall.

All recorded B5 runs of 2026-10-12/13 with a measured minimum in [0, 0.05) m: 24 runs (the 17 campaign runs plus
fix-round and follow-up runs). Columns: the measured minimum, the reference clearance at that moment, the
samples below the margin, and the reference minus measured joint positions there (rad, model order).

| Run | Measured m | Reference m | Below margin | Lag (pan, lift, elbow, w1, w2, w3) |
|---|---:|---:|---:|---|
| `124416Z` narrow_gate_vertical_carry_nav | 0.0138 | 0.0143 | 182 | 0.0005, -0.0003, -0.0002, -0.0002, -0.0002, 0.0 |
| `124719Z` narrow_gate_vertical_carry_nav | 0.0299 | 0.0327 | 80 | -0.0009, -0.0010, -0.0007, 0.0002, -0.0029, 0.0004 |
| `142033Z` overhead_course_nav | 0.0355 | 0.0357 | 2 | 0.0002, 0.0121, 0.0041, -0.0014, -0.0034, -0.0012 |
| `181321Z` overhead_course_worker_nav | 0.0403 | 0.0366 | 3 | -0.0019, 0.0128, 0.0061, -0.0003, -0.0031, 0.0003 |
| `192055Z` persistent_blockage_nav | 0.0454 | 0.0434 | 55 | 0.0015, -0.0010, -0.0010, -0.0016, -0.0110, 0.0004 |
| `192259Z` persistent_blockage_nav | 0.0423 | 0.0419 | 54 | 0.0021, -0.0027, -0.0014, -0.0007, -0.0095, -0.0006 |
| `193811Z` constrained_reconfiguration_mission | 0.0428 | 0.0478 | 19 | -0.0022, -0.0005, -0.0003, -0.0056, -0.0053, -0.0004 |
| `193932Z` constrained_reconfiguration_mission | 0.0446 | 0.0451 | 21 | 0.0041, -0.0022, -0.0012, -0.0015, -0.0004, -0.0027 |
| `194051Z` constrained_reconfiguration_mission | 0.0419 | 0.0450 | 20 | -0.0007, 0.0, 0.0, -0.0067, -0.0033, 0.0024 |
| `195715Z` overhead_course_nav | 0.0431 | 0.0436 | 2 | -0.0008, 0.0133, 0.0050, -0.0015, -0.0029, -0.0001 |
| `195943Z` overhead_course_nav | 0.0393 | 0.0331 | 3 | 0.0017, 0.0133, 0.0054, -0.0016, -0.0037, -0.0013 |
| `200156Z` overhead_course_nav | 0.0382 | 0.0341 | 65 | -0.0003, 0.0114, 0.0041, -0.0003, -0.0030, -0.0001 |
| `200429Z` overhead_course_worker_nav | 0.0412 | 0.0358 | 3 | 0.0010, 0.0136, 0.0050, -0.0018, -0.0036, -0.0008 |
| `200648Z` overhead_course_worker_nav | 0.0416 | 0.0355 | 14 | 0.0003, 0.0113, 0.0037, -0.0001, -0.0035, -0.0003 |
| `200916Z` overhead_course_worker_nav | 0.0397 | 0.0408 | 2 | 0.0014, 0.0124, 0.0042, -0.0019, -0.0034, -0.0006 |
| `202525Z` constrained mission, scripted | 0.0326 | 0.0365 | 18 | 0.0069, -0.0045, 0.0069, -0.0081, 0.0053, -0.0089 |
| `202721Z` constrained mission, scripted | 0.0342 | 0.0437 | 17 | 0.0026, -0.0043, 0.0013, -0.0176, -0.0006, -0.0038 |
| `202912Z` constrained mission, scripted | 0.0299 | 0.0400 | 21 | -0.0026, -0.0037, -0.0009, -0.0135, -0.0013, -0.0031 |
| `203753Z` overhead_course_nav (20 Hz) | 0.0462 | 0.0463 | 2 | -0.0010, 0.0138, 0.0053, -0.0019, -0.0028, -0.0007 |
| `204031Z` overhead_course_nav (20 Hz) | 0.0436 | 0.0398 | 28 | -0.0004, -0.0055, -0.0021, 0.0005, -0.0132, 0.0009 |
| `204308Z` overhead_course_nav (20 Hz) | 0.0434 | 0.0428 | 2 | -0.0002, 0.0187, 0.0096, -0.0039, 0.0011, -0.0003 |
| `080752Z` overhead_course_nav (follow-up) | 0.0374 | 0.0361 | 17 | 0.0012, -0.0039, -0.0015, -0.0015, -0.0120, 0.0019 |
| `092322Z` overhead_course_nav, carry | 0.0468 | 0.0431 | 19 | -0.0040, -0.0014, -0.0016, -0.0005, -0.0126, -0.0022 |
| `123255Z` narrow_gate_home_nav, comfort_only | 0.0442 | 0.0244 | 222 | -0.0003, 0.0057, 0.0134, -0.0007, 0.0146, -0.0008 |

**Verdict: the lag does not explain the shortfall** (`explained` false in all 24 runs; the smallest covered
fraction is 0 in every run). At the closest moment the reference arm is itself inside the margin
(0.014-0.048 m), in 12 runs closer than the measured arm. The lags are real (shoulder lift 0.011-0.019 rad
behind on the overhead course, `wrist_2` about 0.01 rad elsewhere) but their effect on the clearance is of
either sign and never enough in the helpful direction. The JTC's commanded reference, not the tracking, goes
below the margin. The reference used is the latest `controller_state` received before each joint sample
(usually 0.02 s older); re-aligning it to the sample's own stamp (checked on 5 runs) moves the reference
clearance by a few millimetres either way and changes no verdict.
Per the spec (Section 5, step 2b) the fix is chosen with the user; candidates: the JTC reference between the
0.1 s plan stages (the spline through the streamed stages can cut inside the stage-wise constraints), pairs
beyond the per-stage row budget (8 closest pairs per stage) or the 0.8 m activation distance, and the URDF/FCL
primitives against the scoring model. Reproduce with
`ros2 run mobile_manipulator_mpc clearance_3d --bag <run>/bag --scenario <name> --compare-reference`.
