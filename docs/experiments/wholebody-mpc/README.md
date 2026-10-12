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

`position_jumps` is 0 or 1 on the vertical_carry -> home transitions (1 where it occurs; 0.0149 rad, the `wrist_2` hold sag of
1.5856 against 1.5708 rad): with `interpolate_from_desired_state: false` JTC starts an action trajectory at
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
  stream stopped, JTC kept commanding the last point's velocity (0.28 rad/s) indefinitely, and the arm
  settled 0.034 rad off the 1 s segment end instead of at it, the reference velocity still 0.28 rad/s in
  the controller state (runs `jtc1-*`, same tool version as `jtc0`; the velocity from a controller-state log).
  This is a hazard if the MPC dies mid-motion, so the flag stays `false`.
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
`summary.json`); a near miss is a cycle above 0.8 of the period.

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
(at most 175); no cycle needed the fallback plan, and the collision slack stayed at zero.

**Tuning.** No weight in `wholebody_mpc.yaml` was changed: no run failed for a tuning reason.

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
margin, where the 20 Hz runs kept 0.099 and 0.113 m. The plans respected the qualified arm limits
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
above, **the campaign runs at 20 Hz** until the arm streaming tracks at 50 Hz; at 20 Hz the largest p99 was
16.0 ms unscripted and 25.0 ms scripted (period 50 ms).

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
