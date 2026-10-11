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

All 18 runs of the requested scales passed with the base driving at 0.30 m/s and 0.4 rad/s (measured peaks
0.297 to 0.300 m/s): path error at most 0.025 rad (limit 0.10), no watchdog or tilt flag (tilt at most
0.16 degrees against 3), no panel penetration, no saturation, floor clearance 0.709 m. The largest scale,
**1.0**, is chosen. Its segment (1.5708 rad on `shoulder_pan` and `wrist_2` in 7.9 s of cubic) peaks at
1.5 x 1.5708 / 7.9 = 0.298 rad/s and 6 x 1.5708 / 7.9^2 = 0.151 rad/s^2; the `wholebody_arm` block in
`nav_operating_envelope.yaml` uses 0.29 rad/s and 0.15 rad/s^2, rounded down. The block applies these values
to all six joints: the transitions exercise only `shoulder_pan` and `wrist_2`, and the streamed sinusoids
below exercise all six joints at 0.31 rad/s and 0.49 rad/s^2 within 0.09 rad.

The two extra scales (not part of the decision rule) show the arm tracks the same cubic in 4 s, at 0.61
rad/s, with 0.036 rad: the chosen limits are the qualified reference speed, not the arm's capability, and
the tool's own guard (half the URDF velocity limit, 0.5 x the acceleration envelope) would allow about
2.5 times the chosen scale. A later requalification may raise `wholebody_arm` if the MPC needs faster arm
motion.

`position_jumps` is 1 on the vertical_carry -> home transitions (0.0149 rad, the `wrist_2` hold sag of
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
at its torque limit for 24 % of the samples. At the vertical-carry pan the panel is in contact at about
1.80 rad (the colliding body was not identified). Probes at +0.1 and +0.2 rad held within 0.016 rad with no contact, so
`wholebody_arm.joint_bounds` limits `wrist_2` to at most **1.77 rad** and leaves every other joint on its
URDF range. The bound holds for all configurations, although the contact itself depends on the other
joints; the collision model of the MPC remains responsible for the exact geometry. The
lowered pose uses `shoulder_lift` 1.29 rad: the planar formula of the arm-controller README gives 0.295 m
nominal, the measured panel bottom is 0.252 m (0.043 m of payload sag), meeting the 0.25 m target.

## JTC streaming

`arm_experiment --stream-rate R --stream-duration S` publishes, at R Hz on simulation time, overlapping
segments of `q0 + 0.2 sin(2 pi 0.25 t)` on all six joints (peak 0.314 rad/s, 0.49 rad/s^2) on
`/arm_controller/joint_trajectory`: header stamp 0 (start now), points at 0, 0.25, 0.5, 0.75 and 1.0 s with
positions and velocities sampled from the sinusoid at the publish time. The arm starts in `home`.
`--stream-stop-after T` stops the stream at T; the tool then watches the arm settle for 2.5 s after the last
segment ends.

Final design (every segment ends with a 0.5 s braking tail):

| Run | Segments | Achieved rate, Hz | Position jumps | Largest jump, rad | Path error, largest joint (`wrist_1`), rad | Error against the sinusoid, rad | Settle error against the last segment end, rad |
|---|---:|---:|---:|---:|---:|---:|---:|
| 20 Hz, 20 s | 401 | 20.1 | 0 | 0.0061 | 0.0636 (others <= 0.023) | 0.070 | 0.011 |
| 50 Hz, 20 s | 1001 | 50.0 | 0 | 0.0058 | 0.0749 (others <= 0.027) | 0.081 | 0.011 |
| 20 Hz, stopped at 7.3 s (mid-motion) | 147 | 20.1 | 0 | 0.0034 | 0.0736 (others <= 0.026) | 0.080 | 0.009 |

The simulation clock ticks at 50 Hz, so the tool schedules each segment on the average period rather
than resetting the timer. Settle errors of 0.009 to 0.011 rad are the usual `shoulder_lift` hold sag; after
the stop-mid-motion run the arm rests at the braking tail's end (the last segment's end plus 0.25 s times its
end velocity), and the JTC reference is constant there.

JTC setting findings (all on the unchanged `controllers.yaml`; no setting had to be changed):

- With `allow_nonzero_velocity_at_trajectory_end: false` JTC **rejects any segment whose last point still
  moves**. Streaming plain 1 s sinusoid segments (before the braking tail was added) kept only the few
  segments that happened to end near a velocity zero: 18 position jumps of up to 0.32 rad, path error 0.33 rad,
  0.53 rad from the sinusoid, and 0.29 to 0.33 rad error after a stop (runs `jtc0-*`).
- Setting the flag to `true` accepted the segments (0 jumps, path error 0.063 to 0.069 rad) but, when the
  stream stopped, JTC kept commanding the last point's velocity (0.28 rad/s) indefinitely, and the arm
  settled 0.034 rad off the last segment end instead of at it, the reference velocity still 0.28 rad/s in
  the controller state (runs `jtc1-*`, same tool version as `jtc0`; the velocity from a controller-state log).
  This is a hazard if the MPC dies mid-motion, so the flag stays `false`.
- A **braking tail** (a last point 0.5 s after the segment's end, at rest, 0.25 s times the end velocity
  beyond it, i.e. constant deceleration) satisfies the validator, never executes while newer segments keep arriving
  (every segment starts at the current time with the sinusoid's own position and velocity, so consecutive
  segments are continuous: 0 jumps), and brings the arm to rest at its end if the stream stops. The
  Task 10 node must stream this way, or the tail's equivalent (hold at the current joints, zero velocity)
  when it stops.
- `interpolate_from_desired_state` is read-only in this JTC and was not needed: with the first point at time 0
  each segment starts on the commanded trajectory, not on the measured state.

Limits of the evidence: one 20 s run per rate and one stop run, at a single amplitude and frequency
(0.2 rad, 0.25 Hz, inside the tool's guard); no streaming was run while the base drove.
