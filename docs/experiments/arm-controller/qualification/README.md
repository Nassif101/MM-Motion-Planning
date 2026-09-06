# Arm and payload qualification

This report extends the original commissioning with a usable payload-carrying
envelope for planner/MPC research, without MoveIt. The reference is the physical
3 kg, 1.20 × 1.20 × 0.04 m panel in `ConstructionSiteV1`, Unity 6000.5.2f1/PhysX,
and ROS 2 Jazzy in the development container. Results are simulation measurements,
not a claim that every arm configuration, payload or construction-site route is safe.

## Model and control decisions

The six-joint geometry is retained. Shoulder-lift torque is 160 Nm (originally
120 Nm); other limits remain 120/90/40/30/20 Nm. Gravity, payload mass/COM/inertia,
finite drive forces and wheel contact remain active. The articulation uses 12
position and 4 velocity solver iterations and a 20 ms physics period. The saved
scene retains the panel and starts in the home pose.

The two useful payload configurations are:

| Pose | Joint positions in ROS order, rad | Purpose |
|---|---|---|
| Home | `[0, 0, 0, 0, 0, 0]` | Intermediate transition pose |
| Compact vertical carry | `[π/2, 0, 0, 0, π/2, 0]` | Upright proximal arm; panel aligned along the base for narrow passages |
| Level extension | `[0, π/2, 0, 0, −π/2, 0]` | Shoulder reaches horizontally; wrist keeps the panel level |

The straight horizontal arm with an uncompensated vertical panel has only about
2 cm nominal floor clearance and touched the floor in commissioning. It is excluded.
Wrist compensation solves that geometric problem without increasing the pedestal
or removing gravity. Transitions use synchronized 8 s cubic trajectories with zero
endpoint velocities, through home. Arbitrary direct transitions are not qualified.

The machine-readable [reference profile](../../../../ros2_ws/src/mobile_manipulator_control/config/qualified_payload.json)
contains joint order, poses, payload, command envelopes and acceptance limits.
The full model/gain derivation is in [the controller document](../../../unity-arm-controller.md).

## Acceptance and evidence

`final-*.csv` and their matching action JSON contain the final normal-frame-rate
integration runs. [acceptance.json](acceptance.json) is generated directly from
these recordings. Action success alone is insufficient: every accepted run must
also satisfy all physical checks:

- Joint path error below 0.15 rad and observed hold error below 0.06 rad.
- No WATCHDOG_HOLD/FAULT sample and no detected panel penetration.
- No collision-query buffer overflow; panel floor clearance above 0.15 m.
- Base tilt below 3° and each drive below 2% estimated saturation samples.
- Gate traversal must clear the entire robot, with over 0.10 m lateral margin.

`driveForce` is the solver drive contribution, not joint reaction torque or motor
current. The saturation indicator is an estimate at 99% of the configured limit.
Collision checks sample each physics tick and measure panel penetration, including
against robot colliders; they do not prove continuous collision freedom for an
arbitrary path. Gate margins use all active robot collider world-space bounds.

<!-- FINAL_RESULTS -->

The manipulation fixture starts the stopped base at Unity `(0, 0.21, 12)`, yaw
180°, in the surveyed open area. The gate fixture starts at `(7.725, 0.21, −5.3)`,
yaw 180°, already in vertical carry. Placement uses `TeleportRoot` only to set up
the experiment; all recorded traversal and base disturbances use physical wheel
drives. This does not claim a planner found a route to the gate.

The base disturbance schedule tests starts, stops, reversals, yaw and curved travel:
compact carry commands reach ±0.3 m/s and ±0.4 rad/s; level extension reaches
±0.15 m/s and ±0.2 rad/s. Gate traversal commands 0.2 m/s for 18 simulated seconds,
then zero for 3 seconds. These are commanded values, not bounds on every measured
contact-induced velocity peak. Base acceleration/braking settings are unchanged.

## Physics time, transport and low-frame-rate stress

`/clock` is published from early `FixedUpdate`, not render `Update`. Integer
nanosecond accumulation converts the configured physics step to exact 20 ms ticks.
This matters because Unity exposed 0.02 s as 0.0199999921 s: raw float-derived
timestamps made an exact 20 ms ROS timer skip ticks. Arm feedback and packet-age
checks now use the same canonical physics clock. Recorder `sim_time` retains raw
Unity fixed time, so its displayed durations differ by a few microseconds.

The project endpoint on port 10000 keeps the upstream protocol and sensor/service
handling, with depth-one arm/base command subscriptions, two executor workers and
TCP_NODELAY. Duplicate-time ros2_control writes are suppressed. Neither the 0.5 s
wall-clock watchdog nor the 0.25 s simulated packet-age limit was relaxed.

The [final 30 s endpoint observation](final-transport-rates.json) measured 48.6
commands/s and 50 state/clock messages per simulated second, with a maximum
command interarrival gap of 0.0941 s and no non-increasing stamps.
At 15 FPS with TCP_NODELAY, a 30 s observation measured 44.2 commands/s
and 50 state/clock messages per simulated second; command interarrival gaps reached
0.151 s. See [the 15 FPS measurement](nodelay15-transport-rates.json) and
[physics/frame counters](low-frame-clock.json). These are receiver observations,
not one-way latency measurements or hard real-time guarantees.

Low FPS is **not qualified for uninterrupted control**. The initial 15 FPS hold
passed after TCP_NODELAY, and the subsequent vertical/base cases passed, but the
return-home stress trial recorded a 1.065 s wall-time gap between consecutive
20 ms physics samples and entered WATCHDOG_HOLD. It ultimately reached the goal,
but is still a failed continuous-control test. Earlier 15/30 FPS trials also had
watchdog events. TCP_NODELAY does not cure an Editor stall; FixedUpdate controls
simulation-time progression, not guaranteed wall-clock scheduling. Use normal
Editor settings for the qualified runs. A standalone-player performance qualification
is the next step before requiring continuous control under heavy rendering or
planner load; do not conceal stalls by increasing watchdog thresholds.

## Failure handling and shutdown

The retained cancellation test returned action status CANCELED and stopped before
the target. The final one-second joint-speed maximum was below the 0.05 rad/s
stopped criterion. Disabling arm transport caused ROS hardware to become
unconfigured with no available/claimed command interfaces, while Unity retained
finite-torque WATCHDOG_HOLD. Loaded hold error remained below 0.015 rad with no
panel penetration in the original test. The final endpoint repeat held within
0.0152 rad with at least 0.7036 m floor clearance and no penetration; ROS again
reported unconfigured hardware and zero available/claimed command interfaces.
See `safety-cancel.*`, `safety-feedback-loss.*`, `safety-nodelay-feedback-loss.*`
and the recovery recordings.

The original pause snapshots started in WATCHDOG_HOLD. The final
[pause check](final-pause-check.json) instead began in fresh controlled HOLD:
physics, clock and pose froze for three wall-clock seconds, then resumed with
less than 0.003 rad pose change. Hardware remained active and fresh commands
restored HOLD. The test's initial assertion that the final snapshot must remain
WATCHDOG_HOLD was too restrictive and is retained as false with that explanation.
Restart the manager after an actual hardware freshness fault or a new Play epoch;
a pause alone need not fault when fresh feedback is available on resume.

Play-mode teardown previously indexed an empty native articulation reduced-space
buffer during `OnDisable -> CaptureHold`. Quit guards and atomic six-joint hold
capture with DOF checks fix that path. Unity cloud-token authentication errors were
separate Editor/account errors, not arm-controller exceptions.

<!-- FINAL_CHECKS -->

## Reproduce

Start the [documented stack](../../../running-the-stack.md), build/source the ROS
workspace, and run the project endpoint in the container:

```bash
ros2 run mobile_manipulator_control unity_control_endpoint.py --ros-args \
  -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000
```

Enter Play with the construction-site scene, then start a fresh manager:

```bash
ros2 launch mobile_manipulator_control arm_control.launch.py
```

From the host repository root, substitute the actual container name and use a new
prefix (the runner refuses to overwrite evidence):

```bash
python3 tools/run_arm_qualification.py --container CONTAINER --prefix repeat
python3 tools/run_arm_qualification.py --container CONTAINER --suite gate --prefix repeat
python3 tools/analyze_arm_qualification.py docs/experiments/arm-controller/qualification --prefix repeat-
```

The manipulation suite ends in vertical carry, as required by the gate fixture.
`--start-at CASE` is only for a diagnosed interrupted run with a fresh controlled
HOLD. For render stress, set `arm_test_frame_limit --fps 15 --vsync 0`, then use
`--suite timing` while already in vertical carry. Restore `--fps -1 --vsync 1`.
Use compiled `arm_test_record`, `arm_test_end` and `arm_test_snapshot`; runtime
`eval` compilation itself can stall the Editor and invalidate a timing run.

Stop the manager before exiting Play. A new Play session restores the saved scene,
not the temporary fixture location. Do not run this fixture runner on hardware.

## Evidence history and planner handoff

`baseline-*` contains the preceding successful normal-frame suite before TCP_NODELAY.
`lane-*` contains early tests in the narrow spawn lane: `lane-level-base` is rejected
because even its approximately 0.6 mm panel/fence penetration contaminated tracking.
`pre-endpoint-*` and `vertical-transition-clock-trial.*` retain transport/clock
failures. `stress15-*`, `stress30-*` and `nodelay15-*` retain low-frame trials,
including failures; they are not silently included in the normal acceptance set.
`interrupted-base-message-type.csv` predates the experiment runner's float conversion
fix for ROS Twist messages. `safety-paused-hold.csv` is not a feedback-loss trial.

For MPC, keep one ROS command owner and use
`/arm_controller/follow_joint_trajectory` for the current controller. Use actual
joint feedback, the full arm/panel collision geometry and appropriate tracking and
delivery margins. A narrow base footprint alone is insufficient. A different MPC
controller can replace JTC while preserving the named position/velocity boundary.
Requalify payload mass/COM, geometry, gains, timestep, motion rates and transport
changes. This completed reference qualification lets planner development proceed
within a measured envelope; it does not certify the whole workspace or arbitrary
simultaneous base/arm maneuvers.
