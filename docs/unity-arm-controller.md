# Unity arm controller and ros2_control

## Purpose and commissioned scope

The six-DOF research arm uses ROS 2 Jazzy ros2_control for trajectory execution and
Unity 6000.5.2f1/PhysX for finite-torque actuation. It runs without MoveIt. The existing
skid-steer base controller and `/cmd_vel` contract are preserved.

Commissioning covers unloaded and 3 kg reference-panel operation, each joint's positive
sign, a one-radian unit check, coordinated multipoint trajectories, cancellation,
communication loss, and bounded base disturbances. The current qualification adds
compact vertical carry, wrist-compensated level extension, a 60 s loaded hold, and
physical passage through the 1.05 m gate. This is a tested simulation envelope, not a
commercial robot specification or full-workspace payload qualification.

## Architecture and ownership

```mermaid
flowchart LR
  A[Manual FollowJointTrajectory client] --> B[JointTrajectoryController]
  B --> C[UnityArmSystem position/velocity interfaces]
  C --> D[ROS-TCP-Endpoint /arm/command]
  D --> E[ArmRosTransport]
  E --> F[ArmActuatorController FixedUpdate]
  F --> G[Finite Force ArticulationDrives / PhysX]
  G --> H[/arm/state actual position/velocity]
  H --> C
```

`controller_manager` runs the read/update/write cycle. The [standard Jazzy JTC](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html) alone owns trajectory time,
cubic interpolation when endpoint velocities are supplied, synchronization, path/goal
tolerances, cancellation, replacement and action results. Unity receives instantaneous
position and velocity; it neither receives full trajectories nor retimes individual
joints. The hardware plugin validates and transports data without IK, planning or
collision avoidance.

`ArmRosTransport` keeps one newest packet under a lock; callbacks never access Unity
physics. Its FixedUpdate consumes the packet before actuator FixedUpdate. The actuator
validates the entire packet before changing any target. Physical state is sampled in
FixedUpdate. New state-message instances avoid asynchronously serializing a mutated
array. The ROS executor is serviced in the hardware read cycle; this thin simulation
implementation uses ordinary publishers, containers and allocations, not a real-time
transport implementation.

## Interfaces, names and time

| Interface | Type and purpose |
|---|---|
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory`; public trajectory execution |
| `/arm_controller/joint_trajectory` | Standard JTC topic; action interface preferred for results/cancellation |
| `/arm/command` | `sensor_msgs/msg/JointState`; six named instantaneous target positions/velocities; empty effort |
| `/arm/state` | `sensor_msgs/msg/JointState`; six actual physical positions/velocities; empty effort |
| `/arm_controller/controller_state` | Standard JTC reference, feedback, error and output |
| `/arm_joint_state_broadcaster/joint_states` | Arm-only actual state through ros2_control |
| `/joint_states` | Existing Unity ten-joint actual-state authority remains unchanged |
| `/arm/robot_description` | Transient-local augmented URDF for controller_manager; no TF publisher |

The hardware plugin exposes `position` and `velocity` command interfaces and `position`
and `velocity` state interfaces for every arm joint. It exports no effort interface.
All mappings use names. Missing, unknown, duplicate or incomplete arrays and non-finite
values are rejected. Scene mappings explicitly pair URDF joint names and articulation
references, and validate imported `UrdfJoint.jointName` at startup.

| ROS/ros2_control joint | Unity body | URDF positive axis | Sign | Position limits rad | Hard rad/s | Force limit Nm | Kp Nm/rad | Kd Nm s/rad |
|---|---|---|---:|---:|---:|---:|---:|---:|
| shoulder_pan_joint | shoulder_pan_link | +Z | +1 | ±2.9670597 | 1.8 | 120 | 2000 | 240 |
| shoulder_lift_joint | upper_arm_link | +Y | +1 | ±1.7453293 | 1.6 | 160 | 6000 | 360 |
| elbow_joint | forearm_link | +Y | +1 | ±2.3561945 | 1.8 | 90 | 3500 | 180 |
| wrist_1_joint | wrist_1_link | +Z | +1 | ±π | 2.5 | 40 | 500 | 40 |
| wrist_2_joint | wrist_2_link | +Y | +1 | ±2.0943951 | 2.5 | 30 | 650 | 35 |
| wrist_3_joint | wrist_3_link | +Z | +1 | ±2π | 3.2 | 20 | 300 | 28 |

Names, axes, positions and hard velocities are extracted from the model. Effort limits
are model engineering assumptions; shoulder lift was revised from 120 to 160 Nm after
commissioning. Gains are calculated starting assumptions, accepted through experiment.
The source generator, ROS URDF, translated Unity URDF, scene drive limits and imported
joint metadata carry the revised effort value.

Unity's handedness makes these positive angular axes local `-Y` for ROS +Z and local
`+X` for ROS +Y. Positive 0.05 rad commands and local quaternions confirmed all six signs.
`ArmCommandValidation` centralizes the boundary: ROS and `jointPosition/jointVelocity`
use rad/rad/s; revolute `xDrive.target/targetVelocity` use degrees/degrees per second in
the installed Editor. A 1 rad wrist command measured 1.0000304 rad and a 57.29578 degree
drive target; its local quaternion independently matched the rotation. The [Unity ArticulationDrive API](https://docs.unity3d.com/6000.0/Documentation/ScriptReference/ArticulationDrive.html)
pages have inconsistent angular-velocity wording, so the installed-version
experiment and the prior base-controller unit test are the operative evidence.

Unity `/clock` remains the simulation-time authority. Controller manager and JTC use
simulation time. Command stamps come from the manager; state stamps come from Unity
fixed time. A backward state timestamp faults the hardware. Restart the manager after
Play restart, scene reset, or a hardware freshness error. Do not keep an old manager
across simulation epochs.

## Timing, buffering and watchdog

- Controller-manager configuration: 50 Hz.
- Unity physics: 0.020 s, 50 simulated ticks/s; drive application is in FixedUpdate.
- Unity feedback: one packet per fixed tick, Connector outgoing queue depth one.
- Hardware publishers/subscribers: reliable DDS, depth one. Endpoint transport is TCP;
  this does not make the complete path a hard real-time or guaranteed-latency channel.
- `/clock` now runs in early `FixedUpdate`. Integer nanosecond accumulation uses the
  configured timestep rounded to microsecond precision, avoiding sub-period float
  error that caused exact ROS timers to skip ticks. Arm state and command validation
  share this clock. ROS callbacks still arrive asynchronously through the Connector.
- With the final endpoint, a 30 s observation measured 48.6 command packets/s
  and approximately 50 state/clock packets/s. Command interarrival gaps reached
  0.0941 s in that observation. These are receiver measurements, not guaranteed
  latency or one-way network latency. Historical 28–30 Hz observations remain in
  the original evidence set.
- Start `mobile_manipulator_control unity_control_endpoint.py` on port 10000. It
  uses latest-only DDS subscriptions for arm/base commands, two executor workers,
  and TCP_NODELAY on accepted connections,
  retaining upstream protocol and sensor handling. The hardware plugin suppresses
  duplicate-time command writes. No watchdog tolerance was widened.

The 0.5 s monotonic watchdog allows 25 nominal periods. Longer Editor stalls still
trigger HOLD and can require ROS recovery. A packet must also be within 0.25 s of current simulation time, have a
strictly advancing timestamp, and pass complete name/value/limit validation. Repeated
clock stamps can be discarded during controller catch-up; they do not refresh safety
freshness. The bridge is suitable for the qualified trajectories; substantially faster
motions require new measurements. See the [current qualification](experiments/arm-controller/qualification/README.md).
Low-frame-rate stress is not qualified for uninterrupted control: a 15 FPS run
recorded a 1.065 s Editor/physics stall and correctly entered watchdog HOLD despite
the physics-based clock. TCP_NODELAY improves packet flushing but cannot prevent
Editor stalls.

The ROS hardware side refuses activation until actual feedback arrives (10 s bounded
startup wait). It initializes command positions from actual state, not zeros. No command
is sent until a controller claims the arm interfaces. Active feedback older than 0.5 s
in monotonic time, or an incompatible simulation timestamp, faults the hardware and
stops command publication. The local Unity watchdog then captures current actual joints.

## Actuator states and lifecycle

`INITIALIZING -> HOLD` validates configuration and captures physical startup positions,
with desired velocities zero. `HOLD -> EXTERNAL_CONTROL` occurs on a valid nonzero-
velocity packet. A zero-velocity packet selects `HOLD` at the supplied instantaneous
position: this is an actuator mode, not a claim about action success. JTC can retain
HOLD targets after completion or cancellation.

After 0.5 s without a valid packet, either externally supplied mode enters
`WATCHDOG_HOLD`, captures actual position once, and commands zero desired velocity.
Drives stay enabled and torque-limited. Fresh valid packets can resume control; the
operator must restart the ROS manager after hardware or epoch faults. Invalid startup
configuration enters `FAULT` with a clear error. Fix the configuration before Play;
FAULT is not an alternate passive operating mode.

Disabling transport unsubscribes and requests HOLD. Disabling the actuator captures
and writes finite HOLD drives once, leaving them physically engaged. Re-enabling does
not restore arbitrary old zero targets. The retired `ArmJointHoldController` must not
coexist as an enabled drive owner. Scene setup removes it.

Play-mode exit is different from disabling a running component. `OnApplicationQuit`
marks the actuator unavailable before teardown; neither controller nor transport
requests a new physical hold while quitting. Hold capture also checks that every
body is active and its returned reduced-coordinate buffer has one DOF before indexing.
It commits all six positions together. This fixes the shutdown exception from
`OnDisable -> CaptureHold -> Position -> ArticulationReducedSpace[0]` when Unity has
already removed native physics state. Unity documents Play-mode exit calling
[OnApplicationQuit](https://docs.unity3d.com/6000.0/Documentation/ScriptReference/MonoBehaviour.OnApplicationQuit.html).

## Physical model and tuning calculations

Mass, COM and inertia inspection preceded tuning. The imported values agree with the
URDF under ROS FLU to Unity conversion: Unity COM `(-Yros, Zros, Xros)` and diagonal
inertias `(Iyy_ros, Izz_ros, Ixx_ros)`. Off-diagonal inertias are zero in this model.
The physical arm mount is 10 kg with ROS COM `(0,0,0.09)` and diagonal inertia
`(0.052,0.052,0.05)` kg m². The six link values are:

| Link | kg | ROS COM z m | ROS Ixx, Iyy, Izz kg m² |
|---|---:|---:|---|
| shoulder_pan_link | 7 | 0.06 | 0.0259, 0.0259, 0.035 |
| upper_arm_link | 8 | 0.16 | 0.0749333, 0.0766293, 0.0150293 |
| forearm_link | 5 | 0.14 | 0.0356067, 0.0361933, 0.0064667 |
| wrist_1_link | 2.5 | 0.05 | 0.00418583, 0.00418583, 0.004205 |
| wrist_2_link | 2 | 0.05 | 0.00278733, 0.00307733, 0.00253133 |
| wrist_3_link | 1.5 | 0.045 | 0.00195, 0.00195, 0.001875 |

All these inertials are extracted engineering estimates, not measured real hardware.
The reference payload is a physical 1.20 × 0.04 × 1.20 m box in Unity tool-local XYZ,
3 kg, COM `(0,0.035,0)` m, principal inertia `(0.3604,0.7200,0.3604)` kg m². The fixed
`tool0` articulation represents its mass and inertia; the panel collider represents
its contact geometry. Unloaded testing removes the panel collider/visual and returns
`tool0` to the existing 0.001 kg, 1e-6 kg m² frame-only surrogate. The saved scene retains
the reference payload. No gravity, transform lock, kinematic or mass workaround is used.

`tools/calculate_arm_actuators.py` reproduces the source extraction and conservative
serial-lever estimates in `model-calculations.json`. With downstream link COM distances
at full extension:

`tau_g <= sum(m_k * 9.81 * d_k)` and `I_effective <= sum(I_k,max + m_k*d_k²)`.

The shoulder-lift unloaded bound is 78.21 Nm; the panel adds 27.22 Nm, giving 105.43 Nm.
The elbow bound grows from 31.12 to 48.93 Nm. Wrist lever bounds are 17.34, 9.74 and
4.34 Nm loaded; these deliberately overbound some configurations because perpendicular
lever distances can be shorter than serial distances. Pan gravity torque is zero on a
level base about its vertical axis; its 135.45 Nm serial lever result is a tipping-axis
bound, not nominal pan gravity. Use its loaded inertia bound for yaw-disturbance tuning.

The 120 Nm shoulder trial had only 14% static reserve and failed loaded return paths.
160 Nm gives 52% reserve and passed loaded out-and-back motion. Other effort ceilings
remain the model's 120/90/40/30/20 Nm. These are finite provisional actuator assumptions.

Using `Kp ~ tau_g/e_allowed`, the loaded shoulder and elbow gains target approximately
0.0176 and 0.0140 rad ideal static droop; wrist conservative bounds imply 0.0347,
0.0150 and 0.0145 rad. The pan gain covers about 8.7 Nm of nominal yaw inertial disturbance
at 0.8 rad/s² with an approximately 0.0044 rad target error. These are engineering
starting targets, not exact coupled-model guarantees. JTC's accepted goal envelope is
0.04 rad per joint, path tolerance 0.15 rad, stopped velocity 0.05 rad/s, and goal-time
allowance 3 s.

Loaded effective-inertia bounds are approximately 10.88, 7.92, 3.02, 1.19, 0.91 and
0.77 kg m². `Kd = 2*zeta*sqrt(Kp*I_effective)` puts the selected damping in the roughly
0.70–0.92 damping-ratio range. Coupling, implicit drives, contacts and solver accuracy
make this an estimate. Generic body angular damping is 0.05; URDF joint friction is
retained at 0.30/0.25/0.20/0.12/0.10/0.08. Active drive damping, body drag, mechanical
joint friction and wheel/floor contact friction serve distinct purposes.

## Solver, base motion and payload trade-offs

The default 6 position/1 velocity solver iterations gave approximately 0.05 rad loaded
shoulder droop and a goal-tolerance failure. A 12/4 trial reduced it to approximately
0.026 rad without changing gains. The controller applies 12/4 to the articulation at
startup; these properties do not survive scene serialization. The 10 ms trial did not
cure 120 Nm loaded-return failures, and the final timestep remains 20 ms.

The base code and limits remain 0.5/0.8 m/s² acceleration/braking and 0.8/1.2 rad/s²
angular acceleration/braking. Original commissioning upright tests use ±0.2 m/s, ±0.3 rad/s and a 0.15 m/s,
0.15 rad/s curve. Extended loaded tests use ±0.1 m/s and ±0.1 rad/s with stops. Each
command is streamed at 20 Hz for two seconds with one/two-second stop periods.
Actual motion and finite-difference accelerations are logged; these include contact
jitter and should not be mistaken for commanded acceleration limits.

With the arm straight and horizontal and the wrist uncompensated, the vertical panel
touched the floor and therefore cannot qualify as unsupported gravity HOLD. The
current level-extension pose compensates with wrist 2 at −π/2, keeping the panel
horizontal. Compact vertical carry leaves the proximal arm upright and turns the
panel with the wrist. The current qualification tests ±0.3 m/s and ±0.4 rad/s in
vertical carry, ±0.15 m/s and ±0.2 rad/s in level extension, and 0.2 m/s through the
gate. These are command envelopes; actual yaw-rate peaks are reported separately.
Faster base motion, more extension, heavier payloads or different
COMs require new tests and may require a smaller acceleration envelope or a different
arm pose. A stronger simulated actuator cannot fix payload/environment collision.

For the extension options and nominal geometry, see the
[follow-up analysis](experiments/arm-controller/README.md#extension-and-shutdown-follow-up).

## Limits and future controllers

Unity and the hardware plugin reject position/velocity targets beyond model limits;
Unity additionally writes the hard `maxJointVelocity`. Neither retimes individual
joints. JTC performs interpolation, not general velocity/acceleration-constrained
trajectory generation. The manual experiment client checks cubic peak velocity
`1.5*abs(delta_q)/T` and peak acceleration `6*abs(delta_q)/T²` against half the model's
velocity and provisional acceleration envelope. Direct action/topic clients must
provide equally valid trajectories. Planning acceleration assumptions remain
2.0/1.5/2.0/3.0/3.0/4.0 rad/s², not new certified hardware ratings.

MoveIt configuration is deliberately absent. It can later use the same action and
actual `/joint_states` without changing the Unity actuator. Future MPC can replace the
ROS controller while preserving named instantaneous actuator targets. Explicit gravity
feedforward is also deferred: add it at the actuator-effort boundary only if tighter
accuracy or new payload evidence justifies it. The present feedback-only controller
accepts small physical droop under gravity.

## Running, recording and reproducing

Follow [the stack runbook](running-the-stack.md#mobile_manipulator_control). To repair
scene wiring after importing/regenerating the robot, exit Play and run:

```bash
unity command setup_arm_control --project-path ./motion-planning-sim
```

The active scene requires a `MobileManipulator` root, one explicit ROSConnection,
six mapped arm bodies, the existing base body, and the reference payload physical
properties. `ArmControlSetup` authors actuator, transport and recorder references;
`MobileManipulatorRosSetup` invokes it when configuring a scene. The base prefab alone
is not a complete ROS arm experiment scene.

During Play, use the compiled Pipeline commands to avoid runtime C# compilation
stalling physics. Recordings go under `docs/experiments/arm-controller/qualification`:

```bash
unity command arm_test_record --name my-run
# End and flush before analyzing:
unity command arm_test_end
python3 tools/analyze_arm_experiments.py docs/experiments/arm-controller
python3 tools/calculate_arm_actuators.py
```

`startupRecordingPath` can be set temporarily before Play to capture the first physics
tick. Leave it empty in saved scenes. Recorder CSV fields include simulation/monotonic
time, actuator state, per-joint desired/actual position and velocity, both errors,
command age, base linear/yaw velocity and finite-difference acceleration, solver
`driveForce` and a near-limit indicator. `driveForce` is the drive's solver contribution,
not net joint reaction torque; it is never substituted into ROS actual effort. The
99%-of-force-limit indicator is a useful saturation estimate, not motor current data.

The analysis reports maximum/RMS error, final two-second mean droop, maximum velocity
error, maximum drive effort, near-limit percentage and command-envelope overshoot.
Settling requires position error below 0.04 rad and actual speed below 0.05 rad/s for
the rest of the recorded tail, with at least two seconds observed. Null means the
record does not establish that settling criterion. Envelope overshoot for multipoint
motion is not a fitted second-order step overshoot.

See [commissioning results](experiments/arm-controller/README.md), machine-readable
`metrics.json`, compressed raw CSV, and [ADR 0004](adr/0004-ros2-control-unity-arm-actuation.md).
