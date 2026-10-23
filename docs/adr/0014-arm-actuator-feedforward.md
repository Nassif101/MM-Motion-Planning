# ADR 0014: the Unity arm actuator honours the reference's timing and supplies its feedforward

Date: 2026-10-23. Status: accepted (Phase 4B-B). Amends [ADR 0004](0004-ros2-control-unity-arm-actuation.md).

## Decision

The Unity arm actuator (`ArmActuatorController`) acts as a servo drive that executes the ROS reference on time,
with three independent switches:

- **Gravity feedforward** (on by default since 2026-10-23): the static holding torque of the links each joint
  carries, payload included.
- **Motion feedforward** (on by default since 2026-10-24, after the qualification): the inverse dynamics of the commanded joint velocities and
  accelerations, τ = M(q) q̈ + C(q, q̇) q̇ + G(q) by recursive Newton-Euler over Unity's own masses and inertia tensors,
  with the base's reference motion (the skid-steer controller's rate-limited command) as the root's.
- **Setpoint preview** (on by default since 2026-10-24): each command is carried from its stamp to the end of the physics
  step, q + vΔ + ½aΔ² and v + aΔ, Δ capped at 0.1 s, only under external control, clamped to the joint limits.

Torques reach the drive as a shift of its position target by torque / stiffness, so the drive's force limit still
caps the actuator's total.

To give the actuator the reference's acceleration, the JTC commands `[position, velocity, acceleration]` and
`UnityArmSystem` sends `/arm/command` as a `trajectory_msgs/msg/JointTrajectory` holding exactly one point
(positions, velocities, accelerations; the header stamp is the controller time). It replaces the
`sensor_msgs/msg/JointState` command packet of ADR 0004. `/arm/state` is unchanged.

## Context and constraints

- ROS owns the reference and its timing (ADR 0004); Unity owns physics. The actuator must not plan.
- The feedback-only drives left the arm off its reference by 0.005-0.018 rad in the shortfall runs, about half of
  the B5 clearance shortfall (Phase 4B-B campaign 1: 12 of 25 mm in the worst run).
- Measured live (2026-10-23), the error was a constant 20-24 ms per unit speed: commands arrive one physics step
  after their stamp and the drive reaches a target at the end of the next step, so the arm ran 40 ms late. The
  static sag (elbow 0.0075 rad in carry) was the smaller part. Inertial torques are small in slow moves (shoulder
  and elbow coupled inertias 7.6, 2.7 and 4.3 kg m² largely cancel when they turn oppositely).
- Unity's helpers do not serve a floating mobile root: `GetJointGravityForces` returns zero (free fall), and
  writing `ArticulationBody.jointForce` every step held the arm but made every moving joint lag (cause unknown).

## Options considered

- **Raise the collision margin instead.** Rejected: the 1.05 m doors need the panel inside 0.065 m (campaign 1:
  room R4 0/4).
- **Model the lag in the MPC.** Rejected for now: it treats the symptom and couples the planner to one simulator's
  latency.
- **Compute the feedforward in ROS and send effort.** Cleaner ownership, but it changes the command interface to
  effort and needs a dynamics library in the control path; the actuator-internal feedforward of real servo drives
  is the closer analogue. Kept as a revisit option.
- **Differentiate the commanded velocity in Unity instead of an acceleration channel.** No contract change, but
  one step late and sensitive to packet timing. Rejected in favour of the exact JTC acceleration.
- **Unity's `GetJointGravityForces` / `jointForce`.** Rejected on the evidence above.

## Consequences and risks

- The arm runs on the reference's time: the time-aligned elbow error p95 fell from 0.0134 to 0.0004 rad (8 s move)
  and from 0.0269 to 0.0006 rad (4 s move, with motion feedforward).
- The JTC's own path error compares the current reference with a state report one step old, so it shows a floor of
  about speed x 20 ms even when the arm is on time; tracking is judged on the time-aligned error from Unity's
  recorder.
- `/arm/command` changed type: every consumer moved with it (`unity_arm_system`, `measure_transport`, Unity's
  `ArmRosTransport`, the mock-hardware description in `arm_control.launch.py`). Old bags hold the old type.
- Every baseline that moves the arm behaves differently with these switches on: the comparability log records it,
  and the final baseline run uses one arm setting for all baselines.
- A frame hitch (commands paused 80 ms) still disturbs the arm; preview does not extrapolate beyond 0.1 s.

## Validation

EditMode tests: statics, a pendulum, a two-link arm against its closed form, base acceleration as an opposite
gravity, base yaw and centripetal signs, the preview formula and its limits, the command packet and its
validation. ROS tests: the command packet and the limit check; the mock-hardware MoveIt tests (51/51). Live: a
dynamics probe (unit accelerations give the loaded inertias of the qualification, 7.6 and 2.7 kg m²), recorded
home-to-carry moves, the whole-body carry qualification and the Phase 4B-B campaign 1b.

## Revisit

- Motion and preview became defaults by the user's decision after the carry qualification (2026-10-24);
  `--no-arm-motion-feedforward` / `--no-arm-setpoint-preview` give the earlier arm.
- If the physics step, the controller rate or the transport changes, re-measure the command delay (the recorder's
  `setpoint_preview_s`).
- If the MPC sends planned accelerations itself, or the actuator moves to ROS-side effort control.
- If Unity's articulation API exposes reliable floating-base gravity compensation or joint forces without lag.
