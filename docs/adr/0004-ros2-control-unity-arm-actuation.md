# ADR 0004: ros2_control owns arm trajectory execution

Date: 2026-09-06. Status: accepted for the commissioned simulation envelope.

## Decision

Use Jazzy `controller_manager` and `joint_trajectory_controller/JointTrajectoryController`
with `mobile_manipulator_control/UnityArmSystem` as a six-joint simulated hardware plugin.
It exposes position/velocity command and state interfaces. ROS-TCP-Endpoint transports
instantaneous named `sensor_msgs/msg/JointState` command/state packets between the plugin
and a project-owned Unity actuator. No effort state interface is fabricated.

Unity applies finite Force ArticulationDrives in FixedUpdate, captures actual startup
positions for HOLD, validates names/limits/units, and captures current position after
0.5 seconds without valid commands. JTC owns interpolation, synchronization, action
success, tolerances, cancellation and replacement. A zero-velocity instantaneous target
is an actuator HOLD; it does not imply a ROS action has succeeded.

Keep Unity's existing ten-joint `/joint_states` publisher. The arm broadcaster publishes
under its own namespace. An isolated `/arm/robot_description` contains the augmented
control model; its publisher does not publish TF. The existing description launch
remains the only URDF TF owner. The base controller code is unchanged.

## Alternatives and reasoning

- A Unity trajectory controller would duplicate standard timing and action semantics
  and complicate future MoveIt integration. Rejected.
- Direct transform/kinematic actuation or gravity removal would hide actuator and
  payload dynamics. Rejected.
- A second transport stack or custom command message was unnecessary: the existing
  Connector already supports named position/velocity JointState packets. The command
  topic is explicitly actuator setpoints, not measured state.
- Publishing another complete `/joint_states` stream would introduce competing
  authority. Retaining the established stream and a local arm broadcaster avoids it.
- Keeping the old 120 Nm shoulder assumption left only 14% above the calculated
  105.43 Nm gravity bound and saturated in loaded return trials. A finite 160 Nm
  simulation assumption provides about 52% static reserve and passed loaded returns.
  Reducing base acceleration alone cannot increase stationary gravity torque reserve.

## Consequences and evidence

Per-joint gains are engineering estimates followed by measurement. The 20 ms physics
step is retained. Default 6/1 solver iterations failed loaded extension tolerance;
12/4 improved droop with unchanged gains. A 10 ms trial did not resolve the provisional
120 Nm loaded-return limitation, so it was not retained. Solver settings are applied
at runtime because those Unity properties are not serialized.

TCP is not a real-time bus. With 50 Hz configuration, commissioning observed about
28–30 command packets/s and 46–47 state packets/s; callback gaps reached 0.304 seconds
under Editor activity. Scene resets and feedback faults require a controller-manager
restart. The local watchdog remains independent of JTC.

The 1.2 m square panel contacts the ground in the tested fully horizontal straight-arm
pose; that result is excluded from unsupported gravity-HOLD acceptance. The contact-free
1.3 rad shoulder extension is the commissioned difficult pose. No full-workspace or
hardware-rated payload claim follows from these tests.

## Revisit

Revisit torque assumptions, payload profiles, trajectory operating limits and base
acceleration together when payload mass/COM changes. Investigate explicit gravity
feedforward if measured droop is unacceptable for the thesis task; it is not required
for the current 0.04 rad goal envelope. Improve transport scheduling before faster
trajectories or stronger timing claims. MoveIt can later use the same
`/arm_controller/follow_joint_trajectory` action. A future MPC controller may claim
compatible actuator interfaces without introducing planning into Unity.

See [the controller document](../unity-arm-controller.md) and its quantitative evidence.
