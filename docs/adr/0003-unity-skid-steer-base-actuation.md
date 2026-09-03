# ADR 0003: Unity skid-steer base actuation

## Status

Accepted for the construction-site physical-control baseline.

## Context

ROS 2 owns navigation and future whole-body control. Unity owns PhysX, contact, sensors, and visualization. The same low-level base interface must accept commands from manual tools, Nav2, or a later MPC/QP without embedding high-level policy in Unity.

## Options considered

1. Move the chassis transform or assign chassis velocity directly.
2. Replace the imported wheels with `WheelCollider` components.
3. Add `ros2_control` and make it the Unity actuator boundary.
4. Subscribe to Twist in Unity and command the four existing revolute `ArticulationBody` drives through finite torque.

## Decision

Choose option 4.

- `/cmd_vel` is the generic `geometry_msgs/msg/Twist` command boundary.
- Unity clamps and rate-limits planar chassis commands, maps them through skid-steer inverse kinematics, and ratio-scales wheel saturation.
- A 0.5 s monotonic watchdog commands a rate-limited stop.
- Wheel drives use `ArticulationDriveType.Force`, zero stiffness, finite damping and 20 N m force limit.
- Existing continuous wheel joints and colliders remain authoritative; project-owned wheel and floor Physics Materials provide conventional isotropic PhysX friction.
- Physical wheel spacing remains 0.64 m. A separately named 1.50 m effective track is an empirical first-order compensation for scrub.
- No custom tire model is introduced.

## Consequences

- High-level controllers can change without a Unity actuator rewrite.
- Payload mass and inertia naturally affect response because motion comes from wheel torque and contact.
- The watchdog and actuator-side limits protect against stale or unreasonable publishers, but ROS remains responsible for arbitration and higher-level safety.
- Conventional isotropic friction cannot independently tune longitudinal grip and lateral scrub. The effective track is curvature-dependent: the selected compromise fits the commissioned tests but is not a universal tire model.
- The arm needs an independent controller. Passive arm joints can swing when the base accelerates; the base actuator deliberately does not claim them.

## Validation

- EditMode tests cover clamping, acceleration/deceleration, reversal, kinematics, SI-to-drive conversion, timeout boundaries, and ratio-preserving saturation.
- Live ROS tests cover forward, reverse, both yaw directions, both forward arcs, command loss, and contact response.
- Unity telemetry reports rate-limited commands, wheel targets and measurements, chassis speed/yaw, watchdog, and saturation.

## Revisit when

- Hardware motor/transmission data replaces the provisional torque assumption.
- An anisotropic or load-sensitive tire model is required.
- Identified track width varies enough with speed, payload, or curvature to justify a richer model.
- Command arbitration or ros2_control ownership is deliberately moved across the ROS-Unity boundary.
