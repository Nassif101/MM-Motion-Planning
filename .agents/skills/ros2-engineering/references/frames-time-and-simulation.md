# Frames, Time, and Simulation

## Coordinate Conventions

- Follow REP-103 units and axis conventions unless the project explicitly documents a conversion boundary.
- Follow REP-105 frame roles for mobile platforms: `map` is globally corrected, `odom` is locally continuous, and the robot body frame is commonly `base_link`; verify the actual names.
- Treat `frame_id` as a semantic contract, not a label that can be changed without transforming the data.
- Verify transform direction. A transform from A to B is not interchangeable with B to A.
- Normalize quaternions and avoid Euler-angle assumptions at API boundaries.

## TF Diagnosis

1. Discover `/tf` and `/tf_static` endpoint evidence through ROS-MCP.
2. Resolve the source and target frames from actual messages and project contracts.
3. Check timestamp freshness and the active clock.
4. When interpolation, latest-common-time, authority, or full-tree semantics are required, use a scoped TF2 host tool as described in `tool-ownership.md`.
5. Distinguish missing transform, disconnected tree, extrapolation into past or future, stale publisher, duplicate authority, and incorrect frame convention.

Raw topic inspection alone does not reproduce a TF2 buffer.

## Time

- Determine whether `use_sim_time` is set on every relevant node.
- Confirm `/clock` exists and advances when simulation time is expected.
- Compare message stamps in ROS time, not host wall time, when sim time is active.
- Handle paused simulation, reset or backward time jumps, and accelerated or decelerated time.
- Use ROS timestamps and configured tolerances for cross-process sensor/command correlation; do not assume Unity render frames, Unity physics ticks, and ROS callbacks are synchronous.

## ROS–Unity Contract

- ROS owns motion planning, command generation, arbitration, and control policy.
- Unity owns the 3D environment, physics realization, simulated sensors, and execution of defined controller commands.
- Define each bridge contract with topic or service name, exact type, direction, frame, units, QoS, rate, timestamp source, joint ordering, and reset/startup behavior.
- Unity physics work belongs on its physics timestep; ROS messages should carry time explicitly and be buffered or sampled deliberately.
- Verify both endpoints independently. A valid ROS graph endpoint does not prove the Unity component is configured correctly, and a configured Unity component does not prove the live ROS contract matches.
- Treat scene reset, clock reset, reconnection, delayed messages, and command timeout as first-class states.
