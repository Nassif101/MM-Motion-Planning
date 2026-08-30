---
name: ros2-engineering
description: Guide evidence-driven ROS 2 engineering with ROS-MCP as the primary live graph and robot control interface. Use for ROS 2 runtime inspection, diagnosis, control, TF, QoS, lifecycle, ros2_control, Nav2 or MoveIt, sensors, launch and process management, logs, rosbag, package diagnostics, and ROS architecture decisions. Do not trigger for ordinary C++ or Python edits merely because they import ROS unless the task concerns ROS interfaces, runtime behavior, or architecture.
---

# ROS 2 Engineering

Treat ROS-MCP as the authoritative live ROS interface. Use the terminal and filesystem for development artifacts and host processes. Add a narrower fallback only when neither primary plane exposes the required semantics.

## Apply Precedence

Resolve conflicts in this order:

1. User instructions
2. Project architecture and ownership
3. Safety constraints
4. Current live evidence
5. Documented contracts
6. Generic guidance in this skill

Live evidence can show that a documented contract is stale; report the mismatch instead of silently choosing one.

## Choose the Control Plane

1. Use ROS-MCP for live topics, nodes, services, actions, parameters, types, sensor samples, images, and robot state or commands.
2. Use terminal and filesystem tools for packages, source, interfaces, build, test, launch processes, logs, bags, environment, and Git.
3. Use a complementary helper only for a capability identified in `references/tool-ownership.md`.
4. Use direct ROS CLI or `rclpy` as a scoped fallback only when ROS-MCP and the development plane cannot supply the required semantics. State why, keep it narrow, and return to ROS-MCP for verification.

Do not introduce a parallel general-purpose ROS client, persistent daemon, or wrapper CLI.

## Work Resolve → Act → Verify

### Resolve

- Discover exact names, types, fields, frames, QoS, lifecycle state, controller state, and command ownership from the live system when relevant.
- Inspect only the package files and project contracts needed to interpret the task.
- Never invent a topic, service, action, parameter, frame, joint order, controller interface, or message field.
- Cache stable facts during the task, but refresh volatile state immediately before consequential actions.
- Verify Unity and ROS endpoints independently for cross-boundary tasks; report topic, type, frame, rate, QoS, clock, or joint-order mismatches before implementation.

### Act

- Take the smallest action that tests the hypothesis or accomplishes the requested change.
- Separate read-only discovery from state changes.
- Before motion or hardware-affecting work, apply `references/motion-safety.md`.
- Respect the ROS/Unity boundary: ROS owns planning, command generation, and control policy; Unity supplies the simulated world and sensor data and executes defined controller commands unless the project explicitly establishes another contract.

### Verify

- Verify through independent observable state, not only a successful tool return.
- Re-read changed parameters, controller or lifecycle state, action status, resulting pose or velocity, expected topic activity, and relevant logs.
- Classify failures from evidence before retrying. Do not repeat motion, service, or lifecycle changes blindly.

## Route Detailed Work

- Read `references/tool-ownership.md` before selecting tools or fallbacks.
- Read `references/runtime-workflows.md` for topics, services, actions, parameters, lifecycle, ros2_control, Nav2, MoveIt, QoS, and sensors.
- Read `references/motion-safety.md` before motion, actuator, controller, navigation, or other hardware-affecting commands.
- Read `references/frames-time-and-simulation.md` for TF, REP-103/105, timestamps, `/clock`, simulation timing, or Unity integration.
- Read `references/development-and-diagnostics.md` for packages, builds, tests, launches, processes, logs, bags, DDS networking, and evidence-based diagnosis.
- Read `references/attribution.md` when adapting this skill or evaluating what was retained from upstream.

## Use the Complementary Log Helper

Run `python3 scripts/ros_log_inspect.py --help` from this skill directory to inspect host ROS log files without requiring a live graph. It is read-only, standard-library-only, and does not replace ROS-MCP.

Prefer normal project build and test commands over adding wrappers. Preserve generated/vendor boundaries and existing package conventions.
