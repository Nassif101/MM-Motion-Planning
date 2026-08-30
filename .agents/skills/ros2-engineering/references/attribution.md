# Attribution and Adaptation Record

This project-owned skill is derived in part from concepts and workflows in:

- `adityakamath/ros2-skill`, commit `bcd7040238dd9b0acbe0c575df146b28490962d2`
- Source: https://github.com/adityakamath/ros2-skill
- Upstream license: Apache License 2.0
- Upstream copyright notices: Copyright 2026 Aditya Kamath, Kamath Robotics; copyright 2024 Jungsoo Lee as stated in the upstream license appendix.

The project version was rewritten rather than copied as a wholesale snapshot. It retains the evidence-driven Resolve → Act → Verify model, no-guessing rule, REP-103/105 awareness, command-ownership checks, motion preflight, independent verification, and failure classification.

It deliberately changes the upstream tool architecture:

- ROS-MCP is the primary live graph and control plane.
- Static profiles and documentation do not override contradictory live evidence.
- Topic, service, action, node, parameter, Nav2, control, and preflight clients are not bundled.
- The universal wrapper CLI, persistent/fast daemon, tmux ownership, and Discord integration are omitted.
- TF2-specific host commands are narrow fallbacks rather than a second general ROS transport.
- Only a read-only host log inspector is bundled as a complementary helper.

The bundled log inspector is a new project implementation informed by the upstream `ros2_logs.py` capability. It uses no copied upstream source and remains read-only and standard-library-only.

The authoritative live ROS integration is `robotmcp/ros-mcp-server`: https://github.com/robotmcp/ros-mcp-server
