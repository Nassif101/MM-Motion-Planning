# Tool Ownership

## Primary Planes

| Capability | Primary interface | Notes |
|---|---|---|
| Connect to robot or ROS bridge | ROS-MCP | Confirm connection and ROS version before live work. |
| Topics, types, publishers, subscribers, samples, images | ROS-MCP | Resolve the type and inspect message fields before publish. |
| Nodes and node details | ROS-MCP | Treat returned graph data as current evidence. |
| Services and lifecycle transition services | ROS-MCP | Resolve the service type and request schema first. |
| Actions, goals, status, cancellation | ROS-MCP | Resolve action details; verify terminal state independently. |
| Parameters and descriptors | ROS-MCP | Read details before setting; read back afterward. |
| Nav2 and MoveIt runtime behavior | ROS-MCP | Use their actual actions, services, topics, and parameters. |
| ros2_control runtime behavior | ROS-MCP | Use discovered controller-manager services and state topics. |
| Source, packages, interfaces, launch files | Filesystem and terminal | Inspect and edit project-owned artifacts only. |
| Build, test, launch, run, environment, Git | Terminal | Keep long-running processes observable and cancellable. |
| Logs and rosbag files | Terminal/filesystem | Use the bundled log helper when structured host-log filtering helps. |

## Complementary and Fallback Capabilities

- TF2 buffer semantics are not equivalent to reading raw `/tf` messages. Prefer ROS-MCP for graph and sample evidence, then use a scoped host command such as `ros2 run tf2_ros tf2_echo` or `ros2 run tf2_tools view_frames` only when lookup, interpolation, or tree semantics are required.
- Use standard `ros2 doctor`, `ros2 multicast`, and `ros2 bag` commands for host environment, DDS-network, and bag operations. These are development diagnostics, not a second live control plane.
- Use ROS-MCP generic service and action tools for lifecycle, controller manager, Nav2, and MoveIt. Do not add a dedicated `rclpy` transport merely to provide friendlier command names.
- If ROS-MCP cannot express a required QoS subscription or an introspection API is absent, record the gap and use the narrowest available ROS CLI command. Verify the conclusion through ROS-MCP or another independent observation where possible.

## Do Not Add

- A universal ROS CLI wrapper
- A persistent or fast daemon
- Duplicate topic, service, action, node, parameter, Nav2, controller, or preflight clients
- Hidden tmux process ownership
- Chat, Discord, notification, or unrelated integrations
- Static profiles that override contradictory live evidence

Project documentation supplies expected contracts. It does not authorize guessing, and it does not outrank live evidence.
