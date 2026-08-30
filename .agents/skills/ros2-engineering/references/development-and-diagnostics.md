# Development and Diagnostics

## Source and Packages

- Inspect workspace layout, package manifests, build type, interface definitions, launch files, configuration, generated/vendor boundaries, and existing tests before editing.
- Build the narrowest affected package set first, then its dependents when interface or ABI changes require it.
- Source the correct underlay and overlay in the process that runs ROS commands.
- Use `colcon test` and inspect `colcon test-result --verbose`; a successful build is not a successful test.
- For interface changes, rebuild producers and consumers and verify the runtime type after sourcing the new overlay.

## Launch and Processes

- Inspect declared launch arguments, includes, namespaces, remaps, parameters, and conditions before starting a launch.
- Use observable terminal process sessions with captured output and an explicit cancellation path.
- Avoid hidden duplicate launches. Check existing nodes and processes before starting another stack.
- On shutdown, verify processes and their graph entities disappear.
- Do not bundle another tmux or daemon abstraction into this skill.

## Evidence-Based Diagnosis

Collect the smallest evidence set that distinguishes likely causes:

- ROS-MCP connection and version
- Relevant node, topic, service, action, and parameter details
- Timestamps, QoS, frames, rates, and command ownership
- Lifecycle, controller, and hardware state
- Launch output, ROS logs, build/test results, and environment

Form one falsifiable hypothesis at a time. Prefer a read-only observation or bounded test. Record what was expected, what occurred, and which layer failed: code/build, process, discovery/DDS, transport/bridge, interface, lifecycle/control, planning, physics, or sensor data.

Classify a failed operation before retrying:

- graph or discovery
- type or schema
- QoS
- lifecycle
- controller or hardware
- TF or time
- process or launch
- package, build, or environment
- networking or DDS
- application or planning logic

Capture the original error, inspect evidence for that class, apply the smallest corrective action, and verify again.

## Logs

- Inspect process output first for a just-started node.
- Use `scripts/ros_log_inspect.py` for structured queries across host log files.
- Correlate by ROS timestamp, node, severity, goal/request time, and launch run.
- Do not treat the absence of a parsed standard-format line as proof that no error occurred; some backends and applications use other formats.

## Bags

- Use standard `ros2 bag info`, `record`, and `play` commands through the terminal.
- Inspect exact topics, types, QoS metadata, storage plugin, time range, and clock behavior.
- Record the minimum useful topic set and protect sensitive or high-volume data.
- During playback, decide explicitly whether `/clock` is published and whether consumers use simulation time.
- A bag is evidence from its recording interval, not proof of current live state.

## Host and DDS Diagnostics

- Use `ros2 doctor --report` for environment and middleware checks.
- Use `ros2 multicast send/receive` when testing multicast reachability is relevant.
- Check `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, network namespace, firewall, discovery server, rosbridge endpoint, and overlay sourcing.
- Keep host diagnostics separate from live robot control; return to ROS-MCP to confirm the graph after resolving transport issues.
