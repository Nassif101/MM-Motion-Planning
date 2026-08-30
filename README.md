# Dynamic Motion Planning for Transporting Bulky Objects in Constrained Construction Environments

This repository contains the software, simulation assets, configuration files, and supporting tools developed for a master's thesis on dynamic motion planning for mobile robots transporting bulky objects in constrained construction environments.

The project focuses on robot-based object transport tasks such as moving wall panels, beams, or other prefabricated construction components through narrow, obstacle-rich environments while considering static and dynamic obstacles.

## Thesis Context

Mobile robots used in construction must often transport objects that are large relative to the available free space. This creates motion-planning challenges when the robot must move through constrained passages such as corridors, scaffolding, temporary site layouts, or partially built structures.

The thesis investigates dynamic motion-planning methods that support safe and feasible transport of bulky objects using onboard sensing and robot motion planning.

## Objectives

The repository supports the following thesis objectives:

1. Review existing motion-planning approaches for robots transporting bulky objects in construction environments.
2. Design a methodology for dynamic motion planning in constrained construction scenarios.
3. Implement a motion-planning framework based on the developed methodology.
4. Validate the framework through simulated transport experiments.
5. Analyze the results and evaluate the performance, strengths, and limitations of the implemented framework.

## Scope

The project is intended to support research and experimentation related to:

- mobile robot navigation in constrained construction-like environments
- transport of bulky or extended objects
- static and dynamic obstacle avoidance
- onboard sensing for environment perception
- simulation-based validation
- integration of navigation, manipulation, and motion-planning components

## Repository Structure

- `motion-planning-sim/` — Unity 6 HDRP simulation, sensors, and ROS adapters
- `ros2_ws/` — ROS 2 Jazzy workspace and robot description
- `.devcontainer/` — native ARM64/AMD64 ROS development environment
- `.agents/skills/` — project-owned Unity/ROS engineering guidance
- `tools/` — deterministic asset and project tooling

## Development Environment

The project targets the following software environment:

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- MoveIt 2
- Nav2
- RViz2

Unity Editor runs natively on the host. ROS, Gazebo, Nav2, MoveIt, ROS-TCP-Endpoint, and ROS-MCP run in the Linux development container.

Supported development hosts:

- macOS on Apple Silicon using native `linux/arm64` containers
- Windows x86-64 using Docker Desktop with WSL2 and native `linux/amd64` containers

## Getting Started

### Open in Dev Container

Initialize the repository once after cloning:

```bash
git submodule update --init --recursive
```

Start Docker Desktop, then run:

```text
Dev Containers: Reopen in Container
```

The default uses noVNC and is portable. WSLg and NVIDIA support are opt-in overlays documented in [the Dev Container guide](.devcontainer/README.md).

The Dev Container automatically imports `ros2_ws/clearpath-runtime.repos` and runs `rosdep`; do not commit the materialized dependency repositories into this parent repository.

Host setup checks can also be run directly:

```bash
./scripts/setup-host.sh
./scripts/setup-host.sh --install-editor --build-container
```

Windows PowerShell equivalents are `./scripts/setup-host.ps1` and `./scripts/setup-host.ps1 -InstallEditor -BuildContainer`.

### Build the Workspace

Inside the development environment:

```bash
cd "$ROS_WS"
cb --packages-skip-regex '^clearpath_generator_.*_tests$'
srcws
```

The container provides `cb`, which adds the configured build type, parallelism, and persistent `ccache` automatically.

### Clean Build

The ROS `build`, `install`, and `log` directories are Docker volume mount points and should not be removed with `rm -rf`. To deliberately discard their contents, stop the container and follow the named-volume procedure in [the Dev Container guide](.devcontainer/README.md#builds-and-caches).

## Running the System

For the complete terminal-by-terminal startup sequence, ROS-MCP setup, RViz/noVNC instructions, verification commands, and package launch reference, see [Running the Unity and ROS 2 stack](docs/running-the-stack.md).

Inside the container, start the Unity ROS-TCP endpoint:

```bash
ros-tcp-server
```

Start rosbridge when using ROS-MCP:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  address:=0.0.0.0 \
  port:=9090
```

On the host, install the exact Unity version and Pipeline package through Unity CLI:

```bash
unity install 6000.5.2f1 --architecture arm64 --yes --accept-eula  # Apple Silicon
unity pipeline install --project-path ./motion-planning-sim
unity open ./motion-planning-sim --editor-version 6000.5.2f1
```

On Windows x86-64 use `--architecture x86_64`. Do not copy or share Unity's `Library/` directory between hosts.

Unity MCP is not used. Unity CLI communicates with the open Editor through `com.unity.pipeline`.

## Documentation

Documentation is developed alongside the implementation. Relevant documentation may include:

- system architecture
- package descriptions
- simulation setup
- experiment setup
- parameter documentation
- evaluation procedure
- known limitations

Documentation files should be placed in the `docs/` directory where appropriate.

## Development Notes

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

## Acknowledgement

This repository is developed as part of a master's thesis in the field of robotics, motion planning, and construction automation.
