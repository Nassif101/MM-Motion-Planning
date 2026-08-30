# Project agent guidance

## Architecture ownership

- Run the Unity Editor natively on the host and control it through Unity CLI plus the `com.unity.pipeline` package. Do not configure or use Unity MCP.
- Run ROS 2 Jazzy, Gazebo Harmonic, Nav2, MoveIt 2, ROS-TCP-Endpoint, and ROS-MCP in the Linux development container.
- ROS 2 owns robot navigation, motion planning, and control policy. Unity owns rendering, physics, simulated sensors, and visualization.
- Preserve verified ROS topic, frame, clock, QoS, and joint-order contracts across the Unity/ROS boundary.

## Skill routing

Prefer the project-owned Unity/ROS skills under `.agents/skills` and the official Unity CLI and package-management skills.

The project uses HDRP. Do not use URP-specific skills such as `urp-postprocessing` or `validate-urp-render-graph-renderer-feature` unless the project render pipeline is deliberately migrated first.

Do not use Unity AI Navigation for robot navigation or motion planning. ROS 2 owns navigation and planning.

The retained upstream Unity skills are `unity-cli`, `unity-package-management`, and `physics-3d-collision`. Do not add general game-product, UI, rendering, localization, audio, monetization, multiplayer, or new-project skills unless the repository gains that concrete scope.

## Cross-platform constraints

- Supported development hosts are Windows x86-64 with Docker Desktop/WSL2 and macOS Apple Silicon.
- Never commit host-absolute paths, credentials, Unity `Library/`, or ROS `build/`, `install/`, and `log/` output.
- Shell scripts use LF endings. PowerShell scripts use paths derived from the repository root.
- Prefer native `linux/amd64` and `linux/arm64` container images; do not force AMD64 emulation on Apple Silicon for normal development.
