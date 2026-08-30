# Cross-platform ROS 2 development container

Unity runs natively on the host. The container supplies the reproducible Linux robotics environment: ROS 2 Jazzy, Gazebo Harmonic, Nav2, MoveIt 2, ROS-TCP-Endpoint, rosbridge, and ROS-MCP.

The base image and installed packages support native `linux/arm64` on Apple Silicon and native `linux/amd64` on Windows/WSL2. Normal development must not force an emulated platform.

## Portable default

Start Docker Desktop, then use **Dev Containers: Reopen in Container**. The checked-in `devcontainer.json` uses only `docker-compose.yml`, so it works on both hosts and exposes GUI applications through noVNC.

On first creation, the Dev Container imports the runtime repositories declared by `ros2_ws/clearpath-runtime.repos`, refreshes apt and rosdep indexes, and installs declared workspace dependencies. Those imported repositories are generated workspace inputs and are intentionally ignored by the parent Git repository. Clearpath's generator-test repository is excluded because it is not required to run this simulation and has additional unpublished test-only dependencies.

Inside the container:

```bash
cb
colcon test
colcon test-result --verbose
novnc
```

Open `http://localhost:6080/vnc.html` and launch GUI applications such as `rviz2` in the container shell.

On macOS, RViz appears inside this browser desktop rather than as a native Aqua window. Run `novnc` once per recreated container, open the printed URL, then run `rviz2` from the container terminal. Direct host-window forwarding is reserved for the optional Windows/WSLg overlay.

Build, install, and log outputs are named Docker volumes. This avoids compilation-heavy writes through the macOS bind mount. `ccache` is also persisted.

## Unity and ROS connections

Start the ROS-TCP endpoint inside the container:

```bash
ros-tcp-server
```

The native Unity Editor connects to `127.0.0.1:10000` on both Docker Desktop platforms.

ROS-MCP uses rosbridge. Start rosbridge before opening a Codex session that needs the live ROS graph:

```bash
rosbridge
```

The project `.codex/config.toml` launches `ros-mcp` inside this running service. Unity is controlled through Unity CLI/Pipeline, not Unity MCP.

## Windows WSLg and NVIDIA overlays

The default noVNC setup is portable. WSL2 users can opt into direct WSLg forwarding:

```bash
docker compose \
  -f .devcontainer/docker-compose.yml \
  -f .devcontainer/docker-compose.wslg.yml \
  up -d
```

Add `.devcontainer/docker-compose.nvidia.yml` only after this succeeds:

```bash
docker run --rm --gpus all nvidia/cuda:12.5.1-base-ubuntu24.04 nvidia-smi
```

The NVIDIA and WSLg overlays are intentionally never loaded by the portable default.

## Builds and caches

Build the native host architecture:

```bash
docker compose -f .devcontainer/docker-compose.yml build
```

Validate both image architectures without forcing emulation during ordinary development:

```bash
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --target development \
  --file .devcontainer/Dockerfile \
  --output type=cacheonly \
  .
```

For constrained machines, set `COLCON_PARALLEL_WORKERS=1` or `COLCON_BUILD_MODE=queued` before rebuilding the container.

To deliberately discard generated ROS caches:

```bash
docker compose -f .devcontainer/docker-compose.yml down
docker volume ls --filter name=mm-motion-planning
```

Remove only the specific listed volumes you intend to rebuild.
