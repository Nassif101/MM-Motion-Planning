# Running the Unity and ROS 2 stack

This is the operator runbook for the current mobile-manipulator simulation. Unity runs natively on the host. ROS 2 Jazzy, ROS-TCP-Endpoint, rosbridge, ROS-MCP, and RViz run in the development container.

## Daily startup

Start Docker Desktop, open the repository in VS Code, and run **Dev Containers: Reopen in Container**. This is the normal workflow on both macOS Apple Silicon and Windows/WSL2.

### 1. Build and source the ROS workspace

In a container terminal:

```bash
cd "$ROS_WS"
cb --packages-skip-regex '^clearpath_generator_.*_tests$'
srcws
```

`cb` uses symlink install, the configured parallelism and build type, and the persistent `ccache`. The skip expression is harmless on a fresh checkout and also avoids stale Clearpath generator-test packages that may remain in an older workspace.

Run the build again after changing ROS package source or launch files. A new container terminal automatically sources an existing workspace install; use `srcws` in the current terminal immediately after rebuilding.

### 2. Start the Unity ROS-TCP endpoint

Keep this running in its own container terminal:

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args \
  -p ROS_IP:=0.0.0.0 \
  -p ROS_TCP_PORT:=10000
```

The shorter `ros-tcp-server` container alias expands to this exact command. It listens on `0.0.0.0:10000`. The Unity project is already configured to connect to `127.0.0.1:10000` through Docker Desktop's forwarded port.

### 3. Start the ROS-side mobile-manipulator description

Keep this running in another container terminal:

```bash
ros2 launch mobile_manipulator_description simulation.launch.py
```

This launch intentionally accepts no launch arguments; `use_sim_time` is set to `true` inside the launch file.

This launch owns:

- the static `map -> odom` transform
- `robot_state_publisher` and the URDF-derived robot transforms
- simulation-time configuration for its ROS nodes

Unity owns `/clock`, `/joint_states`, `/livox/lidar`, and the dynamic `odom -> base_footprint` transform. Do not start another publisher for these same contracts.

### 4. Start rosbridge for ROS-MCP

Keep this running in another container terminal:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  address:=0.0.0.0 \
  port:=9090
```

The shorter `rosbridge` alias uses port `9090`; the expanded command above also makes the bind address explicit. Start it before opening or restarting a Codex session that needs the live ROS graph.

Do **not** run `ros-mcp --transport=stdio` manually during normal use. [`.codex/config.toml`](../.codex/config.toml) already tells Codex to start that process inside the running `ma-robot-sim` container. ROS-MCP connects to rosbridge at `127.0.0.1:9090` by default.

When asking Codex to inspect ROS for the first time in a session, tell it to connect to `127.0.0.1:9090`, then have it confirm the ROS version and discover the live graph before issuing commands.

### 5. Open and run Unity on the host

Run this from a **host** terminal at the repository root, not from inside the container:

```bash
unity open ./motion-planning-sim --editor-version 6000.5.2f1 --args "-automated"
```

The explicit version prevents an accidental project upgrade. The project is pinned to Unity `6000.5.2f1`; the CLI selects the native host architecture automatically.

In the Editor, open `Assets/Scenes/ConstructionSiteV1.unity` and enter Play mode after `ros-tcp-server` is listening.

The same action can be driven from a host terminal after the Editor and Pipeline connection are ready:

```bash
unity status
unity command editor_play --project-path ./motion-planning-sim
```

Unity MCP is not used. Unity CLI communicates with the native Editor through the installed `com.unity.pipeline` package.

### 6. Start static-map global planning

After the description stack and Unity are publishing the complete `map -> odom -> base_footprint` TF chain, keep this running in another container terminal:

```bash
ros2 launch mobile_manipulator_navigation global_planning.launch.py
```

This launch loads the map exported from `ConstructionSiteV1`, publishes it on `/map`, creates the static global costmap, and exposes Nav2's path-computation actions. It starts only `map_server`, `planner_server`, and their lifecycle manager. It does not start AMCL, a controller server, or base command execution.

If the planner remains in activation while reporting a missing `map -> base_footprint` transform, confirm that step 3 is running and Unity is in Play mode. The map server can publish `/map` without robot TF, but the planner's global costmap cannot activate without the complete chain.

To regenerate the map after changing authoritative scene obstacles, run from a host terminal:

```bash
unity command export_nav2_map --project-path ./motion-planning-sim
unity command validate_nav2_map --project-path ./motion-planning-sim
```

Rebuild and source the ROS workspace afterward so the installed package receives the updated artifact. The complete export contract is in [`mobile_manipulator_navigation/docs/unity-map-export.md`](../ros2_ws/src/mobile_manipulator_navigation/docs/unity-map-export.md).

## RViz on macOS

RViz runs in the container and appears in a browser-based Linux desktop; it does not open as a native macOS window.

Run noVNC once in a container terminal:

```bash
novnc
```

This command starts Xvfb, Openbox, x11vnc, and the noVNC proxy in the background and then returns to the prompt. Do not expect it to remain attached to the terminal. Running it again safely reuses the existing session.

Open [http://localhost:6080/vnc.html?autoconnect=1&resize=scale](http://localhost:6080/vnc.html?autoconnect=1&resize=scale) in the host browser.

For the mobile-manipulator stack with its configured RViz display, use this command **instead of** `simulation.launch.py` in step 3:

```bash
DISPLAY="${NOVNC_DISPLAY:-:99}" \
  ros2 launch mobile_manipulator_description simulation_rviz.launch.py
```

That launch includes the same description stack and starts RViz with `map` as its fixed frame and a `/livox/lidar` PointCloud2 display. Starting both mobile-manipulator launch files would duplicate the TF and state-publisher nodes.

The portable Windows setup can use the same noVNC workflow. The optional WSLg/NVIDIA overlays are documented in [the Dev Container guide](../.devcontainer/README.md).

## Verify the live system

Run these in another container terminal after Unity enters Play mode:

```bash
ros2 node list
ros2 topic list | sort
ros2 topic info --verbose /clock
ros2 topic info --verbose /joint_states
ros2 topic info --verbose /livox/lidar
ros2 topic info --verbose /map
ros2 topic hz /clock
ros2 topic hz /joint_states
ros2 topic hz /livox/lidar
ros2 run tf2_ros tf2_echo map base_footprint
```

Each rate or TF command keeps running until you press `Ctrl-C`. Check them one at a time. Expected Unity publications are:

| Interface | Expected role |
| --- | --- |
| `/clock` | Unity simulation clock |
| `/joint_states` | Simulated robot joint state |
| `/livox/lidar` | Livox Mid-360 `sensor_msgs/msg/PointCloud2` |
| `/map` | Unity-derived static `nav_msgs/msg/OccupancyGrid`, published by Nav2 `map_server` |
| `/tf` | Unity's dynamic `odom -> base_footprint` plus ROS robot transforms |
| `/tf_static` | Static transforms from the ROS description stack |

If ROS topics exist but stay silent, confirm that Unity is in Play mode, the Editor's ROS connection HUD reports connected, and `ros-tcp-server` is still running.

To confirm the relevant TCP listeners from inside the container:

```bash
lsof -nP -iTCP:10000 -sTCP:LISTEN
lsof -nP -iTCP:9090 -sTCP:LISTEN
lsof -nP -iTCP:6080 -sTCP:LISTEN
```

## Package command reference

### `mobile_manipulator_description`

Use with Unity, without RViz:

```bash
ros2 launch mobile_manipulator_description simulation.launch.py
```

Use with Unity and RViz:

```bash
ros2 launch mobile_manipulator_description simulation_rviz.launch.py
```

The package also has a standalone model-inspection launch:

```bash
ros2 launch mobile_manipulator_description display.launch.py
```

`display.launch.py` starts its own joint-state GUI and RViz. It is for inspecting the URDF, not for running alongside the active Unity simulation.

### `ros_tcp_endpoint`

The project alias is convenient in an interactive Dev Container terminal:

```bash
ros-tcp-server
```

Its expanded form is:

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args \
  -p ROS_IP:=0.0.0.0 \
  -p ROS_TCP_PORT:=10000
```

### `clearpath_gz` and Gazebo

Clearpath Gazebo is an **alternative simulator**, not part of the normal Unity run. Do not run it for the same robot at the same time as Unity unless topic namespaces and simulation ownership have first been isolated.

The launch also requires a valid Clearpath `robot.yaml` under a setup directory. The current container does not have `$HOME/clearpath/robot.yaml`, so this stack is not currently configured for this project. Once a valid configuration is deliberately added, the complete upstream launch form is:

```bash
ros2 launch clearpath_gz simulation.launch.py \
  setup_path:="$HOME/clearpath" \
  world:=warehouse \
  use_sim_time:=true \
  rviz:=false \
  auto_start:=true \
  generate:=true \
  x:=0.0 \
  y:=0.0 \
  z:=0.3 \
  yaw:=0.0
```

Available worlds are `construction`, `office`, `orchard`, `pipeline`, `solar_farm`, and `warehouse`.

### Nav2 and MoveIt 2

The project now has a validated static-map and global-planner launch:

```bash
ros2 launch mobile_manipulator_navigation global_planning.launch.py
```

This is not yet a complete navigation or manipulation stack. Project-specific base control, odometry-message publication, the rolling lidar local costmap, controller server, behavior-tree navigator, MoveIt configuration, and lidar-to-planning-scene filtering remain deferred. Stock demo launches should not be treated as the thesis system.

## Shutdown and restart

Stop each ROS launch, endpoint, and bridge with `Ctrl-C`, then exit Unity Play mode. The `novnc` command has already returned because its helper processes run in the background; they stop with the container. VS Code's normal **Reopen Folder Locally** or window close stops the Compose service because the Dev Container uses `shutdownAction: stopCompose`.

If a later session finds Docker stopped, start Docker Desktop and use **Dev Containers: Reopen in Container** again. The ROS build, install, log, and `ccache` data are persisted in named Docker volumes.


## Run Unity automated

```bash
unity open motion-planning-sim --args "-automated"
```