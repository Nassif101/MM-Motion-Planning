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
ros2 run mobile_manipulator_control unity_control_endpoint --ros-args \
  -p ROS_IP:=0.0.0.0 \
  -p ROS_TCP_PORT:=10000
```

After rebuilding the development image, the `ros-tcp-server` alias expands to this command. In an older container use the explicit command above. It listens on `0.0.0.0:10000`. The endpoint is the project's C++ implementation of the ROS-TCP-Connector v0.7.0 protocol ([results](experiments/cpp-endpoint/README.md)). The Unity project is already configured to connect to `127.0.0.1:10000` through Docker Desktop's forwarded port.

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

Unity owns `/clock`, `/joint_states`, `/odom`, `/livox/lidar`, and the dynamic `odom -> base_footprint` transform. Do not start another publisher for these same contracts.

### 4. Start rosbridge for ROS-MCP

Keep this running in another container terminal:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  address:=0.0.0.0 \
  port:=9090
```

The shorter `rosbridge` alias uses port `9090`; the expanded command above also makes the bind address explicit. Start it before opening or restarting a Codex session that needs the live ROS graph.

Do **not** run `ros-mcp --transport=stdio` manually during normal use. The agent MCP configuration starts that process inside the running `ma-robot-sim` container, and [`.mcp.json`](../.mcp.json) gives Claude Code the same server. ROS-MCP connects to rosbridge at `127.0.0.1:9090` by default.

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
# Narrow-passage transport footprint instead of the default home profile:
ros2 launch mobile_manipulator_navigation global_planning.launch.py footprint_profile:=vertical_carry
```

Two global planners are available through the action's `planner_id`: `GridBased` (NavFn baseline) and `Lattice` (Smac State Lattice, footprint-aware). The read-only gate check queries both:

```bash
ros2 run mobile_manipulator_navigation gate_planning_check --label home
```

The footprint profile must match the arm pose actually held in Unity; the profiles are defined in `mobile_manipulator_navigation/config/footprint_profiles.yaml`. This launch loads the map exported from `ConstructionSiteV1`, publishes it on `/map`, creates the static global costmap, and exposes Nav2's path-computation actions. It starts only `map_server`, `planner_server`, and their lifecycle manager. It does not start AMCL, a controller server, or base command execution.

If the planner remains in activation while reporting a missing `map -> base_footprint` transform, confirm that step 3 is running and Unity is in Play mode. The map server can publish `/map` without robot TF, but the planner's global costmap cannot activate without the complete chain.

To regenerate the map after changing authoritative scene obstacles, run from a host terminal:

```bash
unity command export_nav2_map --project-path ./motion-planning-sim
unity command validate_nav2_map --project-path ./motion-planning-sim
```

Rebuild and source the ROS workspace afterward so the installed package receives the updated artifact. The complete export contract is in [`mobile_manipulator_navigation/docs/unity-map-export.md`](../ros2_ws/src/mobile_manipulator_navigation/docs/unity-map-export.md).

## Navigation scenarios

Fixed benchmark scenarios live in `mobile_manipulator_navigation/config/scenarios.yaml`
and run from a **host** terminal at the repository root once the endpoint, the
description launch, and Unity are running ([ADR 0007](adr/0007-navigation-scenario-reset.md)):

```bash
python3 tools/run_nav_scenario.py --new-epoch                 # all scenarios, fresh Play epoch
python3 tools/run_nav_scenario.py open_space narrow_gate_home # selected scenarios, current epoch
```

`--new-epoch` restarts the ROS-TCP endpoint, Play, and `arm_control.launch.py`. For every scenario the runner
restarts Nav2 with the scenario's footprint profile, moves the arm through home to the
scenario pose, teleports the stopped robot after checking the start is free, records a
rosbag, and writes `experiment_runs/<UTC time>-<scenario>/summary.json`. Add
`--record-lidar` to include `/livox/lidar` in the bag. `--global-obstacles` chooses which lidar obstacles reach the
global costmap: `persistent` (default) adds only obstacles seen for 2 s, so a passing worker stays a local obstacle
while a blockage changes the route; `live` adds every lidar obstacle with a 10 s decay (the default from 2026-09-30
to 2026-10-01); `static` (or `--static-global-costmap`) is the Phase 1 static-map baseline. Non-default modes add
`-liveglobal` or `-staticglobal` to the run name. `--controller rpp|dwb|mppi` selects
the local controller for `navigate_to_pose` scenarios (default `rpp`); those runs are named
`<UTC time>-<scenario>-<controller>` and their summaries add the controller server's
loop-rate misses and errors from the launch log.

A host sleep freezes the Docker VM and Unity, and afterwards the container clock jumps forward, so
wall-time checks misfire (an idle Mac sleeps and wakes in a five-minute maintenance cycle). On macOS
the runner holds off idle sleep with `caffeinate` while it runs, and every summary records
`host_woke_during_run`; `summarize_nav_runs.py` skips runs where it is true. Run long unattended
commands such as `colcon test` under `caffeinate -i` too.

## Navigation (Phase 1)

With Unity in Play, arm control active, and the arm held in the pose matching the
profile, start the full stack instead of `global_planning.launch.py`:

```bash
ros2 launch mobile_manipulator_navigation navigation.launch.py footprint_profile:=home
ros2 service call /lifecycle_manager_global_planning/is_active std_srvs/srv/Trigger
ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger
ros2 run mobile_manipulator_navigation check_cmd_vel_ownership --expect collision_monitor
```

The container runs every ROS process on Cyclone DDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
[ADR 0008](adr/0008-ros-middleware-cyclone-dds.md)); do not start nodes with another middleware.

Both `is_active` calls must return `success=True` before sending a goal. A lifecycle reply
lost during DDS discovery can leave a manager waiting indefinitely, and a planner whose
costmap failed to activate still accepts goals and plans on an empty costmap, straight
through walls. If either call fails, stop the launch and start it again. The lifecycle
managers start 3 s after the nodes so that `/clock` and the static `map -> odom`
transform arrive first.

It adds the Livox robot filter, the controller server (local costmap and one local
controller: `controller:=rpp` Regulated Pure Pursuit, the default bring-up controller;
`controller:=dwb` DWB, baseline B1; `controller:=mppi` Nav2 MPPI, baseline B2; all from
`config/nav2_controllers.yaml`), velocity smoother, collision monitor, behavior server (Wait only), and the BT
navigator (`NavigateToPose` with the Lattice planner by default). The default behaviour
tree replans only when the path becomes invalid; `behavior_tree:=replan_1hz` selects the
original 1 Hz replanning tree. Limits follow the
navigation operating envelope; `/cmd_vel` is published only by the collision monitor
([ADR 0006](adr/0006-base-command-ownership.md)).

### Navigation telemetry

In Unity, the **Telemetry** button (top-right of the Game view) or the `T` key shows the
telemetry window and the path overlay (planned path, goal ring, executed trail). From a
host terminal: `unity command telemetry_window --visible true`. Use this compiled command,
not `unity command eval`, during runs: runtime C# compilation stalls the Editor and can
trip the arm feedback watchdog.

`navigation.launch.py` also starts `nav_telemetry`, which aggregates ROS-side navigation state
for the Unity telemetry window: `/mm/telemetry` (`std_msgs/String` JSON at 5 Hz, about 0.6 KB:
goal status, recoveries, distance and time remaining, path length and age, cross-track error,
collision-monitor action, filtered-lidar rate and gaps, recent events) and
`/mm/telemetry/path` (the global plan downsampled to 0.1 m, on change and every 5 s). Label a
manual run with `ros2 param set /nav_telemetry scenario <name>`; the scenario runner does this.

## MoveIt arm reconfiguration (Phase 2, B3)

With Unity in Play and `arm_control.launch.py` active, start MoveIt, the known planning scene
(exported static boxes, the optional scenario's boxes, raised floor, attached panel) and the
`ReconfigurePanel` action server ([ADR 0009](adr/0009-moveit-panel-reconfiguration-baseline.md)):

```bash
ros2 launch mobile_manipulator_manipulation manipulation.launch.py \
  initial_footprint_profile:=home scenario:=<optional scenarios.yaml entry>
# Reconfigure at standstill; the server checks the footprint profile and switches it:
ros2 run mobile_manipulator_manipulation reconfigure_panel --named vertical_carry --profile vertical_carry
ros2 run mobile_manipulator_navigation panel_pose --pose vertical_carry   # panel goal for a qualified pose
ros2 run mobile_manipulator_manipulation reconfigure_panel \
  --panel-pose -0.08 0.225 1.32 -1.5707963 1.5707963 0 \
  --position-tolerance 0.01 0.01 0.01 --orientation-tolerance 0.01 0.01 0.01 --profile vertical_carry
```

`initial_footprint_profile` must be the profile Nav2 was launched with. When Nav2 runs alongside,
launch it with `footprint_mode:=profiles` so the collision monitor's stop and slowdown zones
follow the switched profile. Mission scenarios (`task: mission`) do all of this through the
scenario runner:

```bash
python3 tools/run_nav_scenario.py narrow_gate_mission wide_gate_mission \
  constrained_reconfiguration_mission --new-epoch --controller rpp --runs 3
python3 tools/summarize_nav_runs.py experiment_runs/*mission*
```

Without Unity, `ros2 launch mobile_manipulator_moveit_config mock_stack.launch.py base_pose:="x,y,yaw"`
runs the arm on ros2_control mock hardware (wall time; start `manipulation.launch.py use_sim_time:=false`
and publish a stamped zero `/odom`). Never run the mock stack alongside Unity.
`ros2 run mobile_manipulator_manipulation transition_validity_check --from home --to vertical_carry`
checks the qualified straight transition against the loaded scene without moving anything.
`manipulation.launch.py scene_source:=octomap` (runner: `--scene-source octomap`) adds MoveIt's lidar
Octomap and withholds the scenario boxes; it is experimental and not usable for experiments yet
(see the experiment record). `octomap_box_check --scenario NAME` reports what the Octomap holds in
each scenario box.

## Dynamic footprint (Phase 3, B4)

`footprint_mode` selects the one owner of both costmap footprints and the collision monitor's
zone inputs (ADR 0010): `static` (no publisher; the launch `footprint_profile` stays; the default
for navigate and plan-only scenarios), `profiles` (`ReconfigurePanel` switches named profiles; B3
missions), `dynamic` (`dynamic_footprint_node` publishes the padded convex hull of base, arm and
panel from `/joint_states`; B4). `navigation.launch.py` and `global_planning.launch.py` take
`footprint_mode` and `footprint_model:=mesh|disc` (dynamic only); `manipulation.launch.py` takes
`footprint_mode:=profiles|dynamic`. Settings are in
`mobile_manipulator_navigation/config/dynamic_footprint.yaml`.

```bash
# Any scenario with the dynamic footprint (missions too):
python3 tools/run_nav_scenario.py wide_gate_mission --footprint-mode dynamic --controller rpp
# A static launch footprint other than the scenario's (footprint strategies):
python3 tools/run_nav_scenario.py narrow_gate_home_nav --footprint-profile base_only
# Is the graph's footprint owner the expected one?
ros2 run mobile_manipulator_navigation check_footprint_ownership --mode dynamic
# Footprint update cost of the mesh and disc models (no Unity needed):
ros2 run mobile_manipulator_navigation footprint_benchmark --evaluations 5000
```

The runner checks ownership after the launch, waits for the costmaps to use the strategy's
footprint (`footprint_wait`), cancels a drive at the first robot-environment contact, and stores
the node's timing and publish statistics in the run summary (`footprint_stats`). The summarizer
labels rows `[dyn]`, `[dyndisc]` or `[fp:<profile>]`. Results: `docs/experiments/dynamic-footprint`.

The mock-hardware tests can leave `control_description` running with a latched
`/arm/robot_description`; check `ps -eo args | grep [c]ontrol_description` and stop it before a
Unity arm control launch or another mock test.

## Local costmap qualification harness

`local_costmap.launch.py footprint_profile:=<profile>` runs the Livox robot filter and a
standalone rolling local costmap (`/local_costmap/costmap`) without a controller. Use it
with the global-planning launch; the full navigation launch replaces it. Obstacles can be
added in Play with `unity command scenario_obstacle --name box --x 9.5 --y 0` and removed
with `unity command scenario_obstacle_clear`; `tools/local_costmap_obstacle_trials.py`
repeats the qualification trials. A moving worker is added with
`unity command scenario_mover --name worker --start_x 11 --start_y 2.5 --end_x 11 --end_y -2.5`
(it starts walking when the robot comes within 2.5 m of the segment midpoint); its
ground-truth position is published on `/scenario/movers`, `unity command scenario_movers`
reports its progress, and `scenario_obstacle_clear` removes it.

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
| `/odom` | Unity ground-truth `nav_msgs/msg/Odometry`, same sample as the TF |
| `/tf` | Unity's dynamic `odom -> base_footprint` plus ROS robot transforms |
| `/tf_static` | Static transforms from the ROS description stack |
| `/cmd_vel` | `geometry_msgs/msg/Twist` input to Unity's low-level skid-steer actuator |

If ROS topics exist but stay silent, confirm that Unity is in Play mode, the Editor's ROS connection HUD reports connected, and `ros-tcp-server` is still running.

### Manual base-controller smoke test

For a Unity-only test drive, select the `MobileManipulator` root and enable
`Skid Steer Keyboard Teleop > Enable Keyboard Teleop`. Click the Game view, then use:

- `W/S` or `Up/Down`: forward/reverse
- `A/D` or `Left/Right`: positive/negative ROS yaw
- `Space`: command a stop

Keyboard teleop is disabled by default. While enabled it deliberately overrides `/cmd_vel`, but
the same actuator clamps, acceleration limits, and wheel-speed saturation still apply. Disabling
the checkbox immediately returns command ownership to `/cmd_vel`; an old ROS command must still
pass the existing 0.5 s watchdog.

The same root has `Arm Actuator Controller`, which automatically captures and holds the six arm
joints at Play startup with gravity enabled. Keep it enabled when ROS arm control runs; it is the
physical actuator used by ros2_control. The temporary `Arm Joint Hold Controller` has been removed
from this scene. See [the arm controller document](unity-arm-controller.md).

The `RobotThirdPersonCamera` provides three views. Press `C` to cycle through them:

1. `Orbit`: follows `base_link`; scroll to zoom and hold the right mouse button to orbit.
2. `Rear Left Chase`: a close GTA-style view from behind the robot's left side, aimed across the
   left wheel pair.
3. `Payload First Person`: mounted just above the centre of `PayloadPanel`, aimed along its local
   forward axis and rotating rigidly with it.

Press `F` from any view to return to and recenter the orbit view. Nearby scene geometry
automatically shortens the external camera boom so walls do not hide the robot.

#### ROS command smoke test

Run these commands one at a time from a sourced container terminal. Stop each publisher with `Ctrl-C`; the Unity actuator also starts braking when `/cmd_vel` is silent for 0.5 s.

```bash
# Forward
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Reverse
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: -0.2}, angular: {z: 0.0}}"

# Counter-clockwise rotation in ROS FLU coordinates
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.4}}"

# One-metre nominal radius arc
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.2}}"
```

The command topic is intentionally generic: a manual publisher, Nav2 controller, or future MPC/QP may publish the same Twist without changing Unity. Only one of them may publish at a time ([ADR 0006](adr/0006-base-command-ownership.md)); check before any experiment:

```bash
ros2 run mobile_manipulator_navigation check_cmd_vel_ownership                           # no publisher expected
ros2 run mobile_manipulator_navigation check_cmd_vel_ownership --expect collision_monitor  # full Nav2 chain
``` For ROS tests, leave keyboard teleop disabled. Keep the arm actuator enabled during both Unity-only HOLD and ROS control.

Controller equations, parameters, measured commissioning results, and the reproducible test matrix are in [the skid-steer controller document](unity-skid-steer-base-controller.md).

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

### `mobile_manipulator_control`

After Unity enters Play and `/arm/state` is fresh, run in a sourced container terminal:

```bash
ros2 launch mobile_manipulator_control arm_control.launch.py
```

This starts `controller_manager`, `arm_controller` (JointTrajectoryController), and
`arm_joint_state_broadcaster`. Keep the existing description launch running. The arm control
launch publishes its augmented model on `/arm/robot_description` without publishing TF.
`/joint_states` remains Unity's ten-joint stream; broadcaster output is local to
`/arm_joint_state_broadcaster/joint_states`.

A bounded +0.05 rad trajectory, independently of MoveIt:

```bash
ros2 run mobile_manipulator_control arm_experiment --joint shoulder_pan_joint --delta 0.05 --duration 4
# Test cancellation with an explicit stop request after two wall-clock seconds:
ros2 run mobile_manipulator_control arm_experiment --joint shoulder_pan_joint --delta 0.2 --duration 10 --cancel-after 2
```

Only one action/command source should own the arm. Inspect obstacles and payload clearance
before any experiment. Stop this launch before exiting Play; restart it after every simulation
clock reset or hardware feedback fault. Unity continues finite-torque HOLD when the command
stream stops. Configuration, measured performance, logging, and the complete test matrix are
in [the arm controller document](unity-arm-controller.md).

For the complete 3 kg payload, level-extension and 1.05 m gate tests, use the host
runner and [qualification procedure](experiments/arm-controller/qualification/README.md#reproduce).
It requires a fresh active manager and teleports the stopped robot to declared
simulation fixtures before bounded physical motion. The report includes failed
low-frame-rate stress trials and the limits to use when starting planner work.

### ROS-TCP endpoint (`unity_control_endpoint`)

The project alias is convenient in an interactive Dev Container terminal:

```bash
ros-tcp-server
```

Its expanded form is:

```bash
ros2 run mobile_manipulator_control unity_control_endpoint --ros-args \
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

The full Phase 1 navigation stack is `navigation.launch.py` (above) and the Phase 2 arm planning is
`manipulation.launch.py` (MoveIt with known geometry); lidar-to-planning-scene filtering (Octomap) is
the next scene source. Stock demo launches should not be treated as the thesis system.

## Shutdown and restart

Stop each ROS launch, endpoint, and bridge with `Ctrl-C`, then exit Unity Play mode. The `novnc` command has already returned because its helper processes run in the background; they stop with the container. VS Code's normal **Reopen Folder Locally** or window close stops the Compose service because the Dev Container uses `shutdownAction: stopCompose`.

If a later session finds Docker stopped, start Docker Desktop and use **Dev Containers: Reopen in Container** again. The ROS build, install, log, and `ccache` data are persisted in named Docker volumes.

## Run Unity automated

```bash
unity open motion-planning-sim --args "-automated"
unity open "$PWD/motion-planning-sim" --args "-automated"
```
