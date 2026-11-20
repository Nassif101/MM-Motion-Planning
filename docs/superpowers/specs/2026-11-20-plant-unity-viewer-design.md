# Unity plant viewer design (spec 4)

Date: 2026-11-20. Branch: `plant-routing-unity-viewer`. Status: draft for the user's approval.
Inputs: the approved [requirements brief](2026-11-18-plant-routing-unity-viewer-brief.md) (sections 3-5), the
[spec 4 roadmap](../plans/2026-11-18-plant-unity-viewer.md) and the spec 2 handoff
[viewer-handoff.md](../../experiments/plant-routing/viewer-handoff.md). This design replaces the roadmap's Task 0.
Paths under `Assets/` are relative to `motion-planning-sim/`. "User" marks the user's decisions from the
2026-11-20 discussion; "call" marks the assistant's calls, open to the user's correction.

## 1. Intent

The viewer is a standalone player that shows the ROS-free MuJoCo/F2/B5 backend live, so that the user can **record
demonstration videos**. In the first stage, the user picks a destination on a minimap and the full hierarchy drives
the robot there. The user drives the session live (destinations, cameras, overlays). Development and the first
recordings are on the MacBook Air M4; later runs are on the user's PC (user).

## 2. Goals, non-goals, anti-goals

Goals:
- A macOS standalone player built on the Mac, connecting to the container's `plant_demo_server` (user).
- Two ways to make a video (user): **live**, recorded with a screen recorder (stage 4A); **rendered afterwards**,
  from the server's run log plus the viewer's own journal, at a fixed frame rate (stage 4C).
- At least 30 FPS, held steady (user, section 8).
- The brief's requirements, staged in 4A, 4B and 4C (section 9).

Non-goals:
- A Windows player is not a requirement now. It is built later, and stays unverified until it has run (user).
- Unity does not plan, simulate, sense or control. The backend runs fully without Unity.
- Spec 3 (MuJoCo ROS bridge), new worker prediction and full 3D perception stay in their phases.

Anti-goals (a failure even when it works):
- A smooth picture of stale data. A displayed state older than its limit must say so (section 6.4).
- Showing a destination as accepted before the backend accepts it.
- Any ROS-simulator component (ROS connector, articulation physics, sensors, publishers) starting in the player.
- Viewer settings that change backend results. Rendering, overlays and the journal never feed the backend; only the
  explicit commands of section 5.2 do.

## 3. Constraints

- Native-host Unity 6000.5.2f1 with HDRP 17.5.0 in the existing project; Unity CLI and `com.unity.pipeline` only,
  never Unity MCP. macOS standalone support is installed; Windows support is not.
- No new packages: JSON via `com.unity.nuget.newtonsoft-json` 3.0.2 (already resolved), input via
  `com.unity.inputsystem` 1.19.0.
- Wire format unchanged: `[u32 length][u16 type][u32 json_len][json][u32 n_blocks]{[u32 n][float32 x n]}`, little
  endian (stream-protocol.md, schema 3).
- The MacBook Air is fanless: never overlap Play mode or a player with timed backend runs; stop Play after a task.
- Never commit `Library/`, builds, captures, journals or host-absolute paths.
- Keep the plans compact: no data, settings or features that no requirement needs (user).

## 4. Approaches

**How the viewer gets a physics-free robot and site** (user chose 1):
1. *A dedicated viewer scene* built by an Editor command from the V2 site's art and a copy of the robot with
   physics, ROS and sensor components removed; the player contains only this scene. Clean separation; the copy is
   regenerated when V2's art changes. **Chosen.**
2. *A viewer-mode switch in `ConstructionSiteV2.unity`.* No second scene, but mode checks spread through existing
   scripts, one missed component runs ROS or physics in the viewer, and the player carries the ROS simulator.
3. *A separate Unity project.* Cleanest split, but duplicates gigabytes of site assets and the HDRP setup on a disk
   that was at 98% (2026-11-19). Ruled out.

**How a video is made** (user: both, when needed): live screen recording (4A) and an offline render from the run log
and a view journal (4C). The journal is written from 4A on, so 4A sessions can be rendered later.

## 5. Architecture

```
container                                   host (macOS player)
plant_demo_server ──TCP 7700, frames──────▶ StreamClient (thread) ─▶ FrameCodec ─▶ ViewerState (main thread)
 (Session: MuJoCo, B5, F2,          ◀──── Control JSON commands ◀── ViewerCommands      │
  route planner, goal manager)                                                          ▼
  --log run.mmlog                                       presenters: robot, environment, route, goal, HUD,
                                                        minimap, cameras ─── ViewJournal ─▶ journal file
```

### 5.1 Code layout (call)

- `Assets/Scripts/Viewer/Core/` — assembly `MotionPlanningSim.Viewer.Core`, `noEngineReferences: true`: `FrameCodec`
  (decode/encode frames), `StreamClient` (socket, read thread, bounded queue), `ViewerState` (session, latest
  states, goals, route, connection status; epoch handling), `ViewerCommands` (command JSON, `client_request`
  counter), `ViewJournal` (writer; 4C adds the reader). Pure C#: testable in EditMode without a scene.
- `Assets/Scripts/Viewer/Unity/` — assembly `MotionPlanningSim.Viewer`, referencing Core and the Input System:
  `PlantViewerBootstrap`, `PlantRobotView`, `PlantEnvironmentView`, `PlantRoutePresenter`, `PlantGoalPresenter`,
  `PlantMinimap`, `PlantViewerHud`, `PlantViewerInput`, `FreeCamera`; reuses `ThirdPersonRobotCamera` and
  `RosMapPose` from `MotionPlanningSim.Runtime` (referenced; neither starts ROS).
- `Assets/Scripts/Editor/PlantViewerSetup.cs` — builds `Assets/Scenes/PlantViewer.unity` and
  `Assets/Robots/MobileManipulator/Prefabs/MobileManipulatorView.prefab`; `PlantViewerBuild.cs` builds the player.
- Tests: `Assets/Tests/EditMode/Viewer/` in a new test assembly referencing both viewer assemblies. No PlayMode
  assembly: robot posing and presenters are checked in EditMode by instantiating the prefab (call).

### 5.2 Backend changes (stage 4A, schema stays 3)

- **`cancel_goal`** (user): a Control command `{"cmd": "cancel_goal", "client_request": n}`; generated-route servers
  only. `Session::cancel_goal()` ends the active goal through the goal manager's existing `Canceled` outcome
  (`goal_canceled` event and Goal frame, reason `"client"`), queued submissions included. The robot stops and
  holds; the session keeps running. Acknowledgement as `submit_goal` (`accepted`, or `rejected` with reason
  `no_goal`, `stale`, `backpressure`). The session Hello's `capabilities` gains `"cancel_goal"`; the viewer enables
  its hotkey only when listed. Tests in `test_goal_manager`, `test_generated_session`, `test_demo_server`;
  `plant_stream.py` and stream-protocol.md updated.
- No other backend change in 4A. Wheels are posed from the measured base twist (section 6.2); the minimap reads the
  map file (section 6.5). Schema 4 starts with 4B's new frames.

### 5.3 Transport and state (stage 4A)

- `StreamClient` reads on a background thread and decodes frames there; the main thread drains the queue once per
  rendered frame and applies it to `ViewerState`. No Unity call off the main thread.
- Bounds (call): a frame larger than 16 MiB is malformed and drops the connection. The queue holds at most 256
  frames; on overflow, State, Plans and Scan frames are coalesced to the newest, while Hello, Event, Goal and Route
  frames are never dropped (the backend's own rule). Counters for dropped and malformed frames are shown on the HUD.
- Handshake: read the server Hello, send `hello` with `schema: 3`; a schema other than 3 is shown as an error and
  the viewer stays disconnected.
- A session Hello (after each reset) clears the session's state. Goal and Route frames whose `epoch` is older than
  the newest seen are ignored.
- Reconnect: on disconnect, retry every 2 s; the server resends the latest Hello, Goal and Route frames, so the
  viewer never sends `reset` on reconnect.
- Recorded-route servers (no `generated_routes` in Hello) can be viewed; minimap goals and cancel are disabled with
  the reason shown.

## 6. Stage 4A viewer behavior

### 6.1 Scene and display rig

- `PlantViewerSetup` copies the V2 site's static art into `PlantViewer.unity` and creates `MobileManipulatorView` from
  `MobileManipulator.prefab` with every `ArticulationBody`, collider, ROS, sensor and controller component removed
  (meshes and the link hierarchy kept). Removing an `ArticulationBody` loses its joint axis, so before removal the
  command records, on each moving link, a `ViewJoint` component with the joint's name, local axis and rest rotation;
  `PlantRobotView` rotates links only through `ViewJoint`. The command is rerun when V2 changes; a test fails if the
  viewer scene holds any component from `MotionPlanningSim.Runtime.ROS`, `Runtime.Sensors`, `Runtime.Control`
  (except the camera), ROS-TCP-Connector or UnitySensors.
- The setup command also stores the site model's checksum, read from `<text name="site_checksum">` in
  `ros2_ws/src/mobile_manipulator_plant/models/site_v2.xml` (as `view_run.py` reads it). Before showing a session,
  the viewer compares it with Hello's `site_checksum`; a mismatch is shown and the session is still drawn, marked
  "site differs" (call: a demo stays usable while art is refreshed).
- Scenario boxes from Hello `boxes` (skipping those with a `site/<name>` geom), movers from State `movers` and
  reserved bodies from State `bodies` are drawn as simple boxes of their sizes (call; worker models are 4B polish if
  wanted).

### 6.2 Robot pose

- From State block 0 (true state): base x, y, yaw through `RosMapPose.ToUnityPosition/ToUnityRotation`; the six arm
  joints (shoulder_pan ... wrist_3) set on the view prefab's joint transforms by name.
- Wheels spin from State block 2's base twist (v, omega): each side's angle integrates
  `(v ∓ omega * track / 2) / wheel_radius` with the plant profile's skid-steer mapping
  (`ros2_ws/src/mobile_manipulator_plant/robots/mobile_manipulator/profile.yaml`: `wheel_radius_m` 0.14,
  `effective_track_m` 3.0), copied into the view prefab by the setup command. This is display only (call).
- Display interpolation: while running, the display clock advances every frame with wall time at the rate the States
  arrived over the last 2 s, and is pulled toward `t_newest - 0.1 s` at 2/s; it jumps there when it is more than
  0.5 s off. Paused, it shows the newest State. It never passes the newest State and never runs backwards.
  - History: the lag was 0.04 s, re-anchored on each State arrival (Task 3 review, 2026-11-20).
  - Measured change (user, 2026-11-20): arrivals are 20 ms apart on average but 8-45 ms in practice, and the
    re-anchored clock turned that jitter into visible stutter: 0-29 ms of simulation time per 60 FPS frame, 1.3% of
    frames frozen. The smoothed clock with a 0.1 s lag gave 15.8-17.1 ms.
  - The lag delays only the picture, never the backend. It interpolates between the bracketing states (position linear, yaw and
  joints shortest-angle). Recorded values are never modified.

### 6.3 Cameras (user)

- Orbit, rear-left chase and payload views from `ThirdPersonRobotCamera`, retargeted to the view prefab.
- A free camera: W A S D to move, Q / E down / up, Shift faster, scroll changes the base speed; look by holding the
  right mouse button, or Control and dragging on the trackpad.
- C cycles through orbit, chase, payload and free. Cameras work while paused.

### 6.4 Session controls, HUD and staleness

| Key | Action |
|---|---|
| Space | run / pause |
| . | step one B5 cycle while paused (`step`) |
| [ / ] | speed down / up through 0.25, 0.5, 1, 2, 4 (`run` with `speed`) |
| C | next camera |
| M | minimap |
| X | cancel the current goal (`cancel_goal`) |
| Shift+R | reset the same scenario and seed |
| F1 | scenario picker (from `list_scenarios`), then reset |

Hotkeys are disabled while typing in a text field. The HUD line shows connection state, scenario and seed,
simulation time, requested and achieved speed (simulation time over wall time, 2 s window), FPS, and the dropped
frame counters. **Staleness** (call): while running, if no State frame has arrived for 0.25 s of wall time, the HUD
shows "stale" in red with the age, and the robot is drawn at its last state without extrapolation; paused shows
"paused".

### 6.5 Minimap and goals

- Opened with M as an overlay panel. Its image is an orthographic top-down render of the viewer scene, made once at
  startup, with the occupancy map (`construction_site_v2.pgm`, trinary) drawn over it at 40% opacity (call).
- The map files (`construction_site_v2.{yaml,pgm,metadata.json}` from `mobile_manipulator_navigation/maps/`) are
  copied into the player's StreamingAssets at build time. Their SHA-256 is compared with Hello's `routing`
  fingerprint; on mismatch, goals are disabled and the reason shown. Pixel-to-world uses the YAML's `resolution`
  (0.05 m) and `origin` (-25, -20).
- Press sets the position, dragging sets the final heading (an arrow follows the cursor; a click without a drag keeps
  the heading from the robot to the point). Release sends `submit_goal` with `target` [x, y, yaw] and the next
  `client_request`.
- The marker shows *pending* until the acknowledgement, then follows the Goal frames: *accepted* (planning, then
  tracking), *rejected* with its reason, *superseded*, *reached*, *failed*, *timed out*, *canceled*. A new goal
  replaces the old marker when the backend reports `goal_superseded`, not before.
- The accepted route (Route frame, x/y/yaw) is drawn as a ground line in the scene and on the minimap; it disappears
  when its goal ends or a new route replaces it.

### 6.6 View journal (user)

- A JSON-lines file per live session in the player's output folder (default
  `~/Movies/PlantViewer/<UTC timestamp>/journal.jsonl`; configurable, never in the repository).
- First line: the session's identity from Hello (`scenario`, `seed`, `mode`, `site_checksum`, `routing`
  fingerprint, `provenance`, `epoch`) so 4C can check it against the run log. Each later line:
  `{"t": sim_time, "wall": s, "kind": ..., ...}` for camera changes (view, and the free camera's pose at 10 Hz while
  it moves), panel and overlay toggles, minimap open/close, goal presses/drags/releases, cancel, pause/run/speed/
  step/reset, and the cursor position at 10 Hz while the minimap is open.
- Bounded: at most 100 lines per second and 50 MB per session; beyond that, cursor and free-camera lines are thinned
  first. The journal never sends anything to the backend.

## 7. Errors and edge cases

- Server absent at startup: the HUD shows "connecting to localhost:7700" and retries; the host and port are set
  in a small config file next to the player or by command-line arguments (`-host`, `-port`).
- A goal outside the map or in an occupied cell is still sent; the backend's rejection and reason are shown (the
  viewer does not judge passability).
- Goal replacement while paused, while a search is pending, and right after a goal is reached: the marker follows
  only the backend's Goal frames, so no stale marker remains.
- Reset during delayed delivery: frames from the old epoch are ignored after the new session Hello.
- Malformed frame: dropped connection, counter incremented, reconnect.

## 8. Performance

- Target (user): at least 30 FPS, held steady: the 99th percentile frame time is at most 33.3 ms, measured at
  1920 x 1080 in the macOS player, with the 4A picture (4B: the normal overlays) and a screen recorder running.
  Whether 60 FPS is reached is recorded, not required.
- Reference: the MacBook Air M4 now (user: build on the Mac and see how it performs); the PC is measured when the
  Windows player exists. Protocol (call): demo server in the container at 1x, realistic mode, lidar on, the longest
  V2 destination; 10 s warm-up then 60 s measured; thermal state checked before and after (fanless host).
- A `Viewer` HDRP quality level (call) is the player's default; if the Mac misses the target, lower its settings
  first (shadows, volumetrics, resolution scale) and record the change, before touching code.
- 4A runs this check once, as a smoke test; 4C qualifies all workloads.

## 9. Stages

**4A, first demo (this design in full):** sections 5-8 plus the `cancel_goal` command, the viewer scene and robot
view, cameras, minimap goals, session controls, HUD, journal, the macOS player build, the ownership ADR and the
mode-specific `AGENTS.md` amendment (MuJoCo owns physics and lidar, the framework owns planning and control, Unity
renders and sends explicit user requests). Done when a recorded video shows a minimap destination driven to by the
full hierarchy in the macOS player, and the smoke test's FPS is recorded.

**4B, inspection (designed in detail when it starts; schema 4):** the telemetry panel and the diagnostic console
(brief section 4); overlays for the travelled path, F2's medium plan (the existing Plans frame), B5's MPC prediction
(new frame), lidar points placed with the pose at each ray's time (new data), arm and payload ghosts sampled from the
MPC prediction, active margins and failed or rejected candidates with reasons. Optional and cut first if time is
short: collision-geometry inspection and any continuous swept volume (which needs a defined error bound first).

**4C, qualification:** replay of run logs through the same `ViewerState`; the offline render (run log plus
journal, fixed frame rate, numbered PNG frames, `ffmpeg` to MP4; `ffmpeg` installed with Homebrew); the full FPS
qualification of section 8; operating instructions in `docs/running-the-stack.md`. The Windows player, when
attempted, is recorded as built and unverified until it runs on the PC.

## 10. Testing

- Core (EditMode): decode the existing fixtures `tools/plant/fixtures/{frame_v1,goal_v3,goal_rejected_v3,route_v3}.bin`
  to the values the C++ and Python tests assert; fragmented and oversized input; schema mismatch; queue overflow
  coalescing that keeps Goal/Route/Event order; epoch filtering; command JSON for every command; journal line
  format and thinning.
- Unity layer (EditMode): the view prefab holds no forbidden component; a fixture pose places base, arm joints and
  wheels where expected; minimap pixel-to-world round trip at the map's corners and heading convention.
- Backend: `cancel_goal` with an active goal, with none, with a queued submission and after reset.
- End to end on the Mac (recorded in `docs/experiments/plant-viewer/`): server plus player; destination, replacement,
  cancel, pause/step/speed, reset, reconnect after killing the server; a short live recording; the FPS smoke test.

## 11. Left to the builder

Class internals, HUD and minimap styling, the ground-line rendering technique, the config file's format and the
retry details within the stated limits.
