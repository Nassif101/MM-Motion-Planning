# Plant viewer (spec 4): records

Design: [2026-11-20-plant-unity-viewer-design.md](../../superpowers/specs/2026-11-20-plant-unity-viewer-design.md).
Plan 4A: [2026-11-20-plant-viewer-4a.md](../../superpowers/plans/2026-11-20-plant-viewer-4a.md).
Observed results are recorded here, separately from intended behavior.

## Task 5 live check (2026-11-20, Editor Play, MacBook Air M4)

Setup: `plant_demo_server --mode realistic --routes lattice` in the container (overlay build with `cancel_goal`),
`Assets/Scenes/PlantViewer.unity` in Editor Play. The viewer was driven through `PlantViewerBootstrap.Perform` with
`unity command eval`, because hotkeys cannot be typed from the CLI. The goal was submitted through the viewer's own
connection (the server serves one client at a time); the minimap path follows in Task 6.

| Check | Observed |
|---|---|
| Connect through `localhost` (Task 3 review C1) | `Connected localhost:7700` |
| Reset through the scenario picker | session `v2_open_yard_nav` seed 1, epoch 1 |
| Goal 3 m ahead, run 1x | accepted (`client_request` in Unix ms), Tracking, route 49 poses, achieved 1.00x, reached |
| Speed up | 2x requested, 2.00x achieved |
| Pause / step | clock held (no stale warning while paused); step advanced and paused |
| Cameras | C cycled Orbit, Chase, Payload, Free, Orbit; orbit and payload views captured |
| Shift+R | epoch 1 to 2, the reached goal and its route cleared |
| Server killed, then restarted | Connecting, then Connected about 15 s later; the dead session cleared (no session); no reset sent |
| Journal | `~/Movies/PlantViewer/<UTC>/journal-1.jsonl` (session header, then actions and camera changes), a new file per session |
| Editor FPS (HUD, 1x, not a measurement) | 150-330 |

### Problems found and resolved

| Problem | Cause | Resolution |
|---|---|---|
| After a reset or a new server, the robot stayed at its old pose (or the world origin) and the clock showed the old time | The server sends no State frame while paused; the viewer kept the last pose | The robot is hidden until the session's first State, and the HUD shows `t -` (Task 5) |
| The HUD line was cut off at 1280 px | IMGUI label without word wrap | Word wrap on (Task 5) |
| A second client could not submit goals during the check | The demo server serves one client at a time (stream-protocol.md, Connecting) | By design; the check used the viewer's connection |

## Task 6 live check (2026-11-20, Editor Play, MacBook Air M4)

Setup as in Task 5, with the server rebuilt for `run_stopped` (Task 5 review I2). Goals were entered through the
bootstrap's real gesture path: press, drag and release were appended to `PlantMinimap.Gestures` with `unity command
eval`. The IMGUI mouse handling and the overlay's drawing were not exercised: the CLI captures (`capture_game_view`,
`screenshot`) render the cameras without IMGUI. They need one look by eye (Task 7's player run).

| Check | Observed |
|---|---|
| Map fingerprint | goals enabled: the bundled map's three SHA-256s equal Hello `routing` |
| Top-down render | 1000 x 800, one pixel per map cell, the site in the expected orientation (image dumped and viewed); robot and scenario objects left out. In a probe render with the robot shown, the robot at (-6, -3.5) appeared at pixel (378, 467), against (380, 470) computed |
| Destination 3 m ahead, dragged 1 m along the heading | heading -0.52 rad from the drag; planning, tracking (route 47 poses; the scene LineRenderer held 47 points, a count, not seen by eye here), reached at t 15.6 s; route line cleared (0 points) |
| Replacement mid-drive | goal 2 (route 254 poses) replaced by goal 3 at t 40.3 s; marker and route switched to goal 3 (63 poses) |
| X during a drive | goal 4 canceled; route cleared; robot pose unchanged over 4 s of sim time |
| Press on an occupied cell (-11.7, -5.5) | rejected: "destination footprint is occupied, unknown, or outside the map", on the marker and the HUD |
| Press off the map (-30, 0) | submitted unclamped and rejected with the same reason |
| Journal | `minimap_press`, `minimap_drag`, `minimap_release` and `goal_submitted` (target, `client_request`) for each goal |

### Problems found and resolved

| Problem | Cause | Resolution |
|---|---|---|
| The top-down render was nearly black and blurred | A new HDRP camera has no exposure or temporal history on its first frame | 16 warm-up renders at startup (measured: settled within 10) |
| Creating the presenters threw "CreateImpl is not allowed to be called during serialization" | `MaterialPropertyBlock` in a field initializer | Created in `Awake` |
| The HUD showed `t 0.00 s` before any session | The displayed time started at 0, not unknown | Starts unknown (`t -`) |

### Task 6 review fixes (2026-11-20)

| Problem | Cause | Resolution |
|---|---|---|
| Without the bundled map, M showed nothing and no warning | The minimap returned before drawing when it had no map | The HUD warns "minimap goals off: no map: ..." and the open panel states the reason (seen live by the HUD text; the panel is IMGUI) |
| A click after a reset got heading 0 instead of robot-to-point | No State arrives while paused, so there was no robot pose | The session Hello's `start` pose is used until the first State; live: a click at (-6, 0.5) from start (-6, -3.5) gave 1.571 rad |
| After a server restart, resetting to the same scenario could continue the old journal | The restarted server sends no replay, so the reset's Hello counted as one | A reset sent by the viewer ends the wait for a replay |

## Task 7: macOS player (2026-11-20)

Build: `unity command build_plant_viewer_player` in the open Editor. It succeeded in 227 s: 203 MB, scene list
`PlantViewer.unity` only, Apple Silicon (ARM64). The build rewrote three settings files in Unity's newer serialization
format (`QualitySettings.asset`, `UnityConnectSettings.asset`, `HDRenderPipelineGlobalSettings.asset`). It changed no
values, and the files were reverted.

Launch check, run with the screen locked, so nothing was seen by eye:
- Server: `plant_demo_server --mode realistic --routes lattice --lidar --log /tmp/mm-viewer/e2e-run.log`.
- Player: `-scenario v2_open_yard_nav -goal 18,0,0 -minimap -frame-times`, 1280 x 720 window, 40 s.

| Check | Observed |
|---|---|
| Launches and connects | the journal holds the session header, then the scripted `demo` and `goal_submitted` [18, 0, 0] |
| No ROS simulator in the player | `Player.log` and stdout never mention ROS or TCP connector activity |
| Quits cleanly | `osascript quit` wrote the journal and the frame times |
| First launch after a build | one frame of 19.5 s at startup (the minimap's top-down render); a second launch: 0.5 s. Metal compiles shaders once and caches them |
| Frame times after the run started (locked screen, 1280 x 720, High Fidelity) | p50 16.7 ms, p99 17.7 ms (vsync-capped). Not the 4A measurement: that needs the screen unlocked, 1920 x 1080 and a recorder |

## Task 7: end-to-end run and FPS smoke test (2026-11-20, the macOS player, screen unlocked)

Setup:
- Server: `plant_demo_server --mode realistic --routes lattice --lidar --log /tmp/mm-viewer/e2e-run.log`, in the
  container.
- Host: the Editor was closed, and only the player and `screencapture -v` (whole screen, 2940 x 1912) ran.
- Each take used the scripted demo: `-scenario v2_open_yard_nav -goal ... -minimap` at 1x.

The takes are in `~/Movies/PlantViewer/take-4a-20261120T155005Z/` and are not committed.

**FPS protocol.** Window 1920 x 1080. 10 s of warm-up after the run starts, then 60 s measured. Route: (18, 0, 0)
on the east road, about 24 m from the open-yard start; it was not reached within the window.

| Quality level | Frames | Mean FPS | p50 | p95 | p99 | Max | > 33.3 ms | At 60 FPS (<= 16.9 ms) | Thermal before / after |
|---|---|---|---|---|---|---|---|---|---|
| High Fidelity (the project default) | 2690 | 44.8 | 20.3 ms | 33.8 ms | **34.0 ms** | 34.5 ms | 383 | 35.1% | no warning / no warning |
| Balanced | 3501 | 58.4 | 16.7 ms | 17.7 ms | **30.0 ms** | 49.7 ms | 26 | 72.8% | no warning / no warning |

- High Fidelity misses the target (p99 34.0 ms > 33.3 ms). Its frames sit at 16.7 or 33-34 ms: vsync alternating
  between 60 and 30 FPS.
- Balanced meets it, so the player now defaults to Balanced (`-quality` selects another level).
- 60 FPS is reached most of the time at Balanced, not always.
- `pmset -g therm` recorded no thermal or performance warning before or after any take.

**Demo takes (the 60 s recording).**
- Setup: fullscreen at 1920 x 1080, so the recording shows the viewer only (letterboxed); Balanced; minimap open;
  destination (4, 0, 0), about 11 m.
- Two takes of 80 s each (`demo/demo.mov`, `demo2/demo.mov`): the goal was accepted, tracked and reached at
  t = 40.74 s in both.
- The viewer ran at 60.0 FPS (p99 17.1 ms).
- The backend did **not** keep 1x on this route, in either take:
  - take 1: 53.2 s of simulation in 83.8 s of wall time, about 0.63x (the HUD showed "achieved 0.59x");
  - take 2, after a 4-minute cool-down: 51.1 s in 82.8 s, about 0.62x.
- On the measurement route the backend held about 1.0x: 77.5 s simulated in about 78 s, and the HUD showed "achieved
  1.00x" mid-run.
- The cause is not isolated. The cool-down rules out heat; candidates are lidar or B5 cost near the yard's wall, or
  the container competing with the fullscreen player and the recorder. Left for 4C, which qualifies workloads.
- Until then, a real-time take may be slower than 1x on some routes. The HUD's "achieved" speed shows it, and 4C's
  offline render from the run log removes the limit.

**End-to-end, seen in a screen capture of the player mid-take.** Visible:
- the HUD line (connection, session, t, requested and achieved speed, FPS);
- the minimap: the top-down image with occupancy, the robot dot, the route, the goal marker and "Goal Tracking";
- the scene's route line and goal ring;
- the orbit camera following the robot.

Hotkeys and mouse input were not exercised in the player: the takes were scripted. They were exercised in Editor
Play (Tasks 5 and 6, through the same code), and remain the user's first check by hand.

### Problems found and resolved

| Problem | Cause | Resolution |
|---|---|---|
| High Fidelity missed the 30 FPS target (p99 34.0 ms) | The GPU load at 1920 x 1080 plus a full-screen recorder | The player defaults to Balanced (p99 30.0 ms), test-first (`ViewerOptionsTests.DefaultsWithoutArguments`) |
| The windowed recordings captured the whole desktop | `screencapture -v` records the screen, not a window | The demo takes run fullscreen; the windowed measurement takes stay local and are not shared |
| The first launch after a build stalled for 19.5 s | Metal compiles the shaders once | Documented in running-the-stack.md; later launches take 0.5 s |
| The backend ran at about 0.62x on the demo route | Not isolated (heat ruled out) | Recorded for 4C; the HUD shows the achieved speed |
