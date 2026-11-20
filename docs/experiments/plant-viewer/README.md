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
