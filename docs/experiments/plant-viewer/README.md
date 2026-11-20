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
