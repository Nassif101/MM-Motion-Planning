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
- The cause is not isolated. The 4-minute cool-down did not change it, but no thermal reading was taken for either
  demo take, so heat is unlikely rather than excluded. The run log of these takes was overwritten by later sessions,
  so the simulated-time figures above cannot be rechecked. See "Stutter and slowdown" below: alone, the backend keeps
  1x on this route.
- Until then, a real-time take may be slower than 1x on some routes. The HUD's "achieved" speed shows it, and 4C's
  offline render from the run log removes the limit.

**End-to-end, seen in a screen capture of the player mid-take.** Visible:
- the HUD line (connection, session, t, requested and achieved speed, FPS);
- the minimap: the top-down image with occupancy, the robot dot, the route, the goal marker and "Goal Tracking";
- the scene's route line and goal ring;
- the orbit camera following the robot.

**Not verified (Task 7 review I2).** The real key handling (`PlantViewerInput`) and mouse handling
(`PlantMinimap.HandleMouse`) have never run, in the Editor or in the player:
- Task 5 called `PlantViewerBootstrap.Perform` directly.
- Task 6 appended gestures to `PlantMinimap.Gestures`.
- The player takes were scripted.

The demo destination came from `-goal`, not a minimap click, and the backend ran at about 0.62x. So plan Step 3's "a
minimap destination reached at 1x" is not yet shown. It needs a hand check: keys, a minimap click and drag, and one
take driven by hand.

### Problems found and resolved

| Problem | Cause | Resolution |
|---|---|---|
| High Fidelity missed the 30 FPS target (p99 34.0 ms) | The GPU load at 1920 x 1080 plus a full-screen recorder | The player defaults to Balanced (p99 30.0 ms), test-first (`ViewerOptionsTests.DefaultsWithoutArguments`) |
| The windowed recordings captured the whole desktop | `screencapture -v` records the screen, not a window | Superseded: fullscreen slows the backend (slowdown experiments). Record windowed and select the window in the screen recorder |
| The first launch after a build stalled for 19.5 s | Metal compiles the shaders once | Documented in running-the-stack.md; later launches take 0.5 s |
| The backend ran at about 0.62x on the demo route | Fullscreen: the slowdown experiments isolate it (windowed 0.99x) | Record windowed (running-the-stack.md); the HUD shows the achieved speed |
| The minimap drew a replayed session's robot at its old start pose (Task 7 review I1) | The start-pose fallback also applied to replays | Only for a session the viewer saw start (test `AReplayedSessionHasNoPoseUntilItsFirstState`) |

## Stutter and slowdown (2026-11-20, after Task 7)

The user saw the robot stutter in the takes. Probes with a Python client on the host, with the viewer closed,
measured both questions.

**State arrivals at 1x, viewer closed:**
- demo route (4, 0, 0): 44.8 s simulated in 44.8 s of wall time (1.00x); goal reached at t 40.7 s;
- measurement route (18, 0, 0): 1.00x;
- arrivals: one every 20 ms on average, but the gaps vary: p50 20.1 ms, p90 22 ms, p99 43.6 ms, max 54 ms; no
  bursts.

**Backend headroom, unpaced, viewer closed:** 3.69x real time on the demo route and 3.45x on the measurement route,
which is 5.4-5.8 ms of wall time per 20 ms cycle. A Unity batch test run by the reviewer may have overlapped this
probe.

**What this means:**
- The 0.62x in the fullscreen takes is not a lack of backend capacity. Each cycle took about 32 ms there, about six
  times its cost alone. A scheduling interaction between the Docker VM, the fullscreen player and the recorder is
  suspected, not shown.
- The slowdown does not change the robot's behavior: the plant runs in lockstep, and paced and unpaced runs reach the
  same decisions (test `PacedAndUnpacedLatticeRunsReachTheSameDecisions`).

**The stutter is mainly the viewer's display clock.** `ViewerState.TrySample` re-anchors the displayed time on every
State arrival, so arrival jitter becomes motion jitter:
- an early State makes the robot jump ahead;
- a late State is clamped by monotonicity, and the robot freezes;
- any gap over the 0.04 s lag stops it.

Replaying 20 s of recorded arrivals at 60 FPS (ideal 16.7 ms of simulated time per frame):

| Display clock | Simulated time per frame, p1-p99 | Max | Frozen frames |
|---|---|---|---|
| Current: lag 0.04 s, re-anchored on each arrival | 0.0-29.3 ms | 36.8 ms | 1.3% |
| Smoothed: lag 0.1 s; advances at the arrival rate every frame and is pulled toward newest minus lag at 2/s | 15.8-17.1 ms | 17.2 ms | 0% |

A slowed backend makes arrivals more irregular and the stutter worse, but the clock re-anchoring is the cause. The
smoothed clock changes design 6.2's 0.04 s lag, so it waits for the user's decision.


## Slowdown experiments (2026-11-20, the macOS player with the smoothed clock)

Setup: demo route (4, 0, 0), Balanced, minimap open, 1x, one variable changed per take, 60 s of cooling before each.
- **Backend speed:** simulated time from the server's run log, divided by the player's wall time from the run start
  to quit.
- **Viewer and motion:** from `-frame-times`, over the 60 s window that starts 10 s after the run.
- **Thermal:** `pmset -g therm` reported no warning before or after any take.
- **Recordings:** the experiment recordings were deleted after each take.

| Take | Backend speed | Goal reached (sim t) | Viewer FPS (mean, p99 frame) | Motion per frame, p1 / p50 / p99 | Frozen frames |
|---|---|---|---|---|---|
| E1: fullscreen, recorder (baseline) | **0.64x** | 40.74 s | 60.0, 17.8 ms | 7.3 / 9.9 / 12.6 ms | 0% |
| E2: windowed, recorder | **0.99x** | 40.74 s | 60.0, 17.7 ms | 15.2 / 16.6 / 18.0 ms | 0% |
| E3: fullscreen, no recorder | **0.66x** | 40.74 s | 60.0, 17.8 ms | 7.8 / 10.2 / 17.3 ms | 0% |
| E4: fullscreen, recorder, `-fps 30` | **0.67x** | 40.74 s | 29.6, 41.1 ms | 14.9 / 21.0 / 33.7 ms | 0% |

**Findings:**
- **Fullscreen slows the backend.** It is the only variable that does: windowed, the backend keeps real time (0.99x)
  with the recorder on. Fullscreen, it runs at about 0.65x whether or not the recorder runs and at 60 or 30 FPS.
  Neither the screen recorder nor the viewer's GPU load is the cause.
- **Likely mechanism, not shown:** macOS gives the fullscreen app its own Space, and the Docker Desktop VM, now in a
  background Space, is scheduled less. This would also explain the 0.62x of the Task 7 demo takes, which were
  fullscreen.
- **The smoothed clock works in the player:** no frozen frames in any take. Windowed at 1x, the motion per frame is
  15.2-18.0 ms against the ideal 16.7 ms. Fullscreen, it is even at the slower rate: 0.65 x 16.7 = 10.9 ms ideal,
  9.9 ms median. Before the fix, the replayed arrivals had shown 0-29 ms and 1.3% frozen.
- **The behavior is unchanged:** the goal was reached at the same simulation time, t = 40.74 s, in every take.

## Known limitations (4A, after the final review)

- **Reconnect after a rejection.** If the server's latest Goal frame was a rejection, a viewer that (re)connects does
  not see the goal still active, and its marker stays hidden until the next goal event. Relaunching the viewer, as
  for a windowed recording, reaches this.
- **Click right after a reconnect.** A click without a drag in that state sends heading 0, because no robot pose is
  known yet.
- **Reasons and HUD messages.** A rejected replacement's reason can stay on the minimap's goal line. HUD messages
  now expire after 5 s.
- **Pausing** moves the picture forward by the 0.1 s display lag.

The full list of deferred items is below ("Deferred items after 4A").

## Hand check (user, 2026-11-20): 4A complete

The user drove the rebuilt player by hand, windowed, against `plant_demo_server --mode realistic --routes lattice
--lidar`, and reported that everything worked: keys, minimap click and drag, cameras and recording.
- The server's run log of that session holds 652 s of simulation time and five minimap goals: four reached, one
  superseded by the next click.
- It has no contact events.

This closes plan Step 3, and with it stage 4A.

**User observation, outside the viewer (for 5B).** The robot turns the panel late, close to the door, although F2
plans 8 s ahead. The reason is F2's cost (phase 5A design 3.2):
- The weak comfort term pulls the arm toward `carry` at every sample.
- The clearance hinge acts only within 0.25 m of an obstacle.
- Nothing rewards turning early.

So the cheapest plan turns just in time. The run log shows no contact, but the late turn reads as a near-collision on
screen. Recorded as a 5B design question in the roadmap.

## Deferred items after 4A

Minor findings from the per-task and final reviews, recorded rather than fixed. The README's known limitations above
are the user-visible ones.

- **Backend and protocol:**
  - `cancel_goal` accepts a missing `client_request`.
  - The spec's cancel reason `stale` is never sent.
  - There is no backend test for cancel after reset.
  - `plant_stream.py` has no `cancel_goal`.
- **Journal:**
  - The header can be dropped by the 100 lines/s budget.
  - NaN or alternating wall clocks break the per-second accounting.
  - The session identity lacks config, bag, start and profile (a `hello_sha256` for 4C).
  - The folder name has second resolution; a collision now shows "journal off".
  - The demo's journal lacks the minimap and run lines.
- **Client and state:**
  - The read thread has no catch-all, and the handshake has no timeout.
  - Malformed headers throw in `Apply`.
  - "Pending" sticks after a duplicate ack.
  - The achieved speed is wrong while paused.
  - An overflow disconnect can loop while the main thread is stalled.
  - A non-numeric `start` throws on every frame.
- **Display:**
  - F (the camera's recenter) changes the view behind the HUD and journal.
  - Old scenario boxes stay after a restarted server.
  - The goal marker rebuilds its lines every frame.
  - The top-down RenderTexture is not destroyed.
- **Minimap:**
  - `goal_submitted` is journaled when the send fails.
  - A pending press survives while goals are disabled.
  - The 0.3 m drag threshold is a few pixels.
  - The panel overlaps the picker in narrow windows.
  - Scrolling over the map still drives the camera.
  - Control-drag in the free view may also set a goal.
- **Options and the scripted demo:**
  - The demo submits while goals are disabled, and can stall and then take over a later reset.
  - Option values are not range-checked, and unknown options are ignored.
  - One bad option drops the whole command line.
  - A relative `-frame-times` path fails under `open`.
  - `-quality` needs the exact name.
- **Build and tests:**
  - Each build rewrites three settings files.
  - The ARM64 setting persists.
  - Untested: codec bounds, MapFrame bad inputs, `run_stopped` with reason `error`, and display-clock cases beyond steady
    1x and one speed change.
  - The FK test checks the tool position only.
- **Controls:** Shift+R resets without confirmation.


## 4B-1 Task 4 live check (2026-11-20, Editor Play, overnight, driven by `unity command eval`)

Server: `plant_demo_server --mode realistic --routes lattice --lidar`. Scenario `v2_door_105_nav`: start (12, 0),
destination (16.275, 9.5, pi/2), through the 1.05 m door B into the east bay. The Diagnose preset was applied, and the
goal was submitted through the minimap's gesture path.

| Check | Observed |
|---|---|
| F2 plans | arriving with 21 samples each; seeds `vertical_carry`, `previous`, `level_extension`, `vertical_carry_right` won in turn during the approach |
| Ghosts and sweep | 6 ghosts and a 59-66 ring sweep per plan; seen in a top view threading door B (camera capture; the legend and IMGUI panels are not in camera captures) |
| Lidar | about 17,000-18,600 points in 5-6 scans (half a second); 1 scan skipped (the first, before any pose history) |
| Goal reached | at t 57 s; the plan overlays cleared on arrival |
| A second goal back through the door | ghosts and sweep rebuilt for the new plan |
| Shift+R | plan, ghosts, sweep and lidar points cleared |
| Lidar toggled off | the cloud dropped; the preset marked modified |

Not seen by eye: the legend (IMGUI), and the overlays' look in the player at 1920 x 1080. Both go to Task 6 and the
user's hand check.

## 4B-1 Task 5 live check (2026-11-20, Editor Play, overnight, driven by `unity command eval`)

Same server and scenario (`v2_door_105_nav`) as Task 4.

| Check | Observed |
|---|---|
| P presets | Off to Demo to Diagnose; each journaled with every toggle |
| Telemetry | B5 status, dwell, fallback, gate; F2 plan id, status, homotopy, seed, age, flags and the seeds table; goal and route ids; viewer connection, achieved speed, stream age, counters, FPS, frame time, pace. (Contacts showed "—" throughout: a bug, fixed after review, since the State's `contacts` is a count) |
| Console | three identical Unity errors collapsed into one entry (x3, error, viewer); a refused click shows as warnings (its `goal_ack` rejected and `goal_rejected`); `fallback` as a warning; connection changes as stream entries |
| K pace | `vSyncCount` set to 2 and journaled; that the viewer then runs at a steady 30 is Task 6's measurement, not shown here |
| Not exercised | search, copy and a contact burst (no contact occurred); the panels' look (IMGUI is not in camera captures). The user's hand check covers them |
| After the review fixes | with Diagnose and the minimap open, the four panel areas do not overlap; an export (called directly) wrote `console-<UTC>.txt` beside the journal and logged its path; each new journal file starts with an `overlay_state` line |
