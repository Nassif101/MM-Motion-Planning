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

The full list of deferred items is below ("Deferred items after 4A"). Fixed in 4B-1 (Task 1 review, `94c9cae`): a reconnect after a rejection no longer hides the active goal's route or plan.

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

## 4B-1 Task 6: pacing measurement (2026-11-21, the macOS player, overnight, screen kept awake)

Setup:
- the 4B-1 player, windowed 1920 x 1080, Balanced;
- the screen recorder on (its videos deleted);
- the scripted demo on `v2_open_yard_nav` to (18, 0, 0), with the server's lidar on;
- 10 s of warm-up, then 60 s measured, with 60 s of cooling before each run.

"Steady" means at least 99% of frames within ±20% of the paced period (16.7 ms at 60, 33.3 ms at the 30 lock).
`pmset -g therm` reported no warning before or after any run. The backend held 0.99-1.00x in every run (windowed, as
the slowdown experiments predicted).

| Run | Pace | Mean FPS | p50 | p99 | Max | Within ±20% | Steady |
|---|---|---|---|---|---|---|---|
| Off | 60 | 60.0 | 16.7 ms | 17.7 ms | 26.2 ms | 99.9% | yes |
| Demo | 60 | 60.0 | 16.7 ms | 17.7 ms | 18.6 ms | 100.0% | yes |
| Diagnose | 60 | 59.7 | 16.7 ms | 19.6 ms | 30.1 ms | 99.0% | yes (at the bound) |
| Off | 30 lock | 29.5 | 33.6 ms | 40.0 ms | 41.7 ms | 99.0% | yes |
| Demo | 30 lock | 29.7 | 33.4 ms | 39.2 ms | 40.7 ms | 99.6% | yes |
| Diagnose | 30 lock | 29.4 | 33.7 ms | 48.3 ms | 51.7 ms | 97.8% | no |
| Travelled path alone | 60 | 60.0 | 16.7 ms | 17.6 ms | 19.2 ms | 100.0% | yes |
| F2 plan path alone | 60 | 60.0 | 16.7 ms | 17.7 ms | 19.1 ms | 100.0% | yes |
| F2 ghosts alone | 60 | 60.0 | 16.7 ms | 17.7 ms | 18.6 ms | 100.0% | yes |
| Panel sweep alone | 60 | 60.0 | 16.7 ms | 17.7 ms | 19.1 ms | 100.0% | yes |
| Lidar alone (20,000 points) | 60 | 59.5 | 16.7 ms | 22.2 ms | 33.2 ms | 98.3% | no |

**Findings:**
- **The gate passes.** The Demo preset is steady at both paces.
- **Lidar is the one costly overlay.** Every other overlay alone costs nothing measurable; lidar alone misses the 99%,
  and it takes Diagnose with it at the 30 lock.
- **The Off preset (the 4A picture) is steady at 60:** p99 17.7 ms, against 4A's Balanced p99 of 30.0 ms on the same
  route. The difference was not isolated. The 4A run followed three earlier takes, and the display clock has changed
  since; Off is at least no worse than 4A.
- **Next step, by the design's optimization order:** lower the lidar density first.

**Optimization: lidar budget 20,000 to 10,000 points** (`PlantLidarOverlay`; the player rebuilt):

| Run | Pace | Mean FPS | p99 | Max | Within ±20% | Steady |
|---|---|---|---|---|---|---|
| Lidar alone (10,000 points) | 60 | 60.0 | 17.7 ms | 18.2 ms | 100.0% | yes |
| Diagnose | 60 | 60.0 | 18.7 ms | 20.4 ms | 99.5% | yes |
| Diagnose | 30 lock | 29.4 | 45.5 ms | 48.5 ms | 97.8% | no |

**Result:**
- Every preset is steady at 60.
- Demo and Off are steady at the 30 lock too. Diagnose is not: the 30 lock itself is the weaker pace on this Mac (even
  Off reaches only 99.0% there), so a steady Diagnose picture runs at 60.
- Recorded as the design allows: a Diagnose miss does not block 4B-1. The later optimization steps (simpler ghosts,
  HDRP settings) were not needed for the gate.

## 4B-1: known limitations and deferred items (after the final review, 2026-11-21)

**Known limitations** (visible to the user):
- **The panel sweep is approximate.** Joints are interpolated linearly between F2's samples. An arm turn while the base
  stands still has no samples in between, so the band joins its ends straight (the legend says so). It is not a
  collision envelope.
- **Lidar placement is approximate.** A point's time comes from its order in the scan. The plant drops rays without a
  hit, so a point's time can be off by part of the scan period (up to about 0.4 of it), which shifts points during a
  turn.
- **The minimap is a little narrower than in 4A** (0.36 of the screen width, not 0.42), so the telemetry and legend
  fit beside it.
- **Diagnose at the 30 lock is not steady** (97.8%); use 60 for a steady Diagnose picture.
- **The console search** keeps the keyboard until Escape, Return or a click outside the console.

**Deferred minor findings** (from the per-task and final reviews; recorded, not fixed):
- **Plans:**
  - a run of `stale_reused` plans never reads as stale;
  - a recorded-route session keeps its last plan after the goal ends;
  - the plan overlay is keyed on `plan_id` alone.
- **Sweep:**
  - a wrist full turn or an elbow flip whose end corners nearly coincide shows no band;
  - a NaN sample spoils the band.
- **Lidar:**
  - pending scans dropped past 8 are not counted;
  - there is no key to show ground hits;
  - a missing `livox_frame` fails silently.
- **Console:**
  - repeats collapse only when consecutive, and a collapsed entry keeps only its latest time;
  - events such as `replan_requested` are filed under `plant`, and nothing maps to the `b5` filter;
  - the export has no header, and a failed export is journaled as a path;
  - Unity errors lose their stack location;
  - backend `behind` events do not collapse.
- **Telemetry:**
  - it shows the newest frame, about 0.1 s ahead of the picture;
  - it allocates every frame while open;
  - the medium layer's fallback is shown on the B5 line.
- **Pacing:**
  - the pace label is wrong for other `-fps` values and for displays that are not 60 Hz;
  - the frame-time header records only the final pace.
- **Presets and journal:**
  - `Modified` stays set after a toggle is switched back;
  - `-preset demo,diagnose` is accepted.
- **Input:** scrolling over a panel still zooms the orbit camera.
- **Layout:**
  - the legend draws past its box at 12 rows;
  - the minimap's plan path colour is close to the route's;
  - ghost outlines do not fade.
- **Sessions:**
  - reconnecting to a different server that already has a session continues the old travelled path;
  - the ghosts are created at the first plan, a one-time stutter.
- **Tests:**
  - the panel-corner test tolerance is 0.15 m;
  - the scene-contract test does not check the presenters' own references.

## 4B-1 hand check (user, 2026-11-21): 4B-1 complete

The user drove the rebuilt player by hand against the shared-install server
(`ros2 run mobile_manipulator_campaign plant_demo_server --mode realistic --routes lattice --lidar`) and found that
the overlays look good.

Two observations about the planners, not the viewer, are recorded as 5B design questions in the roadmap:
- **A door stall.** At one door F2's ghosts showed a correct vertical swing, but the robot stalled with no plan
  executed.
- **The low-beam pivot.** At the 1.23 m low beam the robot passed with a side pivot where a rear arm extension would be
  more realistic.


## 4B-2 Task 3 live check (2026-11-21, Editor Play, driven by `unity command eval`)

Server: `plant_demo_server --mode realistic --routes lattice --lidar` from the shared install, with the Task 1 review
fixes. Scenario `v2_door_105_nav`: the Diagnose preset (overlay 7 on), a goal through door B to (16.275, 9.5, pi/2),
then one back to (12, 0, -pi/2). The session hello listed `plan_display_samples`, `scan_point_times` and
`b5_prediction`.

| Check | Observed |
|---|---|
| Idle before the first goal | no Prediction frame; overlay 7 draws nothing. Before the Task 1 review fix, a "held" plan was drawn at the map origin here |
| Tracking, both directions | B5 `ok`, executed; the path and 4 ghosts; B5 followed each F2 plan within one id of the newest |
| F2 plan in time | 81 display samples per plan (8 s every 0.1 s); the sweep through them: 81-108 rings (4B-1: 59-66 from F2's samples) |
| Lidar | placed with its point times ("exact"). The skipped count was not recorded in this run. The Task 3 review found that most scans were skipped: the last rays' float offset lands just past `t_end`. After the fix (point times clamped to the scan), a rerun skipped 1 scan in 41 s of tracking at about 250 FPS (the first, before any pose), with 5 scans and 8,400-9,800 points held |
| After arrival | no new frames; the last prediction turned stale (grey) after 1 s. Before the fix the frozen plan arrived as fresh, marked held |
| Seen in top-view camera captures | at door B, B5's violet ghosts just ahead of the robot, with F2's blue ghost outlines and the orange sweep continuing the turn beyond them; the colours are distinct |
| Not seen | a held period (none occurred in these runs; the held colour and label are covered by `HeldPredictionIsMarked` and `PredictionGhostsAtHalfSecondSteps`); an arm turn with the base at rest in isolation (covered by `SweepUsesDisplaySamples`); the legend and telemetry text (IMGUI is not in camera captures). The user's hand check covers the look |

## 4B-2: results and what changed from 4B-1 (2026-11-21)

From a 4B-2 server (capabilities `b5_prediction`, `plan_display_samples`, `scan_point_times`):
- **B5's prediction (overlay 7, key 7):** B5's solved 2 s plan, its path and ghosts at 0.5, 1, 1.5 and 2 s. Violet
  when B5 executes it (`ok`, `fallback`), red with "held (status)" when it does not, grey when more than 1 s old or while the executor holds (no goal tracked, a route search or a
  replan: B5 does not solve then).
  Telemetry adds B5's status, the F2 plan it follows and its horizon end.
- **Removed 4B-1 limitations:**
  - "The panel sweep is approximate": the sweep now goes through F2's plan every 0.1 s, so an arm turn with the base
    at rest is drawn as it moves. It is still not a collision envelope (B5 tracks the plan with an error).
  - "Lidar placement is approximate": each point takes its own time.
  - F2's ghosts now sit at evenly spaced plan times, not evenly spaced samples.
- **Kept as fallbacks** for an older server: the 4B-1 sweep, ghost spacing and lidar placement. The legend says which
  is in use.
- **Not seen live:** a held period (none occurred on the door runs); the tests cover its colour and label. The user's
  hand check is the first look at the legend and telemetry text.
- **Deferred, to raise before Phase 6 (user, 2026-11-21):** the active safety margins overlay and F2's losing
  candidates' paths.

## 4B-2 Task 4: pacing measurement (2026-11-21, the macOS player, screen awake)

The 4B-1 Task 6 setup (windowed 1920 x 1080, Balanced; the screen recorder on, its videos deleted; the scripted demo on
`v2_open_yard_nav` to (18, 0, 0) with lidar; 10 s warm-up, 60 s measured, 60 s cooling before each run), with the
4B-2 player and server. "Steady": at least 99% of frames within ±20% of 16.7 ms. `pmset -g therm` reported no warning
before or after either run.

| Run | Pace | Mean FPS | p50 | p99 | Max | Within ±20% | Backend | Steady |
|---|---|---|---|---|---|---|---|---|
| Diagnose (overlay 7 included) | 60 | 59.9 | 16.7 ms | 19.2 ms | 28.5 ms | 99.3% | 1.00x | yes |
| B5 prediction alone (`-overlays 7`) | 60 | 60.0 | 16.7 ms | 17.7 ms | 24.6 ms | 99.9% | 1.00x | yes |

B5's prediction costs nothing measurable alone, and Diagnose stays steady at 60 with it (4B-1's Diagnose: 99.5%,
p99 18.7 ms). The ghost count and rate were left as designed.

## 4C Task 4 live check (2026-11-22, Editor Play, driven by `unity command eval`)

**Recording.** The demo server with the Task 1 fixes (`--log`, one file per session, flushed at a pause), on
`v2_door_105_nav`, with a goal through door B and Diagnose, ran for 56 s. The log: 99 MB, 3,957 frames (lidar on: about
1.6 MB/s).

| Check | Observed |
|---|---|
| Overlays at the displayed time, live | In 8 samples over 40 s, the plan's and B5's prediction times were at or before the displayed time, never at the newest State (about 0.1 s ahead) |
| Replay: load | 1.8 s for the 99 MB log; the status line shows "Replay demo.mmlog", the scenario, and t / end |
| Replay: playing | At 2x, the displayed time advanced; the plan and prediction were at or before it; the robot was drawn. The first run of this check showed the robot hidden: the feed stopped at the clock, so no State lay after it. Fixed (the feed runs 0.1 s ahead; `ReplayShowsTheRobotAtTheClock`) |
| Restart (Shift+R) | Back to the start, paused; a new journal file (a take) began with the session header and the camera as it was |
| Journal identity | Both takes' headers name `demo.mmlog` with its SHA-256 (equal to `shasum -a 256` of the file) and the Hello hash |
| Orbit lines | Two orbit changes were each journaled as a moving line, then a line marked `still` at the resting pose |
| Not checked here | The legend and minimap look (IMGUI is not in camera captures); scroll over a panel (no mouse in automation). The Task 5 render captures the screen, panels included; the user's hand check covers the rest |

## 4C Task 5: the render, end to end (2026-11-22, the macOS player)

`tools/plant/render_video.py demo.mmlog journal-1.jsonl render-test.mp4 --from 15 --to 35`: the Task 4 log (99 MB,
56 s) and the take directed on it in the Task 4 live check. The player was rebuilt first.

| Check | Observed |
|---|---|
| Disk check | 1,201 frames estimated at about 1.1 GB (JPEG); 23 GB free; accepted |
| Render | 1,201 frames written at 1920 x 1080 in about 70 s with the encoding, then the player quit; `render-summary.json`: ok, no warning, 1 skipped line (a key action) |
| Video | `ffprobe`: H.264, 1920 x 1080, 60/1 fps, 1,201 frames, 20.0 s, 47 MB (CRF 18, about 19 Mbit/s) |
| Frames deleted | the frames folder was removed after encoding |
| Picture | Three frames from the video: the robot and route at door B under the directed orbit camera; the minimap open as directed; the status line "Replay \| scenario \| t" without viewer measurements |
| Found on the way | The first attempt hung: a relative `-out` path, with the player started by `open` from `/`. Fixed: the script resolves its paths, and the player quits when it cannot create the folder |
| After the Task 5 review fixes | Two check renders launched through `open` stalled with no frame written and no error (the new timeout ended them). The same player run directly rendered every frame, so the script now runs the binary itself (under `caffeinate -d -i`, with its log in the frames folder). A 5 s check: 301 frames, 1920 x 1080, 60 fps, in 21 s |
| Not shown by this take | ghosts, sweep, prediction and legend (the take kept the Off preset); a free-camera move. The user's hand check uses a real directing pass |
