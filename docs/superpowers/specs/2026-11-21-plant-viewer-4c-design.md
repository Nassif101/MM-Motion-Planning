# Plant viewer stage 4C: replay, directing and offline render (design)

Date: 2026-11-21. Status: approved by the user (2026-11-21); the builder check's questions are settled (section 8). Parent: [spec 4 design](2026-11-20-plant-unity-viewer-design.md),
sections 2, 6.6 and 9 (4C). Builds on 4A, 4B-1 and 4B-2 (branch `plant-viewer-4b2`).

## 1. Intent

The user wants a **clean recording to showcase in the thesis defense** (user, 2026-11-21). A live screen recording on
the MacBook Air is limited by what the Mac renders in real time. An offline render is not: it spends as long as it
needs on each frame, so the video plays at an even 60 FPS at the High Fidelity quality level.

A live demo, on the Mac or later on the PC, stays what it is now: the viewer against the server (user). Replay is not
needed for it.

## 2. Scope (user, 2026-11-21)

- **4C = replay plus offline render**, with the camera work directed in a replay (approach B, section 4).
- **Deferred to a later stage (4D), only if a live PC demo is wanted:** the full FPS qualification across workloads,
  and the Windows player.
- **Folded in from the deferred lists:** the 4A, 4B-1 and 4B-2 items that the replay or the render depend on
  (section 5.6), and four look items, because the video shows the minimap and legend (user: "lets show the minimap
  and legend").

Non-goals:
- No changes to what the planners or the plant compute.
- No scripted shot list (approach C), and no camera paths authored outside the player.
- No seeking backward in a replay beyond restarting from the start.
- No replay of logs older than schema 3 (the baseline replicas are schema 1; the viewer speaks schema 3 only).

Anti-goals (a failure even when it works):
- A video whose overlays do not match the robot shown in the same frame.
- A render that silently uses a journal from a different run, or a run log that does not match its journal.
- A render that fills the disk (19 GB free on 2026-11-21; the disk was at 98% on 2026-11-19).

## 3. Constraints

- Unity 6000.5.2f1, HDRP, the existing viewer scene and player; Unity CLI only. No new Unity packages.
- `ffmpeg` installed with Homebrew on the host (planned in the spec 4 design, section 9).
- Run logs as they are: `MMRUNLG1`, then zlib chunks of whole frames (stream-protocol.md, Run logs). Schema 3.
- The fanless Mac: never overlap a render with timed backend runs.
- Never commit frames, videos, journals or logs.
- Keep the plan compact (user).

## 4. Approaches (how the render gets its camera work; user chose B)

- **A. Re-render a live take.** The render replays a live session's log and journal. The camera work must be right
  during the live run; a clumsy move means running the robot again, and a rerun can differ (the realistic plant is
  chaotic).
- **B. Direct the shots in a replay. Chosen.** The run happens once, with its log saved. The user opens the log in
  the player's replay mode and drives the cameras, presets and panels while watching, as often as needed, on the
  identical robot motion. The journal of one directing pass, with the log, drives the render. A live session's
  journal works the same way, so A comes with B.
- **C. A scripted shot list.** Precise and repeatable, but shots are authored in a text file, not by eye. Not now.

## 5. Design

### 5.1 Workflow

1. **Record the run once:**
   - a live session on `plant_demo_server --log FILE` (section 5.2), or
   - a campaign run, `plant_campaign ... --prediction` (its log, `run.mmlog`).
2. **Direct:** `PlantViewer.app --args -replay LOG`.
   - The run plays from its log, with the live viewer's cameras, presets, overlays, panels and minimap.
   - Each pass through the run is one take with its own journal file.
3. **Render:** `tools/plant/render_video.py LOG JOURNAL OUT.mp4 [--speed S] [--from T] [--to T] [--png]`:
   - it runs the player in render mode;
   - it checks free disk space first;
   - it encodes the frames with `ffmpeg`, then deletes them.

### 5.2 Backend: one log per session

Today each reset reopens the same `--log` file, so a server run keeps only its last session. Change:
- the first session writes `FILE`;
- later sessions write `FILE` with `-2`, `-3`... before the extension;
- an existing file is never overwritten: a session takes the next free number (call).

Nothing else changes; the log already carries every frame the viewer draws, B5's prediction included (on by default
in the demo server).

### 5.3 Replay mode in the player

- **Source:** a run-log reader that yields the same frames the stream client yields. The live code path from
  `FrameCodec` on is reused; `ViewerState` does not know the difference.
- **Clock:** a replay clock of simulation time.
  - Space plays and pauses; `[` and `]` change the speed (as live); a restart key (call: Shift+R, the live reset key)
    starts again from the log's start.
  - The end of the log pauses.
- **Disabled in replay:** goal submission, cancel and reset to another scenario. The HUD says "replay" and names the
  log.
- **Window:** the directing pass runs in a 1920 x 1080 window, the video's size (call). Panels are laid out as
  fractions of the screen, so framing and panel layout in the video are then exactly what was directed. The player's
  default stays fullscreen for live use; replay mode sets the window.
- **Overlays at the displayed time** (section 5.6, item 1).

### 5.4 Journal for directing and rendering

- **Identity** (call, after the builder check): two sessions with the same scenario and seed have byte-identical
  Hellos, so the Hello alone cannot tell runs apart.
  - A directing pass in replay writes, in its first line, the log's file name and a SHA-256 of the whole log file.
    The render refuses a journal whose hash does not match the log.
  - A live journal (approach A) has only the session's identity (section 6.6 of the parent design) and a hash of
    the session Hello frame. The render accepts it when these match the log's Hello, with a warning that identical
    sessions cannot be told apart.
- **The first line always survives** the 100-lines-per-second budget (a deferred 4A item).
- **The orbit camera:** its yaw offset, pitch and distance are journaled at 10 Hz while they change. F (recenter) and
  scroll zoom are journaled as well. The free camera's pose is journaled as now.
  - Call: journal the orbit's parameters, not its output pose. The camera then follows the robot smoothly at render
    time, instead of interpolating a moving pose.
- **Time base:** every line's `t` is simulation time, so slow-motion directing maps to the run's own time.
- **One take per pass:** a restart in replay starts a new journal file.

### 5.5 Render mode

- `-render LOG -journal FILE -out DIR [-speed S] [-from T] [-to T] [-png]`:
  - the player sets a fixed capture frame rate of 60 (Unity's `Time.captureFramerate`);
  - each rendered frame advances the replay clock by `speed / 60` s of simulation time;
  - the journal's lines are applied at their simulation times: camera view, orbit parameters, free-camera pose
    (interpolated), presets, overlay and panel toggles, minimap open or closed;
  - lines about pause, speed, restart, cursor, goals, cancel and step are skipped; the summary counts the skipped
    kinds. A goal's marker and route appear when the log's Goal and Route frames do (call).
  - **Camera moves made while paused become cuts** (user, 2026-11-21): the render follows simulation time only, so
    the moves made during a pause take effect at once at that time. Freeze-frame moves (a second clock for paused
    stretches) are not built.
- **Output:** numbered frames at 1920 x 1080; JPEG at quality 95 by default (about 0.5 MB each), PNG with `-png`.
- **What is in the frame is what was on screen:** open panels (legend, minimap, status line) are captured with the
  3D picture.
- **Quality:** High Fidelity by default in render mode (`-quality` overrides).
- **Window:** a 1920 x 1080 window, since the IMGUI panels draw to the screen.
- **Status line and telemetry in render mode** (call): they show "replay", the scenario and the simulation time, and
  leave out the viewer's own measurements (FPS, frame time, stream age, dropped and malformed counts, achieved
  speed), which would show the render's rate.
- **Exit:** the player quits after the last frame and writes a summary: frames written, simulation span, any journal
  lines it could not apply.

### 5.6 Deferred items folded in

1. **Overlays at the displayed time** (4B-1 telemetry; 4B-2 M7). F2's plan, B5's prediction and the telemetry are
   taken at the time the robot is displayed, not from the newest frame about 0.1 s ahead. This applies in replay,
   render and live modes alike.
2. **A recorded-route run keeps its last plan after the goal ends** (4B-1; fixed here): the plan clears when a
   recorded goal ends (its goal events, or a change of the State's `goal` index). Campaign logs are recorded-route
   runs.
3. **Journal:** the identity and first-line items of section 5.4 (4A).
4. **Scrolling over the minimap or a panel no longer zooms the orbit camera** (4A, 4B-1). Otherwise a directing pass
   would journal that zoom into the video.
5. **Look, since the video shows the minimap and legend:**
   - the legend fits its rows (it draws past its box at 12);
   - the legend has a swatch for the grey "stale" colour;
   - the minimap's plan path colour is distinct from the route's;
   - a plan that stands still does not stack F2's six ghosts on one spot.

## 6. Testing

- **EditMode:**
  - the run-log reader on a fixture log (chunk boundaries, a truncated chunk, a non-schema-3 Hello refused);
  - the replay clock (play, pause, speed, restart, end of log);
  - overlays at the displayed time (a Plans frame newer than the displayed time is not shown yet);
  - journal identity and the first line under the budget;
  - orbit-parameter lines written and applied;
  - render frame timing (frame n at `from + n speed / 60`);
  - the mismatch refusal;
  - the four look items where they are testable.
- **Backend (gtest):** a second session on one server writes `FILE-2`.
- **End to end on the Mac:**
  - a live session logged;
  - a directing pass in replay with the orbit and free cameras, the minimap and legend open;
  - a render of 20 s to MP4.
  - Check that the video plays at 60 FPS without dropped or repeated frames, and that the camera follows the
    directed moves. The user watches it (hand check).

## 7. Left to the builder

Class names and internals; the restart key if Shift+R conflicts; the journal's orbit-line field names; the render
summary's format; JPEG encoding through Unity or after capture; how the render script finds the player; the exact
ghost de-stacking rule.

## 8. Builder check (2026-11-21)

Questions a fresh builder raised, and how they were settled:
- Matching a journal to its log, when Hellos can be identical: the log file's hash in replay journals; live journals
  matched by the session, with a warning (section 5.4; call).
- Camera moves while paused: cuts (user).
- The directing window against the 1920 x 1080 video: both at 1920 x 1080 (sections 5.3 and 5.5; call).
- A live journal's goal lines: skipped and counted; goals appear from the log's frames (section 5.5; call).
- The status line and telemetry in a render: no viewer measurements (section 5.5; call).
- The recorded-route plan after its goal: cleared (section 5.6; call, confirming the deferred item as a fix).
