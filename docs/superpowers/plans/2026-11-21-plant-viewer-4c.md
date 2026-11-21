# Plant viewer stage 4C (replay, directing and offline render) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** a clean defense video: a run recorded once, its camera work directed in a replay, then rendered offline at
an even 60 FPS in High Fidelity to MP4.

**Architecture:**
- The backend keeps one log per demo session.
- The viewer core gains a run-log reader, a replay clock and a frame gate (overlays at the displayed time), plus a
  journal track that gives the directed view at any simulation time.
- The Unity layer adds `-replay` (directing) and `-render` (fixed-step frame capture); `render_video.py` wraps the
  render and `ffmpeg`.

**Tech Stack:** C++17 and gtest (container); Unity 6000.5.2f1, HDRP, NUnit EditMode, the Unity CLI; Python 3 and
pytest; `ffmpeg` (Homebrew).

**Spec:** [2026-11-21-plant-viewer-4c-design.md](../specs/2026-11-21-plant-viewer-4c-design.md); parent
[spec 4 design](../specs/2026-11-20-plant-unity-viewer-design.md).

## Global Constraints

- Run logs: `MMRUNLG1`, then chunks `[u32 raw_len][u32 zlib_len][zlib bytes]` of whole frames; schema 3 only in the
  viewer.
- No new Unity packages; `ffmpeg` from Homebrew; never commit frames, videos, journals, logs or host paths.
- Render: 1920 x 1080, a fixed 60 frames per rendered second (`Time.captureFramerate = 60`), frame n at simulation
  time `from + n * speed / 60`; JPEG quality 95 by default, PNG with `-png`; High Fidelity unless `-quality`.
- Directing and rendering both in a 1920 x 1080 window; live use keeps its fullscreen default.
- Camera moves made while paused are cuts (user).
- The render checks free disk before writing and deletes the frames after encoding.
- Never overlap a render or a Play session with timed backend runs (fanless Mac). Stop Play after each task.
- Process: branch `plant-viewer-4c`; per task: commit, a review prompt in `docs/reviews/plant-viewer/` (never
  committed), a subagent review, fixes test-first; a final whole-branch review.

## Review Focus

1. **A log cut short** (a server killed mid-run, a truncated last chunk): the replay plays to the last whole frame and
   says the log ends early; it does not refuse the whole log. Test: Task 2 `TruncatedLogPlaysToTheLastWholeFrame`.
2. **A journal from another run:** a replay journal whose log hash differs is refused; a live journal whose session
   Hello matches is accepted with a warning. Test: Task 3 `MismatchedJournalIsRefused`.
3. **Overlays ahead of the robot:** a Plans or Prediction frame newer than the displayed time is not drawn yet, in
   replay, render and live modes. Test: Task 2 `OverlaysWaitForTheDisplayedTime`.
4. **A restart while directing:** a new journal file starts, so one file never runs backward in time. Test: Task 3
   `RestartStartsANewTake`.
5. **A render range past the log's end, or a render too big for the disk:** the range is clamped to the log; the
   script refuses before writing. Tests: Task 3 `RenderRangeIsClampedToTheLog`; Task 5
   `RefusesWhenTheDiskIsTooSmall`.

---

### Task 1: one log per demo session, and a run-log fixture

**Files:**
- `mobile_manipulator_campaign/{include,src}/session.{hpp,cpp}` (`SessionConfig::number_logs`), `src/demo_server.cpp`
  (sets it), `src/plant_demo_server.cpp` (usage comment);
- a new `tools/plant/make_runlog_fixture.py` writing `tools/plant/fixtures/runlog_v3.mmlog`;
- tests in `test/test_demo_server.cpp` and `tools/plant/test_plant_stream.py`.

**Interfaces:**
- Produces: `numbered_log_path(const std::string & path, int session) -> std::string` (session 1: `path`; n: the stem
  plus `-n` before the extension), skipping names that exist; `SessionConfig::number_logs` (default false, so
  `plant_campaign` keeps writing exactly `run.mmlog`).
- Produces the fixture: a schema-3 session Hello (capabilities `b5_prediction`, `plan_display_samples`,
  `scan_point_times`; scenario `v2_door_105_nav`), States at t = 0.02 ... 0.40 (20), one Plans at t 0.1 with blocks 0
  and 1, Predictions at 0.1 and 0.2, one Goal frame; in two chunks split after the 7th frame.

- [ ] **Step 1: Write the failing tests:**
  - `EachSessionWritesItsOwnLog`: two resets on one demo server with `--log DIR/demo.mmlog` give `demo.mmlog` and
    `demo-2.mmlog`, each starting with its own Hello.
  - `AnExistingLogIsNeverOverwritten`: with `demo.mmlog` present, the first session writes `demo-2.mmlog`.
  - `test_runlog_fixture_reads_back`: `RunLog(fixture).frames()` yields the frames above, in order.
- [ ] **Step 2:** Build in the container and run. Expected: fail.
- [ ] **Step 3: Implement** (the session opens `numbered_log_path` when `number_logs` is on; the fixture script
  uses `plant_stream.encode` and `zlib`).
- [ ] **Step 4:** Run the new tests and the `mobile_manipulator_campaign` suite, and pytest `tools/plant`. Expected:
  pass.
- [ ] **Step 5:** Commit, review prompt, subagent review, fixes.

### Task 2: viewer core: replay and overlays at the displayed time (C#)

**Files:** new `Core/{RunLogReader,ReplayClock,ReplayFeed,FrameGate}.cs`; `Core/ViewerState.cs`; tests in a new
`Viewer/ReplayTests.cs` and `Viewer/ViewerStateTests.cs`.

**Interfaces:**
- Consumes: the Task 1 fixture `tools/plant/fixtures/runlog_v3.mmlog`.
- Produces:
  - `RunLog RunLogReader.Load(string path)`, with `IReadOnlyList<Frame> Frames`, `string Sha256` (of the whole file),
    `bool EndsEarly`, `double StartT`, `double EndT` (first and last State); throws `MalformedFrameException` for a
    bad magic or a non-schema-3 Hello;
  - `ReplayClock { double T; bool Playing; double Speed; void Advance(double wallDt); void Restart(); bool AtEnd; }`
    (speed steps as live; it stops at `EndT`);
  - `ReplayFeed(RunLog log)`, `void FeedUntil(double t, Action<Frame> apply)` (in order, each frame once), `void
    Reset()`;
  - `FrameGate { void Offer(Frame f); void Release(double displayT, Action<Frame> apply); void Clear(); }`: holds
    Plans and Prediction frames until `displayT >= header t`; any other frame passes at once;
  - `ViewerState.TrySampleAt(double simT, out RobotSample sample)` (interpolated from the State history, as
    `TryPoseAt`, with joints);
  - `ViewerState.StateHeaderAt(double simT)` (the newest header at or before `simT`);
  - a recorded-route session clears `Plan` when its goal ends (a `goal_reached`/`goal_failed` event, or a change of
    the State's `goal` index).

- [ ] **Step 1: Write the failing tests:**
  - `RunLogReadsTheFixtureAcrossChunks`: 25 frames, Hello first, `StartT` 0.02, `EndT` 0.40, a stable `Sha256`.
  - `TruncatedLogPlaysToTheLastWholeFrame`: the fixture cut inside its last chunk gives the first chunk's frames and
    `EndsEarly`.
  - `NonSchema3LogIsRefused`
  - `ReplayClockPlaysPausesAndRestarts`: speed 0.5 over 1 s of wall time is 0.5 s; pause holds; `Restart` returns to
    `StartT`; it stops at `EndT` with `AtEnd`.
  - `ReplayFeedAppliesEachFrameOnce`
  - `OverlaysWaitForTheDisplayedTime`: a Plans frame at 0.1 offered at display time 0.05 is not applied; at 0.1 it
    is.
  - `SampleAtIsExactBetweenStates`
  - `RecordedPlanClearsWhenItsGoalEnds`
- [ ] **Step 2: Run** (batch `unity test . --mode EditMode`, Editor closed). Expected: fail to compile.
- [ ] **Step 3: Implement.** zlib chunks: skip the 2-byte zlib header and read with `DeflateStream`.
- [ ] **Step 4: Run** the whole EditMode suite. Expected: pass.
- [ ] **Step 5:** Commit, review prompt, subagent review, fixes.

### Task 3: viewer core: journal identity, orbit lines and the journal track (C#)

**Files:** `Core/{ViewJournal,JournalRecorder}.cs`; new `Core/{JournalTrack,RenderPlan}.cs`; tests in
`Viewer/{ViewJournalTests,JournalRecorderTests}.cs` and a new `Viewer/RenderTests.cs`.

**Interfaces:**
- Consumes: `RunLog` (Task 2).
- Produces:
  - journal header fields: `log` (file name) and `log_sha256` in replay; `hello_sha256` (SHA-256 of the session Hello
    frame's bytes) always;
  - the header is never thinned or dropped by the line budget;
  - an `orbit` line kind: `{"yaw": deg, "pitch": deg, "distance": m}`; a `recenter` line;
  - `JournalRecorder.NewTake()` starts a new file (a replay restart);
  - `JournalTrack.Load(string path)`, `ViewAt(double simT) -> JournalView` with the camera view, the orbit parameters
    and the free-camera pose (both interpolated between lines), the preset, overlay and panel toggles, minimap open;
    `IReadOnlyDictionary<string,int> SkippedKinds` (pause, speed, restart, cursor, goals, cancel, step);
  - `RenderPlan.Check(RunLog log, JournalTrack journal) -> (bool ok, string warning, string refusal)`;
  - `RenderPlan.Frames(RunLog log, double from, double to, double speed) -> (int count, Func<int,double> simTimeOf)`,
    with the range clamped to `[StartT, EndT]`.

- [ ] **Step 1: Write the failing tests:**
  - `HeaderSurvivesTheLineBudget`
  - `ReplayHeaderNamesTheLog`, `LiveHeaderHasTheHelloHash`
  - `OrbitLinesAreWrittenAndInterpolated`: lines at t 1 (yaw 0) and t 2 (yaw 90) give 45 at t 1.5.
  - `ToggleStateIsTheLatestAtOrBeforeT`
  - `SkippedKindsAreCounted`
  - `RestartStartsANewTake`
  - `MismatchedJournalIsRefused`: a replay journal with another `log_sha256` is refused; a live journal with a
    matching `hello_sha256` is accepted with a warning.
  - `FrameTimesFollowSpeed`: from 2, speed 0.5: frame 120 is at t 3.
  - `RenderRangeIsClampedToTheLog`
- [ ] **Step 2: Run.** Expected: fail to compile.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** the whole suite. Expected: pass.
- [ ] **Step 5:** Commit, review prompt, subagent review, fixes.

### Task 4: Unity: replay and directing, live sync, look items

**Files:** `Unity/{PlantViewerBootstrap,PlantViewerHud,PlantLegend,PlantMinimap,PlantPlanOverlay,PanelLayout}.cs`,
`Runtime/Control/ThirdPersonRobotCamera.cs`, `Core/ViewerOptions.cs`, `Editor/PlantViewerSetup.cs` if wiring changes;
tests in `Viewer/{ViewerOptionsTests,PlantOverlayTests,PlantViewerInputTests}.cs`.

**Interfaces:**
- Consumes: Tasks 2 and 3.
- Produces:
  - `ViewerOptions.Replay` (path; `-replay FILE`);
  - `ThirdPersonRobotCamera.Orbit { get; set; }` as `(float yaw, float pitch, float distance)`; `bool InputBlocked`
    (the bootstrap sets it while the pointer is over the minimap or a panel); an event `Recentered`;
  - `PanelLayout.Legend` sized to its rows (12 rows fit);
  - `PlantPlanOverlay.GhostStates` drops a state within 0.05 m and 0.05 rad (base) and 0.05 rad (joints) of the
    previous one kept.
- Behaviour:
  - In replay: frames come from `ReplayFeed` through `FrameGate` at the replay clock's time, and the robot is drawn
    with `TrySampleAt`. Space, `[`, `]` drive the clock, and Shift+R restarts it (and `JournalRecorder.NewTake()`).
    Goals, cancel and scenario reset are off. The HUD shows "replay", the log's name and its early end if any. The
    window is 1920 x 1080.
  - Live: Plans and Prediction pass through `FrameGate` at the displayed time; telemetry uses `StateHeaderAt`.
  - Journal: orbit lines at 10 Hz while the orbit changes; recenter lines.
  - Look: the legend has a grey "stale" swatch row; the minimap's plan path colour is distinct from the route's.

- [ ] **Step 1: Write the failing tests:** `ReplayOptionParses`, `OrbitParametersRoundTrip`,
  `LegendFitsTwelveRows`, `StillPlanGhostsDoNotStack`, `MinimapPlanColourDiffersFromRoute`.
- [ ] **Step 2: Run.** Expected: fail.
- [ ] **Step 3: Implement**, then rebuild the scene (`unity command build_plant_viewer`) if wiring changed.
- [ ] **Step 4: Run** the whole suite. Expected: pass.
- [ ] **Step 5: Live check in Editor Play:**
  - record a short live session with the new server (`--log`);
  - replay it: play, pause, speed, restart; check that the overlays sit on the robot;
  - direct a pass with the orbit and free cameras, the minimap and legend open;
  - check the journal's header and orbit lines.

  Stop Play; record in the README.
- [ ] **Step 6:** Commit, review prompt, subagent review, fixes.

### Task 5: render mode, the render script, the video and records

**Files:** `Unity/PlantViewerBootstrap.cs` (or a new `Unity/PlantRenderDriver.cs`), `Core/ViewerOptions.cs`; a new
`tools/plant/render_video.py` with `tools/plant/test_render_video.py`; `docs/running-the-stack.md`,
`docs/experiments/plant-viewer/README.md`, `docs/roadmap.md`.

**Interfaces:**
- Consumes: `RenderPlan`, `JournalTrack` (Task 3); replay (Task 4).
- Produces:
  - `-render LOG -journal FILE -out DIR [-speed S] [-from T] [-to T] [-png] [-quality NAME]`. It refuses a mismatched
    pair before the first frame (exit code 2 and the reason in `DIR/render-summary.json`).
  - Frames `DIR/frame_000000.jpg`... captured at the end of each frame (panels included).
  - The status line shows "replay", the scenario and the simulation time, without viewer measurements.
  - `render-summary.json`: frames, the simulation span, the skipped kinds, any warning.
  - `render_video.py LOG JOURNAL OUT.mp4 [--speed] [--from] [--to] [--png]`. It estimates the frames' size (0.6 MB
    JPEG, 3.5 MB PNG, times 1.5) and refuses when it exceeds the free disk. It runs the player and waits, then runs
    `ffmpeg -framerate 60 -i frame_%06d.<ext> -c:v libx264 -pix_fmt yuv420p -crf 18 OUT.mp4`, and deletes the frames.

- [ ] **Step 1: Write the failing tests:**
  - `RenderOptionsParse`
  - pytest `test_estimate_and_refusal` (`RefusesWhenTheDiskIsTooSmall`)
  - pytest `test_ffmpeg_command`
- [ ] **Step 2: Run.** Expected: fail.
- [ ] **Step 3: Implement**; install `ffmpeg` with Homebrew; rebuild the player in batch and revert the three
  rewritten settings files.
- [ ] **Step 4: Run** the whole EditMode suite and pytest. Expected: pass.
- [ ] **Step 5: End to end:**
  - render 20 s of the Task 4 directing pass to MP4;
  - check with `ffprobe`: 1920 x 1080, 60 FPS, the expected frame count;
  - check the frame-to-frame timing: no repeated or dropped frames (the simulation time of each frame from the
    summary);
  - confirm the frames are deleted;
  - the user watches the video (hand check).
- [ ] **Step 6: Records:**
  - running-the-stack: the record, direct and render workflow;
  - README: results and the 4C deferred items;
  - roadmap: the 4C outcome and 4D (FPS qualification, Windows player).
- [ ] **Step 7:** Commit; the final whole-branch review and its fixes; push and merge on the user's word.
