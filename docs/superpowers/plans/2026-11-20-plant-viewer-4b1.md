# Plant viewer stage 4B-1 (inspection from existing data) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** the plant viewer shows what F2 plans (path, ghosts, the panel's planned sweep), what happened (travelled
path), why (telemetry, console) and what the robot senses (lidar), each togglable, with Off/Demo/Diagnose presets and
steady frame pacing, from schema-3 data only.

**Architecture:** the pure logic (plan keying, path and pose history, sweep construction, lidar placement, console
store, overlay settings, options) goes into `MotionPlanningSim.Viewer.Core` with EditMode tests. Thin Unity presenters
in `MotionPlanningSim.Viewer` draw it. The backend and the protocol are unchanged.

**Tech Stack:** Unity 6000.5.2f1, HDRP 17.5.0, Input System, Newtonsoft Json, NUnit EditMode, Unity CLI.

**Spec:** [2026-11-20-plant-viewer-4b1-design.md](../specs/2026-11-20-plant-viewer-4b1-design.md); parent
[plant viewer design](../specs/2026-11-20-plant-unity-viewer-design.md); 4A plan
[2026-11-20-plant-viewer-4a.md](2026-11-20-plant-viewer-4a.md).

## Global Constraints

- Schema stays 3. No backend, protocol, campaign or package change.
- Branch `plant-viewer-4b1` (from `plant-routing-unity-viewer`). One or more commits per task. After each task: commit,
  write a review prompt in `docs/reviews/plant-viewer/` (git-ignored), and stop for the user. A final whole-branch
  review at the end.
- The viewer scene holds no ROS, physics, sensor or controller component (scene-contract test).
- Overlays: 1 route, 2 travelled path, 3 F2 plan path, 4 F2 ghosts, 5 panel sweep (planned, approximate), 6 lidar
  (approximate placement).
- Keys: 1-6 overlays, P presets, T telemetry, backquote console, L legend, H 4A status line, K pace (60 / 30 lock);
  M stays the minimap. Nothing fires while a text field has focus.
- Presets:
  - **Off:** exactly the 4A picture (route line and goal marker on, status line on, minimap unchanged, no 4B overlays).
  - **Demo:** route, travelled path, ghosts and sweep; status line, minimap, telemetry, console and lidar off.
  - **Diagnose:** everything on.
- Limits:
  - ghosts: 6 by default, 12 at most;
  - sweep: at most 0.05 m of corner movement between steps;
  - lidar: at most 20,000 points, kept for 0.5 s of simulation time, ground hits hidden by default;
  - travelled path: 5,000 poses;
  - a plan more than 1 s older than the newest State is drawn stale;
  - console: 2,000 entries and 200 new entries per frame, with repeats collapsed.
- Panel geometry: 1.2 x 1.2 x 0.04 m (`qualified_payload.json`) at `tool0/PayloadPanel`.
- Pacing:
  - `-fps 30` and K mean `QualitySettings.vSyncCount = 2`; other `-fps` values keep `targetFrameRate`.
  - Stable means at least 99% of frames within ±20% of the paced period.
  - The Demo preset must be stable at 60 or at the 30 lock. Diagnose is measured and recorded.
- Overlay toggles, preset and pace changes, and console exports are journaled (thinnable: no).
- Unity via CLI only; `unity test . --mode EditMode` from `motion-planning-sim/` with the Editor closed, or `unity command
  run_tests` with it open. Remove `test-results.xml`. Revert the three settings files a build rewrites. Stop Play
  after each task. Never overlap Unity with timed runs (fanless Mac).

## Review Focus

1. **A reset or new goal while a Plans frame is in flight:** no ghosts, sweep or plan path of the old goal may appear.
   Test: Task 1 `PlansFollowTheActiveGoalAndEpoch`.
2. **A scan whose times fall outside the pose history** (just after a reset, while paused, a stalled main thread): it
   is skipped and counted, never drawn at a wrong pose. Test: Task 2 `AScanWithoutPoseHistoryIsSkipped`.
3. **A console flooded by repeated errors** (a contact every cycle, a Unity exception every frame): bounded,
   collapsed, and within its per-frame budget. Test: Task 3 `ConsoleCollapsesRepeats`, `ConsoleKeepsItsFrameBudget`.
4. **A server without the data** (recorded routes, B5 alone, no `--lidar`): the legend says "no data", and nothing
   stale is drawn. Test: Task 1 `AvailabilityFollowsTheSession`.
5. **Inspection while paused:** toggles, presets and panels work, and their journal lines carry the displayed time.
   The last plan's ghosts stay. Test: Task 3 `PresetsSetTheDesignedToggles` (journal fields), Task 5 keymap tests.

---

## File map

| Path (under `motion-planning-sim/Assets/`) | Responsibility |
|---|---|
| `Scripts/Viewer/Core/MediumPlanStore.cs` | Plans decoding, keying to goal and epoch, staleness, availability |
| `Scripts/Viewer/Core/PathHistory.cs` | the bounded travelled path |
| `Scripts/Viewer/Core/ViewerState.cs` | applies Plans and Scan frames; pose history by simulation time |
| `Scripts/Viewer/Core/PanelSweep.cs` | sweep rings from plan samples and a panel-corner function |
| `Scripts/Viewer/Core/LidarCloud.cs` | approximate per-point times, placement, the bounded cloud |
| `Scripts/Viewer/Core/ConsoleLog.cs` | console entries, collapse, bounds, budget, filters, export, event mapping |
| `Scripts/Viewer/Core/OverlaySettings.cs` | toggles, presets, journal fields |
| `Scripts/Viewer/Core/ViewerOptions.cs` | `-fps 30` as the vsync lock, `-preset` |
| `Scripts/Viewer/Unity/PlantOverlays.cs` and siblings | paths, ghosts (pooled), sweep mesh, lidar points, legend |
| `Scripts/Viewer/Unity/PlantTelemetryPanel.cs`, `PlantConsolePanel.cs` | IMGUI panels |
| `Scripts/Viewer/Unity/{PlantViewerBootstrap,PlantViewerInput,PlantViewerHud}.cs` | wiring, keys, the status-line toggle |
| `Scripts/Viewer/Editor/PlantViewerSetup.cs` | adds and wires the presenters, the ghost material |
| `Tests/EditMode/Viewer/*.cs` | tests named below |
| `docs/experiments/plant-viewer/README.md` | the 4B-1 records |

---

### Task 1: plans, paths and pose history (Core)

**Files:** create `Core/MediumPlanStore.cs`, `Core/PathHistory.cs`; modify `Core/ViewerState.cs`; tests
`Viewer/MediumPlanStoreTests.cs`, `Viewer/ViewerStateTests.cs`.

**Interfaces:**
- Produces:
  - `sealed class MediumPlan { long PlanId, RouteId, Epoch, GoalId; string Status, Homotopy, Seed; bool Capped, Waiting,
    FullMultistart, HomotopySwitched; IReadOnlyList<(string name, double merit, bool feasible, double progress, bool
    merged)> Seeds; IReadOnlyList<double[]> Samples /* 9: x y yaw q0..q5 */; double ReceivedAtSimT; }`;
  - `ViewerState.Plan` (`MediumPlan` or null);
  - `ViewerState.PlanIsStale(double newestSimT)`, meaning older than 1 s;
  - `ViewerState.HasMediumPlans` and `ViewerState.HasLidar`, from the Hello's `config.medium` and `config.lidar`;
  - `ViewerState.TryPoseAt(double simT, out (double x, double y, double yaw) pose)`, interpolated over the State history.
    The history grows from 64 to 256 States.
  - `PathHistory(int cap = 5000, double minSpacingM = 0.02)`, with `Add(x, y)`, `Clear()` and `Points`.
- Consumes: `ViewerState.Apply`, `GoalMarkerModel` (4A).

- [ ] **Step 1: Write the failing tests:**
  - **`PlansFollowTheActiveGoalAndEpoch`:**
    - a Plans frame for the active goal is kept;
    - one from an older epoch, or for another `goal_id`, is ignored;
    - a Goal frame ending the goal, or a new session, clears `Plan`;
    - a recorded-route session, which has no `goal_id`, keeps every plan.
  - **`StalePlanAfterOneSecond`:** stale at newest State t greater than `ReceivedAtSimT` + 1.0.
  - **`AvailabilityFollowsTheSession`:** `config.medium` false means no plans, and `config.lidar` false means no lidar.
  - **`PoseAtInterpolatesTheStateHistory`:**
    - a time between two States gives the linear pose, with yaw taking the short way;
    - a time outside the history returns false.
  - **`TravelledPathIsBoundedAndSpaced`:** points under 0.02 m apart are merged; 6,000 adds keep 5,000; `Clear` empties.
- [ ] **Step 2: Run the tests.** Expected: fail to compile.
- [ ] **Step 3: Implement.** Plans decoding follows stream-protocol.md, "Plans".
- [ ] **Step 4: Run the viewer tests, then the whole EditMode suite.** Expected: pass.
- [ ] **Step 5: Commit** `Spec 4B-1 Task 1: ...`. Write the review prompt, then stop.

### Task 2: the panel sweep and lidar placement (Core)

**Files:** create `Core/PanelSweep.cs`, `Core/LidarCloud.cs`; tests `Viewer/PanelSweepTests.cs`,
`Viewer/LidarCloudTests.cs`.

**Interfaces:**
- **`static class PanelSweep`:** `IReadOnlyList<Vector3[]> Rings(IReadOnlyList<double[]> samples,
  Func<double[], Vector3[]> panelCorners, double maxCornerStepM = 0.05, int maxRings = 2000)`.
  - It uses `System.Numerics.Vector3` in map coordinates (ROS).
  - The base x and y and the joints are interpolated linearly, and the yaw along the shortest angle.
  - The number of substeps between two samples is the ceiling of (the largest corner displacement between their end
    poses ÷ `maxCornerStepM`).
- **`sealed class LidarCloud`:**
  - `LidarCloud(double keepS = 0.5, int maxPoints = 20000)`.
  - `bool AddScan(double tStart, double tEnd, float[] xyz, float[] hitClass, Func<double, (double x, double y, double
    yaw)?> baseAt, Func<Vector3, Vector3> livoxToBase)`. Point i of n takes the time `tStart + (i / n)(tEnd - tStart)`.
    It returns false, and counts the scan in `Skipped`, when any point's time has no pose.
  - `Points`, as (map position, hit class); `bool ShowGround`; `int Skipped`.
  - It drops scans older than `keepS` before the newest. Past `maxPoints`, it thins the oldest scan with a stride.

- [ ] **Step 1: Write the failing tests:**
  - **`SweepInterpolatesUntilCornersMoveAtMostFiveCentimetres`:** with a fake corner function (a 1.2 m square rotated
    by the first joint), two samples 0.5 rad apart give rings whose consecutive corners are at most 0.05 m apart.
  - **`SweepJoinsAStillBaseReconfigurationDirectly`:** two samples with equal base and different joints give rings that
    interpolate the joints (the known gap: no samples in between).
  - **`SweepIsCappedAtMaxRings`.**
  - **`LidarPointTimesAreSpreadOverTheScan`:** 5 points over [1.0, 1.1] get times 1.0, 1.02, 1.04, 1.06 and 1.08.
  - **`LidarPlacementUsesTheBasePoseAtEachPointsTime`:** with a base moving 1 m/s along x and the mount at (0.3, 0,
    0.5), the first and last points land 0.08 m apart in x.
  - **`AScanWithoutPoseHistoryIsSkipped`:** `baseAt` returns null; `AddScan` is false, `Skipped` is 1, and no points
    are added.
  - **`CloudKeepsHalfASecondAndAtMostTwentyThousandPoints`.**
  - **`GroundHitsAreHiddenByDefault`.**
- [ ] **Step 2: Run the tests.** Expected: fail to compile.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run the viewer tests, then the whole suite.** Expected: pass.
- [ ] **Step 5: Commit** `Spec 4B-1 Task 2: ...`. Write the review prompt, then stop.

### Task 3: console store, overlay settings and options (Core)

**Files:** create `Core/ConsoleLog.cs`, `Core/OverlaySettings.cs`; modify `Core/ViewerOptions.cs`; tests
`Viewer/ConsoleLogTests.cs`, `Viewer/OverlaySettingsTests.cs`, `Viewer/ViewerOptionsTests.cs`.

**Interfaces:**
- **`ConsoleLog`:**
  - Entries: `enum Severity { Info, Warning, Error }`, and `sealed class ConsoleEntry { Severity; string Subsystem;
    double? SimT; double Wall; string Text; int Count; }`.
  - Construction and input: `ConsoleLog(int cap = 2000, int perFrame = 200)`, `BeginFrame()`, and `bool Add(Severity,
    string subsystem, double? simT, double wall, string text)`. `Add` collapses a repeat of the previous entry (same
    severity, subsystem and text) into its `Count`.
  - Bounds: `Dropped`.
  - Output: `IEnumerable<ConsoleEntry> Filter(Severity minimum, ISet<string> subsystems, string search)` and
    `Export(TextWriter, IEnumerable<ConsoleEntry>)`.
  - Event mapping: `static (Severity, string subsystem, string text) FromEvent(JObject e)`:
    - `goal_*` events go to `goal`. Rejected, failed and timeout are warnings.
    - `route_*` go to `route`.
    - `fallback` goes to `medium` as a warning; `contact` to `plant` as an error.
    - `behind` and `dropped` go to `stream` as warnings, and `error` is an error.
    - `run_stopped` goes to `plant`: a warning with reason `error`, info otherwise.
    - Anything else is info, with subsystem `plant`.
- **`OverlaySettings`:**
  - Types: `enum Overlay { Route, Travelled, PlanPath, Ghosts, Sweep, Lidar }`, `enum Panel { Hud, Minimap,
    Telemetry, Console, Legend }` and `enum Preset { Off, Demo, Diagnose }`.
  - Methods: `bool this[Overlay]`, `bool this[Panel]`, `Preset Current`, `bool Modified`, `JObject Apply(Preset)`, and
    `JObject Toggle(Overlay)`, `JObject Toggle(Panel)`. The `JObject` they return is the journal fields.
- **`ViewerOptions`:** `bool VsyncLock30` (true for `-fps 30`) and `Preset StartPreset` (`-preset off|demo|diagnose`,
  default off).

- [ ] **Step 1: Write the failing tests:**
  - **Console:**
    - `ConsoleCollapsesRepeats`: three identical errors give one entry with Count 3.
    - `ConsoleIsBoundedAndCountsDrops`: 2,500 distinct entries keep 2,000, with `Dropped` 500.
    - `ConsoleKeepsItsFrameBudget`: the 201st `Add` in a frame is false and counted; `BeginFrame` resets the budget.
    - `ConsoleFiltersBySeverityAndSubsystemAndSearch`.
    - `ConsoleExportsVisibleEntries`.
    - `EventsMapToSeverityAndSubsystem` (the table above).
  - **Overlay settings:**
    - `PresetsSetTheDesignedToggles`: Off, Demo and Diagnose as in Global Constraints. Off keeps the route on and leaves the minimap unchanged.
      The journal fields name the preset and every toggle.
    - `AToggleAfterAPresetMarksItModified`.
  - **Options:**
    - `Fps30MeansTheVsyncLock`: `-fps 30` gives `VsyncLock30`; `-fps 45` does not.
    - `PresetOptionParses`, including a bad value naming `-preset`.
- [ ] **Step 2: Run the tests.** Expected: fail to compile.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run the viewer tests, then the whole suite.** Expected: pass.
- [ ] **Step 5: Commit** `Spec 4B-1 Task 3: ...`. Write the review prompt, then stop.

### Task 4: overlay presenters, ghosts and the legend (Unity)

**Files:** create in `Unity/` the presenters for the travelled path, the plan path, ghosts, the sweep, lidar and the
legend (names left to the builder; one MonoBehaviour per file). Modify `PlantRoutePresenter.cs` (toggle),
`PlantMinimap.cs` (the plan path when overlay 3 is on) and `Editor/PlantViewerSetup.cs` (adds and wires them; creates
`ViewerGhost.mat`, translucent HDRP). Tests `Viewer/ViewerSceneContractTests.cs` and `Viewer/PlantRobotViewTests.cs`.

**Interfaces:**
- Consumes Tasks 1-3.
- **Panel corners:** a hidden probe copy of the view robot, posed per sample. It gives the panel's four corners for
  `PanelSweep.Rings` and the ghost transforms.
- **Mount:** the `livox_frame` transform relative to `base_link`, read from the view robot and converted to the ROS
  base frame. This gives `livoxToBase`.
- **Rebuilds:** ghosts and the sweep rebuild only when `ViewerState.Plan` changes (plan id). The lidar mesh updates
  when a scan is added.
- **Hidden overlays** do no drawing or rebuild work; their data stay stored, and they rebuild once when toggled on.

- [ ] **Step 1: Write the failing tests:**
  - `PanelCornersFollowThePlannedJoints`: the probe's corners at a pose equal the corners derived from 4A's independent
    URDF forward kinematics of `tool0` plus the panel's half sizes, within 1 mm.
  - `ViewerSceneContractTests`: the new presenters are wired, and the contract still forbids ROS, physics and sensors.
  - `GhostsAreCappedAtTwelve`.
- [ ] **Step 2: Run the tests.** Expected: fail.
- [ ] **Step 3: Implement.**
  - Rebuild the scene with `unity command build_plant_viewer`.
  - The legend lists each overlay's key, colour and state, with "no data" or "approximate" where they apply.
- [ ] **Step 4: Run the whole suite.** Expected: pass.
- [ ] **Step 5: Live check in Editor Play** against `plant_demo_server --mode realistic --routes lattice --lidar`, on a door crossing (`v2_open_yard_nav` toward room R4 or the store). Check each overlay, the sweep turning the panel just before the frame, a reset clearing the plan overlays, and lidar on and off.
- [ ] **Step 6:** Stop Play. Commit `Spec 4B-1 Task 4: ...`. Record in the README. Write the review prompt, then stop.

### Task 5: telemetry, console, keys, presets and pacing (Unity)

**Files:** create `Unity/PlantTelemetryPanel.cs`, `Unity/PlantConsolePanel.cs`; modify
`Unity/{PlantViewerBootstrap,PlantViewerInput,PlantViewerHud}.cs` and `Editor/PlantViewerSetup.cs`; tests
`Viewer/PlantViewerInputTests.cs`, `Viewer/ViewerSceneContractTests.cs`.

**Interfaces:**
- **New `ViewerAction` values:** Overlay1 to Overlay6, NextPreset, Telemetry, Console, Legend, Hud and Pace.
- **Telemetry:** reads `ViewerState` (State header fields, `Plan`, goal, stream). Values not yet received show as "—".
- **Console:** fed by `ConsoleLog.FromEvent`, by the viewer's own messages and by `Application.logMessageReceived`
  (warnings and errors only, with a re-entrancy guard). Search is a text field; copy goes to
  `GUIUtility.systemCopyBuffer`; export writes to the session's journal folder.
- **Pace:** K, and `-fps 30` at start, switch `vSyncCount` between 1 and 2. `-preset` applies at start.

- [ ] **Step 1: Write the failing tests:**
  - `NewKeysMapToTheirActions`.
  - `NothingFiresWhileTheConsoleSearchHasFocus`.
  - `BootstrapIsWired`, extended to the panels.
- [ ] **Step 2: Run the tests.** Expected: fail.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run the whole suite.** Expected: pass.
- [ ] **Step 5: Live check in Editor Play:**
  - the panels with a lattice server;
  - a contact burst and a Unity error collapsing in the console;
  - search, copy and export;
  - presets and pace switching;
  - journal lines.
- [ ] **Step 6:** Stop Play. Commit `Spec 4B-1 Task 5: ...`. Write the review prompt, then stop.

### Task 6: player, pacing measurement and records

**Files:** `docs/experiments/plant-viewer/README.md`, `docs/running-the-stack.md` (keys and options),
`docs/roadmap.md` (4B-1 outcome).

- [ ] **Step 1: Rebuild the player** with `unity build motion-planning-sim --target StandaloneOSX --execute-method
  MotionPlanningSim.Editor.PlantViewerBuild.Build`, then revert the three rewritten settings files. Expected: exit 0.
- [ ] **Step 2: Pacing measurement:**
  - **Setup:** windowed 1920 x 1080, Balanced, recorder on, scripted demo to (18, 0, 0) with lidar, 60 s after 10 s of
    warm-up, thermal state before and after.
  - **Runs:** Off, Demo and Diagnose, each at 60 and at the 30 lock (`-preset`, `-fps 30`), plus each overlay alone at
    60.
  - **Expected:** Demo stable (99% of frames within ±20% of the period) at 60 or 30; Off within the noise of 4A's
    Balanced result (p99 30.0 ms on this route).
  - **If Demo misses:** optimize in the design's order (density, simpler ghosts, HDRP settings), re-measuring each
    change.
- [ ] **Step 3: Record** the table, the optimizations and the overlay costs in the README. Update the running-the-stack
  keys and the roadmap outcome.
- [ ] **Step 4: Commit** `Spec 4B-1 Task 6: ...`. Write the review prompt. Then the user's hand check, and the final
  whole-branch review.
