# Plant viewer stage 4B-2 (B5's prediction, plan and lidar times) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** the viewer shows B5's solved 2 s plan beside F2's, with held plans marked. F2's ghosts and sweep follow
real plan times, and lidar points use their own times.

**Architecture:** the backend adds a Prediction frame, a Plans display block and a Scan point-time block, advertised
by capabilities, without changing what the planners compute. The viewer core decodes them, and Unity presenters draw
them with 4B-1's fallbacks kept.

**Tech Stack:** C++17, gtest and colcon in the container; Unity 6000.5.2f1, HDRP, NUnit EditMode, the Unity CLI.

**Spec:** [2026-11-21-plant-viewer-4b2-design.md](../specs/2026-11-21-plant-viewer-4b2-design.md); parent
[4B-1 design](../specs/2026-11-20-plant-viewer-4b1-design.md).

## Global Constraints

- Schema stays 3; new data is announced by Hello capabilities: `b5_prediction`, `plan_display_samples`,
  `scan_point_times`.
- **Compatibility.** Both existing readers reject unknown frame types: the C# `FrameCodec` and Python `plant_stream`'s
  `KINDS`. Therefore:
  - The demo server sends Prediction frames only to a client whose hello lists `accepts: ["prediction"]`. The 4B-2
    viewer sends it; the 4A/4B-1 viewer does not, so it never receives them.
  - Run logs carry them only with `plant_campaign --prediction`.
  - The new blocks (Plans block 1, Scan block 2) are extra blocks, which old readers ignore.
- **Prediction frame:**
  - type 13;
  - header `t`, `status`, `stage_dt`, `stages`, `plan_id`;
  - block 0: (stages + 1) x 9 floats (x, y, yaw, q0..q5) from `Controller`'s solved plan;
  - every 5th B5 cycle (10 Hz).
- **Plans block 1:** display samples every 0.1 s from 0 to the horizon, `[t, x, y, yaw, q0..q5]` (10 floats each),
  computed after the solve (outside any timed or budgeted section).
- **Scan block 2:** each point's time offset from `t_start` (the plant's `dt`), one float per point.
- **Campaign identity.** The planners' decisions do not change. A session with the prediction on and one with it off
  give identical State streams and outcomes, and the existing suites pass unchanged.
- **Viewer:**
  - overlay 7 (key 7) is B5's prediction: path plus 4 ghosts at 0.5, 1, 1.5 and 2 s; a warning colour and "held" when
    `status` says the plan is not executed; stale after 1 s;
  - Diagnose includes overlay 7, Demo does not, and `-overlays` accepts 1-7;
  - F2's ghosts and sweep use the display samples when present;
  - lidar uses point times when present;
  - 4B-1's approximations stay as fallbacks.
- **Process:** branch `plant-viewer-4b2`. After each task: commit, a review prompt, a subagent review, the fixes. A
  final whole-branch review at the end. Build in the shared `ros2_ws/install`. Never overlap Unity with timed runs.

## Review Focus

1. **An old viewer against the new server:** no Prediction frame reaches it, and nothing new is counted malformed.
   Test: Task 1 `PredictionOnlyForClientsThatAcceptIt`.
2. **The prediction on or off changes nothing in the run.** Test: Task 1 `PredictionDoesNotChangeTheRun`.
3. **An arm turn with the base still is drawn as it moves** (the 4B-1 gap). Test: Task 2 `SweepUsesDisplaySamples`.
4. **A held B5 plan is marked as held, not shown as executed.** Test: Task 2 `HeldPredictionIsMarked`; live check in
   Task 3.
5. **A server without the capabilities:** 4B-1's behaviour, with no stale or "no data" mix-ups. Test: Task 2
   `FallbacksWithoutTheCapabilities`.

---

### Task 1: backend frames and blocks (C++)

**Files:**
- `mobile_manipulator_mpc/include/.../controller.hpp` (a read-only accessor to the solved plan);
- `mobile_manipulator_medium/{include/.../planner.hpp,src/planner.cpp}` (`PlanOutput::display`, filled after the
  solve);
- `mobile_manipulator_campaign/{include,src}/{session,demo_server}` (`SessionConfig::prediction`, the Prediction frame
  every 5th cycle, Plans block 1, Scan block 2, capabilities, the hello's `accepts`, `--prediction`);
- `mobile_manipulator_plant/include/.../frames.hpp` (`FrameType::Prediction = 13`);
- `tools/plant/plant_stream.py` (`PREDICTION`, `KINDS`);
- `docs/experiments/mujoco-plant/stream-protocol.md`;
- tests in `test_session.cpp`, `test_demo_server.cpp` and `test_closed_loop.cpp` (or `test_planner.cpp`).

- [ ] **Step 1: Write the failing tests:**
  - `PredictionFrameLayoutAndRate`: 21 x 9 floats, `stage_dt` 0.1, one frame per 5 cycles.
  - `PredictionDoesNotChangeTheRun`: the same seed with and without the prediction gives byte-identical State frames
    and the same outcome.
  - `DisplaySamplesMatchThePlanSamples`: each display sample is 0.1 s apart and spans the horizon; at an existing
    sample's arc length its pose matches within 1e-6.
  - `ScanCarriesPointTimes`: block 2 equals the plant's `dt`, one per point, increasing.
  - `PredictionOnlyForClientsThatAcceptIt`: the demo server sends no type-13 frame to a plain hello, and does to
    `accepts: ["prediction"]`; capabilities are listed.
- [ ] **Step 2:** Build and run them in the container. Expected: they fail (missing symbols, or no frames).
- [ ] **Step 3: Implement.**
- [ ] **Step 4:** Run the new tests and the full `mobile_manipulator_campaign`, `_medium` and `_mpc` suites. Expected:
  all pass.
- [ ] **Step 5:** Commit `Spec 4B-2 Task 1: ...`, write the review prompt, run the subagent review and fix.

### Task 2: viewer core (C#)

**Files:**
- `Core/{FrameCodec,ViewerState,MediumPlanStore,LidarCloud,OverlaySettings,ViewerOptions,ViewerCommands}.cs`
- a new `Core/B5Prediction.cs`
- tests in `Viewer/{B5PredictionTests,MediumPlanStoreTests,LidarCloudTests,OverlaySettingsTests,ViewerOptionsTests}.cs`

**Interfaces:**
- `FrameType.Prediction = 13`.
- `ViewerCommands.Hello()` adds `accepts: ["prediction"]`.
- `sealed class B5Prediction { double T; string Status; bool Held; double StageDt; IReadOnlyList<double[]> States; }`
- `ViewerState.Prediction`, `ViewerState.HasPrediction` (the capability), `ViewerState.PredictionIsStale(newestT)`.
- `MediumPlan.DisplaySamples` (10 floats each, or empty).
- `LidarCloud.AddScan(..., float[] pointTimes = null)`.
- `Overlay.Prediction` (the seventh).

- [ ] **Step 1: Write the failing tests:**
  - `PredictionDecodesAndClearsWithTheSession`
  - `HeldPredictionIsMarked`: B5's holding statuses set `Held`.
  - `PredictionIsStaleAfterOneSecond`
  - `DisplaySamplesDecode`
  - `SweepUsesDisplaySamples`: an arm turn at rest gives rings along the turn.
  - `LidarUsesPointTimes`: placement uses `t_start + dt[i]`.
  - `FallbacksWithoutTheCapabilities`
  - `PresetsIncludePredictionInDiagnoseOnly`
  - `OverlaysOptionAcceptsSeven`
  - `HelloAcceptsPredictions`
- [ ] **Step 2: Run.** Expected: fail to compile.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** the whole EditMode suite. Expected: pass.
- [ ] **Step 5:** Commit, review prompt, subagent review, fixes.

### Task 3: Unity overlays and live check

**Files:**
- a new `Unity/PlantPredictionOverlay.cs` (path plus 4 pooled ghosts; the warning colour when held);
- `Unity/{PlantPlanOverlay,PlantLidarOverlay,PlantViewerBootstrap,PlantViewerInput}.cs`;
- `Editor/PlantViewerSetup.cs`;
- tests `Viewer/{PlantViewerInputTests,ViewerSceneContractTests,PlantOverlayTests}.cs`.

The changes:
- F2's ghosts move to the display samples' times.
- Lidar uses point times.
- Key 7, the legend rows ("held", "exact"), and telemetry lines for B5's horizon end and the held state.

- [ ] **Step 1: Write the failing tests:** `Key7TogglesThePrediction`, `BootstrapIsWired` (the prediction overlay), and
  `PredictionGhostsAtHalfSecondSteps`.
- [ ] **Step 2: Run.** Expected: fail.
- [ ] **Step 3: Implement**, then rebuild the scene with `build_plant_viewer`.
- [ ] **Step 4: Run** the whole suite. Expected: pass.
- [ ] **Step 5: Live check in Editor Play** against the rebuilt shared-install server, on `v2_door_105_nav`:
  - B5's prediction beside F2's plan through the door, and a held period if one occurs;
  - the sweep of an arm turn at rest;
  - exact lidar.

  Stop Play and record in the README.
- [ ] **Step 6:** Commit, review prompt, subagent review, fixes.

### Task 4: player, pacing and records

- [ ] **Step 1:** Rebuild the player in batch and revert the three rewritten settings files.
- [ ] **Step 2: Pacing**, as in 4B-1 Task 6: Diagnose (now with overlay 7) at 60, and overlay 7 alone at 60. Expected:
  steady (99% within ±20%). If not, lower the prediction ghost count or rate first.
- [ ] **Step 3: Records:**
  - the README (results; the 4B-1 known limitations that 4B-2 removes);
  - `running-the-stack.md` (key 7, `--prediction`);
  - the roadmap outcome;
  - a reminder that safety margins and losing candidates are deferred to before Phase 6.
- [ ] **Step 4:** Commit; the final whole-branch review and its fixes; the user's hand check; push and merge on the
  user's word.
