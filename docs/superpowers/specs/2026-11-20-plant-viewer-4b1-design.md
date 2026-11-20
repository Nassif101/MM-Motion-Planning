# Plant viewer stage 4B-1: inspection from existing data (design)

Date: 2026-11-20. Status: draft for the user's approval (builder check done, 2026-11-20). Parent design:
[plant viewer design](2026-11-20-plant-unity-viewer-design.md) (section 9 stages 4B as "designed in detail when it
starts"). Stage 4A is complete ([records](../../experiments/plant-viewer/README.md)).

## 1. Intent

4B adds inspection to the plant viewer. The user's purposes are two, and only these two (user, 2026-11-20):
- **Demo:** videos that show the system: what the robot will do, and where the panel goes.
- **Diagnosis:** answering questions about behaviour, the first being the user's own observation that F2 turns the
  panel late, just before a door frame, although it plans 8 s ahead.

4B is split in two (user, 2026-11-20: approach B below). **4B-1, this design**, draws only data the backend already
streams, and changes neither the backend nor the protocol. **4B-2** adds new backend data. It is optional: the user
decides after 4B-1 whether it is needed, deferred or dropped.

## 2. Goals, non-goals, anti-goals

**Goals:**
- Show what F2 plans: its path, ghosted arm-and-panel poses along it, and the panel's planned sweep.
- Show what happened: the travelled path.
- Show why: a telemetry panel (B5, F2, goal and stream state) and a diagnostic console.
- Show what the robot senses: lidar points, placed approximately and coloured by hit class.
- Every overlay can be toggled on its own, and presets switch between a demo picture and a diagnosis picture. 4A's
  status line and minimap become togglable too.
- Steady frame pacing with any preset; a high frame rate where it can be held (section 7.5).

**Non-goals (4B-1):**
- No new backend data, no schema change. The schema stays 3, and the campaigns and their logs are untouched.
- B5's prediction, exact lidar placement, active safety margins, losing seeds' paths and plan time stamps belong to 4B-2.
- No collision-geometry inspection, and no swept volume presented as a guarantee.
- No replay of run logs (4C).

**Anti-goals:**
- A sweep or ghost that looks like a collision guarantee. The sweep is labelled "planned sweep (approximate)".
- Diagnostic clutter in demo videos that cannot be switched off.
- Uneven frame pacing (the 60/30 alternation measured in 4A) in a demo take.

## 3. Constraints

- Everything from 4A holds: Unity 6000.5.2f1, HDRP, Balanced quality default, the viewer scene without ROS, physics,
  sensors or controllers, Unity only through the CLI.
- No new packages. The overlays use IMGUI, as the HUD and minimap do, and runtime meshes and LineRenderers with the
  existing `ViewerOverlay.mat`.
- The pure logic lives in `MotionPlanningSim.Viewer.Core` and is tested in EditMode, as in 4A.
- Bounded memory: every history, buffer and pool has a fixed cap.
- The view journal records overlay toggles and preset changes (design 6.6), so 4C can re-render a take with the same
  overlays.

## 4. Approaches (decided)

- **A. One plan for all of 4B.** Backend and overlays together; long, with the protocol change mid-plan.
- **B. Two stages: viewer first, then backend.** 4B-1 on existing data, 4B-2 the schema-4 additions. **Chosen by the
  user**, because 4B-1 may prove sufficient and 4B-2 can then be deferred or dropped.
- **C. Backend first.** All schema-4 data before any overlay; nothing visible until late.

## 5. What the user decided

- Purpose: demo and diagnosis only.
- Every overlay is togglable, so an overlay that disturbs a take is switched off rather than designed away.
- Priority: 1. what the robot will do (F2 ghosts, the panel sweep, the paths); 2. why (telemetry, console); 3. what
  it senses (lidar); 4. deeper diagnosis (4B-2).
- The panel sweep is in priority 1, as F2's planned sweep, approximate, panel only, and not a collision envelope.
- Approach B (section 4).
- 4A's status line gets its own toggle; the presets set it and the minimap (user, 2026-11-20).
- Frame-rate stability matters more than its value. Optimize as far as reasonable, and accept that some overlays cost
  frames (user, 2026-11-20). Section 7.5 turns this into a pacing rule.

## 6. The data used (all in schema 3)

- **Plans frame** (F2, about 2.5 Hz):
  - header: `plan_id`, `route_id`, `status` (`ok`, `infeasible`, `stale_reused`), `homotopy`, `capped`, `seed`,
    `full_multistart`, `waiting`, `homotopy_switched`, `seeds` (`[name, merit, feasible, progress_m, merged]`), and
    `epoch`, `goal_id` and `request_id` in generated-route sessions;
  - block 0: samples of `[x, y, yaw, q0..q5]`. A sample is emitted every 0.1 m of arc length or 0.1 rad of yaw. The
    stream carries no sample times.
- **State frame** (50 Hz): `status`, `fallback`, `gate` `[slack, violation, unsafe]`, `dwell`, `guard` (when on),
  `contacts`; block 0 the true pose and joints.
- **Scan frame** (only when the server runs with `--lidar`): header `t_start`, `t_end`, `rays`; block 0 points in
  `livox_frame` at each ray's time; block 1 the hit class per point. The plant emits points in ray-time order; the
  per-point time offset exists in the plant but is not streamed.
- **Event, Goal and Route frames:** as in 4A.

## 7. The 4B-1 design

### 7.1 Overlays

Each overlay has its own toggle. When its data are missing (a recorded-route server, no `--lidar`, B5 alone), the
legend says "no data" instead of drawing nothing.

1. **Route.** The 4A ground line, now behind a toggle.
2. **Travelled path.** The displayed robot base, as a ground line. Bounded to the last 5,000 poses (call). Cleared on a
   new session.
3. **F2 plan path.** The plan's base path as a ground line, coloured by plan status: ok, infeasible, `stale_reused`.
4. **F2 ghosts.** Translucent copies of the arm and panel, with a flat base outline, at evenly spaced plan samples:
   - 6 ghosts by default (call; adjustable, at most 12). They fade with distance along the plan.
   - Keyed to the plan's `plan_id`, `route_id` and `epoch`, so a new plan replaces them, and the end of a goal or a
     reset removes them.
   - A plan more than 1 s older than the newest State (call) is drawn dimmed, as stale.
5. **Panel sweep, planned (approximate).** A translucent band through the panel's corners along the plan.
   - The panel pose at each sample is computed with the view robot's joint transforms.
   - Between samples, the viewer interpolates the joints linearly until each corner moves at most 0.05 m (call).
   - Labelled "planned sweep (approximate)" in the legend.
   - Known gap, stated in the legend's tooltip and in the README: an arm reconfiguration while the base stands still
     has no samples between its ends, so the band joins the end poses directly. Plan time stamps (4B-2) close this.
6. **Lidar points.** The latest scans, coloured by hit class (site, scenario, mover, ground, robot, panel, other):
   - Placement: point i of n takes the time `t_start + (i / n)(t_end - t_start)`, the base pose interpolated from the
     State history at that time, and `livox_frame` from the view robot. Labelled "approximate placement"; exact
     placement is 4B-2.
   - Density: a display decimation (call: at most 20,000 points on screen) and a persistence of the last 0.5 s (call).
     Ground hits are hidden by default.
   - Pooled buffers. A scan the viewer cannot place, because no State covers its times, is skipped and counted.
7. **Goal marker and the minimap.** As in 4A. The minimap also shows the F2 plan path when overlay 3 is on.

### 7.2 Telemetry panel

An IMGUI panel, toggled with a key. Values not received show as "—":
- **B5:** status, holding or dwell, `gate` slack, violation and unsafe, `guard` when present, current contacts.
- **F2:** `plan_id`, status, homotopy, winning seed, capped, waiting, full multistart, homotopy switched, the plan's
  age; a seeds table: name, merit, feasible, progress, merged.
- **Goal:** id, marker state, reason, `route_id`.
- **Stream and viewer:** connection, achieved speed, stream age, dropped and malformed counts, FPS and frame time.
  These are viewer measurements, not planner metrics, and are labelled that way.
- No solve timings: the stream leaves wall-clock times out on purpose (same-seed logs stay identical, E2). A live-only
  timing field is a 4B-2 candidate.

### 7.3 Diagnostic console

An IMGUI panel, toggled with a key:
- **Sources:**
  - backend Events (all kinds, with simulation time);
  - viewer messages (connection, journal, map, options, the status messages of 4A, with wall time);
  - Unity warnings and errors from `Application.logMessageReceived`, with a re-entrancy guard so the console never
    logs itself.
- **Each entry:** severity (info, warning, error), subsystem (goal, route, medium, b5, plant, stream, viewer), the
  times, and the text.
- **Controls:** filters by severity and subsystem, a text search, pause autoscroll, copy the visible entries to the
  clipboard, export them to a text file in the session's journal folder.
- **Bounds:**
  - at most 2,000 entries (call), and the oldest are dropped and counted;
  - a burst of identical messages collapses into one entry with a count;
  - at most 200 new entries per frame are processed (call), and the rest are dropped and counted.

### 7.4 Presets, keys and the legend

- **Presets**, switched with one key (P, call):
  - **Off:** the 4A picture: status line on, minimap as it was, no 4B overlays.
  - **Demo:** route, travelled path, F2 ghosts and panel sweep; status line, minimap, telemetry, console and lidar off.
  - **Diagnose:** every overlay, the status line, telemetry and console.

  A preset sets the toggles; changing a toggle afterwards keeps the preset's name, marked as modified.
- **Keys (calls; none taken by 4A or the free camera):**
  - 1 to 6 toggle overlays 1 to 6 of section 7.1;
  - K switches the pace between 60 and the 30 lock (section 7.5);
  - H toggles 4A's status line (and its warning and message lines); M still toggles the minimap;
  - T toggles the telemetry panel;
  - backquote toggles the console;
  - L toggles the legend.

  As in 4A, nothing fires while a text field (the console search) has focus.
- **The legend:** a small panel listing each overlay, its key, its colour, its state, and "no data" or "approximate"
  where those apply.
- **Journal:** each toggle, preset change and console export is journaled with the displayed simulation time (thinnable:
  no).

### 7.5 Performance: steady pacing first

The user's rule (2026-11-20): a steady frame rate matters more than a high one, and some overlays may cost frames.

- **Frame pacing.** The viewer runs either at 60 FPS (vsync on every refresh) or locked at 30 FPS (vsync every second
  refresh, `QualitySettings.vSyncCount = 2`). The 4A `-fps 30` used `targetFrameRate` without vsync and measured a p99
  of 41 ms, so 4B-1 changes `-fps 30` to the vsync lock and adds a key for it (call: K; F stays the camera's recenter). Other `-fps` values keep `targetFrameRate`.
- **Stability measure** (frame times from `-frame-times`, 60 s after 10 s of warm-up): the share of frames within
  ±20% of the paced period (16.7 or 33.3 ms), and the p99 frame time.
  - **Stable:** at least 99% of frames within ±20%.
  - The 4A target (p99 ≤ 33.3 ms at 60 FPS) stays the reference for the 4A picture.
- **Gate.** The Demo preset must be stable at 60 or at the 30 lock. Diagnose is measured at both paces and recorded; a
  miss does not block 4B-1, but its cost per overlay is recorded so the user knows which toggle to turn off.
- **Conditions:** 1920 x 1080, windowed, Balanced, the screen recorder on, the scripted demo with lidar on, on the 4A
  measurement route (18, 0, 0), with the thermal state recorded.
- **Optimization, in this order** (call), before accepting a cost:
  1. the design rules below;
  2. overlay density (lidar points, ghost count, sweep resolution);
  3. simpler ghosts (opaque or outline instead of translucent HDRP materials);
  4. the Balanced level's HDRP settings (shadows, volumetrics), as the parent design allows.

  Each change and its measured effect is recorded in the README.
- **Design rules:**
  - Ghosts and sweep meshes are rebuilt only when a new plan arrives (about 2.5 Hz), never per frame.
  - Ghost instances are pooled.
  - Lidar uses one reusable point mesh.
  - Console and telemetry strings are rebuilt only while their panel is open.

## 8. 4B-2, for the record (not designed; the user decides after 4B-1)

Schema 4 additions:
- B5's prediction as a new frame, with its ghosts;
- the per-point time offsets in the Scan frame, for exact lidar placement;
- active safety margins per pair near the robot;
- the losing seeds' paths;
- sample times (and arc length, speed) in the Plans frame, which close the sweep's gap;
- optionally, a live-only solve-timing field.

Each needs backend tests and a check that campaign outputs stay identical.

## 9. Testing

- **EditMode, Core (pure logic):**
  - plan decoding and keying: a new plan replaces the old, and a route or goal end or reset clears it;
  - stale-plan dimming;
  - the sweep construction: corners at the samples, interpolation until each corner moves at most 0.05 m, the gap case;
  - lidar time and pose assignment, against hand-computed poses, and skipping a scan with no State history;
  - the console store: bounds, collapse of repeats, filters, search, the per-frame limit;
  - presets and toggles, and their journal lines;
  - travelled-path bounds.
- **EditMode, scene:** the contract still forbids ROS, physics and sensor components, and the new presenters are wired.
- **Live check** (Editor Play, then the player):
  - each overlay against a lattice server with lidar;
  - presets;
  - the console with a burst of errors;
  - the late-turn scene: F2's ghosts and sweep visibly turning the panel just before a door.
- **Pacing measurement** as in section 7.5, per preset (Off, Demo, Diagnose) and pace; the cost of each overlay
  alone at 60 FPS; recorded in the README.
- **The user's hand check** in the rebuilt player.

## 10. Process

- **Branch:** `plant-viewer-4b1`, made from `plant-routing-unity-viewer` (pushed, holding spec 2 and 4A, not merged),
  so that branch can be merged on its own (call).
- **Execution:** as in 4A, compact tasks. After each task: commit, a review prompt in `docs/reviews/plant-viewer/`,
  and a stop. A final whole-branch review at the end.

## 11. Left to the builder

- Colours, line widths, translucency, fonts and panel layout.
- How the ghost instances are made from the view robot: a pooled copy of the arm subtree with a translucent material,
  or another way.
- The point-mesh technique for lidar.
- The exact console text format.
- The values marked "call" above, if measurement shows a better one; each change is recorded.
