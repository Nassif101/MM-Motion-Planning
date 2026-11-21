# Plant viewer stage 4B-2: B5's prediction, exact plan and lidar times (short design)

Date: 2026-11-21. Status: approved by the user (2026-11-21). Parent:
[4B-1 design](2026-11-20-plant-viewer-4b1-design.md), section 8. Scope decided by the user (2026-11-21).

## Intent

Diagnosis the 4B-1 viewer cannot do yet. The first case is the user's: at a door, F2's ghosts showed a correct
vertical swing, but the robot stalled without executing it. Seeing B5's own prediction next to F2's plan shows which
layer refused.

The same stage removes 4B-1's two approximations:
- the panel sweep joined an arm turn made with the base standing still straight across;
- lidar point times were guessed from their order in the scan.

## Scope (user, 2026-11-21)

- **B5's prediction:** a new overlay (key 7), with the path and ghosts of B5's solved 2 s plan.
- **F2's plan in time:** the sweep and the F2 ghosts use times, closing the arm-turn gap.
- **Exact lidar placement:** each point's own time.
- **Deferred, to raise before Phase 6:** active safety margins, and F2's losing candidates' paths.

## Non-goals and constraints

- No change to what the planners compute: campaign decisions and summaries must stay identical.
- New data is added beside the existing data: old readers and old logs keep working.
- B5's prediction is opt-in on `plant_campaign`, so campaign logs do not grow; it is on in `plant_demo_server`.

## Approaches (calls, for the user to check)

1. **Protocol versioning.** A schema bump to 4, against keeping schema 3 with new capabilities.
   - **Call: schema 3 with capabilities.** The protocol already says readers key on Hello `capabilities`, not on the
     schema number.
   - Every addition is a new block or a new frame type. The 4A/4B-1 viewer refuses any schema but 3, so a bump would
     break it.
2. **F2's plan times.** Adding a time to each existing plan sample, against a separate display block.
   - Adding times alone does not close the gap: F2 emits samples only per 0.1 m or 0.1 rad of base motion.
   - Emitting samples on arm motion too would change the reference B5 follows, and with it the campaign decisions.
   - **Call: a separate display block.** The plan is sampled every 0.1 s over its 8 s horizon (t and the 9 values),
     beside the unchanged samples. It is for display only; B5 never reads it.

## Design

**Backend** (`mobile_manipulator_campaign`, `mobile_manipulator_mpc`, `mobile_manipulator_medium`):
- **B5's prediction.** A new **Prediction frame** (type 13).
  - Header: `t`, `status` (B5's cycle status: whether the plan is executed or held), `stage_dt` (0.1), `stages`
    (20), `plan_id` of the medium plan followed (if any).
  - Block 0: the solved plan's states, 21 x 9 floats (x, y, yaw, the six joints).
  - Sent every fifth B5 cycle (10 Hz; call). On in the demo server; off by default in `plant_campaign`
    (`--prediction`).
  - Source: `Controller`'s solved plan (`nominal_`), exposed read-only.
- **Plans frame.** A new block 1, the display samples: every 0.1 s from 0 to the horizon,
  `[t, x, y, yaw, q0..q5]` (10 floats each), evaluated from the same spline.
- **Scan frame.** A new block 2: each point's time offset from `t_start` (the plant's existing `dt`).
- **Hello capabilities:** `b5_prediction` (when on), `plan_display_samples`, `scan_point_times`.
- **Protocol and readers:** `stream-protocol.md`, `tools/plant/plant_stream.py` (the new type and blocks).
- **Identity check:**
  - a session with the prediction on and one with it off produce identical State streams and outcomes;
  - the existing campaign and demo-server tests pass unchanged.

**Viewer:**
- **Overlay 7, B5 prediction:** the predicted base path and 4 ghosts at 0.5, 1, 1.5 and 2 s, in a colour distinct from
  F2's.
  - When the frame's `status` says the plan is held, not executed, it is drawn in a warning colour, with "held" in the
    legend.
  - Stale after 1 s, as F2's plan is. "No data" without the capability.
- **F2 ghosts and sweep** use the display samples when present:
  - ghosts at real times along the plan;
  - the sweep through every 0.1 s sample, so an arm turn at rest is drawn as it moves;
  - the 4B-1 approximate path stays as the fallback for old servers.
- **Lidar** uses the point times when present: exact placement, and the legend says so. The approximation stays as
  the fallback.
- **Telemetry:** B5's predicted horizon end, and whether the plan is held.
- **Presets:** Diagnose includes overlay 7, Demo does not.

## Testing

- **Backend (gtest, container):**
  - the Prediction frame's layout and rate;
  - display samples match the spline at the existing samples' times;
  - Scan point times equal the plant's;
  - the prediction on and off give identical State streams;
  - the existing suites pass.
- **Viewer (EditMode):**
  - decoding the new frame and blocks;
  - prediction keying and staleness;
  - the sweep from display samples, including an arm turn at rest;
  - lidar placement from point times;
  - fallbacks without the capabilities.
- **Live check** on the door scenario: B5's prediction beside F2's plan, held versus executed.
- **Pacing check:** Diagnose with overlay 7 at 60.

## Left to the builder

Colours, ghost count and spacing beyond the four, the exact frame and block field names, and the decimation if
measurement favours another rate.
