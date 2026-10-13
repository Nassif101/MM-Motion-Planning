# Construction-site overhaul: brief for a separate session

**Date:** 2026-10-13
**Status:** brief (input to a design discussion and spec in its own session; no Unity work was
done when this was written)
**Before:** Phase 4B (MPC qualification) runs on the new site, so this overhaul comes first.

## 1. Why

The current scene (`motion-planning-sim/Assets/Scenes/ConstructionSiteV1.unity`) grew one test
at a time: gates, chicane blocks, fences and stacks were dropped wherever a scenario needed
them, and the Phase 4 overhead beam floats in the air. It looks empty, unorganized and
ill-structured, and does not read as a construction site. The thesis demo and the remaining
phases (4B, 5, 6) need a site that looks plausible and still contains every test feature,
placed where a real site would have it.

## 2. Hard requirements

1. **Own branch.** All work happens on a dedicated branch (e.g. `site-overhaul`) created
   from `main`; nothing is committed to `main` directly; the user merges.
2. **The old scene stays runnable.** Every recorded baseline (B1-B5, tag
   `baselines-2026-10-13`) ran on `ConstructionSiteV1` with `maps/construction_site.*` and
   `config/scenarios.yaml`. Build the new site as a new scene (e.g. `ConstructionSiteV2`)
   with its own exported map and planning boxes; do not modify V1, its exports or the existing
   scenarios. Scenario selection of the map must be explicit (for example a `map` key per
   scenario or a separate scenario file), and the runner and launch files must load the right
   map, boxes and scene for each scenario.
3. **Visually organized and physically plausible.** No floating or unsupported elements:
   every overhead beam, pipe, duct or scaffold board rests on columns, brackets, scaffolding or
   a structure. Walls are aligned to a common grid and connect at corners; doors and openings
   sit in walls with frames, aligned with the walls; fences run in straight, joined segments
   with gates; stacks sit on pallets or bearers; nothing intersects anything else. Materials,
   colours and signage are consistent across the site.
4. **A real site layout.** Recognizable zones connected by marked driving lanes, for example:
   fenced perimeter with a vehicle gate; site office and welfare containers; laydown yard
   (pallets, panel stacks, timber, rebar); a building shell under construction (column grid,
   slab edges, door and window openings, a stair or lift core); a scaffolding or formwork zone
   with overhead members; a services corridor with pipe racks; walkways where workers cross
   the lanes.
5. **Every element that matters for navigation or collision has an exported box proxy.** The
   planning-box exporter (`unity command export_planning_boxes`, consumed by Nav2's map, MoveIt
   and the MPC through `known_boxes`) supports oriented boxes only (centre, size, yaw; floating
   boxes are allowed through the centre height). Detailed meshes need box proxies under the
   navigation-obstacle hierarchy that match the mesh within about 2 cm; decorative items that
   the robot cannot touch need none.
6. **Flat drivable floor.** Nav2, the footprint model and the MPC assume a planar base. No ramps,
   steps or slopes on drivable areas (kerbs and level changes only outside them).
7. **Unity rules from AGENTS.md:** HDRP (no URP-specific tooling), Unity controlled through the
   Unity CLI and the project's `com.unity.pipeline` commands, never Unity MCP; follow the
   repository's skill routing (do not add general game or rendering skills unless the
   repository gains that scope deliberately).
8. **Real-time factor.** The full stack ran at a real-time factor of 1.00-1.04 on the
   development Mac. The new scene must keep it (measure before and after); prefer instanced,
   low-poly assets and baked lighting where possible.

## 3. Test features the site must contain

Place these inside the site's zones so they look natural (a doorway, not two free posts):

| Feature | Purpose | Notes |
|---|---|---|
| 1.05 m opening (doorway or gap between stacks) | Tight-passage stress test (vertical carry needed) | Straight approach of at least 3 m on both sides; the Nav2 route enters off-centre otherwise (Phase 4 finding) |
| 1.30 m opening | Main comparison passage | Same approach rule |
| Overhead members at several heights | Arm/panel must lower or reorient while driving | Bottom heights to be set against the arm poses (table below); include at least one that only the comfortable carry pose clears and one that requires reorientation; supported by columns or scaffolding |
| Low obstacles under the panel | Panel passes over, base does not | 0.6-1.0 m tall (pallet stacks, barriers) |
| Chicane / slalom | Continuous turning with the panel | |
| Worker crossings | Moving-obstacle scenarios | Walkway crossing a lane with sight lines; room for a worker to stop in the lane |
| Corridor driven both ways or a U-turn | Route passing near itself | Exercises the MPC's route-progress hint |
| Dead end and a turnaround space | Recovery behaviour, in-place turns | |
| Open area | Free-drive demo and open-space scenarios | At least 8 x 8 m clear |
| Goals near structures | Goals beside walls or posts | Leave about 0.5 m beyond inflation so goals are reachable |

## 4. Robot dimensions for layout

- Base box 0.85 x 0.59 m, top at 0.32 m; arm mount top 0.50 m; wheels outside the box (base
  footprint 0.92 x 0.77 m).
- Panel 1.20 x 1.20 x 0.04 m held at the tool flange.
- Footprints: `home` (panel horizontal) 1.24 x 1.24 m (x -0.70..0.54, y +-0.62); `vertical_carry`
  1.24 x 0.77 m; in vertical carry the panel sits 0.225 m to one side of the arm.
- Panel heights: straight `home` panel top about 1.565 m; vertical carry panel top about 1.92 m.
  A bent-arm `carry` pose is being added (phase4-followups branch) with a lower panel; take
  its height from `qualified_payload.json` once qualified, and set the overhead-member heights
  relative to these poses (with at least 0.10 m clearance for the pose meant to pass).
- Lidar: the safety layer and the B5 route see obstacles from 0.05 to 0.60 m (B1-B4 0.05-2.0 m);
  surfaces that matter must be lidar-visible (no glass, no thin wires as the only geometry).
- Main lanes at least 1.6 m wide (2 m where two features meet); the map is 40 x 40 m at
  0.05 m resolution today and may grow if needed (update the map export and costmap bounds).

## 5. Deliverables

- New scene with the zones and features above, on the branch.
- Exported Nav2 map (`.pgm`/`.yaml`), planning boxes and metadata for the new scene, passing
  the existing validators (`validate_construction_site`, `validate_nav2_map`,
  `validate_planning_boxes`) and the navigation tests (`test_unity_nav2_map.py`,
  `test_planning_boxes.py`), extended where needed.
- Explicit map selection in scenarios, runner and launch (old scenarios keep V1).
- A first scenario set for the new site covering section 3 (validated with `scenario_spec`
  and plan-only checks), ready for Phase 4B.
- Screenshots of each zone, the real-time factor before and after, and a short
  `docs/experiments/site-v2/README.md`; a row in the roadmap's comparability log.

## 6. Out of scope

- MPC, Nav2, MoveIt or controller changes (Phase 4B and later).
- Rerunning baselines (one final baseline run on the final site happens before the thesis
  tables; see the comparability log).
- Perceived-obstacle mapping (distance fields), which is a later baseline.
