# ADR 0012: Site selection and box proxies as the only colliders

Date: 2026-10-17. Status: accepted for site V2; revisit when a third site or a non-box obstacle is needed.

## Context

The recorded baselines B1-B5 ran on `ConstructionSiteV1`, which grew one test at a time and whose map
path, `scenarios.yaml` and 40 x 40 m size were hard-coded in the launch files, `scenario_spec`,
`planning_scene_loader`, `wholebody_mpc_node`, `clearance_3d`, `octomap_box_check`, `tools/demo.py` and
the Unity exporters. Phase 4B and the thesis tables need a second, organized site (zones, marked lanes,
every test feature) without disturbing V1, which the frozen baselines (tag `baselines-2026-10-13`)
depend on. Two properties of the stack decide how: the lidar (`IidNoiseRaycastLiDARSensor`) is a
physics raycast, so it sees colliders and not meshes; and the global planner, MoveIt and the MPC all
read geometry exported from the Unity colliders (ADR 0002 map handoff, `PlanningBoxExporter`). Design:
`docs/superpowers/specs/2026-10-16-construction-site-v2-design.md`; results:
`docs/experiments/site-v2/README.md`.

## Options considered

- How the site is chosen: (1) a launch argument alone, left to the user to match to the scenario; (2)
  the site in each scenario entry; (3) one scenario file per site that names its site, plus one `site`
  argument for runs without a scenario.
- What collides: (1) the visual meshes, as mesh colliders; (2) hand-placed colliders next to the
  visuals; (3) box proxies owned by each kit piece, and only those.
- Where the 2D map band sits: the V1 band (0.02-3.2 m) for both sites, or a per-site band.
- Tier 2 terrain disturbances: baked into the exports, or a switchable group the planners do not know.

## Decision

- **Scenario files name the site; one `site` argument.** `scenarios.yaml` is implicitly
  `construction_site`; `scenarios_site_v2.yaml` declares `site: construction_site_v2`. The C++
  `ScenarioConfig` is the single resolver: it reads every `config/scenarios*.yaml`, `resolve(name)`
  returns the scenario and its site, a name defined in two files is an error. Nodes that load the world
  take a `site` parameter (default `maps/<site>.boxes.yaml`); when a scenario is given its site wins and
  a disagreeing explicit `site` is an error. The launch files take `site` (V1 commands run unchanged).
  `scenario_spec` prints `site`, `scene` and `disturbances`; the scene comes from the exported
  `maps/<site>.metadata.json` (`sourceScene`). The runner and the demo switch the Unity scene
  themselves (`site_open`, Edit mode) and record site, scene and disturbances in each run.
- **Box proxies are the only colliders.** Visual meshes carry no colliders. Every kit piece carries box
  proxies (local centre, size, yaw) from one spec (`tools/site_kit/kit_v2.json`), placed under
  `Environment/NavigationObstacles/<Zone>/<Piece>`. Physics, the lidar, the Nav2 map, the planning
  boxes, MoveIt and the MPC therefore see identical geometry, and the exports are exact for the
  proxies. Proxies enclose the mesh and are never smaller than it (under 0.02 m of error on square
  sections, one box per round pipe, boxy barriers instead of tapered ones); the build compares each
  imported mesh with the spec (0.02 m) and validates the layout (`validate_construction_site_v2`).
- **Per-site 2D map band.** V1 keeps 0.02-3.2 m (byte-identical exports). V2 rasterizes colliders whose
  vertical extent intersects 0.02-0.60 m (B5's base-band top), so overhead members (H1 ledger, H3 pipe
  bridge) and door heads are not in V2's 2D map, as V1's overhead scenario boxes were not. The planning
  boxes keep 0.02-3.2 m for both sites. B1-B4 still see overhead members through their 0.05-2.0 m lidar
  and local-costmap band, as a lane-blocking obstacle whatever the arm pose; B5 knows them in 3D.
- **Tier 2 disturbances are switchable and unknown to the planners.** They sit under
  `Environment/Disturbances` (rumble strips, road plates, cable protector, gravel and mud patches),
  inactive by default, never exported; a scenario's `disturbances: true` switches them on
  (`site_disturbances`, Play only). Heights are at most 0.035 m, below the lidar's 0.05 m band; the
  friction patches are 3 mm slabs under the Minimum friction combine. The contact monitor ignores
  wheel contacts with colliders under `Environment/Disturbances`; other links still count.

## Consequences

- V1 outputs, `scenarios.yaml` and all V1 commands are unchanged; a V2 scenario runs from the scenario
  name alone. A new site needs a scene, exports, a metadata file and a scenario file.
- The proxy rule keeps the sensors and planners consistent and makes validation mechanical, at the
  cost of box-shaped approximations (scaffold, pipes and stacks add hundreds of boxes: 628 colliders in
  V2 against 28 in V1; MoveIt loads about 600 static boxes) and of a build that fails on
  intersections or floating pieces. The MPC and MoveIt timing at this box count is measured, not
  assumed (docs/experiments/site-v2/smoke-runs.md).
- Overhead members are invisible to a 2D-map planner by design, so B1-B4 stop before the H1 ledger in
  the tunnel scenarios and only B5 passes; scenario texts say so.
- Runs on V2 are not comparable to the recorded V1 baselines until the final baseline run (roadmap
  comparability log).
- Hand dressing (`Environment/HandDressing`) and Megascans props have no proxies; anything the base
  could reach must be a kit piece. The Megascans scans are git-ignored, as in V1.

## Revisit when

- A site needs a non-box obstacle that a set of boxes cannot enclose tightly (slopes and ramps, Tier 3).
- A third site is added, or two scenario files must share a name.
- Perceived-obstacle mapping replaces the exported geometry.
- The MPC or MoveIt timing at the V2 box count is a problem in the Phase 4B sweep (the 0.02 m proxy rule
  is not relaxed silently).
- Disturbances must be visible to the planners.
