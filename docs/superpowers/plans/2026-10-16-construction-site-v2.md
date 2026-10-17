# Construction Site V2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `ConstructionSiteV2` (a 50 x 40 m spine-road construction site from a box-proxied modular kit) and make the site an explicit input of the runner, the demo, Nav2, MoveIt and the MPC, without changing anything V1 produces.

**Architecture:** One kit spec JSON drives the headless Blender meshes and the Unity proxies; a C# builder places the kit per zone and a validator checks support, overlap and feature dimensions. On the ROS side, `ScenarioConfig` resolves every scenario to a site, and a `site` parameter and launch argument replace the hard-coded `maps/construction_site.*` paths; the runner switches the Unity scene and toggles the Tier 2 disturbances.

**Tech Stack:** Unity 6 Editor C# (HDRP 17.5, Pipeline `CliCommand`, EditMode NUnit tests), Blender 5.2.2 LTS headless Python (`bpy`), ROS 2 Jazzy C++17 + gtest, Nav2, MoveIt 2, pytest, Python tooling in `tools/`.

**Spec:** docs/superpowers/specs/2026-10-16-construction-site-v2-design.md (read it with this plan; Section numbers below refer to it).

## Global Constraints

- All work on branch `site-overhaul`; one commit per step or fix; never commit to `main`; the user merges.
- V1 stays byte-identical: `ConstructionSiteV1.unity`, `maps/construction_site.*`, `config/scenarios.yaml` are not modified. Every V1 command (launch, runner, demo) works unchanged with its defaults.
- Site names: `construction_site` (V1, default) and `construction_site_v2`. Scenario names in `scenarios_site_v2.yaml` start with `v2_` and are unique across all `config/scenarios*.yaml`.
- V2 map: ROS frame x -25..25, y -20..20, origin [-25, -20, 0], resolution 0.05 m, 1000 x 800 cells; Unity-ROS axis mapping as V1.
- Box proxies are the only colliders of kit pieces; proxies enclose the mesh, never smaller, at most 0.02 m larger per side (round pipes and tubes: one enclosing box each).
- Tier 2 disturbances: height <= 0.035 m, chamfered, under `Environment/Disturbances`, inactive by default, never exported.
- Overhead rules (Section 4): H1 = carry top + 0.10 m and at least 0.10 m below the home and vertical-carry tops; H3 = 2.40 m; no H2 (no qualified pose is lower than carry, 1.320 m vs `lowered` 1.458 m; decided 2026-10-16).
- Blender 5.2 LTS at `/Applications/Blender.app/Contents/MacOS/Blender`, run `--background --factory-startup`; no Blender MCP.
- Unity only through the Unity CLI and `com.unity.pipeline` commands, never Unity MCP; HDRP only. Stop Play (`unity command editor_stop`) when a task that used it finishes. Long Unity batches under `caffeinate -i`.
- Runtime nodes and ROS package code in C++; launch files, tests of configuration and `tools/` in Python.
- No host-absolute paths, credentials, Unity `Library/`, or ROS `build/ install/ log/` in commits; shell scripts LF. Kit assets live under `motion-planning-sim/Assets/SiteKit/` (never under a folder named `Models/`, which is git-ignored).
- Container commands: `C='docker exec mm-motion-planning-ma-robot-sim-1 bash -ic'`; `cb` builds; package tests: `$C 'cd $ROS_WS && colcon build --packages-select <pkgs> && colcon test --packages-select <pkgs> --event-handlers console_direct+ && colcon test-result --verbose'`. Unity EditMode tests through the Unity CLI (unity-cli skill) with a test-name filter. `pkill -f control_description` before mock tests.

### Plan-level decisions not spelled out in the spec

- **Scene path source:** `maps/<site>.metadata.json` already carries `sourceScene` and the grid size (`widthCells`, `heightCells`, `originRosX/Y`, `resolutionMetres`); nothing is added to it. The spec's "gains `source_scene`" is met by the existing `sourceScene` key (amend spec Section 2 in Task 1's commit).
- **Sites in Unity** are a small registry (`SiteRegistry`) in the Runtime assembly: name, scene path, grid. Exporter commands take `--site` (default `construction_site`).
- **MoveIt floor** (`scene_diff.hpp` `floor_size_m = 40.0`) becomes the site map's extent.
- **Plan-only checks** use a new runner flag `--plan-only`, which runs a `_nav`/`_mission` scenario as a `compute_path` task (planners GridBased and Lattice) after the usual placement, together with the existing `--footprint-profile` override. `tools/check_site_plans.py` loops scenarios x profiles and compares with `docs/experiments/site-v2/plan-expectations.yaml`.
- **Kit and layout logic that can be tested** (spec parsing, proxy placement, validation) lives in the Runtime assembly (`Scripts/Runtime/Environment/SiteKit/`) so the EditMode tests can reach it, as `PlanningBoxGeometry` does; the Editor code only instantiates.
- **Kit spec JSON is JsonUtility-shaped** (objects and arrays only, no dictionaries) so Unity parses it without a new dependency.

## Review Focus

1. **A V2 scenario launched by hand** (`ros2 launch ... scenario:=v2_door_105_nav`, with no `site:=` or with `site:=construction_site`): expected the V2 map without `site:=`, and a clear error naming both sites with the wrong one, never a run on the wrong map. Tests in Task 2 (`EffectiveSite.MismatchThrows`) and Task 3 (`test_launch_site_follows_the_scenario`, `test_launch_site_mismatch_raises`).
2. **The runner switching scenes mid-batch** (V1 scenario, then a V2 scenario, then V1 again): expected each run on its own scene with Play restarted only when the scene changes. Test in Task 5 (`test_mixed_site_batch_switches_scene_only_when_it_changes`).
3. **A run that fails or is interrupted with disturbances on**: expected the next run to start with them off. Test in Task 5 (`test_disturbances_are_switched_off_after_a_failed_run`).
4. **An imported mesh whose axes are swapped by the FBX import** (Blender Z-up vs Unity Y-up): expected the bounds check to reject it by piece name. Test in Task 8 (`MeshBoundsCheckRejectsSwappedAxes`).
5. **A kit piece edited in the spec without re-exporting the mesh**: expected `build_construction_site_v2` to fail with the piece name and both sizes. Covered by the same bounds check at build time (Task 8 Step 6).

---

### Task 1: Site-aware scenario resolution

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_navigation/include/mobile_manipulator_navigation/scenario_spec.hpp`
- Modify: `ros2_ws/src/mobile_manipulator_navigation/src/scenario_spec.cpp`
- Modify: `ros2_ws/src/mobile_manipulator_navigation/src/scenario_spec_main.cpp`
- Test: `ros2_ws/src/mobile_manipulator_navigation/test/test_scenario_spec.cpp`
- Modify: `docs/superpowers/specs/2026-10-16-construction-site-v2-design.md` (Section 2 metadata note)

**Interfaces:**
- Produces (namespace `mobile_manipulator_navigation`):
  - `inline constexpr const char * kDefaultSite = "construction_site";`
  - `StaticMap read_map(const std::string & root, const std::string & site = kDefaultSite);` (reads `maps/<site>.yaml`)
  - `ScenarioConfig::map(const std::string & site = kDefaultSite) const -> const StaticMap &` (lazy per-site cache; the constructor no longer reads a map)
  - `ScenarioConfig::names() const -> std::vector<std::string>` (sorted, all files)
  - `ScenarioConfig::site_of(const std::string & name) const -> std::string`
  - `ScenarioConfig::scene_of(const std::string & site) const -> std::string` (`sourceScene` of `maps/<site>.metadata.json`)
  - `resolve()` output gains top-level `"site"`, `"scene"`, `"disturbances"` (bool, the scenario's `disturbances`, default false); every check uses `map(site)`.
  - Exceptions: a name in two files -> `InvalidScenario("Scenario '<name>' is defined in <file1> and <file2>")`.

- [ ] **Step 1: Capture the V1 baseline output.** `$C 'cd $ROS_WS && source install/setup.bash && for n in $(ros2 run mobile_manipulator_navigation scenario_spec --list | python3 -c "import json,sys; print(\" \".join(json.load(sys.stdin)))"); do ros2 run mobile_manipulator_navigation scenario_spec $n; done' > <scratchpad>/v1-spec-before.jsonl`. Expected: one JSON line per V1 scenario.
- [ ] **Step 2: Write the failing gtests** in `test_scenario_spec.cpp` using a temporary root (copy `config/footprint_profiles.yaml`, write two tiny 20 x 20 cell maps `maps/construction_site.*` and `maps/site_b.*` with metadata `sourceScene`, `config/scenarios.yaml` with `a_nav`, `config/scenarios_site_b.yaml` with `site: site_b` and `b_nav`):
  - `ScenarioSites.FileWithoutSiteIsDefault`: `site_of("a_nav") == "construction_site"`.
  - `ScenarioSites.FileSiteIsUsed`: `resolve("b_nav")["site"] == "site_b"`, `["scene"] == "Assets/Scenes/B.unity"`, `["disturbances"] == false`.
  - `ScenarioSites.NamesSpanAllFiles`: `names() == {"a_nav", "b_nav"}`.
  - `ScenarioSites.DuplicateNameThrows`: add `a_nav` to the site_b file; `resolve("a_nav")` throws `InvalidScenario` whose message contains both file names.
  - `ScenarioSites.ChecksUseTheScenarioSitesMap`: `b_nav` start lies in a cell occupied only in `site_b`'s map -> `start_free == false`.
- [ ] **Step 3: Run** `$C 'cd $ROS_WS && colcon build --packages-select mobile_manipulator_navigation && colcon test --packages-select mobile_manipulator_navigation --ctest-args -R test_scenario_spec'`. Expected: the new tests fail to compile or fail.
- [ ] **Step 4: Implement** the interfaces above. `scenario_spec_main --list` prints `names()`; `--start-free-for` uses `map(resolved["site"])`.
- [ ] **Step 5: Run the tests.** Expected: PASS, including every existing test in the file.
- [ ] **Step 6: V1 regression.** Rebuild, rerun Step 1's loop into `v1-spec-after.jsonl`; compare with `python3 -c` that loads both line by line, drops `site`, `scene`, `disturbances` from each after-line, and asserts equality. Expected: identical; every after-line has `"site": "construction_site"`, `"scene": "Assets/Scenes/ConstructionSiteV1.unity"`.
- [ ] **Step 7: Commit** `git commit -m "Resolve each scenario to its site across scenario files"` (with the spec note).

### Task 2: Site parameter in the world loaders

**Files:**
- Modify: `scenario_spec.hpp/.cpp` (navigation)
- Modify: `ros2_ws/src/mobile_manipulator_manipulation/src/planning_scene_loader.cpp`, `src/octomap_box_check.cpp`, `include/mobile_manipulator_manipulation/scene_diff.hpp`, `src/scene_diff.cpp`
- Modify: `ros2_ws/src/mobile_manipulator_mpc/src/wholebody_mpc_node.cpp`, `src/clearance_3d.cpp`
- Test: `test_scenario_spec.cpp`, `ros2_ws/src/mobile_manipulator_manipulation/test/test_scene_diff.cpp`

**Interfaces:**
- Consumes: Task 1 (`ScenarioConfig::site_of`, `map(site)`, `kDefaultSite`).
- Produces:
  - `std::string effective_site(const std::string & requested, const std::optional<std::string> & scenario_site);` empty `requested` -> scenario site, else `kDefaultSite`; both given and different -> `InvalidScenario("Scenario site <s> differs from site parameter <r>")`.
  - `std::string site_boxes_file(const std::string & root, const std::string & site);` -> `<root>/maps/<site>.boxes.yaml`.
  - `struct MapExtent { double min_x, min_y, max_x, max_y; };` and `MapExtent extent_of(const StaticMap &);`
  - `SceneInputs::floor_extent` (`MapExtent`) replaces `floor_size_m`.
  - Node parameters: `site` (string, default `""`) on `planning_scene_loader`, `wholebody_mpc`; CLI `--site` on `clearance_3d` and `octomap_box_check`. `boxes_file` default becomes `""`, meaning `site_boxes_file(navigation, effective site)`. `octomap_box_check` and `planning_scene_loader` read the scenario through `resolve()`.

- [ ] **Step 1: Write the failing tests:**
  - `EffectiveSite.EmptyRequestTakesScenarioSite`, `EffectiveSite.NoScenarioTakesRequestOrDefault` (`effective_site("", std::nullopt) == "construction_site"`), `EffectiveSite.MismatchThrows` (`effective_site("construction_site", "construction_site_v2")` throws).
  - `SceneDiff.FloorCoversTheMapExtent`: extent {-25, -20, 25, 20} -> floor box centre (0, 0), size x 50, y 40 (z as today).
- [ ] **Step 2: Run** the navigation and manipulation tests. Expected: FAIL.
- [ ] **Step 3: Implement** the interfaces; each node logs `site <name>, boxes <file>` once at start.
- [ ] **Step 4: Run** `colcon test` for `mobile_manipulator_navigation mobile_manipulator_manipulation mobile_manipulator_mpc`. Expected: PASS (all existing tests unchanged).
- [ ] **Step 5: Commit** `git commit -m "Load the map boxes and MoveIt floor of the scenario's site"`.

### Task 3: Site launch argument

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_navigation/launch/site_selection.py` (installed with the launch files)
- Modify: `ros2_ws/src/mobile_manipulator_navigation/launch/global_planning.launch.py`, `launch/navigation.launch.py`
- Modify: `ros2_ws/src/mobile_manipulator_manipulation/launch/manipulation.launch.py`
- Test: `ros2_ws/src/mobile_manipulator_navigation/test/test_navigation_config.py`

**Interfaces:**
- Consumes: Task 2 node parameters.
- Produces: launch argument `site` (default `""`: the scenario's site, else `construction_site`, so V1 commands behave as before; this refines the spec's "default `construction_site`") in all three files. A shared helper module `mobile_manipulator_navigation/launch/site_selection.py` with:
  - `def scenario_site(share: Path, scenario: str) -> str` (reads the top-level `site` and the scenario names of `config/scenarios*.yaml`; unknown scenario -> `RuntimeError`),
  - `def launch_site(share: Path, site: str, scenario: str) -> str` (empty site -> scenario site or `construction_site`; both given and different -> `RuntimeError(f"site {site} differs from scenario {scenario}'s site {s}")`),
  - `def site_map_yaml(share: Path, site: str) -> str` raising `RuntimeError(f"Unknown site '{site}'; expected one of {sorted(...)}")` when `maps/<site>.yaml` is missing.
  `navigation.launch.py` passes the resolved site to `global_planning.launch.py` and to `wholebody_mpc`; `manipulation.launch.py` to `planning_scene_loader`. Nodes always receive an explicit site.

- [ ] **Step 1: Write the failing tests** `test_site_map_yaml_resolves_v1` (returns `.../maps/construction_site.yaml`), `test_site_map_yaml_rejects_unknown_site` (message names the known sites), `test_launch_site_defaults_to_v1`, `test_launch_site_follows_the_scenario` (a temporary share with a `site: construction_site_v2` scenario file), `test_launch_site_mismatch_raises` (Review Focus 1), and `test_launch_files_declare_site` (the three launch descriptions declare `site` with default `""`).
- [ ] **Step 2: Run** the navigation package tests. Expected: FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run.** Expected: PASS.
- [ ] **Step 5: Commit** `git commit -m "Add the site launch argument to navigation and manipulation"`.

### Task 4: Site registry, site-aware exporters and scene commands (Unity)

**Files:**
- Create: `motion-planning-sim/Assets/Scripts/Runtime/Environment/SiteRegistry.cs`
- Modify: `motion-planning-sim/Assets/Scripts/Editor/Nav2MapExporter.cs`, `Scripts/Editor/PlanningBoxExporter.cs`, `Scripts/Editor/ConstructionSiteTools.cs` (V1 build keeps calling the V1 export)
- Create: `motion-planning-sim/Assets/Scripts/Editor/SiteCommands.cs`
- Test: `motion-planning-sim/Assets/Tests/EditMode/SiteRegistryTests.cs`

**Interfaces:**
- Produces:
  - `public sealed class SiteDefinition { public string Name; public string ScenePath; public Nav2GridSpec Grid; }` and `public static class SiteRegistry { public static SiteDefinition Get(string name); public static IReadOnlyList<SiteDefinition> All; }` with `construction_site` (`Assets/Scenes/ConstructionSiteV1.unity`, grid -20, -20, 20, 20, 0.05) and `construction_site_v2` (`Assets/Scenes/ConstructionSiteV2.unity`, grid -25, -20, 25, 20, 0.05). Unknown name -> `ArgumentException` listing the known sites.
  - CLI: `export_nav2_map`, `validate_nav2_map`, `export_planning_boxes`, `validate_planning_boxes` take `string site = "construction_site"`; outputs `<site>.pgm|yaml|metadata.json|boxes.yaml`. Objects under `Environment/Disturbances` are never collected.
  - CLI `site_active` (any mode) -> `{ scene, playing }`; `site_open(string scene)` (Edit mode only; Play -> `InvalidOperationException("Stop Play before opening a scene.")`) -> `{ scene, changed }`, no-op when active.

- [ ] **Step 1: Write the failing EditMode tests:** `SiteRegistryTests.V1GridIsUnchanged` (-20, -20, 20, 20, 0.05), `SiteRegistryTests.V2GridIs50By40` (width 1000, height 800 cells), `SiteRegistryTests.UnknownSiteListsKnownSites`.
- [ ] **Step 2: Run** EditMode tests filtered to `SiteRegistryTests`. Expected: FAIL.
- [ ] **Step 3: Implement** the registry and the `site` parameter in both exporters.
- [ ] **Step 4: Run** the EditMode tests. Expected: PASS (all suites).
- [ ] **Step 5: V1 byte identity.** `unity command export_nav2_map` and `unity command export_planning_boxes` (defaults), then `git diff --exit-code ros2_ws/src/mobile_manipulator_navigation/maps/`. Expected: no diff. Also `unity command validate_nav2_map` and `validate_planning_boxes` -> validated.
- [ ] **Step 6: Implement `SiteCommands.cs`; check** `unity command site_active` -> V1 scene; `unity command site_open --scene Assets/Scenes/ConstructionSiteV1.unity` -> `changed: false`.
- [ ] **Step 7: Commit** `git commit -m "Make the map and box exporters site-aware and add scene commands"`.

### Task 5: Runner and demo select the site

**Files:**
- Modify: `tools/run_nav_scenario.py`, `tools/demo.py`
- Test: `tools/test_run_nav_scenario.py`, `tools/test_demo.py`

**Interfaces:**
- Consumes: `scenario_spec` fields `site`, `scene`, `disturbances` (Task 1); launch `site` (Task 3); `site_active`, `site_open` (Task 4); `site_disturbances --enabled true|false` (Task 12; called only for scenarios with `disturbances: true`).
- Produces:
  - `Runner.site` (str, default `"construction_site"`), `Runner.ensure_scene(scene: str, site: str) -> bool` (True when it switched; calls `new_epoch(scene=scene)`), `Runner.new_epoch(scene: str | None = None)` (between `editor_stop` and `editor_play` calls `site_open --scene`), `Runner.set_disturbances(enabled: bool)`.
  - Every `ros2 launch` the runner starts (navigation, global planning, manipulation, including the recovery path) gets ` site:={runner.site}`.
  - Run summary keys `site`, `scene`, `disturbances`.
  - `--plan-only`: a `navigate_to_pose` or `mission` scenario runs as `compute_path` with planners `[GridBased, Lattice]` after placement; run dir suffix `-plan`.
  - `demo.py --site NAME` (default from the given scenario, else `construction_site`); `scenarios()` lists every `scenarios*.yaml` with its site.

- [ ] **Step 1: Write the failing pytest cases** (extend `FakeRunner` to record `unity` calls and launch strings):
  - `test_mixed_site_batch_switches_scene_only_when_it_changes`: active V1; run V1, V2, V2, V1 specs -> `site_open` called twice (V2, V1), `editor_stop`/`editor_play` around each.
  - `test_every_launch_carries_the_site`: a V2 navigate spec -> navigation launch string contains `site:=construction_site_v2`.
  - `test_disturbances_are_switched_on_for_the_scenario_and_off_after`: `disturbances: true` -> `site_disturbances --enabled true` after placement, `--enabled false` at the end.
  - `test_disturbances_are_switched_off_after_a_failed_run`: the task raises -> `--enabled false` still called.
  - `test_v1_scenarios_never_touch_disturbances`.
  - `test_plan_only_runs_a_nav_scenario_as_compute_path`.
  - `test_summary_records_site_scene_and_disturbances`.
  - In `test_demo.py`: `test_demo_site_defaults_to_the_scenario_site`.
- [ ] **Step 2: Run** `python3 -m pytest tools/test_run_nav_scenario.py tools/test_demo.py -q`. Expected: the new tests fail.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run.** Expected: PASS (all).
- [ ] **Step 5: V1 smoke run.** `caffeinate -i python3 tools/run_nav_scenario.py open_space_nav --new-epoch`. Expected: success, summary `site: construction_site`, no contact. `unity command editor_stop`.
- [ ] **Step 6: Commit** `git commit -m "Select the site's scene, map and disturbances in the runner and demo"`.

### Task 6: Overhead heights from the poses

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_geometry/include/mobile_manipulator_geometry/footprint_projection.hpp`, `src/footprint_projection.cpp`
- Test: `ros2_ws/src/mobile_manipulator_geometry/test/test_projection.cpp`
- Create: `ros2_ws/src/mobile_manipulator_navigation/src/robot_heights.cpp` (+ CMake target)
- Create: `tools/site_kit/overhead_heights.py`, `tools/site_kit/test_overhead_heights.py`
- Create: `tools/site_kit/kit_v2.json` (only the `overhead` object in this task)

**Interfaces:**
- Produces:
  - `double FootprintProjector::top_z(const JointMap & joints) const;` highest point of every primitive (box corners, cylinder end-cap circles at `cylinder_sides`) and the payload box, in `base_footprint`.
  - `ros2 run mobile_manipulator_navigation robot_heights --poses FILE` reads `{"name": [6 joints], ...}` (order of `qualified_payload.json` `joint_order`) and prints `{"name": top_z, ...}`.
  - `overhead_heights.py`: `def overhead(tops: dict[str, float], named: tuple = ("home", "vertical_carry", "carry")) -> dict` returning `{"h1_bottom_m", "h3_bottom_m": 2.40, "tops_m": tops}` per the Global Constraints rules; raises `HeightConflict` (message with all tops) when H1 cannot satisfy both limits. `main()` takes the named poses from `poses_rad`, calls `robot_heights` in the container, writes `overhead` into `kit_v2.json`.

- [ ] **Step 1: Write the failing tests:** `Projection.TopZOfHomeIsThePanelTop` (`top_z(home)` within 0.01 m of 1.565), `Projection.TopZOfVerticalCarry` (within 0.01 m of 1.92); pytest `test_h1_is_carry_top_plus_clearance` (tops home 1.565, vertical 1.92, carry 1.20 -> h1 1.30), `test_h1_conflict_raises` (carry 1.50), `test_h2_raised_to_lowest_candidate` (carry 1.20, candidate `lowered` 1.00 -> h2 1.10, `h2_passing == "lowered"`), `test_h2_conflict_raises` (no candidate below 1.10).
- [ ] **Step 2: Run** geometry gtests and `python3 -m pytest tools/site_kit -q`. Expected: FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run.** Expected: PASS.
- [ ] **Step 5: Compute** `python3 tools/site_kit/overhead_heights.py`. Expected: `kit_v2.json` gets `overhead`. On `HeightConflict`: **stop the plan and report the tops to the user** (spec Section 4).
- [ ] **Step 6: Commit** `git commit -m "Compute the overhead member heights from the qualified poses"`.

### Task 7: Kit spec and Blender meshes

**Files:**
- Modify: `tools/site_kit/kit_v2.json` (add `pieces`)
- Create: `tools/site_kit/test_kit_spec.py`
- Create: `tools/blender/site_kit/build_kit.py`, `tools/blender/site_kit/common.py`, one script per family: `scaffold.py`, `fence.py`, `cabin.py`, `stacks.py`, `barrier.py`, `pipe_rack.py`, `props.py` (skip, IBC), `disturbances.py`
- Create: `motion-planning-sim/Assets/SiteKit/Meshes/*.fbx`, `docs/experiments/site-v2/kit-previews/*.png`

**Interfaces:**
- Produces the kit spec schema (JsonUtility-shaped):
  - `{"version": 1, "blender": "5.2", "overhead": {...}, "pieces": [Piece]}`
  - `Piece = {"id": str, "family": str, "mesh": "Meshes/<id>.fbx" or "" (Unity primitive), "size": [x, y, z], "parametric_x": bool, "round": bool, "proxies": [{"centre": [x, y, z], "size": [x, y, z], "yaw": rad}], "material": str, "physics_material": str}`
  - Piece frame: right-handed, x along the length, z up, origin at the bottom centre of the mesh bounds; metres.
  - Pieces (ids fixed here): `wall_blockwork` (0.2 thick, 3.0 high, parametric), `column_rc` (0.4 x 0.4 x 3.0), `beam_rc`, `core_wall`, `slab_finish` (no proxies), `scaffold_standard`, `scaffold_ledger_2p5`, `scaffold_transom_1p2`, `scaffold_board_2p5`, `scaffold_toe_board_2p5`, `fence_panel_3p5`, `hoarding_panel_2p4`, `cabin_office`, `cabin_welfare`, `cabin_drying`, `pallet_euro`, `stack_block_tall`, `stack_pallet_low_0p8`, `stack_panel`, `bearers_timber`, `bearers_rebar`, `barrier_block_0p8`, `pipe_rack_support`, `pipe_run`, `pipe_bridge`, `skip`, `ibc_tank`, `spoil_heap` (stepped proxies), `trench_barrier`, `cable_protector_0p035`, `road_plate_0p025`, `rumble_strip_0p02`, `door_frame_1p30`, `door_frame_1p05`.
  - `build_kit.py [--only ID...]`: exports FBX (`axis_forward='-Z'`, `axis_up='Y'`, `apply_scale_options='FBX_SCALE_UNITS'`, mesh only) and a 480 x 360 Workbench preview per piece; exits non-zero when `bpy.app.version[:2] != (5, 2)` or when an exported object's bounds differ from `size` by more than 0.02 m on any axis.

- [ ] **Step 1: Write the failing pytest** `test_kit_spec.py`: `test_ids_are_unique`; `test_proxies_enclose_the_mesh` (the union of a piece's proxies covers `size` on every axis and exceeds it by at most 0.02 m per side unless `round`); `test_tier2_pieces_are_below_the_lidar_band` (ids `cable_protector*`, `road_plate*`, `rumble_strip*`: `size[2] <= 0.035`); `test_overhead_present`.
- [ ] **Step 2: Run** `python3 -m pytest tools/site_kit -q`. Expected: FAIL (no pieces).
- [ ] **Step 3: Write the spec pieces and the Blender scripts,** family by family, one commit per family (`git commit -m "Add the <family> kit pieces"`). Run per family: `/Applications/Blender.app/Contents/MacOS/Blender --background --factory-startup --python tools/blender/site_kit/build_kit.py -- --only <ids>`. Expected: FBX and preview written, bounds within 0.02 m. Look at each preview PNG (Read tool) before committing.
- [ ] **Step 4: Run** the pytest. Expected: PASS.
- [ ] **Step 5: User checkpoint.** Publish the previews as one page (Artifact) and ask the user to review them. Apply the requested changes before Task 8.

### Task 8: SiteKit in Unity (spec, proxies, bounds check, materials)

**Files:**
- Create: `motion-planning-sim/Assets/Scripts/Runtime/Environment/SiteKit/KitSpec.cs`, `KitGeometry.cs`
- Create: `motion-planning-sim/Assets/Scripts/Editor/SiteKit.cs`
- Create: `motion-planning-sim/Assets/SiteKit/Materials/*.mat`, `Assets/SiteKit/Textures/*` + `Assets/SiteKit/Textures/SOURCES.md`, `Assets/SiteKit/PhysicsMaterials/*.physicMaterial`
- Test: `motion-planning-sim/Assets/Tests/EditMode/SiteKitTests.cs`

**Interfaces:**
- Consumes: kit spec (Task 7); `RosBox` and `PlanningBoxGeometry` (existing); `RosMapPose` (existing).
- Produces:
  - `KitSpec.Parse(string json) -> KitSpec` with `Pieces`, `Overhead`, `Piece Get(string id)`.
  - `KitGeometry.ProxyBoxes(KitPiece piece, double rosX, double rosY, double rosZ, double yaw, float lengthX = 0) -> RosBox[]` (map-frame proxies; `lengthX` scales `parametric_x` pieces and their proxies along x).
  - `KitGeometry.BoundsMismatch(KitPiece piece, Vector3 importedRosSize, float tolerance = 0.02f) -> string` (null when it fits, else `"<id>: mesh <size> vs spec <size>"`).
  - Editor `SiteKit.Place(string pieceId, string name, Transform visualParent, Transform proxyParent, double rosX, double rosY, double rosZ, double yaw, float lengthX = 0) -> GameObject`: visual without colliders (static, shared material, GPU instancing); one child `BoxCollider` per proxy under `proxyParent/<name>`; missing FBX -> primitive placeholder of `size`; `physics_material` assigned to the proxies.
  - Materials: asphalt, compacted_ground, gravel, concrete, blockwork, timber, galvanized_steel, painted_steel, plastic, cabin_paint, lane_marking, walkway_marking (HDRP Lit, CC0 1K textures from ambientCG or Poly Haven; `SOURCES.md` lists URL and licence per texture). Physics materials: `floor_default` (0.05/0.02, combine minimum, as the current floor), `gravel_patch` (0.10/0.05), `mud_patch` (0.03/0.01).

- [ ] **Step 1: Write the failing EditMode tests:** `SiteKitTests.ParsesPiecesAndOverhead`; `ProxyBoxesAreTranslatedAndYawed` (piece at (2, 3, 0) yaw pi/2: proxy centre rotated, `Yaw == pi/2`); `ParametricWallScalesItsProxy` (`wall_blockwork` with `lengthX` 4.5 -> proxy size x 4.5); `MeshBoundsCheckAcceptsWithinTolerance` (0.015 m off -> null); `MeshBoundsCheckRejectsSwappedAxes` (size y and z swapped -> message contains the id).
- [ ] **Step 2: Run** filtered to `SiteKitTests`. Expected: FAIL.
- [ ] **Step 3: Implement** `KitSpec`, `KitGeometry`.
- [ ] **Step 4: Run.** Expected: PASS.
- [ ] **Step 5: Add materials, textures and physics materials; implement `SiteKit.Place`.**
- [ ] **Step 6: Build-time bounds check.** `SiteKit.Place` throws with `BoundsMismatch`'s message when an imported FBX does not fit. Check by temporarily editing one piece's spec size by 0.05 m and placing it in a scratch scene: expected the error naming the piece; revert.
- [ ] **Step 7: Commit** `git commit -m "Place kit pieces with box proxies and shared materials"`.

### Task 9: V2 builder and validator, first zones (user checkpoint)

**Files:**
- Create: `motion-planning-sim/Assets/Scripts/Runtime/Environment/SiteKit/SiteValidation.cs`
- Create: `motion-planning-sim/Assets/Scripts/Editor/SiteV2Layout.cs` (all zone coordinates and feature declarations as constants), `Scripts/Editor/ConstructionSiteV2Builder.cs`
- Create: `motion-planning-sim/Assets/Scenes/ConstructionSiteV2.unity`
- Test: `motion-planning-sim/Assets/Tests/EditMode/SiteValidationTests.cs`

**Interfaces:**
- Consumes: `SiteKit.Place`, `KitSpec` (Task 8), `SiteRegistry` (Task 4).
- Produces:
  - `public sealed class NamedBox { public string Name; public RosBox Box; }`
  - Feature declarations: `DoorFeature { Name, CentreX, CentreY, WallYaw, ClearWidth, ApproachOut = 3.0, ApproachIn = 5.0 }`, `ClearRect { Name, CentreX, CentreY, SizeX, SizeY, Yaw }` (lanes, open area, approaches), `OverheadFeature { Name (proxy name), BottomZ }`, `JointPair { A, B }`.
  - `SiteValidation.Problems(IReadOnlyList<NamedBox> boxes, SiteFeatures features, Nav2GridSpec grid) -> List<string>` checking: inside the grid; pairwise intersection deeper than 0.005 m unless declared; support (bottom > 0.01 m needs a box below whose top is within 0.01 m and whose footprint overlaps); door clear width (nearest boxes along the wall line from the door centre) within 0.005 m of `ClearWidth`; approach rectangles and `ClearRect`s free of boxes below 2.5 m; overhead bottom within 0.005 m of the kit spec value.
  - CLI `build_construction_site_v2` (rebuilds the generated roots listed in spec Section 2, leaves `Environment/HandDressing`, places the default spawn marker at (-21, 0, 0), saves, runs the validator) and `validate_construction_site_v2`.

- [ ] **Step 1: Write the failing EditMode tests:** `SiteValidationTests.FloatingBoxIsReported`; `IntersectingBoxesAreReportedUnlessJoined`; `TouchingBoxesAreFine` (0.004 m overlap); `DoorNarrowerThanDeclaredIsReported` (1.03 m vs 1.05 m); `BoxInApproachIsReported`; `OverheadAtWrongHeightIsReported`; `BoxOutsideGridIsReported`.
- [ ] **Step 2: Run** filtered to `SiteValidationTests`. Expected: FAIL.
- [ ] **Step 3: Implement** `SiteValidation`.
- [ ] **Step 4: Run.** Expected: PASS.
- [ ] **Step 5: Builder, first zones:** ground (50 x 40 m, `floor_default`), perimeter (hoarding and fence, gate y -2.5..2.5), spine, building shell (walls, columns, core, door frames, slab finish), lighting and cameras reused from V1 (`ConstructionSiteTools` helpers made `internal`). Coordinates from spec Section 4 in `SiteV2Layout`.
- [ ] **Step 6: Run** `unity command build_construction_site_v2` then `validate_construction_site_v2`. Expected: no problems.
- [ ] **Step 7: Screenshots** of the gate, spine and building (overview camera and robot height), saved to `docs/experiments/site-v2/screenshots/`.
- [ ] **Step 8: Commit** `git commit -m "Build the V2 spine, gate and building shell with a validator"`.
- [ ] **Step 9: User checkpoint.** Show the screenshots; apply the requested changes before Task 10.

### Task 10: V2 remaining zones

**Files:**
- Modify: `SiteV2Layout.cs`, `ConstructionSiteV2Builder.cs`, `ConstructionSiteV2.unity`

**Interfaces:**
- Consumes: Task 9.
- Produces: zones laydown yard (chicane, low pallets, bearers), scaffold tunnel (bays both sides, H1 ledger at y 7, turnaround pocket), office and welfare cabins, walkways W1 and W2 (decals), delivery yard with skip, services corridor (pipe rack, H3 pipe bridge at y -14, block barriers, turn bay, dead-end wall), Tier 1 terrain (spoil heap, trench inside barriers), lane and edge markings, `Environment/HandDressing` (created empty if missing), optional Megascans dressing (placed only when the scan exists). Each zone a builder method; each zone's features declared in `SiteV2Layout`.

- [ ] **Step 1: Add one zone at a time;** after each, run `build_construction_site_v2` (validator included). Expected: no problems. Commit per zone (`git commit -m "Add the V2 <zone>"`).
- [ ] **Step 2: Screenshots** of every zone into `docs/experiments/site-v2/screenshots/`; view them before committing.
- [ ] **Step 3: Rebuild twice and diff** the scene file: the second build changes nothing (`git diff --exit-code motion-planning-sim/Assets/Scenes/ConstructionSiteV2.unity` after committing the first). Expected: no diff.

### Task 11: V2 exports and map tests

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_navigation/maps/construction_site_v2.{pgm,yaml,metadata.json,boxes.yaml}`
- Modify: `ros2_ws/src/mobile_manipulator_navigation/test/test_unity_nav2_map.py`, `test/test_planning_boxes.py`, `CMakeLists.txt` (install, if maps are listed)

**Interfaces:**
- Consumes: Task 4 exporters, Task 10 scene.
- Produces: V2 map and boxes; tests parametrized over `construction_site` and `construction_site_v2` with per-site origin and size (V2: `origin: [-25.000, -20.000, 0.000]`, 1000 x 800).

- [ ] **Step 1: Parametrize the map and box tests** over both sites; add `test_v2_door_cells_are_free` (cells at the two door centres free, cells 0.2 m beside each reveal occupied) and `test_v2_disturbances_are_not_in_the_boxes` (no box name contains `Disturbances`). Run. Expected: V2 cases fail (no files).
- [ ] **Step 2: Export** `unity command export_nav2_map --site construction_site_v2`, `export_planning_boxes --site construction_site_v2`; `validate_*` for both sites.
- [ ] **Step 3: Run** the navigation package tests. Expected: PASS for both sites.
- [ ] **Step 4: Audit the 40 x 40 m assumptions.** `git grep -nE "40\.0|20\.0|-20\.0|construction_site[^_]" -- ros2_ws/src/mobile_manipulator_* tools motion-planning-sim/Assets/Scripts` and review each hit; every one that encodes the V1 map extent or name outside V1-only code takes the site (or is listed in the commit message as intentionally V1). Expected: no unparameterized V1 extent left in shared code.
- [ ] **Step 5: Commit** `git commit -m "Export the V2 Nav2 map and planning boxes"`.

### Task 12: Tier 2 disturbances

**Files:**
- Modify: `ConstructionSiteV2Builder.cs`, `SiteV2Layout.cs`, `Scripts/Editor/SiteCommands.cs`
- Test: `motion-planning-sim/Assets/Tests/EditMode/SiteKitTests.cs`

**Interfaces:**
- Consumes: Tier 2 kit pieces (Task 7), physics materials (Task 8), runner calls (Task 5).
- Produces: `Environment/Disturbances` (inactive in the saved scene) with rumble strips (spine inside the gate), road plates (spine x 9..11), cable protector (tunnel y 4.0..4.4), gravel patch (turnaround pocket, `gravel_patch`), mud patch (delivery yard around x -3.5, y -14, `mud_patch`). CLI `site_disturbances(bool enabled)` (Play only; returns `{ enabled, count }`).

- [ ] **Step 1: Write the failing test** `SiteKitTests.Tier2PiecesStayBelowTheLidarBand` (every kit piece placed under `Disturbances` in `SiteV2Layout` has top <= 0.035 m) — run, expect FAIL until the layout declares them.
- [ ] **Step 2: Implement** the zone and the command; rebuild; re-export V2 and confirm `validate_nav2_map --site construction_site_v2` still passes (disturbances not exported).
- [ ] **Step 3: Play check:** `site_disturbances --enabled true` -> `count` 5 (one per disturbance group: rumble strips, road plates, cable protector, gravel patch, mud patch); drive the base by keyboard teleop or `cmd_vel` over the road plates at 0.3 m/s and confirm in `/global_costmap/costmap` (RViz or one `ros2 topic echo --once` sample) no marks at the plates; `--enabled false`; `editor_stop`.
- [ ] **Step 4: Commit** `git commit -m "Add the switchable Tier 2 disturbances"`.

### Task 12A: Interior layouts (building fit-out, yard store) and walkway W2

Added 2026-10-17 at the user's request (spec Section 4 "Interior layouts"; W2 moved in line with door A).

**Files:**
- Modify: `motion-planning-sim/Assets/Scripts/Runtime/Environment/SiteKit/SiteV2Layout.cs`, `Assets/Scripts/Editor/ConstructionSiteV2Builder.cs` (if needed), `Assets/Scenes/ConstructionSiteV2.unity`
- Modify: `ros2_ws/src/mobile_manipulator_navigation/maps/construction_site_v2.*` (re-export)
- Test: `motion-planning-sim/Assets/Tests/EditMode/SiteValidationTests.cs` (layout test), `ros2_ws/src/mobile_manipulator_navigation/test/test_unity_nav2_map.py`

**Interfaces:**
- Consumes: Tasks 9-12 (layout, validator, kit, exporters, disturbances).
- Produces: the fit-out and yard store of spec Section 4 as validator-declared features: every new door a `DoorFeature` (clear width 1.30 or 1.05, `ApproachOut`/`ApproachIn` sized to the room or hallway it opens into, at least 1.8 m), hallway, corridor and aisles as `ClearRect`s, the open area `ClearRect` x -7..7, y -10.5..-2.5; W2 at x 4.5..6.0; skip and mud patch moved as the spec says.

- [ ] **Step 1: Write the failing tests:** EditMode `SiteValidationTests.V2InteriorDoorsHaveTheirClearWidths` (all nine fit-out doors declared, widths 1.30/1.05) and pytest `test_v2_interior_doors_are_free` (map cells at every fit-out door centre free, cells 0.2 m beside each reveal occupied). Run: FAIL.
- [ ] **Step 2: Implement** the fit-out, the yard store, the W2 move, skip and mud patch moves; `build_construction_site_v2`, `validate_construction_site_v2` -> no problems.
- [ ] **Step 3: Re-export V2** (`export_nav2_map`, `export_planning_boxes` with `--site construction_site_v2`); run the EditMode suite and the navigation package tests. Expected: PASS; V1 exports unchanged.
- [ ] **Step 4: Screenshots** of the fit-out (hallway, a 1.05 m room door, store S) and the yard store into `docs/experiments/site-v2/screenshots/`; look at them.
- [ ] **Step 5: Commit** `git commit -m "Add the building fit-out and the yard store, align walkway W2 with door A"`.

### Task 13: V2 scenario set and plan checks

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_navigation/config/scenarios_site_v2.yaml` (+ install in `CMakeLists.txt` if config files are listed)
- Create: `tools/check_site_plans.py`, `docs/experiments/site-v2/plan-expectations.yaml`
- Test: `ros2_ws/src/mobile_manipulator_navigation/test/test_scenario_spec.cpp` (one test over the real files)

**Interfaces:**
- Consumes: Tasks 1, 5 (`--plan-only`), 11, 12.
- Produces: the 20 scenarios of spec Section 6 (names verbatim), `site: construction_site_v2`, starts and goals in the zones of Section 4, movers on W1 and W2 mirroring `worker_crossing_nav`, `worker_late_crossing_nav`, `worker_stops_in_lane_nav`, `disturbances: true` on the two `_rough_` scenarios. Each `description` states the expected outcome per baseline where it is a failure (for example "B1-B4 expected to stop before H1; B5 expected to pass"), so Phase 4B does not count those as regressions. `plan-expectations.yaml`: `{scenario: {home: bool, vertical_carry: bool, carry: bool}}` for every scenario. `check_site_plans.py [--scenarios ...]` runs the runner with `--plan-only --footprint-profile P` for P in home, vertical_carry, carry and prints a table plus mismatches; exit 1 on a mismatch.

- [ ] **Step 1: Write the failing gtest** `ScenarioSites.InstalledV2ScenariosResolve`: every name from `names()` starting with `v2_` resolves on the installed share with `site == "construction_site_v2"` and `start_free == true`. Run: FAIL.
- [ ] **Step 2: Write the scenarios.** Run the test. Expected: PASS. Also `scenario_spec --list` contains V1 and V2 names.
- [ ] **Step 3: Plan checks.** `caffeinate -i python3 tools/check_site_plans.py`. Expected: no mismatches. Where the chicane plans with no heading change above 30 degrees, tighten its gaps (Task 10 layout) and re-export (Task 11) before committing.
- [ ] **Step 4: Commit** `git commit -m "Add the V2 scenario set with plan expectations"`; `unity command editor_stop`.

### Task 14: Smoke runs and real-time factor

**Files:**
- Create: `docs/experiments/site-v2/smoke-runs.md` (table), run directories referenced by name only

**Interfaces:**
- Consumes: everything above.

- [ ] **Step 1: Real-time factor pair.** `caffeinate -i python3 tools/run_nav_scenario.py open_space_nav --new-epoch` (V1) and `v2_open_yard_nav` (B1 = `--controller rpp`). Expected: both summaries have `real_time_factor`; V2 >= 1.00. If V2 < 1.00: bake lighting (HDRP baked GI on the static pieces), re-measure; if still low, report to the user before going on.
- [ ] **Step 2: Smoke runs,** one each with `--controller rpp` and `--controller wholebody` (current defaults on `main`): `v2_scaffold_tunnel_nav`, `v2_door_105_nav`, `v2_chicane_nav`, `v2_spine_rough_nav`. Expected: the runner completes each run (success or a labelled expected failure), no contact on B5 rows, the scene switch from V1 happened once, `v2_spine_rough_nav` shows `disturbances: true` and no costmap marks on the plates. Record MPC timing p99 on the tunnel run and the MoveIt box count from `/tmp/mm_moveit.log` on any mission start.
- [ ] **Step 3: Write `smoke-runs.md`** (scenario, controller, outcome, contact, real-time factor, MPC p99) and commit `git commit -m "Record the V2 smoke runs and real-time factor"`. `unity command editor_stop`.

### Task 15: Documentation and ADR

**Files:**
- Create: `docs/experiments/site-v2/README.md`, `docs/adr/0012-site-selection-and-box-proxies.md`
- Modify: `docs/roadmap.md` (comparability log row, site overhaul entry, open items of spec Section 11), `docs/running-the-stack.md` (`site` argument, `site_open`, `site_active`, `site_disturbances`, rebuilding the kit with Blender), `ros2_ws/src/mobile_manipulator_navigation/docs/unity-map-export.md` (`--site`)

- [ ] **Step 1: Write the docs.** README: zones with screenshots, kit and proxy rule, Tier 2, overhead heights with the computed tops, real-time factor before and after, smoke-run table link, plan expectations. ADR 0012 in the format of ADR 0011: context, decision (scenario files name the site; one `site` argument; box proxies are the only colliders), consequences, revisit conditions. Comparability-log row: V2 site, map 50 x 40 m, new scenario set, disturbances; provisional until the final baseline run.
- [ ] **Step 2: Check** every path and command in the docs exists (`ls`, `--help`).
- [ ] **Step 3: Commit** `git commit -m "Document site V2, site selection and the kit"`.
