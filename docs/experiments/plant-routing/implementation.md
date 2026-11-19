# Plant routing implementation record

Branch: `plant-routing-unity-viewer`. Design approved 2026-11-18. Work is in progress; this record
separates implemented/checked foundations from later integration and performance claims.

[The 2026-11-19 handoff](handoff-2026-11-19.md) is the pause checkpoint before Task 7's completion; the section
"Task 7 integration (2026-11-19)" below supersedes its integration status.

## Frozen initial contracts

- The core uses plain C++17 values and no ROS/Unity/URDF/bag types. The campaign adapter resolves robot geometry.
- The immutable grid has separate known/observed occupancy, 0.05 m resolution for V2 and normalized soft inflation costs. Unknown/outside is blocked.
- Primitive poses have relative XY and absolute heading angles; the explicit origin is retained. Headings follow the actual nonuniform array.
- Motion validity covers the declared piecewise SE(2) interpolation. Each subinterval bounds any footprint-point displacement by translation plus footprint radius times angular travel; occupied cell rectangles are conservatively expanded by that bound.
- Maximum displacement per interpolation subinterval is initially 0.0125 m. This is a conservative geometric construction, not a measured tracking guarantee.
- Search uses 256 node expansions and 524,288 cell tests per default slice (cell budget raised 2026-11-19 from 16,384; see below), with resumable collision cursors, 1,000,000 total expansions and a 256 MiB routing-owned storage budget. These are work limits, not wall-time guarantees.
- Start/goal connectors use checked rotate/forward-translate/rotate motions in a deterministic two-cell neighbourhood. `no_path` is relative to this connector/lattice policy.
- Requested goal coordinates/yaw are preserved. The existing controller completion tolerances remain separate from search endpoint selection.
- Heading-only output remains a stationary pose path; the hierarchy's explicit zero-length route support is implemented and checked separately from search.
- Generated-goal timing uses one session clock and acceptance-time deadlines. Active-route updates and activation after completion/failure are distinct operations.
- Goal/request IDs include session and snapshot revisions. Superseded results cannot be adopted. Goal and Route schema-3 records are planned as types 20 and 21; historical schema readers stay separate.

## Provisioned motion input

The source is the installed `ros-jazzy-nav2-smac-planner` version `1.3.13-1noble.20260921.071143`;
relative path `sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json`.
Its 71,540 bytes are retained unchanged under the new package's `data/diff_5cm.json`, with installed
copyright metadata and the full Apache-2.0 licence. SHA-256:
`105b37351d15e340cc2921fa8878c8ea727a65cd7e89c39c977be0654be3b2a9`.
The parser checks 16 headings, 112 motions and 32 pure rotations in a pinned-data test.

## Completed checks

The first standalone configure/build ran in an isolated Linux arm64 container build directory with
`AMENT_PREFIX_PATH`, `COLCON_PREFIX_PATH` and `CMAKE_PREFIX_PATH` unset. Dependencies resolved from system
yaml-cpp, nlohmann JSON 3.11.3 and GTest 1.14.0; a ROS environment was not sourced.

Four test executables passed after the initial collision-boundary correction: grid (5 cases), map I/O
(7), primitive parsing (9), and footprint/motion checks (12). They cover asymmetric PGM orientation,
binary whitespace/CRLF handling, invalid map metadata, independent layers, exact Euclidean inflation,
nonuniform/wrapped headings, malformed primitives, footprint interiors, rotating sweeps and resumable checks.

A separate consumer configured against the installed package with ROS paths unset, linked and ran.
Its dynamic dependencies were the system C/C++ runtime libraries, with no ROS libraries. This checks the
installed grid interface; search and full hierarchy are still separate pending checks.

## Correction found before search integration

At 0.05 m resolution, binary floating-point division could exclude a cell numerically touching a footprint
boundary before SAT intersection was considered. The enumerated envelope now includes a coordinate-scale
numerical allowance. A regression tests the exact `0.25 + 0.1` versus `7 * 0.05` boundary and a clear
1 mm separation. This correction is conservative and does not expand the envelope by an entire cell.

## Task 7 integration (2026-11-19)

Implemented: the persistent executor's `adopt_route` (same-goal update or new-goal rearm on one continuing clock;
an absolute goal-manager deadline), the solver-free `step_holding` tick (base braked at the envelope's limits from
the command sent last, arm velocities ramped to zero as B5's own braking hold; status Holding, so the realistic arm
path sends its one checked braking hold) and the `command_guard` hook (a refusal or an exception sends a separate
Blocked hold, never a zeroed base under an Ok/Fallback status; B5's history records the sent command). The campaign
build links the routing package, the goal manager and the routing adapter. The routing package's CMake config
exports its installed data directory; the adapter receives it at compile time instead of an ament-index lookup (the
package is plain CMake and not in the ament index).

Correction to the routing core (plan Task 3's unchecked swept-mask item): the sampled check re-enumerated the whole
footprint envelope (~400 cells) at every 1.25 cm sample, ~134,000 cell tests per expansion with the V2 base
footprint. Lattice edges now use a per-primitive swept mask (the same sampling, dilation and contact test, as cell
offsets from the start cell's centre); connectors keep the sampled check. Measured on the V2 map from the open-yard
start (container, arm64, one run each, wall times include building the masks on first use):

| Goal | Expansions | Cell tests before | Cell tests after | Wall before | Wall after |
|---|---|---|---|---|---|
| 0.5 m ahead | 9,996 | 1,031,436,544 | 17,229,693 | 2.88 s | 0.24 s |
| 1.0 m ahead | 164 | 29,145,946 | 571,630 | 0.077 s | 0.069 s |
| 2.0 m ahead | 1,560 | 209,625,960 | 3,575,069 | 0.58 s | 0.10 s |

Routes, costs and expansion counts were unchanged. A test compares the masked and sampled checks for all 112
pinned primitives from six random starts each on six random maps (4,032 cases; unknown cells and the map exterior included): no
disagreement. The default cell budget per tick was raised to 524,288, about one slice of operations at the measured
~2,300 cell tests per expansion; this is an engineering limit, not a wall-time guarantee.

Checks run (isolated overlay `/tmp/mm-routing-integration`, container): routing CTest 5/5; MPC and medium full
suites 618 tests, 0 failures (5 skipped); medium closed-loop executor tests 13/13 (6 new: holding after
completion, a new goal's own timeout after idle, an absolute deadline, the hold's braking and a same-goal rearm, the
guard); campaign goal manager 22/22, routing adapter 3/3, generated session 8/8 (idle clock and a scheduled goal,
replacement while moving, a second goal after idle past the first deadline, an invalid replacement, the scenario
goal at reset, base and arm at rest while a replacement is planned on the realistic plant, reset during a search).

Open items found:

- **Heuristic for reorientation goals (first part fixed 2026-11-19, user-approved).** The Euclidean heuristic with
  in-place rotation at 2.5 m/rad left goals that need a turn to a near-Dijkstra search (1.5 m behind: more than
  40,000 expansions, no route within a 30 s deadline). The heuristic is now max(Euclidean, a free-space lattice
  table): per goal heading bin, a backward Dijkstra over the primitives without obstacles or soft cost within 80 cells
  of the goal cell, from the goal-edge states at a lower bound of their connector cost; paths leaving the window are
  bounded by their out-and-back travel; an off-bin goal yaw subtracts its rotation cost. It is a lower bound (no
  weighting), so optimal costs are unchanged: tests compare four reorientation cases with the pinned bank against the
  independent Dijkstra, and the bound against found costs from 40 random states. Tables are built on first use per bin
  (about 1.7 MB each) and shared by a session's searches. Open-yard start, default budgets, planning hold (goal
  accepted to route accepted):

  | Goal | Expansions before | Expansions after | Hold after | Route length |
  |---|---|---|---|---|
  | 1.5 m ahead | 2,045 | 310 | 0.02 s | 1.56 m |
  | 3.0 m ahead | 3,436 | 1,754 | 0.14 s | 3.02 m |
  | 1.5 m behind | > 40,000 (none in 30 s) | 6,432 | 0.52 s | 4.44 m |
  | turn around in place | not measured | 8,111 | 0.68 s | 4.09 m |
  | 8 m ahead, 3 m left, yaw +1 rad | not measured | 36,586 | 2.90 s | 8.85 m |

  With rotation at 2.5 m/rad, the cheapest turn-around is a 4 m loop, not an in-place spin: a cost-model question.

- **Long routes on V2 (open).** Each V2 scenario's own destination from its start (one run each, simulation-time
  hold at 256 expansions per tick; the wall time to plan was 0.4-5.3 s): holds 0.8-57 s (spine, 35 m: 684,205
  expansions, 53.9 s; services dead end: 727,586, 56.9 s; store room: 458,892, 35.9 s). `v2_chicane_nav` needs
  782,729 expansions (more than 60 s); `v2_yard_store_nav` exceeds the 1,000,000-expansion limit. Beyond the 4 m
  table the heuristic is Euclidean, and it ignores walls and the soft inflation cost; an obstacle- and cost-aware term
  is the next step (an admissibility decision for the user).

- **The slice budget's unit (fixed 2026-11-19).** `WorkBudget::expansions` counted every search operation (each
  successor, seed and stale heap entry; about 28 expansions per 256). It now counts node expansions, as the plan
  states; every other operation costs one cell unit, so a slice stays bounded. The session shares one swept-mask
  bank across its searches (masks depend on footprint, primitives, step and grid geometry, not occupancy). Planning
  hold in the generated session, goal accepted to route accepted, open-yard start, default budgets:

  | Goal ahead | Expansions | Hold before | Hold after |
  |---|---|---|---|
  | 1.0 m | 164 | not measured | 0.02 s |
  | 1.5 m | 2,045 | 1.64 s | 0.18 s |
  | 2.0 m | 1,560 | not measured | 0.12 s |
  | 3.0 m | 3,436 | not measured | 0.28 s |

- Same-goal replans (`request_replan`) hold the plant until the new route is adopted.

## Pending evidence

Independent path validation on V2, campaign/demo drivers (Task 8), goal protocol, acquisition-time mapping, observed-base guard, Nav2 comparison,
resume fingerprints and viewer behavior have not yet been validated. No FPS or real-time planning claim
is established by the foundation build. Timed integration uses bounded workloads and conservative build
parallelism; it does not share ROS workspace build output with separate diagnostic worktrees.

## Smac Lattice port (spec 2 Amendment 1, from 2026-11-19)

The user ruled on 2026-11-19 that the route search should be Smac Lattice itself, ported to C++
([ADR 0019](../../adr/0019-route-search-ports-smac-lattice.md); plan tasks S0-S6).

### S0: provenance

The port follows these upstream files. SHA-256 prefixes (16 hex digits) were taken from the clones of
`navigation2` tag `1.3.13` (commit `f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501`) and OMPL tag `1.7.0` (commit
`eabb4d0f825666e4553a2ec0ecb9c7a5a33637dd`). The container's installed `nav2_smac_planner` headers equal the tag's.

| File | SHA-256 prefix |
|---|---|
| nav2_smac_planner/src/a_star.cpp | 5e33a489ff3eb77f |
| nav2_smac_planner/src/node_lattice.cpp | 0e7431d8e65b20a8 |
| nav2_smac_planner/src/node_hybrid.cpp | e398942dc085e8d0 |
| nav2_smac_planner/src/analytic_expansion.cpp | 47b90b88223825ce |
| nav2_smac_planner/src/collision_checker.cpp | de383f6e993ffdf1 |
| nav2_smac_planner/src/smoother.cpp | eb971009c1ceebf2 |
| nav2_smac_planner/src/smac_planner_lattice.cpp | 21510b24f2b60b06 |
| nav2_smac_planner/include/nav2_smac_planner/node_lattice.hpp | 077b55225f6c1069 |
| nav2_smac_planner/include/nav2_smac_planner/types.hpp | eb28aeda350c7f1e |
| nav2_smac_planner/include/nav2_smac_planner/utils.hpp | 1dcfc6d72a9417fd |
| nav2_smac_planner/include/nav2_smac_planner/constants.hpp | 58b128c745193c69 |
| nav2_smac_planner/include/nav2_smac_planner/smoother.hpp | 09305a27fa661285 |
| nav2_costmap_2d/src/costmap_2d.cpp | ce221510cc609a43 |
| nav2_costmap_2d/src/footprint.cpp | 8631554f4471ee52 |
| nav2_costmap_2d/src/costmap_math.cpp | f72b73284c515d9a |
| nav2_costmap_2d/src/footprint_collision_checker.cpp | 9e71e825544f7196 |
| nav2_costmap_2d/plugins/static_layer.cpp | 81606bf320d68290 |
| nav2_costmap_2d/plugins/inflation_layer.cpp | b2271d598f5b4732 |
| nav2_costmap_2d/include/nav2_costmap_2d/inflation_layer.hpp | af717d346c297296 |
| nav2_util/include/nav2_util/line_iterator.hpp | 9c81a10c4f52f67c |
| ompl/base/spaces/src/DubinsStateSpace.cpp | 1c336f591106eabb |
| ompl/base/spaces/DubinsStateSpace.h | b54ee94009db5d59 |

The licences and attributions are in the routing package's `third_party/NOTICE.md`.

Reading the source showed several behaviors that the port must keep:

- **No downsampling in Lattice's obstacle heuristic.** Lattice calls `NodeHybrid`'s obstacle heuristic but never
  initializes Hybrid's motion table. That table is a static object, so `downsample_obstacle_heuristic` and
  `use_quadratic_cost_penalty` are zero-initialized (false). The heuristic therefore runs at full resolution with a
  linear cost penalty.
- **Up to half a cell between checks and output.** `mapToWorld` takes whole cells, so the collision checker centres
  the footprint on the centre of the truncated cell: the origin plus (floor(m) + 0.5) times the resolution. The output
  path is converted with the origin plus m times the resolution, from the continuous coordinate. The checked footprint
  and the returned pose therefore differ by up to half a cell on each axis. The validation pass of A1.2 checks the
  returned path.
- **Per-node validity cache.** A node's validity is cached on its first check. A node later reached by another
  primitive is not checked again.
- **The resolution is a 32-bit float.** The costmap's resolution comes from the map message's float field: 0.05 becomes
  0.0500000007. Every Smac quantity derived from the resolution inherits this, including the inscribed-radius
  comparison of the inflation layer. Found by S1's first run: 12,867 cells were 253 instead of 251 with 0.05 exactly.
- **Unknown cells are free.** With `track_unknown_space` false, the static layer maps unknown cells to free. The
  earlier routing core treated unknown cells as blocked.

### S1: costmap semantics

The routing core's `smac::build_costmap` ports these parts:
- the static layer's interpretation;
- `padFootprint`;
- the inscribed and circumscribed radii;
- the inflation layer, with its caches, distance bins and spreading order.

The container-only package `mobile_manipulator_routing_nav2_check` compares the port with `nav2_costmap_2d`'s own
`LayeredCostmap`, which runs the `StaticLayer` and `InflationLayer` with the reference parameters.

| Case | Result |
|---|---|
| V2 map, `home` footprint and an irregular 16-vertex footprint | identical bytes |
| three seeded 240 x 180 box maps with unknown cells, both footprints | identical bytes |
| inscribed and circumscribed radii, both footprints | exactly equal |

### S2: collision checker

The routing core ports three parts:
- `LineIterator`;
- `FootprintCollisionChecker`, with costs along the footprint's edges only;
- Smac's `GridCollisionChecker`, with 72 orientation bins, the centre-cost shortcut below the circumscribed cost, and
  the float bin index truncated as upstream.

The circumscribed cost uses the inflation function at the circumscribed radius, as `findCircumscribedCost` does.

| Case | Result |
|---|---|
| 200,000 seeded poses on the V2 costmap, two footprints, unknown traversal on and off: `inCollision` and centre cost against `nav2_smac_planner::GridCollisionChecker` | all equal (the full-footprint branch was reached more than 1,000 times) |
| 100,000 seeded world poses: `footprintCostAtPose` against `nav2_costmap_2d::FootprintCollisionChecker` | all equal |

Upstream reads the centre cell at (x + 0.5, y + 0.5) after checking x and y only against the map size. Coordinates
in the last half cell before the size therefore read outside the map. The test samples avoid that band. The port keeps
this upstream behavior unchanged; S5 records whether any search reaches that band.

### S3: Dubins curves and the lattice planner port

The routing core now has the complete port under `smac/`:
- `dubins`: OMPL's classification code, copied unchanged;
- `node_lattice`: the motion table, `NodeLattice`, the distance table and the obstacle heuristic;
- `a_star`: `AStarAlgorithm` and `AnalyticExpansion`;
- `smoother`;
- `planner`: `configure` and `createPlan`.

Upstream's static state lives in one `LatticeContext` per planner.

Two upstream details were checked empirically against the installed headers, because they change results:

- **The math overloads Smac calls.** Unqualified `cos`, `sin`, `round` and `abs` on floats resolve to the float
  overloads in Smac's translation units, so its trig table is float precision. The port uses the same overloads. The
  smoother's `abs` on doubles is the double overload, so its convergence test is not truncated.
- **Floating-point contraction.** The aarch64 OMPL binary fuses multiply-adds (fmadd) in its distance code. It does not
  fuse in its interpolation, except for the two final `x * radius + from.x` lines. The port keeps the compiler's
  default contraction, which matches the installed Nav2 and OMPL builds in the container. It compiles the
  interpolation without contraction and writes those two lines as `std::fma`.

| Check (container) | Result |
|---|---|
| 200,000 seeded Dubins queries at the radii Smac uses (9.99999985 and 10.5 to 25 cells; 0.5 m): distance and interpolation against `ompl::base::DubinsStateSpace` | bit-identical |
| first try, interpolation compiled with contraction | differed in the 15th digit (4.5e-15) |

Equality outside the container is not claimed (A1.2 item 5): x86-64 builds do not fuse, and Apple Clang fuses within
expressions.

### S5: end-to-end equivalence with SmacPlannerLattice

`test_planner_equivalence` (container-only package) has this setup:
- It configures the installed `nav2_smac_planner::SmacPlannerLattice` from a `Costmap2DROS`, with the static and
  inflation layers and the reference footprint settings.
- The costmap's master grid holds the port's bytes, which S1 showed are Nav2's own.
- Both planners use the reference `Lattice` parameters and the same request order. Smac's 5 s wall-time limit is
  raised to 1,000 s, so only its iteration limit applies (A1.2 item 1).
- The port runs in slices of 37 iterations.

It compares the outcome, the smoothed plan and the raw plan (Smac's `unsmoothed_plan` topic). Positions must agree
within 1e-4 m and yaw within 1e-4 rad.

| Corpus | Cases | Found | Other outcomes | Mismatches |
|---|---|---|---|---|
| `home` footprint: 38 V2 scenario pairs, 200 seeded random free pairs (one in five within 1.5 m), 6 edge cases | 244 | 161 (raw compared in all 161) | 47 iteration limit, 18 no path, 15 start occupied, 2 goal outside, 1 start outside | 0 |
| irregular 16-vertex footprint: 38 scenario pairs, 60 random pairs | 98 | 95 (raw compared in all) | 3 iteration limit | 0 |

Smac's iteration count is not observable through `createPlan`. Outcomes that depend on it (start occupied after one
iteration, the 1,000,000-iteration limit) agree in every case.

The scenario doors and rooms fail with the 1.24 m `home` footprint in both planners, as the scenario file expects for
that profile.

Wall time in the container is similar for both planners. The `home` corpus took 103 s for Smac and 115 s for the
port. Most of that time is in the 47 cases that ran to the iteration limit, at about 2 s each.

Iterations per V2 scenario destination (`home` footprint) against the old search:

| Destination | Iterations | Old search expansions |
|---|---|---|
| `spine` | 231 | 684,205 |
| `yard_store` | 5,432 | more than 1,000,000 |
| `scaffold_tunnel` | 75,159 | not recorded |
| `chicane` | 520,212 | 782,729 |
| `services_deadend` | 204,239 | not recorded |

### S4: integration

The integration changes these parts:

- **`Search`** is an adapter over a session's `RoutePlanner`. That object holds:
  - the costmap built once from the grid snapshot;
  - the grid with unknown cells free, for the validation pass;
  - one `SmacPlannerLattice`.

  One call runs `expansions` Smac iterations. When Smac finds a route, the validation pass checks the smoothed plan
  with the swept footprint check, `cell_tests` per call. A failure falls back to the raw plan. If both fail, the new
  outcome is `rejected`. Smac's exceptions map to the existing outcomes.
- **`RoutingInputs`** now carries three more things:
  - the global costmap's footprint (the projected hull with the 0.02 m geometric padding; Smac adds the 0.01 m
    costmap padding itself);
  - the inflation settings;
  - the `Lattice` block read from `nav2_global_planning.yaml`.

  The validation pass keeps the earlier routing footprint, which has both allowances.
- **The session** builds one `RoutePlanner` at reset. The `RouteAccepted` event records the planner, whether the plan
  was smoothed, the resolved goal and the route length. The old fields `lattice_cost` and `connector_policy` are gone.
- **Removed:** `FreeSpaceHeuristic`, the old A* and `SweptMaskBank`. The swept masks of `FootprintChecker` remain,
  with their tests.
- **Tools:**
  - `route_check` and `tools/plant/qualify_routing.py` follow Smac's semantics: start position preserved, end within
    0.20 m plus half a cell, no `cost`. The `occupied_goal` fixture now expects `no_path`, because every pose within
    the tolerance lies in the box's inflated ring.
  - `config/routing.yaml` lists only the budgets.

Results:
- routing tests: 7/7 (search);
- generated session: 8/8;
- routing inputs: 3/3;
- qualification fixtures: 8/8;
- `test_qualify_routing.py`: 14/14.

Two generated-session tests needed a search that spans many ticks. They now use a goal behind the robot, because a
goal ahead is found within a few iterations by the analytic expansion.

One build pitfall: configuring the routing package with `-DROUTING_WITH_AMENT=ON` lets ament overwrite its CMake
config. The data directory then comes out empty and the session cannot find the primitives. The integration build
uses the default (off).

The full suites after S4 pass:
- the campaign package: 12/12 test targets, 739 s;
- the routing package: 5/5 targets.

### S6: V2 destinations with the plant's routing inputs

`test_v2_routes` (campaign package) plans every V2 scenario start/destination pair with the session's inputs:
- the projected base footprint;
- the global costmap's padding and inflation;
- the `Lattice` block.

It runs at the default budget of 256 iterations and 524,288 validation cell tests per 20 ms tick. A route that Smac
finds but the validation pass rejects fails the test.

| Result | Value |
|---|---|
| destinations | 38 |
| found | 38 |
| rejected by the validation pass | 0 |
| smoothed plan used | 38 |
| end distance from the requested goal | at most 7.6e-7 m |

The smoother's end-boundary step and the analytic expansion bring the path onto the goal. The 0.20 m tolerance was not
needed on these cases.

| Destination | Iterations | Planning hold at 256/tick (simulation) | Wall | Old search hold (2026-11-19) |
|---|---|---|---|---|
| `open_yard` | 62 | 0.04 s | 6 ms | 4.50 s |
| `door_130` | 113 | 0.04 s | 5 ms | 6.74 s |
| `room_r1` | 839 | 0.10 s | 6 ms | 19.66 s |
| `room_r4` | 5,060 | 0.42 s | 13 ms | not recorded |
| `yard_store` | 19,210 | 1.56 s | 30 ms | budget exhausted (1,000,000) |
| `chicane` | 62,066 | 4.90 s | 80 ms | more than 60 s |
| `scaffold_tunnel` | 83,444 | 6.54 s | 88 ms | 24.46 s |
| `store_room` | 120,943 | 9.48 s | 138 ms | 35.88 s |
| `services_deadend` | 187,025 | 14.66 s | 235 ms | 56.92 s |
| `spine` (35 m) | 231 | 0.06 s | 7 ms | 53.90 s |

The other 28 pairs take 33 to 192 iterations and at most 0.04 s.

The wall time is about 1.3 µs per iteration in the container. The planning hold therefore comes from the per-tick
budget, not from computation. The budget is an open decision for the user:
- At 4,096 iterations per tick, the longest hold (`services_deadend`) would be about 0.92 s.
- That budget costs about 5 ms of wall time per planning tick at the measured rate.

**Per-tick budget raised to 4,096 iterations (user, 2026-11-19).** At the new default, the V2 holds are:

| Destination | Ticks | Hold |
|---|---|---|
| `services_deadend` | 48 | 0.96 s |
| `store_room` | 31 | 0.62 s |
| `scaffold_tunnel` | 22 | 0.44 s |
| `chicane` | 18 | 0.36 s |
| `yard_store` | 7 | 0.14 s |
| the other 33 | 1-2 | at most 0.04 s |

`test_v2_routes`, the generated session (8/8) and routing (5/5) pass. The two hold tests keep one iteration per tick
on purpose.
