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

  The validation pass keeps the earlier routing footprint, which has both allowances (until Amendment 3, 2026-11-19:
  geometric padding only).
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

## Task 7 review fixes (2026-11-19)

The review of Task 7 (`4d9a16f..eb5eb20`) reported six important findings and nothing critical. Five were fixed; I5
no longer applies.

| Finding | Resolution | Regression |
|---|---|---|
| I1: in ideal mode, the hold's arm stop was not collision-checked | The executor picks its hold's arm stop with `mmm::safe_arm_hold`, as the realistic arm path and the node do: a faster stop near obstacles, or the emergency Hermite. Each tick sends that stop's velocity, in both modes. | `TheHoldsArmStopIsCheckedAgainstTheObstacles`: pan at 0.5 rad/s, box 0.10 m from the panel; fails with the old nominal stop |
| I2: hold ticks recorded the nominal ramp in B5's delay history | `step_holding()` corrects B5's previous interval to `previous_arm_velocity()` before appending, as ordinary ticks do. Adds `Controller::sent_history()` | `HoldTicksRecordTheStreamedArmReferenceInTheDelayHistory`; fails without the fix |
| I3: contacts during planning and idle holds were missing from the summary | `RunSafety` counts every physics tick. A generated run's summary reports it (`safety_scope: every_tick`); goal records stay tracking-only | `ContactsDuringAPlanningHoldReachTheRunSummary` |
| I4: a guard-refused proposal counted as an executed slacked plan | Status and slack come from the sent output | `AGuardRefusalReportsTheSentStatusAndNoExecutedSlack`, plus an end-status check in the refusal test |
| I5: swept-mask construction bypassed the work and memory limits | Not applicable: S4 removed that search path | none |
| I6: superseded or expired goals archived incomplete summaries | `ClosedLoopStepper::snapshot()`; the session archives and reports live goals from it | `ASnapshotMaterializesTheLiveSummariesWithoutEndingTheGoal` |

`ClosedLoopOptions::initial_command` (the command in effect when a stepper starts) was added so the I1 fixture can
seed a moving arm without private access.

## Task 8: headless goal sources and live controls (2026-11-19)

The headless goal path has three parts:

- **Goal script** (`goal_script.{hpp,cpp}`). Bounded JSON: at most 1,000 goals, 128 of them timed, ids unique, at
  most 1 MiB. Each goal has a target, an optional timeout and exactly one trigger:
  - `at_s`: goes into the goal manager's schedule, so a goal due while another runs replaces it;
  - `after: "previous"`: submitted at the first boundary with nothing active or scheduled.

  The script may also set `time_limit_s` and `end: idle|time_limit`. `run_goal_script` runs a generated session by
  the script, one `Session::advance()` per tick, and returns each submission's acknowledgement.
- **`plant_campaign --routes recorded|lattice`** with `--goals FILE` and `--sim-time-limit S`. Lattice mode refuses
  `--bag`; without a script it submits the scenario's destination. The summary records `routes`, the script's
  submissions and its SHA-256. `tools/plant/plant_campaign.py run --routes lattice` passes the mode to every run:
  full hierarchy only, a cell's optional `goals` file, and `-lattice` in the run names.
- **Demo server.** Clients send `submit_goal`, `schedule_goal` and `request_replan` through the goal manager, and get
  `goal_ack` events back. `set_goal` is refused in lattice mode, and the goal commands in recorded mode.
  `plant_demo_server --routes lattice` serves a generated session without the scenario's destination; it keeps
  stepping while idle. A paced server that falls behind keeps the timestep and sends a `behind` event with the
  achieved speed, at most once a second. A disconnect still pauses the run (the demo's policy).

| Check | Result |
|---|---|
| goal-script parsing: valid script and 11 malformed ones | pass |
| a goal scheduled at t = 5 while idle is reached | pass |
| two `after: "previous"` goals run one after the other | pass |
| a replacement at t = 3 supersedes the moving goal; an outside-map goal is refused at submission; the run still ends idle | pass |
| a time limit ends an idle run (3 s); a recorded session is refused | pass |
| demo: lattice goal accepted and reached, the session stepping on while idle | pass |
| demo: goal commands refused without lattice mode | pass |
| demo: paced (8x) and unpaced runs reach the goal at the same simulation time | pass |
| `test_plant_campaign.py` | 31/31 |

The session's existing completion rule applies unchanged: the current destination decides `reached`. A submission
refused at the goal manager never becomes current. Its refusal is visible in the script's submissions.

**Open: a turn-around from rest stalls.** A smoke run of `plant_campaign --routes lattice` on the open yard had two
goals:
- the scenario's destination at t = 2 s: reached at 38.8 s;
- `after: "previous"`, back to the start facing the other way (about 10 m behind the robot): stalled. The robot did
  not leave the first destination; B5 followed medium plans for 46 s without progress until the 45 s stall window
  ended the goal.

The route came from the Smac port, and planning and adoption worked. The stall is in executing a route that starts
behind the robot. This belongs to the hierarchy's execution, the E3 passage-stall diagnosis family, not to routing.
It matters for the click-to-goal demo and is raised with the user.

**Turn-around fixed (2026-11-19).** The Nav2-era fix (Phase 4) was in B5: it starts an in-place turn from rest
through the breakaway lift. The test `Controller.TurnsInPlaceFromRestToGoalBehind` and the 2026-10-06 probe confirm
it. That fix still works. An executor probe (route 4 m behind, robot at rest) gave:

| Executor | Result |
|---|---|
| B5 alone | reached in 21.8 s |
| with F2 | 60 s at rest, following F2's plans that never turn |

F2's Bézier plan cannot start with a turn on the spot; Phase 5A covered turns on the spot only at the goal. Recorded
Nav2 routes were built the same way: `Route` keeps distinct XY points only, so Smac's in-place rotation was dropped
there too. No V2 scenario needs a turn-around.

The fix stays in the session and leaves B5 and F2 unchanged, so the frozen baselines are unaffected. When an adopted
route's first direction of travel is more than 0.785 rad from the base's heading, the goal runs as two legs:
- a stationary turn onto that direction (Task 5's heading-only route, where B5 turns in place);
- then the drive (`drive_leg_started` event).

0.785 rad is the threshold the Nav2 baselines used to turn in place first: RPP's `rotate_to_heading_min_angle` in
`nav2_controllers.yaml`.

| Check | Result |
|---|---|
| `GeneratedSession.AGoalBehindTheRobotTurnsInPlaceFirst` | pass; generated session 10/10 |
| the two-goal smoke script, ideal | both goals reached (38.8 s, 83.0 s) |
| the two-goal smoke script, realistic | both goals reached (84.0 s) |

## Task 9: schema 3 goal and route records, resume integrity (2026-11-19)

Schema 3 is the protocol's new version; `stream-protocol.md` documents it.

- **Goal frame (type 20).** One per goal-manager event of a generated-route session, written beside its Event frame.
  It carries the event, its sequence numbers, epochs, the client and request, `scheduled_for`, and the goal: IDs,
  state, target, timeout, timing and revisions.
- **Route frame (type 21).** One per adopted route. The header carries the planner, path kind, `smoothed`,
  `turn_first`, length, requested and resolved goals, and the work counts. Blocks: the adopted path (x, y, yaw) and
  the hierarchy route (x, y).
- **Capabilities.** The session Hello lists the optional record types it writes: `["goal_frames", "route_frames"]` in
  generated mode, `[]` otherwise. State, Plans and Scan layouts are unchanged.
- **Builders.** `goal_records.{hpp,cpp}` builds both records as pure functions. The session and the fixture test use
  them.
- **Fingerprints.**
  - The routing metadata already hashed the map YAML and image, the primitives, the URDF and the two configs, plus the
    footprint. It now also records the effective `lattice` and `costmap` settings and the costmap footprint.
  - Generated sessions put the metadata in the Hello and in the summary, which also has the goal script's SHA-256
    (Task 8).
  - `tools/plant/plant_campaign.py`'s resume treats a lattice run as stale when its goal script, or any installed
    routing input, has other bytes than its summary records.
- **Python.** `plant_stream.py` gains `SCHEMA = 3`, reads logs of schemas 1-3, adds the `GOAL` and `ROUTE` kinds, and
  adds `route_path()`. `view_run.py` draws the last adopted route and prints Route frames. Its route drawing could not
  be run here: neither the container's Python nor the host's has the `mujoco` module, and that suite's viewer tests
  are skipped for the same reason.

| Check | Result |
|---|---|
| C++ golden records against committed `goal_v3.bin` and `route_v3.bin` (`Frames.GoalAndRouteRecordsMatchTheCommittedFixtures`) | pass |
| Python decodes the same fixtures and re-encodes them byte for byte (`test_schema3_goal_and_route_records_match_cpp`) | pass |
| `frame_v1.bin` unchanged; schema 1, 2 and 3 logs readable, schema 4 refused | pass |
| a generated session writes Goal (accepted, reached) and Route frames with consistent block sizes (`GeneratedRecords.AGeneratedSessionWritesGoalAndRouteFrames`) | pass |
| two identical `plant_campaign --routes lattice` runs write byte-identical logs | pass |
| resume: a goal script edited at the same path, then a reinstalled global planning config, each make the run stale | pass |
| plant ctest | 12/12 |
| campaign ctest | 14/14 |
| `tools/plant` Python suite | 347 passed, 2 skipped (the viewer's mujoco tests) |

Malformed and truncated input are covered by the existing decoder and run-log tests, which now also accept the two
new types. Simulated log-write failure is deferred (plan Task 9, revised).

## Task 8 review fixes (2026-11-19)

The review of Task 8 (`11da9ae..36729a7`) reported five important findings and nothing critical. All five are fixed.

| Finding | Resolution | Regression |
|---|---|---|
| I1: the emergency arm hold sampled its velocity per tick and drifted off the checked Hermite (1.9 mm into a corner box) | Each hold tick sends the interval's average velocity of `safe_arm_hold`'s position profile: the parabola within the joint limits, or the emergency Hermite. The executed joints are on the checked stop at every tick. | `TheEmergencyHoldExecutesItsCheckedStop`: the reviewer's forward wall plus corner box, every emergency case; fails with the per-tick derivative |
| I2: timed goals beyond one client's 64 receipts were dropped as backpressure | A script holds at most 64 timed goals (`GoalManagerLimits::receipts_per_client`; the schedule holds 128). An `after` submission refused for backpressure is retried at the next boundary, not recorded as handed over. | `TheMaximumNumberOfTimedGoalsIsAllScheduled` (64 of 64 Scheduled); the parser refuses 65 |
| I3: resume ignored a goal script edited at the same path | Fixed by Task 9 (`602b9e0`): resume compares the script's SHA-256, and a missing script hashes to "" and is stale | `test_resume_reruns_a_lattice_run_whose_goal_script_or_routing_inputs_changed` |
| I4: a scripted run's success was the current destination's | `run_goal_script` reports each script goal's outcome: reached, superseded, failed, timeout, canceled, refused, pending or unsubmitted. The script succeeds when every goal is reached or superseded and the last one handed over is reached. `plant_campaign` writes `script_success`, and `plant_campaign.py`'s tables count scripted runs by it. The session's `reached` keeps its current-destination meaning; goal records gain `client` and `client_request` (live rows also `state`) | outcome assertions in the replacement and refusal test; `AGoalStillScheduledAtTheTimeLimitIsPendingAndTheScriptFails`; `test_a_scripted_run_succeeds_by_its_script` |
| I5: a fractional `epoch` was truncated and could replace the active goal | `submit_goal` and `schedule_goal` require an unsigned-integer `epoch` and a finite numeric `at_s` | `AFractionalEpochIsRefusedAndTheActiveGoalStays` |

The review also listed coverage gaps. Of those, the maximum-capacity test, the same-path script resume and the
malformed-epoch test are now covered. These remain:
- a scripted replacement while a search is pending;
- an accepted goal with no route;
- mixed timed and `after` chains;
- TCP `schedule_goal`/`request_replan` round trips;
- the `behind` event's values;
- a multi-goal paced/unpaced trace comparison.

## Task 9 review fixes (2026-11-19)

The review of `36729a7..602b9e0` (the turn-around fix and Task 9) reported six important findings and nothing
critical. All six are fixed.

| Finding | Resolution | Regression |
|---|---|---|
| I1: the turn-to-drive handoff made B5 start from its unsent goal-reached zero; yaw command 0.357 -> 0 rad/s in one tick | `ClosedLoopStepper::continue_with()` starts B5 on the drive route and records the command actually in effect as its last command, so the rate limits continue from what the plant received | `TheTurnToDriveHandoffKeepsTheCommandAndTheGoalsStatistics`: each yaw step at the handoff within 0.02 rad/s (yaw deceleration 1.0 rad/s^2); fails with the old handoff |
| I2: the handoff reset the goal's statistics (b5_cycles 181 -> 0) | `continue_with()` keeps the goal's result, timeout window and deadline. A same-goal replan that starts with a turn updates the running goal's route instead of rearming. | same test: the goal's B5 cycle count keeps growing |
| I3: Goal and Route frames could be dropped by a slow client | They are kept frames like Hello and Event. A client whose kept backlog passes 16 times the cap is disconnected. After a client's hello the server replays the latest session Hello, Goal and Route frames. | `KeepsHelloAndEventFramesOverTheCap` (now Goal and Route too); `AReconnectingClientGetsTheSessionHeaderAndTheLatestGoalAndRoute` |
| I4: queued and rejected Goal records had no target or timeout | `GoalEvent::request` keeps the submission for every event, and Goal records carry `request` {target, timeout_s} | `goal_rejected_v3.bin` fixture (`goal: null` with its request), checked in C++ and Python; `goal_v3.bin` regenerated |
| I5: resume hashed `ros2_ws/install` while the session may load another tree | `plant_campaign --print-routing-fingerprint` prints the routing fingerprint of the binary campaigns run: the files it resolves, through the same code as a lattice run's summary. Resume compares it whole with the summary's `routing`, asked once per check. A run whose fingerprint cannot be obtained is not reused. | `test_resume_reruns_a_lattice_run_whose_goal_script_or_routing_inputs_changed`; end to end, the printed fingerprint equals a lattice run's summary `routing` (0.19 s to print) |
| I6: a changed URDF, the footprint's source, did not invalidate resume | covered by I5's whole-fingerprint comparison: `urdf_sha256`, the projected and costmap footprints, and the effective settings | the same test: a changed URDF hash, then a changed footprint, each make the run stale; a missing goal script too |

## Task 10: acquisition-time observations and one scan owner (2026-11-19)

- **The plant's casts (`lidar.hpp`).** Every cast is a `CastBatch`: one per physics step in sweep mode, one per scan
  in instant mode. It records its time, its point and ray ranges, the lidar site's world pose at that cast, and the
  robot configuration the plant supplied (`AcquisitionState`). Points still sit in the lidar frame; nothing is
  copied per point, and the batches partition the points.
- **Where the configuration comes from.** `Plant::finish_substep` passes the configuration of the step it casts in:
  - the base pose is MuJoCo's ground truth, the experiments' declared localization assumption;
  - the arm joints are MuJoCo's positions, standing in for the joint encoders;
  - the rigid payload follows the arm.
- **The production contract (`mobile_manipulator_routing/observation.hpp`).** It is plain values, with no simulator,
  no ROS and no labels:
  - `LidarObservation`: the scan's id, its acquisition interval, `t_received`, its rays, its batches and its points;
  - `ObservationBatch`: time, sensor pose in the map frame, robot configuration and point range.

  It also documents the frames and times. Acquisition times are the plant clock; `t_received` is when the session
  drained the scan; a consumer records its own application time. `to_map()` and `validate()` come with it.
- **The adapter (`observation_adapter.{hpp,cpp}`).** It turns a `PlantScan` into a `LidarObservation` plus
  `ScanLabels` (geom and class, aligned with the kept points). The labels are for validation and diagnostics only,
  and the consumers' type cannot carry them. A batch without its acquisition state is dropped with its points and
  counted, never given another time's pose.
- **One scan owner.** `Session::execute_cycle` drains every completed scan once and hands each one to the
  `ObservationQueue` before the Scan frames' decimation. The queue:
  - accepts scans in acquisition order and refuses a scan whose `t_end` is not newer than the last accepted one (the
    defined out-of-order policy: refuse, never reorder or merge);
  - holds at most 64 scans for the consumer, dropping and counting the oldest beyond that.

  `Session::take_observations()` and `observation_counts()` expose it until the perception phase connects a consumer
  (Tasks 11-12 deferred on 2026-11-19). With the lidar on, the summary's `observations` counts the scans, points, missing-state batches and points, out-of-order
  refusals and overflow.

| Check | Result |
|---|---|
| sweep while the base turns at 2 rad/s and the arm pans: 50 batches partition the points; each has its own time, configuration and lidar site pose; every return lies in its batch's ray range | pass |
| wall returns taken to the world through their own batch's sensor pose lie on the wall's surface (within 1 mm); through the scan's last pose, more than a quarter of them are misplaced by more than 1 cm | pass |
| adapter: a batch without its state is dropped with its 2 points, the rest re-indexed with their own poses, labels aligned | pass |
| queue: equal or older scans refused, the oldest dropped from a full queue, all counted | pass |
| session: 2 s with Scan-frame decimation 5 gives 20 observations (every scan) and 4 Scan frames; all observations validate | pass |
| session: the observations are identical with a streaming session (frames queued, decimation 3) and a headless one | pass |
| routing `test_observation` | 2/2 |
| plant `test_lidar` | 19/19 |
| campaign `test_observations` | 4/4 |
| routing, plant, campaign ctest | 6/6, 12/12, 15/15 |

## Task 10 review fixes (2026-11-19)

The review of `d1887b1` reported no critical or important findings and two minor ones. Both are fixed. The
reviewer's independent probe matched each batch's sensor pose, base pose and joints with the physics state at its
time in ideal, realistic planar and wheeled modes, sweep and instant scans, and a lidar enabled mid-run (largest
difference 2.2e-15).

| Finding | Resolution | Regression |
|---|---|---|
| M1: `validate()` summed batch counts in an unsigned accumulator that could wrap, so `(0, SIZE_MAX), (SIZE_MAX, 1)` passed as a partition | each batch must fit in the points left before its count is added; `t_received` before `t_end` is also refused (the review's additional note) | `ValidationRejectsBatchCountsThatWrapAround`; the drained-early case in the partition test |
| M2: `adapt_scan` indexed the plant scan's ranges and labels before checking them (overlapping batches duplicated points, out-of-range offsets read past the vectors, a `-1` count wrapped the drop count) | the adapter first checks the scan's invariants: labels aligned with the points, nonnegative cast-batch ranges that partition the points in order, also for batches it will drop; malformed input throws, separately from the missing-state drop | `AMalformedPlantScanIsRefusedBeforeItIsRead` |

Suites: routing 6/6; campaign 14/14 without `test_equivalence`, which does not enable the lidar and so does not reach
the adapter (15/15 with it at `d1887b1`).

## Task 13: the two route conditions, kept apart (2026-11-19)

[`task13_conditions.sh`](task13_conditions.sh) ran campaign B's 19 V2 cells
([`corpus_campaign_b.json`](../mujoco-plant/corpus_campaign_b.json)) on the realistic plant, F2 above B5, seeds 1-5,
once per route condition:
- **recorded:** each cell's recorded Nav2 route (its bag);
- **lattice:** routes the Smac Lattice port plans in the session (`--routes lattice`, run names `-lattice`).

The build was the branch at `e2ff548`, installed into the shared `ros2_ws/install` (RelWithDebInfo). The runs went to
`experiment_runs/plant-routing-task13/` (80 MB, 190 runs, about 0.5 h of run time per condition on 4 jobs).
`plant_campaign.py`'s tables gained a Routes column. The ensemble table groups by cell, kind and route condition, so
the conditions are never pooled; a summary without `routes` counts as recorded.

| Condition | Runs | Reached | Plant contacts | Smallest clearance of a reached run |
|---|---:|---:|---:|---:|
| recorded | 95 | 91 | 0 | 0.044 m |
| lattice | 95 | 90 | 0 | 0.043 m |

| Cell | Recorded | Lattice | Median time, lattice minus recorded (reached runs) |
|---|---|---|---:|
| `v2_store_room_nav` | 5/5 | 3/5 | +1.6 s |
| `v2_worker_stops_in_lane_nav` | 1/5 | 2/5 | -0.2 s |
| the other 17 cells | 5/5 each | 5/5 each | -0.5 to +0.8 s, except `v2_room_r4_nav` +2.8 s |

The lattice times include the planning hold at the start (at most about 1 s at 4,096 iterations per tick; section S6).

The lattice failures:
- `v2_store_room_nav` seed 4 failed at 0.7 s without moving: Smac found a route (about 120,000 iterations), and the
  wrapper's swept validation rejected both the smoothed and the raw plan. The start differed from the exact pose
  (`test_v2_routes`, accepted) by 0.57 mm and 0.19 mrad. Nav2 does not run this check and would have executed the plan.
  The failure is safe (no motion) but the goal is lost. Where the sweep touches was not located. **Open for the
  user.**
- `v2_store_room_nav` seed 3 stalled after 264 s. Its route is the same as seed 1's within 0.02 m (15.64 m, same
  end approach), and seed 1 reached in 64.6 s. The robot passed the goal, turned about, and switched sides 6 times
  with 2,824 infeasible-fallback cycles: the medium-planner stall class whose diagnosis follows E5 (roadmap
  2026-11-19), not a route difference.
- `v2_worker_stops_in_lane_nav`: the worker stops in the lane; both conditions mostly stall or time out (recorded
  4 of 5, lattice 3 of 5). Not a route difference.

What the comparison does not claim: the recorded routes came from the same planner with Nav2's costmap timing (map
updates, TF and start-pose quantization). The differences above are reported, not tuned away; the route-level
equivalence is Amendment 1's S5, nothing more.

### Seed 4 diagnosis (2026-11-19)

`V2Routes.DISABLED_StoreRoomSeed4Diagnostic` (`test_v2_routes.cpp`; run with `--gtest_also_run_disabled_tests
--gtest_filter='*StoreRoom*'`) replans the cell from the exact start and from seed 4's start, then checks each path
segment alone.

| Start | Smac | Validation |
|---|---|---|
| exact `(12, 0, 0)` | found, 120,943 iterations | smoothed plan clear |
| seed 4, measured pose or start plus its perturbation (same result) | found, 128,423 iterations, a different path | smoothed and raw plan each blocked on 2 of 282 segments |

What blocks:
- The blocked segments are 237 and 238 of 282: the turn-around at the east end of the store room, around
  (14.7, 9.04) with heading 2.9 to 2.7 rad. Each segment is about 5 cm long with about 4.5 degrees of rotation.
- Both ends of each segment are clear. Only the sweep between them touches.
- What it touches is one occupied wall cell, (799, 569) or its neighbour (800, 569), centred at (14.975, 8.475). That
  is a lethal cell in Smac's costmap (cost 254).
- The cell centre is 0.0296 to 0.0352 m from the validation footprint. A 5 cm cell reaches at most 0.0354 m from
  its centre, so the swept footprint enters the cell's square by at most 6 mm.

What the overlap means:
- The validation footprint carries both allowances: the geometric padding (0.02 m) and the costmap's
  `footprint_padding` (0.01 m), 0.03 m in all.
- So the robot's hull stays at least 24 mm outside the wall cell's square. The cell's square is also not the wall
  surface.
- The contact is inside the allowances, not a body collision.
- Smac accepts the path because it checks its padded footprint's outline only at its own poses, and both poses are
  clear. The between-pose sweep is what the validation pass adds.

### Amendment 3: the sweep checks the geometric padding only (user, 2026-11-19)

The user chose to stop applying the costmap's 0.01 m padding a second time in the sweep (spec Amendment 3). The
validation footprint is now the costmap's configured polygon: the hull plus the 0.02 m geometric padding. The routing
fingerprint records `validation_footprint: geometry_padding`, so a resumed campaign reruns earlier lattice runs.

The other Task 13 runs cannot change:
- All 94 accepted lattice routes validated Smac's smoothed plan.
- A smaller validation footprint still accepts those plans, and Smac's search is unchanged.
- Only `v2_store_room_nav` was rerun: 5 seeds, same configuration, into `experiment_runs/plant-routing-task13-amendment3/`.

| Seed | Before | After |
|---|---|---|
| 1 | reached, 64.6 s | reached, 64.6 s |
| 2 | reached, 61.3 s | reached, 61.3 s |
| 3 | stalled, 264.3 s | stalled, 264.3 s (the medium-planner stall, unchanged) |
| 4 | failed at 0.7 s: both plans rejected | reached, 66.9 s; clearance 0.047 m (planner), 0.041 m (plant), no contacts |
| 5 | reached, 63.7 s | reached, 63.7 s |

With this, the lattice condition reaches 91/95, the same as the recorded condition. The remaining failures are
`v2_worker_stops_in_lane_nav` in both conditions and the seed 3 stall in lattice.

| Check | Result |
|---|---|
| `test_routing_inputs` (new: `TheSweptValidationUsesTheGeometricPaddingOnly`) | 4/4 |
| `test_v2_routes` (new: `TheStoreRoomRouteFromTask13Seed4sStartIsAccepted`, 128,423 iterations, found) | 2/2; 38 of 38 V2 destinations found, 0 rejected |
| campaign ctest without `test_equivalence` (recorded routes only; it never reaches the validation) | 14/14 |


## Task 14: final backend checks, documentation and viewer handoff (2026-11-20)

### Tests

Branch build on the native arm64 container (RelWithDebInfo; overlay `/tmp/mm-routing-integration`, then the shared
install):

| Suite | Result |
|---|---|
| routing ctest | 6/6 |
| MPC ctest | 23/23 |
| medium ctest | 6/6 |
| plant ctest | 12/12 |
| campaign ctest | 15/15 (with the Task 14 changes; `test_demo_server` 22/22 with the new pause test) |
| Python `tools/plant` | 350 passed, 2 skipped |

No amd64 host was available. The suites have no architecture-specific code paths beyond the container image.

### The headless suite

The suite is these named tests, deterministic at simulation time, without a viewer unless a test says otherwise:

| Concern | Tests |
|---|---|
| multi-goal, scheduling, idle | `GeneratedSession.IdleAdvancesThePlantClockAndAScheduledGoalArrives`, `AGoalAfterCompletionAndIdleGetsItsOwnInterval`; `GoalScript.*` (7) |
| replacement | `GeneratedSession.AReplacementWhileMovingSupersedesTheOldGoal`, `AnInvalidReplacementLeavesTheCurrentGoalIntact`, `RealisticPlantHoldsBaseAndArmWhileAReplacementIsPlanned`; `GoalManager.RapidReplacementsArchiveOldGoalsAndRejectTheirResults` |
| reset | `GeneratedSession.ResetDuringASearchStartsAnIdleRunInANewEpoch`; `GoalManager.ResetPreservesUndrainedHistoryAndRejectsOldEpochInputs`, `ResetBackpressureDoesNotPretendToInvalidateTheOldEpoch` |
| pause and pacing | `DemoServer.PausingAndSteppingALatticeRunDoesNotChangeItsDecisions` (new), `PacedAndUnpacedLatticeRunsReachTheSameDecisions`, `StaysResponsiveWhenBehindRealTime` |
| stressed transport | `StreamServer.KeepsHelloAndEventFramesOverTheCap` (with Goal and Route frames), `DropsAnOversizedControlFrame`; `DemoServer.SurvivesClientDisconnect`, `DropsAClientThatNeverSaysHello`, `AReconnectingClientGetsTheSessionHeaderAndTheLatestGoalAndRoute` |
| goal manager bounds and idempotence | `GoalManager.*` (22) |

### Changes

- `--profile-timing` gains `routing_setup_ms` (routing inputs, costmap and planner at reset) and `route_search_ms`
  (the wall time of each tick with a pending search). A search tick's wall time now counts in `wall_s`, so
  `real_time_factor` includes it.
- The routing fingerprint (summary and Hello `routing`) gains `search`: Smac iterations and validation cell tests per
  tick, the validation sampling step, the adoption connector and the pose limit. The budget sets the planning hold,
  so a campaign resume reruns a lattice run made with another budget.
- Tests: `GeneratedSession.ProfileTimingMeasuresTheRoutePlannerSetupAndEverySearchTick`,
  `TheRoutingFingerprintRecordsTheSearchBudgetAndValidationSettings`,
  `DemoServer.PausingAndSteppingALatticeRunDoesNotChangeItsDecisions`.

### Measurements

[`task14_measure.sh`](task14_measure.sh), MacBook Air M4 (fanless), Unity closed, one run at a time, realistic plant,
F2 above B5, seed 1, `experiment_runs/plant-routing-task14/`. The cells with the longest searches (section S6) and a
short one:

| Cell | Lidar | Simulated | Wall | Real-time factor | Search ticks | Search per tick p50 / max | Setup |
|---|---|---:|---:|---:|---:|---:|---:|
| `v2_services_deadend_nav` | off | 69.2 s | 12.3 s | 5.6 | 50 | 6.1 / 15.0 ms | 152 ms |
| `v2_services_deadend_nav` | on | 69.2 s | 20.0 s | 3.5 | 50 | 6.0 / 20.8 ms | 155 ms |
| `v2_store_room_nav` | off | 64.6 s | 16.4 s | 3.9 | 29 | 4.8 / 9.3 ms | 147 ms |
| `v2_store_room_nav` | on | 64.6 s | 24.1 s | 2.7 | 29 | 5.2 / 9.2 ms | 145 ms |
| `v2_open_yard_nav` | off | 37.2 s | 5.9 s | 6.3 | 2 | 5.2 / 5.2 ms | 147 ms |
| `v2_open_yard_nav` | on | 37.2 s | 9.9 s | 3.8 | 2 | 6.1 / 6.1 ms | 147 ms |

All six runs reached their goal. Per-cycle context from the same runs:
- B5's cycle: p50 1.1 to 1.5 ms, p99 1.3 to 3.1 ms.
- The plant step: p50 0.4 to 0.6 ms without the lidar, 2.6 to 3.1 ms with it.
- A lidar scan cast: p50 10 to 13 ms at 10 Hz.

What the numbers say:
- **Search.** A search tick costs about 5 to 6 ms of wall time at 4,096 iterations. The longest tick took 20.8 ms,
  one 20 ms tick's worth. Searches last at most 50 ticks (1 s of simulation) on V2.
- **Costmap.** It is built once per reset, with the routing inputs: about 150 ms. It does not change during a run
  (the persistent observed layer moved to perception).
- **Guard.** None: the observed-base command guard moved to perception (Amendment 2).
- **Paced throughput.** A generated-route demo server ran `v2_services_deadend_nav` (the longest search) at 1x with
  the lidar on: 60.0 s simulated in 60.2 s of wall time (0.997x, the client's connection and start included), with
  no `behind` event. Headless, the same work runs 3.5 times faster than real time, so a 1x demo has that headroom on
  this host.

These are single runs on one host: they size the viewer workload, they are not a performance claim.

### Documentation

- The [routing README](../../../ros2_ws/src/mobile_manipulator_routing/README.md): the API, the effective
  configuration and where each value comes from, the routing fingerprint, and the licences.
  `third_party/NOTICE.md` and `data/README.md` already hold the source and licence manifest (S0).
- [running-the-stack.md](../../running-the-stack.md): generated routes in campaigns and the demo server.
- [viewer-handoff.md](viewer-handoff.md): the contracts, fixtures, costs and open gaps for spec 4 Task 0.
