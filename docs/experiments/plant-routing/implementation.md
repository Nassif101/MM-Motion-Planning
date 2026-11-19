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

- **Heuristic for reorientation goals.** The Euclidean heuristic with in-place rotation at 2.5 m/rad leaves goals
  that need a turn (for example 1.5 m behind the robot) to a near-Dijkstra search: more than 40,000 expansions
  without a result within a 30 s deadline. A heading-aware admissible heuristic (Smac's precomputed non-holonomic
  table, or a simpler bound) is a search-policy change for the user to rule on.
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
