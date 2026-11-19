# Mobile manipulator routing

Portable C++17 route planning for the MuJoCo plant (spec 2). The search is a port of Nav2's Smac Lattice planner
1.3.13 with its costmap semantics (spec 2 Amendment 1, ADR 0019). Around it are this package's own map grid,
swept-footprint validation, resumable work budgets and the lidar observation type.

The core has no ROS, TF, Nav2, OMPL, Unity, URDF or bag dependency. Callers provide resolved map, footprint and
motion data. YAML and JSON parsing use yaml-cpp and nlohmann JSON; tests use system GTest.

## Build and test

In the container's colcon workspace, the campaign package builds it:
`colcon build --packages-up-to mobile_manipulator_campaign --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`.

Standalone, with those development dependencies available:

```sh
cmake -S ros2_ws/src/mobile_manipulator_routing -B build/routing -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build/routing --parallel 2
ctest --test-dir build/routing --output-on-failure
```

The default package build uses plain CMake. The optional ament export needs `-DROUTING_WITH_AMENT=ON`; it does not
change runtime ownership. Data is installed under `share/mobile_manipulator_routing`.

The equivalence checks against the installed Nav2 Smac planner need Nav2. They are in the container-only package
`mobile_manipulator_routing_nav2_check`.

## API in brief

- `GridSnapshot` (`grid.hpp`): an immutable occupancy grid with a revision.
- `FootprintChecker` (`footprint.hpp`): pose and swept-motion checks of a convex footprint. They are resumable and
  count each tested cell against a budget.
- `RoutePlanner` and `Search` (`search.hpp`):
  - One `RoutePlanner` holds the Smac port, its costmap and the validation footprint. It serves one search at a
    time.
  - A `Search` advances by a `WorkBudget` per call (Smac iterations, then validation cell tests).
  - A search ends as `found`, `invalid_start`, `invalid_goal`, `no_path`, `budget_exhausted`, `rejected` (both the
    smoothed and the raw plan failed validation), `canceled` or `stale` (its input revisions changed).
- `LidarObservation` (`observation.hpp`): one scan in acquisition-time batches, with the sensor pose and robot
  configuration of each batch. It carries no simulator labels. `validate()` checks its invariants.

## Configuration

The session (`mobile_manipulator_campaign`) assembles the effective configuration. Each value has one source:

| Setting | Source |
|---|---|
| Smac Lattice settings: costs, tolerance, iteration limit, analytic expansion, smoother | the `Lattice` block of `mobile_manipulator_navigation/config/nav2_global_planning.yaml` |
| Costmap: `footprint_padding` 0.01 m, inflation radius 1.0 m, cost scaling 5.0 | that file's global costmap |
| Base footprint: projected URDF base links plus 0.02 m geometric padding | `mobile_manipulator_navigation/config/dynamic_footprint.yaml` and the URDF |
| Validation footprint: the base footprint with the geometric padding only | spec 2 Amendment 3 |
| Map and known obstacles: `construction_site_v2`, band 0.02-0.60 m | `mobile_manipulator_navigation/maps/` |
| Motion primitives: Nav2's 5 cm differential-drive set | `data/diff_5cm.json` (see `data/README.md`) |
| Budgets per 20 ms tick: 4,096 Smac iterations, 524,288 validation cell tests; validation sampling step 0.0125 m; adoption connector 2 cells | `WorkBudget` and `SearchConfiguration` defaults, listed in `config/routing.yaml` |

Every generated-route run records the result as its routing fingerprint: the summary's `routing` and the session
Hello's `routing`. The fingerprint holds:
- the hashes of the map, primitives, URDF and both configuration files;
- the footprints and paddings;
- the `Lattice` and costmap settings;
- the search budget and validation settings (`search`);
- the planner identity.

`plant_campaign --print-routing-fingerprint` prints what a run of the installed build would record. Campaign resume
reruns a lattice run whose fingerprint differs.

The budgets are work limits, not timing guarantees. Spec 2 Task 14 measured about 5-6 ms of wall time per search
tick at 4,096 iterations (maximum 20.8 ms) on a MacBook Air M4; see `docs/experiments/plant-routing/implementation.md`.

## Licences

The Smac, costmap and Dubins sources follow Apache-2.0 and BSD-3-Clause upstream files. See
`third_party/NOTICE.md` for the upstream versions, commits and licence texts. `data/README.md` covers the primitive
file's provenance and licence.
