# ADR 0019: The plant's route search is a C++ port of Nav2's Smac Lattice planner

Date: 2026-11-19. Status: accepted (user decision, 2026-11-19). Design:
[spec 2, Amendment 1](../superpowers/specs/2026-11-18-plant-route-planner-design.md#amendment-1-2026-11-19-the-route-search-is-a-port-of-nav2s-smac-lattice-planner);
record: [implementation.md](../experiments/plant-routing/implementation.md). Builds on
[ADR 0016](0016-mujoco-campaign-plant.md) (the MuJoCo plant).

## Context

Spec 2 gives the MuJoCo plant its own ROS-free route planner. The first version had its own cost model and an
admissible heuristic: the larger of the Euclidean distance and an obstacle-free lattice table within 4 m of the goal.
Goals behind the robot became fast: 1.5 m behind took 6,432 expansions, against more than 40,000 before. Routes through
V2's corridors and doors stayed slow, because the estimate ignored walls beyond the table's window:

- `spine` (35 m) held 54 s of simulation time at 256 expansions per tick.
- `chicane` needed 783,000 expansions.
- `yard_store` exceeded the 1,000,000-expansion limit, although Nav2's Smac Lattice found its route in the recordings.

The baselines' global planner was Smac Lattice (`nav2_smac_planner` 1.3.13) with the settings in
`nav2_global_planning.yaml`.

## Options considered

- **A: a conservative wall-aware bound.** It keeps cheapest routes guaranteed. It is weaker than Smac's estimate, needs
  its own proofs and tests, and leaves an in-house planner to describe and defend.
- **B: Smac's search, ported.** Smac's wall-aware 2D estimate, its Dubins table, its direct-to-goal curve connection,
  its costs and its smoother. Routes are not guaranteed cheapest, but the behavior is the planner the baselines used.
- **B as an in-house design inspired by Smac.** Rejected: it would need the same qualification without the benefit of a
  known reference.

## Decision

Option B, as a direct port of the 1.3.13 source (Apache-2.0, with its notices kept) and the costmap semantics of
`nav2_costmap_2d` 1.3.13. OMPL's Dubins curves (BSD) are copied so the core stays free of OMPL. The deliberate
differences are listed in the amendment (A1.2):

1. the search can pause between iterations;
2. Smac's wall-time limit is replaced by its iteration limit;
3. Smac's exceptions become typed outcomes;
4. one swept-footprint validation pass checks the returned path;
5. Smac's static state becomes one planner object per session;
6. its ROS publishers and parameters are dropped.

## Consequences

- The thesis can name the route search "a port of Smac Lattice 1.3.13, equivalent on the qualification fixtures".
  That claim needs the amendment's end-to-end test against the real planner in the container.
- The earlier admissible heuristic (commit `209179c`), our lattice A* internals and our own soft-cost field are
  removed. The goal manager, grid revisions, primitive loader and `Search` interface stay.
- The goal policy follows Nav2: the search may stop within 0.20 m of the requested pose. The requested and resolved
  endpoints are recorded separately. The controller's completion tolerances are unchanged.
- The 2.5 m/rad rotation cost question is closed: Smac's rotation penalty (5.0 per primitive) applies.
- The per-tick budget counts Smac iterations. The per-tick cell-test cap is removed.

## Revisit when

- the equivalence test finds a mismatch that A1.2 cannot explain;
- the validation pass rejects Smac routes on the fixtures;
- a second robot or a reverse-capable primitive set needs Reeds-Shepp curves or reverse expansion.
