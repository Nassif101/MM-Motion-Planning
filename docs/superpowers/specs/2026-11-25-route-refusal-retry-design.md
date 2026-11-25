# Route search: re-plan around a refused segment (design)

Date: 2026-11-25. Branch `route-refusal-check`. Status: approved by the user ("go ahead", 2026-11-25).

## Intent

Smac (the Lattice port) checks its footprint at the lattice poses only; in turns in place beside walls the footprint
sweeps centimetres between them, and the swept validation then refuses the route; the goal fails with no second
attempt (docs/experiments/plant-routing/route-refusals-2026-11-25.md: 9 of 60 door runs, seed-dependent). The user's
ruling (2026-11-25): fix it now, before 5B, on this branch.

## Goals, non-goals

- Goal: a route Smac finds but the validation refuses is planned again around the refusing spot, a bounded number of
  times; a validated retry is accepted as any route is.
- Non-goals: no change to Smac's search (ADR 0019: an exact port), to the validation's strictness, or to routes the
  validation accepts. Anti-goal: a run without a refusal changes in any way.

## Approaches

1. **Re-plan on a costmap copy marked at the blocking cell (chosen; the user's pick of candidate 1).** Keeps Smac exact
   and the validation strict; costs a search per retry.
2. Repair the refused turn in the wrapper (move it along the route): invents path geometry outside Smac. Not chosen.
3. Change what Smac checks (a larger padding, denser rotations): leaves the exact port or costs passage. Not chosen.

## Design

- `MotionCheck::blocking_cell()`: the grid cell that blocked a sweep (occupied, or the exterior: none).
- `Search` on a refusal (raw and smoothed both blocked): if a blocking cell is known and fewer than
  `SearchConfiguration::refusal_retries` (default 3) retries ran, copy the costmap Smac searched (the planner's, or the
  last retry's), set the blocking cell's free 8-neighbours to LETHAL (254) in the copy, and begin Smac again on it from
  the same start and goal; validate its plan against the unchanged validation grid. The retries' work counts against
  the same per-tick budget (`advance`). If every retry ends without a validated route (refused again, or Smac finds no
  path on the marked copy), the result is `Rejected`, its reason the original one plus the retries' outcome.
- A validated retry's result: `Found`, reason "validated after N re-plans around refused segments";
  `SearchStatistics::refusal_retries` counts them.
- Determinism: the copy and the marks follow from the inputs only; a search without a refusal is unchanged byte for
  byte.

## My calls (the user can override)

3 retries; one-cell marks (5 cm); the 8-neighbourhood; retries within the normal per-tick budget.

## Checks

- Tests: the two reproduced refusals (west seed 11, east seed 4) become `Found` with a validated route, retries > 0;
  the existing route tests pass; a search without a refusal has the same plan and statistics as before; a retry that
  cannot validate ends `Rejected`.
- Evidence: the 60 door runs rerun (0 refusals expected); the three reference logs of the medium diagnosis unchanged.
- A subagent review of the branch.
