# Route refusal retry: implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:executing-plans. Steps use checkbox (`- [ ]`) syntax.

**Goal:** a route the swept validation refuses is re-planned around the blocking cell, up to 3 times.

**Architecture:** the search wrapper (`mobile_manipulator_routing/src/search.cpp`) copies the costmap, marks the
blocking cell's free neighbours LETHAL and begins Smac again; the validation is unchanged.

**Tech Stack:** C++17, gtest, colcon in the dev container (`mm-motion-planning-ma-robot-sim-1`).

**Spec:** [docs/superpowers/specs/2026-11-25-route-refusal-retry-design.md](../specs/2026-11-25-route-refusal-retry-design.md)

## Global Constraints

- A search without a refusal: same plan, statistics and frames as before (the medium diagnosis's three reference logs).
- No change to the Smac port (`src/smac/`) or the validation's test.
- One commit per task on `route-refusal-check`; no main commits.

## Review Focus

1. A refusal whose blocking cell is the map's exterior (no cell): no retry, `Rejected` as before.
2. A retry whose marked copy leaves Smac no path: `Rejected` with the original reason, not `NoPath`.
3. Budget: a retry started mid-tick continues on the next `advance` (resumable), cancellation and stale revisions still
   end it.
4. The marks accumulate over retries and never touch occupied cells.

---

### Task 1: The blocking cell and the retry

**Files:** `ros2_ws/src/mobile_manipulator_routing/include/mobile_manipulator_routing/{footprint,search}.hpp`,
`src/{footprint,search}.cpp`; tests `ros2_ws/src/mobile_manipulator_campaign/test/test_v2_routes.cpp`,
`ros2_ws/src/mobile_manipulator_routing/test/` (the checker's test file).

**Interfaces:** `std::optional<CellIndex> MotionCheck::blocking_cell() const`; `SearchConfiguration::refusal_retries`
(int, default 3, >= 0); `SearchStatistics::refusal_retries` (std::size_t).

- [ ] **Step 1: Failing tests.** `TEST(V2Routes, HallwayRefusalsAreReplanned)`: the west seed 11 and east seed 4 starts
  of `DISABLED_HallwaySeedsDiagnostic` end `Found`, `refusal_retries > 0`, and their poses pass a fresh
  `FootprintChecker::motion` check. `TEST(V2Routes, NoRetryWithoutARefusal)`: the exact-start west route has
  `refusal_retries == 0` and the same expansions as before (196661). With `refusal_retries = 0` the seed 11 route is
  `Rejected` (the old behaviour). A checker test `BlockedSweepReportsItsCell`: a motion into a wall reports an occupied
  cell; a clear motion reports none.
- [ ] **Step 2: Run them to see them fail** (build errors: no `blocking_cell`, no `refusal_retries`).
- [ ] **Step 3: Implement** as in the design; the retry costmap is a `std::shared_ptr<const smac::LayeredCostmap>` held
  by the search.
- [ ] **Step 4: Run** the routing package's tests and `test_v2_routes`; expected PASS.
- [ ] **Step 5: Commit.**

### Task 2: Evidence and records

- [ ] **Step 1:** rerun the door runs (`bash docs/experiments/medium-diagnosis/diag_ensembles.sh door` with
  `OUT=experiment_runs/route-retry/door`); count refusals and R4 outcomes against `experiment_runs/medium-diag/door`.
- [ ] **Step 2:** the three reference logs of `docs/experiments/medium-diagnosis/reference_logs.json` unchanged.
- [ ] **Step 3:** record in `docs/experiments/plant-routing/route-refusals-2026-11-25.md` ("The fix") and the roadmap
  entry's follow-up; commit; a subagent review of the branch; fix its Critical/Important findings test-first.
