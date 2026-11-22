# The medium-planner diagnosis (the start of 5B): results

Spec: [2026-11-22-medium-diagnosis-design.md](../../superpowers/specs/2026-11-22-medium-diagnosis-design.md).
Plan: [2026-11-22-medium-diagnosis.md](../../superpowers/plans/2026-11-22-medium-diagnosis.md).

Three questions: why F2 fails on E3's gap cells; which layer refused at the R4 door the user saw stall; why F2 chose
the side pivot over `fold_back` at the 1.23 m beam.

## Baseline on main (Task 2, 2026-11-22)

Runs: `bash docs/experiments/medium-diagnosis/diag_ensembles.sh baseline` (build 144c3bd: main's behavior plus the
opt-in seed detail), 200 runs in 20 minutes, no host sleep in the window (`pmset -g log`). Data (git-ignored):
`experiment_runs/medium-diag/baseline`, report `experiment_runs/medium-diag/baseline_report/` (`report.md`,
`fork_points.json`), produced by `python3 tools/plant/medium_diag.py report`.

- E3's eight gap cells: F2, realistic plant, the installed 0.05 m margins, their own scenario routes, seeds 1-20.
- R4's door (`v2_room_r4_nav`) and the 1.23 m low beam (`v2_low_beam_123_nav`): F2, realistic, lattice routes, seeds
  1-20.

| Cell | Reached, E3 sweep m050 (2026-11-16) | Reached, baseline | Failures by class |
|---|---|---|---|
| e3_gap_straight_1 | 17/20 | 17/20 | infeasible_b5_fallback 1, ok_plan_b5_creep 2 |
| e3_gap_straight_2 | 17/20 | 17/20 | infeasible_gate_hold 2, ok_plan_b5_creep 1 |
| e3_gap_straight_3 | 13/20 | 13/20 | infeasible_b5_fallback 2, infeasible_gate_hold 3, ok_plan_b5_creep 2 |
| e3_gap_straight_4 | 2/20 | 2/20 | infeasible_b5_fallback 11, infeasible_gate_hold 6, ok_plan_b5_creep 1 |
| e3_gap_turn_1 | 17/20 | 17/20 | infeasible_b5_fallback 1, ok_plan_b5_creep 2 |
| e3_gap_turn_2 | 9/20 | 9/20 | infeasible_b5_fallback 4, infeasible_gate_hold 1, ok_plan_b5_creep 6 |
| e3_gap_turn_3 | 8/20 | 8/20 | infeasible_b5_fallback 1, infeasible_gate_hold 2, ok_plan_b5_creep 7, timeout_moving 2 |
| e3_gap_turn_4 | 3/20 | 3/20 | infeasible_b5_fallback 14, infeasible_gate_hold 3 |
| v2_room_r4_nav (lattice) | - | 20/20 | - |
| v2_low_beam_123_nav (lattice) | - | 20/20 | - |

**Main repeats E3's gap runs exactly.** All 160 gap runs equal their E3 sweep twins in the reached flag, the
simulated time, the minimum clearance, the count of infeasible medium cycles and the end status (checked here). The
Task 2 review compared every summary field and every frame, the fields of the options that differ (E3's row trace,
this baseline's seed detail and prediction) set aside: 160 of 160 equal. Specs 2 and 4 (the turn leg, the route
search, the run-log schema) left these cells unchanged, so E3's numbers stand on main.

**Where the robot stopped, and which layer stopped it.** A failed run's stop is the last time its base made progress
(more than 0.05 m or 0.1 rad, the session's base rule on the true pose; the arm is left out, because a parked base
with a slowly drifting arm is stopped). Its fork point is the first cycle of the streak of F2 cycles that are not ok
and run to the stop (within one medium period), else the stop itself. Of the 74 failed gap runs:

- **51: F2 infeasible until the robot stopped.** In 34 (`infeasible_b5_fallback`) B5 then runs on its `infeasible`
  fallback (it drives the route on its own) and the base still does not move; in 17 (`infeasible_gate_hold`) B5's
  slack gate holds. F2 turned infeasible 0.4 s after to 23 s before the stop (median about 4 s before).
- **21 (`ok_plan_b5_creep`): F2's plans are ok, B5 tracks them, the base does not move.** After the stop F2 is ok in
  79-100 % of the cycles and not waiting, B5 reports ok with no fallback, and B5's own predicted motion over its 2 s
  horizon is 4-28 mm (median over the run's predictions after the stop). A new failure class: no layer reports a
  problem.
- **2 (`timeout_moving`): still moving at the end** (`e3_gap_turn_3` seeds 18 and 20, the base stopped in the last
  0.15 s of the 120 s timeout).

No run was classed `ok_plan_b5_held` (B5 holding an ok plan) or `f2_waiting`. The fork point of each failed run, with
where and when the base stopped and B5's state there, is in `fork_points.json`.

**The door and the beam did not fail from the scenarios' starts.** R4 reached in 62-75 s and the beam in 33-37 s, on
every seed. The user's stall at R4 came from the hallway towards a goal in R4 (Task 3, A3); the user's pivot at the
beam is a passage, not a failure (Task 3, A4).

A third reference log (`e3_gap_turn_4-s14`, a stalled run) joins the two of Task 1 in `reference_logs.json` (the
Task 1 review, M1).

### Problems and resolutions (Task 2)

| Problem | Cause | Resolution | Where |
|---|---|---|---|
| 21 of 36 forks at a stall's onset came 13.5 to 102.5 s after the base had stopped; 7 of 9 `timeout_moving` runs had a parked base | The onset copied the session's progress rule, whose joint test counts a slowly drifting arm as progress | The stop is the base's progress alone (position or yaw, on the true pose) | Task 2 review C1 |
| 11 runs infeasible until the stop were not forked at their infeasibility | The streak had to run to the log's end; the 45 s after the stall carry ok, waiting cycles | The streak runs to the stop, within one medium period | Task 2 review I1 |
| `slack_stall` mixed B5's gate holding on infeasible plans with B5 creeping on ok plans, and counted B5's `infeasible` fallback as a hold | A catch-all class | Classes named by the layer that stopped (`infeasible_gate_hold`, `infeasible_b5_fallback`, `ok_plan_b5_creep`, ...); only B5's `holding` status is a hold | Task 2 review I2 |
| A bad summary crashed the report; a run directory without one was dropped silently | Not handled | Both are listed as `unreadable`, as are summaries without `reached` and logs without States | Task 2 review M3 |
