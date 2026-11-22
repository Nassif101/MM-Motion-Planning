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

## Stage A: what the logs say (Task 3, 2026-11-22)

Tool: `python3 tools/plant/medium_diag.py seeds --run RUN --from T0 --to T1` (the seeds tried per cycle, with the
seed detail). Counts below are from the baseline's and the door runs' logs; the cycles counted are named with each.

### Question 1: E3's gap cells

**The 51 runs F2 left infeasible until the robot stopped are infeasible by millimetres.** Over the fork cycle and the
next 2 s (five cycles, 653 seed rows):

- the incumbent's collision violation has quartiles 2, 4 and 44 mm; the best seed of each cycle 2, 6 and 44 mm (the
  feasibility tolerance is 1 mm, the margin 50 mm);
- 627 of the violating rows are the panel, against `brace_right` (251), `brace_left` (229) and the floor (140);
- the iteration budget cut 40 of the 653 rows (6 %); 440 rows ran the per-seed cap of 15 SCP iterations
  (`scp.max_iter`) without converging. A larger budget alone does not raise that cap.

So the planner ends a few millimetres short of feasible, mostly with the panel between a brace and the floor. Logs
cannot say whether the margin, the per-seed iteration cap or the geometry keeps it there: that is the fork's job.

**The 21 `ok_plan_b5_creep` runs: F2's plan rides the margin and B5 does not follow it.** Over the 10 s after the
stop (526 cycles): the incumbent wins 513, in the comfort homotopy 488; its deepest row is a brace at 0.0499-0.050 m
(the margin) in 432; it claims 1.42-1.66 m of progress over its horizon (quartiles); B5's slack gate is clear (slack
0) and B5's own plan moves 4-28 mm in 2 s. A cheaper feasible seed than the winner existed in 6 cycles.
Hypothesis for the fork: B5, with the same 0.05 m margin, cannot advance along a plan that keeps the panel exactly at
the margin.

### Question 2: the R4 door stall

**Reproduced once in 60 runs, from the hallway's east part.** The door runs (`diag_ensembles.sh door`,
`corpus_door.json`): the robot first stops in the hallway (at C1's mouth, its west part, its east part), then drives
to the goal in R4 from a standstill; 20 seeds each, realistic, lattice routes.

| Start in the hallway | R4 reached | R4 failed | Hallway goal failed |
|---|---|---|---|
| C1's mouth | 20/20 | 0 | 0 |
| west part (3.5, 12.65) | 19/20 | 1 (route rejected) | 5 (route rejected) |
| east part (11.5, 12.65) | 19/20 | 1 (timeout at the door) | 3 (route rejected) |

`r4_from_east`, seed 18: the base stops at (7.87, 13.22), 0.53 m before the door frame and 36 degrees off its axis,
and stays there for 98 s until the goal's timeout. In the 246 cycles after the stop F2 is `ok` and feasible, B5
tracks (no fallback, no hold, its Prediction frames `ok`) and B5's plan moves 7 mm in 2 s (median). **It is the gap
cells' creep:** F2's incumbent (comfort homotopy in all 246) claims 1.82 m of progress in 241 of them, with its
deepest row at the R4 door frame, and B5 does not follow it. The feasible `vertical_carry` and `vertical_carry_right`
plans (79 of the cycles, 1.37-1.82 m of progress) cost 1.78-4.67 and are never cheaper than the incumbent in the same
cycle. So no layer refuses: F2 publishes a feasible comfort plan through the door, and B5 tracks it without moving.
This matches the user's report only in part (the user saw the ghosts of a vertical swing; here the published plan is
the comfort incumbent): the user's own case may differ.

**A separate finding (not the medium planner):** all 8 hallway-goal failures and the R4 failure from the west are the
route search refusing its route ("the smoothed and the raw plan failed the swept validation"), spec 2's layer.

### Question 3: the low-beam pivot

**`fold_back` loses on feasibility and on cost.** Over the cycles with the base between 3 m before the beam and
0.3 m past it (20 runs, 654 cycles):

- `fold_back` is tried in 138 cycles (every fifth: one pose seed per cycle in turn), feasible in 62;
- when infeasible: the panel against the beam (51), a hull bound, joint motion between samples (20), the floor (2),
  the base (1), the base speed (2);
- when feasible its merit is a median 20 times the winner's (1.0-431);
- the winner is the incumbent in nearly every cycle; the winners' homotopies are comfort (302), left (225, with or
  without lowered), lowered (82) and right (45). The side pivot is a left or right swing.

Which cost term makes the feasible `fold_back` plans 20 times costlier is not in the log (the seed row carries the
merit, not its terms). The viewer's losing-seed paths would show its shape, not its cost; the term split would answer
it.

### Proposed for Stage B (checkpoint A)

1. **Gap infeasible stalls (51 fork points, 5 per cell):** one-cycle forks with the medium margin at 0.04 and 0.03 m,
   the budget at 60, 120 and 300, and **a new key, the per-seed iteration cap `max_iter` at 30 and 60** (the cap the
   logs show binding; not in the plan); the seeds pair at budget 300.
2. **Creep (21 gap runs and the door run):** B5's margin at 0.03 m (does B5 then follow a plan at F2's margin?),
   one-cycle and closed loop.
3. **The cost split (small code, not a fork):** add the per-term costs of each seed to `--seed-detail`
   (`MediumProblem::cost_terms` exists) and reread the pivot's and the door's cycles: which term makes `fold_back`
   costlier, and why the comfort incumbent beats the vertical swings at the door.

### Problems and resolutions (Task 2)

| Problem | Cause | Resolution | Where |
|---|---|---|---|
| 21 of 36 forks at a stall's onset came 13.5 to 102.5 s after the base had stopped; 7 of 9 `timeout_moving` runs had a parked base | The onset copied the session's progress rule, whose joint test counts a slowly drifting arm as progress | The stop is the base's progress alone (position or yaw, on the true pose) | Task 2 review C1 |
| 11 runs infeasible until the stop were not forked at their infeasibility | The streak had to run to the log's end; the 45 s after the stall carry ok, waiting cycles | The streak runs to the stop, within one medium period | Task 2 review I1 |
| `slack_stall` mixed B5's gate holding on infeasible plans with B5 creeping on ok plans, and counted B5's `infeasible` fallback as a hold | A catch-all class | Classes named by the layer that stopped (`infeasible_gate_hold`, `infeasible_b5_fallback`, `ok_plan_b5_creep`, ...); only B5's `holding` status is a hold | Task 2 review I2 |
| A bad summary crashed the report; a run directory without one was dropped silently | Not handled | Both are listed as `unreadable`, as are summaries without `reached` and logs without States | Task 2 review M3 |
