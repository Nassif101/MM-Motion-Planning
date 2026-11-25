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

## Stage A: what the logs say (Task 3, 2026-11-22; corrected by its review)

Tool: `python3 tools/plant/medium_diag.py seeds --run RUN --from T0 --to T1` (the seeds tried per cycle, with the
seed detail). The Task 3 review recomputed every number with its own code and corrected two readings (the
infeasible stalls and the creep); the corrected readings are below, each with the window it is taken over.

### Question 1: E3's gap cells

**The 51 runs F2 left infeasible until the robot stopped: F2 starts a few millimetres short, then B5 gives up on F2
and drives into the gap alone.**

- *At the fork* (the fork cycle and the next 2 s, six cycles per run, 306 incumbents): the incumbent's collision
  violation has quartiles 2, 4 and 44 mm (tolerance 1 mm, margin 50 mm), mostly the panel against a brace or the
  floor. 130 incumbents converged while still infeasible and 138 stopped at the per-seed cap of 15 SCP iterations
  (`scp.max_iter`) unconverged; the budget cut 40 of 653 logged seed rows, and further scheduled pose seeds never ran
  (the review counts 183; they leave no row).
- *Then B5's layer:* in all 34 `infeasible_b5_fallback` runs B5 drops F2's plan a median 2.04 s after the fork
  (2.04-3.64 s): its `medium_max_infeasible_s` is 2.0 (E1's limit, ADR 0017, against following infeasible plans into
  a frame). It then drives the route alone, the panel into the gap; at the stop F2's incumbent is 0.19-0.26 m inside
  the margin (quartiles 191, 220, 257 mm). B5 alone fails the straight gap cells (0/20 each, the 2026-11-18 data). In
  the 17 `infeasible_gate_hold` runs B5's slack gate holds instead.

So "millimetres short" holds at the fork only; the stall itself comes from B5 driving alone after 2 s.

**The 21 `ok_plan_b5_creep` runs: a speed standstill, not a margin hold.** Every new F2 plan starts from the measured
near-zero speed and accelerates over its horizon (`e3_gap_straight_1` seed 12: 0.04 m in its first second, 0.11 m in
two, 1.6 m at 8 s). B5 follows the start of the plan, so the base moves millimetres (B5's own plan: 4-28 mm in 2 s),
the next measured speed is again near zero, and the next plan starts slow again. F2's claimed progress (1.4-1.7 m:
where the plan ends after 8 s) and its waiting test (where the plan ends) never flag it: 0 of 526 cycles waiting (the 10 s after each stop).
The review further reads B5's command below the base's 5 mm/s dead band, so the breakaway lift never applies; that
needs B5's command (`--diagnostics`), which the baseline did not record. The margin is not what binds: at the door
(below) the winning plan has no violating collision row in 236 of its 246 cycles.

### Question 2: the R4 door stall

**Reproduced once, from the hallway's east part, as the same speed standstill; probably the user's case.** The door
runs (`diag_ensembles.sh door`, `corpus_door.json`): the robot first stops in the hallway (at C1's mouth, its west
part, its east part), then drives to the goal in R4 from a standstill; 20 seeds each, realistic, lattice routes.
Counted over the runs that reached the hallway point (8 were refused their route to it and started from elsewhere):

| Start in the hallway | Runs from the hallway | R4 reached | R4 failed |
|---|---|---|---|
| C1's mouth | 20 | 20 | 0 |
| west part (3.5, 12.65) | 15 | 14 | 1 (route refused) |
| east part (11.5, 12.65) | 17 | 16 | 1 (timeout at the door) |

`r4_from_east`, seed 18: the base stops at (7.87, 13.22), about 0.4 m before the door frame and 36 degrees off its
axis, and stays there for 98 s until the goal's timeout. In the 246 cycles after the stop F2 is `ok` and feasible and
B5 tracks (no fallback, no hold, its Prediction frames `ok`), moving 7 mm in 2 s (median). F2's plan starts slow (8 cm
in its first second, 28 cm in two) and ends 1.84 m on (median), as on the gap cells. **No layer refuses; the plan's
slow start is followed literally.** The winner is the incumbent in 233 cycles and `vertical_carry` in 13; the
incumbent is labelled comfort (the label reads only the shoulder pan and the panel's height) but tilts wrist_2 to
1.62 rad (the vertical-carry pose: 1.571), so its ghosts would show a vertical panel through the door: likely the
user's observation.

**A separate finding (not the medium planner):** all 8 hallway-goal failures and the R4 failure from the west are the
route search refusing its route ("the smoothed and the raw plan failed the swept validation"), spec 2's layer.

### Question 3: the low-beam pivot

Over the cycles with the base between 3 m before the beam and 0.3 m past it (20 runs, 654 cycles):

- `fold_back` (pan 0, the arm folded back) is tried in 138 cycles (one pose seed per cycle in turn), feasible in 62;
- when infeasible: a collision with the beam (51, 7 of them the wrist), a hull bound on the joint motion between
  samples (20), the floor (2), the base (1), the base speed (2);
- when feasible it mostly ran cold and unconverged (58 of 62) against a refined incumbent, so its logged merit (a
  median 20 times the winner's) overstates the gap; the review puts the factor at the moment of the swing decision at
  about 1.8-3.5;
- the winners' homotopies are comfort (302), left (225), lowered (82) and right (45); 5 of 20 runs pass without a side
  swing.

Which term of the cost makes `fold_back` costlier is not in the log (the row carries the merit, not its terms).

### Proposed for Stage B (checkpoint A)

1. **Infeasible stalls (51 fork points, at most 5 per cell):** one-cycle forks with the medium margin at 0.04 and
   0.03 m, the budget at 300, a new key for the per-seed cap (`max_iter` 30), and the seeds pair at budget 300: can
   F2 become feasible at the same state? Closed-loop forks with B5's `medium_max_infeasible_s` raised (a new key;
   E1's safety limit, so a passage-against-contact test): does following F2's nearly feasible plan pass where B5 alone
   does not?
2. **The speed standstill (21 gap runs and the door run):** a control fork with `--diagnostics` at the stop (B5's
   command, the dead band and the breakaway at the same state). A single-setting fork on the standstill itself needs
   a key on F2's start speed or B5's breakaway; which one is a 5B design choice, so only the control fork is proposed.
3. **The cost split (small code, not a fork):** the per-term costs of each seed in `--seed-detail`
   (`MediumProblem::cost_terms` exists), then reread the pivot's decision cycles.

**Ruling at checkpoint A (user, 2026-11-24):** run all three (A, B, C). The user recreates the door stall in the
viewer later, outside this stage.

## Stage B: the replays (Task 5, 2026-11-25)

Runs: `bash docs/experiments/medium-diagnosis/diag_ensembles.sh forks` (build 8dcda54), 290 fork runs in 43 minutes,
no host sleep in the window; data `experiment_runs/medium-diag/forks_{one,closed,control}`, report
`experiment_runs/medium-diag/forks_report.md` (`tools/plant/fork_replay.py report`). Each fork reruns a failed
baseline run byte for byte to its fork point and changes one setting there (Task 4; the Task 4 review checked a fork
frame for frame against its baseline). **The control forks reproduce their original run in 27 of 27.**

### A: the infeasible stalls (27 fork points, at most 5 per cell)

**One cycle at the fork: the margin makes the plan feasible, iterations and seeds barely do.**

| Setting (alone) | Forked plan feasible | Incumbent violation, median | Cycle wall time, median |
|---|---|---|---|
| control | 0/27 | 10.5 mm | 0.095 s |
| medium margin 0.04 m | 10/27 | 0.6 mm | 0.095 s |
| medium margin 0.03 m | 11/27 | 0.4 mm | 0.094 s |
| budget 300 | 2/27 | 10.5 mm | 0.272 s |
| per-seed cap 30 iterations | 3/27 | 10.5 mm | 0.098 s |
| every seed every cycle, budget 300 (against budget 300) | 2/27 | 10.5 mm | 0.278 s |

No cycle was cut by the 10 s deadline. Budget 300 costs 0.27-0.28 s per cycle on this host, inside the 0.4 s period.

**Closed loop from the fork: no single setting gets the robot through.**

| Setting (alone) | Runs | Reached | Stalled | Runs with a contact |
|---|---|---|---|---|
| control | 27 | 0 | 24 | 1 |
| B5's `medium_max_infeasible_s` 6 s | 27 | 0 | 23 | 1 |
| B5's `medium_max_infeasible_s` off | 27 | 0 | 18 | 1 |
| medium margin 0.04 m (the 10 feasible) | 10 | 0 | 9 | 0 |
| medium margin 0.03 m (the 11 feasible) | 11 | 1 | 7 | 1 |
| per-seed cap 30 (the 3 feasible) | 3 | 0 | 2 | 0 |
| budget 300 (the 2 feasible) | 2 | 0 | 2 | 0 |
| every seed, budget 300 (the 2 feasible) | 2 | 1 | 1 | 1 |

Classed as in Stage A (`medium_diag.py report` on the fork runs): with B5's limit raised or off, B5 follows F2's
nearly feasible plan further and its slack gate then holds (gate holds 15-16 of 27, against 9 in the control); with
the smaller margin F2 is feasible for a while and turns infeasible again deeper in the gap (8 of 10 at 0.03 m, 10 of
10 at 0.04 m), and 2 runs then creep.

**Reading:** E3's gap stalls are not caused by one setting. The plan in front of the gap is a few millimetres from
feasible at the 0.05 m margin, and a 1-2 cm smaller margin makes it feasible there, but the passage gets tighter
beyond that point; following F2 longer only moves the stop to B5's own gate. Neither the iteration budget, the
per-seed cap nor the seeds tried is the limit.

### B: the speed standstill (18 creep runs, 5 per cell, and the R4 door run)

A control fork at the stop with B5's diagnostics (the command it sends each cycle): over the 81,494 cycles from 1 s
after the stop, **B5 sends a zero base command in 99.2 %** (forward and yaw); its breakaway lift fires 200 times in
all (0.0375 m/s), on 14 of the 19 runs, without getting the base going; the arm's joint-speed commands are a median
2.5 mrad/s (largest joint). The door run: no non-zero base command at all. B5's output stage zeroes a base command
below its 5 mm/s dead band (`dead_linear_mps`); its command before that stage is not logged, but a sent zero on a
plan that asks for 4-8 cm in its first second is the plan's slow start falling under the dead band.

**Reading:** the standstill is a fixed point between the layers: F2's plan starts at the measured (zero) speed, B5
tracks its start, its command falls under the dead band and is zeroed, so the measured speed stays zero. Which layer
should break it (F2's start-speed handling or waiting test, B5's dead band or breakaway) is a 5B design choice.

### C: the cost split (the pivot and the door)

Cost terms per seed (`--seed-detail`, Task 4b). The beam: the 20 low-beam runs rerun on build 8dcda54 (equal to the
baseline in outcome, time, clearance and infeasible cycles, 20 of 20). The door: its control fork.

- **The low-beam pivot:** over 56 cycles with a feasible `fold_back` beside the winner (55 unconverged), `fold_back`
  costs more by a median of 1.23 in comfort, 0.17 in progress and 0.16 in smoothness; the other terms differ by
  under 0.03. **The comfort pull decides it.**
- **The R4 door:** over 77 feasible vertical-swing candidates against the winning incumbent after the stop, the swing
  costs more by 0.51 in comfort, 0.25 in progress and 0.24 in goal position, and less by 0.12 in skid. **The comfort
  pull again**, with progress (the swing's start) second.

### Problems and resolutions (Task 2)

| Problem | Cause | Resolution | Where |
|---|---|---|---|
| 21 of 36 forks at a stall's onset came 13.5 to 102.5 s after the base had stopped; 7 of 9 `timeout_moving` runs had a parked base | The onset copied the session's progress rule, whose joint test counts a slowly drifting arm as progress | The stop is the base's progress alone (position or yaw, on the true pose) | Task 2 review C1 |
| 11 runs infeasible until the stop were not forked at their infeasibility | The streak had to run to the log's end; the 45 s after the stall carry ok, waiting cycles | The streak runs to the stop, within one medium period | Task 2 review I1 |
| `slack_stall` mixed B5's gate holding on infeasible plans with B5 creeping on ok plans, and counted B5's `infeasible` fallback as a hold | A catch-all class | Classes named by the layer that stopped (`infeasible_gate_hold`, `infeasible_b5_fallback`, `ok_plan_b5_creep`, ...); only B5's `holding` status is a hold | Task 2 review I2 |
| A bad summary crashed the report; a run directory without one was dropped silently | Not handled | Both are listed as `unreadable`, as are summaries without `reached` and logs without States | Task 2 review M3 |

### Problems and resolutions (Task 3)

| Problem | Cause | Resolution | Where |
|---|---|---|---|
| The door stall read as "F2 prefers a near-still incumbent" | The progress came from the cycles before the stop | Read over the cycles after the stop (1.82 m in 241 of 246) | executor's check before commit |
| The infeasible stalls read as "millimetres short" | Read at the fork only; the stop came after B5's 2 s fallback | The B5 fallback chain, with the violation at the stop (0.19-0.26 m) | Task 3 review C1 |
| The creep read as B5 held by its margin against F2's | The seed row's deepest row taken for the binding constraint | A speed standstill: the plan's slow start, followed literally; the margin is not active at the door | Task 3 review C2 |
| The door's comfort plan read as no vertical swing | The homotopy label ignores the wrist | Wrist_2 at 1.62 rad: a vertical panel | Task 3 review I1 |
| The door table counted runs that never started from the hallway | Route refusals to the hallway point | Counted over the runs from the hallway | Task 3 review I2 |
| "The per-seed cap binds"; "the budget cut 6 %" | Counted capped rows only | 130 converged infeasible against 138 capped; unscheduled seeds leave no row | Task 3 review I3, I4 |
| `fold_back` "20 times costlier" | Cold, unconverged attempts against a refined incumbent | Restated with the review's 1.8-3.5 at the decision | Task 3 review I5 |
