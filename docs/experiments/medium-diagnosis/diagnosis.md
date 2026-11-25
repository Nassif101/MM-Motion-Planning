# The medium-planner diagnosis (the start of 5B): results

Spec: [2026-11-22-medium-diagnosis-design.md](../../superpowers/specs/2026-11-22-medium-diagnosis-design.md).
Plan: [2026-11-22-medium-diagnosis.md](../../superpowers/plans/2026-11-22-medium-diagnosis.md).

Three questions: why F2 fails on E3's gap cells; which layer refused at the R4 door the user saw stall; why F2 chose
the side pivot over `fold_back` at the 1.23 m beam.

## Summary: the answers and where 5B starts (Task 6, 2026-11-25)

The findings below are each tied to the stage that measured them; how sure each is follows it. All runs: realistic
plant, F2, the installed 0.05 m margins, build of this branch (main's behavior plus opt-in diagnostics; default logs
checked unchanged against main at every code task).

**1. E3's gap cells: the hand-over at the margin, not one setting.** (Baseline; Stage A; Stage B A and A2.)

- 51 of the 74 failed gap runs stall after F2 turns infeasible a few millimetres short of the 0.05 m margin (incumbent
  violation median 4 mm); after 2 s B5 drops F2's plan (`medium_max_infeasible_s`) and drives into the gap alone, or
  its slack gate holds. 21 more are the speed standstill (point 2), and 2 time out moving.
- Replayed at the same state (27 points), a 1-2 cm smaller margin or a larger iteration supply (budget 300 with a
  raised per-seed cap or waiting share) makes the plan feasible for a cycle at up to 10-11 of 27 points. In closed loop
  the plan turns infeasible again 0.4-6.4 s later, and the robot passes in 0-2 runs per setting. How far it is then
  outside the margin varies: 1-3.5 mm in 9 of the 21 margin forks and 7-130 mm in the other 12; 1.3-6.6 mm in 14 of the
  22 iteration forks and 21-152 mm in the other 8. Following F2 longer ends at B5's gate or in the standstill. The
  larger iteration supply took 0.30-0.38 s a cycle at the median (up to 0.58 s), measured with four runs in parallel on
  this host (an upper estimate) against the Mac profile's 0.4 s period.
- *How sure:* the 2 s hand-over holds over every fork point. "Just outside the margin" holds for about half the forks;
  in the others the plan goes deeper, so a tolerance band alone would not cover them. Not tested: two layers with
  different margins, a tolerance on "feasible", and combinations.

**2. The R4 door stall: a speed standstill between the layers.** (Stage A; Stage B B.)

- Reproduced once in 60 runs, from the hallway's east part; F2's plan there tilts the panel vertical (wrist_2
  1.62 rad), likely the user's case. 21 gap runs are classed the same way (18 of them replayed and confirmed).
- Every new F2 plan starts from the measured zero speed; B5 reads F2's speed at its own arc position, solves a command
  of 1.4-2.3 mm/s, below its 5 mm/s dead band, and sends zero (99.2 % of 81,494 cycles); its breakaway lift fires
  rarely (200 times, never at the door). F2's waiting test reads where the plan ends, so it never flags it.
- *How sure:* B5's own logged commands show it in all 19 replayed runs. The user may still recreate their own case in
  the viewer (they will, after this stage).

**3. The low-beam pivot: `fold_back` is tried rarely, hits the beam, and costs more.** (Stage A; Stage B C.)

- One pose seed per cycle in turn: `fold_back` in 138 of 654 approach cycles; infeasible in 76 (the beam 51, a hull
  bound 20); when feasible it is almost always unconverged and costlier, by comfort and progress about equally against
  the side swings that win.
- *How sure:* the counts hold; which term would decide a converged comparison is not measured.

**Not diagnosed here:** the late panel turn at doors (a cost-design question from the start: the comfort pull at
every sample, the clearance hinge only within 0.25 m, nothing rewarding an early turn). **Outside the medium planner:**
9 of the 60 door runs failed when the route search refused its route (swept validation), spec 2's layer.

**5B starting points** (each a design question, none measured as a fix):

1. *The hand-over at the margin:* what B5 does with a plan a few millimetres infeasible (follow it under its own
   checks, a tolerance band) instead of dropping it after 2 s; and whether F2 should plan with more clearance than B5
   enforces, so the two do not meet at the same boundary.
2. *The standstill:* B5's lookup of F2's speed (time-indexed, or with a lookahead), F2's start-speed handling and its
   waiting test, or B5's dead band and breakaway.
3. *Seed coverage:* every seed when the incumbent is near the limit, or a converged comparison of `fold_back` before
   the swing is decided; then the cost terms (comfort, progress), with the late panel turn.
4. *The iteration supply* only within the cycle's period (0.4 s on the Mac's 2.5 Hz profile, 0.2 s on the PC's 5 Hz
   one; the PC's timing was not measured here).
5. *The tightest E3 cells' geometry:* how much room a passage leaves the panel, before tuning for it.

**Preliminary data, not built on:** the runs of 2026-11-18 (`experiment_runs/e3/diag`: B5 alone on the gap cells, F2
with the 0.03 m margin in B5 or in F2) ran on d633e94, before specs 2 and 4; they agree with the above in direction (B5
alone fails every straight gap cell; a smaller margin moves passage both ways by cell).

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

## Stage B: the replays (Task 5, 2026-11-25; readings corrected by its review)

Runs: `bash docs/experiments/medium-diagnosis/diag_ensembles.sh forks` (build 8dcda54), 290 fork runs in 43 minutes,
no host sleep in the window; data `experiment_runs/medium-diag/forks_{one,closed,control}`, report
`experiment_runs/medium-diag/forks_report.md` (`tools/plant/fork_replay.py report`). Each fork reruns a failed
baseline run byte for byte to its fork point and changes one setting there (Task 4; the Task 4 review checked a fork
frame for frame against its baseline). **The control forks reproduce their original run: 27 of 27 (A), 19 of 19
(B).** The Task 5 review recomputed every table; the readings below are its corrections, each checked on the logs.

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

**The iteration tests are confounded.** In 12 of the 27 fork cycles the incumbent gets only 5 iterations under every
setting (the planner's share for an incumbent that waits, `kWaitingIncumbentIterations`); there the margin makes no
plan feasible (0 of 12 at 0.04 and 0.03 m). In the other 15 it does in 10 and 11. The per-seed cap of 30 inside the
default budget of 30 only moves iterations between seeds, and budget 300 is capped by 6 seeds x 15 iterations; the
cap and the budget raised together were not run. So "more iterations do not help" is not shown; only "these two
single settings do not". The "every seed" comparison is also empty at one cycle (that cycle already tries every
seed).

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

**Where the forks stop.** The margin forks do not get further: all 21 turn infeasible again 0.4-6.4 s after the
fork (11 at the very next cycle), the base having moved 0-171 mm, short of the original stop in every one; the
violation there, against the smaller margin, is again a few millimetres in 9 of 21 (1.0-3.5 mm). With B5's limit off,
B5 follows F2's infeasible plan, and 11 of 27 runs end in the speed standstill (B below), the others at the slack
gate (15) or elsewhere. One margin-0.03 fork held 17 mm inside an obstacle for about 45 s (2,243 contact ticks), the
"1" in the table.

**Reading:** E3's gap stalls are not caused by one setting. F2 ends a few millimetres short of feasible at whatever
margin it is given, at nearly the same place: the plan rides the margin boundary. A smaller margin moves the boundary
but not that pattern; following F2 longer ends at B5's gate or in the standstill. Whether more iterations would help is
open (the test above is confounded).

### A2: the iteration supply (user, 2026-11-25)

The single iteration settings above were confounded, so the supply was raised as one test at the same 27 fork points
(fork key `waiting_iterations`: a waiting incumbent's share, default 5; `fork_replay.py --mode iter`, `iter_closed`;
data `experiment_runs/medium-diag/forks_iter`, `forks_iter_closed`):

| Setting (together) | Forked plan feasible | Incumbent violation, median | Cycle wall time, median (max) |
|---|---|---|---|
| control | 0/27 | 10.5 mm | 0.095 s |
| budget 300 + per-seed cap 30 | 5/27 | 10.5 mm | 0.348 s (0.58 s) |
| budget 300 + waiting share 15 | 7/27 | 2.5 mm | 0.298 s (0.35 s) |
| budget 300 + cap 30 + waiting share 30 | 10/27 | 1.9 mm | 0.377 s (0.56 s) |

Closed loop, each setting that made a point's plan feasible (22 runs): 2 reached (both `e3_gap_straight_4`, one
point, two settings), the others stall again: infeasible (8 at B5's gate, 8 on B5's fallback), 3 creep, 1 other; no
contact.

**Reading:** more iterations help a cycle as much as a smaller margin does (up to 10 of 27 feasible), and the
waiting incumbent's 5-iteration share is part of why it was short; but as with the margin, the plan does not stay
feasible, and the passage still fails (2 of 22). The cost is compute: 0.30-0.38 s a cycle at the median and up to
0.58 s, against the 0.4 s period of the Mac's 2.5 Hz profile. These wall times were measured with four runs in
parallel on this host (an upper estimate; a single run would be faster), and the closed-loop forks apply plans at the
profile's fixed latency, so whether this much iteration fits in real time is not settled here.
Neither the margin nor the iteration supply is the lever by itself; together with the standstill (B), this points 5B at
the hand-over between F2 and B5 (what B5 does with a plan a few millimetres infeasible, and F2 planning to the
boundary of the same margin B5 enforces).

### B: the speed standstill (18 creep runs, 5 per cell, and the R4 door run)

A control fork at the stop with B5's diagnostics (B5's solved command before its output stage, `diag_plan_u0`, and
the command it sends, `diag_cmd`): over the 81,494 cycles from 1 s after the stop **B5 sends a zero base command in
99.2 %**. Its own solved forward command is a median 1.4-2.3 mm/s (the three runs checked; the review: a median 0.35 of
the dead band over the runs), below the 5 mm/s dead band (`dead_linear_mps`), which zeroes it. The breakaway lift fires
200 times in all (180 of them on the yaw), on 14 of the 19 runs, without getting the base going; the door run never.
B5's own 2 s plan moves 4-28 mm while F2's plan moves 75-466 mm over the same 2 s; while driving, the two agree.

**Reading:** the standstill sits in B5's use of F2's plan: B5 looks up F2's speed by arc length at its own position
(`formulation.cpp`, the medium reference, lines 611-628), and every new F2 plan starts there from the measured zero
speed; B5's command stays below the dead band and is zeroed, so the measured speed stays zero. F2's waiting test reads
where its plan ends and never flags it. The 5B candidates are the layers that could break this: B5's lookup of F2's
speed (a time-indexed reference, or a lookahead), F2's start-speed handling or waiting test, B5's dead band or
breakaway.

### C: the cost split (the pivot and the door)

Cost terms per seed (`--seed-detail`, Task 4b). The beam: the 20 low-beam runs rerun on build 8dcda54 (equal to the
baseline in outcome, time, clearance and infeasible cycles, 20 of 20). The door: its control fork.

- **The low-beam pivot:** over 56 cycles with a feasible `fold_back` beside the winner, `fold_back` costs more by a
  median of 1.23 in comfort, 0.17 in progress and 0.16 in smoothness. That median rests mostly on 32 cycles against
  a comfort winner, before any swing; against the side-swing winners themselves comfort and progress differ about
  equally (the review: +0.47 and +0.46), and every `fold_back` solution there is unconverged. **The comfort pull and
  the progress term together, against unconverged attempts:** the cost split shows the terms, not a decided cause.
- **The R4 door:** over 79 feasible vertical-swing candidates against the winning incumbent after the stop, the swing
  costs more by 0.51 in comfort, 0.25 in progress and 0.24 in goal position, and less by 0.12 in skid. This explains
  why F2 keeps the comfort-labelled incumbent, not why the robot stays: the stop is the speed standstill (B).

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

### Problems and resolutions (Task 5)

| Problem | Cause | Resolution | Where |
|---|---|---|---|
| "The passage gets tighter beyond that point" | Read from the classes, not from where the forks stopped | The margin forks turn infeasible again within 0.4-6.4 s, 0-171 mm on, a few mm short again: the plan rides the margin | Task 5 review C1 |
| "B5's command before the dead band is not logged" | The diagnostics key `diag_plan_u0` was missed | Read: 1.4-2.3 mm/s, under the 5 mm/s dead band; the cause placed in B5's lookup of F2's speed | Task 5 review I1 |
| "More iterations do not help" | 12 of 27 fork cycles give the incumbent 5 iterations; the cap stays inside the budget of 30; budget 300 stops at 6 x 15 | Restated as untested (confounded); budget and cap together not run | Task 5 review I2 |
| "The comfort pull decides" the pivot | The median mixed cycles before the swing with unconverged attempts | The terms reported, comfort and progress together, no decided cause | Task 5 review I3 |
| A sustained contact shown as "1" | The table counts runs with a contact | Named in the text (17 mm, about 45 s) | Task 5 review I4 |
