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
| e3_gap_straight_1 | 17/20 | 17/20 | infeasible_stall 1, slack_stall 2 |
| e3_gap_straight_2 | 17/20 | 17/20 | slack_stall 2, timeout_moving 1 |
| e3_gap_straight_3 | 13/20 | 13/20 | infeasible_stall 2, slack_stall 4, timeout_moving 1 |
| e3_gap_straight_4 | 2/20 | 2/20 | infeasible_stall 13, slack_stall 5 |
| e3_gap_turn_1 | 17/20 | 17/20 | infeasible_stall 1, slack_stall 1, timeout_moving 1 |
| e3_gap_turn_2 | 9/20 | 9/20 | infeasible_stall 5, slack_stall 4, timeout_moving 2 |
| e3_gap_turn_3 | 8/20 | 8/20 | infeasible_stall 2, slack_stall 6, timeout_moving 4 |
| e3_gap_turn_4 | 3/20 | 3/20 | infeasible_stall 14, slack_stall 3 |
| v2_room_r4_nav (lattice) | - | 20/20 | - |
| v2_low_beam_123_nav (lattice) | - | 20/20 | - |

**Main repeats E3's gap runs exactly.** All 160 gap runs equal their E3 sweep twins in the reached flag, the
simulated time, the minimum clearance, the count of infeasible medium cycles and the end status. Specs 2 and 4 (the
turn leg, the route search, the run-log schema) left these cells unchanged, so E3's numbers stand on main.

**Classes (initial; Task 3 checks them on the logs).** 74 failed gap runs: 38 `infeasible_stall` (F2 infeasible from
the fork point to the end, the streak starting no later than 2 s after the stall's onset), 27 `slack_stall` (stalled,
F2 not persistently infeasible), 9 `timeout_moving` (still progressing within 5 s of the end). No run was classed
`b5_refused` or `f2_waiting`. The fork point of each failed run is in `fork_points.json`.

**The door and the beam did not fail from the scenarios' starts.** R4 reached in 62-75 s and the beam in 33-37 s, on
every seed. The user's stall at R4 came from the hallway towards a goal in R4 (Task 3, A3); the user's pivot at the
beam is a passage, not a failure (Task 3, A4).

A third reference log (`e3_gap_turn_4-s14`, a stalled run) joins the two of Task 1 in `reference_logs.json` (the
Task 1 review, M1).
