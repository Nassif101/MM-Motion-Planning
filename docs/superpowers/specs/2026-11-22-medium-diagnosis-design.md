# The medium-planner diagnosis (the start of 5B): design

Date: 2026-11-22. Branch `medium-diagnosis` (off main 48c1aa4). Status: written up after the builder check; awaiting the user's approval.

## Intent

Find out why the medium planner (F2) and the layer below it (B5) stall or choose oddly in three observed cases, so
that 5B's fixes target measured causes. The order is the user's (roadmap decision log, 2026-11-22): this diagnosis,
then 5B, then perception.

The three questions:

1. **E3's gap cells.** The failed runs stall near the obstacle, with F2 infeasible in most cycles (a median of about
   69 % over all 666 failed runs of E3's sweep: 16 cells, both plant modes, four margins; e3.md). Why is F2 infeasible
   there: too few iterations, the margin, the seeds tried, or a geometry no seed reaches?
2. **The user's door stall** (roadmap step 4, 5B, 2026-11-21). F2's ghosts showed a correct vertical swing through a
   door, but nothing executed it. The user's account (2026-11-22): the small door inside the building ahead of the wide
   entrance (door A), not door B; the robot came from the hallway towards a goal inside the small room behind it; the
   realistic plant with lattice routes. That is room R4's 1.05 m door on the hallway's north wall (x 7.825), the
   route of `v2_room_r4_nav` *(my identification; R3 and R5 have 1.30 m doors)*. Which layer refused: B5's slack gate or hold, a medium fallback such as `no_plan`, or
   plan adoption?
3. **The user's low-beam pivot** (same place; the realistic plant). At the 1.23 m beam the robot passed with a side
   pivot. Why did F2 not
   choose its `fold_back` seed: never tried in that cycle, infeasible (static torque or a collision), or feasible but
   costlier (the comfort pull, progress, the homotopy hysteresis)?

## Goals, non-goals, anti-goals

Goals (the user's choice A, 2026-11-22):

- For each question, the layer and the decision that fails, with evidence: log counts and same-state replays.
- A same-state replay may change **one setting** at a time (the iteration budget, a margin, the seeds tried). Such a
  check tells causes apart, so it counts as diagnosis.
- A written result per question and a short list of what 5B should consider, each item tied to its evidence.

Non-goals:

- No fixes, no tuning and no new seeds or cost terms. Fixes are chosen in 5B's design.
- The late panel turn at doors (roadmap step 4, 2026-11-20) is not diagnosed. Its cause is already known: nothing in
  the cost rewards turning early. It stays a 5B design question.
- B5-alone holds are not studied. F2 resolves them (the user, E3, 2026-11-15).
- The viewer's deferred items (the active-margin overlay and F2's losing seeds' paths) are not built here. Stage A
  says whether question 3 needs them, and the user decides then.

Anti-goals (the work fails if these happen, even when it runs):

- Default behavior changes. With the new options off, run logs are byte-identical to main's for the same seed.
- A cause is claimed from a single run. Each cause is checked across the failing runs of a cell, as
  [tune-with-ensembles] asks: a chaotic loop flips on rounding.
- The replay matrix grows into 5B's tuning sweep.

## Constraints

- Campaign runs plan on an iteration budget of 30 with a 10 s deadline (`session.cpp`, `iteration_budget`), so a run
  repeats byte for byte on this host. The replay depends on that.
- Runtime and session code stays in C++; the analysis tools stay in Python in `tools/plant/` (project rule).
- Host: a fanless MacBook Air. No Play mode during timed runs; long runs under `caffeinate -i` and checked against
  `pmset -g log`; a new output directory for every changed cell or setting (E3 review I1).
- Process: one commit per task or fix on this branch, never on main; the user reviews each task from a
  self-contained prompt (the E2 process); `docs/reviews/` is git-ignored and never committed or cited.

## Approaches

Considered (user's pick: 1, preceded by 3's log reading):

1. **Re-run to the stall, then change one setting (chosen).** A session option forks a run at a time T: from the
   first medium cycle at or after T, one setting differs. It answers both "does this one plan become feasible?" and
   "does the robot then pass?". It relies on the byte-identical reruns. Cost: a small session option, one richer log
   record and one tool. Each replay re-simulates up to T, from seconds to about a minute.
2. **Save the planner's state, solve it offline.** Fast per query, but the snapshot must hold every piece of the
   planner's internal state (the incumbent, the warm start, the seed rotation, the hysteresis). A missed piece silently
   changes the answer, and it gives no closed-loop result. Not chosen.
3. **Logs only.** Free, and enough for "which layer refused". It cannot test a setting, so it cannot separate the gap
   cells' causes. Kept as the first stage of 1.

## What exists today

- The Plans frame carries each cycle's status, winner, `full_multistart`, `waiting`, `homotopy_switched` and one row
  per seed: `[name, merit, feasible, progress_m, merged]` (`session.cpp`, `seed_rows`). It does not carry the seed's
  slack, which constraint failed, its iterations or its homotopy, although `SeedStats` and `ScpResult::violations`
  have most of them.
- The State frames carry B5's fallback, the slack gate `[slack, violation, unsafe]`, dwell, and `execution`
  (tracking or holding). `--prediction` adds B5's solved plan (spec 4B-2).
- `seeds_per_cycle: 1`. Beside the incumbent, F2 tries one pose seed per cycle, in turn; a full multi-start runs on a
  new route, on `request_full_multistart()`, and every `full_multistart_period_s` (2 s) only with `seeds_per_cycle: 0`
  (which tries the incumbent alone in the other cycles). So `fold_back` may be absent from the cycle that decided the
  pivot.
- The iteration budget is shared by the cycle's seeds in order: the incumbent first, then the pose seeds. A seed the
  budget cuts is dropped for that cycle and resumes from its iterate next time (`planner.cpp`, `plan`). More seeds at
  budget 30 starve the later ones (`fold_back` is the fourth pose seed), so the seeds tried and the budget cannot be
  changed independently at 30.
- With a budget set, the session applies the plan at a fixed latency (the profile's `solve_cap_s`, `session.cpp`)
  whatever the cycle's real wall time; the 10 s deadline stays as a wall-clock backstop.
- F2's arm margin also sets the clearance hinge's reach (`clearance_target_m - arm_margin_m`) and the seeds' tight-speed
  scaling (`problem.cpp`), not only the collision rows.
- The demo server writes one log per session (spec 4C), and the viewer replays logs.
- Data from 2026-11-18 in `experiment_runs/e3/diag` (not written up): 480 runs on E3's gap cells. B5 alone 0/20 on
  every straight gap cell, 18-20/20 on turn cells 1-3, 0/20 on turn 4. F2 with the 0.03 m margin moved to B5 or to F2
  changes passage in both directions, by cell. **It ran on d633e94**, before specs 2 and 4 changed the session (the
  turn leg, Smac routing, the run-log schema), so it is recorded as preliminary and is not built on.
- The `--iteration-budget` spike (two lines, uncommitted, worktree `.worktrees/diag`, branch `diag-medium-budget`) is
  superseded by Stage B's setting.

## Design

Three stages, each ending at a checkpoint with the user.

### Stage A: read the logs (no planner changes)

- **A1. Baseline on main.** Rerun E3's eight gap cells (realistic, the 0.05 m margin, 20 seeds each, with
  `--prediction`; their own scenario routes, as in E3), and `v2_room_r4_nav` and `v2_low_beam_123_nav` (realistic,
  lattice routes as in the user's demo session, 20 seeds each, with `--prediction`). Compare the gap cells' stall
  rates with E3's sweep (m050, 2026-11-16). A changed rate is a finding in itself.
- **A2. `tools/plant/medium_diag.py`.** Per run, from the log: where and when the robot stopped; the first
  persistently infeasible cycle (the fork point; re-implemented from the 2026-11-18 `failed_order` data, which no script
  in the repository produces: the first infeasible cycle after which the plan stays infeasible until the stop); the
  seeds around it; B5's state (gate, hold, fallback, executing or holding); a stall class. A failed run that never
  becomes persistently infeasible (a stall on a slack plan, a timeout; 22 of 74 failed gap runs at 0.05 m on
  2026-11-18) is in scope (user, 2026-11-22): its fork point is the stall's onset. Out: a per-cell table and the list
  of fork points.
- **A3. The door stall.** Reproduce it headless first, from A1's `v2_room_r4_nav` runs (if none stalls at R4's door,
  a start in the hallway with `plant_campaign --goals`). If that does not reproduce it, the user recreates it once in
  the viewer (`--mode realistic --routes lattice`); the demo server's log captures it. Read which layer refused from
  the log.
- **A4. The pivot.** From A1's beam runs: per cycle on the approach, whether `fold_back` was tried, feasible, and its
  merit against the winner's and the hysteresis.

**Checkpoint A:** the user sees the classification and the answers that logs alone give. Questions 2 and 3 may close
here. The user decides which fork questions Stage B runs.

### Stage B: the fork

- **B1. The richer seed row, behind `--seed-detail` (off by default, on in every diagnosis run).** Append to each
  seed's row: slack, the violation per kind (collision, torque, speed, hull), iterations, converged, resumed, homotopy,
  and the deepest collision row (robot part, obstacle, distance, time along the horizon). Off, logs are byte-identical
  to main's; on, fields are appended, so readers of the first five keep working (the viewer's seeds table is checked).
  *(Builder check: the anti-goal and an always-on row could not both hold; my call.)*
- **B2. `plant_campaign --fork-at T --fork-set key=value` (repeatable).** From the first medium cycle at or after T,
  the settings apply. Keys: the medium iteration budget, the medium arm margin (the whole config value, as E3's
  `medium_margin_0xx` configs set it: the collision rows, the clearance hinge's reach and the tight-speed scaling move
  together; written up as such), B5's arm margin, `full_every_cycle` (a full multi-start every cycle, through
  `request_full_multistart()`) and `seeds` (the pose seeds tried, e.g. `fold_back` alone). `--fork-cycles 1` runs one
  forked cycle, logs it and stops; without it the run continues to its end. The fork is recorded in the Hello and the
  summary. Each forked cycle logs its wall time and whether the 10 s deadline cut it (a cut cycle is not reproducible
  and is flagged, not counted).
  Tests: a run without the options is byte-identical to main's; a forked run is byte-identical to the unforked one up
  to T; a fork with no setting changed reproduces the original run whole.
- **B3. The replay matrix.** For each fork point (at most five failing runs per cell, in seed order), against the
  control (budget 30, margins 0.05, one pose seed per cycle):
  - budget 60, 120 and 300;
  - medium margin 0.04 and 0.03;
  - B5 margin 0.03 (the 2026-11-18 data says B5's margin moves passage);
  - the seeds, compared at budget 300 so that the budget does not starve them: budget 300 with every seed every cycle
    against budget 300 alone. One change against that row, so still a single setting.
    *(Builder check: at budget 30 the seeds and the budget cannot be separated; my call.)*

  One-cycle forks first (seconds each). Closed-loop forks only for the settings that made the one-cycle plan
  feasible. Out: per fork point and setting, the one-cycle feasibility and violation, then reached, stalled and the
  minimum clearance.
  Closed-loop forks keep the session's fixed latency, so a large budget acts as fast as budget 30. The write-up reports
  each setting's measured wall time per cycle against the 0.4 s period and marks what would not fit the rate on this
  host. *(Builder check: my call.)*

**Checkpoint B:** the user sees the matrix.

### Stage C: the write-up

`docs/experiments/medium-diagnosis/diagnosis.md`: per question, the cause (layer and decision), the evidence, how
sure it is, and what 5B should consider; the 2026-11-18 data as a preliminary note; a problems-and-resolutions table.
A roadmap decision-log entry. Thesis evidence rows only if the user asks.

## Left to the builder

- The exact stall classes in A2, after looking at the logs.
- Where the fork hooks into the session and how a setting reaches F2 and B5 mid-run (F2's margin is in its problem,
  built at start; a rebuild at T is acceptable if it keeps the incumbent and the rotation).
- The run count of B3 after Stage A, within the cap above; a time estimate before starting it.
- Tool names other than the two above.
