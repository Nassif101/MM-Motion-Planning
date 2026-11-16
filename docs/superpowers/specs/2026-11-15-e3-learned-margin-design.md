# E3: learned tracking margin (execution fixes, sub-project 3)

Date: 2026-11-15. Branch: `e3-learned-margin` (off main 73248a0). Status: design approved section by section (user,
2026-11-15); this document awaits review.

## 1. Context and scope

The execution fixes (roadmap "Priorities after Phase 5A", step 3) are split into five sub-projects (E1 spec, Section
1). E1 (the slack gate's absolute rule, the infeasibility fallback; ADR 0017) and E2 (the tracking model in B5's
prediction; ADR 0018, [e2.md](../../experiments/execution-fixes/e2.md)) are accepted and merged. E3 is roadmap part
(c) of the tracking-aware B5: a state-dependent tracking margin in B5's collision rows, learned from MuJoCo plant
runs as a function of the commanded motion, in place of the fixed margin.

**Where the margins stand.** Arm and panel rows keep 0.05 m (`collision.margin_m`); base rows against obstacles keep
0.12 m (`collision.base_margin_m` = padding 0.02 + StopZone 0.02 + measured lidar intrusion 0.08 m). A larger fixed
arm and panel margin, 0.065 m, stalled the doors (Phase 4B-B). After E2 the one-stage lateral error is 0.31-0.53 mm,
and under low_beam_123 the panel is still 5-12 mm inside its margin in about half of F2's runs, in the baseline and
the candidate alike (e2.md, "CP4 checks"). How much of what remains is tracking error, and how much is planned slack
or linearization error, has not been measured.

**The goal (user, 2026-11-15): passage.** A margin that follows the commanded motion lets B5 pass tighter gaps than
any fixed margin does at the same contacts and realised clearance: small where tracking is good (slow, straight, the
arm still), larger where it is poor (turns, arm swings).

**The comparison.** A learned margin is only worth building if it beats a smaller fixed margin. The baseline is
therefore a sweep of fixed margins, not 0.05 m alone, on cells that separate the two: the same gap met in different
motion (Section 3.2).

**A go/no-go.** Stage 1 measures and sweeps; the user rules at CP1 whether the learned margin is built (Section 3.5).

**In scope:** a per-row trace in B5 and its offline analysis; the tightness cells (axis-aligned, on the plant only);
the fixed-margin sweep; after a go: the margin model, its fit and calibration, the margin in B5's rows (approach A,
then B), the judgment on the plant's ensembles.

**Out of scope:** the medium planner's formulation (its margin's value follows the variant, decision 6);
continuous-time collision rows and the rows' linearization error (E4: measured here, not learned); Unity, the ROS
node and the B5 re-freeze (E5); irregular passages that need a tilted panel (campaign revision, roadmap step 7); the
approach and hold policy (Phase 6); changes to the plant's physics.

**Literature (2026-11-15; abstract level for the items marked \*).** Margins learned as a function of the planned
motion exist as a method: Compton et al. (Dynamic Tube MPC, a hopping robot, tube learned from parallel simulation:
a fixed tube is infeasible or conservative in clutter), Fan et al. (RSS 2020, quantile-regression tubes). Bounds from
closed-loop tracking data calibrated by conformal prediction and used as a collision buffer: Wei, Gan and Burdick\*
(a ground vehicle). Disturbance bounds estimated iteratively from closed-loop data: Benders et al.\* (2025, a
quadrotor in Gazebo). An analytic state- and input-dependent bound for a manipulator: Wullt et al.\* Known weak
points: fixed worst-case margins block narrow passages; coverage statements assume the calibration data resembles
deployment, while the margin itself changes the motion; a margin that depends on the decision variables makes the
problem harder to solve; margins learned in simulation carry the gap to the real system. Not found in these searches
or the check of 2026-11-03 (`thesis/literature/review/novelty-boundary-2026-11-03.md`): a learned tracking margin in
the collision rows of a whole-body mobile-manipulator MPC carrying a payload. The contribution is the application and
its evaluation against fixed margins; the method's origin is cited (Fan et al., Compton et al.).

**Decisions (user, 2026-11-15):**

1. Purpose: passage (above). Safety is a constraint on the result, not its headline.
2. The baseline is a sweep of fixed margins. The measurement and the sweep end in a go/no-go at CP1.
3. Which rows take the learned margin (arm and panel only, or the base's too) is ruled at CP1 from the trace.
4. Passage is measured on a tightness sweep: cells tighter than today's, sensible for a construction site, in pairs of
   the same geometry and different approach motion. The existing cells are the no-regression guard, not a result.
5. Irregular passages are deferred to the campaign revision; the learned-against-fixed comparison is rerun on them
   there (a roadmap line).
6. The medium planner's margin value follows the variant: the fixed value in the sweep, the model's floor with the
   learned margin. Its code and formulation do not change.
7. Method: approach B (the margin and its slope inside the rows), built through approach A (the margin from the
   previous plan, as a constant) as a step and an ablation. Approach C (the bound as a state with a command history)
   is the named escalation, taken only if CP1 shows the error depends on history.

## 2. Structure

Two stages; Stage 2 only after a go.

| Stage | Content | Ends at |
|---|---|---|
| 1 Measure | the row trace and its analysis; the tightness cells; the fixed-margin sweep | CP1 (go/no-go) |
| 2 Build | the margin model, fitted and calibrated; approach A in B5 | CP2 |
| | approach B; ensembles on the tightness cells, the existing cells, three grips, fresh seeds | CP3 |
| | acceptance, ADR 0019, documents | CP4 |

**If CP1 is a no-go,** E3 closes with the measurement as its result (e3.md, the roadmap). Whether the best fixed
margin of the sweep replaces 0.05 m is a separate ruling. The trace and the tightness cells stay, for E4 and the
campaign revision.

## 3. Stage 1: the measurement

### 3.1 Row trace

B5's own predictions are not in the run log (the Plans frames are the medium planner's; `diag` is three numbers). A
trace, behind the diagnostics switch, records per cycle and active collision pair:

- the pair, its row type (arm, panel, base) and its distance at the current state;
- the plan's predicted distance at lookaheads 0.1, 0.2, 0.5 and 1.0 s;
- the commanded motion at those stages: base speed, turn rate, the six joint speeds.

The trace must not change a run: with it on, the frames other than the trace are identical to a run with it off.

### 3.2 Tightness cells

Cells where the arm and panel rows bind (a narrower door binds on the base's 0.12 m, which is lidar and StopZone
allowance: 0.73 m of wheels and two base margins need 0.97 m). Two geometries: an overhead beam; a gap beside the
panel. Each in two approach motions: straight and slow; just after a turn. Each in four tightness steps, starting at
today's tightest. 16 cells, axis-aligned, built on the plant's site model only. The dimensions are fixed in the plan
after a dry run finds where the 0.05 m margin stops passing; each cell passes at its easiest step with 0.05 m.

Why pairs: any small margin passes a tight gap, so tightness alone does not separate a learned margin from a small
fixed one. The same gap in different motion does: the hypothesis is that a small fixed margin passes the straight
approach and intrudes after the turn, a large one holds at both, and the learned one passes both.

### 3.3 Fixed-margin sweep

Arm and panel margin 0.05, 0.04, 0.03 and 0.02 m, B5 and the medium planner at the same value (decision 6), F2, 20
seeds per cell, mu 0.6: about 1,300 runs. The base margin stays 0.12 m; the base rows are judged from the trace.

**Safe** (a definition that does not depend on the margin): zero contacts and a realised clearance never below 0.02 m.
**The best fixed margin:** the value with the most passages among those safe on every tightness cell.

### 3.4 Analysis

A Python tool beside `tools/plant/model_rollout.py`. Per trace record, three errors, positive when the pair is closer
than predicted:

- **tracking:** B5's model rolled out with the executed commands against the true state, mapped to the pair's
  distance. B5 changing its plan at the next cycle is not in it;
- **linearization:** the row's linear distance against the true distance at the planned stage. Reported and handed to
  E4;
- **total:** what the margin covers today.

Reported per row type, lookahead and motion regime (slow and straight; turning; arm moving), with the dependence on
the previous commands (history) and a first fit of the Section 4.1 model on a held-out split.

### 3.5 CP1 criteria

All four for a go; the numbers are fixed here, before the data.

| Criterion | Go if |
|---|---|
| Size | the 95th-percentile tracking error in the worst motion regime is at least 5 mm at the lookahead the margin must cover |
| Dependence | that percentile is at least twice, and at least 5 mm above, the slow-and-straight regime's |
| Predictability | on held-out runs a simple model reaches 95 % coverage with a mean margin at least 30 % below the constant with the same coverage |
| Trade | at the same tightness the after-turn cell needs a fixed margin at least 5 mm larger than the straight cell to stay safe |

Also ruled at CP1: the lookahead the margin covers; the rows (decision 3); the feature set; whether history matters
(decision 7's escalation).

## 4. Stage 2: the learned margin

### 4.1 The model

For a pair at a horizon stage, an upper bound on the tracking error: its 95th percentile. A row's margin is

    margin = clamp(floor + bound, floor, upper)

- **floor:** what is not tracking (linearization, geometry): the smallest fixed margin that stayed safe in the
  slow-and-straight cells of the sweep;
- **bound:** a constant per row type plus non-negative weights on smooth features of the stage's commanded motion.
  Candidates: base speed squared, turn rate squared, arm joint speeds squared, the lookahead. CP1 picks the set;
- **upper:** a configuration number, set at CP2.

### 4.2 Fit and calibration

A Python tool, offline. Quantile regression with non-negative weights on plant runs at mu 0.4, 0.6 and 0.8 pooled
(the robot does not know the grip). Calibration on held-out runs, split by run, not by sample. Coverage reported
overall and per motion regime, cell and grip.

### 4.3 In B5

- **A:** each row's margin from the previous solution's commands at that stage, a constant in the QP.
- **B:** the row also carries the margin's slope with respect to the stage's commands, so the solver can trade speed
  for clearance. The state stays 9.
- The slope linearizes a curved function and can understate the margin when the command changes much: the slack gate
  checks every plan against the exact margin at the plan's own commands.
- The medium planner's margin value is the floor.
- A and B are switches; with no margin file B5 uses the fixed margins and reproduces today's runs frame by frame.

### 4.4 The data file

`tracking_margin_mujoco.yaml`, loaded like E2's tracking model; its hash is in the run log; the loader refuses unknown
features and a file that does not match its hash.

### 4.5 Closed-loop shift

The margin changes how B5 drives, and so the errors it was fitted on. Coverage is measured again on runs driven with
the margin. Below target: one refit on those runs. A second miss is a finding for the user, not another iteration.

### 4.6 Not built

No error state and no command history (approach C), no online adaptation of the margin, no neural network. Each is
an escalation that needs evidence from CP1 or CP3.

## 5. Acceptance

**Stage 1 (before CP1):** the trace changes no run (Section 3.1); the analysis tool reproduces known errors on a
synthetic log; the tightness cells load and pass at their easiest step with 0.05 m.

**Stage 2:**

| # | Criterion |
|---|---|
| M1 Passage | In the after-turn cells the learned margin (B) passes at least one tightness step beyond the best fixed margin, or significantly more runs at the step where the fixed one starts to fail (Fisher, p < 0.05). In the straight cells it is no worse |
| M2 Safety | Zero contacts and a realised clearance never below 0.02 m in every run with the learned margin |
| M3 Coverage | On held-out runs driven with the margin: 95 % overall within 3 points; no motion regime or grip below 90 % |
| M4 No regression | The existing campaign and grip sets against E2's baselines: no significant loss of reached in any cell, zero contacts, no step locks |
| M5 Ablation | B against A reported on passage, holds and time through the gap; B no worse than A |
| M6 Identity | With no margin file B5 reproduces today's defaults frame by frame |
| M7 Timing | B5's cycle p99 rises by at most 0.5 ms (4.8 ms today, of 20 ms) |
| M8 Fresh seeds | Seeds 21-40, never used to fit or choose, confirm M1 and M2 |
| M9 Decisions | The twins' flip rate is not above E2's |

## 6. Testing

- **C++:** a row's margin and slope against finite differences; the gate's exact-margin check; the loader's
  refusals; the identity case against the golden data.
- **Python:** the fit recovers known weights from synthetic data; the calibration split keeps a run on one side; the
  coverage report; the analysis tool on a synthetic log.
- **Ensembles:** 20 seeds per cell, judged on distributions; three grips for M3, M4 and M8. The host on power with the
  lid open, no Play mode alongside.

## 7. Process and records

As in E2: one commit per task; after each task a self-contained review prompt for the user (the reviews stay outside
git); rulings at the checkpoints; problems and decisions recorded as solved. Records:
`docs/experiments/execution-fixes/e3.md` (with the problem, cause and resolution table), ADR 0019 at acceptance, the
roadmap's E3 entry and the line on rerunning the comparison on the irregular passages (decision 5).

## Amendment 1 (2026-11-16): the cells as built, two controls, CP1's criteria

After the Task 3 review (two protocol findings) and the user's rulings of 2026-11-16. It changes Sections 3.2, 3.3
and 3.5; where they differ, this amendment holds.

**The cells as built (3.2).** The gap is two braces raised above the base (0.75 to 2.2 m), their faces 0.30, 0.27,
0.24 and 0.21 m from the route; the beam's underside is 1.25, 1.23, 1.21 and 1.19 m; in a turn cell the obstacle's
centre is 0.5 m after the turn's end (at 1.5 m the turn was over before the rows were active). The levels are an
exploratory grid: only the 0.05 m margin was piloted, on 5 seeds. A cell where every margin passes, every margin
fails, or the outcome is not ordered in the margin is reported as uninformative and takes no part in a threshold. No
threshold is chosen from the main sweep's results after the fact.

**Why two controls.** The review showed that (1) a straight-against-turn difference at a fixed margin cannot be
attributed to tracking: the turn cell is also geometrically harder (a shorter lead-in, the corner inside the
horizon); (2) the slow-and-straight regime CP1 compares against has no replicated support in the cells: its only rows
came from one run's 53 s pause.

**Control 1: the ideal-mode twin (user ruling).** The sweep of 3.3 runs twice: on the realistic plant, and on the
plant's ideal mode, which executes B5's commands as sent, with B5 predicting with the unicycle (`--tracking-model
none`). Same cells, margins and seeds. What differs between the two at a cell and margin is tracking; what remains in
the ideal runs is geometry and planning. A check on the method itself: in the ideal runs `e_track` must be near zero.

**Control 2: the slow passage (user ruling).** Each of the eight straight cells is also run with the medium planner's
route reference at 0.12 m/s (`configs/medium_slow.yaml`, `speed_fraction` 0.4 against 0.8), realistic, margin 0.05 m,
20 seeds, with the row trace: the same obstacle, passed slowly. A 5-seed pilot must show slow rows against the
obstacle in every run before the sweep; its result is reported to the user first.
**Pilot result (2026-11-16): not usable.** At 0.12 and at 0.18 m/s the gap cell stops passing (0 and 1 of 5 against
5 of 5 at 0.24 m/s) and the slow rows are pauses, not passages (e3.md, "Slow-control pilot"). **User ruling (2026-11-16):** the slow
control is dropped; the sweep runs its realistic and ideal legs (2,560 runs, about 4 hours); CP1 reports Dependence
as unavailable, Predictability carries the question of whether the error depends on the commanded motion, and the
slow-speed failure goes into the CP1 report as a finding against the premise that passage is easier when slow.

**Statistics.** Rows of one run are not independent. Every CP1 number is computed per run first (a run's percentile
over its rows against the cell's obstacle) and then summarised over runs (median and range); a comparison needs at
least 5 runs with rows on each side, else it is reported as unavailable, neither passed nor failed.

**CP1's criteria (3.5), restated.** All four for a go.

| Criterion | Go if |
|---|---|
| Size | unchanged: the 95th-percentile tracking error in the normal passages (straight and turn cells, realistic) is at least 5 mm at the lookahead the margin must cover, as a median over runs |
| Dependence | that figure is at least twice, and at least 5 mm above, the same figure in the slow passages of the same cells |
| Predictability | unchanged, with the split by run |
| Trade | on at least one informative cell the realistic plant needs a larger fixed margin than the ideal plant to stay safe, or passes significantly fewer runs than the ideal plant at the same margin (Fisher, p < 0.05) |

The straight-against-turn difference is reported per mode as a description and is not a criterion. If the ideal plant
passes no tighter than the realistic one anywhere, tracking is not what limits passage: a no-go on its own.

**Runs.** Realistic 16 cells x 4 margins x 20 seeds (1,280); ideal the same (1,280); slow 8 cells x 20 seeds (160,
about 80 s each). About 5 hours at 4 jobs, in sittings. The row trace on the realistic 0.05 and 0.03 m legs, the slow
runs, and the ideal 0.05 m leg (for the check).

## 8. Risks

- **Nothing to learn.** After E2 the tracking error may be small everywhere: the no-go of CP1, a reportable result.
- **The floor dominates.** If linearization error sets the floor near 0.05 m, the learned part has no room: reported
  at CP1 with E4's share.
- **The medium plan decides the passage.** With its margin at the floor the medium planner may thread gaps B5 then
  meets at speed: M2 and M4 judge it; the medium planner's own margin model is 5B's question.
- **Uneven coverage.** Pooled coverage can hide a regime that is under-covered: M3 reports per regime and grip.
- **Plant to Unity.** The margin is the plant's; Unity gets its own file or the fixed margin at E5.
