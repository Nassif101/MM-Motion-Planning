# E1: slack gate and decision robustness (execution fixes, sub-project 1)

Date: 2026-11-08. Branch: `e1-gate-robustness` (off main afdab32). Status: design approved section by section
(user, 2026-11-08); this document awaits review.

## 1. Context and scope

The execution fixes (roadmap "Priorities after Phase 5A", step 3) are split into five sub-projects, each with its own
spec, plan and execution (user decision 2026-11-08):

| | Sub-project | Content |
|---|---|---|
| **E1** | gate and decision robustness (this spec) | the slack gate's lock at a margin; decisions that flip under rounding-level perturbation; findings (c) and (d) of the plant's gate R |
| E2 | tracking-aware prediction | delay, base lag, understeer and sideslip in B5's model; arm closed-loop lag; arm motion feedforward in the plant |
| E3 | learned tracking margin | a state-dependent margin learned from plant runs in place of the fixed 0.05 m |
| E4 | continuous-time collision | the Phase 4B-C mid-stage rows or swept volumes |
| E5 | re-freeze | Unity door-cell confirmation, B5 re-freeze, tag, comparability log |

E1 comes first because the gate's lock makes every hold at a margin final, which hides on the ensembles whether E2's
tracking changes help.

**In scope:** a new slack-gate allowance rule; a per-cycle decision trace in the plant's run logs; a twin-run
divergence tool and the flip-rate metric; commitment fixes (hysteresis, enter/leave thresholds, dwell, tie-breaks)
for the flip sites the tool finds; the attribution of finding (c) (F2 runs starting left at gate_right_swing fail 3 of
4: seeds s4, s7, s19 fail, s11 passes) and finding (d) (door_105 F2 seed s20 touches the door frame, 0.4 mm, after 862
executed slack plans and 3920 cycles following infeasible medium plans).

**Out of scope:** modelling sideslip, understeer or lag (E2); store_room B5 reaching its goal (E2's target: E1 only
removes the lock); Unity runs and the B5 re-freeze (E5); route replanning, the collision monitor and Nav2 recoveries
(the plant's specs 2 and 3).

**Success (user decision 2026-11-08):** no permanent locks, flip-prone decisions made to commit (flip rate measured
down), no regressions on the replica ensembles. Findings (c) and (d) are investigated and fixed in E1 when their cause
is a decision; otherwise their cause is recorded and handed to E2 or E4 (user decision 2026-11-08).

## 2. The gate lock (measured)

`gate_allowance` (`mobile_manipulator_mpc/src/controller.cpp`) allows a pair outside its margin a slack up to
`slack_gate_tolerance_m` (5 mm; `slack_gate_arm_tolerance_m` for arm and panel rows when set) and a pair inside its
margin by v a slack up to v + 0.1 mm (`kGateInsideAllowanceM`). The 0.1 mm rule was written so a robot cannot creep in
by the tolerance on every cycle.

At store_room's door (replicas_notes.md, "Outcomes") the wheel-contact base's sideslip leaves a base pair 0.1-1.7 mm
inside its margin. A skid-steer base cannot move sideways: every plan that makes progress briefly swings some point of
the robot 2-3 mm deeper. No plan passes the 0.1 mm rule and the hold is permanent (B5 0/20 on the corpus route, 0/15 on
the routes of Unity's three successful B5 runs). The allowance steps from 5 mm just outside the margin to 0.1 mm just
inside it.

## 3. Design

### 3.1 The allowance rule (`slack_gate_inside`)

A new key in `wholebody_mpc.yaml`, overridable by the node and launch parameter `slack_gate_inside` (as `slack_gate`
is):

- `increment` (the default until E1's acceptance; the frozen rule): a pair inside its margin by v > 0 is allowed
  v + 0.1 mm.
- `absolute`: a pair is allowed max(T, v + 0.1 mm), where T is the outside tolerance for the row
  (`slack_gate_tolerance_m`, or `slack_gate_arm_tolerance_m` for arm and panel rows when it is set). Outside the margin
  (v = 0) this is T, as today.

Consequences of `absolute`:
- The step at v = 0 is gone: a pair outside, or inside by less than T, may plan up to T into its margin.
- Creep stays bounded: the bound is a depth (T), not an increment per cycle, so repeated cycles cannot accumulate
  beyond T. Beyond T the 0.1 mm noise term applies as today (stay or plan your way out). A pair deeper than T can only
  have got there through execution (sideslip, lag), never through a plan that passed.
- The resume horizon (2 s) and dwell (0.2 s) are unchanged; they address the alternation of one-iteration
  linearizations, a separate mechanism.

The plant campaign has no B5 config option today. Both runners gain `--b5-config FILE` (a variant of
`wholebody_mpc.yaml`, repository-relative, as `--profile` is; the C++ `plant_campaign` and `tools/plant/plant_campaign.py`,
with `--tag` naming the variant's runs), which carries the rule and every E1 switch; the run log's session Hello and the
run summary record the rule that ran (`config.gate_rule`) and the B5 config's SHA-256.

store_room's first trip (a pair outside its margin asking 5.1-5.4 mm against the 5 mm allowance, holds of 0.3-0.5 s)
is not changed by this rule. If the ensembles show holds of that kind lasting longer than a few seconds, they become a
flip site for Section 3.4, not a new rule here.

### 3.2 Decision trace (run log schema bump)

The run log (stream-protocol.md "Server frames", "Run logs") is the per-cycle record; the trace adds fields to it. The
schema version increases and the reader keeps reading older logs.

| Frame | Already logged | Added |
|---|---|---|
| State (each B5 cycle) | `status`, `gate` [slack, violation, unsafe], `fallback` | `dwell`: 1 while holding with a passing plan inside the resume dwell, else 0 |
| Plans (each medium cycle) | `status`, `homotopy`, `capped` | `seed` (the winning seed), `full_multistart`, `waiting`, `homotopy_switched`, and per seed its name, merit, feasibility and progress (`SeedStats` without its timings, so same-seed logs stay byte-identical) |
| Session Hello | `config` | `config.gate_rule`, and every E1 switch's value |

### 3.3 Twin runs and the divergence tool

`plant_campaign` gains `--start-nudge EPS` (rad, default 0): EPS is added to the start yaw after the seed's ±1 mm /
±1 mrad perturbation, so a twin keeps the same seed's noise stream and differs only at rounding level. Twins use
EPS = 1e-12.

`tools/plant/divergence.py` (Python, beside the other plant tools):
- `pair LOG_A LOG_B`: the first cycle at which a discrete decision differs (B5 status, gate unsafe, dwell, medium
  fallback reason, the plan's seed, homotopy, status and waiting flag); the state distance (base m, yaw rad, joints
  rad) at that cycle and at the end; both sides' context at that cycle (the seed merits that crossed, the gate slack
  against its allowance, the fallback inputs); both outcomes.
- `twin --cell C --kind f2|b5 --seed S [--mode realistic]`: runs the seed and its twin through `plant_campaign`, then
  `pair`.
- `ensemble --corpus FILE --seeds N [--kinds ...]`: N twin pairs per cell and layer; per cell the **flip rate** (the
  share of pairs whose outcomes differ: reached against not reached, contact against none) and, over all pairs, a
  histogram of the first diverging decision by kind. Outputs a JSON report and a Markdown table.

The flip rate is E1's robustness metric, measured before and after each fix on the same twin pairs.

### 3.4 Fixing flip sites

Order of work:
1. **Baseline flip rates** on the frozen code (`increment`, every switch off): twin ensembles on campaign A's cells
   (7 cells x F2/B5, 20 pairs) and the grip corpus's cells at mu 0.6.
2. **The allowance rule** (3.1), then the flip rates again: some flips are the lock itself (one twin creeps through,
   the other locks).
3. **The top flip sites** from the histogram, one at a time: replay the pair, identify what crossed, make that choice
   commit with the smallest mechanism that does it, re-measure that cell's pairs, then all cells.

| Kind of decision | Mechanism |
|---|---|
| a choice between near-equal options (F2's winning seed, the side label) | a hysteresis band relative to the incumbent; a deterministic tie-break that keeps the incumbent within the band |
| a threshold crossed by a measured value (gate pass, the heading-error turn in place, the stall and blocked detectors) | separate enter and leave thresholds, or a minimum dwell |
| a timer or cycle count that one cycle shifts | unchanged unless it is the flip site (a timer moves a flip, it does not remove it) |

Each fix is a config switch whose off value is the frozen behaviour, its own commit, and a unit test that reproduces
the crossing it guards.

**Stop rule:** E1 stops fixing flips when the remaining first-divergence sites are spread with no dominant one, or when
the top site is one E2 owns (the measured state drifting from B5's prediction).

### 3.5 Findings (c) and (d)

- **(c) gate_right_swing, F2 left starts.** Twin pairs on seeds s4, s7, s11, s19; checks whether the first side label
  itself flips between twins and how the waiting incumbent, the seed rotation and the 0.8 rad pan label take part.
  A decision cause is fixed through 3.4; an execution cause (the left swing tracked poorly through the gate) is
  recorded and handed to E2. Why the plant's F2 prefers the right side goes into replicas_notes.md either way.
- **(d) door_105, F2 seed s20.** The replay separates the two decision sites: B5 executing slack plans up to the whole
  base margin (why the gate passed them) and B5 following infeasible medium plans (why it did not fall back). If the
  long following of infeasible plans is the cause, the fix is a decision switch: B5 falls back to nav2 after N
  consecutive infeasible medium plans. A base-tracking cause goes to E2.

## 4. Acceptance

All measures on the realistic plant, 20 seeds, the recorded routes, `--stall-s 45`, with E1's final configuration (the
adopted switches on) against a fresh frozen-code baseline at the same settings (amended at CP1, user decision
2026-11-08): the twin ensembles' plain runs (campaign A, chicane) and `e1_ensembles.sh a3` (campaign B F2, the grip
corpus at mu 0.4 and 0.8) in `experiment_runs/e1/baseline`. The 1145-run replicas (`experiment_runs/plant-replicas`)
ran before the stall detector and before the State frames carried the gate, so their reached counts and holds are not
comparable; they remain A5's equivalence reference (rerun without the stall detector):

| # | Criterion |
|---|---|
| A1 | **No permanent locks:** no run ends in a hold (`holding` through its last 10 s) whose gate fails at a pair inside its margin by less than T (logged `0 < gate_violation < T`): the step's lock. Runs ending in a hold at a pair deeper than T (execution pushed it past the bound) are listed with their cause and counted as E2 cases, not E1 failures. |
| A2 | **Flip rate down:** the pooled flip rate over campaign A's cells is lower than the frozen baseline's on the same twin pairs, and no cell's flip rate rises significantly (Fisher p < 0.05). |
| A3 | **No regressions:** per cell and layer, reached counts not lower than the baseline beyond noise (Fisher p < 0.05) on campaign A, campaign B (F2) and the grip corpus (mu 0.4, 0.8); contacts not higher; the median of the true 3D minimum clearance not more than 5 mm lower in any cell. |
| A4 | **Findings (c) and (d) attributed:** each fixed by a decision switch, or recorded with its cause and handed to E2 or E4. |
| A5 | **Tests and reproducibility:** the C++ (ctest, RelWithDebInfo) and Python suites pass; `increment` with every switch off reproduces the frozen baseline's same-seed runs on three cells (store_room B5, gate_right_swing F2, door_105 F2): the schema-2 logs carry new fields, so the check is frame by frame on the shared fields (no diverging decision, zero state difference, the same outcome), and same-seed schema-2 logs are byte-identical to each other. |

Unity is not run in E1. The defaults change in one final commit after A1-A5 pass, with a provisional row in the
roadmap's comparability log; the Unity door-cell check and the re-freeze belong to E5.

## 5. Checkpoints

The work stops for the user's ruling at:
- **CP1:** the baseline flip rates and the first-divergence histogram.
- **CP2:** the allowance rule's ensemble (A1; its share of the flips).
- **CP3:** after each adopted flip fix, or at the stop rule; findings (c) and (d).
- **CP4:** the acceptance run (A1-A5) and the default change.

## 6. Tests

- `gate_allowance` under both rules at v = 0, 0 < v < T and v > T, for base and arm rows, with and without
  `slack_gate_arm_tolerance_m`.
- A `Controller` test reproducing the lock: a base pair 1 mm inside its margin and a plan asking 2.5 mm holds under
  `increment` and passes under `absolute`.
- The existing golden and frozen-behaviour tests pass with the defaults.
- `divergence.py pair` on synthetic logs (identical; one field differing at a known cycle); a same-seed twin with
  EPS = 0 gives byte-identical logs; the reader reads old and new schema logs.
- One unit test per adopted flip fix reproducing the crossing.

## 7. Records and process

- `docs/experiments/execution-fixes/e1.md`: measurements, the flip-rate tables and a problems table (problem / cause /
  resolution / where) kept current as problems are solved.
- The roadmap decision log: one entry per adopted switch; the comparability log: a provisional row for the default
  change.
- ADR 0017 (short): the gate allowance's new meaning, superseding the inside-margin clause of ADR 0011.
- `docs/running-the-stack.md` and `stream-protocol.md`: the new switches, `--start-nudge`, the divergence tool and
  the schema bump.
- Process (user rules): one commit per fix on `e1-gate-robustness`; a fresh reviewer after every task; timed
  ensembles never overlap Unity Play or each other; caffeinate long jobs; runtime code in C++, tools in Python.
