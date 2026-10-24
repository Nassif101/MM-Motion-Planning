# Phase 4B-C: B5 keeps its margin along the executed path

Spec: [2026-10-24-phase4b-c-clearance-design.md](../../superpowers/specs/2026-10-24-phase4b-c-clearance-design.md).
Plan: [2026-10-24-phase4b-c-clearance.md](../../superpowers/plans/2026-10-24-phase4b-c-clearance.md).
Restore point: tag `pre-4b-c-2026-10-24`.

## Bench check (plan Task 4, 2026-10-24)

Corpus [phase4b-c/corpus.json](phase4b-c/corpus.json) (15 runs), variants
[phase4b-c/variants.yaml](phase4b-c/variants.yaml), base the installed configuration, 50 Hz, one repeat, idle host
(thermals Nominal). Tables: [bench-table.md](phase4b-c/bench-table.md) (per variant),
[bench-per-run.md](phase4b-c/bench-per-run.md) (per run; cycle 0, the never-executed first plan, left out).

| | frozen4bb | mid | all4bc (entry 1000) | all4bc_e3000 (cap 150) | all4bc_mid05 |
|---|---|---|---|---|---|
| p99 median / worst / max (ms) | 2.9 / 4.3 / 5.1 | 3.8 / 6.0 / 8.4 | 5.6 / 6.3 / 13.6 | 8.0 / 8.8 / 16.7 | 5.4 / 6.4 / 11.1 |
| holds (all runs) | 272 | 250 | 198 | 174 | 197 |
| door runs (1.05 m, 4B-A store S / R4) holds | 0 / 9 / 0 | same | same | same | same |
| solver fallbacks | 0 | 0 | 0 | 0 | 0 |

- **Doors:** no variant adds a hold on any door run.
- **Timing:** within the bench rule (p99 <= 10 ms, max <= 20 ms) for every variant; the entry weight 3000 needs HPIPM's
  cap raised to 150 (up to 60 iterations) and comes closest to it.
- **Entry weight:** 3000 with the installed 50-iteration cap made 240 of 2275 cycles of the 1.05 m door run fall back
  (smoke test); iterations grow with the weight (300: 20, 1000: 30, 3000: 55 at most).
- **Between stages:** the replay's between-stage clearance split by interval (diagnostic, frozen4bb vs all4bc):

  | Run | stages | interval 0 (from x0) | intervals 1+ frozen4bb -> all4bc |
  |---|---|---|---|
  | store S 213155 | 0.050 | 0.0405 | 0.0506 -> 0.0505 |
  | R4 212426 | 0.0496 | 0.0432 | 0.0455 -> 0.0486 (share >= 0.05: 0.998 -> 0.998) |
  | scripted 220342 | 0.0456 -> 0.0487 | 0.0371 -> 0.0377 | 0.0464 -> 0.0481 (share >= 0.05: 0.937 -> 0.968) |

  The mid-stage rows lift the plan between its stages 1+ to about the margin. Every cycle whose interval 0 falls below
  0.045 m already starts there: the current state x0 is below (store 0.038-0.044, scripted 0.035-0.042) and the plan's
  first 20 ms climbs back out. The between-stage loss of the Phase 4B-B decomposition (7-13 mm, measured from x0
  including interval 0) was therefore mostly the state already below the margin (execution), not a plan dip.
- **Scripted stage points:** the mid-stage rows and the gate raise the stage-point minimum (run 215633: 0.0365 ->
  0.044 / 0.047).
- **Finding (gate):** with the entry weight the QP takes no slack where the linearization is badly off, so the slack
  gate passes a plan the frozen settings held on: scripted run 220538 cycle 32, a scripted reference step, 0.13 m
  linearization error, true planned clearance -0.082 m for one cycle (frozen4bb and mid: holding). The frozen B5 has
  the same gap less often (run 215749 cycle 26: -0.034 m, linearization error 0.085 m, published by every variant).
  The gate checks slack, not true clearance.
- **Decision (user, 2026-10-24): no true-clearance check.** Spike: holding a plan whose rows' true distances within the
  gate horizon fall below min(margin, current) - 5 mm. Over 1 s it deadlocked the 1.34 m low-beam unit loop (492 held
  cycles before the beam: while lowering the panel every one-iteration plan was 3-17 cm off its linearization) and
  held 11-12 cycles per scripted bench run; over 0.1 / 0.3 s it never fired. The bench plans' true clearance at
  stage 1 (the part executed at 50 Hz) was >= 0.0449 m on every scripted run without it: the -0.082 / -0.046 m plans
  fall short at stages 2-10 only and are replaced 20 ms later. The far-stage gap stays a known property of the
  one-iteration linearization (as in the frozen B5).

## Targeted live runs (plan Task 5, 2026-10-24)

Configuration [phase4b-c/all4bc.yaml](phase4b-c/all4bc.yaml) (the 4B-B defaults with mid-stage rows, the dense stream,
arm tolerance 0.0001, entry weight 1000; lead 0), 50 Hz, gravity and motion feedforward, thermals Nominal. No contact
in any run. Timing within R26 (MPC cycle p99 8-14 ms, deadline misses at most 0.3 %). Decomposition (as the Phase 4B-B
near-term one: plan stages 1-3 and between them near the measured minimum, from the replay):
[phase4b-c/live-decomposition.json](phase4b-c/live-decomposition.json).

| Run | Outcome | Measured min (m) | Commanded reference at it (m) | Plan stages near (m) | Plan between near (m) |
|---|---|---|---|---|---|
| scripted 162210 | succeeded | 0.0279 | 0.0354 | 0.0500 | 0.0362 |
| scripted 162320 | succeeded | 0.0451 | 0.0523 | 0.0499 | 0.0437 |
| scripted 162455 | succeeded | 0.0298 | 0.0388 | 0.0483 | 0.0359 |
| store S 162708 | succeeded | 0.0432 | 0.0433 | 0.0563 | 0.0462 |
| store S 162920 | aborted (crooked-door dead end) | 0.1166 (wheel) | 0.1166 | 0.120 | 0.117 |
| R4 163331 | aborted (crooked-door dead end) | 0.0481 | 0.0478 | 0.0500 | 0.0481 |
| R4 163743 | aborted (crooked-door dead end) | 0.0434 | 0.0431 | 0.0500 | 0.0459 |
| unscripted 164227 | succeeded | 0.0598 | 0.0576 | 0.0577 | 0.0574 |

Against the 4B-B runs at lead 0 ([near-term decomposition](phase4b-b/shortfall-analysis-near3.json)):

- **Plan stages: improved.** Scripted 0.048-0.050 m (4B-B gravity and motion FF: 0.045-0.046, the plan paying slack).
- **The rest: unchanged.** Scripted measured 0.028-0.045 m (4B-B 0.031-0.036); commanded reference at the minimum
  0.035-0.052 (4B-B 0.037-0.043); arm execution (measured below its command) 7-9 mm (4B-B 6.5-7 mm). Store S and
  R4: the reference 0.043-0.048 m with the plan's stages at 0.050-0.056, the arm at or above its command (as in 4B-B).
  The "between near" values start from the measured state x0 (the bench: interval 0 is where the state already is).
- **Doors:** R4 0 of 2 and store S 1 of 2 succeeded, every failure the crooked-door dead end (Phase 4B-A, accepted,
  Phase 5); the 4B-B configuration plans no progress either from the stalled pose (replay of run 163743, every switch
  separately: planned progress 0.009 m/s). Rates: 4B-A R4 2/3, store 3/3; 4B-B 1b R4 2/3, store 2/3; margin 0.065
  R4 0/4, store 2/4; 4B-C R4 0/2, store 1/2 (small samples; a stricter arm clearance plausibly makes crooked
  arrivals more frequent, as the 0.065 margin did).
- **Lead (ADR 0014):** the three 4B-B scripted runs at lead 0.04 measured 0.009-0.021 m against 0.031-0.036 m at
  lead 0.

## Outcome (user decision 2026-10-24)

The 4B-C switches stay in the code, **off by default** (`collision.midstage_rows`, `stream_dense_s`,
`slack_gate_arm_tolerance_m`, `weights.slack_entry`); the installed arm stream lead is **0** (ADR 0014: 0.04 s was
not adopted); campaign 2 runs on the 4B-B defaults. The plan layer is fixed when they are on (stage points 0.048-0.050
m in the scripted cell, between stages at the margin), but it was 4-5 mm of the shortfall: what remains is the robot's
state below the margin before each plan (base tracking, arm lag; 7-15 mm at the commanded reference) and the arm's
execution in the scripted cell (7-9 mm), for Phase 6 (delay-aware MPC, base tracking). The plan's lead re-measurement
(Task 6) was skipped: the scripted runs already decide it.

## Problems and resolutions

| # | Problem | Cause | Resolution | Where |
|---|---|---|---|---|
| 1 | Dense stream: the first spline segment shrank to 20 ms (Task 2 review) | Dense points replaced the first stage point, sharpening the kink from point 0 (reference velocity) to the plan | The first stage point always stays; dense points start after it; stage points within half a dense step after the dense part are skipped | `f15a6cb` |
| 2 | The entry keys had no parse test (Task 3 review) | Test gap; the loader does not reject unknown keys, so a misplaced key would silently run the frozen behaviour | Parse test; both keys must be -1 or finite >= 0 | `8f11298` |
| 3 | `slack_entry` 3000 (spec) made HPIPM fall back on 240 of 2275 cycles of a door run | An L1 weight that large on every arm row outside the margin slows the interior-point solver past its 50-iteration cap (iterations: 300 -> 20, 1000 -> 30, 3000 -> 55) | 1000 for the live runs (ruling); 3000 needs the cap at 150 and comes close to the timing rule | bench |
| 4 | With the entry weight, a plan 8 cm inside an obstacle passed the slack gate (one cycle) | The QP takes no slack where the linearization is off (reference step, 0.13 m error), and the gate checks slack only; the frozen B5 has the same gap, more rarely | A true-clearance check deadlocked the low-beam loop (1 s) or never fired (0.1-0.3 s); the shortfalls are at stages 2-10, stage 1 kept >= 0.045 m: no check (user decision) | spike, `Record the dropped true-clearance check` |
| 5 | Between-stage clearance did not rise on the bench | The dips were in interval 0, from a state already below the margin, which no plan row covers | Finding: the 4B-B decomposition's between-stage loss was mostly execution (second correction there) | bench |
| 6 | R4 0/2, store S 1/2 live | The crooked-door dead end (4B-A, accepted, Phase 5); the 4B-B configuration stalls from the same pose; arrivals plausibly more crooked with the stricter arm clearance | Switches off by default | live runs |
| 7 | Measured clearance unchanged live | The plan layer was 4-5 mm; state and arm execution dominate | Switches off by default; Phase 6 | live runs |
