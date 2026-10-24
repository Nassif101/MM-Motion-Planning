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
