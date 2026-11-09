# ADR 0017: the slack gate bounds a pair's depth, and B5 leaves infeasible medium plans

Date: 2026-11-09. Status: accepted (user decision 2026-11-09, E1 of the execution fixes). Design:
[spec](../superpowers/specs/2026-11-08-e1-gate-robustness-design.md); results:
[e1.md](../experiments/execution-fixes/e1.md). Supersedes the inside-margin clause of
[ADR 0011](0011-wholebody-mpc-command-ownership.md)'s slack gate note (2026-10-13); amends the medium switch of
[ADR 0015](0015-medium-horizon-layer.md). Judged on the MuJoCo plant ([ADR 0016](0016-mujoco-campaign-plant.md)).

## Context

B5's slack gate holds when a plan's collision-row slack exceeds an allowance: 5 mm (T) for a pair outside its margin;
for a pair inside it by v, v + 0.1 mm, so a robot inside the margin may stay or leave but not plan deeper (ADR 0011).
On the MuJoCo plant's realistic base that rule locked: at store_room's 1.05 m door the base slid 0.1-1.7 mm inside a
door pair's margin, every progressing plan swung some point 2-3 mm deeper, and no plan passed again (19 permanent
holds over the ensembles, B5 0/20 there). Separately, four runs touched an obstacle while every plan passed the gate:
the medium plans had been infeasible for 7-27 s, B5 followed them (the medium switch fell back to Nav2's route only for
no plan, a stale plan, another route or an offset), and the nearly stationary base ended each 0.1 s 1-2 mm deeper than
every plan predicted until it had used up the base's whole 0.12 m margin.

Variants measured (20 seeds a cell, recorded routes, twin ensembles for decision flips; e1.md): the absolute rule
alone (no step lock, outcomes unchanged, one released run touched); a measured-depth guard (hold when a pair is
measured deeper than its reference + 2 mm; all contacts gone but 18 runs held for good at 7-23 mm, store_room B5 at mu
0.8 9 -> 0 of 20); an infeasibility fallback (contacts gone, reached within noise); the absolute rule with the fallback
(adopted).

## Decision

- **The gate bounds the depth.** `slack_gate_inside: absolute`: a pair inside its margin by v may plan up to
  max(T, v + 0.1 mm) deep, T the row's outside tolerance (`slack_gate_tolerance_m`, or `slack_gate_arm_tolerance_m`
  for arm and panel rows when set). No step at the margin; creep from plans stops at T; beyond T the old stay-or-leave
  rule holds. `increment` (the old rule) stays selectable.
- **B5 leaves a run of infeasible medium plans.** `medium_max_infeasible_s: 2.0`: B5 follows a run of infeasible
  medium plans for 2 s from the first one's stamp, then falls back to Nav2's route (fallback reason `infeasible`) until
  a feasible plan arrives; a `stale_reused` plan neither starts nor ends a run; a route change ends it. -1 is the old
  behaviour.
- **The measured-depth guard stays a switch, off** (`slack_gate_depth_guard_m: -1`): its 2 mm trigger turned the
  observed drift into permanent holds; a looser trigger or an exit from its hold is open.

## Consequences

- On the plant (F2, 700 plain runs; B5 from the absolute set): step locks 19 -> 0, contacts 4 -> 0 (twins included),
  reached 639 -> 634 of 700 with no cell significant (store_room F2 5 -> 3 of 20, p 0.35); decision flips unchanged.
  The lock's removal changes no outcome on its own: B5 alone still reaches store_room 0 of 20 (the released robots hold
  at the 5 mm bound instead); store_room's sideslip is E2's.
- The gate still trusts the plan inside a margin: if execution drifts deeper than every plan predicts (the base model
  against the plant, or commands below the breakaway), the depth can still ratchet; the fallback removes the observed
  cause (B5 sitting at a door on infeasible references), not the mechanism. E2 (tracking-aware prediction) owns it.
- B5 is no longer the frozen B5 of `f2-frozen-2026-11-02`: a provisional comparability-log row until the final baseline
  run; the golden test keeps pinning the frozen configuration (test/data). Unity confirmation and the re-freeze are E5.
- Holds that remain: plans asking more than 5 mm at a pair less than 5 mm deep (the bound working), pairs deeper than
  5 mm (execution cases for E2).

## Revisit

When E2's tracking-aware prediction lands (the drift's cause), and before the final baseline run (re-freeze, E5).
