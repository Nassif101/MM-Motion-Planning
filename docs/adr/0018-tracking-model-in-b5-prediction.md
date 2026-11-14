# ADR 0018: B5 predicts with the plant's tracking model, and its output stage follows the plan

Date: 2026-11-14. Status: accepted (user decision at E2's CP4, 2026-11-14). Design:
[spec](../superpowers/specs/2026-11-11-e2-tracking-prediction-design.md) (Amendments 1-3); results:
[e2.md](../experiments/execution-fixes/e2.md). Judged on the MuJoCo plant ([ADR 0016](0016-mujoco-campaign-plant.md));
builds on [ADR 0017](0017-slack-gate-absolute-allowance-and-infeasibility-fallback.md).

## Context

B5's prediction model was a unicycle. On the MuJoCo plant's realistic base, three things went wrong:
- **Hand-off cells failed.** store_room failed for B5 alone (0 of 20), gate_right_swing failed on its left swings, and
  runs ended in deep holds.
- **The depth ratchet** came from the output stage: the breakaway lift re-kicked the old turn against a plan turning
  back.
- **B5's prediction missed the base's motion.**
  - The turn gain depends on grip: g_inf 1.27 at mu 0.4, 0.98 at 0.6, 0.80 at 0.8.
  - The base turns about a point off its frame. The point follows the centre of mass, d = 0.003 - 2.23 x_com (all five
    excitation poses within 3.4 mm), up to 0.13 m with the arm extended.

E2 measured, identified and judged in stages:
- the follow switch (Task 4b);
- a static map with an online correction (Tasks 5-9);
- the ensembles (Task 10, which failed T3 at mu 0.8);
- the turning-centre offset (Amendment 3, Tasks 10b-10c);
- a fresh-seed confirmation and margin checks (CP4).

## Options considered (spec Section 2)

- **A:** a static command map, the state unchanged. Chosen and extended:
  - a turn map;
  - a turning-centre offset through the generated centre of mass;
  - a sideslip term, identified at about 0.
- **B:** a lateral-velocity state (state 10). Not needed: the offset explained the lateral motion with the state at 9.
- **C:** a learned residual. Not needed for the measured cells.

Inside A:
- the speed-in-turns term was dropped (CP3);
- the arm-delay advance was dropped (Task 8 review);
- the online correction of k was dropped at Task 10c: it drifted at high grip, and k is about 0 once the offset is in.

## Decision

- **`breakaway_follows_plan: true`** in the shared `wholebody_mpc.yaml`: the output stage lifts a sub-breakaway command
  only in the plan's direction. For the ROS node (Unity) as well.
- **Plant runs predict with `tracking_model_mujoco.yaml`:**
  - the turn map: g_inf 0.982, g_0 0.025, c 0.0225;
  - the turning-centre offset: turn_offset 0.0103 m, turn_offset_com -2.15;
  - sideslip -0.023.

  They correct the turn gain online (a Kalman filter; `tracking_adapt_sideslip: false`, k frozen). This holds whenever
  the B5 config names no model; `--tracking-model none` is the unicycle.
- **The ROS node keeps the unicycle** (`tracking_model: ""`) until E5 identifies and judges Unity's base.

## Consequences

- **Outcomes** (1340-run ensembles against E1's baselines; confirmed on fresh seeds 21-40):
  - store_room B5 alone 0 -> 20 of 20 (fresh: 0 -> 20);
  - at mu 0.8, 10 -> 19 (fresh: 7 -> 18);
  - store_room F2 3 -> 20 (fresh: 10 -> 19);
  - gate_right_swing F2 16 -> 20;
  - deep holds anywhere in a run near 0 in every set;
  - reached up in every set, with no significant loss in any cell;
  - twins diverge in 4 of 280 pairs (the follow switch alone: 13; Task 10's candidate, with k learned: 21).
- **Prediction:** the one-stage lateral error falls to 0.31-0.53 mm, against the unicycle's 0.67-0.82, and yaw falls at
  every grip.
- **Timing:** the B5 cycle is faster: p50 3.15 -> 2.78 ms, p99 4.99 -> 4.81 ms.
- **Known limits (E2's acceptance exceptions, ruled at CP4):**
  - **T2 clearance medians fall by more than 1 mm in 13 cells,** with no contacts. Where measured against each pair's
    own margin, none is a margin regression:
    - at store_room's door the nearest part moves from a wheel inside the base margin to the panel outside its own;
    - at low_beam_123 both variants dip the panel 5-12 mm into its margin.

    The clearance metric mixes parts with different margins, and the margin depth while moving is logged only under
    `--diagnostics` (follow-up: log it by default).
  - **T11 at mu 0.6:** the correction is 17 % worse in 2 s yaw than the fixed model, which is right at that grip.
    Accepted; a slower drift is future work.
  - **T5:** B5 alone holds for good in front of the low beams (0-2 of 20 in every variant; F2 passes them). That is
    outside E2: it goes to the approach/hold policy, deferred to Phase 6 (user, 2026-11-14); revisit at E5 if Unity's doors hold, or earlier for the live click-to-goal demo.
- **Comparability.** Plant results before 2026-11-14 used the unicycle. Configs that name no model reproduce them only
  with `--tracking-model none`, which the ensemble scripts pass. The roadmap's comparability log has the row.
- **Unity** runs the follow switch from now on and is not re-measured until E5.

## Revisit when

- E5 identifies Unity's base: its own model file, then the node's default.
- A plant change (profile, base, wheels, payload) invalidates the identified parameters: rerun `e2_identify.sh`.
- The online residual becomes worth its cost: a grip change beyond the correction's range, or the mu 0.6 cost.
- The approach/hold policy changes what B5 does once the gate keeps rejecting.
