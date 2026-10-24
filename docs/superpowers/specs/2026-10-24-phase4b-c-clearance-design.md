# Phase 4B-C: B5 keeps its margin along the executed path (design)

Date 2026-10-24. Branch `phase4b-c-clearance` off `phase4b-b-sweep`; restore point: tag `pre-4b-c-2026-10-24`
(f149b36). Before Phase 5 and before the Phase 4B-B re-freeze (campaign 2).

## 1. Goal and evidence

B5's plans keep the 0.05 m arm/panel margin at their 0.1 s stage points only. The shortfall decomposition
([phase4b-b.md](../../experiments/wholebody-mpc/phase4b-b.md) "Shortfall decomposition") locates the losses:

| Cell | Plan pays slack | Between stages | Commanded spline | Execution |
|---|---|---|---|---|
| Scripted constrained (0.05 m, no lead) | 3-16 mm | 0-10 mm | 0-4 mm | 4-19 mm |
| Store S, R4 (realistic) | none (stage points at 0.050 m) | 7-10 mm | 2-7 mm | arm better than its command |

Goal: the plan honours the margin along the path the arm is commanded to follow; what remains is execution, which
the arm stream's lead (ADR 0014) is then re-evaluated against. Success is measured, not assumed: the user was told
~40-50 % that every targeted run reaches 0.045 m, ~75-80 % that the shortfall shrinks substantially.

User decisions (2026-10-24): collision wins over references (a reference the plan cannot reach safely is followed late
or partly, never bought with slack); approaches 1a, 2a, 3a below; testing targeted and short (campaign 2 follows);
every change behind a switch whose default is the current behaviour.

Constraints: the 1.05 m doors must stay passable (campaign 1: a 0.065 m stage margin stalled them); 50 Hz timing
within the bench rule (p99 <= 10 ms, max <= 20 ms) and live R26; the frozen B5 and the golden test unchanged.

## 2. Mid-stage collision rows (1a)

- For stages k < K (K = ceil(midstage_horizon_s / dt), default horizon 1.0 s: 10 stages), each collision pair selected
  for stage k also gets a row at the interval's midpoint state x_{k+1/2} = x_k + f(x_k, u_k) dt/2 (the formulation's
  model, half a step). Linearized at the nominal (x_k, u_k): d_{k+1/2} + J_x dx_k + J_u du_k >= margin - s, with
  J_x = n^T dp/dx (I + df/dx dt/2) and J_u = n^T dp/dx df/du dt/2; it is a stage-k row in the existing
  `Cx dx_k + Cu du_k` form. Pairs are chosen at the midpoint by the same rule as the stage's (closest within
  activation, the stage's budget, arm and panel and self rows alike).
- Slack, weights and the slack gate treat a mid-stage row like the stage's own rows; the gate's horizon covers them.
- Keys: `collision.midstage_rows` (false: frozen), `collision.midstage_horizon_s` (1.0). Budget: mid-stage rows have
  their own `row_budget` per stage (the QP's stage row count grows by it for k < K).

## 3. The stream follows the plan (2a)

- The arm stream's first `stream_dense_s` (0.5 s) carries points every `stream_dense_dt_s` (0.02 s) from the plan's
  own model between stages (the controls of each stage integrated in sub-steps, positions and velocities), then the
  stage points and the braking tail as now. The JTC's cubic segments become 20 ms long and cannot bulge between stages.
- Keys: `stream_dense_s` (0: frozen, stage points only), `stream_dense_dt_s` (0.02). Point 0 and the lead are
  unchanged.

## 4. Collision over references (3a)

- **Entering the margin:** an arm or panel pair outside the margin may not take slack beyond solver noise:
  `slack_gate_arm_tolerance_m` (0.005: frozen, the shared tolerance) for arm/panel rows; 4B-C 0.0001. Base rows keep
  `slack_gate_tolerance_m`.
- **The current violation** of an arm or panel pair inside the margin is taken at the commanded arm state (the
  reference the MPC plans from) instead of the measured one: `slack_gate_violation_at: measured | reference`
  (measured: frozen). An on-time arm then no longer widens the allowance (the lead's effect in the decomposition).
- **The cost of entering:** the L1 slack weight of an arm/panel row whose pair is outside the margin,
  `weights.slack_entry` (-1: the plain `slack`, frozen), so the QP prefers to deviate from a reference over entering.
- A plan the gate rejects holds as today (CycleStatus::Holding); a scripted reference that cannot be met safely is
  followed late.

## 5. The lead, re-measured

After 2-4, the arm actuation delay is measured end to end: the node's reference (sampled by time since sending)
against Unity's recorded arm, in the targeted runs with `arm_stream_lead_s: 0`. The lead is set to that value (it was
40 ms from the JTC's stamp; the node's clock adds 0-20 ms) and re-evaluated in the same cells (Section 6).

## 6. Testing (targeted)

- **Unit:**
  - mid-stage row gradients against finite differences;
  - mid-stage rows keep the interval's midpoint at the margin in the unit loops (1.34 m beam, shelf), where the frozen
    rows dip;
  - the dense stream sampled as the JTC samples it stays within 1 mm of the plan's model path;
  - the gate rules: arm/panel tolerance, violation at the reference, the entry weight;
  - every switch off reproduces the frozen behaviour (golden test).
- **Bench, before any live run:** replay the R4, store S and scripted constrained bags with the new switches:
  - the doors: holds at the door frames, with 0.05 m kept between stages;
  - between-stage clearance share;
  - timing: p99 and max at 50 Hz on the corpus subset.

  A door that blocks on the bench is decided with the user before live runs (options: mid-stage rows only within
  0.5 s, or excluding door-frame pairs).
- **Live, about 8 runs:** scripted constrained x3, store S x2, R4 x2, unscripted constrained x1, at 50 Hz on the
  4B-B defaults plus the 4B-C switches, `arm_stream_lead_s: 0`. Then, if the lead is re-evaluated, the same 8 with it.
- **Acceptance:**
  - no contact;
  - success as in 4B-B (the crooked-door dead end accepted);
  - planned clearance at least the margin at stages and between them within the gate horizon;
  - measured clearance reported per run against 0.045 m;
  - R26 timing.

## 7. Outcome and records

- The chosen switches become B5's defaults for campaign 2 (re-freeze; reduced set plus these cells).
- The problems table, the decision log, the comparability log and ADR 0014 (lead) are updated; a new ADR only if the
  gate's semantics change in a way later phases depend on.
- If the acceptance fails, the remainder is reported with its layer (plan, between stages, spline, execution) and the
  user rules before campaign 2.

## 8. Risks

- **The doors stall again** (estimated 30-40 %): a continuous margin is stricter than one at stage points. This is
  checked on the bench first.
- **Timing:** about double the rows in the first 10 stages, probably under 1 ms more on HPIPM; checked on the bench.
- **Scripted references are followed later:** the mission is slower, but should still succeed.
- **The bench's plans are replays** (fidelity 0.87-1.0 on these bags), indicative rather than exact.
