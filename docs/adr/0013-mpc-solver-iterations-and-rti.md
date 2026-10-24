# ADR 0013: B5 solves one QP per cycle with HPIPM; no RTI split; B6 deferred

Date: 2026-10-24. Status: accepted (Phase 4B-B re-freeze, tag `b5-frozen-2026-10-24`).

## Context

The frozen B5 (tag `b5-frozen-2026-10-22`) solved its stage-wise LTV QP with OSQP: up to 500 ADMM iterations near
obstacles. That gave a p99 of up to 25 ms and a maximum of 145 ms on the replay corpus at 50 Hz (20 ms period), and
two cells failed the R26 timing rule live. Phase 4B-B had to settle the solver, the number of QP iterations per
cycle, the horizon and grid, and whether to split preparation and feedback (RTI). The bar was measured headroom,
because Phase 5 adds a medium layer to the same host. Evidence: [phase4b-b.md](../experiments/wholebody-mpc/phase4b-b.md)
(offline sweep: 984 + 533 replays; campaigns 1, 1b and 2).

## Options considered

- **Solver:**
  - OSQP (tolerance, iteration cap, rho adaptation);
  - HPIPM in-house: dense or partial condensing, three modes, with or without a warm start from the previous
    multipliers;
  - acados SQP-RTI (B6).
- **QP iterations per cycle:** one (real-time iteration), or two with a step check (the second iterate kept only if
  its near-term and horizon violation and its cost are no worse).
- **RTI preparation/feedback split:** linearize and build before the state arrives, then solve with the measured
  state.
- **Horizon and grid:** N 10/15/20/30 at dt 0.1, dt 0.075/0.15, and a two-segment grid (only if N 30 beat N 20 on
  quality but failed timing).

## Decision

- **HPIPM, dense, balance mode, one QP iteration per cycle, N 20 at dt 0.1 s, 50 Hz.** Its warm start is off: the
  collision rows are re-ranked every cycle, so the previous multipliers belong to other rows (875 failures).
- **No second iteration.** It never raised the clearance on the bench (share 0.9993-0.9994 against 0.9998 with
  one), it doubles the cycle, and the unit loops showed no step check that both helps and does not stall.
- **No RTI split by default.** The switch exists (`rti_split`, `rti_max_deviation`). With HPIPM's 1-2 ms solve it
  buys nothing at 50 Hz.
- **No two-segment grid.** Its trigger was not met: N 30 is not better than N 20.
- **B6 (acados) is deferred to Phase 6 as optional.** The in-house HPIPM met the bar, so B6 is no longer needed as
  a fallback. The formulation stays solver-agnostic, so B6 remains cheap.

The OSQP backend, the second iteration, the RTI split and the deadline cap all remain selectable. Their defaults
are the frozen B5's except where the installed `wholebody_mpc.yaml` sets them (the golden test pins the frozen
behaviour with every switch at its default).

## Consequences and risks

- **Timing headroom.** Bench p99 4.5 ms in the worst run at 50 Hz (frozen: 24.5). Live in campaign 2: MPC p99
  3.5-9.1 ms per cell, deadline misses below 0.1 %.
- **One QP per cycle keeps the linearization error.** It is small in realistic cells (p95 0.1-0.2 mm) and large
  during big arm motions: up to 17 mm in the scripted cell, centimetres under a low beam. The slack gate cannot see
  it at far stages. Phase 4B-C checked whether such far-stage plans are executed (they are not) and dropped a
  true-clearance check, which deadlocked large motions.
- **The 3D clearance shortfall is not a solver matter.** It sits in execution (Phase 4B-C decomposition).

## Validation

- The golden test reproduces the frozen B5 with every switch off.
- Bench rules 1-4 (failures, clearance share, timing, tracking) on the replay corpus.
- Campaigns 1, 1b and 2.

## Revisit

- If Phase 5 makes the local QP larger (more rows, a longer horizon) or the host slower: partial condensing and the
  speed mode are measured alternatives.
- If large-motion linearization error becomes the limiting factor: acados SQP-RTI with a real step acceptance
  (B6), or a second iteration with a merit function.
- If HPIPM's dense solve stops scaling with the stage count: partial condensing (bench: 2.5-3.8 ms p99 at the
  median run).
