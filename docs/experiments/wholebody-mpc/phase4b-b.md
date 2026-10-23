# Phase 4B-B: performance sweep and re-freeze

[Design](../../superpowers/specs/2026-10-22-phase4b-b-performance-sweep-design.md) ·
[plan](../../superpowers/plans/2026-10-22-phase4b-b-performance-sweep.md) · branch `phase4b-b-sweep`

This file grows with the phase: the replay bench, the offline sweep (this section, 2026-10-23), then campaign 1 and
campaign 2.

## Replay bench

`mpc_replay` replays a recorded B5 run through a controller built from any `wholebody_mpc.yaml` variant
(`tools/mpc_sweep.py` runs the grids). It is open-loop: the robot follows the recorded states, so the bench
measures computation and the plan at the recorded state, not closed-loop outcomes.

- **Corpus:** 41 runs ([corpus.json](phase4b-b/corpus.json)): the 4B-A campaign-3 runs without the mover cells and
  the removed terrain scenarios (34), and the seven undisturbed default low-pass runs. Four are the clearance
  shortfall runs (scripted constrained 4 and 9, store S 40 and 42).
- **Fidelity** (spec Section 3.5, amended): the 4B-A bags record the solved plan's stage 0, which differs from the
  controller's input by the solver's tolerance (up to about 2e-6 m). Near active constraints that offset grows to
  1-10 mm at stage 1. On those bags the status agreement is at least 0.94 on every run and 93.7 % of 98 309 stage-1
  states agree within 1 mm (a synthetic loop recorded the same way: 0.34 / 0.74; recorded exactly: 1.00). The node now
  records the exact input; campaign 1's bags re-check the bench at the strict bar (user decision 2026-10-22).
- **Timing** is single-threaded on the idle dev container (M4 host, thermal pressure nominal); `total_s` is the
  controller's step plus, with the preparation/feedback split, the preparation run in the same cycle.

## Offline sweep (2026-10-23)

Rules (spec Section 5.4): 1 no more solver failures than frozen (and no arm-link pair below its margin), 2 the
shortfall runs' planned clearance at least 0.045 m in 99 % of their cycles, 3 p99 at most 10 ms and max at most 20 ms
on every run at 50 Hz, 4 tracking not worse; then the lowest p99.

### Stage A: one factor at a time (984 replays, one repeat)

[Full table](phase4b-b/stage-a-table.md). Only the HPIPM family passes:

| Variant | p50 ms | p99 ms (median run) | p99 ms (worst run) | max ms | Failures | Fails rules |
|---|---|---|---|---|---|---|
| frozen (OSQP, eps 1e-3, 500 iterations) | 2.14 | 5.82 | 24.83 | 144.55 | 6 | 3 |
| HPIPM | 1.16 | 2.20 | 4.14 | 4.98 | 0 | - |
| HPIPM speed mode | 0.89 | 2.04 | 3.17 | 11.00 | 0 | - |
| HPIPM partial condensing 4 / 5 / 2 | 1.30-2.04 | 2.48-3.82 | 4.51-7.62 | 9.19-10.10 | 0 | - |
| HPIPM warm start (previous multipliers) | 0.59 | 5.58 | 14.21 | 16.49 | 875 | 1, 3 |
| OSQP eps 5e-3 / 200 iterations / rho interval 25 | 2.14-2.15 | 3.35-6.95 | 10.89-24.90 | 14.14-40.10 | 0-193 | 3 (1) |
| N 10 / 15 / 30 (dt 0.1) | 1.12-3.20 | 1.86-8.52 | 13.42-36.92 | 14.71-42.75 | 4-22 | 3 (1) |
| dt 0.075 / 0.15 (2 s) | 2.89 / 1.40 | 6.05 / 2.86 | 33.34 / 16.01 | 36.58 / 16.65 | 43 / 3 | 3 (1) |
| warm start off, RTI split, deadline cap, arm rows, pairs 6 / 12, activation 0.6 / 1.0 | 2.07-2.30 | 4.30-6.90 | 18.15-25.68 | 18.25-35.42 | 10-33 | 3 (1) |
| two QP iterations | 4.44 | 7.08 | 35.19 | 39.61 | 11 | 1, 3 |

- OSQP's tail comes from its iteration count: up to 500 ADMM iterations near obstacles, against 35-43 for HPIPM's
  interior-point solve. No OSQP setting removes it.
- HPIPM's warm start with the previous cycle's multipliers fails: the collision rows are re-ranked every cycle, so
  the multipliers belong to other rows.
- **Grid trigger not met.** N 30 is not better than N 20 on the shortfall runs' clearance (share 0.9997 against
  0.9998) or progress, so the two-segment grid is not added.
- **The planned clearance holds in the shortfall runs.** Apart from cycle 0 of the two scripted constrained missions
  (the first plan, linearized at rest, sweeps the panel through the tall box at later stages; it is replanned on the
  next cycle and not executed: the -0.19 m in the tables), every planned stage within the slack gate's horizon keeps
  0.045 m, store S included (minima 0.0494 and 0.0512 m). Live, those runs went to 0.032-0.044 m. The shortfall is
  made during execution or between the planned stages, not in the plans.

### Stage B: factorial (533 replays, one repeat)

[Full table](phase4b-b/stage-b-table.md). Solver {OSQP eps 5e-3, HPIPM, HPIPM speed} x QP iterations {1, 2} x N
{20, 10}, all with the arm-link rows.

- **The second QP iteration is dropped.** It never raises the shortfall runs' clearance (share 0.9993-0.9994 with
  two iterations against 0.9998 with one) and doubles the cycle time. This matches the unit loops (one iteration
  0.0509 m under a 1.34 m beam; every second-iteration step check tried 0.029-0.0492 m, and the Gauss-Newton /
  merit variants stalled beside a shelf).
- OSQP fails the timing rule at N 20 even at eps 5e-3 (p99 23.1 ms worst run).
- The arm-link rows cost about 0.2 ms at the median.

### Finalists (three repeats at 50 Hz; 20 Hz emulated)

[50 Hz](phase4b-b/finalists-50hz-table.md) · [20 Hz](phase4b-b/finalists-20hz-table.md). All with the arm-link rows
and one QP iteration.

| Variant | p50 ms | p99 ms (worst run) | max ms | Iterations max | Failures | Planned clearance share | Fails rules |
|---|---|---|---|---|---|---|---|
| frozen (OSQP) | 2.12 | 24.49 | 25.54 | 500 | 6 | 0.9998 | 3 |
| HPIPM, N 20 | 1.42 | 4.48 | 5.63 | 38 | 0 | 0.9998 | - |
| HPIPM speed, N 20 | 1.14 | 3.54 | 18.75 | 36 | 0 | 0.9998 | - |
| HPIPM, N 10 | 0.72 | 2.33 | 3.08 | 42 | 0 | 0.9998 | - |

At 20 Hz (emulated) the cycle times are the same (they do not depend on the rate) and every variant, frozen included,
keeps 0.045 m in 99.94 % of the shortfall runs' cycles. The single 50.7 ms cycle of plain HPIPM in Stage B did not
recur in three repeats (a host spike). The rate itself is a closed-loop question for campaign 1.

### Margin check (2026-10-23)

The shortfall's first cause is the plan dipping between its collision stages. HPIPM N 20 with the arm and panel
collision margin at 0.05 (frozen), 0.065 and 0.07 m, one repeat on the gate, door, beam and constrained cells
([corpus](phase4b-b/corpus-margin.json), [variants](phase4b-b/variants-margin.yaml),
[table](phase4b-b/margin-table.md)). Clearance between the stages is the replay's five-substep check (cycle 0 left out).

| Margin m | Shortfall runs: cycles at least 0.045 m between stages | Constrained runs: lowest m | Holding cycles | p99 ms (worst run) |
|---|---|---|---|---|
| 0.05 | 0.9932 | 0.0371 / 0.0402 | 786 | 4.59 |
| 0.065 | 0.9984 | 0.0521 / 0.0449 | 964 | 6.33 |
| 0.07 | 0.9987 | 0.0593 / 0.0458 | 1012 | 4.73 |

- **0.065 m is adopted for campaign 1.** It removes most of the dips; 0.07 m adds little and holds more.
- The cost is holding cycles, mostly under the low beams (1.30 m: 152, 228, 252; 1.23 m: 491, 537, 544). Live,
  that may slow those passes; campaign 1 shows it, and a clear slowdown sends the margin back to 0.05 m.
- One store-room run keeps 0.0385 m at every margin: the open-loop replay starts each cycle at the live run's state,
  which was already that close. Only a live run shows what the margin does there.
- Timing matches the sweep (0.05 m: p99 4.59 ms against 4.48 ms). The two 0.065 m spikes (19.6 and 18.6 ms, single
  cycles) ran while the host was thermally loaded (Unity Play on the fanless host, 15:30-15:42); both are within the
  20 ms rule.
- Some mid-run cycles of the low beam 1.23 m replays plan negative clearance to the beam (about -0.3 m) at every
  margin, frozen included; checked against campaign 1's live clearance.

### Gravity feedforward (2026-10-23)

The shortfall's second cause is the arm trailing its reference. The Unity arm drives are springs and dampers without
gravity compensation, so the loaded arm sags (carry: shoulder 0.003, elbow 0.0075 rad). `ArmActuatorController` now
has a gravity feedforward (on by default since 2026-10-23): each drive's position target shifts by the joint's static holding
torque over its stiffness, so the drive supplies that torque at zero error and its force limit still caps the total.
The torque is the statics of the links the joint carries (the 3 kg payload included). Two implementations failed
first: Unity's `GetJointGravityForces` returns zero for the floating mobile root (free fall), and writing
`ArticulationBody.jointForce` each step held the arm still but made every moving joint lag, wrist 2 included with no
torque of its own (observed; the cause inside Unity's `jointForce` path is unknown, so it does not rule out torque
feedforward as such). For an unsaturated drive, shifting the target is the same as adding the torque. The
`gffon` entries of `arm-qualification.json.gz` are that superseded `jointForce` version.

Whole-body carry suite, two repeats each, off against on (`arm-qualification.json.gz` prefixes `gffoff` and
`gffon2`; rad, the worst joint over the repeats: the hold rows give the error during the hold, the others the path
error):

| Case | Off | On |
|---|---|---|
| Hold in carry, still | 0.0075 | 0.0009 |
| Hold while driving, compact / extended | 0.0087 / 0.0085 | 0.0062 / 0.0039 |
| Transitions against gravity (carry to home, carry to vertical) | 0.027-0.036 | 0.023-0.024 |
| Gravity-assisted transitions (home to carry; vertical to carry) | 0.019-0.020; 0.023-0.024 | 0.024; 0.023-0.026 |
| Approach to carry before the holds (path) | 0.007-0.010 | 0.007-0.014 |
| 50 Hz streaming around carry | 0.022-0.024 | 0.019 / 0.028 |
| Worst of all cases | 0.036 | 0.028 |

All cases pass with no watchdog, no saturation and the usual physics gaps. The gravity-assisted moves get slightly
worse: the sag used to push the arm along its path and hide the drives' dynamic lag (0.007-0.010 rad), which motion
feedforward (Phase 6) addresses. The 0.028 streaming run had two command jumps and its largest error on wrist 3, which
carries no gravity load; the other run is better than both off runs on every joint. **On for campaign 1 and the
default from now on** (user decision 2026-10-23; `--no-arm-gravity-feedforward` is the frozen arm).

## Problems and resolutions

Every problem met in Phase 4B-B, what caused it and how it was solved (user request 2026-10-23: the record outlives
the working ledger). Decisions with options and trade-offs are also in the roadmap's decision log; architecture
decisions in ADRs.

| # | Problem | Cause | Resolution | Where |
|---|---|---|---|---|
| 1 | Bench fidelity below the 95 % bar on the 4B-A bags | The node recorded the solved stage 0, which differs from the controller's input by the solver tolerance (2e-6 m, growing to 1-10 mm near constraints) | The node records the exact input; the old bags were accepted at 93.7 % and campaign 1 re-checks the bench on its own bags (user decision 2026-10-22) | Task 2, spec Section 3.5 |
| 2 | Spurious replay restarts and broken rate emulation | Simulation stamps are quantized to the 0.02 s physics step | Rate selection and goal gaps on the bag's wall-clock time | `0d7bc6a` |
| 3 | Missed restarts when the behaviour tree resent a goal without a new `/plan` | The bench restarted only on a new plan | Restart after the replay's own failure | `0d7bc6a` |
| 4 | Old bags would not decode | The `MpcCycle` message gained fields | Zero-padded decoding of the old layout, tested | Task 2 |
| 5 | The deadline cap did not budget the whole cycle | It counted from `step()` entry, not the node's cycle start | `Controller::set_cycle_start`; the cap reserves the preparation time | Task 3 review |
| 6 | HPIPM build would crash on weaker x86 hosts | BLASFEO/HPIPM picked the build host's CPU kernels | AVX run-check with a generic fallback; documented | Task 4 review |
| 7 | HPIPM warm start had no effect | Level 1 uses only the zeroed primal | Level 2 (multipliers and slacks) after a successful solve; with the previous cycle's multipliers it fails anyway (rows re-ranked each cycle), so it stays off | Task 5, Stage A |
| 8 | The second QP iteration made the unit loops worse | The step check accepted aggressive second iterates; Gauss-Newton and merit variants stalled at the shelf | Variant B kept for the sweep, then dropped: never more clearance, twice the time | Task 6, Stage B |
| 9 | One of two arm rows per stage was dead | Wrist 2's cylinder is coaxial with the wrist 3 axis (constant distance, zero gradient) | Wrist 2 excluded; unknown links rejected | Task 7 review |
| 10 | The preparation/feedback split advanced the tightness twice | A dropped preparation was not restored | `discard_preparation` restores the snapshot | Task 8 review |
| 11 | A commit went in with two failing tests (`8fa9cd0`) | The commit command did not check the failure count | Fixed in `5022568`; every commit since requires " 0 failures" | process |
| 12 | Planned clearance of -0.19 m in the sweep tables | Cycle 0 of the scripted missions: the first plan, linearized at rest, is replanned and never executed | Documented; excluded from the clearance share | sweep |
| 13 | OSQP's timing tail (p99 up to 25 ms, 500 iterations) | ADMM iteration count near obstacles; no setting removes it | HPIPM (interior point, 35-43 iterations) | Stage A |
| 14 | Clearance shortfall (4B-A: 4 runs between 0.032 and 0.045 m) | Plans dip about 1 cm between their 0.1 s collision stages, and the arm trails its reference by about 1 cm | Margin 0.065 m (bench, campaign 1); gravity feedforward; continuous-time collision and motion feedforward recorded for later (the motion-feedforward decision follows campaign 1) | margin check, roadmap Phase 6 |
| 15 | Timing measurements skewed | The host is a fanless MacBook Air: Unity Play during the bench pushed thermal pressure to Heavy | Never overlap Play with timed runs; thermal checks before campaigns; affected replays flagged | margin check |
| 16 | Gravity feedforward did nothing | Unity's `GetJointGravityForces` treats the floating mobile root as in free fall and returns zero | Own statics of the carried links | `92995ad` |
| 17 | Gravity feedforward held the arm but every moving joint lagged | Writing `ArticulationBody.jointForce` each step (cause inside Unity unknown) | The drive's target shifts by torque / stiffness instead | `1e25e59` |
| 18 | A run could use the old default after the default flip | An open Editor keeps the scene loaded with the old field value | The runner sets the switch every run; the demo sets it; a test guards the scenes | `279a5aa`, `4546db3` |
| 19 | Qualification start failed once | The Unity pipeline server is unreachable during the Play transition and the runner retries only another message | Retry in the batch script; the runner's retry list is a known gap | qualification |
| 20 | Campaign 1 interrupted | Host restart (user) | Stopped between runs; the started run set aside in `experiment_runs/_discarded`; resumed from the next run | campaign 1 |
| 21 | Narrow gate run held about 100 s and aborted (campaign 1) | Plans needed 9 mm more slack than the slack gate allows: the 0.065 m margin | Margin back to 0.05 m (row 22) | campaign 1 |
| 22 | Room R4 0/4 and store S 2/4 in campaign 1, stuck at the 1.05 m doors (no holds, no near contact) | The doors need the panel inside 0.065 m (4B-A passes: 0.047 m from R4's frame, 0.042 m from S's); the plan stops advancing rather than enter the margin. The bench showed it only as more holds (R4 0 -> 10 -> 28): open loop cannot show a stall | Margin back to 0.05 m (user ruling 2026-10-23); the shortfall goes to arm tracking (motion feedforward) and, later, continuous-time collision | campaign 1 |
| 23 | Scripted constrained run at 0.0396 m with the 0.065 m margin | 13 mm lost in the plan/reference (0.0519 m at the minimum), 12 mm in tracking lag, mostly wrist 1 (0.019 rad, the softest drive accelerating the panel) | Motion feedforward before campaign 2 (user decision, option A); the plan-side half stays for continuous-time collision | campaign 1 |
