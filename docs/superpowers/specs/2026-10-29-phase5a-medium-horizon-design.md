# Phase 5A design: medium-horizon layer with piecewise Bézier (F2) above B5

Date 2026-10-29. Branch `phase5a-medium` off `demo-draggables` (10e4928; `main` is 2f91e01, the re-frozen B5 at
the local tag `b5-frozen-2026-10-24`, plus the four drag-demo commits).

## 1. Purpose and scope

Phase 5 adds the medium-horizon layer of the roadmap's hierarchy (Nav2 route → medium layer → local whole-body MPC).
It is split into sub-projects, each with its own spec (user decision 2026-10-29):

- **5A (this spec):** medium-layer infrastructure and F2 (piecewise Bézier), always on, static known geometry.
- **5B:** F1 dense and F3 MINCO (F4 B-spline optional) as comparison rows on the same decision variables, and their
  campaign.
- **5C:** mover prediction (stop and reverse hypotheses), the late-crossing deadlock, conditional triggers.
- **Demo:** RViz click-to-goal on the full hierarchy, after 5A.

5A must fix all four B5 limitations handed to Phase 5 (roadmap "From Phase 4B-A", "From Phase 4B-B"):

1. **Aligned door approach.** The crooked-door dead end (`v2_room_r4_nav`, `v2_store_room_nav`, about 1 in 3 fail).
2. **Swing side by planning.** Choose the swing side from candidate configurations, replacing B5's heuristic side
   picker (4B-A item (a)).
3. **Low-pass fold.** Choose and time a lowered configuration under the low beams instead of B5's 72° side pivot at
   71-79 Nm shoulder (qualified 19-22 Nm).
4. **Close-section sequencing.** A narrow section followed closely by a low one: reconfigure and slow before B5's 2 s
   horizon sees the second (4B-A item (d)).

Plus: no regression on the B5 exit set, and B5's timing unchanged with the medium layer running on the same host.

### Decisions taken in the design discussion (2026-10-29)

| Topic | Decision |
|---|---|
| Sub-project split | 5A / 5B / 5C / demo as above; 5A first |
| Branch | `phase5a-medium` off `demo-draggables` |
| F4 (B-spline) | Optional, decided at 5B. 5A's basis code keeps knots and degree general so a B-spline basis can be added without rework |
| B6 | Stays deferred to Phase 6, optional (ADR 0013) |
| Bézier local-MPC ablation | Stays a Phase 6 item (it changes B5's layer, not the medium layer) |
| Base friction feedforward | The first task of 5A, so F2 is developed on the final base actuation |
| Obstacles seen by the medium layer | Known static boxes, plus demo draggables as quasi-static (re-plan when one moves). Movers stay B5's job. The input interface takes a tracked-object list in the `/scenario/movers` shape, so 5C adds predictions without changing it |
| Formulation | 1b (Section 3): continuous base SE(2) + four arm joints as piecewise Bézier; qualified poses are multi-start seeds and a weak comfort term only; "qualified" becomes a continuous static-torque + joint-limit constraint |
| Dense-QP solver | Pluggable; HPIPM dense, OSQP and DAQP benchmarked from the start (Section 6.1) |
| Live budget | About two campaigns (Section 7); tuning offline first |

The user's concern behind 1b: blending between a finite list of qualified poses would make the arm part a scheduled
mode switch, against accepted decision 1 (no fundamentally discrete mode planner). In 1b the arm moves continuously;
the only discrete element left is choosing the cheapest of several continuous solutions from different
initializations (multi-start SCP over homotopy classes), which the swing-side problem requires of any local optimizer.

### Out of scope

- Movers in the medium layer, mover prediction, conditional triggers (5C).
- F1, F3, F4 (5B).
- Perception input (the Octomap fault is unresolved; known geometry only).
- Changes to B5's collision rows, margins, slack gate, arm stream or `/cmd_vel` ownership.
- The final baseline run (B1-B5 on the new base) and Phase 6 items (delay-aware MPC, continuous-time collision in
  B5).

## 2. Architecture and data flow

```text
Nav2 BT ──FollowPath(path)──▶ B5 wholebody_mpc_node ──▶ /cmd_vel, arm stream   (ownership unchanged)
                               │  ▲
       /wholebody_mpc/route    │  │ /medium/plan (latest valid, arc-length indexed)
       (transient local,       ▼  │
        route_id)        medium_horizon_node (C++, own process, ~5 Hz)
                           ▲ /odom, /joint_states, known boxes, draggables
```

- **Ownership.** Nav2 sends FollowPath goals to B5 as today. B5 alone publishes `/cmd_vel` and the arm stream
  (ADR 0011). The medium layer publishes only a reference.
- **Route announcement.** On every FollowPath goal (and every path update within one) B5 publishes its active route
  with a monotonically increasing `route_id` on `/wholebody_mpc/route` (transient-local QoS). The medium node plans
  against that route, not `/plan`, so a behaviour-tree smoother or replan cannot desynchronize the layers. Every
  medium plan carries the `route_id` it was built on.
- **`medium_horizon_node`.** C++, its own process, single-threaded, a timer at `rate_hz` (5 Hz; 2.5 Hz with twice the
  cap on the fanless MacBook Air, amendment 2026-11-01, Section 12). Each cycle
  warm-starts from the previous solution shifted to now. The full multi-start (Section 3.5) runs on a new
  `route_id`, on a draggable change, and every `full_multistart_period_s` (default 2 s). It links B5's model (FK),
  collision primitives, broad-phase prefilter and scene loader as a library, so both layers see identical geometry.
  If `mobile_manipulator_mpc` does not export these as a library target yet, exporting them is part of this work.
- **`/medium/plan`** (new message `MediumPlan` in `mobile_manipulator_interfaces`): header (stamp, `map` frame),
  `route_id`, `plan_id`, status, the chosen seed and homotopy label, cost, solve time, and samples every
  `sample_ds` (0.1 m) of arc length along the planned base path, each with x, y, yaw, speed and a six-joint posture
  target in the model's joint order (masked joints from the solution, the others at the comfort pose). The arc
  length of the last sample's projection onto the route is included for the splice (Section 4).
- **`/medium/cycle`** (new message `MediumCycle`): per-cycle timing (total, per seed, linearization, QP), SCP
  iterations per seed, convergence flags, seed costs, collision slack, homotopy switches, trigger of a full
  multi-start. Recorded in every run bag.
- **Visualization.** RViz markers for the medium base path, panel poses along it and the chosen homotopy (also used
  by the later demo).

## 3. The medium optimization problem (formulation 1b)

### 3.1 Representation

- Time-parameterized piecewise Bézier for the base pose (x, y, yaw) and the masked arm joints. Default mask: shoulder
  pan, shoulder lift, elbow, wrist_2 (every qualified pose has wrist_1 = wrist_3 = 0); the mask is a config value
  (`arm_joints`), so a six-joint variant is a switch.
- Default horizon 8 s (`horizon_s`), 4 segments (`segments`), cubic (`degree`), C2 continuity as linear equalities.
  Start position and start velocity are fixed to the measured state (base from `/odom`, joints from
  `/joint_states`). With C2 cubic segments that leaves 5 free parameters per coordinate, 35 variables (amended
  2026-10-29 in the plan; "about 77" counted C1). Horizon, segment count and degree are benchmark parameters
  (roadmap Phase 5 "Questions to benchmark").
- Cartesian x, y, yaw rather than Frenet (s, d, ψ): Frenet coordinates are singular at the 90° Nav2 corners right
  before the crooked doors. Route progress is measured by projection with B5's `Route` (windowed, monotone hint).
- Arc length: the planned path is sampled densely in time, converted to cumulative arc length, and resampled every
  `sample_ds` for the message.

### 3.2 Cost

All terms quadratic or quadratic hinges, evaluated at the time samples (Section 3.3) unless noted:

- **Progress:** lag pull towards the route point a nominal speed ahead of each sample's projection (as B5's lag
  term), tapering at the goal.
- **Contour (weak):** squared distance to the Nav2 route, low weight, so the plan may leave the route for alignment
  and clearance.
- **Clearance hinge:** for base, arm links and panel, `max(0, d_target - d)^2` with `d_target` 0.25 m (start value).
  This produces centred, aligned door approaches and early slowing in tight sections without door-specific rules.
- **Comfort (weak):** `|q - q_comfort|^2` over the masked joints (accepted decision 10; `q_comfort` is B5's comfort
  pose, `carry`).
- **Smoothness:** integral of squared acceleration of base and arm, analytic in the control points.
- **Skid-steer residual (soft):** squared body-frame lateral velocity (the base cannot strafe).

### 3.3 Constraints

- **Collision:** at time samples every `collision_dt` (0.1 s, about 80 samples), signed distance ≥ margin for each
  candidate pair after B5's broad-phase prefilter: base 0.12 m, arm and panel 0.05 m (B5's margins, the contract of
  roadmap "From Phase 4B-B"). Linearized every SCP iteration (FK Jacobian through the masked joints and base pose),
  with L1 slack.
- **Joint limits, velocity and acceleration bounds:** linear constraints on the control points and on the
  derivative curves' control points; by the convex-hull property they bound the whole curve, not only the samples.
- **Static torque:** per masked joint, the gravity torque of the carried links and panel at each sample ≤ the
  qualified limit (source: the arm qualification record; calibrated by the fold pose in Section 5.2). Linearized.
  This replaces the finite list of qualified poses as the definition of "holdable".
- **Forward-only base:** velocity along the heading ≥ 0 (B5's base is forward-only), linearized.

### 3.4 SCP

- Each outer iteration builds one dense QP (Section 6.1) with a trust region on the control points, accepts the step
  if the true merit (cost + slack penalty, collision and torque evaluated exactly) decreases, and shrinks the region
  otherwise. At most `scp_max_iter` (10) outer iterations per seed; converged when the step and merit change are
  below tolerance.
- A solution is **feasible** when total collision slack < 1 mm and the torque and hull constraints hold.

### 3.5 Seeds, selection and hysteresis

- **Seeds:** the previous solution shifted to now (every cycle), and on a full multi-start also left
  (`vertical_carry`), right (`vertical_carry_right`), fold (Section 5.2) and extend (`level_extension`). A pose seed
  initializes the arm curve as a ramp from the measured joints to the seed pose over the horizon and the base on the
  route at nominal speed. Seeds that converge to the same solution (control points within a tolerance) are merged.
- **Selection:** the lowest-cost feasible solution. If none is feasible the lowest-slack one is published with
  status `infeasible`, which B5 treats as a valid reference (its own rows still hold); the count is a metric.
- **Homotopy label:** derived from the solution (swing side from the sign of the pan excursion beyond a threshold,
  lowered when the panel top drops below `carry`'s). **Hysteresis:** the published homotopy changes only if the new
  best is at least 15 % cheaper than the incumbent's re-optimized solution (`homotopy_hysteresis`), so the side
  choice cannot flip between cycles.
- **Budget:** `solve_cap_s` 0.15 s per cycle. Seeds are solved incumbent first; a seed not finished within the cap is
  dropped this cycle. If even the incumbent misses, the last valid plan stays published (roadmap 6.2) and the miss is
  counted.

### 3.6 Obstacles

- Known static boxes from the same scene source as B5.
- Draggables (`draggables_topic`, demo only, off by default): treated as static boxes at their current pose; a change
  beyond a small tolerance triggers a full multi-start.
- A tracked-object input in the `/scenario/movers` shape is declared but unused in 5A (movers stay B5's job).

## 4. B5 in `reference_source: medium` mode

New B5 key `reference_source: nav2 | medium`; the default `nav2` is the frozen B5, which the golden test keeps
pinning. In `medium` mode:

- **Route.** On each new medium plan B5 rebuilds its `Route` from the plan's samples and splices the remaining Nav2
  route beyond the plan's last sample (from that sample's projected arc length on). Within B5's 2 s horizon the route
  is the medium path. Progress is re-projected with the existing windowed search, as B5 already does when Nav2
  replans. The route is not indexed on the Nav2 route with a lateral offset, because projections around a 90° corner
  are ambiguous where the crooked doors are.
- **Per-stage references by arc length.** Each stage's projection on the route gives:
  - yaw(s): the heading term's target (instead of the route tangent);
  - v(s): the progress speed, capped at B5's own limits (how "slow before the close section" reaches the base);
  - q(s): the posture term's target, replacing the fixed comfort target, with its own weight
    `weights.medium_posture` (tuned on the bench; B5's posture weight 2 is a weak pull and a fold needs more).
- **Off in medium mode:** the posture blend, the tightness filter and the side picker. Posture mode `comfort_only`
  with the medium target.
- **Unchanged:** collision rows and margins, the slack gate, the arm stream, the blocked detector, goal checks,
  `/cmd_vel` ownership. B5 keeps the final say on safety.
- **Fallback.** No plan, a plan older than `medium_stale_s` (0.5 s; the medium period plus its cap plus 0.16 s, 0.84 s
  at 2.5 Hz, amendment 2026-11-01), a different `route_id`, or a plan more than
  `medium_max_offset_m` (0.3 m) from B5's measured base: B5 uses frozen-B5 behaviour for that cycle (Nav2 route,
  blend, side picker). Thresholds are start values for the bench.
- **`MpcCycle` fields:** reference source used, medium plan age, `plan_id`, `route_id`, fallback reason.

## 5. Prerequisites (before the medium layer)

### 5.1 Base friction feedforward

- In the Unity skid-steer actuator: a friction feedforward (or small integral term) so the base starts from rest
  without the 0.0375 m/s / 0.07 rad/s breakaway every controller works around (roadmap Phase 6 "From the Unity
  controller review"; it causes the RPP rotate deadlock and the gravel goal stop).
- Qualification: the base checks of [unity-skid-steer-base-controller.md](../../unity-skid-steer-base-controller.md),
  then a B5 smoke run (open yard, one door, chicane). B5's breakaway handling (`own_*` in the blocked config) is
  re-checked against the new base.
- A comparability-log row: it changes every baseline (B1-B5), resolved in the final baseline run.

### 5.2 Fold pose (seed and torque calibration)

- One lowered configuration found with the existing `carry_pose_search`: panel top ≤ 1.20 m, inside the footprint,
  about 22 Nm static shoulder torque (4B-A's fold estimate).
- Held and transitioned (from `carry`) in Unity like `carry` was qualified, then added to `qualified_payload.json`
  as `fold`.
- In 1b it is a seed, not the reachable set; the live hold and tracking check calibrates the static-torque limits of
  Section 3.3.

### 5.3 Sequencing scenario `v2_door_then_beam_nav`

- A narrow door with a low beam about 1-1.5 m behind it, so the plan must go from a tight pose (for the door) to a
  lowered pose (for the beam) inside B5's 2 s horizon.
- Built as a switchable V2 obstacle that only this scenario enables (the other 19 scenarios are unchanged).
  Feasibility of the spacing is checked in the kinematic bench (Section 6.2) before the spacing is fixed.
- A comparability-log row (scenario-set change).

## 6. Offline medium bench

Most tuning happens here, not in Unity.

### 6.1 Open-loop mode and solver choice

- Inputs: state snapshots (base, joints) and routes from the campaign 2 bags, every 0.5 s along each run, plus the
  scene; the five target cells are generated from their scenario start and route where no bag exists
  (`v2_door_then_beam_nav`).
- Per snapshot: one medium cycle from a cold start (full multi-start) and a sequence of warm cycles along the
  recorded path.
- Measures: cycle time (mean, median, p90, p95, p99, max) and its split (linearization, QP, other); SCP outer
  iterations and convergence rate per seed; seed chosen; plan clearance and torque margin; collision slack.
- **Dense-QP interface** with three backends, all benchmarked: HPIPM dense QP (already vendored), OSQP (already a
  dependency), DAQP (a small C library, added as a vendor package like `hpipm_vendor`, pinned tarball). The default
  is the backend with the lowest cycle p99 at a convergence rate not worse than the best; HPIPM dense is the expected
  default.
- The same runs pick horizon, segment count, degree and `collision_dt` within the budget.

### 6.2 Closed-loop kinematic mode

- The medium cycle at its rate plus B5's real `Controller::step` at 50 Hz (as `mpc_replay` uses it), with ideal
  actuators: the base and arm integrate B5's commands (with the arm stream's sampled reference), no Unity.
- Runs the seven campaign-A cells end to end (Section 7.1) and tunes `weights.medium_posture`, the clearance hinge
  and the progress weights. Catches homotopy flip-flop, fallbacks, stalls and B5 fighting the reference before any
  live run.
- **Gate to live runs:** all seven cells reach the goal; no executed plan with collision slack; medium p99
  ≤ 75 % of the medium period (150 ms at 5 Hz, 300 ms at 2.5 Hz; amendment 2026-11-01); no homotopy switch after the
  first choice in the swing-side cells; and (amendment 2026-11-01) the same at the hot host's compute, an iteration
  budget per cycle standing in for the cap.

## 7. Live campaigns and acceptance

### 7.1 Campaign A: acceptance (about 1.75 h)

Seven cells, F2 and B5, 3 runs each (42 runs), both on the friction-feedforward base, so this is the like-for-like
comparison.

| Must-fix | Cells | F2 passes if |
|---|---|---|
| Door alignment | `v2_room_r4_nav`, `v2_store_room_nav` | 3/3 each |
| Swing side | `v2_gate_right_swing_nav`, `v2_door_105_nav` (symmetric) | 3/3 each, no homotopy switch after the first choice |
| Low pass | `v2_low_beam_123_nav`, `v2_low_beam_130_nav` | 3/3, peak static shoulder torque within the calibrated limit |
| Sequencing | `v2_door_then_beam_nav` | 3/3 (B5 reported, not judged) |

Every F2 run also:
- no contact;
- 3D clearance ≥ 0.045 m (a miss is attributed to plan or execution as in Phase 4B-C);
- B5 MPC p99 ≤ 10 ms and misses < 1 % with the medium node running;
- medium cycle p99 ≤ 75 % of the medium period (150 ms at 5 Hz, 300 ms at 2.5 Hz; amendment 2026-11-01);
- fallback cycles < 1 % of the moving time;
- arm tracking within 0.10 rad.

### 7.2 Campaign B: no regression (about 3 h)

F2 on the campaign 2 exit set (34 runs). Passes with ≥ 30/34 successes, no new failure mode and no more contacts than
campaign 2.
- The worker cells stay B5's job in 5A; a worse F2 result there is recorded and handed to 5C, not fixed in 5A.
- B5 on the new base is not rerun on this set (the final baseline run does that; comparability log), so campaign B
  compares against campaign 2 with the base change stated.

### 7.3 Metrics

Roadmap Section 7: medium timing statistics and CPU share per process; SCP iterations and convergence; seed and
homotopy chosen, homotopy switches and time per homotopy; full multi-starts; fallback count and reasons; infeasible
plans; traversal time, path length, clearance, torque peaks; B5 timing and tracking as in campaign 2.

### 7.4 Failures

A failed criterion is reported with its data and cause for a user ruling, as in Phase 4B. Criteria are not relaxed
silently.

## 8. Order of work and checkpoints

One commit per change on `phase5a-medium`. CP = checkpoint report to the user before continuing.

1. Branch `phase5a-medium` off `demo-draggables` (done with this spec).
2. Base friction feedforward, base qualification, B5 smoke, comparability row. **CP**
3. Fold pose: search, Unity hold and transition, `qualified_payload.json`. **CP**
4. `v2_door_then_beam_nav`: switchable obstacle, scenario entry, comparability row.
5. Interfaces and building blocks: `MediumPlan` / `MediumCycle` messages; B5's FK, collision and scene code exported
   as a library if needed; a Bézier basis library (degree, segments, continuity, derivatives, hull bounds).
6. The medium problem: cost, constraints, linearization, dense-QP interface with HPIPM dense, OSQP and DAQP
   (`daqp_vendor`), SCP, seeds and hysteresis.
7. Open-loop bench and solver comparison; choose the default backend and the representation parameters. **CP**
8. B5: route announcement, `reference_source: medium`, fallback, `MpcCycle` fields; golden test unchanged.
9. `medium_horizon_node`, launch argument `medium:=bezier|none` (default `none`), runner flag `--medium`, RViz markers.
10. Closed-loop kinematic bench, tuning, gate. **CP**
11. Campaign A. **CP**
12. Campaign B. **CP**
13. Documentation and freeze (Section 9).

Working rules: runtime nodes in C++, tools and launch files in Python; stop Play mode after each task; `caffeinate`
for long runs; never overlap Play with timed runs (fanless host); `pkill` orphaned mock processes before mock tests.

## 9. Documentation and decisions

- `docs/experiments/medium-horizon/phase5a.md`: results, solver comparison, campaigns, and the problem / cause /
  resolution table, kept as problems are solved.
- ADR 0015: the medium-layer interface and ownership (route announcement, arc-length references, fallback) and
  formulation 1b, including the solver choice.
- Roadmap: Phase 5A outcome; decision-log entries (formulation 1b, F4 optional, friction feedforward timing, solver);
  the F4 open decision marked resolved (optional); comparability rows (friction feedforward, the new scenario, the
  fold pose); the F2 row of the baseline matrix filled in.
- A local tag `f2-frozen-<date>` at the end (ask before pushing).

## 10. Testing

- **Unit (C++, gtest):**
  - Bézier basis: C2 continuity at the joins, derivatives against finite differences, hull bounds containing sampled
    values, start-state equalities.
  - Problem: collision, torque and skid-steer linearizations against finite differences; QP backends agree on random
    small QPs and on recorded medium QPs within tolerance.
  - SCP on synthetic scenes: an angled 1.05 m door (the plan enters within a few degrees of the door axis), a beam
    under which `carry` collides (the solution lowers the panel within the torque limit), a symmetric gate with a
    bias (the side follows the bias; no flip over repeated warm cycles), a seed merge case.
  - B5 medium mode: route rebuild and splice, per-stage yaw/v/q lookup, every fallback reason, and that
    `reference_source: nav2` reproduces the golden test bit for bit.
- **Integration:** the closed-loop kinematic bench (Section 6.2) as a scripted test on at least the angled-door and
  beam cells; the medium node's launch with `medium:=none` leaves the B5 graph unchanged.
- **Live:** campaigns A and B.

## 11. Risks

| Risk | Mitigation |
|---|---|
| SCP misses its budget or converges poorly (nonconvex collision) | Seeds, trust region, cap with the last valid plan; then fewer samples, one pose seed per cycle round-robin beside the incumbent, or 2-3 Hz |
| CPU and heat on the fanless host (5 Hz × up to 150 ms is up to 75 % of a core) | Measured in step 7; B5 p99 criterion in campaign A; lower rate if needed (happened: 2.5 Hz, amendment 2026-11-01) |
| B5 fights the medium reference (its rows push off a reference near an obstacle) | Kinematic loop; `weights.medium_posture`; the medium margins equal B5's |
| The static torque proxy misses dynamic loads | Live tracking criterion; Unity motion feedforward already on |
| The door-then-beam spacing is infeasible | Feasibility checked in the kinematic bench before the spacing is fixed |
| A fold pose meeting 1.20 m / 22 Nm does not exist or does not hold in Unity | CP after step 3: user ruling on a higher top or a different lowered seed (extend only) |

## 12. Amendments

**2026-11-01: the medium rate profile (user decision).** Campaign A's first attempt failed at the 1.05 m doors and the
gate (`docs/experiments/medium-horizon/phase5a.md`, "Campaign A, first attempt"): live, on the fanless MacBook Air, the
medium cycles got about half the compute the bench had once the host was hot, and the planner, after the selection fixes
it led to, still stalled at that compute at 5 Hz. At 2.5 Hz with a 0.28 s cap (the same 70 % of a core) the closed-loop
gate passed at the hot host's compute. So:
- `medium.yaml` (the installed default) runs at 2.5 Hz with a 0.28 s cap; `medium_5hz.yaml` keeps 5 Hz with 0.14 s for
  a faster host (the user's PC); both plan the same problem, and a launch argument selects the file;
- B5's `medium_stale_s` follows the profile: the medium period plus its cap plus 0.16 s (0.5 s at 5 Hz, as verified in
  Tasks 10-12; 0.84 s at 2.5 Hz), set by `navigation.launch.py`;
- the medium p99 criteria (Sections 6.2 and 7.1) are 75 % of the medium period; runs record their profile (rate and
  cap) and host, and results of different profiles or hosts are never pooled;
- the kinematic gate also runs under an SCP iteration budget per cycle (reproducible, the hot host's compute) beside
  the wall-clock cap.

**2026-11-01: the selection (Section 3.5), after campaign A's first attempt (user approval of the fix direction).** A
pose seed the cap cuts short resumes from its last iterate on its next turn (within 2 s). A published plan that waits
(its path below half of min(the reach at the reference speed, the route left), unless it is short for a turn in place)
widens the next cycle to every pose seed in turn order, with seeds that hold the base while the arm reconfigures, and
limits the incumbent to 5 SCP iterations. When the cheapest feasible plan slows (below 75 % of that reach), a feasible
plan reaching 0.25 m further wins; when it moves, plans that slow are not eligible. The homotopy hysteresis (15 %) holds
among the eligible plans, and a plan on the other swing side than the last published one must be 50 % cheaper
(`side_hysteresis`). The bench can cut cycles by an SCP iteration budget instead of the wall-clock cap.
