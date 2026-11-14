# E2: tracking-aware prediction (execution fixes, sub-project 2)

Date: 2026-11-11. Branch: `e2-tracking-prediction` (off main 5914d27). Status: design approved section by section
(user, 2026-11-11); this document awaits review.

## 1. Context and scope

The execution fixes (roadmap "Priorities after Phase 5A", step 3) are split into five sub-projects (E1 spec, Section
1). E1 (the slack gate's absolute rule and the infeasibility fallback) is accepted and merged (ADR 0017,
[e1.md](../../experiments/execution-fixes/e1.md)). E2 is the second: B5 plans what the plant will do.

**The goal.** B5's prediction model is a kinematic unicycle with arm joint velocities as controls
(`mobile_manipulator_mpc/model.hpp`): no lateral motion, no delay, no lag. The realistic MuJoCo plant differs
(roadmap step 3, "Plant behaviours that change planner results"; commissioning.md): inward sideslip in turns (about
55 mm entering store_room's door), understeer (yaw rate x0.79 at (0.3 m/s, 0.1 rad/s), x0.96 at 0.2 rad/s in place,
x0.87 at 0.6 rad/s; 0.67-0.77 on the gentlest turn across arm poses), turning chatter (about 2 mrad and 1 mm per
20 ms cycle), speed tracking 0.99, a 30 ms arm command delay. The planners must become robust to these, not have them
tuned away in the plant (user, CP5 2026-11-07).

**E1's hand-offs** (e1.md, "Findings (c) and (d)"):

- the depth ratchet's cause: in the door cells every plan predicted the deepest pair leaving (-0.9 mm per 0.1 s) while
  the nearly stationary base (0.002 m/s, about 0.03 rad/s) ended each 0.1 s 1-2 mm deeper. Two candidates, not told
  apart: B5's base model against the plant's wheel-contact base, or commands below the breakaway that the plant does
  not execute as B5 assumes;
- store_room: B5 alone reaches 0 of 20 (the 1.05 m door leaves 4 cm per side against about 55 mm of sideslip);
- gate_right_swing F2: the left swing fails on 2 of 3 seeds (not a flip);
- the `deep` holds under the absolute rule (pairs more than 5 mm inside a margin).

**In scope:** a measurement of B5's prediction against the plant and its attribution per effect; an identified
tracking model (parameters as data, one file per plant) in B5's prediction; delay compensation; the on/off effect of
the plant's arm motion feedforward; the judgment on the plant's ensembles.

**Out of scope:** the learned tracking margin (E3); continuous-time collision rows and the panel and low-beam rows'
linearization error (E4); Unity runs, Unity's own parameter set and the B5 re-freeze (E5); the medium planner's model
and its sensitivity to sub-millimetre differences (5B); changes to the plant's physics.

**Found while exploring (2026-11-11).** Roadmap part (a), the arm motion feedforward in the plant, is already built
and on: `ArmServo` adds inverse dynamics along the reference beside the gravity term (`motion_ff: true` in
`profile.yaml` and `profile_unity.yaml`). What remains of (a) is one on/off measurement (Section 3.1, T8).

**Decisions (user, 2026-11-11):**

1. Measure first: the model terms are chosen at a checkpoint from a short list this spec fixes, after the measurement.
2. Plant knowledge enters as identified parameters in a per-plant file. An online residual estimate on top is the
   named escalation, taken only if the grip ensemble shows the identified parameters are not enough.
3. Success: the gates of Section 4. Of the three hand-off cells at least one improves significantly and none gets
   worse; each one not fixed must be shown, with evidence, not to be a prediction problem.
4. Architecture: a static command map and delay compensation with the state unchanged (9). A lateral-velocity state
   is the named escalation, taken only if the measured build-up time is one stage (0.1 s) or longer.
5. E2 changes B5's model only; the medium planner keeps its model (assumption stated to the user, not contradicted).

## 2. Structure

Four stages, a checkpoint after each (Section 5):

| Stage | Content | Ends at |
|---|---|---|
| 1 Measure | the rollout tool; B5's model against the measured state on diagnostic runs; attribution per degree of freedom and per effect | CP1 |
| 2 Identify | the identification run on the plant; the chosen terms' parameters in a per-plant file | CP2 |
| 3 Model | the terms in B5 behind the parameter file; identity reproduces today's B5; prediction error re-measured | CP3 |
| 4 Judge | the ensembles, the grip ensemble, the hand-off cells, attribution of what is not fixed; defaults, ADR 0018, e2.md, roadmap | CP4 |

**The short list of terms** (CP1 picks; a term not picked is recorded with its number):

| Term | What it models |
|---|---|
| delay compensation | the initial state advanced to the actuation time with the commands already sent |
| speed ratio | executed forward speed against the commanded |
| turn gain | executed yaw rate against the commanded, as a function of the command |
| sideslip | a lateral body velocity as a function of the commanded twist |
| deadband | commands below the breakaway that the base does not execute |
| arm command delay | through `arm_stream_lead_s` or the same forward prediction |
| escalation: lateral-velocity state | sideslip that builds and decays over a stage or longer (state 9 -> 10) |

## 3. Design

### 3.1 Stage 1: the measurement

**Tool.** `tools/plant/model_rollout.py` with `test_model_rollout.py`. From a run's logs it takes the measured state
at a frame and the commands executed after it, rolls B5's model forward and compares with the measured state later.
It calls the generated model B5 uses (no second implementation of the model).

**Reports**, per run and per window:

- the prediction error over 20 ms, 0.1 s (one stage) and 0.5 s: body-frame forward, lateral and yaw for the base, per
  joint for the arm;
- the error against the commanded twist, binned by |v| and |omega|, with bins below the plant's breakaway (0.005 m/s,
  0.02 rad/s);
- the delay and the time constant that best align command and response, for the base twist and each arm joint;
- on the ratchet runs: the measured pose change projected onto the deepest pair's distance gradient, per degree of
  freedom.

**Diagnostics extension.** `plant_campaign --diagnostics` gains the deepest pair's name and its distance gradient
with respect to the state in the State frame's `diag`. Off by default: logs without the flag do not change.

**Runs:**

- the five runs of `experiment_runs/e1/ratchet`, rerun with the extended diagnostics (each must reproduce its
  original's State and Event frames);
- store_room, B5 alone, 5 seeds: the turn into the door;
- gate_right_swing F2: the seeds starting left and as many starting right;
- an open-loop excitation on the plant (no B5): steps and slow ramps in v and omega through the breakaway range, and
  turns at the commissioning points;
- the arm motion feedforward: the commissioning move with `motion_ff` on and off (T8).

**Decision rule at CP1.** A term enters when it explains at least 20 % of the one-stage error on the windows where
it applies, or when it reverses the predicted direction of the deepest pair on the ratchet runs. The lateral-velocity
state enters when the fitted build-up time is 0.1 s or longer. The 20 % is a starting figure: if several small terms
matter only together, that goes to CP1 as a finding, the rule is not applied mechanically.

**Output.** e2.md "Measurement" with the tables; the CP1 report with a recommendation per term.

### 3.2 Stage 2: identification

**Tool.** `tools/plant/identify_tracking.py` with tests. It drives the plant open loop through a fixed excitation,
fits the chosen terms and writes the parameter file. It reuses the plant driving and fitting of `commission_plant.py`
and `calibrate_plant.py` where they fit.

**Excitation:** steady twists on a grid inside the operating envelope, denser at small yaw rates; steps and slow
ramps through the breakaway range, from rest and moving; transitions straight -> turn -> straight (sideslip building
and decaying); each in the five qualified arm poses (carry, home, vertical carry, level extension, fold); for the arm
delay the commissioning move and a sine per joint.

**Fit.** Least squares of the model's one-stage prediction against the measured motion, per term; half the
excitation held out. The fit is checked a second time on stage 1's closed-loop runs (B5's own commands: chatter,
small corrections).

**Pose dependence.** If one parameter set does not cover the five poses, CP2 chooses between a single compromise set
and a parameter that depends on the arm's reach. The second is built only if the data requires it.

**CP2 shows:** the functional forms and parameters; the held-out one-stage error before and after, per body axis and
per joint; the same on the closed-loop runs, the ratchet windows included; the fit repeated on plants at mu 0.4 and
0.8 (how far the parameters move with grip).

**Accepted at CP2 when** the held-out one-stage error falls on every axis that got a term and no axis gets worse. A
parameter whose sign at mu 0.4 or 0.8 is opposite to mu 0.6 is flagged there for the escalation to the online
residual, before stage 3.

**Determinism.** A rerun of the tool writes the same file (tested).

### 3.3 Stage 3: the model in B5

**Parameters.** A struct `TrackingModel` with the chosen terms, loaded from a YAML file in
`mobile_manipulator_mpc/config/` (`tracking_model_mujoco.yaml`; Unity's in E5), selected by `tracking_model` in
`wholebody_mpc.yaml`. B5 does not read the plant's `profile.yaml` (the framework does not depend on the simulator).

**Command map.** The model's body motion from the commanded (v, omega):

- forward speed = speed ratio x v;
- yaw rate = turn gain(v, omega) x omega;
- lateral speed = sideslip(v, omega), zero today.

The functional forms are fixed at CP2. Every form is smooth (the QP linearizes the model each cycle; a hard deadband
has zero slope below the breakaway, so the QP would see such commands as having no effect) and is the identity at
zero parameters.

**Where.** The map goes into the CasADi model (`scripts/generate_model.py`) with the parameters as runtime inputs:
one build for every plant. `discrete_step` takes the `TrackingModel`; its callers in the formulation, the
controller, the slack gate and the replay bench pass B5's loaded set. The medium planner and its bench pass the
identity.

**Reproducibility.** The existing generated function stays as it is and an identity `TrackingModel` dispatches to it;
only a non-identity set uses the new function. Identity therefore reproduces today's runs frame by frame by
construction (T6).

**Delay compensation.** Before each solve the controller advances the measured state by the identified delay with the
commands it sent and plans from that state (it already keeps its last commands and a one-step prediction). Base and
arm delays are separate parameters.

**Errors and logging.** The loader rejects unknown keys, non-finite values and values outside stated ranges; the node
and the campaign session refuse to start on a bad file. A run's header records the tracking model's file and hash.

**Escalations** are spec amendments, written when taken: the lateral-velocity state at CP1 (state 10, solver
backends, golden tests); the online residual at CP2 or after T3.

### 3.4 Stage 4: the judgment

Closed-loop prediction first (CP3, about 20 runs with the model on, through the rollout tool), then the ensembles:
the corpora, 20 seeds, realistic mode and stall setting of the baselines, through `e1_ensembles.sh variant` with a B5
config selecting the tracking model:

- F2 against `experiment_runs/e1/absinf`, B5 alone against `experiment_runs/e1/abs`: campaign A twins, the chicane
  twins, campaign B;
- the grip ensemble (mu 0.4 and 0.8) with the parameters identified at mu 0.6, against the grip baseline in
  `absinf`.

Tools: `compare_variants.py` (regressions with Fisher, locks by kind), `divergence.py` (flips), `model_rollout.py`.
Long runs start only with the user's go-ahead (power, lid open: host-sleep-runs).

## 4. Acceptance

The criteria are numbered T1-T8 (T for tracking; B5 is the baseline's name).

| # | Criterion |
|---|---|
| T1 | Prediction: on the closed-loop reference runs the one-stage error falls on every axis that got a term, and on the ratchet runs the predicted direction of the deepest pair matches the measured one |
| T2 | No regressions against `absinf` (F2) and `abs` (B5): reached per cell (Fisher), contacts stay 0, clearance medians within 1 mm, no step locks |
| T3 | Grip: no regression at mu 0.4 and 0.8 against the grip baseline. A failure triggers the escalation to the online residual (amendment) |
| T4 | Hand-off cells: at least one of store_room B5 alone (0/20), the gate_right_swing left swing and the `deep` hold count improves significantly, and none gets worse |
| T5 | Every hand-off cell not fixed is cleared: the one-stage prediction error in its failure windows is below the deepest pair's clearance change over that stage (prediction could not have changed the gate's verdict), and the failure is attributed with evidence to a named cause outside E2. A cell neither fixed nor cleared keeps E2 open |
| T6 | Reproducibility and tests: the identity model reproduces `absinf` and `abs` runs frame by frame on three cells; the C++ and Python suites are green |
| T7 | Solve time: B5's median and p99 solve time with the model on within 10 % of today's |
| T8 | The arm motion feedforward's on/off effect on the arm tracking error is measured and recorded |

If T5's measure cannot be computed for a cell, that is raised at CP4; the wording is not stretched.

**Adoption.** With T1-T8 met the MuJoCo tracking model becomes the default for plant runs; for Unity the default
stays the identity until E5 identifies Unity's set. ADR 0018; the comparability log entry is provisional until E5.

## 5. Checkpoints

| | After | The user rules on |
|---|---|---|
| CP1 | stage 1 | the terms to build; the lateral-velocity state or not; which ratchet candidate the data supports |
| CP2 | stage 2 | the functional forms and parameters; pose dependence; the grip sensitivity (escalation or not) |
| CP3 | stage 3 | T1 on closed-loop runs; the go-ahead for the ensembles |
| CP4 | stage 4 | T1-T8; adoption; merge and push |

## 6. Tests

Test-first. C++ (`mobile_manipulator_mpc`, `mobile_manipulator_campaign`):

- identity reproduces the golden plans bit for bit;
- the model's Jacobians against finite differences with non-zero parameters;
- each term moves the prediction in the expected direction on a hand-computed case;
- delay compensation: a known command history gives the hand-computed advanced state;
- the loader's rejections; the run header's tracking-model entry;
- the diagnostics extension: the named pair and its gradient against a finite difference of the distance.

Python (`tools/plant`): the rollout tool on a synthetic log with a known model error; the identification tool
recovering known parameters from a synthetic plant response, and its determinism.

## 7. Records and process

- Branch `e2-tracking-prediction`, one commit per task, none on main (git-branch-workflow).
- Task reviews by the user: after each task a commit, a review prompt, the user's review, then the response (the
  review files are kept outside git); the next task starts after that.
- Judgments on ensembles, never on single runs (the loop is chaotic). Every attribution is checked against the data
  before it is claimed.
- e2.md carries the measurement, identification, results, acceptance, and the problem / cause / resolution table;
  the roadmap's decision log and step 3 are updated as decisions fall; ADR 0018 at adoption.
- Container builds: `MAKEFLAGS=-j3 colcon build --parallel-workers 1`.

## 8. Risks

- **A correct prediction makes B5 more cautious.** store_room may turn from late holds into earlier ones, not into
  passes. T4 and T5 are worded for that.
- **Chatter is not predictable by a smooth map.** Stage 1 measures its share of the error; what remains goes to E3's
  margin with its number.
- **Grip.** Parameters identified at mu 0.6 may mislead at 0.4 or 0.8 (T3; CP2's sensitivity previews it).
- **The deadband's smoothing** trades fidelity below the breakaway against the QP's gradient; its width is an
  identified parameter with a stated lower bound.

## Amendment 1 (CP1, 2026-11-12)

**Finding** (e2.md "Measurement", Finding 1). E1's depth ratchet is caused by B5's output stage, not by the plant.
The plan's first control is clamped to the rate band around the last command. `breakaway_output` then raises a
sub-breakaway command to the breakaway (0.07 rad/s) in the clamped command's sign whenever the base reads at rest.
After one kick, a plan correcting the other way is clamped to the old sign and boosted again: the turn latches against
the plan. B5's plans predict the pair leaving; the sent commands drive it in.

**User rulings (CP1):**

1. Fix the latch in E2: new Task 4b, before Task 5. The breakaway boost applies only in the direction the plan asks
   for. When the rate band leaves a component opposing the plan, or the plan's component is inside the dead zone, that
   component is sent as clamped (decaying at the rate limit) and not boosted. Switch
   `breakaway_follows_plan: true|false` in `wholebody_mpc.yaml`; false is the frozen behaviour until it is judged.
2. Model terms to build: turn gain as a function of |omega| (its form fixed at CP2), sideslip k v omega (k from Task
   6), speed falling with |omega| (added by the user although its share is 8 / 15 %; dropped at CP3, user
   2026-11-13: it made the closed-loop forward prediction worse at every grip), and arm delay compensation
   (reversed after the Task 8 review, user 2026-11-13: B5's arm state is the streamed reference, so there is no pending
   arm command to advance; `arm_delay_s: 0`, see the roadmap decision log). Out:
   the lateral-velocity state (provisionally: Task 6 re-tests it on held-out data), a constant speed ratio, base delay
   compensation, a hard deadband.
3. store_room for B5 alone: a probe first (Task 4b). Under `--diagnostics`, each State frame records the true nearest
   robot-obstacle pair at B5's state, its distance, whether the cycle's collision rows carry that pair (at stage 1 and
   at any stage), and the rows dropped by the per-stage budget. The probe tells whether the pair that appears 5-7 mm
   deep was missing from the rows (budget or activation: collision rows, E4 or a configuration change) or present but
   predicted clear (prediction or linearization). The user then rules: fix in E2 or defer.

**Acceptance additions:**

- T9: with `breakaway_follows_plan` on, the three ratchet episodes' runs send no yaw command that grows in magnitude
  against the plan's turn (the rate-limited decay of a reversed command is allowed), and show no contact from the
  drift at rest. Wording made precise after the Task 4b review: the "latched cycles" indicator also counts the
  legitimate decay.
- T10: the switch is judged on ensembles like E1's switches: no regression against `absinf` / `abs` (T2's criteria),
  before it is adopted.

T4's hand-off cells and T5's clearing apply to the fixed B5.

**Ruling on store_room (user, 2026-11-12, after Task 4b's probe):** defer the approach and hold policy (5B, or a gate
phase). store_room stays an E2 hand-off cell: it is re-measured at CP4 with the model in place, and T5 decides
whether a remaining failure is cleared as outside E2. The probe did not clear prediction: the base ended 2-4 mm
closer than B5's model over the turn-in and the braking.

## Amendment 2 (CP2, 2026-11-12): identified model plus an online correction

**Finding** (e2.md "Identification", closed-loop grip fits). The turn response depends strongly on grip:

| mu | g_inf | g_0 |
|---|---|---|
| 0.4 | 1.27: the base slides | 0.64 |
| 0.6 | 1.01 | 0.14 |
| 0.8 | 0.84 | 0.01 |

g_inf's delta changes sign across the grips. The mu 0.6 model makes the yaw prediction 41 % worse than the unicycle
at mu 0.4. That is the escalation trigger of Section 3.2 (decision 2, Section 1).

**User rulings (CP2, 2026-11-12):**

- Adopt the closed-loop fit as `tracking_model_mujoco.yaml`.
- One parameter set across arm configurations, with documented gaps: the yaw near vertical carry, and the base while
  the arm reconfigures. Arm-base coupling goes to E3.
- Escalate to an identified model plus an online correction, with this design (approved as proposed).

**Design.**

- **What is estimated.** g_inf, g_0 and the sideslip k. The turn map is linear in (g_inf, g_0) for a fixed c, and the
  lateral speed is linear in k. c and the speed-in-turn term stay at their identified values.
- **The estimator.** A Kalman filter with a random-walk state on (g_inf, g_0, k).
  - Prior: the identified values, with spreads that reach the mu 0.4 and 0.8 fits (g_inf +-0.3, g_0 +-0.5,
    k +-0.2).
  - Measurement: every 0.1 s, the mean measured yaw rate and lateral speed over the last five 20 ms cycles, from B5's
    measured base states, against the map's regressors for the commands B5 sent (the time convention).
  - Process noise: sized so the estimate follows a grip change within a few seconds of turning.
  - Bounds: the state is clamped to the loader's ranges, and the covariance is capped (no windup).
  - Only excited directions update. Turning at speed informs g_inf, slow turns g_0, and straight driving neither;
    without excitation the estimate stays where it was.
- **Where.** `TrackingEstimator` in `mobile_manipulator_mpc` (ROS-free), behind the switch `tracking_adaptation`
  (false: the fixed identified model). Each cycle the controller updates it from the measured state and the command
  it sent, and uses its current values in `discrete_step` for the whole QP of that cycle. It is deterministic.
- **Logged** under `--diagnostics` (`diag_track`: g_inf, g_0, k), so the rollout checks the prediction B5 made.

**Acceptance addition:**

- T11: at mu 0.4, 0.6 and 0.8, after the estimate settles, the adapted model's one-stage yaw and lateral errors are no
  worse than the unicycle's. At each grip it must also remove at least half of what that grip's own fixed fit removes.

T3 (the grip ensemble) stays the outcome judgment.

**Out (YAGNI).** Estimating c or the speed term; configuration-dependent parameters; arm-base coupling.

**Amendment 2, narrowed after the Task 6b review (2026-11-12; the review's C1, I1-I3).** Replayed over the 52 real
logs, the first estimator improved the one-stage prediction at every grip. Over B5's 1-2 s horizon, though, it was
worse than the fixed prior at mu 0.6. Two kinds of window the static map cannot explain pulled g_inf down: ramps at
the start of motion (the base's lag) and slow in-place turns (stiction). g_0 and k wandered to their bounds.

- **What is estimated: g_inf and k only.** g_0 stays at its identified value: it matters only below c, inside the
  stiction band, and is weakly identified.
- **Gates: these windows are not learned.**
  - a window whose commanded yaw rate spans more than 0.03 rad/s (a ramp);
  - a window before the command has been steady for 0.3 s (the lag tail);
  - for the turn gain, a window whose largest commanded |yaw rate| is below 0.08 rad/s (the stiction band);
  - for k, a window with |mean v x omega| below 0.005;
  - a window whose normalized innovation squared exceeds 9;
  - a turn window that measured less than 0.2 of the predicted turn (blocked wheels).
- **The initial covariance is 0.1 of the cap,** because the prior is an identified model.
- **T11 is restated.** At B5's horizon, from every 0.1 s start over the logged commands at mu 0.4, 0.6 and 0.8, the
  adapted model's yaw and lateral error rms at 1 s and 2 s are no worse than the fixed prior's at mu 0.6, and no worse
  than the unicycle's at every grip. Two slices are reported beside all moving starts: the first second of motion,
  and the starts after an in-place slow turn. The tool is `identify_tracking.py horizon` (the C++ estimator through
  `tracking_replay`).
- **Logged values.** `diag_track` values are effective; only run medians of g_inf separate grips.

## Amendment 3 (after Task 10, 2026-11-13): the turning centre follows the centre of mass

**Finding** (e2.md "Ensembles and hand-off cells"). Task 10 fails T3 at mu 0.8 (store_room B5 10 -> 1 of 20, the
correction's doing), and T5 clears none of the nine remaining door holds. In every one, the prediction misses lateral
motion. The base turns about a point that is not its frame. Fitted on the Task 6 excitation set per arm pose
(`identify_tracking.py offset-fit`, mid-heading velocities, non-overlapping 0.1 s windows; corrected after the Task 10
review I1), the body lateral speed is d x yaw rate:

| Pose | Centre of mass x in base_footprint (m) | d (m) |
|---|---|---|
| fold | -0.008 | 0.017 |
| carry | -0.010 | 0.029 |
| home | -0.028 | 0.068 |
| vertical_carry | -0.028 | 0.066 |
| level_extension | +0.060 | -0.131 |

The least-squares line d = 0.003 - 2.23 x_com matches all five within 3.4 mm. With d in the fit, the sideslip
coefficient k is -0.03 to -0.06 at every pose. Pooled across poses the offsets largely cancel, and Task 6's pooled
identification found k = 0.19 instead: k stood in, in part, for a pose-averaged offset. The centre-of-mass values
came from an ad hoc script over the URDF and the payload; Task 10b's `wholebody_com_x` test pins them. The online correction then drove k to its bound in tight turns, and on a grippy
floor that k predicts slides that do not happen.

**User ruling (2026-11-13):** add the offset term. This reverses CP2's "configuration-dependent parameters out" for
this one term only. The term is a function of the centre of mass, not a table per configuration. Other arm-base
coupling stays with E3.

**Design.**
- **The map.** Lateral speed = (turn_offset + turn_offset_com x x_com(q)) x yaw rate + sideslip x forward speed x yaw
  rate. The yaw rate is the turn map's output.
- **x_com(q).** The x of the whole robot's centre of mass in base_footprint: the URDF links' masses and the qualified
  payload, through the generated forward kinematics. It is a function of the arm joints in B5's state, so the state
  stays 9 and the QP's Jacobians include it.
- **Parameters.** The map's parameter vector grows from 5 to 7 (`turn_offset` in m, `turn_offset_com` dimensionless,
  both 0 in the identity). The bounds are [-0.3, 0.3] m and [-5, 5].
- **Identification.** `e2_identify.sh` fits the six terms (turn_delta_inf, turn_delta_zero, turn_scale, sideslip,
  turn_offset, turn_offset_com). The report shows the held-out one-stage rms per pose.
- **The online correction.** It still estimates g_inf and k. Its lateral measurement subtracts the offset term at the
  window's mean arm state. The offset is not estimated online: it is geometry and load, not grip.
- **Unity.** Identity until E5, as before.

**Acceptance.** T1-T11 unchanged. The candidate, with the new model and the correction on, reruns:
- Task 9's closed-loop check: T1, T11 and T7;
- Task 10's ensembles, the judge and T5.

The fixed-prior variant runs again where the correction regresses.

**Out.** Estimating the offset online; a lateral-velocity state; the medium planner's model (5B).
