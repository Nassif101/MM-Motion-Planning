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
- Task reviews by the user (docs/reviews/README.md): after each task a commit, `docs/reviews/e2/task-NN-prompt.md`,
  the user's `task-NN-review.md`, then `task-NN-response.md`; the next task starts after that.
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
