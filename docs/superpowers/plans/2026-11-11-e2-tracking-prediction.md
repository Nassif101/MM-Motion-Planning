# E2: Tracking-Aware Prediction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Measure B5's prediction against the realistic MuJoCo plant, put an identified per-plant tracking model
(command map + delay compensation) into B5's prediction, and judge it on the plant's ensembles.

**Architecture:** Diagnostics in the run logs name the deepest gated pair, its gradient and the command B5 sent; a
Python rollout tool replays B5's own model (through a small C++ CLI) against the logged motion. A `TrackingModel`
parameter set, loaded from a per-plant YAML, enters `discrete_step`; an identity set dispatches to today's generated
function, so today's runs reproduce by construction. The controller advances the measured state by the identified
delays before each solve.

**Tech Stack:** C++17 (ROS-free libraries `mobile_manipulator_mpc`, `mobile_manipulator_medium`,
`mobile_manipulator_campaign`), CasADi code generation at build time, gtest/ament, Python 3 (`tools/plant`; standard
library on the host's 3.9, numpy in the container for the identification), pytest in the container, MuJoCo plant.

**Spec:** `docs/superpowers/specs/2026-11-11-e2-tracking-prediction-design.md`

## Global Constraints

- Branch `e2-tracking-prediction`; one commit per task or fix; no commits on main.
- Identity is the default everywhere until Task 11: `tracking_model: ""` in `wholebody_mpc.yaml`, `plant_campaign`
  without `--tracking-model`. An identity `TrackingModel` must reproduce today's runs frame by frame.
- C++ for runtime and library code; `tools/` in Python. `model_rollout.py` is standard library only and Python
  3.9-compatible (no `match`); `identify_tracking.py` runs in the container (numpy), like `calibrate_plant.py`.
- Builds and ctest always `-DCMAKE_BUILD_TYPE=RelWithDebInfo`, with `MAKEFLAGS=-j3 colcon build --parallel-workers 1`
  (default parallelism runs the container out of memory).
- pytest in the container needs ROS sourced: `source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash`.
- Ensembles: realistic mode, 20 seeds, stall 45 s, the corpora `docs/experiments/mujoco-plant/corpus_{campaign_a,
  campaign_b,grip}.json`, 4 jobs; never overlap two timed ensembles or Unity Play; power and lid open; each long run
  starts only with the user's go-ahead. New runs go to `experiment_runs/e2/<set>` (git-ignored).
- Baselines: F2 `experiment_runs/e1/absinf`, B5 alone `experiment_runs/e1/abs`; grip baseline inside `absinf`.
- Statistics: Fisher exact one-sided, p < 0.05 counts as an improvement or a regression; clearance regression = a
  cell's median true 3D minimum lower by more than 0.001 m.
- CP1 rule: a term enters when it explains at least 20 % of the one-stage (0.1 s) error on the windows where it
  applies, or reverses the predicted direction of the deepest pair on the ratchet runs; the lateral-velocity state
  enters when the fitted build-up time is 0.1 s or longer, measured against the executed twist (`build_time`'s
  `executed` fit; the `command` fit also carries the base's delay and lag: Task 3 review F1).
- Plant breakaway: 0.005 m/s, 0.02 rad/s. B5's stage: 0.1 s; the cycle: 0.02 s.
- After every task: commit, write `docs/reviews/e2/task-NN-prompt.md` (self-contained), wait for the user's
  `task-NN-review.md`, write `task-NN-response.md`; only then the next task. Stop at CP1-CP4 for the user's ruling.
- Records as problems are solved: `docs/experiments/execution-fixes/e2.md` (problem / cause / resolution table);
  the roadmap's decision log per ruling. Every attribution is checked against the data before it is written.

Container helpers (recreate in the session scratchpad, not in the repo): `cbuild PKG...` (the colcon build above with
`--packages-select`), `ctest_pkg PKG REGEX` (`colcon test --packages-select PKG --ctest-args -R REGEX` then `colcon
test-result --verbose`), `cpytest FILE` (sourced `python3 -m pytest FILE -q` in `/workspaces/mm-motion-planning`).
Container: `mm-motion-planning-ma-robot-sim-1`.

**Order against the spec's stages.** The model's plumbing (Task 5) is built before the identification (Task 6), so
the fit is evaluated through B5's own model and not a second implementation. The functional forms of Task 5 are
initial; CP2 fixes them. Tasks 5-8 are written for the full short list; CP1 strikes the terms not picked and CP2 may
change a form, each by a plan amendment commit before the affected task starts.

## Review Focus

1. **Zero and sub-breakaway commands with a non-identity model:** `discrete_step` at v = omega = 0 and at 1e-6 must
   return finite `x_next`, `A`, `B` (the deadband form divides by a command magnitude). Test in Task 5.
2. **Reverse and mirrored commands:** the prediction for (-v, omega), (v, -omega) must be the mirror image of
   (v, omega)'s (sideslip changes sign with omega and with v; the turn gain does not). Test in Task 5.
3. **A delay longer than the command history, and the first cycles of a run:** the advance uses the commands it has
   and zeros before the first one; no exception, no NaN. Test in Task 8.
4. **Logs the rollout tool cannot use:** a log recorded without `--diagnostics`, or with a gap (a goal change, a
   stalled run's last frames), must give a message naming the missing field or skip the window; never a `KeyError`
   or a window spanning the gap. Tests in Task 2.
5. **A tracking-model file that is missing, relative, or has a typo in a key:** the node and `plant_campaign` refuse
   to start and name the file and the key; a relative path resolves against the B5 config's directory. Tests in
   Tasks 5 and 7.

---

## Stage 1: measure

### Task 1: Diagnostics name the pair, its gradient and the sent command

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/controller.hpp` (CycleOutput, after
  `gate_depth_stage1_m`), `ros2_ws/src/mobile_manipulator_mpc/src/controller.cpp` (the gate, lines 360-380 and the
  reset at 689)
- Modify: `ros2_ws/src/mobile_manipulator_medium/include/mobile_manipulator_medium/closed_loop.hpp` (`Gate`),
  `ros2_ws/src/mobile_manipulator_medium/src/closed_loop.cpp:178`
- Modify: `ros2_ws/src/mobile_manipulator_campaign/src/session.cpp` (the State frame, lines 320-333)
- Test: `ros2_ws/src/mobile_manipulator_mpc/test/test_controller.cpp`,
  `ros2_ws/src/mobile_manipulator_campaign/test/test_session.cpp`

**Interfaces:**
- Produces, in `CycleOutput`: `std::string gate_pair;` (`"<primitive>|<link>|<obstacle>"` of the deepest gated pair
  with slack, empty without one) and `Eigen::Matrix<double, kNx, 1> gate_gradient = zero;` (d distance / d x of that
  pair at the measured state, the row's own linearization). Only filled with `ControllerOptions::diagnostics`.
- Produces, in `ClosedLoopStepper::Gate`: `std::string pair; Eigen::Matrix<double, 9, 1> gradient;` and
  `ClosedLoopStepper::last_command() -> const mmm::Control &` (the command B5 sent this cycle).
- Produces, in State frame headers under `--diagnostics`: `"diag_pair"` (string), `"diag_grad"` (9 numbers),
  `"diag_cmd"` (8 numbers: the sent v, omega, qdot1..6). `"diag"` keeps its three entries. Without `--diagnostics`
  the header is unchanged.

- [ ] **Step 1: Failing controller test** `SlackGateDiagnostics.NamesTheDeepestPairAndItsGradient` in
  `test_controller.cpp`: reuse the fixture of the existing `gate_depth_m` test (E1 Task 9); assert `gate_pair` is
  non-empty and names the fixture's obstacle; assert `gate_gradient` matches a central finite difference (step 1e-6)
  of that pair's true distance in x, y and yaw within 1e-4; assert both are empty / zero without diagnostics.
- [ ] **Step 2: Run** `ctest_pkg mobile_manipulator_mpc test_controller`; expected: FAIL (no member `gate_pair`).
- [ ] **Step 3: Implement** in the gate: keep the deepest row's pointer (already `deepest`); fill `gate_pair` from
  its primitive, link and obstacle names and `gate_gradient` from its linearized row; reset both where
  `gate_depth_m` is reset.
- [ ] **Step 4: Failing session test** `Session.DiagnosticsCarryPairGradientAndCommand` in `test_session.cpp`: a
  short diagnostics run; every State header has `diag_cmd` of length 8; frames with `diag[0] > 0` have a non-empty
  `diag_pair` and a `diag_grad` of length 9; a run without diagnostics has none of the three keys and is
  byte-identical to today's (compare with the existing no-diagnostics reference of that test file).
- [ ] **Step 5: Implement** the `Gate` fields, `last_command()`, and the three header entries in `session.cpp`.
- [ ] **Step 6: Run** `ctest_pkg mobile_manipulator_mpc test_controller`, `ctest_pkg mobile_manipulator_campaign
  "test_session|test_equivalence"`; expected: PASS.
- [ ] **Step 7: Commit** `E2 Task 1: diagnostics name the deepest pair, its gradient and the sent command`.

### Task 2: `model_step` CLI and the rollout core

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_mpc/src/model_step.cpp`; modify its `CMakeLists.txt` (executable
  `model_step`, linked to `mpc`, installed like `mpc_replay`)
- Create: `tools/plant/model_rollout.py`, `tools/plant/test_model_rollout.py`
- Test: `ros2_ws/src/mobile_manipulator_mpc/test/test_model.cpp` is not touched; the CLI is tested from pytest.

**Interfaces:**
- Produces: `model_step` reads lines of 18 numbers `dt x0..x8 u0..u7` on stdin (Task 5 adds 7 optional map
  parameters) and writes `x_next` (9 numbers, `%.17g`) per line; a malformed line: exit 2 with the line number.
- Produces in `model_rollout.py`:
  - `records(log_path) -> (hello, list[dict])`: one dict per State frame: `t`, `goal`, `pose` (9, the true state),
    `b5` (9, the state B5 received; in realistic mode its arm joints are the arm trajectory's reference, except in a
    reanchored cycle (reference more than 0.15 rad from the measured joints), where they are the measured joints),
    `twist` (the measured v, omega: block 2's first two), `qd_ref` (block 2's last six: the arm trajectory's
    velocity, clipped to the joint velocity limits; the JTC itself takes it unclipped), `cmd`
    (8, `diag_cmd`: the control B5 requested), `depth`, `depth_stage1`, `pair`, `grad`, `kink` (`diag_kink`).
    Raises `ValueError("<log>: recorded without --diagnostics (no diag_cmd)")` when `diag_cmd` is absent.
  - **Time convention (Task 1 review F2, pinned by `Session.DiagnosticCommandDrivesTheTransitionIntoItsFrame`):**
    record j's `cmd`, `pair`, `grad`, `kink`, `depth`, `depth_stage1` were evaluated and sent at `t[j] - 0.02`, at
    record j-1's `b5` state; the transition `pose[j-1] -> pose[j]` ran under `cmd[j]`. A rollout from `i0` to `i1`
    therefore uses `cmd[i0+1 .. i1]`, and a window's gradient is record `i0+1`'s. The measured `twist` and `qd_ref`
    are samples at `t[j]`, after the transition (Task 1 response check R2): `inputs="twist"` takes the mean of
    `twist[j-1]` and `twist[j]` (a trapezoid; the base's motion between samples is not logged) and `qd_ref[j-1]` (the
    sample the JTC holds over the interval). Test `test_twist_inputs_use_each_series_at_its_sample_time`.
  - `windows(records, horizon_s, stride=1) -> list[(i0, i1)]`: frame index pairs `horizon_s` apart; a window is
    dropped when `goal` changes inside it or two consecutive frames are more than 0.03 s apart.
  - `rollout(records, i0, i1, inputs, step) -> list[float]`: the model's state at `i1` from `pose[i0]`, stepping
    frame by frame (records `i0+1 .. i1`, the time convention above) with `inputs` = `"cmd"` (the requested control)
    or `"twist"` (the measured twist and `qd_ref`); `step`
    is `callable(list[(dt, x, u)]) -> list[x_next]`.
  - `error(records, i0, i1, predicted) -> dict`: `forward`, `lateral` (m, in the body frame at `i0`), `yaw` (rad,
    wrapped), `q` (6): measured minus predicted.
  - `container_step(container=CONTAINER, params=None)`: the `step` callable running `model_step` through `docker
    exec` in one batch per call.

- [ ] **Step 1: Failing tests** in `test_model_rollout.py` with a synthetic log written through `plant_stream.encode`
  (helper `write_log(tmp_path, frames)`) and a fake `step` (a Python unicycle):
  - `test_rollout_reproduces_a_unicycle_log`: frames generated by the fake unicycle under a command that changes
    every frame, with record j carrying the command of the transition into it; every 0.1 s window's `error` is
    below 1e-9 in all fields, and shifting the commands by one record makes it exceed 1e-4 (the convention matters).
  - `test_lateral_error_is_in_the_body_frame`: the log's poses get 1 mm of body-left drift per frame at yaw 1.0 rad;
    over 0.1 s `lateral == pytest.approx(0.005, abs=1e-6)`, `forward == pytest.approx(0, abs=1e-6)`.
  - `test_twist_and_cmd_inputs_differ`: the log's measured yaw rate is 0.79 of the command; with `inputs="twist"`
    the yaw error is ~0, with `"cmd"` it is `-0.21 * omega * 0.1`.
  - `test_windows_skip_goal_changes_and_gaps`: no window spans a `goal` change or a 0.1 s gap between frames.
  - `test_log_without_diagnostics_is_named`: `records` raises the `ValueError` above.
  - `test_container_step_matches_the_fake_on_a_unicycle` (marked `container`): `container_step()` against the fake
    for 20 random (x, u, dt = 0.02): within 1e-9.
- [ ] **Step 2: Run** `cpytest tools/plant/test_model_rollout.py`; expected: FAIL (no module).
- [ ] **Step 3: Implement** `model_step.cpp` (a loop over stdin calling `discrete_step`) and build it (`cbuild
  mobile_manipulator_mpc`).
- [ ] **Step 4: Implement** the five functions; frame layout from `divergence.decision_stream` (State blocks: true
  pose, B5's state, executed).
- [ ] **Step 5: Run** `cpytest tools/plant/test_model_rollout.py`; expected: PASS.
- [ ] **Step 6: Commit** `E2 Task 2: model_step CLI and the rollout core`.

### Task 3: The rollout reports

**Files:**
- Modify: `tools/plant/model_rollout.py`, `tools/plant/test_model_rollout.py`

**Interfaces:**
- Consumes: Task 2's `records`, `windows`, `rollout`, `rollout_many`, `error`, `container_step` (a context
  manager: every caller uses `with`). A record's `depth`, `depth_stage1`, `pair`, `grad`, `kink` are `None` where the
  log does not carry them (Task 2 review F3): `gradient_projection` skips such windows and reports their count per
  missing field; a missing `kink` never counts as smooth. Test `test_gradient_projection_skips_unavailable_fields`.
- Produces:
  - `horizon_table(records, step, horizons=(0.02, 0.1, 0.5), inputs=("cmd", "twist")) -> dict`: per horizon and
    input the p50, p90 and max of |forward|, |lateral|, |yaw| and of each joint's error.
  - `twist_bins(records, step, v_edges=(0, 0.005, 0.0375, 0.1, 0.3, 1.0), w_edges=(0, 0.02, 0.07, 0.2, 0.6, 2.0))
    -> {"bins": list[dict], "reversing": int}`: per (|v|, |omega|) bin of the net mean sent command over a 0.1 s
    window: the count, the mean signed forward, lateral and yaw error, and the command-weighted ratio measured /
    commanded for v and omega; windows whose command changes sign inside are left out and counted (Task 3 review).
  - `alignment(records) -> dict`: per channel (`v`, `omega`, `q0..q5`) the delay (0 to 0.2 s in 0.005 s steps) and
    first-order time constant (0 to 0.5 s in 0.005 s steps) minimizing the squared error between the filtered,
    delayed command and the measured response. Arm channels twice (Task 1 review F1): `pipeline` = `cmd` joint
    velocity against the measured joint velocity (finite difference of `pose`): what B5's model must account for,
    the input of the CP1 delay/lag terms; `servo` = `qd_ref` against the measured joint velocity: the plant's servo
    and command delay with the trajectory construction taken out, reported so its share is visible. Sample times
    (Task 1 response check R2), each series on its own clock: `cmd[j]` applies from `t[j] - 0.02`; `qd_ref[j]` and
    `twist[j]` are samples at `t[j]` (the JTC holds `qd_ref[j]` from its next tick); the measured velocities
    `(pose[j] - pose[j-1]) / dt` are interval means, stamped `t[j] - 0.01`. The delay is fitted on that common time
    axis (0.005 s grid); a sub-cycle phase the sampling cannot resolve is stated with the result (+-0.01 s), not
    reconstructed. Windows with a reanchored cycle (the `b5` arm joints equal to the `pose` ones) or a clipped
    `qd_ref` (at a joint's velocity limit) are left out of `servo` and counted.
  - `gradient_projection(records, depth_min=0.005, kink_max=1e-3) -> dict`: over frames with `depth >= depth_min`
    and a `grad`: per 0.1 s window the measured depth change, the plan's stage-1 prediction (`depth_stage1 -
    depth`), and `-slope[i] * dx[i]` summed per group: `forward`, `lateral` (the x, y parts of the pose change
    resolved in the body frame), `yaw`, `arm`; plus the same from the `cmd` rollout's pose change (what B5's model
    says), using the central `grad`. Windows where any component of `kink` exceeds `kink_max` in magnitude are
    `feature_switch` windows (Task 1 response check R1): per-coordinate one-sided slopes do not add up for a motion
    of several coordinates at a feature tie (a coupled shoulder-lift/elbow motion against a floor row reversed the
    sign), so these windows get no per-degree-of-freedom split. They report only the measured depth change and the
    plan's stage-1 prediction, counted separately and never pooled with the smooth ones. `dx` is taken from the `b5`
    states (the gradient's state), and the true-pose `dx` reported beside it.
  - `build_time(records) -> dict | None`: delay and first-order time constant of the lateral body velocity against
    `v * omega`, twice (Task 3 review F1): `executed` (the measured twist's product: the lateral dynamics, CP1's
    evidence) and `command` (the requested product: the effective response with the base's own delay and lag); None
    with fewer than 3 transitions of the requested product counted inside the parts (review F4).
  - Fits run through goal changes (a goal change does not reset the robot) and leave out the first 0.5 s of responses
    after a gap mid-run (review F2); the filter's interval means are exact. `servo` exclusions are counted per reason
    (`ideal`, `equal_states`, `at_limit`). `gradient_projection` counts censored windows as `pair_changed` (another
    deepest pair) or `pair_lost` (none with slack), every missing field, and adds per-window sums beside the medians.
    Report runs are keyed by their directory under the logs' common root (review F3).
  - CLI: `python3 tools/plant/model_rollout.py report LOG... --out DIR` writes `DIR/rollout.json` and
    `DIR/rollout.md` (one section per function above, per run and pooled).

- [ ] **Step 1: Failing tests** on synthetic logs:
  - `test_twist_bins_recover_a_turn_gain`: logs where measured omega = 0.79 omega_cmd at |omega| in [0.07, 0.2) and
    1.0 elsewhere: that bin's omega ratio is `pytest.approx(0.79, abs=0.01)`, the others 1.0.
  - `test_alignment_recovers_delay_and_lag`: v measured = the command delayed 0.04 s through a 0.06 s first-order
    filter: `delay == pytest.approx(0.04, abs=0.005)`, `tau == pytest.approx(0.06, abs=0.01)`.
  - `test_gradient_projection_splits_the_depth_change`: a pair with `grad` along body-lateral, the base drifting
    1 mm laterally per 0.1 s with zero commands: `lateral == pytest.approx(0.001, rel=0.05)`, `forward`, `yaw`,
    `arm` below 1e-6, and the `cmd` rollout's contribution 0.
  - `test_gradient_projection_does_not_split_feature_switch_windows`: `grad` yaw 0, `kink` yaw -1.0, a +0.002 rad
    turn with the depth growing 1 mm: the window is counted under `feature_switch` with its measured depth change and
    has no per-degree-of-freedom entries; the smooth windows' pooled split does not include it.
  - `test_alignment_separates_pipeline_and_servo`: the arm `qd_ref` is `cmd` ramped over 0.04 s, the measured joint
    velocity is `qd_ref` delayed 0.03 s, each series written at its sample time: `servo` delay 0.03 (+-0.01),
    `pipeline` delay larger; a window with a reanchored cycle is left out of `servo` and counted.
  - `test_build_time_recovers_a_first_order_sideslip`: lateral velocity = a 0.3 s filter of `0.05 * v * omega`:
    `pytest.approx(0.3, abs=0.05)`.
  - `test_report_writes_json_and_markdown`: the CLI on two synthetic logs writes both files; the JSON has the five
    keys per run and `pooled`.
- [ ] **Step 2: Run**; expected: FAIL.
- [ ] **Step 3: Implement** the five functions and the `report` subcommand.
- [ ] **Step 4: Run** `cpytest tools/plant/test_model_rollout.py`; expected: PASS.
- [ ] **Step 5: Commit** `E2 Task 3: rollout reports (horizons, twist bins, alignment, gradient projection, build time)`.

### Task 4: The measurement runs and CP1

**Files:**
- Create: `docs/experiments/execution-fixes/e2_measure.sh`, `docs/experiments/execution-fixes/e2.md`
- Create: `docs/experiments/execution-fixes/configs/profile_no_motion_ff.yaml` (the installed profile with
  `motion_ff: false`, made by the script the way `e1_ensembles.sh profiles` makes the grip profiles)
- Modify: `tools/plant/model_rollout.py` (subcommand `excite`), `tools/plant/test_model_rollout.py`

**Interfaces:**
- Produces: `model_rollout.py excite --out DIR`: builds `plant_replay` inputs through
  `commission_plant.steady_input`-style dictionaries and `calibrate_plant.plant_replay`, writes
  `DIR/excitation.json` (per segment: the command series, the measured twist and pose series):
  - steps from rest to each of v in (0.002, 0.005, 0.01, 0.02, 0.0375, 0.1, 0.3) m/s and omega in (0.01, 0.02,
    0.03, 0.05, 0.07, 0.1, 0.2, 0.4, 0.6) rad/s, 4 s each;
  - ramps 0 -> 0.05 m/s and 0 -> 0.1 rad/s over 20 s, and the same ramps down from moving;
  - the commissioning turns `commission_plant.TURNS`, 8 s each, entered from 3 s straight and left into 3 s straight.
- Produces: `e2_measure.sh STAGE` with stages `ratchet` (the five runs of `experiment_runs/e1/ratchet` rerun with
  `--diagnostics` into `experiment_runs/e2/measure/ratchet`, each compared to its original with `divergence.py
  pair`: State and Event frames identical apart from the three new header keys), `store_room` (B5 alone, seeds
  1-5), `gate` (gate_right_swing F2: the seeds starting left in `experiment_runs/e1/absinf` and as many starting
  right), `excite`, `arm_ff` (the commissioning arm move, `commission_plant.arm_move`, on the installed profile and
  on `profile_no_motion_ff.yaml`), `report` (`model_rollout.py report` over all of them into
  `experiment_runs/e2/measure/report`).

- [ ] **Step 1: Failing test** `test_excitation_segments_cover_the_breakaway_range` (pure: the input builder
  `excitation_inputs()` returns segments whose commanded |v| and |omega| include values below and above 0.005 m/s
  and 0.02 rad/s, and one segment per entry of `TURNS` with straight parts before and after).
- [ ] **Step 2: Implement** `excitation_inputs()` and `excite`; run the test; expected: PASS.
- [ ] **Step 3: Write** `e2_measure.sh` (the header comment in the style of `e1_ensembles.sh`; a rerun skips
  finished runs).
- [ ] **Step 4: Run** `bash docs/experiments/execution-fixes/e2_measure.sh ratchet`; expected: five reruns, each
  `pair` report "identical". If one differs, stop: that is a reproducibility finding for the user.
- [ ] **Step 5: Run** the stages `store_room`, `gate`, `excite`, `arm_ff`, `report` (about 30 runs; ask the user
  before starting).
- [ ] **Step 6: Write** e2.md "Measurement": the horizon table (sent command and measured twist as inputs: the
  command-tracking part against the kinematic part), the twist bins, the alignment delays and time constants, the
  gradient projection on the two door-cell ratchet runs (which degree of freedom carries the 1-2 mm per 0.1 s, and
  what B5's model predicted for it), the sideslip build-up time, the arm tracking error with `motion_ff` on and off
  (T8), the chatter's share of the 20 ms error. Per short-list term: its share of the one-stage error on its
  windows and the CP1 rule's verdict. Start the problems table.
- [ ] **Step 7: Commit** `E2 Task 4: measurement runs and results`.
- [ ] **Step 8: CP1.** Report to the user: the table per term with a recommendation, which ratchet candidate the
  data supports, the lateral-velocity state or not. After the ruling: amend this plan (strike terms not picked; if
  the state escalation is taken, write the spec amendment first) and record the ruling in the roadmap's decision
  log; commit `E2 CP1: ...`.

### Task 4b: The output stage follows the plan; the store_room probe (CP1 amendment)

**Spec:** Amendment 1. **Files:**
- Modify: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/controller.hpp`,
  `src/controller.cpp` (`breakaway_output` and the `ok` branch of `Controller::step`), `formulation.hpp` /
  `src/formulation.cpp` (`MpcConfig::breakaway_follows_plan`, loader), `config/wholebody_mpc.yaml` (the switch, false),
  `src/wholebody_mpc_node.cpp` (parameter and config dump), `test/test_controller.cpp`, `test/test_mpc_config.py`
- Modify (probe): `controller.hpp` / `controller.cpp` (`CycleOutput::probe_*`, diagnostics only),
  `mobile_manipulator_medium` closed loop (`Gate` fields), `mobile_manipulator_campaign/src/session.cpp`
  (`diag_nearest`), `test/test_session.cpp`; `tools/plant/model_rollout.py` (`nearest` in records, `same_run` keys)
- Create: `docs/experiments/execution-fixes/configs/b5_follow_plan.yaml` (absinf + the switch on)

**Interfaces:**
- `Control breakaway_output(const Control & clamped, const Control & planned, bool base_at_rest, const Breakaway & b,
  bool follow_plan)`: with `follow_plan` false, the frozen function (the planned argument unused); with true, a base
  component is boosted only when it has the planned component's sign and the planned component is outside the dead
  zone. A component that opposes the plan or whose plan is inside the dead zone goes out as clamped (dead zone
  applied). The old two-argument form stays as the frozen call.
- `MpcConfig::breakaway_follows_plan` (bool, YAML `breakaway_follows_plan`, default false); passed to the Controller
  through `ControllerOptions::breakaway_follows_plan`.
- `CycleOutput` (diagnostics only): `probe_link` (string), `probe_obstacle` (string), `probe_distance` (m, the true
  nearest robot-obstacle pair at the measured state, `CollisionModel::nearest`), `probe_in_stage1` and `probe_in_rows`
  (bool: a row for that link and obstacle at stage 1, at any stage), `probe_dropped` (int, `CollisionStats::
  dropped_pairs`). State header `diag_nearest`: `[link, obstacle, distance, in_stage1 0/1, in_rows 0/1, dropped]`.

- [ ] **Step 1: Failing tests** (`test_controller.cpp`):
  - `BreakawayOutput.FollowPlanNeverBoostsAgainstThePlan`: clamped +0.054, planned -0.026, at rest: frozen +0.07,
    follow-plan +0.054. Clamped -0.016, planned -0.026, at rest: -0.07 both. Planned +0.004 (inside the 0.01 dead zone),
    clamped +0.03, at rest: follow-plan +0.03 (no boost). Not at rest: identical to the frozen function. The linear
    component the same way.
  - `Controller.FollowPlanSendsNoLatchedTurn`: the SlackGate funnel with a KinematicPlant whose static friction holds
    the base below 0.07 rad/s from rest, 300 cycles with follow-plan on: no cycle where the sent |omega| is at least
    0.038 and opposes the plan's omega.
  - `test_mpc_config.py`: the installed yaml has `breakaway_follows_plan: false`.
- [ ] **Step 2: Implement** the switch and the function; the `ok` branch passes `nominal_.u[0]` as `planned`.
- [ ] **Step 3: Failing tests (probe):** `SlackGateDiagnostics.ProbeNamesTheNearestPairAndItsRows` (the funnel: the
  nearest pair is a wall, it is in the rows at stage 1, dropped is the stats' count; without diagnostics all empty);
  `Session.DiagnosticsCarryTheNearestPairProbe` (key present with 6 entries under diagnostics, absent without).
- [ ] **Step 4: Implement** the probe, records `nearest` (None when absent), `same_run` ignores `diag_nearest`.
- [ ] **Step 5: Run** the mpc, medium and campaign suites and `tools/plant`; expected PASS (goldens unchanged).
- [ ] **Step 6: The judgment runs** (`e2_measure.sh` stages with `--b5-config configs/b5_follow_plan.yaml --tag
  follow`): the ratchet runs, store_room B5 1-5, gate 8 seeds into `experiment_runs/e2/follow`. T9: latched cycles 0,
  the ratchet runs' contacts. The store_room probe on `experiment_runs/e2/measure/store_room` reruns (the probe needs
  a rerun: same configs, `same-run` against the earlier ones). Write e2.md "Output stage" and "store_room probe".
- [ ] **Step 7: Commit** `E2 Task 4b: the breakaway boost follows the plan (switch); the nearest-pair probe`.
- [ ] **Step 8: Report** to the user: T9; the probe's verdict on store_room (fix in E2 or defer); the go-ahead for the
  T10 ensemble of the switch (about 1 h 50, 4 jobs).

---

## Stage 2 and 3: the model and its parameters

### Task 5: `TrackingModel`, its loader and the generated map (amended at CP1, 2026-11-12)

**CP1 amendment.** The terms are the ones the user ruled in: the turn map, the speed falling in turns, sideslip, and
the delays (Task 8). The constant speed ratio and the hard deadband are out. The turn map replaces the planned
polynomial. The measured response is 0.10 / 0.51 / 0.85 / 0.95 of the command at abs omega 0.01 / 0.045 / 0.135 /
0.4 rad/s, which the form below fits within 0.015 (g0 0.05, g_inf 1.0, c 0.025). It keeps a slope g0 > 0 at zero,
so the linearized QP still sees a small turn act: a gain that vanishes at zero would read small turns as useless
(spec Section 3.3, smoothness).

**Files:**
- Create: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/tracking_model.hpp`,
  `ros2_ws/src/mobile_manipulator_mpc/src/tracking_model.cpp`,
  `ros2_ws/src/mobile_manipulator_mpc/test/test_tracking_model.cpp`
- Modify: `ros2_ws/src/mobile_manipulator_mpc/scripts/generate_model.py`, `include/.../model.hpp`,
  `src/model.cpp`, `src/model_step.cpp`, `CMakeLists.txt` (source, test)

**Interfaces:**
- Produces:

```cpp
struct TrackingModel
{
  // the turn map: yaw rate = g_inf omega - (g_inf - g_0) turn_scale tanh(omega / turn_scale), with
  // g_inf = 1 + turn_delta_inf and g_0 = 1 + turn_delta_zero (the slope at zero); both deltas 0: yaw rate = omega
  double turn_delta_inf = 0.0;
  double turn_delta_zero = 0.0;
  double turn_scale = 0.05;      // rad/s; only used when a turn delta is non-zero
  double speed_turn = 0.0;       // forward speed = v (1 - speed_turn sabs(yaw rate)), sabs(x) = sqrt(x^2 + 1e-4) - 0.01
  double sideslip = 0.0;         // lateral speed = sideslip x forward speed x yaw rate (s; body left positive)
  double base_delay_s = 0.0;     // Task 8
  double arm_delay_s = 0.0;      // Task 8
  bool map_identity() const;     // turn_delta_inf, turn_delta_zero, speed_turn and sideslip all exactly 0
  bool identity() const;         // map_identity() and both delays 0
  std::array<double, 5> map_parameters() const;  // turn_delta_inf, turn_delta_zero, turn_scale, speed_turn, sideslip
};
TrackingModel load_tracking_model(const std::string & yaml_path);  // throws std::runtime_error naming file and key
Step discrete_step(const State & x, const Control & u, double dt, const TrackingModel & model);
```

- The existing `discrete_step(x, u, dt)` stays and is what `map_identity()` dispatches to.
- Generated: `wholebody_step_tracking(x, u, dt, p[5]) -> (x_next, A, B)`, RK4 with the map applied to (v, omega)
  inside the base kinematics: `xdot = vf cos(yaw) - vl sin(yaw)`, `ydot = vf sin(yaw) + vl cos(yaw)`,
  `yawdot = wf`, with `wf` the turn map, `vf` the speed and `vl = sideslip vf wf`.
- Loader ranges: `turn_delta_inf` in [-0.5, 0.5]; `turn_delta_zero` in [-0.99, 0.5]; `turn_scale` in [0.001, 1];
  `speed_turn` in [0, 1]; `sideslip` in [-0.5, 0.5]; delays in [0, 0.2]. An unknown key, a non-finite value, or one
  outside its range: `std::runtime_error("<file>: <key> ...")`. A missing key keeps its default.
- `model_step` accepts 23 numbers per line (the 18 and the 5 map parameters).

- [ ] **Step 1: Failing tests** in `test_tracking_model.cpp`:
  - `Identity.DispatchesToTheUntouchedStep`: for 50 random (x, u) and dt in {0.02, 0.05, 0.1}: `discrete_step(x, u,
    dt, TrackingModel{})` equals `discrete_step(x, u, dt)` bit for bit (`x_next`, `A`, `B`, `memcmp`).
  - `Map.JacobiansMatchFiniteDifferences`: parameters (0.0, -0.95, 0.025, 0.17, 0.2), 50 random (x, u) with abs
    omega up to 0.6: `A` and `B` against central differences (1e-6) within 1e-5.
  - `Map.EachTermMovesThePredictionTheExpectedWay`, from x = 0 and dt = 0.1. Each expected value is computed in the
    test from the formula above, with the exact arc of a constant twist:
    - `turn_delta_zero = -0.95, turn_scale = 0.025` with (0, 0.045): yaw == 0.1 x (0.045 - 0.95 x 0.025 x
      tanh(1.8)) (1e-12);
    - `speed_turn = 0.17` with (0.3, 0.4): the arc length == 0.1 x 0.3 x (1 - 0.17 x sabs(0.4)) (1e-9);
    - `sideslip = 0.2` with (0.3, 0.4): the body-lateral displacement against the identity's is positive (left) and
      within 2 % of 0.2 x 0.3 x 0.4 x 0.1.
  - `Map.ZeroAndTinyCommandsAreFinite` (Review Focus 1): the full parameter set, u with v = omega = 0 and 1e-6:
    `x_next`, `A`, `B` all finite; at zero command the base does not move; d yaw_next / d omega at omega = 0 equals
    g_0 dt (0.05 x 0.1) within 1e-9 (the slope the QP sees).
  - `Map.MirroredCommandsGiveMirroredPredictions` (Review Focus 2): the full parameter set, (v, omega) = (0.3, 0.4):
    against (v, -omega) y and yaw change sign and x is equal; against (-v, omega) x and y change sign and yaw is
    equal; each within 1e-12.
  - `Loader.RejectsUnknownKeysNonFiniteAndOutOfRange` (Review Focus 5): three temporary files; each message contains
    the file name and the key. `Loader.MissingFileIsNamed`. `Loader.EmptyFileIsIdentity`.
- [ ] **Step 2: Run** `ctest_pkg mobile_manipulator_mpc test_tracking_model`; expected: FAIL (no header).
- [ ] **Step 3: Implement** the generator's new function (the existing `wholebody_step` expression untouched), the
  struct, the loader, the overload, and the CLI's optional parameters.
- [ ] **Step 4: Run** `ctest_pkg mobile_manipulator_mpc "test_tracking_model|test_model|test_golden"`; expected:
  PASS (golden unchanged: nothing uses the model yet).
- [ ] **Step 5: Commit** `E2 Task 5: TrackingModel, its loader and the generated command map (identity dispatches to the untouched step)`.

### Task 6: Identification and CP2

**Files:**
- Create: `tools/plant/identify_tracking.py`, `tools/plant/test_identify_tracking.py`
- Create (output): `ros2_ws/src/mobile_manipulator_mpc/config/tracking_model_mujoco.yaml`
- Modify: `docs/experiments/execution-fixes/e2.md`

**Interfaces:**
- Consumes: `model_step` with parameters (Task 5), `model_rollout.container_step(params=...)`, `error`, `windows`,
  `alignment`; `calibrate_plant.plant_replay`; `commission_plant.pose`.
- Produces:
  - `excitation(poses=("carry", "home", "vertical_carry", "level_extension", "fold")) -> list[dict]`: per pose a
    grid of steady twists v in (0, 0.1, 0.2, 0.3) x omega in (0, +-0.05, +-0.1, +-0.15, +-0.2, +-0.3, +-0.4, +-0.6)
    inside the envelope |v| + 1.5 |omega| <= 1.12, 6 s each; Task 4's steps and ramps; transitions straight 3 s ->
    turn 6 s -> straight 3 s for the `TURNS`; each segment tagged `fit` or `held_out` alternately.
  - `fit(segments, terms) -> dict`: least squares of the model's one-stage (0.1 s) prediction against the measured
    body motion over the `fit` segments, for the terms listed (names of `TrackingModel`'s fields); the delays from
    `alignment`. Returns the parameters and, per axis (`forward`, `lateral`, `yaw`, `q0..q5`), the held-out RMS error
    before (identity) and after.
  - `write_model(path, parameters, provenance)`: the YAML with a header comment (date, profile sha256, terms, the
    held-out errors); keys in `TrackingModel`'s order; values rounded to 6 significant digits.
  - CLI: `identify_tracking.py fit --terms a,b,c --out FILE [--profile P]`, `identify_tracking.py check --model FILE
    --logs LOG...` (the one-stage error before and after on closed-loop logs, the ratchet windows separately),
    `identify_tracking.py grip --terms ... ` (the fit on the mu 0.4 and 0.8 profiles: the parameters side by side).

- [ ] **Step 1: Failing tests** (a synthetic plant: a Python function applying a known map to the commands):
  - `test_fit_recovers_known_parameters`: synthetic segments from `turn_delta_zero = -0.9`, `turn_scale = 0.03`,
    `sideslip = 0.2`, `speed_turn = 0.15`: the fit returns each within 5 %.
  - `test_held_out_error_falls`: the `after` RMS is below the `before` on `yaw` and `lateral`, and not above it on
    `forward`.
  - `test_terms_not_listed_stay_zero`.
  - `test_write_model_is_deterministic_and_loadable`: two writes are byte-identical; the file has only
    `TrackingModel`'s keys.
  - `test_excitation_alternates_fit_and_held_out_and_stays_in_the_envelope`.
- [ ] **Step 2: Run** `cpytest tools/plant/test_identify_tracking.py`; expected: FAIL.
- [ ] **Step 3: Implement.** For each `turn_scale` on a grid (0.005 to 0.1 rad/s), the turn map is linear in g_inf
  and g_0: regress the measured yaw rate on (omega, c tanh(omega / c)). `speed_turn` and `sideslip` are then linear
  in the measured body speeds. Fit on the closed-loop runs and the excitation separately, and report the two (CP1:
  the sideslip coefficient differs fourfold between them). The held-out error is always computed through
  `container_step`.
- [ ] **Step 4: Run** the tests; expected: PASS.
- [ ] **Step 5: Run** `fit` with CP1's terms, then `check` on `experiment_runs/e2/measure` and `grip`; run `fit`
  twice and compare the files (identical).
- [ ] **Step 6: Write** e2.md "Identification": the forms and parameters; held-out error before and after per
  axis; the closed-loop check with the ratchet windows; per pose residuals; the grip table.
- [ ] **Step 7: Commit** `E2 Task 6: identification tool and the MuJoCo plant's tracking model`.
- [ ] **Step 8: CP2.** Report: forms, parameters, fit quality, pose dependence (one set or a reach-dependent
  parameter), grip sensitivity (a parameter changing sign at mu 0.4 or 0.8: the escalation to the online residual).
  After the ruling: amend the plan where a form changes; decision log; commit `E2 CP2: ...`.

### Task 7: B5 uses the tracking model

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/formulation.hpp` (`MpcConfig`),
  `src/formulation.cpp` (`load_mpc_config`; `discrete_step` at 739, 893, 918, 936), `src/controller.cpp` (172, 285,
  417, 463, 481), `src/replay.cpp` (176), `src/wholebody_mpc_node.cpp` (parameter, the config dump near 467),
  `config/wholebody_mpc.yaml`
- Modify: `ros2_ws/src/mobile_manipulator_campaign/src/plant_campaign.cpp`, `src/session.cpp` (hello),
  `tools/plant/plant_campaign.py` (`--tracking-model`)
- Test: `test/test_formulation.cpp`, `test/test_controller.cpp`, `test/test_golden.cpp`, `test/test_mpc_config.py`,
  `mobile_manipulator_campaign/test/test_session.cpp`, `tools/plant/test_plant_campaign.py`

**Interfaces:**
- Consumes: Task 5's `TrackingModel`, `load_tracking_model`, the `discrete_step` overload.
- Produces: `MpcConfig::tracking_model` (`std::string`, the YAML's `tracking_model`, default `""` = identity; a
  relative path resolves against the B5 config file's directory) and `MpcConfig::tracking` (`TrackingModel`, loaded
  by `load_mpc_config`). Every B5 call of `discrete_step` listed above passes `config.tracking`; the medium package
  and `kinematic_plant.hpp` keep the three-argument form.
- Produces: `plant_campaign --tracking-model PATH|none` (overrides the B5 config's entry; `none` = identity) and
  `plant_campaign.py --tracking-model` passing it through (repository-relative, like `--b5-config`); the hello
  frame's config gains `"tracking_model"` (the path or `""`) and `"tracking_model_sha256"` (`""` for identity).
- Produces: node parameter `tracking_model` (string, overrides the YAML's).

- [ ] **Step 1: Failing tests:**
  - `test_formulation.cpp` `TrackingModel.LinearizationUsesTheModel`: a config with `turn_delta = -0.2`: the
    dynamics rows' `B` yaw-omega entry of stage 0 is 0.8 of the identity config's (1e-9).
  - `test_controller.cpp` `TrackingModel.PlanStatesFollowTheModel`: with `sideslip = 0.05`, a solved plan in a turn:
    `plan.x[k+1] == discrete_step(plan.x[k], plan.u[k], dt, model).x_next` for every k (1e-12), and differs from
    the three-argument step by more than 1e-6 somewhere.
  - `test_golden.cpp` `TrackingModel.IdentityFileReproducesTheGoldenPlans`: the golden cases with `tracking_model`
    pointing at a file of explicit zeros: bit-identical to the goldens.
  - `test_mpc_config.py`: `tracking_model: ""` in the installed YAML; a relative path resolves against the config's
    directory; a missing file's error names it (Review Focus 5).
  - `test_session.cpp` `Session.HelloRecordsTheTrackingModel`: with a model file the hello has its path and sha256;
    without, both `""`; a session with a bad file throws before the first frame.
  - `test_plant_campaign.py`: `--tracking-model` appears in the run's command.
- [ ] **Step 2: Run** the three ctest targets and the two pytest files; expected: FAIL.
- [ ] **Step 3: Implement** the config fields, the call sites, the CLI flags, the hello entries, the node parameter;
  add `tracking_model: ""` with a comment to `wholebody_mpc.yaml`.
- [ ] **Step 4: Run** `ctest_pkg mobile_manipulator_mpc ".*"`, `ctest_pkg mobile_manipulator_medium ".*"`,
  `ctest_pkg mobile_manipulator_campaign ".*"`, `cpytest tools/plant`; expected: PASS.
- [ ] **Step 5: Equivalence (T6).** Three cells (v2_store_room_nav, v2_door_105_mission, v2_gate_right_swing_nav),
  seed 1, F2 and B5, rerun without a tracking model and compared with `divergence.py pair` to their runs in
  `experiment_runs/e1/absinf` and `experiment_runs/e1/abs`: State and Event frames identical (the hello differs by
  the two new config keys only). Record in e2.md.
- [ ] **Step 6: Commit** `E2 Task 7: B5's prediction takes the tracking model (identity default)`.

### Task 8: Delay compensation

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/tracking_model.hpp`,
  `src/tracking_model.cpp`, `include/.../controller.hpp`, `src/controller.cpp`
- Test: `test/test_tracking_model.cpp`, `test/test_controller.cpp`

**Interfaces:**
- Produces:

```cpp
struct SentCommand { double t; Control u; };  // a command and the time it was sent
// The state at actuation: x (measured at `now`) advanced through the commands not yet acted on. The base states
// roll through the base parts of the commands sent in [now - base_delay_s, now), each held until the next one (the
// last until now); the arm states likewise with arm_delay_s. Commands before the first sent one are zero.
State advance_to_actuation(const State & x, const std::vector<SentCommand> & sent, double now,
                           const TrackingModel & model);
```

- `Controller` keeps its sent commands of the last 0.25 s and, when `!config.tracking.identity()` and a delay is
  positive, solves from `advance_to_actuation(measured, sent, now, config.tracking)`. The gate's measured depth, the
  guard and `CycleOutput`'s measured quantities keep the measured state. `CycleOutput::x0_planned` (`State`) is the
  state the QP started from.

- [ ] **Step 1: Failing tests:**
  - `Advance.HandComputedCase`: x = 0, `base_delay_s = 0.04`, `arm_delay_s = 0.03`, sent at now - 0.04: u with
    v = 0.3, omega = 0, qdot1 = 0.5; at now - 0.02: v = 0.1: result x == 0.3 * 0.02 + 0.1 * 0.02 = 0.008, q1 ==
    0.5 * 0.03 = 0.015 (both commands carry qdot1 = 0.5; the first covers [now - 0.03, now - 0.02), the second the
    rest) within 1e-12.
  - `Advance.ZeroDelaysReturnTheState` (bit-identical).
  - `Advance.ShortHistoryAndNoHistory` (Review Focus 3): a delay of 0.2 s with one command 0.02 s old, and with an
    empty history: finite, and equal to the hand-computed values (0.02 s of that command; x).
  - `Advance.UsesTheCommandMap`: `turn_delta = -0.2`, omega = 0.5 for 0.04 s: yaw == 0.016.
  - `test_controller.cpp` `Delay.PlansFromTheAdvancedState`: `base_delay_s = 0.04`, a controller driven two cycles
    at 0.3 m/s: `out.x0_planned` equals `advance_to_actuation(...)` of the measured state; with zero delays
    `x0_planned` is the measured state bit for bit.
- [ ] **Step 2: Run**; expected: FAIL.
- [ ] **Step 3: Implement.** Note for the RTI split (`controller.cpp:455-470`): the prepared prediction `x_pred`
  rolls from `last_x0_`, which becomes the advanced state; keep that consistent.
- [ ] **Step 4: Run** `ctest_pkg mobile_manipulator_mpc ".*"`; expected: PASS, goldens unchanged.
- [ ] **Step 5: Commit** `E2 Task 8: delay compensation (plan from the state at actuation)`.

---

## Stage 3 and 4: check and judge

### Task 9: Closed-loop prediction, solve time, CP3

**Files:**
- Create: `docs/experiments/execution-fixes/configs/b5_tracking.yaml` (the installed `wholebody_mpc.yaml` with
  `tracking_model:` the repository-relative `tracking_model_mujoco.yaml`),
  `docs/experiments/execution-fixes/e2_ensembles.sh`
- Modify: `docs/experiments/execution-fixes/e1_ensembles.sh` (`OUT_ROOT`, default `experiment_runs/e1`),
  `tools/plant/model_rollout.py` (`report --model FILE`: the rollout with the file's map parameters),
  `tools/plant/test_model_rollout.py`, `docs/experiments/execution-fixes/e2.md`

**Interfaces:**
- Produces: `e2_ensembles.sh STAGE [CONFIG TAG]` = `OUT_ROOT=experiment_runs/e2 e1_ensembles.sh "$@"` with a header
  comment; `e1_ensembles.sh` writes variants to `${OUT_ROOT}/<TAG>`.
- Produces: `model_rollout.py report --model FILE` (the map parameters read from the YAML by a small standard-library
  parser of `key: number` lines).

- [ ] **Step 1: Failing test** `test_report_with_a_model_uses_its_parameters`: a synthetic log with a 0.79 turn
  gain; `report --model` with `turn_delta: -0.21` gives a yaw error ~0 for `cmd` inputs (the fake step honouring
  the parameters); a file with a key that is not a `TrackingModel` field: `ValueError` naming it.
- [ ] **Step 2: Implement**; run; PASS. Add `OUT_ROOT` to `e1_ensembles.sh` and write `e2_ensembles.sh`.
- [ ] **Step 3: Run** the measurement cells with the model on: `e2_measure.sh` stages `ratchet`, `store_room`,
  `gate` with `--b5-config configs/b5_tracking.yaml --tag trk` into `experiment_runs/e2/measure_trk` (about 20
  runs; the script gains an optional config and tag).
- [ ] **Step 4: T1.** `model_rollout.py report --model` on those logs against Task 4's report: the one-stage error
  per axis that got a term; on frames with depth >= 5 mm the sign agreement between the predicted and the measured
  depth change (share of windows), before and after.
- [ ] **Step 5: T7.** Median and p99 solve time from the summaries of Step 3's runs against the same cells and
  seeds in `absinf` / `abs`: within 10 %.
- [ ] **Step 6: Write** e2.md "Closed-loop prediction" and "Solve time"; commit `E2 Task 9: closed-loop prediction
  and solve time with the tracking model`.
- [ ] **Step 7: CP3.** Report T1 and T7; ask for the go-ahead for the ensembles.

### Task 10: The ensembles and the hand-off cells

**Files:**
- Modify: `docs/experiments/execution-fixes/e2.md`

- [ ] **Step 1: Run** (after the go-ahead; power, lid open): `caffeinate -i bash
  docs/experiments/execution-fixes/e2_ensembles.sh variant docs/experiments/execution-fixes/configs/b5_tracking.yaml
  trk` (F2 and B5; twins A, the chicane twins, campaign B, grip at mu 0.4 and 0.8; about 1340 runs, about 1 h 50).
  A run spanning a host sleep is rerun.
- [ ] **Step 2: T2 and T3.** `compare_variants.py` of `experiment_runs/e2/trk` against `experiment_runs/e1/absinf`
  (F2) and `experiment_runs/e1/abs` (B5): reached per cell with Fisher, contacts, clearance medians, locks by kind;
  the grip cells separately. `divergence.py` flips (reported).
- [ ] **Step 3: T4.** store_room B5 alone: reached of 20 against 0 of 20. gate_right_swing F2: runs starting left
  that reach, against the baseline's. `deep` holds: runs with at least one `deep` hold of all runs, per kind,
  against the baseline (Fisher), and the hold counts.
- [ ] **Step 4: T5**, for each hand-off cell not improved: `model_rollout.py report --model` on its failing runs
  (rerun with `--diagnostics`); in each failure window the one-stage prediction error projected on the deepest
  pair's gradient against that pair's measured depth change over the stage; then the named cause with its evidence
  (the linearization error from `diag[2]`, the medium plans' feasibility, the first diverging decision). If the
  measure cannot be computed for a cell, say so for CP4.
- [ ] **Step 5: Write** e2.md "Results" and "Hand-off cells"; every number checked against the JSON reports before
  it is written; commit `E2 Task 10: ensembles with the tracking model, the hand-off cells`.

### Task 11: Acceptance, defaults, records, CP4

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_campaign/src/plant_campaign.cpp` (the default tracking model),
  `tools/plant/plant_campaign.py`, `docs/experiments/execution-fixes/e1_ensembles.sh` (the baseline stages' guard
  text), `docs/experiments/execution-fixes/configs/b5_abs_inf.yaml` (unchanged content; a note that it now needs
  `--tracking-model none` to reproduce `absinf`)
- Create: `docs/adr/0018-tracking-model-in-b5-prediction.md`
- Modify: `docs/experiments/execution-fixes/e2.md`, `docs/roadmap.md` (step 3, decision log, comparability log:
  provisional), `docs/running-the-stack.md`
- Test: `mobile_manipulator_campaign/test/test_session.cpp`, `tools/plant/test_plant_campaign.py`

Only if CP4's ruling is adoption; otherwise the defaults stay and the documents record the result.

**Interfaces:**
- Produces: `plant_campaign` without `--tracking-model` uses the installed
  `share/mobile_manipulator_mpc/config/tracking_model_mujoco.yaml`; `--tracking-model none` is the identity. The ROS
  node's `wholebody_mpc.yaml` keeps `tracking_model: ""` (Unity: identity until E5).

- [ ] **Step 1: e2.md "Acceptance":** the table T1-T8 with each result and its evidence; the problems table
  complete.
- [ ] **Step 2: CP4.** Report to the user; ask for the adoption ruling.
- [ ] **Step 3 (adoption): failing tests** `Session.DefaultTrackingModelIsTheMujocoOne` (hello's path and a
  non-empty sha256 without the flag; `none` gives `""`) and the pytest twin; tests that pinned the identity default
  are pinned to `--tracking-model none`.
- [ ] **Step 4: Implement** the default; run the campaign ctest and `cpytest tools/plant`; PASS.
- [ ] **Step 5: T6 once more** under the new default: Task 7 Step 5's three cells with `--tracking-model none`
  reproduce `absinf` / `abs` frame by frame.
- [ ] **Step 6: Write** ADR 0018 (context, decision, the options A/B/C of the spec, consequences, revisit: E5's
  Unity set, the online residual), the roadmap entries, `running-the-stack.md`.
- [ ] **Step 7: Commit** `E2 accepted: the MuJoCo tracking model the plant runs' default; ADR 0018; docs and roadmap`.
- [ ] **Step 8:** the whole-branch review prompt (`docs/reviews/e2/final-prompt.md`), its response, then the
  finishing choice (merge and push: ask).
