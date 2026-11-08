# E1: Slack Gate and Decision Robustness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the slack gate's lock at a margin, make flip-prone B5/F2 decisions commit, and attribute the plant's
findings (c) and (d), judged on realistic-plant ensembles.

**Architecture:** A config-selected allowance rule in B5's `Controller`; decision-trace fields in the MuJoCo plant's
run logs (schema 2) plus a B5-config variant and a start nudge for twin runs; Python tools that diff twin logs (flip
rate, first diverging decision) and judge variants against the 1145-run baseline; then measured, switch-guarded
commitment fixes.

**Tech Stack:** C++17 (ROS-free libraries `mobile_manipulator_mpc`, `mobile_manipulator_medium`,
`mobile_manipulator_campaign`), gtest/ament, Python 3 standard library (tools run on the host's 3.9), pytest in the
container, MuJoCo plant.

**Spec:** `docs/superpowers/specs/2026-11-08-e1-gate-robustness-design.md`

## Global Constraints

- Branch `e1-gate-robustness`; one commit per task or fix; no commits on main.
- Every new behaviour is a switch whose default is the frozen behaviour (`slack_gate_inside: increment`, each fix
  off) until Task 10.
- C++ for runtime and library code; `tools/` in Python, standard library only, Python 3.9-compatible (no `match`).
- Builds and ctest always `-DCMAKE_BUILD_TYPE=RelWithDebInfo` (an unoptimized build once gave wrong timings).
- Ensembles: realistic mode, 20 seeds, recorded routes (`corpus_campaign_a.json`, `corpus_campaign_b.json`,
  `corpus_grip.json`), `--stall-s 45` (the runner's default), `JOBS` = performance cores (4 on this host); never
  overlap two timed ensembles or Unity Play; `caffeinate -i` around every ensemble.
- Baseline: `experiment_runs/plant-replicas` (git-ignored, schema-1 logs); new runs go to
  `experiment_runs/e1/<set>` (git-ignored).
- Twin nudge EPS = 1e-12 rad on the start yaw, after the seed's ±1 mm / ±1 mrad perturbation.
- Gate tolerance T = `slack_gate_tolerance_m` (0.005 m), or `slack_gate_arm_tolerance_m` for arm and panel rows
  when it is >= 0; `kGateInsideAllowanceM` = 1e-4 m.
- Statistics: Fisher exact one-sided, p < 0.05 counts as a rise / regression; clearance regression = median true 3D
  minimum lower by more than 0.005 m.
- After every task: a fresh reviewer (user rule); stop at CP1-CP4 for the user's ruling.
- Records as problems are solved: `docs/experiments/execution-fixes/e1.md` problems table; roadmap decision log per
  adopted switch.

Container helpers (recreate in the session scratchpad, not in the repo):
- `ct.sh PKG [REGEX]`: `docker exec mm-motion-planning-ma-robot-sim-1 bash -lc "cd /workspaces/mm-motion-planning/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-up-to $PKG --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo > /tmp/ct_build.log 2>&1 || { tail -30 /tmp/ct_build.log; echo BUILD FAILED; exit 1; }; source install/setup.bash && cd build/$PKG && ctest --output-on-failure ${RE:+-R '$RE'}"`
- `pt.sh`: `docker exec -w /workspaces/mm-motion-planning mm-motion-planning-ma-robot-sim-1 python3 -m pytest -q -p no:cacheprovider "$@"`

## Review Focus

1. **Truncated run log** (a run that crashed or was interrupted): `divergence.py pair` must report "truncated at
   cycle N" for the short side and compare up to there, never a false divergence or a traceback (Task 4 test).
2. **Mismatched pair** (two logs of different scenario, bag, seed, mode or B5 config): `pair` must refuse with the
   differing Hello field named, since a nudge is the only allowed difference (Task 4 test).
3. **Schema-1 baseline log against a schema-2 log** (A5's equivalence check): compare on the shared fields only and
   say which trace fields were skipped (Task 4 test).
4. **An edited B5 variant file under an unchanged path**: the runner's resume must re-run, not skip, runs whose
   summary's `b5_config_sha256` differs from the file's current hash (Task 3 test).
5. **A bad `slack_gate_inside` value or a B5 variant missing the key**: a bad value fails at load naming the key; a
   missing key means `increment` (Task 1 test).

---

### Task 1: The gate allowance rule

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/formulation.hpp` (`SlackGate`, around line 117)
- Modify: `ros2_ws/src/mobile_manipulator_mpc/src/formulation.cpp` (slack gate loading, around line 224)
- Modify: `ros2_ws/src/mobile_manipulator_mpc/src/controller.cpp:477-481` (`gate_allowance`)
- Modify: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/controller.hpp` (comments at 64, 152)
- Modify: `ros2_ws/src/mobile_manipulator_mpc/src/wholebody_mpc_node.cpp:271-281` (node parameter)
- Modify: `ros2_ws/src/mobile_manipulator_mpc/config/wholebody_mpc.yaml:97-113` (key + comment)
- Modify: `ros2_ws/src/mobile_manipulator_navigation/launch/navigation.launch.py` (launch argument `slack_gate_inside`, beside `slack_gate`)
- Test: `ros2_ws/src/mobile_manipulator_mpc/test/test_controller.cpp`

**Interfaces:**
- Produces: `enum class GateInsideRule { Increment, Absolute };` and `SlackGate::inside_rule = GateInsideRule::Increment`
  (formulation.hpp); `GateInsideRule parse_gate_inside_rule(const std::string & s)` (throws `std::invalid_argument`
  with "slack_gate_inside must be increment or absolute"); `std::string to_string(GateInsideRule r)` returning
  `"increment"` / `"absolute"`. YAML key `slack_gate_inside` (optional; missing = increment); node and launch
  parameter `slack_gate_inside` (string).

- [ ] **Step 1: Write the failing tests** in `test_controller.cpp` beside `SlackGate.ArmRowsEnterOnlyByTheArmTolerance`:

```cpp
TEST(SlackGate, AbsoluteRuleBoundsTheDepthNotTheStep)
{
  SlackGate g = gate();
  g.inside_rule = GateInsideRule::Absolute;
  EXPECT_DOUBLE_EQ(gate_allowance(0.0, true, g), 0.005);                      // outside: T, as before
  EXPECT_DOUBLE_EQ(gate_allowance(0.001, true, g), 0.005);                    // inside by < T: up to T deep
  EXPECT_DOUBLE_EQ(gate_allowance(0.02, true, g), 0.02 + kGateInsideAllowanceM);  // deeper than T: stay or get out
  EXPECT_DOUBLE_EQ(gate_allowance(0.001, false, g), 0.005);                   // arm rows share T without the arm switch
  g.arm_tolerance_m = 0.0001;
  EXPECT_DOUBLE_EQ(gate_allowance(0.001, false, g), 0.001 + kGateInsideAllowanceM);  // arm T 0.1 mm
  EXPECT_DOUBLE_EQ(gate_allowance(0.001, true, g), 0.005);                    // base rows keep T
  g.inside_rule = GateInsideRule::Increment;
  EXPECT_DOUBLE_EQ(gate_allowance(0.001, true, g), 0.001 + kGateInsideAllowanceM);  // the frozen rule
}

TEST(SlackGate, InsideRuleFromTheConfiguration)
{
  EXPECT_EQ(gate().inside_rule, GateInsideRule::Increment);  // the shipped yaml (default until Task 10)
  EXPECT_EQ(parse_gate_inside_rule("absolute"), GateInsideRule::Absolute);
  EXPECT_THROW(parse_gate_inside_rule("deep"), std::invalid_argument);
  // a copy of wholebody_mpc.yaml without the key loads as increment; with "deep" load_mpc_config throws naming
  // slack_gate_inside (write both copies to a temp dir)
}

TEST(SlackGate, AbsoluteRuleReleasesTheLockAtAMargin)
{
  // InsideTheMarginNeverGetsDeeper's funnel, the robot starting 1 mm inside the 0.05 m margin (inside = 0.049) and a
  // slope that makes every progressing plan ask 1-4 mm deeper. Under increment the loop holds and makes < 0.5 m
  // progress; under absolute it progresses farther than under increment and never gets closer than
  // 0.05 - 0.005 - 1e-3 (T plus the solver's 1 mm).
}
```

Fill `AbsoluteRuleReleasesTheLockAtAMargin` from `InsideTheMarginNeverGetsDeeper` (same `run_gate_loop`, `Plant`,
weights divided by 10); assertions: increment `run.holding > 0` and `plant.x(0) < 0.5`; absolute
`plant.x(0) > x_increment + 0.2` and `run.closest >= 0.044`. Pick the slope (start from 0.01) so that increment holds;
print both runs as the other funnel test does.

- [ ] **Step 2: Run to verify they fail**

Run: `RE=SlackGate ct.sh mobile_manipulator_mpc`
Expected: build fails (`GateInsideRule` undeclared).

- [ ] **Step 3: Implement** the enum, `parse_gate_inside_rule`, `to_string`, the optional YAML key (absent:
  Increment; present: parsed, error message prefixed with the file's `where` and naming `slack_gate_inside`), the node
  parameter (`declare_parameter("slack_gate_inside", to_string(gate.inside_rule))`, parsed), the launch argument, and
  in `gate_allowance`:

```cpp
const double t = !base && gate.arm_tolerance_m >= 0.0 ? gate.arm_tolerance_m : gate.tolerance_m;
if (!(violation > 0.0)) return t;
const double inside = violation + kGateInsideAllowanceM;
return gate.inside_rule == GateInsideRule::Absolute ? std::max(t, inside) : inside;
```

  Add `slack_gate_inside: increment` to `wholebody_mpc.yaml` with a comment stating both rules (spec 3.1). Update the
  `SlackGate` and `CycleOutput::gate_*` comments to name both rules.

- [ ] **Step 4: Run the package's tests**

Run: `ct.sh mobile_manipulator_mpc`
Expected: all pass, including `test_golden` (the default is the frozen rule) and the three new tests.

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/mobile_manipulator_mpc ros2_ws/src/mobile_manipulator_navigation/launch/navigation.launch.py
git commit -m "B5 slack gate: slack_gate_inside absolute bounds a pair's depth at the tolerance (off by default)"
```

---

### Task 2: Decision trace, B5 config variant and start nudge in the plant session

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_mpc/include/mobile_manipulator_mpc/controller.hpp` (`CycleOutput`)
- Modify: `ros2_ws/src/mobile_manipulator_mpc/src/controller.cpp` (set it where `dwelling` is computed, ~line 780)
- Modify: `ros2_ws/src/mobile_manipulator_medium/include/mobile_manipulator_medium/planner.hpp` (`PlanOutput`)
- Modify: `ros2_ws/src/mobile_manipulator_medium/src/planner.cpp` (set `waiting` where `waiting_` is set, ~line 574)
- Modify: `ros2_ws/src/mobile_manipulator_medium/include/mobile_manipulator_medium/closed_loop.hpp:102-106` and `src/closed_loop.cpp:166` (`Gate::dwell`)
- Modify: `ros2_ws/src/mobile_manipulator_campaign/include/mobile_manipulator_campaign/{session,closed_loop_case}.hpp`, `src/{session,closed_loop_case,plant_campaign}.cpp`
- Modify: `ros2_ws/src/mobile_manipulator_plant/include/mobile_manipulator_plant/frames.hpp:15` (`kSchemaVersion = 2`)
- Modify: `tools/plant/plant_stream.py` (`SCHEMA = 2`; `RunLog` accepts 1 and 2; the demo client requires 2)
- Test: `ros2_ws/src/mobile_manipulator_campaign/test/test_session.cpp`, `tools/plant/test_plant_stream.py`

**Interfaces:**
- Consumes: Task 1's `to_string(GateInsideRule)`.
- Produces:
  - `CycleOutput::gate_dwelling` (bool: holding with a passing plan inside the resume dwell); `ClosedLoopStepper::Gate::dwell`.
  - `PlanOutput::waiting` (bool: the published ok plan waits).
  - `ClosedLoopCase make_case(scenario, bag, medium_config = "", b5_config = "")` (empty: the installed `wholebody_mpc.yaml`).
  - `SessionConfig::b5_config` (std::string, repository- or absolute path; empty: installed) and
    `SessionConfig::start_nudge_rad` (double, default 0, added to the start yaw after the perturbation).
  - `plant_campaign` flags `--b5-config FILE`, `--start-nudge EPS`.
  - State header `"dwell"`: 0/1. Plans header adds `"seed"`, `"full_multistart"`, `"waiting"`, `"homotopy_switched"`,
    `"seeds"`: `[[name, merit, feasible 0/1, progress_m], ...]` (no timings).
  - Hello `config` adds `"b5_config"`, `"b5_config_sha256"`, `"gate_rule"`, `"start_nudge_rad"`; the summary adds the
    same four keys at top level.

- [ ] **Step 1: Write the failing tests** in `test_session.cpp`:
  - `Session.StateFramesCarryTheDwell`: extend the setup of `StateFramesCarryTheSlackGate`; every State header has
    `dwell` in {0, 1}, and `dwell == 1` only on frames whose `status` is `holding`.
  - `Session.PlansFramesCarryTheDecision`: a short F2 run; each Plans header has `seed` (equal to one of `seeds[i][0]`),
    `waiting`, `full_multistart`, `homotopy_switched`, and `seeds` rows of 4 entries.
  - `Session.StartNudgeMovesOnlyTheYaw`: same seed with nudge 0 and 1e-12: `start_state()` differs only in index 2, by
    1e-12 (EXPECT_NEAR 1e-15); the summaries' `start_nudge_rad` are 0 and 1e-12.
  - `Session.B5ConfigVariantIsLoadedAndHashed`: a temp copy of the installed `wholebody_mpc.yaml` with
    `slack_gate_inside: absolute`; the Hello's `config.gate_rule` is `absolute`, `b5_config_sha256` is the copy's
    SHA-256; without the option `gate_rule` is `increment`.
  - Keep `Session.SameSeedSameLogBytes` and `SameSeedSameLogBytesRealisticWithLidar` passing (determinism with the
    new fields).
  - In `test_plant_stream.py`: `test_run_log_reads_schema_1_and_2` (encode a Hello with `schema` 1 and 2 into a
    minimal log; both open; schema 3 raises ValueError).

- [ ] **Step 2: Run to verify they fail**

Run: `RE=Session ct.sh mobile_manipulator_campaign`
Expected: build fails or the new tests fail (missing fields / setters).

- [ ] **Step 3: Implement** the fields and flags. The nudge is applied after the perturbation in `Session::reset`
  (`start_new(2) += config_.start_nudge_rad`), so a twin keeps the seed's noise stream. `make_case` loads
  `b5_config` with `load_mpc_config` when non-empty. Merits go into the JSON as doubles (deterministic).

- [ ] **Step 4: Run the tests**

Run: `ct.sh mobile_manipulator_campaign && ct.sh mobile_manipulator_medium && pt.sh tools/plant`
Expected: all pass.

- [ ] **Step 5: Update `docs/experiments/mujoco-plant/stream-protocol.md`** (schema 2: the new State, Plans and Hello
  fields; schema-1 logs still read) and commit:

```bash
git add ros2_ws/src tools/plant docs/experiments/mujoco-plant/stream-protocol.md
git commit -m "Plant runs: decision trace (dwell, plan seed and seeds), B5 config variant, start nudge; run log schema 2"
```

---

### Task 3: Runner options for variants and twins

**Files:**
- Modify: `tools/plant/plant_campaign.py` (`runs_of`, `pending`, `run`, argument parser)
- Test: `tools/plant/test_plant_campaign.py`

**Interfaces:**
- Consumes: Task 2's `--b5-config`, `--start-nudge`, summary key `b5_config_sha256`.
- Produces: `runs_of(cells, seeds, mode, kinds=("f2","b5"), lidar=False, timeout_s=None, profile=None, tag=None,
  b5_config=None, start_nudge=None)`; a run with a nudge gets the name suffix `-n` after `-s{seed}`
  (`{cell}-{kind}-{mode}[-{tag}]-s{seed}-n`) and its record `"twin": True`; CLI `--b5-config FILE`,
  `--start-nudge EPS`; `pending` also re-runs a finished run whose summary's `b5_config_sha256` differs from the
  file's current SHA-256; `run_all(runs, out_dir, jobs, container) -> int` (the thread pool, per-run logs and the
  Ctrl-C stop, extracted from `run`; returns the number of failed runs), which `run` and Task 4 call.

- [ ] **Step 1: Write the failing tests**
  - `test_runs_take_a_b5_config_and_a_nudge`: args contain `--b5-config cfg.yaml` and `--start-nudge 1e-12`; names end
    in `-s3-n` for the nudged runs; `twin` is True only for them.
  - `test_resume_reruns_when_the_b5_config_changed` (Review Focus 4): a finished run's summary with matching args but
    an old `b5_config_sha256`; editing the file makes `pending` return it.

- [ ] **Step 2: Run to verify they fail** — `pt.sh tools/plant/test_plant_campaign.py` — Expected: 2 failures.
- [ ] **Step 3: Implement** (hash with `hashlib.sha256` of the file bytes, repository-relative path).
- [ ] **Step 4: Run** — `pt.sh tools/plant/test_plant_campaign.py` — Expected: all pass.
- [ ] **Step 5: Commit** — `git commit -m "plant_campaign.py: --b5-config variants and --start-nudge twins"`

---

### Task 4: The divergence tool

**Files:**
- Create: `tools/plant/divergence.py`
- Test: `tools/plant/test_divergence.py`

**Interfaces:**
- Consumes: `plant_stream.RunLog`, Task 3's `plant_campaign.runs_of(..., start_nudge=...)` and `run_all`.
- Produces:
  - `DECISIONS = ("status", "gate_unsafe", "dwell", "fallback", "plan_seed", "plan_homotopy", "plan_status", "plan_waiting")`
  - `decision_stream(log_path) -> (hello, list[dict])`: one dict per State frame (`t`, the decision fields, the true
    pose block, `gate`), with the latest Plans fields carried onto it; schema-1 logs lack `dwell` and the `plan_*`
    trace fields (`plan_status`, `plan_homotopy` exist in schema 1).
  - `compare(a, b) -> dict` with keys `first` (None or `{"cycle", "t", "decision", "a", "b", "context_a", "context_b"}`),
    `distance_at_first`, `distance_end` (`{"base_m", "yaw_rad", "joints_rad"}`), `truncated` (None or
    `{"side": "a"|"b", "cycle": N}`), `skipped` (trace fields absent on one side), `outcomes` (`{"a", "b"}` from the
    summaries beside the logs: reached / end_status / plant_contacts).
  - `flipped(outcome_a, outcome_b) -> bool`: reached differs, or contact (>0) against none.
  - CLI: `pair LOG_A LOG_B`, `twin --corpus FILE --cell C --kind f2|b5 --seed S --out DIR [--b5-config F]`,
    `ensemble --corpus FILE --seeds N --out DIR [--kinds f2,b5] [--b5-config F] [--tag T] [--jobs J]` writing
    `DIR/flips.json` and `DIR/flips.md` (per cell and kind: pairs, flips, flip rate; then the histogram of `first.decision`).
  - Context at the first divergence: for `plan_seed` the two seed tables; for `gate_unsafe`/`status` the gate
    `[slack, violation, unsafe]` and the allowance under the run's `gate_rule`; for `fallback` both reasons.

- [ ] **Step 1: Write the failing tests** (synthetic logs built with `plant_stream.encode` and the run-log chunk
  format; a helper `write_log(path, hello, frames)` in the test):
  - `test_identical_logs_do_not_diverge`: `first` is None, distances 0.
  - `test_first_differing_decision_is_reported`: status `ok` vs `holding` from cycle 7; `first.cycle == 7`,
    `first.decision == "status"`.
  - `test_plan_seed_divergence_has_the_seed_tables`: plan seeds differ at the 3rd Plans frame; context holds both tables.
  - `test_truncated_log_is_reported_not_diverged` (Review Focus 1): b stops after 5 of 10 cycles; `truncated ==
    {"side": "b", "cycle": 5}`, `first` None.
  - `test_mismatched_pair_is_refused` (Review Focus 2): different `seed` (or `scenario`, `bag`, `mode`,
    `config.b5_config_sha256`) raises ValueError naming the field; differing `config.start_nudge_rad` is allowed.
  - `test_schema_1_against_schema_2_skips_trace_fields` (Review Focus 3): `skipped` lists `dwell`, `plan_seed`,
    `plan_waiting`; comparison runs on the rest.
  - `test_flip_rules`: reached vs not → True; both failed, one with contacts → True; both reached → False.
  - `test_ensemble_table_and_histogram`: three fake pair results → flip rates and histogram counts.

- [ ] **Step 2: Run to verify they fail** — `pt.sh tools/plant/test_divergence.py` — Expected: ImportError.
- [ ] **Step 3: Implement** `divergence.py`. Cycles align by index of State frames (both runs step the same 20 ms
  cycles); `t` is checked equal at the first divergence and reported if not.
- [ ] **Step 4: Run** — `pt.sh tools/plant` — Expected: all pass.
- [ ] **Step 5: Live smoke** — `python3 tools/plant/divergence.py twin --corpus docs/experiments/mujoco-plant/corpus_campaign_a.json --cell v2_store_room_nav --kind b5 --seed 1 --out experiment_runs/e1/smoke` (Expected: a report, exit 0). Then the determinism check: the same seed run twice without a nudge into two directories, `cmp` of the two `run.mmlog` (Expected: identical bytes).
- [ ] **Step 6: Commit** — `git commit -m "tools/plant/divergence.py: twin runs, first diverging decision, flip rates"`

---

### Task 5: The variant judge

**Files:**
- Create: `tools/plant/compare_variants.py`
- Test: `tools/plant/test_compare_variants.py`

**Interfaces:**
- Consumes: run directories (`summary.json`, `run.mmlog`), `divergence.decision_stream`, `divergence.flipped`.
- Produces:
  - `fisher_greater(k1, n1, k2, n2) -> float`: one-sided exact p that rate 2 is lower than rate 1 (stdlib `math.comb`).
  - `regressions(baseline_dir, variant_dir, variant_tag=None) -> list[dict]` per cell and kind: reached counts, p,
    contacts, median clearance delta, `"regressed": bool` (A3's rules from Global Constraints).
  - `locks(run_dir, tolerance_m=0.005) -> list[dict]`: runs whose last 10 s (500 State frames) are all `holding` with
    `0 < gate violation < tolerance_m` (A1's lock), and separately those with violation >= tolerance_m (E2 cases).
  - CLI `regressions BASE VARIANT [--tag T] --report FILE.md`, `locks DIR --report FILE.md`; exit code 1 if any
    regression or lock.

- [ ] **Step 1: Write the failing tests**
  - `test_fisher_matches_known_values`: `fisher_greater(20, 20, 15, 20)` ≈ 0.0236 (1e-3); `fisher_greater(5, 20, 5, 20)` > 0.5.
  - `test_regression_rules`: reached 18/20 → 12/20 regressed; 18/20 → 16/20 not; contacts 0 → 1 regressed; clearance
    median −0.006 m regressed, −0.004 not.
  - `test_locks_separates_the_step_lock_from_deep_holds`: synthetic logs ending with 500 holding frames at violation
    0.001 (lock) and 0.02 (E2 case); a log that holds 400 frames then moves is neither.
- [ ] **Step 2: Run to verify they fail** — `pt.sh tools/plant/test_compare_variants.py`.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** — `pt.sh tools/plant` — Expected: all pass.
- [ ] **Step 5: Commit** — `git commit -m "tools/plant/compare_variants.py: regressions against the baseline (Fisher), gate locks"`

---

### Task 6: Baseline flip rates and frozen equivalence (CP1)

**Files:**
- Create: `docs/experiments/execution-fixes/e1.md` (sections: Baseline flip rates, Gate rule, Flip fixes, Findings c
  and d, Acceptance, Problems table)
- Create: `docs/experiments/execution-fixes/e1_ensembles.sh` (every ensemble of this plan, as `replicas.sh` is: a
  rerun skips finished runs)

- [ ] **Step 1: Frozen equivalence (A5 part 1).** Re-run store_room B5 s1, gate_right_swing F2 s4 and door_105
  mission F2 s20 at the current code into `experiment_runs/e1/equivalence`; `divergence.py pair` each against its
  baseline log in `experiment_runs/plant-replicas`. Expected: `first` None, `distance_end` 0, `skipped` lists the
  trace fields only, same outcomes. A difference stops the task (the code no longer reproduces the baseline).
- [ ] **Step 2: Baseline flip ensembles.** `caffeinate -i bash docs/experiments/execution-fixes/e1_ensembles.sh baseline`:
  `divergence.py ensemble` on `corpus_campaign_a.json` (7 cells × F2/B5 × 20 pairs = 560 runs; the replicas' campaign A runs average 28 s wall, so about 1 h at 4 jobs,
less with the 45 s stall limit cutting store_room's holds)
  and `corpus_grip.json` cells chicane (the only grip cell not in campaign A; × F2/B5 × 20 pairs) into
  `experiment_runs/e1/baseline-twins`.
- [ ] **Step 3: Record** the flip-rate table, the first-divergence histogram and three representative first
  divergences in `e1.md` "Baseline flip rates".
- [ ] **Step 4: Commit** — `git commit -m "E1: baseline flip rates on the plant (CP1)"`
- [ ] **Step 5: CP1.** Report the table and histogram to the user; wait for the ruling.

---

### Task 7: The gate rule on the ensembles (CP2)

**Files:**
- Create: `docs/experiments/execution-fixes/configs/b5_absolute.yaml` (a copy of `wholebody_mpc.yaml` with
  `slack_gate_inside: absolute`; Task 9 adds adopted switches to `b5_e1.yaml`)
- Modify: `docs/experiments/execution-fixes/e1.md`, `e1_ensembles.sh`

- [ ] **Step 1: Ensembles with `--b5-config .../b5_absolute.yaml --tag abs`:** campaign A and grip chicane twins (the
  flip rates on the same pairs), campaign B F2 (19 cells × 20), grip mu 0.4 / 0.8 (the profiles from `replicas.sh`),
  into `experiment_runs/e1/gate-abs` (about 1300 runs, roughly 2-2.5 h at 4 jobs; Task 10's set is the same size).
- [ ] **Step 2: Judge** — `compare_variants.py locks` (A1) and `regressions` against `experiment_runs/plant-replicas`
  (A3); `divergence.py` tables for the flip rate (A2's comparison against Task 6).
- [ ] **Step 3: Record** in `e1.md` "Gate rule": locks before/after, flip rates, regressions, store_room's first-trip
  holds (spec 3.1: holds at a pair outside the margin asking just over T, their count and length); add the
  problems-table row (store_room lock: cause, resolution); roadmap decision-log entry (provisional).
- [ ] **Step 4: Commit** — `git commit -m "E1: the absolute gate rule on the plant ensembles (CP2)"`
- [ ] **Step 5: CP2.** Report; the user rules whether the rule is adopted for the rest of E1.

---

### Task 8: One flip fix (repeat per flip site, CP3 after each)

Run this task once per flip site, in histogram order, until the spec's stop rule (3.4) holds. Each pass is its own
commit and review.

**Files (per site):** the C++ file owning the decision (for example `mobile_manipulator_medium/src/planner.cpp` for
the winning seed or side label, `mobile_manipulator_mpc/src/controller.cpp` for gate pass and dwell, the blocked or
stall detector's file), its config yaml (the switch, default off), its test file; `e1.md`;
`docs/experiments/execution-fixes/configs/b5_e1.yaml` or a medium variant via `--medium-config` when the switch is
the medium layer's.

- [ ] **Step 1: Replay** the most frequent first divergence: `divergence.py pair` on three of its pairs; write in
  `e1.md` what crossed (the two values, their difference, the threshold or band).
- [ ] **Step 2: Choose the mechanism** from spec 3.4's table (hysteresis band relative to the incumbent with a
  deterministic tie-break; enter/leave thresholds; minimum dwell) and name the switch (`<layer>.<decision>_<mechanism>`,
  e.g. `seed_commit_band`), default off.
- [ ] **Step 3: Write the failing unit test** reproducing the crossing: the two inputs from Step 1 that flipped,
  fed so that one side of the crossing gives decision A and the other B with the switch off; with it on both give the
  incumbent.
- [ ] **Step 4: Run to verify it fails**, implement, run the package's tests (all pass, golden included).
- [ ] **Step 5: Re-measure**: that cell's twin pairs, then all campaign A pairs, with the switch on (variant config);
  `regressions` on the cell set.
- [ ] **Step 6: Record** (`e1.md` "Flip fixes", problems-table row, decision-log entry) and commit:
  `git commit -m "E1 flip fix: <decision> commits by <mechanism> (switch <name>, off by default)"`.
- [ ] **Step 7: CP3.** Report the before/after flip rates; the user rules on adoption and whether to continue.

---

### Task 9: Findings (c) and (d) (CP3)

**Files:** `docs/experiments/execution-fixes/e1.md` "Findings c and d", `docs/experiments/mujoco-plant/replicas_notes.md`
(the attribution), and a Task 8 pass if a decision is the cause.

- [ ] **Step 1: (c)** `divergence.py twin` on `v2_gate_right_swing_nav` F2 seeds 4, 7, 11, 19 (with the adopted
  config); for each: does `plan_homotopy`'s first side flip between twins; which seed won and with what margins; the
  waiting flag and seed rotation before the gate; whether the left swing's executed pan tracks its plan (execution:
  arm lag at the gate posts).
- [ ] **Step 2: (d)** replay `v2_door_105_mission` F2 s20: per cycle the executed slack (gate slack against its
  allowance) and the stretch of followed infeasible medium plans; which came first; the base's cross-track at the
  contact. If following infeasible plans is the cause, run Task 8 with the switch
  `medium_max_infeasible_cycles` (B5 falls back to nav2 after N consecutive infeasible medium plans; N from the
  replay).
- [ ] **Step 3: Record** each cause and where it went (fixed by switch X, or handed to E2/E4 with the evidence) in
  `e1.md`, the problems table and `replicas_notes.md`; commit
  `git commit -m "E1: findings (c) and (d) attributed"`.
- [ ] **Step 4: CP3** with the attributions.

---

### Task 10: Acceptance and the default change (CP4)

**Files:**
- Modify: `ros2_ws/src/mobile_manipulator_mpc/config/wholebody_mpc.yaml` and the adopted switches' yamls (defaults on)
- Create: `docs/adr/0017-slack-gate-absolute-allowance.md` (supersedes ADR 0011's inside-margin clause)
- Modify: `docs/adr/0011-wholebody-mpc-command-ownership.md` (a "superseded in part by 0017" line), `docs/roadmap.md`
  (step 3: E1 outcome; decision log; comparability-log row, provisional until the final baseline run),
  `docs/running-the-stack.md` (`slack_gate_inside`, `--b5-config`, `--start-nudge`, `divergence.py`,
  `compare_variants.py`), `docs/experiments/execution-fixes/e1.md` "Acceptance"

- [ ] **Step 1: Acceptance ensembles** with `b5_e1.yaml` (and the medium variant if any) as a variant: campaign A
  and grip chicane twins, campaign B F2, grip mu 0.4 / 0.8, into `experiment_runs/e1/acceptance`.
- [ ] **Step 2: Judge A1-A5:** `locks` (A1: no step locks; deep holds listed as E2 cases), flip rates against Task 6
  (A2: pooled lower, no cell's rise with p < 0.05), `regressions` (A3), the findings' records (A4), suites green and
  Task 6's equivalence check still passing with every switch off (A5). Record the table in `e1.md`.
- [ ] **Step 3: If A1-A5 pass:** flip the defaults to the adopted values. `test_golden` reproduces the frozen B5, so
  it pins the frozen behaviour explicitly (after loading the config: `slack_gate.inside_rule = GateInsideRule::Increment`
  and every adopted switch off) and keeps passing unchanged; add one test that the shipped yaml now loads `absolute`.
  Write ADR 0017 and the doc updates.
- [ ] **Step 4: Full suites** — `ct.sh mobile_manipulator_mpc`, `ct.sh mobile_manipulator_medium`,
  `ct.sh mobile_manipulator_campaign`, `pt.sh tools/plant` — Expected: all pass.
- [ ] **Step 5: Commit** — `git commit -m "E1 accepted: the absolute gate rule and adopted commitment fixes on by default; ADR 0017"`
- [ ] **Step 6: CP4**, then the whole-branch review and the finishing choice (superpowers:finishing-a-development-branch).
