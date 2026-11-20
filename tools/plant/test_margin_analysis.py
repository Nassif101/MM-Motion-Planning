"""Tests of the row-trace analysis (E3 plan Task 2): synthetic traced logs with known errors."""
import pytest

import margin_analysis as ma
from test_model_rollout import DT, state_frame, unicycle, write_log

PRIOR = {"turn_delta_inf": 0.0, "turn_delta_zero": 0.0, "turn_scale": 0.05, "speed_turn": 0.0, "sideslip": 0.0,
         "turn_offset": 0.0, "turn_offset_com": 0.0}
GX = [1.0] + [0.0] * 8  # a pair whose distance grows with the base's x


def traced_log(path, n=80, v_cmd=0.2, v_true=0.2, rows_at=(1,), stages=(1,), d_lin=0.061, d_plan=0.0605, pair="3|9|beam",
               status="ok", goal_change_at=None, plan_offset=0.0, stage_s=0.1, config=None, w_cmd=0.0, gradient=None,
               v_before=None):
    """A straight drive: B5 commands v_cmd, the plant moves at v_true. Frame j in `rows_at` carries a trace made at
    record j - 1's state: one row per stage with gradient GX, the plan's states (the command's prediction plus
    plan_offset in x) and the command held over ten stages."""
    cmd = [v_cmd, w_cmd] + [0.0] * 6
    x = [0.0] * 9
    frames = [state_frame(0.0, x, [0.0] * 8)]
    poses = [list(x)]
    for j in range(1, n):
        x = unicycle([(DT, x, [v_true, w_cmd] + [0.0] * 6)])[0]
        poses.append(list(x))
        goal = 1 if goal_change_at is not None and j >= goal_change_at else 0
        sent = cmd if v_before is None or j >= v_before[0] else [v_before[1]] + cmd[1:]
        frames.append(state_frame(j * DT, x, sent, twist=[v_true, w_cmd], goal=goal))
    for j in rows_at:
        kind, h, blocks = frames[j]
        start = poses[j - 1]
        h["status"] = status
        h["rows"] = [[pair, "panel", k, 0.05, d_lin, d_plan] + (gradient or GX) for k in stages]
        h["rows_x"] = [c for k in (1, 2, 5, 10) for c in [start[0] + v_cmd * stage_s * k + plan_offset] + start[1:]]
        h["rows_u"] = cmd * 10
    header = {"schema": 2, "scenario": "v2_x", "seed": 1, "mode": "realistic",
              "config": dict({"row_trace": 0.1, "row_trace_stage_s": stage_s, "diagnostics": True}, **(config or {}))}
    return write_log(path, frames, header)


def offset_step(batch):
    """A model step that feels the tracking model: the unicycle, plus a lateral slide of turn_offset (the sixth
    appended parameter) x omega, as B5's turning-centre term does. Identity parameters: the plain unicycle."""
    out = []
    for (dt, x, u), plain in zip(batch, unicycle([(dt, x, u[:8]) for dt, x, u in batch])):
        offset = u[13] if len(u) > 13 else 0.0
        out.append([plain[0], plain[1] + offset * u[1] * dt] + plain[2:])
    return out


def model_file(path, turn_offset):
    path.write_text(f"turn_scale: 0.05\nturn_offset: {turn_offset}\n")
    import hashlib
    return hashlib.sha256(path.read_bytes()).hexdigest()


def records(path, **kwargs):
    return ma.trace_records(path, step=unicycle, prior=PRIOR, **kwargs)


def test_tracking_error_recovers_a_known_offset(tmp_path):
    # the plant runs 0.04 m/s faster than commanded: after one stage (0.1 s) the true base is 0.004 m further along x
    # than B5's model says; the pair's distance grows with x, so it is 0.004 m farther than predicted: e_track < 0
    recs = records(traced_log(tmp_path / "a" / "run.mmlog", v_true=0.24))
    assert len(recs) == 1
    r = recs[0]
    assert r["e_track"] == pytest.approx(-0.004, abs=1e-6)
    assert (r["type"], r["stage"], r["pair"], r["run"]) == ("panel", 1, "3|9|beam", "a")
    assert r["lookahead_s"] == pytest.approx(0.1)
    assert r["t"] == pytest.approx(0.0)  # the planning epoch: the traced frame's time less one cycle


def test_tracking_error_grows_with_the_lookahead(tmp_path):
    recs = records(traced_log(tmp_path / "run.mmlog", v_true=0.24, stages=(1, 2, 5, 10)))
    assert [r["stage"] for r in recs] == [1, 2, 5, 10]
    for r in recs:
        assert r["e_track"] == pytest.approx(-0.04 * r["lookahead_s"], abs=2e-6)


def test_linearization_error_is_d_lin_minus_d_plan(tmp_path):
    r = records(traced_log(tmp_path / "run.mmlog", d_lin=0.061, d_plan=0.0605))[0]
    assert r["e_lin"] == pytest.approx(0.0005, abs=1e-9)
    assert r["e_track"] == pytest.approx(0.0, abs=1e-6)


def test_total_error_adds_the_plans_departure_from_the_true_state(tmp_path):
    # the plan's stage state is 0.003 m short of where the robot ends: the pair is 0.003 m farther than d_plan, on top
    # of the linearization's 0.0005
    r = records(traced_log(tmp_path / "run.mmlog", plan_offset=-0.003))[0]
    assert r["e_total"] == pytest.approx(0.0005 - 0.003, abs=1e-6)


def test_state_error_is_the_offset_between_b5s_state_and_the_true_one(tmp_path):
    # B5 plans from the arm's streamed reference; the measured joint lags it. With a gradient on the first joint the
    # offset at the planning cycle is e_state, and e_track (from B5's state) holds it too; without motion they are equal
    import plant_stream as ps
    log = traced_log(tmp_path / "run.mmlog", v_cmd=0.0, v_true=0.0)
    states = [f for f in ps.RunLog(log).frames() if f[0] == ps.STATE]
    for kind, h, blocks in states:
        blocks[1][3] = 0.01  # B5's first joint 0.01 rad ahead of the true one, throughout
        if "rows" in h:
            h["rows"][0][6:15] = [0.0, 0.0, 0.0, 0.5] + [0.0] * 5  # 0.5 m/rad on that joint
    write_log(tmp_path / "b" / "run.mmlog", states, ps.RunLog(log).header)
    r = records(tmp_path / "b" / "run.mmlog")[0]
    assert r["e_state"] == pytest.approx(0.005, abs=1e-6)  # the true joint is behind: the pair is 5 mm closer
    assert r["e_track"] == pytest.approx(0.005, abs=1e-6)


def test_regime_boundaries():
    def window(v=0.0, w=0.0, qd=0.0):
        return [[v, w, qd, 0, 0, 0, 0, 0]] * 3
    assert ma.regime(window(v=0.15)) == "straight"
    assert ma.regime(window(v=0.16)) == "other"
    assert ma.regime(window(v=0.1, w=0.05)) == "straight"
    assert ma.regime(window(v=0.1, w=0.06)) == "other"
    assert ma.regime(window(w=0.21)) == "turning"
    assert ma.regime(window(w=-0.21)) == "turning"
    assert ma.regime(window(qd=-0.21)) == "arm"
    assert ma.regime(window(w=0.21, qd=0.21)) == "turning+arm"
    # the largest magnitude over the window decides
    assert ma.regime([[0.1, 0.0] + [0.0] * 6, [0.1, 0.3] + [0.0] * 6]) == "turning"


def test_features_are_the_windows_largest_squares(tmp_path):
    r = records(traced_log(tmp_path / "run.mmlog", v_cmd=0.2))[0]
    assert r["features"] == pytest.approx({"v2": 0.04, "w2": 0.0, "qd2": 0.0})
    assert r["regime"] == "other"


def test_rows_past_the_end_are_dropped(tmp_path):
    # 80 frames (1.58 s): a trace at frame 41 (epoch 0.8 s) has realised states for 0.1, 0.2 and 0.5 s, not for 1.0 s
    recs = records(traced_log(tmp_path / "run.mmlog", rows_at=(41,), stages=(1, 2, 5, 10)))
    assert [r["stage"] for r in recs] == [1, 2, 5]


def test_rows_across_a_goal_change_are_dropped(tmp_path):
    recs = records(traced_log(tmp_path / "run.mmlog", rows_at=(1,), stages=(1, 2, 5), goal_change_at=8))
    assert [r["stage"] for r in recs] == [1]


def test_a_pair_absent_in_later_frames_is_kept_once(tmp_path):
    # Review Focus 1: a pair traced once and gone afterwards (dropped by the budget, an obstacle gone) gives its one
    # record and nothing else: no error, no stale distance carried forward
    log = traced_log(tmp_path / "run.mmlog", rows_at=(1, 6))
    recs = records(log)
    assert len(recs) == 2
    import plant_stream as ps
    frames = list(ps.RunLog(log).frames())
    states = [f for f in frames if f[0] == ps.STATE]
    states[6][1]["rows"] = [["7|4|crate", "arm", 1, 0.05, 0.2, 0.2] + GX]
    write_log(tmp_path / "b" / "run.mmlog", states, ps.RunLog(log).header)
    recs = records(tmp_path / "b" / "run.mmlog")
    assert sorted(r["pair"] for r in recs) == ["3|9|beam", "7|4|crate"]


def test_held_plans_are_left_out_unless_asked(tmp_path):
    log = traced_log(tmp_path / "run.mmlog", status="holding")
    assert records(log) == []
    assert len(records(log, executed_only=False)) == 1


def test_first_frame_trace_without_a_start_state_is_dropped(tmp_path):
    # a trace on the log's first State frame has no previous frame: its planning state is not in the log
    recs = records(traced_log(tmp_path / "run.mmlog", rows_at=(0, 6)))
    assert len(recs) == 1 and recs[0]["t"] == pytest.approx(0.1)


def test_quantile_table_groups_and_counts(tmp_path):
    recs = []
    for i, e in enumerate([0.001, 0.002, 0.003, 0.004, 0.010]):
        recs.append({"type": "panel", "lookahead_s": 0.5, "regime": "turning", "e_track": e, "e_lin": 0.0,
                     "e_total": e, "e_state": 0.0})
    recs.append({"type": "arm", "lookahead_s": 0.5, "regime": "straight", "e_track": 0.0005, "e_lin": 0.0,
                 "e_total": 0.0005, "e_state": 0.0})
    table = ma.quantile_table(recs)
    assert [(r["type"], r["regime"], r["n"]) for r in table] == [("arm", "straight", 1), ("panel", "turning", 5)]
    panel = table[1]
    assert panel["e_track"][0.5] == pytest.approx(0.003)
    assert panel["e_track"][0.95] == pytest.approx(0.0088)  # linear between the 4th and 5th of five
    assert table[0]["e_track"][0.95] == pytest.approx(0.0005)


def test_history_dependence_sees_the_motion_before_the_plan():
    # the error follows the turn rate sent BEFORE the planning epoch, not the plan's: the rank correlation says so
    recs, prev = [], 0.0
    for i in range(40):
        w2 = ((i * 7) % 11) / 11.0
        recs.append({"run": "a", "t": 0.1 * i, "type": "panel", "stage": 5, "lookahead_s": 0.5,
                     "features": {"v2": 0.04, "w2": w2, "qd2": 0.0}, "e_track": 0.001 + 0.01 * prev,
                     "history": {"v2": 0.04, "w2": prev, "qd2": 0.0} if i else None})
        prev = w2
    h = ma.history_dependence(recs)["panel"]
    assert h["history"]["w2"] > 0.95
    assert abs(h["current"]["w2"]) < 0.5
    assert h["n"] == 39  # the first record has no history


def test_report_writes_the_tables(tmp_path):
    traced_log(tmp_path / "runs" / "cell-f2-s1" / "run.mmlog", v_true=0.24, rows_at=(1, 6, 11), stages=(1, 2))
    out = tmp_path / "out"
    ma.report([tmp_path / "runs"], out, step=unicycle, prior=PRIOR)
    text = (out / "margin_analysis.md").read_text()
    assert "| panel | 0.1 | other | 3 |" in text
    assert "first order" in text  # the e_total approximation is stated
    assert "exploratory" in text  # the history table is an association, not a test of memory (review I1)


def test_report_refuses_mixed_route_conditions(tmp_path):
    # spec 2 Task 13 review I1: generated-route and recorded-route runs are separate conditions, never pooled
    traced_log(tmp_path / "runs" / "cell-f2-s1" / "run.mmlog", v_true=0.24, rows_at=(1, 6), stages=(1,))
    traced_log(tmp_path / "runs" / "cell-f2-lattice-s1" / "run.mmlog", v_true=0.24, rows_at=(1, 6), stages=(1,),
               config={"generated_routes": True})
    with pytest.raises(ValueError, match="route conditions"):
        ma.report([tmp_path / "runs"], tmp_path / "out", step=unicycle, prior=PRIOR)


# ---- Task 2 review C1: each run is scored with the tracking model it planned with ----

GY = [0.0, 1.0] + [0.0] * 7


def turning_log(path, config):
    return traced_log(path, w_cmd=0.3, stages=(5,), gradient=GY, config=config)


def test_an_identity_run_is_scored_with_the_identity_model(tmp_path):
    # the run names no tracking model (empty hash): the unicycle, whatever the installed model file says
    log = turning_log(tmp_path / "run.mmlog", {"tracking_model": "", "tracking_model_sha256": ""})
    r = ma.trace_records(log, step=offset_step)[0]
    assert r["e_track"] == pytest.approx(0.0, abs=1e-6)


def test_a_run_is_scored_with_its_own_model_file(tmp_path):
    sha = model_file(tmp_path / "model.yaml", 0.1)
    log = turning_log(tmp_path / "run.mmlog", {"tracking_model": "/x/model.yaml", "tracking_model_sha256": sha})
    r = ma.trace_records(log, step=offset_step, model_path=tmp_path / "model.yaml")[0]
    # the model slides 0.1 x 0.3 x 0.5 s = 0.015 m in y, the true robot does not: the pair (distance grows with y) is
    # 0.015 m closer than B5 predicts
    assert r["e_track"] == pytest.approx(0.015, abs=1e-5)


def test_a_model_file_that_is_not_the_runs_is_refused(tmp_path):
    sha = model_file(tmp_path / "model.yaml", 0.1)
    log = turning_log(tmp_path / "run.mmlog", {"tracking_model": "/x/model.yaml", "tracking_model_sha256": sha})
    model_file(tmp_path / "other.yaml", 0.2)
    with pytest.raises(ValueError, match="not the run's model"):
        ma.trace_records(log, step=offset_step, model_path=tmp_path / "other.yaml")
    # the installed default is not this run's model either: no silent substitute
    with pytest.raises(ValueError, match="not the run's model"):
        ma.trace_records(log, step=offset_step)
    # asked for by name, another model is a counterfactual
    r = ma.trace_records(log, step=offset_step, model_path=tmp_path / "other.yaml", counterfactual=True)[0]
    assert r["e_track"] == pytest.approx(0.030, abs=1e-5)


def test_a_log_without_model_provenance_is_refused(tmp_path):
    log = turning_log(tmp_path / "run.mmlog", None)  # no tracking_model_sha256 in the hello
    with pytest.raises(ValueError, match="tracking_model_sha256"):
        ma.trace_records(log, step=offset_step)


def test_report_names_each_runs_model(tmp_path):
    sha = model_file(tmp_path / "model.yaml", 0.1)
    turning_log(tmp_path / "runs" / "a" / "run.mmlog", {"tracking_model": "", "tracking_model_sha256": ""})
    turning_log(tmp_path / "runs" / "b" / "run.mmlog", {"tracking_model": "", "tracking_model_sha256": ""})
    ma.report([tmp_path / "runs"], tmp_path / "out", step=offset_step)
    text = (tmp_path / "out" / "margin_analysis.md").read_text()
    assert "| none (the unicycle) | - | 2 | no |" in text
    # mixed models in one report: refused unless every run resolves to its own
    turning_log(tmp_path / "runs" / "c" / "run.mmlog", {"tracking_model": "/x/model.yaml", "tracking_model_sha256": sha})
    with pytest.raises(ValueError, match="not the run's model"):
        ma.report([tmp_path / "runs"], tmp_path / "out", step=offset_step)


# ---- Task 2 review C2: the realised state is the one at the lookahead's time ----

def test_a_stage_that_is_not_a_whole_number_of_cycles_is_refused(tmp_path):
    # 0.05 s stages are 2.5 cycles: rounding would take stage 10 at 0.4 s and call it 0.5 s
    log = traced_log(tmp_path / "run.mmlog", stage_s=0.05, stages=(10,), v_true=0.189)
    with pytest.raises(ValueError, match="whole number of cycles"):
        records(log)


def test_a_whole_cycle_stage_other_than_the_default_lands_on_its_time(tmp_path):
    # 0.04 s stages (2 cycles): stage 10 is 0.4 s ahead; 0.011 m/s slow for 0.4 s is 4.4 mm
    r = records(traced_log(tmp_path / "run.mmlog", stage_s=0.04, stages=(10,), v_true=0.189))[0]
    assert r["lookahead_s"] == pytest.approx(0.4)
    assert r["e_track"] == pytest.approx(0.0044, abs=2e-6)


# ---- Task 2 review I1: history is what was sent before the plan; attrition is counted ----

def test_history_features_are_the_commands_sent_before_the_planning_epoch(tmp_path):
    # 0.5 m/s was sent until frame 20, 0.2 after: a trace planned at record 30 (0.6 s) has 0.5 s of history reaching
    # back to 0.1 s, where the faster command was still being sent; the plan itself holds 0.2
    log = traced_log(tmp_path / "run.mmlog", rows_at=(31,), v_before=(20, 0.5))
    r = records(log)[0]
    assert r["features"]["v2"] == pytest.approx(0.04)
    assert r["history"]["v2"] == pytest.approx(0.25)
    # too early in the run for 0.5 s of history: no history, the row is kept
    early = records(traced_log(tmp_path / "b" / "run.mmlog", rows_at=(6,)))[0]
    assert early["history"] is None


def test_dropped_rows_are_counted_by_reason(tmp_path):
    dropped = {}
    log = traced_log(tmp_path / "run.mmlog", rows_at=(0, 41), stages=(1, 10))
    recs = ma.trace_records(log, step=unicycle, prior=PRIOR, dropped=dropped)
    assert [r["stage"] for r in recs] == [1]
    assert dropped == {("first frame", 1): 1, ("first frame", 10): 1, ("past the end", 10): 1}
    held = {}
    ma.trace_records(traced_log(tmp_path / "b" / "run.mmlog", status="holding"), step=unicycle, prior=PRIOR, dropped=held)
    assert held == {("not executed", 1): 1}


# ---- Task 5: the matched true-state start, per-run statistics, the first fit, the history gain ----

def offset_log(tmp_path):
    """B5's first joint 0.01 rad ahead of the true one throughout, a row with 0.5 m/rad on that joint, no motion."""
    import plant_stream as ps
    log = traced_log(tmp_path / "run.mmlog", v_cmd=0.0, v_true=0.0)
    states = [f for f in ps.RunLog(log).frames() if f[0] == ps.STATE]
    for kind, h, blocks in states:
        blocks[1][3] = 0.01
        if "rows" in h:
            h["rows"][0][6:15] = [0.0, 0.0, 0.0, 0.5] + [0.0] * 5
    return write_log(tmp_path / "b" / "run.mmlog", states, ps.RunLog(log).header)


def test_the_true_state_start_takes_the_standing_offset_out(tmp_path):
    log = offset_log(tmp_path)
    assert records(log)[0]["e_track"] == pytest.approx(0.005, abs=1e-6)                 # from B5's state: the offset
    assert records(log, start="true")[0]["e_track"] == pytest.approx(0.0, abs=1e-6)     # from the true state: none
    both = records(log, start="both")[0]
    assert (both["e_track"], both["e_track_true"]) == (pytest.approx(0.005, abs=1e-6), pytest.approx(0.0, abs=1e-6))
    with pytest.raises(ValueError):
        records(log, start="plan")


def rec(run, e, pair="12|13|brace_left", kind="panel", la=0.5, **f):
    feats = {"v2": f.get("v2", 0.0), "w2": f.get("w2", 0.0), "qd2": f.get("qd2", 0.0)}
    hist = None if f.get("no_history") else {"v2": f.get("h_v2", 0.0), "w2": f.get("h_w2", 0.0), "qd2": f.get("h_qd2", 0.0)}
    return {"run": run, "e_track": e, "pair": pair, "type": kind, "lookahead_s": la, "features": feats, "history": hist}


def test_per_run_takes_each_runs_percentile_on_the_obstacles_rows():
    records_ = ([rec("a", 0.001 * k) for k in range(101)] + [rec("b", 0.002)] * 3
                + [rec("a", 0.5, pair="12|13|floor", kind="floor"), rec("a", 0.5, pair="12|13|self/base_link", kind="self")])
    table = ma.per_run(records_, q=0.95)
    assert table[("panel", 0.5)] == {"a": pytest.approx(0.095), "b": pytest.approx(0.002)}
    assert set(table) == {("panel", 0.5)}                                # floor and self rows are not the obstacle's
    assert ("floor", 0.5) in ma.per_run(records_, obstacle_only=False)


def test_over_runs_needs_five_runs():
    assert ma.over_runs({"a": 1.0, "b": 2.0, "c": 3.0, "d": 4.0}) is None  # unavailable, neither passed nor failed
    s = ma.over_runs({"a": 1.0, "b": 2.0, "c": 3.0, "d": 4.0, "e": 10.0})
    assert (s["n"], s["median"], s["min"], s["max"]) == (5, 3.0, 1.0, 10.0)


def synthetic(n_runs=40, per_run=400, error=None, seed=1):
    import random
    rng = random.Random(seed)
    out = []
    for run in range(n_runs):
        for _ in range(per_run):
            f = {"v2": rng.uniform(0, 1), "w2": rng.uniform(0, 1), "qd2": rng.uniform(0, 1),
                 "h_v2": rng.uniform(0, 1), "h_w2": rng.uniform(0, 1), "h_qd2": rng.uniform(0, 1)}
            out.append(rec(f"r{run}", error(f, rng), **f))
    return out


def test_first_fit_beats_the_constant_on_feature_driven_errors():
    fit = ma.first_fit(synthetic(error=lambda f, rng: 0.002 + 0.02 * f["w2"] + rng.gauss(0, 0.001)))
    assert fit["saving"] > 0.3
    assert 0.92 <= fit["held_out_coverage"] <= 0.98
    assert fit["weights"]["w2"] == pytest.approx(0.02, abs=0.004)
    assert fit["weights"]["v2"] < 0.003 and fit["weights"]["qd2"] < 0.003
    assert fit["mean_margin"] < fit["constant_margin"]


def test_first_fit_saves_nothing_on_an_error_without_structure():
    fit = ma.first_fit(synthetic(error=lambda f, rng: abs(rng.gauss(0, 0.005))))
    assert abs(fit["saving"]) < 0.1


def test_first_fit_split_keeps_runs_whole():
    data = synthetic(n_runs=20, per_run=300, error=lambda f, rng: 0.01 * f["w2"])
    fit = ma.first_fit(data)
    assert not set(fit["train_runs"]) & set(fit["held_out_runs"])
    assert len(fit["train_runs"]) == 14 and len(fit["held_out_runs"]) == 6
    assert fit["n_train"] == 14 * 300 and fit["n_held_out"] == 6 * 300
    assert ma.first_fit(data)["held_out_runs"] == fit["held_out_runs"]          # the same seed, the same split
    assert ma.first_fit(data, seed=1)["held_out_runs"] != fit["held_out_runs"]


def test_first_fit_is_unavailable_with_fewer_than_five_held_out_runs():
    assert ma.first_fit(synthetic(n_runs=10, per_run=50, error=lambda f, rng: 0.01 * f["w2"])) is None


def test_history_gain_is_nil_for_a_memoryless_error():
    # the review's negative control: the error depends on the current motion alone; the history is correlated with it
    def control(f, rng):
        f["h_v2"] = min(1.0, max(0.0, f["v2"] + rng.gauss(0, 0.1)))
        return 0.01 * (f["v2"] - 0.5) ** 2 + rng.gauss(0, 0.0003)
    gain = ma.history_gain(synthetic(error=control))[("panel", 0.5)]
    assert gain["gain"] < 0.05


def test_history_gain_sees_an_error_driven_by_the_history():
    gain = ma.history_gain(synthetic(error=lambda f, rng: 0.002 + 0.02 * f["h_w2"] + rng.gauss(0, 0.001)))[("panel", 0.5)]
    assert gain["gain"] > 0.3
    assert gain["history"]["saving"] > gain["current"]["saving"]


def test_history_gain_uses_only_the_records_with_a_history():
    with_gap = synthetic(error=lambda f, rng: 0.02 * f["h_w2"]) + [rec("r0", 9.0, no_history=True)] * 50
    assert ma.history_gain(with_gap)[("panel", 0.5)]["n"] == 40 * 400


def test_the_binned_first_fit_follows_a_structure_the_linear_one_cannot():
    # an error largest at both ends of w2: no non-negative slope fits it; the bins' own quantiles do
    data = synthetic(error=lambda f, rng: 0.02 * abs(f["w2"] - 0.5) + rng.gauss(0, 0.0005))
    linear, binned = ma.first_fit(data), ma.first_fit(data, model="binned")
    assert binned["saving"] > 0.2 and linear["saving"] < 0.1   # bins 0.2 wide in w2: about 0.29 against about 0
    assert binned["weights"] is None and 0.92 <= binned["held_out_coverage"] <= 0.98
    assert binned["held_out_runs"] == linear["held_out_runs"]
    with pytest.raises(ValueError):
        ma.first_fit(data, model="spline")
