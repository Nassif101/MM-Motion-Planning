"""Tests of the row-trace analysis (E3 plan Task 2): synthetic traced logs with known errors."""
import pytest

import margin_analysis as ma
from test_model_rollout import DT, state_frame, unicycle, write_log

PRIOR = {"turn_delta_inf": 0.0, "turn_delta_zero": 0.0, "turn_scale": 0.05, "speed_turn": 0.0, "sideslip": 0.0,
         "turn_offset": 0.0, "turn_offset_com": 0.0}
GX = [1.0] + [0.0] * 8  # a pair whose distance grows with the base's x


def traced_log(path, n=80, v_cmd=0.2, v_true=0.2, rows_at=(1,), stages=(1,), d_lin=0.061, d_plan=0.0605, pair="3|9|beam",
               status="ok", goal_change_at=None, plan_offset=0.0):
    """A straight drive: B5 commands v_cmd, the plant moves at v_true. Frame j in `rows_at` carries a trace made at
    record j - 1's state: one row per stage with gradient GX, the plan's states (the command's prediction plus
    plan_offset in x) and the command held over ten stages."""
    cmd = [v_cmd] + [0.0] * 7
    x = [0.0] * 9
    frames = [state_frame(0.0, x, [0.0] * 8)]
    poses = [list(x)]
    for j in range(1, n):
        x = unicycle([(DT, x, [v_true] + [0.0] * 7)])[0]
        poses.append(list(x))
        goal = 1 if goal_change_at is not None and j >= goal_change_at else 0
        frames.append(state_frame(j * DT, x, cmd, twist=[v_true, 0.0], goal=goal))
    for j in rows_at:
        kind, h, blocks = frames[j]
        start = poses[j - 1]
        h["status"] = status
        h["rows"] = [[pair, "panel", k, 0.05, d_lin, d_plan] + GX for k in stages]
        h["rows_x"] = [c for k in (1, 2, 5, 10) for c in [start[0] + v_cmd * 0.1 * k + plan_offset] + start[1:]]
        h["rows_u"] = cmd * 10
    header = {"schema": 2, "scenario": "v2_x", "seed": 1, "mode": "realistic",
              "config": {"row_trace": 0.1, "row_trace_stage_s": 0.1, "diagnostics": True}}
    return write_log(path, frames, header)


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


def test_history_dependence_sees_the_previous_windows_motion():
    # the error follows the PREVIOUS trace's turn rate, not the current one: the rank correlation says so
    recs, prev = [], 0.0
    for i in range(40):
        w2 = ((i * 7) % 11) / 11.0
        recs.append({"run": "a", "t": 0.1 * i, "type": "panel", "stage": 5, "lookahead_s": 0.5,
                     "features": {"v2": 0.04, "w2": w2, "qd2": 0.0}, "e_track": 0.001 + 0.01 * prev})
        prev = w2
    h = ma.history_dependence(recs)["panel"]
    assert h["previous"]["w2"] > 0.95
    assert abs(h["current"]["w2"]) < 0.5
    assert h["n"] == 39  # the run's first trace has no previous one


def test_report_writes_the_tables(tmp_path):
    traced_log(tmp_path / "runs" / "cell-f2-s1" / "run.mmlog", v_true=0.24, rows_at=(1, 6, 11), stages=(1, 2))
    out = tmp_path / "out"
    ma.report([tmp_path / "runs"], out, step=unicycle, prior=PRIOR)
    text = (out / "margin_analysis.md").read_text()
    assert "| panel | 0.1 | other | 3 |" in text
    assert "first order" in text  # the e_total approximation is stated
