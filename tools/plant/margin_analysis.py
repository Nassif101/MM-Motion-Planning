"""B5's tracking and linearization error per traced collision row (E3 plan Task 2; spec
docs/superpowers/specs/2026-11-15-e3-learned-margin-design.md Section 3.4).

A run recorded with `plant_campaign --row-trace` carries, once per 0.1 s, the rows of B5's solved plan at stages 1, 2,
5 and 10 (`rows`), the plan's states there (`rows_x`) and its controls at stages 0..9 (`rows_u`); stream-protocol.md.
Per traced row this tool gives three errors in metres, positive when the pair ends up closer than predicted:

- e_track: B5's model rolled from B5's state at the planning cycle with the commands that were then sent, against the
  true state one lookahead later, projected on the row's gradient: -g . (x_true - x_model). B5 changing its plan at a
  later cycle is not in it; the arm's reference-to-measured offset at the start is (B5 plans from the streamed
  reference: identify_tracking._b5_start, E2 Task 10c review I1).
- e_state: the part of e_track present at the planning cycle: -g . (x_true - x_B5) there (the arm's reference
  against its measured joints; zero for the base, which B5 measures).
- e_lin: the row's linearized distance less the pair's exact distance at the solved plan's stage state.
- e_total: the row's linearized distance less the pair's distance at the true state, to first order:
  d_true = d_plan - g . (x_plan - x_true).

Time convention (model_rollout's): a frame at t carries what B5 evaluated at t - 0.02 s, at the previous frame's
state. A trace on frame j was planned at record j - 1; its stage k lies k stages later, at record j - 1 + k x (stage /
0.02 s). The model's parameters are B5's own for that cycle (identify_tracking.b5_parameters).

Standard library only (the host's Python 3.9).
"""
import argparse
import math
import sys
from pathlib import Path

import identify_tracking as it
import model_rollout as mr
import plant_stream as ps

ROOT = Path(__file__).resolve().parents[2]
TRACKING_MODEL = ROOT / "ros2_ws/src/mobile_manipulator_mpc/config/tracking_model_mujoco.yaml"
CYCLE_S = 0.02
TRACE_STAGES = (1, 2, 5, 10)
FEATURES = ("v2", "w2", "qd2")
ERRORS = ("e_track", "e_state", "e_lin", "e_total")
IDENTITY = {"turn_delta_inf": 0.0, "turn_delta_zero": 0.0, "turn_scale": 0.05, "speed_turn": 0.0, "sideslip": 0.0,
            "turn_offset": 0.0, "turn_offset_com": 0.0}
# the motion regimes (plan Global Constraints), on the largest magnitudes of the plan's commands over the lookahead
STRAIGHT_V, STRAIGHT_W, STRAIGHT_QD = 0.15, 0.05, 0.05
TURNING_W, ARM_QD = 0.2, 0.2


def _magnitudes(u_window):
    """The largest |v|, |omega| and |joint speed| over a window of controls (8 each)."""
    return (max(abs(u[0]) for u in u_window), max(abs(u[1]) for u in u_window),
            max(abs(q) for u in u_window for q in u[2:8]))


def regime(u_window):
    """straight | turning | arm | turning+arm | other, from a window of the plan's controls."""
    v, w, qd = _magnitudes(u_window)
    turning, arm = w > TURNING_W, qd > ARM_QD
    if turning and arm:
        return "turning+arm"
    if turning:
        return "turning"
    if arm:
        return "arm"
    if v <= STRAIGHT_V + 1e-12 and w <= STRAIGHT_W + 1e-12 and qd <= STRAIGHT_QD + 1e-12:
        return "straight"
    return "other"


def features(u_window):
    v, w, qd = _magnitudes(u_window)
    return {"v2": v * v, "w2": w * w, "qd2": qd * qd}


def _state_delta(a, b):
    """a - b for two states (x, y, yaw, six joints), the yaw wrapped."""
    d = [a[i] - b[i] for i in range(9)]
    d[2] = math.remainder(d[2], 2 * math.pi)
    return d


def _dot(g, d):
    return sum(g[i] * d[i] for i in range(9))


def _whole(recs, i0, i1):
    """The records i0..i1 belong to one goal and follow each other without a gap (model_rollout.windows' rule)."""
    return all(recs[j]["goal"] == recs[i0]["goal"] and 0.0 < recs[j]["t"] - recs[j - 1]["t"] <= mr.MAX_GAP_S
               for j in range(i0 + 1, i1 + 1))


def trace_records(log_path, step=None, prior=None, executed_only=True):
    """One dict per traced row with a realised state: run (the log's directory name), t (the planning epoch), pair,
    type, stage, lookahead_s, margin, d_lin, d_plan, status, regime, features (v2, w2, qd2: the largest squared base
    speed, turn rate and joint speed of the plan's controls over stages 0..stage-1), e_track, e_state, e_lin, e_total.

    Dropped: a trace on the log's first State frame (its planning state is not logged); a row whose realised state lies
    beyond the log's end, across a goal change or across a gap; with `executed_only`, the rows of a plan B5 did not
    execute (the frame's status is not "ok": a hold, a fallback). `step`: the model step (model_rollout.container_step
    by default); `prior`: the tracking model's parameters (the installed MuJoCo file by default), which each cycle's
    logged estimate replaces where the run carries one."""
    log_path = Path(log_path)
    hello, recs = mr.records(log_path)
    headers = [h for kind, h, _ in ps.RunLog(log_path).frames() if kind == ps.STATE]
    stage_s = (hello.get("config") or {}).get("row_trace_stage_s", 0.1)
    per_stage = int(round(stage_s / CYCLE_S))
    prior = prior if prior is not None else it.read_model(TRACKING_MODEL)
    rows = []  # (i0, i1, frame index, row)
    for j, h in enumerate(headers):
        if "rows" not in h or j == 0:
            continue
        if executed_only and h.get("status") != "ok":
            continue
        for row in h["rows"]:
            i0, i1 = j - 1, j - 1 + int(row[2]) * per_stage
            if i1 < len(recs) and _whole(recs, i0, i1):
                rows.append((i0, i1, j, row))
    wins = sorted({(i0, i1) for i0, i1, _, _ in rows})
    if not wins:
        return []
    params = [it.b5_parameters(recs, i0, prior) for i0, _ in wins]
    own = step is None
    step = step if step is not None else mr.container_step()
    try:
        model = dict(zip(wins, it._rollout_with_params(recs, wins, params, step)))
    finally:
        if own:
            step.close()
    out = []
    for i0, i1, j, row in rows:
        h = headers[j]
        pair, kind, stage, margin, d_lin, d_plan = row[:6]
        stage = int(stage)
        g = row[6:15]
        true = recs[i1]["pose"]
        plan = h["rows_x"][9 * TRACE_STAGES.index(stage):9 * TRACE_STAGES.index(stage) + 9]
        window = [h["rows_u"][8 * k:8 * k + 8] for k in range(stage)]
        d_true = d_plan - _dot(g, _state_delta(plan, true))
        out.append({"run": log_path.parent.name, "t": recs[i0]["t"], "pair": pair, "type": kind, "stage": stage,
                    "lookahead_s": stage * stage_s, "margin": margin, "d_lin": d_lin, "d_plan": d_plan,
                    "status": h.get("status"), "regime": regime(window), "features": features(window),
                    "e_track": -_dot(g, _state_delta(true, model[(i0, i1)])),
                    "e_state": -_dot(g, _state_delta(recs[i0]["pose"], it._b5_start(recs[i0]))),
                    "e_lin": d_lin - d_plan,
                    "e_total": d_lin - d_true})
    return out


def quantile(values, p):
    """The p-quantile by linear interpolation between order statistics; None for no values."""
    v = sorted(values)
    if not v:
        return None
    at = p * (len(v) - 1)
    lo = int(math.floor(at))
    hi = min(lo + 1, len(v) - 1)
    return v[lo] + (v[hi] - v[lo]) * (at - lo)


def quantile_table(records, by=("type", "lookahead_s", "regime"), q=(0.5, 0.95, 0.99)):
    """Per group (sorted by its key): the key's fields, n, and per error {p: quantile}."""
    groups = {}
    for r in records:
        groups.setdefault(tuple(r[k] for k in by), []).append(r)
    table = []
    for key in sorted(groups):
        row = dict(zip(by, key))
        row["n"] = len(groups[key])
        for e in ERRORS:
            row[e] = {p: quantile([r[e] for r in groups[key]], p) for p in q}
        table.append(row)
    return table


def _ranks(values):
    order = sorted(range(len(values)), key=lambda i: values[i])
    ranks = [0.0] * len(values)
    i = 0
    while i < len(order):
        j = i
        while j + 1 < len(order) and values[order[j + 1]] == values[order[i]]:
            j += 1
        for k in range(i, j + 1):
            ranks[order[k]] = 0.5 * (i + j) + 1.0
        i = j + 1
    return ranks


def spearman(a, b):
    """The rank correlation of two series; None when either is constant or shorter than three."""
    if len(a) < 3:
        return None
    ra, rb = _ranks(a), _ranks(b)
    ma_, mb = sum(ra) / len(ra), sum(rb) / len(rb)
    va, vb = sum((x - ma_) ** 2 for x in ra), sum((x - mb) ** 2 for x in rb)
    if va == 0.0 or vb == 0.0:
        return None
    return sum((x - ma_) * (y - mb) for x, y in zip(ra, rb)) / math.sqrt(va * vb)


def history_dependence(records):
    """Per row type: the rank correlation of e_track with each feature of its own window (`current`) and of the
    previous trace's window of the same run and stage (`previous`), over the n records that have a previous trace.
    A `previous` correlation well above `current`'s says the error follows the motion before the plan, which a margin
    on the current command cannot see (the spec's escalation to a command history). Where the motion changes slowly
    the two windows resemble each other and the two correlations agree: that is not evidence of history."""
    windows = {}  # (run, stage) -> {t: features}: one window per trace (its rows share the plan's commands)
    for r in records:
        windows.setdefault((r["run"], r["stage"]), {})[r["t"]] = r["features"]
    before = {}  # (run, stage, t) -> the previous trace's features
    for key, by_time in windows.items():
        times = sorted(by_time)
        for earlier, later in zip(times, times[1:]):
            before[key + (later,)] = by_time[earlier]
    paired = {}
    for r in records:
        previous = before.get((r["run"], r["stage"], r["t"]))
        if previous is not None:
            paired.setdefault(r["type"], []).append((r["e_track"], r["features"], previous))
    out = {}
    for kind, rows in paired.items():
        e = [x[0] for x in rows]
        out[kind] = {"n": len(rows),
                     "current": {f: spearman(e, [x[1][f] for x in rows]) for f in FEATURES},
                     "previous": {f: spearman(e, [x[2][f] for x in rows]) for f in FEATURES}}
    return out


def logs_of(paths):
    """The run logs under the given run directories, ensemble directories or log files, sorted."""
    out = []
    for p in map(Path, paths):
        out += [p] if p.is_file() else sorted(p.rglob("run.mmlog"))
    return out


def _mm(v):
    return "-" if v is None else f"{1e3 * v:.1f}"


def _rho(v):
    return "-" if v is None else f"{v:+.2f}"


def markdown(records, runs):
    lines = ["# Row-trace analysis (E3 Task 2)", "",
             f"{len(records)} traced rows with a realised state from {runs} runs. Errors in mm, positive when the pair "
             "is closer than predicted. e_track: B5's model with the sent commands against the true state, on the "
             "row's gradient (from B5's state: the arm's reference-to-measured offset is in it; e_state is that offset "
             "alone, at the planning cycle, on the same gradient, which belongs to the stage's configuration: compare "
             "it with e_track at the short lookaheads). e_lin: the row's "
             "linearized less its exact distance at the solved plan. e_total: the linearized distance less the "
             "distance at the true state, to first order (d_true = d_plan - g . (x_plan - x_true)); it also holds B5 "
             "changing its plan. Rows of plans B5 did not execute are left out. A mover stale by a stage's time has no "
             "row: the counts show what was traced, not every pair that was near.", "",
             "| Type | Lookahead (s) | Regime | n | e_track p50 | p95 | p99 | e_state p95 | e_lin p95 | e_total p95 |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for r in quantile_table(records):
        lines.append(f"| {r['type']} | {r['lookahead_s']:g} | {r['regime']} | {r['n']} | {_mm(r['e_track'][0.5])} | "
                     f"{_mm(r['e_track'][0.95])} | {_mm(r['e_track'][0.99])} | {_mm(r['e_state'][0.95])} | "
                     f"{_mm(r['e_lin'][0.95])} | "
                     f"{_mm(r['e_total'][0.95])} |")
    lines += ["", "## History", "",
              "Rank correlation of e_track with the features of its own window and of the previous trace's window "
              "(same run and stage).", "",
              "| Type | n | v2 now | w2 now | qd2 now | v2 before | w2 before | qd2 before |", "|---|---|---|---|---|---|---|---|"]
    for kind, h in sorted(history_dependence(records).items()):
        lines.append(f"| {kind} | {h['n']} | " + " | ".join(_rho(h["current"][f]) for f in FEATURES) + " | "
                     + " | ".join(_rho(h["previous"][f]) for f in FEATURES) + " |")
    return "\n".join(lines) + "\n"


def report(paths, out_dir, step=None, prior=None):
    """margin_analysis.md under out_dir from the run logs under `paths`; returns the records."""
    logs = logs_of(paths)
    if not logs:
        raise ValueError("no run.mmlog under " + ", ".join(str(p) for p in paths))
    own = step is None
    step = step if step is not None else mr.container_step()
    records = []
    try:
        for log in logs:
            records += trace_records(log, step=step, prior=prior)
    finally:
        if own:
            step.close()
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "margin_analysis.md").write_text(markdown(records, len(logs)))
    return records


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    r = sub.add_parser("report", help="margin_analysis.md from traced runs")
    r.add_argument("runs", nargs="+", help="run directories, ensemble directories or run.mmlog files")
    r.add_argument("--out", required=True)
    r.add_argument("--tracking-model", default=str(TRACKING_MODEL),
                   help="the tracking model the runs planned with (a file, or none: the unicycle)")
    args = parser.parse_args(argv)
    prior = dict(IDENTITY) if args.tracking_model == "none" else it.read_model(args.tracking_model)
    records = report(args.runs, args.out, prior=prior)
    print(f"{len(records)} traced rows -> {Path(args.out) / 'margin_analysis.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
