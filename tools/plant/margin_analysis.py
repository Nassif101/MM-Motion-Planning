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
0.02 s). The model's parameters are B5's own for that cycle (identify_tracking.b5_parameters), on the tracking model
the run planned with: each log's hello names its hash, and a file that is not that model is refused unless asked for as
a counterfactual (resolve_model; Task 2 review C1).

Standard library only (the host's Python 3.9).
"""
import argparse
import hashlib
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
MIN_RUNS = 5   # a comparison over runs needs this many on a side, else it is unavailable (spec Amendment 1)
MIN_BIN = 20   # rows a feature bin needs for its quantile to enter the first fit
IDENTITY = {"turn_delta_inf": 0.0, "turn_delta_zero": 0.0, "turn_scale": 0.05, "speed_turn": 0.0, "sideslip": 0.0,
            "turn_offset": 0.0, "turn_offset_com": 0.0}
# the motion regimes (plan Global Constraints), on the largest magnitudes of the plan's commands over the lookahead
STRAIGHT_V, STRAIGHT_W, STRAIGHT_QD = 0.15, 0.05, 0.05
TURNING_W, ARM_QD = 0.2, 0.2
HISTORY_S = 0.5  # the sent commands this long before a planning epoch are its history


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


def resolve_model(hello, model_path=None, counterfactual=False, log="the log"):
    """(prior, provenance) for one run: the tracking model's parameters its B5 planned with. The hello's
    config.tracking_model_sha256 says which: empty is the unicycle, else the hash of the model file. Without
    `model_path` the unicycle or the installed MuJoCo file is taken, whichever the hello names; `model_path` (a file, or
    "none") names one. A model whose hash is not the run's is refused (ValueError) unless `counterfactual`. A hello
    without the hash is refused: no silent substitute (Task 2 review C1). provenance: model, sha256, counterfactual."""
    config = hello.get("config") or {}
    if "tracking_model_sha256" not in config:
        raise ValueError(f"{log}: the hello has no config.tracking_model_sha256: the run's tracking model is unknown")
    used = config["tracking_model_sha256"] or ""
    if model_path is None:
        model_path = "none" if used == "" else TRACKING_MODEL
    if str(model_path) == "none":
        prior, sha, name = dict(IDENTITY), "", "none (the unicycle)"
    else:
        path = Path(model_path).resolve()
        name = str(path.relative_to(ROOT)) if ROOT in path.parents else str(model_path)  # no host path in a report
        prior, sha = it.read_model(path), hashlib.sha256(path.read_bytes()).hexdigest()
    if sha != used and not counterfactual:
        raise ValueError(f"{log}: planned with tracking model {used or 'none'}, {name} is {sha or 'none'}: not the run's "
                         "model; score another model only as a counterfactual (--counterfactual)")
    return prior, {"model": name, "sha256": sha, "counterfactual": sha != used}


def _history(recs, i0):
    """The features of the commands sent in the HISTORY_S before record i0's time, None without that much unbroken
    history in the log (the run's start, a goal change, a gap)."""
    n = int(round(HISTORY_S / CYCLE_S))
    if i0 - n < 0 or not _whole(recs, i0 - n, i0):
        return None
    return features([recs[j]["cmd"] for j in range(i0 - n + 1, i0 + 1)])


def trace_records(log_path, step=None, prior=None, executed_only=True, model_path=None, counterfactual=False,
                  dropped=None, provenance=None, start="b5"):
    """One dict per traced row with a realised state: run (the log's directory name), t (the planning epoch), pair,
    type, stage, lookahead_s, margin, d_lin, d_plan, status, regime, features (v2, w2, qd2: the largest squared base
    speed, turn rate and joint speed of the plan's controls over stages 0..stage-1), history (the same of the commands
    sent in the 0.5 s before the planning epoch; None without that much history), e_track, e_state, e_lin, e_total.

    Dropped: a trace on the log's first State frame (its planning state is not logged); a row whose realised state lies
    beyond the log's end, across a goal change or across a gap; with `executed_only`, the rows of a plan B5 did not
    execute (the frame's status is not "ok": a hold, a fallback). `dropped` (a dict) receives the counts, keyed
    (reason, stage). `step`: the model step (model_rollout.container_step by default). The tracking model is the run's
    own (resolve_model with `model_path`, `counterfactual`; `provenance`, a list, receives its record), whose gains each
    cycle's logged estimate replaces where the run carries one; `prior` gives the parameters directly instead (tests).
    ValueError for a stage that is not a whole number of 0.02 s cycles (Task 2 review C2).

    `start`: where the model's rollout begins. "b5" (the default): B5's state, the measured base and the arm's streamed
    reference, so e_track holds the arm's standing reference-to-measured offset. "true": the true state at the planning
    cycle, a matched diagnostic without that offset. "both": e_track from B5's state and e_track_true beside it."""
    if start not in ("b5", "true", "both"):
        raise ValueError(f"start: b5, true or both, not {start!r}")
    log_path = Path(log_path)
    hello, recs = mr.records(log_path)
    headers = [h for kind, h, _ in ps.RunLog(log_path).frames() if kind == ps.STATE]
    stage_s = (hello.get("config") or {}).get("row_trace_stage_s", 0.1)
    per_stage = int(round(stage_s / CYCLE_S))
    if per_stage < 1 or abs(stage_s / CYCLE_S - per_stage) > 1e-6:
        raise ValueError(f"{log_path}: B5's stage of {stage_s} s is not a whole number of cycles ({CYCLE_S} s): a row's "
                         "lookahead has no State frame at its time")
    if prior is None:
        prior, record = resolve_model(hello, model_path, counterfactual, log_path)
    else:
        record = {"model": "given parameters", "sha256": "", "counterfactual": None}
    if provenance is not None:
        provenance.append(record)

    def drop(reason, row):
        if dropped is not None:
            key = (reason, int(row[2]))
            dropped[key] = dropped.get(key, 0) + 1

    rows = []  # (i0, i1, frame index, row)
    for j, h in enumerate(headers):
        for row in h.get("rows", []):
            i0, i1 = j - 1, j - 1 + int(row[2]) * per_stage
            if j == 0:
                drop("first frame", row)
            elif executed_only and h.get("status") != "ok":
                drop("not executed", row)
            elif i1 >= len(recs):
                drop("past the end", row)
            elif not _whole(recs, i0, i1) or abs(recs[i1]["t"] - recs[i0]["t"] - int(row[2]) * stage_s) > 1e-3:
                drop("goal change or gap", row)
            else:
                rows.append((i0, i1, j, row))
    wins = sorted({(i0, i1) for i0, i1, _, _ in rows})
    if not wins:
        return []
    params = [it.b5_parameters(recs, i0, prior) for i0, _ in wins]
    own = step is None
    step = step if step is not None else mr.container_step()
    try:
        model = dict(zip(wins, it._rollout_with_params(recs, wins, params, step))) if start != "true" else None
        if start != "b5":  # the same rollout from the true state: a record without B5's state starts at its pose
            bare = [{k: v for k, v in r.items() if k != "b5"} for r in recs]
            from_true = dict(zip(wins, it._rollout_with_params(bare, wins, params, step)))
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
                    "history": _history(recs, i0),
                    "e_track": -_dot(g, _state_delta(true, (from_true if start == "true" else model)[(i0, i1)])),
                    "e_state": -_dot(g, _state_delta(recs[i0]["pose"], it._b5_start(recs[i0]))),
                    "e_lin": d_lin - d_plan,
                    "e_total": d_lin - d_true})
        if start == "both":
            out[-1]["e_track_true"] = -_dot(g, _state_delta(true, from_true[(i0, i1)]))
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


def obstacle_row(record):
    """Whether a record's row is against an obstacle of the world: not the floor, not the robot itself."""
    other = record["pair"].rsplit("|", 1)[-1]
    return other != "floor" and not other.startswith("self/")


def per_run(records, obstacle_only=True, q=0.95, by=("type", "lookahead_s"), error="e_track"):
    """{group key: {run: the q-quantile of the error over the run's rows}}; with `obstacle_only`, over its rows against
    the world's obstacles (spec Amendment 1, Statistics: rows of one run are not independent)."""
    groups = {}
    for r in records:
        if obstacle_only and not obstacle_row(r):
            continue
        groups.setdefault(tuple(r[k] for k in by), {}).setdefault(r["run"], []).append(r[error])
    return {key: {run: quantile(v, q) for run, v in runs.items()} for key, runs in groups.items()}


def over_runs(values, min_runs=MIN_RUNS):
    """A per-run table's summary over the runs: n, median, min, max; None (unavailable) below `min_runs` runs."""
    v = sorted(values.values())
    if len(v) < min_runs:
        return None
    return {"n": len(v), "median": quantile(v, 0.5), "min": v[0], "max": v[-1]}


def _feature(record, name):
    return record["history"][name[2:]] if name.startswith("h_") else record["features"][name]


def _solve(a, b):
    """Gaussian elimination with partial pivoting; None for a singular system."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for c in range(n):
        piv = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[piv][c]) < 1e-14:
            return None
        m[c], m[piv] = m[piv], m[c]
        for r in range(n):
            if r != c:
                f = m[r][c] / m[c][c]
                m[r] = [x - f * y for x, y in zip(m[r], m[c])]
    return [m[i][n] / m[i][i] for i in range(n)]


def _nnls(rows, targets):
    """Least squares of targets on [1, features] with non-negative feature weights and a free constant: the best
    feasible fit over the subsets of active features (few features: every subset is tried). (constant, weights)."""
    k = len(rows[0])
    best = None
    for mask in range(1 << k):
        cols = [i for i in range(k) if mask >> i & 1]
        x = [[1.0] + [row[i] for i in cols] for row in rows]
        n = len(cols) + 1
        ata = [[sum(r[i] * r[j] for r in x) for j in range(n)] for i in range(n)]
        sol = _solve(ata, [sum(r[i] * t for r, t in zip(x, targets)) for i in range(n)])
        if sol is None or any(w < 0 for w in sol[1:]):
            continue
        res = sum((sum(c * v for c, v in zip(sol, r)) - t) ** 2 for r, t in zip(x, targets))
        if best is None or res < best[0] - 1e-18:
            weights = [0.0] * k
            for i, w in zip(cols, sol[1:]):
                weights[i] = w
            best = (res, sol[0], weights)
    return best[1], best[2]


def first_fit(records, features=("v2", "w2", "qd2"), coverage=0.95, seed=0, error="e_track", model="linear"):
    """A first margin model, to see whether one can beat a constant (the full fit is Task 6): margin = constant +
    sum(weight x feature), weights non-negative. The runs are split 70 / 30, whole (shuffled with `seed`). On the
    training runs the rows are binned on a grid of the features' quantiles (5 a feature; 3 above three features), and
    the bins' `coverage`-quantiles of the error (bins of at least 20 rows) are fitted by least squares. On the held-out
    runs one offset is added so the margin covers `coverage` of their rows (a conformal offset), and the constant
    bound with the same held-out coverage is their quantile. A feature named h_x is the history's x.

    Returns weights, constant (with the offset), held_out_coverage, mean_margin (held-out, the margin floored at
    zero), constant_margin, saving (1 - mean_margin / constant_margin; None when the constant bound is not positive),
    n_train, n_held_out, train_runs, held_out_runs. None (unavailable) with fewer than 5 held-out runs or no bin.

    `model="binned"`: the margin is the row's bin's own training quantile instead of the linear form (a bin without 20
    training rows: the training rows' quantile); weights None. A check that a failed linear fit is not the form's
    fault."""
    import random
    if model not in ("linear", "binned"):
        raise ValueError(f"model: linear or binned, not {model!r}")
    runs = sorted({r["run"] for r in records})
    random.Random(seed).shuffle(runs)
    n_train = int(round(0.7 * len(runs)))
    train_runs, held_runs = sorted(runs[:n_train]), sorted(runs[n_train:])
    if len(held_runs) < MIN_RUNS:
        return None
    held_set = set(held_runs)
    train = [r for r in records if r["run"] not in held_set]
    held = [r for r in records if r["run"] in held_set]
    per_dim = 5 if len(features) <= 3 else 3
    edges = []
    for name in features:
        v = [_feature(r, name) for r in train]
        edges.append([quantile(v, k / per_dim) for k in range(1, per_dim)])
    bins = {}
    for r in train:
        f = [_feature(r, name) for name in features]
        key = tuple(sum(1 for e in edges[i] if f[i] > e) for i in range(len(features)))
        bins.setdefault(key, []).append((f, r[error]))
    rows, targets, level = [], [], {}
    for key, members in bins.items():
        if len(members) >= MIN_BIN:
            rows.append([sum(f[i] for f, _ in members) / len(members) for i in range(len(features))])
            targets.append(quantile([e for _, e in members], coverage))
            level[key] = targets[-1]
    if not rows or not held:
        return None
    if model == "binned":
        constant, weights = 0.0, None
        fallback = quantile([r[error] for r in train], coverage)
        predict = []
        for r in held:
            f = [_feature(r, name) for name in features]
            predict.append(level.get(tuple(sum(1 for e in edges[i] if f[i] > e) for i in range(len(features))), fallback))
    else:
        constant, weights = _nnls(rows, targets)
        predict = [constant + sum(w * _feature(r, name) for w, name in zip(weights, features)) for r in held]
    errors = [r[error] for r in held]
    offset = quantile([e - m for e, m in zip(errors, predict)], coverage)
    margins = [max(0.0, m + offset) for m in predict]
    covered = sum(1 for e, m in zip(errors, margins) if e <= m + 1e-15) / len(held)
    constant_margin = quantile(errors, covered)
    mean_margin = sum(margins) / len(margins)
    return {"weights": dict(zip(features, weights)) if weights is not None else None, "constant": constant + offset,
            "held_out_coverage": covered,
            "mean_margin": mean_margin, "constant_margin": constant_margin,
            "saving": 1.0 - mean_margin / constant_margin if constant_margin > 0 else None,
            "n_train": len(train), "n_held_out": len(held), "train_runs": train_runs, "held_out_runs": held_runs}


def history_gain(records, coverage=0.95, seed=0, obstacle_only=True):
    """Per row type and lookahead, over the records with a history: first_fit with the current features ("current")
    and with them plus the history's ("history"), on the same split by run; n; gain = 1 - the history model's
    held-out mean margin / the current model's (None when either is unavailable). Approach C is proposed only at a
    gain of at least 0.10 (plan Task 5 amendment)."""
    groups = {}
    for r in records:
        if r["history"] is not None and (not obstacle_only or obstacle_row(r)):
            groups.setdefault((r["type"], r["lookahead_s"]), []).append(r)
    out = {}
    for key, recs in sorted(groups.items()):
        cur = first_fit(recs, ("v2", "w2", "qd2"), coverage, seed)
        hist = first_fit(recs, ("v2", "w2", "qd2", "h_v2", "h_w2", "h_qd2"), coverage, seed)
        ok = cur and hist and cur["mean_margin"] > 0
        out[key] = {"n": len(recs), "current": cur, "history": hist,
                    "gain": 1.0 - hist["mean_margin"] / cur["mean_margin"] if ok else None}
    return out


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
    """Per row type: the rank correlation of e_track with each feature of the plan's window (`current`) and of the
    commands sent in the 0.5 s before the planning epoch (`history`), over the n records that have that history.

    Exploratory only (Task 2 review I1): a marginal association, pooled over lookaheads and over the rows of one trace,
    not conditioned on the current features. A strong `history` correlation can come from the current motion
    resembling the motion before it, or from a nonlinear dependence on a current feature; it does not show that the
    error has memory. The escalation to a command history is ruled on a held-out comparison of a current-feature model
    with a current-plus-history one (plan Task 5)."""
    paired = {}
    for r in records:
        if r.get("history") is not None:
            paired.setdefault(r["type"], []).append((r["e_track"], r["features"], r["history"]))
    out = {}
    for kind, rows in paired.items():
        e = [x[0] for x in rows]
        out[kind] = {"n": len(rows),
                     "current": {f: spearman(e, [x[1][f] for x in rows]) for f in FEATURES},
                     "history": {f: spearman(e, [x[2][f] for x in rows]) for f in FEATURES}}
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


def markdown(records, runs, provenance=(), dropped=None):
    lines = ["# Row-trace analysis (E3 Task 2)", "",
             f"{len(records)} traced rows with a realised state from {runs} runs. Errors in mm, positive when the pair "
             "is closer than predicted. e_track: B5's model with the sent commands against the true state, on the "
             "row's gradient (from B5's state: the arm's reference-to-measured offset is in it; e_state is that offset "
             "alone, at the planning cycle, on the same gradient, which belongs to the stage's configuration: compare "
             "it with e_track at the short lookaheads). e_lin: the row's linearized less its exact distance at the "
             "solved plan. e_total: the linearized distance less the distance at the true state, to first order "
             "(d_true = d_plan - g . (x_plan - x_true): a diagnostic, without a bound on the higher-order part at long "
             "lookaheads); it also holds B5 changing its plan. The rows are those of plans B5 executed; rows and "
             "overlapping windows are not independent samples. A mover stale by a stage's time has no row.", "",
             "## Tracking models", "",
             "Each run is scored with the model it planned with (the hello's hash); a counterfactual is another model, "
             "asked for by name.", "",
             "| Tracking model | SHA-256 | Runs | Counterfactual |", "|---|---|---|---|"]
    models = {}
    for m in provenance:
        key = (m["model"], m["sha256"], m["counterfactual"])
        models[key] = models.get(key, 0) + 1
    for (model, sha, counterfactual), n in sorted(models.items(), key=str):
        flag = "-" if counterfactual is None else ("yes" if counterfactual else "no")
        lines.append(f"| {model} | {sha[:12] or '-'} | {n} | {flag} |")
    lines += ["", "## Errors", "",
              "| Type | Lookahead (s) | Regime | n | e_track p50 | p95 | p99 | e_state p95 | e_lin p95 | e_total p95 |",
              "|---|---|---|---|---|---|---|---|---|---|"]
    for r in quantile_table(records):
        lines.append(f"| {r['type']} | {r['lookahead_s']:g} | {r['regime']} | {r['n']} | {_mm(r['e_track'][0.5])} | "
                     f"{_mm(r['e_track'][0.95])} | {_mm(r['e_track'][0.99])} | {_mm(r['e_state'][0.95])} | "
                     f"{_mm(r['e_lin'][0.95])} | {_mm(r['e_total'][0.95])} |")
    lines += ["", "## Rows left out", "",
              "Traced rows without a record, by reason and stage (the tables above hold the rest).", "",
              "| Reason | Stage | Rows |", "|---|---|---|"]
    for (reason, stage), n in sorted((dropped or {}).items()):
        lines.append(f"| {reason} | {stage} | {n} |")
    lines += ["", "## History (exploratory)", "",
              "Rank correlation of e_track with the features of the plan's window (now) and of the commands sent in the "
              f"{HISTORY_S:g} s before the planning epoch (before). An exploratory association, pooled over lookaheads "
              "and not conditioned on the current features: it does not show that the error has memory.", "",
              "| Type | n | v2 now | w2 now | qd2 now | v2 before | w2 before | qd2 before |", "|---|---|---|---|---|---|---|---|"]
    for kind, h in sorted(history_dependence(records).items()):
        lines.append(f"| {kind} | {h['n']} | " + " | ".join(_rho(h["current"][f]) for f in FEATURES) + " | "
                     + " | ".join(_rho(h["history"][f]) for f in FEATURES) + " |")
    return "\n".join(lines) + "\n"


def report(paths, out_dir, step=None, prior=None, model_path=None, counterfactual=False):
    """margin_analysis.md under out_dir from the run logs under `paths`; returns the records. Every run is resolved to
    its own tracking model before any is analysed (a mixed set with an unresolved run is refused whole)."""
    logs = logs_of(paths)
    if not logs:
        raise ValueError("no run.mmlog under " + ", ".join(str(p) for p in paths))
    if prior is None:
        for log in logs:
            resolve_model(ps.RunLog(log).header, model_path, counterfactual, log)
    own = step is None
    step = step if step is not None else mr.container_step()
    records, provenance, dropped = [], [], {}
    try:
        for log in logs:
            records += trace_records(log, step=step, prior=prior, model_path=model_path, counterfactual=counterfactual,
                                     dropped=dropped, provenance=provenance)
    finally:
        if own:
            step.close()
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "margin_analysis.md").write_text(markdown(records, len(logs), provenance, dropped))
    return records


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    r = sub.add_parser("report", help="margin_analysis.md from traced runs")
    r.add_argument("runs", nargs="+", help="run directories, ensemble directories or run.mmlog files")
    r.add_argument("--out", required=True)
    r.add_argument("--tracking-model", help="a tracking model file, or none (the unicycle); by default each run's own, "
                                            "as its hello names it (the unicycle or the installed MuJoCo model)")
    r.add_argument("--counterfactual", action="store_true",
                   help="score the runs with --tracking-model although they planned with another")
    args = parser.parse_args(argv)
    records = report(args.runs, args.out, model_path=args.tracking_model, counterfactual=args.counterfactual)
    print(f"{len(records)} traced rows -> {Path(args.out) / 'margin_analysis.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
