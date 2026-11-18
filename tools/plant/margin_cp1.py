"""CP1's tables from the traced legs of the fixed-margin sweep (E3 plan Task 5; spec Amendment 1, Statistics).

Two steps, because a traced leg is 320 runs and millions of rows:

    python3 tools/plant/margin_cp1.py gather experiment_runs/e3/sweep --tag m050 --out experiment_runs/e3/cp1_m050.pkl
    python3 tools/plant/margin_cp1.py report experiment_runs/e3/cp1_m050.pkl --out docs/.../e3_cp1_m050.md

gather  runs margin_analysis.trace_records run by run (start="both": the error from B5's state and, matched, from the
        true state), keeps the rows against the world's obstacles as compact tuples, and each run's outcome.
report  every number per run first (a run's 95th percentile over its rows; runs with fewer than MIN_ROWS rows in a
        group do not count), then over runs (median and range; fewer than 5 runs: unavailable). Runs that reached the
        goal (the normal passages) and runs that did not are reported apart.

Standard library only (the host's Python 3.9).
"""
import argparse
import json
import pickle
import re
import sys
from multiprocessing import Pool
from pathlib import Path

import margin_analysis as ma

FIELDS = ("type", "lookahead_s", "regime", "v2", "w2", "qd2", "h_v2", "h_w2", "h_qd2", "e_track", "e_track_true",
          "e_state", "e_lin", "e_total")
CELL = re.compile(r"^e3_(?P<geometry>beam|gap)_(?P<motion>straight|turn)_(?P<step>\d+)-")
MIN_ROWS = 20          # rows a run needs in a group for its percentile to count
LOOKAHEADS = (0.1, 0.2, 0.5, 1.0)
TYPES = ("arm", "panel")
SIZE_M = 0.005         # CP1 Size: at least 5 mm
SAVING = 0.30          # CP1 Predictability: at least 30 % below the constant bound
COVERAGE_BAND = 0.03   # ... at a held-out coverage within 3 points of 95 %
HISTORY_GAIN = 0.10    # approach C is proposed only at this gain


def compact(record):
    """A trace record as a tuple in FIELDS order (history features None without a history)."""
    f, h = record["features"], record["history"] or {}
    return (record["type"], round(record["lookahead_s"], 6), record["regime"], f["v2"], f["w2"], f["qd2"],
            h.get("v2"), h.get("w2"), h.get("qd2"), record["e_track"], record.get("e_track_true"), record["e_state"],
            record["e_lin"], record["e_total"])


def expand(run, row):
    """The tuple back as the dict margin_analysis's functions take (pair: a placeholder obstacle)."""
    d = dict(zip(FIELDS, row))
    d.update(run=run, pair="obstacle",
             features={"v2": d["v2"], "w2": d["w2"], "qd2": d["qd2"]},
             history=None if d["h_v2"] is None else {"v2": d["h_v2"], "w2": d["h_w2"], "qd2": d["h_qd2"]})
    return d


def extract(run_dir):
    """(run name, {reached, geometry, motion}, obstacle rows as tuples, dropped counts) of one run directory."""
    run_dir = Path(run_dir)
    summary = json.loads((run_dir / "summary.json").read_text())
    m = CELL.match(run_dir.name)
    meta = {"reached": bool(summary.get("reached")), "geometry": m.group("geometry") if m else None,
            "motion": m.group("motion") if m else None}
    dropped = {}
    recs = ma.trace_records(run_dir / "run.mmlog", start="both", dropped=dropped)
    rows = [compact(r) for r in recs if ma.obstacle_row(r)]
    return run_dir.name, meta, rows, {f"{reason}|{stage}": n for (reason, stage), n in dropped.items()}


def gather(directory, tag, out, jobs=3):
    dirs = sorted(p for p in Path(directory).iterdir() if p.is_dir() and re.search(rf"-{tag}-s\d+$", p.name))
    with Pool(jobs) as pool:
        results = pool.map(extract, dirs, chunksize=2)
    data = {"tag": tag, "runs": {name: meta for name, meta, _, _ in results},
            "rows": {name: rows for name, _, rows, _ in results}, "dropped": {}}
    for _, _, _, dropped in results:
        for key, n in dropped.items():
            data["dropped"][key] = data["dropped"].get(key, 0) + n
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    with open(out, "wb") as f:
        pickle.dump(data, f)
    return data


def records_of(data, kind, lookahead, runs=None):
    """The dict records of one row type and lookahead, of the named runs (all by default)."""
    out = []
    for run, rows in data["rows"].items():
        if runs is None or run in runs:
            out += [expand(run, row) for row in rows if row[0] == kind and abs(row[1] - lookahead) < 1e-9]
    return out


def run_sets(data):
    """The run groups CP1 reports: name -> set of runs."""
    runs = data["runs"]
    reached = {r for r, m in runs.items() if m["reached"]}
    return {"reached, all cells": reached,
            "reached, straight cells": {r for r in reached if runs[r]["motion"] == "straight"},
            "reached, turn cells": {r for r in reached if runs[r]["motion"] == "turn"},
            "reached, beam": {r for r in reached if runs[r]["geometry"] == "beam"},
            "reached, gap": {r for r in reached if runs[r]["geometry"] == "gap"},
            "did not reach": set(runs) - reached}


def per_run_summary(records, error, by_regime=False, q=0.95, min_rows=MIN_ROWS):
    """over_runs of each run's q-quantile of `error`, over the runs with at least `min_rows` rows; keyed by regime
    with `by_regime`, else one summary (None: unavailable)."""
    groups = {}
    for r in records:
        if r[error] is not None:
            groups.setdefault(r["regime"] if by_regime else None, {}).setdefault(r["run"], []).append(r[error])
    out = {key: ma.over_runs({run: ma.quantile(v, q) for run, v in runs.items() if len(v) >= min_rows})
           for key, runs in groups.items()}
    return out if by_regime else out.get(None)


def _mm(v):
    return "-" if v is None else f"{1e3 * v:.1f}"


def _summary(s):
    return "unavailable | - | -" if s is None else f"{_mm(s['median'])} | {_mm(s['min'])} to {_mm(s['max'])} | {s['n']}"


def _pct(v):
    return "-" if v is None else f"{100 * v:.0f} %"


def markdown(data):
    sets = run_sets(data)
    lines = [f"# CP1 tables, leg {data['tag']} (E3 Task 5)", "",
             f"{len(data['runs'])} runs, {sum(1 for m in data['runs'].values() if m['reached'])} reached. Rows against "
             "the cell's obstacle only. Errors in mm, positive when the pair is closer than B5 predicted. Size, Regimes and "
             f"Linearization: a run's 95th percentile over its rows (runs with fewer than {MIN_ROWS} rows in the group "
             "left out), then the median and range over runs; fewer than 5 runs: unavailable. Predictability and "
             "History are not per run: the split keeps runs whole, but the offset, coverage, mean margin and constant "
             "bound are over the pooled held-out rows (a marginal comparison; Task 5 review I2).", "",
             "## Size: e_track per run", "",
             "From B5's state (what a margin in B5's rows must cover) and, matched on the same rows, from the true state "
             "at the planning cycle (without the arm's standing reference-to-measured offset); e_state is that offset "
             "at the planning cycle.", "",
             "| Runs | Type | Lookahead (s) | e_track median | range | runs | from the true state, median | range | runs | "
             "e_state median | range | runs |", "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    size = {}
    for name, runs in sets.items():
        for kind in TYPES:
            for la in LOOKAHEADS:
                recs = records_of(data, kind, la, runs)
                s = per_run_summary(recs, "e_track")
                size[(name, kind, la)] = s
                lines.append(f"| {name} | {kind} | {la:g} | {_summary(s)} | {_summary(per_run_summary(recs, 'e_track_true'))}"
                             f" | {_summary(per_run_summary(recs, 'e_state'))} |")
    lines += ["", f"Size holds (median over the runs that reached at least {1e3 * SIZE_M:g} mm) at: " + (", ".join(
        f"{kind} {la:g} s" for kind in TYPES for la in LOOKAHEADS
        if size[("reached, all cells", kind, la)] and size[("reached, all cells", kind, la)]["median"] >= SIZE_M) or "none")
        + ".", "",
        "## Regimes (descriptive)", "",
        "The runs that reached; e_track at 0.5 s per run and regime.", "",
        "| Type | Regime | e_track median | range | runs |", "|---|---|---|---|---|"]
    reached = sets["reached, all cells"]
    for kind in TYPES:
        by = per_run_summary(records_of(data, kind, 0.5, reached), "e_track", by_regime=True)
        for regime in sorted(by):
            lines.append(f"| {kind} | {regime} | {_summary(by[regime])} |")
    lines += ["", "## Predictability: the first fit", "",
              "margin = constant + weights x (v2, w2, qd2) of the plan's window, fitted on 70 % of the runs that reached "
              "and judged on the other 30 % (split by run): the held-out mean margin against the constant bound with "
              f"the same held-out coverage. Go: a saving of at least {100 * SAVING:.0f} % at a coverage within "
              f"{100 * COVERAGE_BAND:.0f} points of 95 %. The offset is set on the same held-out rows its coverage is "
              "measured on: the coverage is an in-sample calibration check, not a test of future coverage, and it is "
              "marginal over the pooled rows, not per run or per regime. The fit minimises a least-squares surrogate "
              "over the bins, not the mean margin, so its saving is not the best its form can do.", "",
              "Saving, binned: the same with each feature bin's own quantile in place of the linear form (a check that a "
              "failed linear fit is not the form's fault; not the criterion).", "",
              "| Type | Lookahead (s) | Error | Rows (train / held out) | Held-out runs | Coverage | Mean margin (mm) | "
              "Constant bound (mm) | Saving | Weights v2, w2, qd2 (mm per unit) | Go | Saving, binned |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for kind in TYPES:
        for la in LOOKAHEADS:
            recs = records_of(data, kind, la, reached)
            for error in ("e_track", "e_track_true"):
                fit = ma.first_fit(recs, error=error) if recs else None
                if fit is None:
                    lines.append(f"| {kind} | {la:g} | {error} | - | - | - | - | - | unavailable | - | - | - |")
                    continue
                go = (fit["saving"] is not None and fit["saving"] >= SAVING
                      and abs(fit["held_out_coverage"] - 0.95) <= COVERAGE_BAND)
                w = fit["weights"]
                lines.append(f"| {kind} | {la:g} | {error} | {fit['n_train']} / {fit['n_held_out']} | "
                             f"{len(fit['held_out_runs'])} | {_pct(fit['held_out_coverage'])} | {_mm(fit['mean_margin'])} | "
                             f"{_mm(fit['constant_margin'])} | {_pct(fit['saving'])} | {_mm(w['v2'])}, {_mm(w['w2'])}, "
                             f"{_mm(w['qd2'])} | {'yes' if go else 'no'} | "
                             f"{_pct((ma.first_fit(recs, error=error, model='binned') or {}).get('saving'))} |")
    lines += ["", "## History, held out", "",
              "The first fit with the current features against the same plus the features of the commands sent in the "
              f"0.5 s before the planning epoch, on the records with a history. Approach C is proposed only at a gain of "
              f"at least {100 * HISTORY_GAIN:.0f} %.", "",
              "| Type | Lookahead (s) | Rows | Saving, current | Saving, with history | Gain |", "|---|---|---|---|---|---|"]
    for kind in TYPES:
        for la in LOOKAHEADS:
            recs = records_of(data, kind, la, reached)
            g = ma.history_gain(recs).get((kind, la)) if recs else None
            if not g or not g["current"] or not g["history"]:
                lines.append(f"| {kind} | {la:g} | {g['n'] if g else 0} | unavailable | unavailable | - |")
            else:
                lines.append(f"| {kind} | {la:g} | {g['n']} | {_pct(g['current']['saving'])} | "
                             f"{_pct(g['history']['saving'])} | {_pct(g['gain'])} |")
    lines += ["", "## Linearization (for E4)", "",
              "The runs that reached. e_total is first-order in the plan's departure and is reported up to 0.2 s only.", "",
              "| Type | Lookahead (s) | e_lin median | range | runs | e_total median | range | runs |",
              "|---|---|---|---|---|---|---|---|"]
    for kind in TYPES:
        for la in (0.1, 0.2):
            recs = records_of(data, kind, la, reached)
            lines.append(f"| {kind} | {la:g} | {_summary(per_run_summary(recs, 'e_lin'))} | "
                         f"{_summary(per_run_summary(recs, 'e_total'))} |")
    lines += ["", "## Rows left out", "",
              "All traced rows of the leg (every type), by reason and stage; the tables above hold the obstacle's rows "
              "of the rest.", "", "| Reason | Stage | Rows |", "|---|---|---|"]
    for key in sorted(data["dropped"], key=lambda k: (k.split("|")[0], int(k.split("|")[1]))):
        reason, stage = key.split("|")
        lines.append(f"| {reason} | {stage} | {data['dropped'][key]} |")
    kept = {}
    for rows in data["rows"].values():
        for row in rows:
            kept[(row[0], row[1])] = kept.get((row[0], row[1]), 0) + 1
    lines += ["", "Obstacle rows kept: " + ", ".join(f"{k} {la:g} s {n}" for (k, la), n in sorted(kept.items())) + "."]
    return "\n".join(lines) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    g = sub.add_parser("gather")
    g.add_argument("directory")
    g.add_argument("--tag", required=True)
    g.add_argument("--out", required=True)
    g.add_argument("--jobs", type=int, default=3)
    r = sub.add_parser("report")
    r.add_argument("data")
    r.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    if args.command == "gather":
        data = gather(args.directory, args.tag, args.out, args.jobs)
        print(f"{len(data['runs'])} runs, {sum(len(v) for v in data['rows'].values())} obstacle rows -> {args.out}")
        return 0
    with open(args.data, "rb") as f:
        data = pickle.load(f)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(markdown(data))
    print(f"-> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
