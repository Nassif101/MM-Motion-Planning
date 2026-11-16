"""The fixed-margin sweep's tables (E3 plan Task 4; spec docs/superpowers/specs/2026-11-15-e3-learned-margin-design.md
Section 3.3 and Amendment 1).

The sweep runs the tightness cells at arm and panel margins of 0.05, 0.04, 0.03 and 0.02 m on the realistic plant
(run tags m050 ... m020) and on the ideal plant, which executes the commands as sent (tags i050 ... i020). From the
runs' summaries:

- per cell, mode and margin: runs, reached, runs with a contact, the smallest true clearance, and whether the margin
  is safe there: zero contacts and a clearance never below 0.02 m in every run, reached or not (a definition that
  does not depend on the margin). A run without a summary, or a summary without a finite clearance and a contact
  count, makes the cell incomplete: neither safe nor unsafe;
- the same pooled per obstacle (beam, gap), with the clearance of the runs that reached as a labelled diagnostic;
- the best fixed margin: the one with the most passages among those safe on every cell of a complete grid (realistic);
- realistic against ideal at the same cell and margin. The two differ in the plant AND in B5's prediction model (the
  identified model with its gain correction against the unicycle), so this compares two plant-and-model packages; it
  does not isolate tracking (Task 4 review C3). Both branches of CP1's Trade criterion are reported;
- the same seeds at adjacent margins per cell (exploratory: does the smaller margin pass more?);
- straight against turn per pair (a description: the pair also differs in geometry);
- which cells are informative: not all passing at every margin, not all failing, and ordered in the margin.

Standard library only (the host's Python 3.9).
"""
import argparse
import math
import re
import sys
from pathlib import Path

import compare_variants as cv
import margin_analysis as ma

SAFE_CLEARANCE_M = 0.02
TAG = re.compile(r"^(?P<mode>[mi])(?P<mm>\d{3})$")
CELL = re.compile(r"^e3_(?P<geometry>beam|gap)_(?P<motion>straight|turn)_(?P<step>\d+)$")
ORDER_TOLERANCE = 0.1  # a smaller margin may pass this share of the runs fewer than a larger one before it is disorder
P_LIMIT = 0.05


def sweep_table(directory):
    """One row per cell, mode and margin with runs under `directory` (run tags m050 ... / i050 ...): cell, mode, tag,
    margin (m), n (summaries read), missing (runs without a readable summary), unmeasured (summaries without a finite
    clearance and a contact count), reached, contacts (runs with any), min_clearance (the smallest over the runs, m;
    None without one), unsafe_runs (a contact or a clearance below 0.02 m), unsafe_reached, safe_reached, safe (True,
    False, or None when the cell is incomplete), reached_seeds and reached_clearances (of the runs that reached).
    Sorted by cell, mode, then the margin from large to small."""
    tags = sorted({m.group("tag") for m in map(cv.RUN_NAME.match, (p.name for p in Path(directory).iterdir() if p.is_dir()))
                   if m and m.group("tag") and TAG.match(m.group("tag"))})
    rows = []
    for tag in tags:
        groups, missing = cv.runs(directory, tag)
        for key in sorted(set(groups) | set(missing)):
            cell, _kind, mode = key
            summaries = groups.get(key, [])
            measured = [s for s in summaries if _measured(s)]
            unsafe = [s for s in measured if _unsafe(s)]
            clearances = [s["plant_min_clearance_m"] for s in measured]
            complete = len(measured) == len(summaries) and not missing.get(key) and bool(summaries)
            rows.append({"cell": cell, "mode": mode, "tag": tag, "margin": int(TAG.match(tag).group("mm")) / 1000.0,
                         "n": len(summaries), "missing": missing.get(key, 0), "unmeasured": len(summaries) - len(measured),
                         "reached": sum(1 for s in summaries if s.get("reached")),
                         "contacts": sum(1 for s in measured if s["plant_contacts"]),
                         "min_clearance": min(clearances) if clearances else None,
                         "unsafe_runs": len(unsafe), "unsafe_reached": sum(1 for s in unsafe if s.get("reached")),
                         "safe_reached": sum(1 for s in measured if s.get("reached") and not _unsafe(s)),
                         "safe": (not unsafe) if complete else None,
                         "reached_seeds": sorted(s["seed"] for s in summaries if s.get("reached") and "seed" in s),
                         "seeds": sorted(s["seed"] for s in summaries if "seed" in s),
                         "reached_clearances": [s["plant_min_clearance_m"] for s in measured if s.get("reached")]})
    return sorted(rows, key=lambda r: (r["cell"], r["mode"], -r["margin"]))


def _measured(summary):
    c, k = summary.get("plant_min_clearance_m"), summary.get("plant_contacts")
    return isinstance(c, (int, float)) and math.isfinite(c) and isinstance(k, (int, float))


def _unsafe(summary):
    return summary["plant_contacts"] > 0 or summary["plant_min_clearance_m"] < SAFE_CLEARANCE_M - 1e-12


def best_fixed(table):
    """The realistic tag with the most reached runs among those safe on every cell (the larger margin on a tie). A tag
    counts only on a complete grid: every realistic cell of the table, as many runs as the fullest row, none missing
    or unmeasured. None when no margin qualifies."""
    real = [r for r in table if r["mode"] == "realistic"]
    cells, full = {r["cell"] for r in real}, max((r["n"] for r in real), default=0)
    by_tag = {}
    for r in real:
        by_tag.setdefault(r["tag"], []).append(r)
    safe = [(sum(r["reached"] for r in rows), rows[0]["margin"], tag) for tag, rows in by_tag.items()
            if {r["cell"] for r in rows} == cells and all(r["safe"] is True and r["n"] == full for r in rows)]
    return max(safe)[2] if safe else None


def obstacle_table(table):
    """The cells of an obstacle (the cell name's beam or gap) pooled per mode and margin: n, reached, contacts,
    unsafe_runs, unsafe_reached, safe_reached, and the clearance of the runs that reached (median, 5th percentile by
    margin_analysis.quantile, minimum; m). The clearance columns condition on success: a diagnostic, not the safety
    verdict, which counts every run."""
    pools = {}
    for r in table:
        m = CELL.match(r["cell"])
        if m:
            pools.setdefault((m.group("geometry"), r["mode"], r["margin"]), []).append(r)
    out = []
    for (geometry, mode, margin), rows in sorted(pools.items(), key=lambda kv: (kv[0][0], kv[0][1], -kv[0][2])):
        c = [v for r in rows for v in r["reached_clearances"]]
        row = {"geometry": geometry, "mode": mode, "margin": margin,
               "clearance_median": ma.quantile(c, 0.5) if c else None,
               "clearance_p05": ma.quantile(c, 0.05) if c else None, "clearance_min": min(c) if c else None}
        for key in ("n", "reached", "contacts", "unsafe_runs", "unsafe_reached", "safe_reached"):
            row[key] = sum(r[key] for r in rows)
        out.append(row)
    return out


def paired_steps(table):
    """Per cell and mode, each pair of adjacent margins on the seeds both ran: seeds, only_smaller (reached at the
    smaller margin alone), only_larger, and the one-sided exact sign-test p that the smaller margin passes more.
    Exploratory: 3 steps a cell, no correction."""
    out = []
    for (cell, mode), rows in sorted(_by_cell(table).items()):
        rows = sorted(rows, key=lambda r: -r["margin"])
        for a, b in zip(rows, rows[1:]):
            seeds = set(a["seeds"]) & set(b["seeds"])
            ra, rb = set(a["reached_seeds"]) & seeds, set(b["reached_seeds"]) & seeds
            k, j = len(rb - ra), len(ra - rb)
            p = sum(math.comb(k + j, i) for i in range(k, k + j + 1)) / 2 ** (k + j)
            out.append({"cell": cell, "mode": mode, "larger": a["margin"], "smaller": b["margin"], "seeds": len(seeds),
                        "only_smaller": k, "only_larger": j, "p": p})
    return out


def _smallest_safe(rows):
    margins = [r["margin"] for r in rows if r["safe"] is True]
    return min(margins) if margins else None


def _by_cell(table):
    cells = {}
    for r in table:
        cells.setdefault((r["cell"], r["mode"]), []).append(r)
    return cells


def pair_trade(table):
    """Per geometry, step and mode with both cells of the pair: the smallest safe margin of the straight cell and of
    the turn cell (m, or None when no margin is safe) and turn minus straight. A description only: the pair differs in
    geometry as well as in motion (Task 3 review C2)."""
    cells = _by_cell(table)
    out = []
    for (cell, mode), rows in sorted(cells.items()):
        m = CELL.match(cell)
        if not m or m.group("motion") != "straight":
            continue
        turn = cells.get((cell.replace("_straight_", "_turn_"), mode))
        if turn is None:
            continue
        a, b = _smallest_safe(rows), _smallest_safe(turn)
        out.append({"geometry": m.group("geometry"), "step": int(m.group("step")), "mode": mode, "straight": a,
                    "turn": b, "difference": None if a is None or b is None else b - a})
    return out


def mode_trade(table):
    """Realistic against ideal. `cells`: per cell and margin run in both modes, the reached counts ("k/n") and the
    one-sided Fisher p that the realistic plant passes fewer; `safe_margin`: per cell the smallest safe margin in each
    mode (None when none is safe)."""
    cells = _by_cell(table)
    rows, safe = [], []
    for (cell, mode), real in sorted(cells.items()):
        ideal = cells.get((cell, "ideal"))
        if mode != "realistic" or ideal is None:
            continue
        by_margin = {r["margin"]: r for r in ideal}
        for r in real:
            i = by_margin.get(r["margin"])
            if i is not None:
                rows.append({"cell": cell, "margin": r["margin"], "realistic": f"{r['reached']}/{r['n']}",
                             "ideal": f"{i['reached']}/{i['n']}",
                             "p": cv.fisher_greater(i["reached"], i["n"], r["reached"], r["n"])})
        safe.append({"cell": cell, "realistic": _smallest_safe(real), "ideal": _smallest_safe(ideal)})
    return {"cells": rows, "safe_margin": safe}


def informative(table):
    """{(cell, mode): bool}. A cell says something about a threshold only if not every margin passes every run, not
    every margin passes none, and reached is ordered in the margin: a smaller margin passes at least as often, less a
    tenth of the runs for noise (spec Amendment 1; the rule is fixed before the sweep)."""
    out = {}
    for key, rows in _by_cell(table).items():
        rows = sorted(rows, key=lambda r: -r["margin"])
        all_pass = all(r["reached"] == r["n"] for r in rows)
        all_fail = all(r["reached"] == 0 for r in rows)
        ordered = all(b["reached"] / b["n"] >= a["reached"] / a["n"] - ORDER_TOLERANCE - 1e-12 for a, b in zip(rows, rows[1:]))
        out[key] = not all_pass and not all_fail and ordered and len(rows) > 1
    return out


def _m(v):
    return "-" if v is None else f"{v:.2f}"


def _mm(v):
    return "-" if v is None else f"{1e3 * v:.1f}"


def markdown(table):
    info = informative(table)
    lines = ["# Fixed-margin sweep (E3 Task 4)", "",
             "Safe: zero contacts and a true clearance never below 0.02 m in every run, reached or not. Clearance: the "
             "smallest over the runs, any robot part against any obstacle. Incomplete: a run without a summary, or a "
             "summary without a finite clearance and a contact count; such a cell is neither safe nor unsafe.", "",
             "| Cell | Mode | Margin (m) | Reached | Runs with contact | Min clearance (mm) | Safe |", "|---|---|---|---|---|---|---|"]
    for r in table:
        clearance = "-" if r["min_clearance"] is None else f"{1e3 * r['min_clearance']:.1f}"
        lines.append(f"| {r['cell']} | {r['mode']} | {r['margin']:.2f} | {r['reached']}/{r['n']} | {r['contacts']} | "
                     f"{clearance} | {'incomplete' if r['safe'] is None else 'yes' if r['safe'] else 'no'} |")
    lines += ["", f"Best fixed margin (realistic, safe on every cell of a complete grid, most passages): "
              f"{best_fixed(table) or 'none'}.", "",
              "## Per obstacle", "",
              "The cells of an obstacle pooled. Unsafe runs: a contact or a clearance below 0.02 m, every run counted. "
              "The clearance columns are of the runs that reached (median, 5th percentile, minimum): a diagnostic "
              "conditioned on success, not the safety verdict.", "",
              "| Obstacle | Mode | Margin (m) | Reached | Runs with contact | Unsafe runs | Unsafe and reached | Safe and "
              "reached | Clearance median (mm) | 5th percentile | Min |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in obstacle_table(table):
        lines.append(f"| {r['geometry']} | {r['mode']} | {r['margin']:.2f} | {r['reached']}/{r['n']} | {r['contacts']} | "
                     f"{r['unsafe_runs']} | {r['unsafe_reached']} | {r['safe_reached']} | {_mm(r['clearance_median'])} | "
                     f"{_mm(r['clearance_p05'])} | {_mm(r['clearance_min'])} |")
    lines += ["", "## Adjacent margins on the same seeds", "",
              "Exploratory. Seeds that reached at the smaller margin alone and at the larger alone; p: one-sided exact "
              "sign test, the smaller margin passing more; three steps a cell, no correction.", "",
              "| Cell | Mode | Larger (m) | Smaller (m) | Seeds | Only at the smaller | Only at the larger | p |",
              "|---|---|---|---|---|---|---|---|"]
    for r in paired_steps(table):
        mark = " *" if r["p"] < P_LIMIT else ""
        lines.append(f"| {r['cell']} | {r['mode']} | {r['larger']:.2f} | {r['smaller']:.2f} | {r['seeds']} | "
                     f"{r['only_smaller']} | {r['only_larger']} | {r['p']:.3g}{mark} |")
    lines += ["", "## Realistic against ideal", "",
              "The ideal plant executes the commands as sent, and B5 plans there with the unicycle: the two modes differ "
              "in the plant and in B5's prediction model, so a difference is of the two packages, not of tracking alone. "
              "p: one-sided Fisher per cell and margin, the realistic plant passing fewer, uncorrected; a p above 0.05 "
              "is no detected deficit, not equivalence.", "",
              "| Cell | Margin (m) | Realistic | Ideal | p |", "|---|---|---|---|---|"]
    trade = mode_trade(table)
    for r in trade["cells"]:
        mark = " *" if r["p"] < P_LIMIT else ""
        lines.append(f"| {r['cell']} | {r['margin']:.2f} | {r['realistic']} | {r['ideal']} | {r['p']:.3g}{mark} |")
    lines += ["", "| Cell | Smallest safe margin, realistic (m) | Ideal (m) |", "|---|---|---|"]
    for r in trade["safe_margin"]:
        lines.append(f"| {r['cell']} | {_m(r['realistic'])} | {_m(r['ideal'])} |")
    lines += ["", "## Straight against turn", "",
              "A description, not a criterion: the pair differs in geometry as well as in motion.", "",
              "| Geometry | Step | Mode | Smallest safe margin, straight (m) | Turn (m) | Turn minus straight (m) |",
              "|---|---|---|---|---|---|"]
    for r in pair_trade(table):
        lines.append(f"| {r['geometry']} | {r['step']} | {r['mode']} | {_m(r['straight'])} | {_m(r['turn'])} | "
                     f"{_m(r['difference'])} |")
    lines += ["", "## Informative cells", "",
              "A cell where every margin passes every run, every margin passes none, or reached is not ordered in the "
              "margin is uninformative and takes no part in a threshold. The rule is an eligibility screen fixed before "
              "the sweep: a flat pass rate counts as informative, so the flag is not evidence of a threshold.", "",
              "| Cell | Mode | Informative |", "|---|---|---|"]
    for (cell, mode), ok in sorted(info.items()):
        lines.append(f"| {cell} | {mode} | {'yes' if ok else 'uninformative'} |")
    return "\n".join(lines) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("directory", help="the sweep's run directory")
    parser.add_argument("--out", required=True, help="the Markdown report")
    args = parser.parse_args(argv)
    table = sweep_table(args.directory)
    if not table:
        print(f"no sweep runs (tags m### or i###) under {args.directory}", file=sys.stderr)
        return 1
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(markdown(table))
    print(f"{len(table)} cell-margin rows -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
