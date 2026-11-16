"""The fixed-margin sweep's tables (E3 plan Task 4; spec docs/superpowers/specs/2026-11-15-e3-learned-margin-design.md
Section 3.3 and Amendment 1).

The sweep runs the tightness cells at arm and panel margins of 0.05, 0.04, 0.03 and 0.02 m on the realistic plant
(run tags m050 ... m020) and on the ideal plant, which executes the commands as sent (tags i050 ... i020). From the
runs' summaries:

- per cell, mode and margin: runs, reached, runs with a contact, the smallest true clearance, and whether the margin
  is safe there: zero contacts and a clearance never below 0.02 m in every run (a definition that does not depend on
  the margin);
- the best fixed margin: the one with the most passages among those safe on every cell (realistic);
- realistic against ideal at the same cell and margin (the tracking effect; CP1's Trade criterion);
- straight against turn per pair (a description: the pair also differs in geometry);
- which cells are informative: not all passing at every margin, not all failing, and ordered in the margin.

Standard library only (the host's Python 3.9).
"""
import argparse
import re
import sys
from pathlib import Path

import compare_variants as cv

SAFE_CLEARANCE_M = 0.02
TAG = re.compile(r"^(?P<mode>[mi])(?P<mm>\d{3})$")
CELL = re.compile(r"^e3_(?P<geometry>beam|gap)_(?P<motion>straight|turn)_(?P<step>\d+)$")
ORDER_TOLERANCE = 0.1  # a smaller margin may pass this share of the runs fewer than a larger one before it is disorder
P_LIMIT = 0.05


def sweep_table(directory):
    """One row per cell, mode and margin with runs under `directory` (run tags m050 ... / i050 ...): cell, mode, tag,
    margin (m), n, reached, contacts (runs with any), min_clearance (the smallest over the runs, m; None without one)
    and safe. Sorted by cell, mode, then the margin from large to small."""
    tags = sorted({m.group("tag") for m in map(cv.RUN_NAME.match, (p.name for p in Path(directory).iterdir() if p.is_dir()))
                   if m and m.group("tag") and TAG.match(m.group("tag"))})
    rows = []
    for tag in tags:
        groups, _ = cv.runs(directory, tag)
        for (cell, _kind, mode), summaries in groups.items():
            clearances = [s["plant_min_clearance_m"] for s in summaries if s.get("plant_min_clearance_m") is not None]
            contacts = sum(1 for s in summaries if s.get("plant_contacts"))
            rows.append({"cell": cell, "mode": mode, "tag": tag, "margin": int(TAG.match(tag).group("mm")) / 1000.0,
                         "n": len(summaries), "reached": sum(1 for s in summaries if s.get("reached")),
                         "contacts": contacts, "min_clearance": min(clearances) if clearances else None,
                         "safe": contacts == 0 and all(c >= SAFE_CLEARANCE_M - 1e-12 for c in clearances)})
    return sorted(rows, key=lambda r: (r["cell"], r["mode"], -r["margin"]))


def best_fixed(table):
    """The realistic tag with the most reached runs among those safe on every cell they ran (the larger margin on a
    tie); None when no margin is safe everywhere."""
    by_tag = {}
    for r in table:
        if r["mode"] == "realistic":
            by_tag.setdefault(r["tag"], []).append(r)
    safe = [(sum(r["reached"] for r in rows), rows[0]["margin"], tag) for tag, rows in by_tag.items()
            if all(r["safe"] for r in rows)]
    return max(safe)[2] if safe else None


def _smallest_safe(rows):
    margins = [r["margin"] for r in rows if r["safe"]]
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


def markdown(table):
    info = informative(table)
    lines = ["# Fixed-margin sweep (E3 Task 4)", "",
             "Safe: zero contacts and a true clearance never below 0.02 m in every run. Clearance: the smallest over the "
             "runs, any robot part against any obstacle.", "",
             "| Cell | Mode | Margin (m) | Reached | Runs with contact | Min clearance (mm) | Safe |", "|---|---|---|---|---|---|---|"]
    for r in table:
        clearance = "-" if r["min_clearance"] is None else f"{1e3 * r['min_clearance']:.1f}"
        lines.append(f"| {r['cell']} | {r['mode']} | {r['margin']:.2f} | {r['reached']}/{r['n']} | {r['contacts']} | "
                     f"{clearance} | {'yes' if r['safe'] else 'no'} |")
    lines += ["", f"Best fixed margin (realistic, safe on every cell, most passages): {best_fixed(table) or 'none'}.", "",
              "## Realistic against ideal", "",
              "The ideal plant executes the commands as sent: what differs at a cell and margin is tracking. p: one-sided "
              "Fisher, the realistic plant passing fewer.", "",
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
              "margin is uninformative and takes no part in a threshold.", "",
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
