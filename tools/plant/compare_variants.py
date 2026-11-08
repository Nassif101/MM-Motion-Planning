"""Judge a variant's plant runs against a baseline's (E1 plan Task 5; spec
docs/superpowers/specs/2026-11-08-e1-gate-robustness-design.md Section 4, A1 and A3).

    python3 tools/plant/compare_variants.py regressions BASE VARIANT [--baseline-tag T] [--tag T] --report FILE.md
    python3 tools/plant/compare_variants.py locks DIR [--tolerance-m 0.005] --report FILE.md

`regressions` pairs the runs of each cell, kind and mode (run directories <cell>-<kind>-<mode>[-<tag>]-s<seed>; twins,
ending -n, are left out) and flags a regression: reached lower with Fisher's exact one-sided p < 0.05, more runs with
contacts, or the median true 3D minimum clearance more than 5 mm lower. `locks` lists runs whose last 10 s (500 B5
cycles) all hold: `lock` when the gate fails at a pair inside its margin by less than the tolerance (the step's lock,
A1), `deep` when deeper (an execution case for E2), `outside` when the failing pair is outside its margin, `passing`
when the gate passes throughout (the resume dwell, solver failures while holding), `unknown` for logs without the
State frames' gate field (before 93a4d91: the baseline replicas). Each exits 1
on a regression or a step lock; `regressions` exits 2 when no cell matched (a wrong tag compares nothing). Standard library only (the host's Python 3.9).
"""
import argparse
import json
import math
import re
import statistics
import sys
from pathlib import Path

import plant_stream as ps

RUN_NAME = re.compile(r"^(?P<cell>[^-]+)-(?P<kind>f2|b5)-(?P<mode>ideal|realistic)(?:-(?P<tag>[^-]+))?-s(?P<seed>\d+)$")
P_LIMIT = 0.05
CLEARANCE_DROP_M = 0.005
LOCK_CYCLES = 500  # 10 s of B5 cycles at 50 Hz


def fisher_greater(k1, n1, k2, n2):
    """One-sided Fisher exact p that group 2's success rate is lower than group 1's: P(X >= k1) for X the successes of
    group 1 under the hypergeometric law with the pooled successes fixed."""
    total, successes = n1 + n2, k1 + k2
    denom = math.comb(total, n1)
    return sum(math.comb(successes, x) * math.comb(total - successes, n1 - x)
               for x in range(k1, min(n1, successes) + 1)) / denom


def runs(directory, tag=None):
    """({(cell, kind, mode): [summary, ...]}, {(cell, kind, mode): runs without a readable summary}) of the runs with
    this tag (None: untagged) under a directory; twins (-n) are not runs of a variant."""
    groups, missing = {}, {}
    for run_dir in sorted(p for p in Path(directory).iterdir() if p.is_dir()):
        m = RUN_NAME.match(run_dir.name)
        if not m or m.group("tag") != tag:
            continue
        key = (m.group("cell"), m.group("kind"), m.group("mode"))
        try:
            groups.setdefault(key, []).append(json.loads((run_dir / "summary.json").read_text()))
        except (OSError, ValueError):  # a run that died or never ran
            missing[key] = missing.get(key, 0) + 1
    return groups, missing


def _median_clearance(summaries):
    values = [s["plant_min_clearance_m"] for s in summaries if s.get("plant_min_clearance_m") is not None]
    return statistics.median(values) if values else None


def regressions(baseline_dir, variant_dir, baseline_tag=None, variant_tag=None):
    (base, base_missing), (var, var_missing) = runs(baseline_dir, baseline_tag), runs(variant_dir, variant_tag)
    rows = []
    for key in sorted(set(base) & set(var)):
        b, v = base[key], var[key]
        kb, kv = sum(1 for s in b if s.get("reached")), sum(1 for s in v if s.get("reached"))
        cb = sum(1 for s in b if s.get("plant_contacts", 0) > 0)
        cvn = sum(1 for s in v if s.get("plant_contacts", 0) > 0)
        mb, mv = _median_clearance(b), _median_clearance(v)
        p = fisher_greater(kb, len(b), kv, len(v))
        drop = (mv - mb) if mb is not None and mv is not None else None
        reasons = []
        if kv / len(v) < kb / len(b) and p < P_LIMIT:
            reasons.append(f"reached {kb}/{len(b)} -> {kv}/{len(v)} (p {p:.3f})")
        if cvn > cb:
            reasons.append(f"contacts {cb} -> {cvn} runs")
        if drop is not None and drop < -CLEARANCE_DROP_M:
            reasons.append(f"median clearance {mb:.3f} -> {mv:.3f} m")
        if var_missing.get(key):
            reasons.append(f"{var_missing[key]} variant runs without a summary")
        rows.append({"cell": key[0], "kind": key[1], "mode": key[2], "base_runs": len(b), "variant_runs": len(v),
                     "base_reached": kb, "variant_reached": kv, "p": p, "base_contacts": cb, "variant_contacts": cvn,
                     "base_clearance_m": mb, "variant_clearance_m": mv, "variant_missing": var_missing.get(key, 0),
                     "regressed": bool(reasons), "reasons": reasons})
    return rows


def one_sided(baseline_dir, variant_dir, baseline_tag=None, variant_tag=None):
    """The groups found on one side only: "<cell> <kind> <mode>: baseline only" or "...: variant only"."""
    base, var = runs(baseline_dir, baseline_tag)[0], runs(variant_dir, variant_tag)[0]
    return ([f"{' '.join(k)}: baseline only" for k in sorted(set(base) - set(var))] +
            [f"{' '.join(k)}: variant only" for k in sorted(set(var) - set(base))])


def locks(directory, tolerance_m=0.005, cycles=LOCK_CYCLES, tag=None):
    """Runs under `directory` (of this tag, None: untagged; twins left out) whose last `cycles` State frames all hold,
    with their kind (lock, deep, outside, passing, unknown) and the largest gate violation over those frames."""
    found = []
    for log_path in sorted(Path(directory).glob("*/run.mmlog")):
        name = log_path.parent.name
        m = RUN_NAME.match(name)
        if name.endswith("-n") or (m.group("tag") if m else None) != tag:
            continue
        tail = []
        for kind, header, _ in ps.RunLog(log_path).frames():
            if kind == ps.STATE:
                tail.append(header)
                if len(tail) > cycles:
                    tail.pop(0)
        if len(tail) < cycles or any(h.get("status") != "holding" for h in tail):
            continue
        if any("gate" not in h for h in tail):  # logs from before the State frames carried the gate (93a4d91)
            found.append({"run": log_path.parent.name, "kind": "unknown", "violation_m": None})
            continue
        worst = max(h["gate"][1] for h in tail)
        if not any(h["gate"][2] for h in tail):  # the gate passes throughout: the resume dwell, solver failures
            kind = "passing"
        else:
            kind = "outside" if not worst > 0.0 else ("lock" if worst < tolerance_m else "deep")
        found.append({"run": log_path.parent.name, "kind": kind, "violation_m": worst})
    return found


def regressions_markdown(rows):
    lines = ["| cell | kind | mode | reached base | reached variant | p | contacts | median clearance m | regressed |",
             "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        cl = (f"{r['base_clearance_m']:.3f} -> {r['variant_clearance_m']:.3f}"
              if r["base_clearance_m"] is not None and r["variant_clearance_m"] is not None else "-")
        lines.append(f"| {r['cell']} | {r['kind']} | {r['mode']} | {r['base_reached']}/{r['base_runs']} | "
                     f"{r['variant_reached']}/{r['variant_runs']} | {r['p']:.3f} | {r['base_contacts']} -> "
                     f"{r['variant_contacts']} | {cl} | {'; '.join(r['reasons']) or 'no'} |")
    return "\n".join(lines) + "\n"


def locks_markdown(found):
    lines = ["| run | kind | largest violation m |", "|---|---|---|"]
    lines += [f"| {f['run']} | {f['kind']} | " + ("-" if f["violation_m"] is None else f"{f['violation_m']:.4f}") + " |"
              for f in found]
    return "\n".join(lines) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="step", required=True)
    r = sub.add_parser("regressions")
    r.add_argument("base")
    r.add_argument("variant")
    r.add_argument("--baseline-tag")
    r.add_argument("--tag")
    r.add_argument("--report", required=True)
    k = sub.add_parser("locks")
    k.add_argument("dir")
    k.add_argument("--tolerance-m", type=float, default=0.005)
    k.add_argument("--tag")
    k.add_argument("--report", required=True)
    args = ap.parse_args(argv)
    if args.step == "regressions":
        rows = regressions(args.base, args.variant, args.baseline_tag, args.tag)
        sided = one_sided(args.base, args.variant, args.baseline_tag, args.tag)
        text = regressions_markdown(rows) + "".join(f"\n- {line}" for line in sided) + ("\n" if sided else "")
        if not rows:
            text += "\nNo cell matched between baseline and variant (check the tags): nothing was compared.\n"
        code = 2 if not rows else (1 if any(row["regressed"] for row in rows) else 0)
    else:
        found = locks(args.dir, args.tolerance_m, tag=args.tag)
        text = locks_markdown(found)
        code = 1 if any(f["kind"] == "lock" for f in found) else 0
    Path(args.report).write_text(text)
    print(text)
    return code


if __name__ == "__main__":
    sys.exit(main())
