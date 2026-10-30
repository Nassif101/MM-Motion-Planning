#!/usr/bin/env python3
"""The Phase 5A medium-layer bench (plan Task 9, spec Section 6.1): run medium_bench over a corpus and variants, and
summarize the cycle times, the split, the SCP convergence and the plan status.

    python3 tools/medium_bench.py sweep --corpus FILE --variants NAME[,NAME] --out DIR [--spacing-s S]
    python3 tools/medium_bench.py summarize --out DIR [--table FILE]

The corpus (JSON) lists cells: {"cell", "scenario", "bag" (a run's bag directory, relative to the repository) or
"synthetic": true}. Variants are named option sets of medium_bench (VARIANTS below). The bench runs in the container
(docker exec); outputs go to DIR (relative to the repository, git-ignored under experiment_runs/) as
<variant>/<cell>.json. summarize prints one row per variant and kind (cold: a full multi-start from scratch; warm: the
planner carried along the snapshots) and the backend choice by the rule of choose_backend.
"""
import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = "/workspaces/mm-motion-planning"
CONTAINER = "mm-motion-planning-ma-robot-sim-1"
BENCH = "install/mobile_manipulator_medium/lib/mobile_manipulator_medium/medium_bench"

# Stage A: backend x tolerance at the default representation; stage B: one factor at a time around the default
# (plan Task 9 Step 4, ruled one-factor-at-a-time: the full grid would take about 14 h on the host).
VARIANTS = {
    "hpipm_1e-6": ["--backend", "hpipm", "--tolerance", "1e-6"],
    "hpipm_1e-8": ["--backend", "hpipm", "--tolerance", "1e-8"],
    "osqp_1e-6": ["--backend", "osqp", "--tolerance", "1e-6"],
    "osqp_1e-8": ["--backend", "osqp", "--tolerance", "1e-8"],
    "daqp_1e-6": ["--backend", "daqp", "--tolerance", "1e-6"],
    "daqp_1e-8": ["--backend", "daqp", "--tolerance", "1e-8"],
    "seg6": ["--segments", "6"],
    "seg8": ["--segments", "8"],
    "h6": ["--horizon-s", "6"],
    "h10": ["--horizon-s", "10"],
    "dt015": ["--collision-dt", "0.15", "--torque-dt", "0.15"],
    "joints6": ["--joints", "6"],
    "torque04": ["--torque-dt", "0.4"],
}


# A plan's reach at the reference speed over the horizon (medium.yaml: speed_fraction 0.8 x v_max 0.3 m/s x 8 s).
REFERENCE_REACH_M = 1.92


def progressing(record, reach_m=REFERENCE_REACH_M):
    """An ok plan that moves: at least half of min(reach_m, the route left), or within 0.1 m of the goal (a feasible plan
    that waits in front of a tight place is not a good one). Records without the lengths count by their status."""
    if record["status"] != "ok":
        return False
    if "plan_length_m" not in record or "remaining_m" not in record:
        return True
    remaining = record["remaining_m"]
    return remaining < 0.1 or record["plan_length_m"] >= 0.5 * min(reach_m, remaining)


def percentile(values, q):
    """Nearest-rank percentile (q in 0..100) of a non-empty list."""
    ordered = sorted(values)
    rank = max(1, math.ceil(q / 100.0 * len(ordered)))
    return ordered[rank - 1]


def statistics(values):
    """mean, median, p90, p95, p99, max of a list (None for an empty one)."""
    if not values:
        return None
    return {"mean": sum(values) / len(values), "median": percentile(values, 50), "p90": percentile(values, 90),
            "p95": percentile(values, 95), "p99": percentile(values, 99), "max": max(values)}


def summarize_records(records):
    """One row for a list of bench records (one variant, one kind)."""
    total = [r["total_s"] for r in records]
    seeds = sum(r["seeds"] for r in records)
    row = {"cycles": len(records), "total_s": statistics(total),
           "linearize_share": sum(r["linearize_s"] for r in records) / max(sum(total), 1e-12),
           "qp_share": sum(r["qp_s"] for r in records) / max(sum(total), 1e-12),
           "convergence": sum(r["converged_seeds"] for r in records) / max(seeds, 1),
           "feasible_seeds": sum(r["feasible_seeds"] for r in records) / max(seeds, 1),
           "ok": sum(r["status"] == "ok" for r in records) / max(len(records), 1),
           "progressing": sum(progressing(r) for r in records) / max(len(records), 1),
           "iterations": sum(r["iterations"] for r in records) / max(len(records), 1),
           "capped": sum(bool(r.get("capped")) for r in records) / max(len(records), 1),
           "seeds": seeds / max(len(records), 1)}
    return row


def choose_backend(rows):
    """The backend variant (rows: name -> summary of its cold cycles) with the lowest cycle p99 among those whose SCP
    convergence rate is within 1 % (absolute) of the best, variants that reached the bench cap in any cycle excluded
    (they finish fewer seeds, so their convergence share does not compare); None without rows."""
    rows = {name: row for name, row in rows.items() if not row.get("capped")}
    if not rows:
        return None
    best = max(row["convergence"] for row in rows.values())
    eligible = {name: row for name, row in rows.items() if row["convergence"] >= best - 0.01}
    return min(eligible, key=lambda name: eligible[name]["total_s"]["p99"])


def load_corpus(path):
    corpus = json.loads(Path(path).read_text())
    for cell in corpus["cells"]:
        if cell.get("synthetic"):
            if "bag" in cell:
                raise ValueError(f"{cell['cell']}: a synthetic cell has no bag")
            continue
        bag = Path(cell["bag"])
        if bag.is_absolute() or ".." in bag.parts:
            raise ValueError(f"{cell['cell']}: bag paths are relative to the repository")
    return corpus


def run(cell, variant, out_dir, spacing_s):
    out = Path(out_dir) / variant / f"{cell['cell']}.json"
    (ROOT / out.parent).mkdir(parents=True, exist_ok=True)
    mode = ["synthetic"] if cell.get("synthetic") else ["open", "--bag", f"{WORKSPACE}/{cell['bag']}"]
    command = [BENCH, *mode, "--scenario", cell["scenario"], "--out", f"{WORKSPACE}/{out}", "--spacing-s", str(spacing_s),
               *VARIANTS[variant]]
    script = "source $ROS_WS/install/setup.bash 2>/dev/null; cd $ROS_WS && " + " ".join(command)
    subprocess.run(["docker", "exec", CONTAINER, "bash", "-c", script], check=True)


def summarize(out_dir):
    rows = {}
    for variant_dir in sorted((ROOT / out_dir).iterdir()):
        records = []
        for f in sorted(variant_dir.glob("*.json")):
            records += json.loads(f.read_text())["records"]
        for kind in ("cold", "warm"):
            subset = [r for r in records if r["kind"] == kind]
            if subset:
                rows[(variant_dir.name, kind)] = summarize_records(subset)
    return rows


def table(rows):
    lines = ["| Variant | Kind | Cycles | mean | median | p90 | p95 | p99 | max (s) | linearize | QP | converged | feasible "
             "seeds | ok | progressing | iterations | seeds | capped |",
             "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for (variant, kind), row in rows.items():
        t = row["total_s"]
        lines.append(f"| {variant} | {kind} | {row['cycles']} | {t['mean']:.3f} | {t['median']:.3f} | {t['p90']:.3f} | "
                     f"{t['p95']:.3f} | {t['p99']:.3f} | {t['max']:.3f} | {row['linearize_share']:.0%} | "
                     f"{row['qp_share']:.0%} | {row['convergence']:.0%} | {row['feasible_seeds']:.0%} | {row['ok']:.0%} | "
                     f"{row['progressing']:.0%} | "
                     f"{row['iterations']:.1f} | {row['seeds']:.1f} | {row['capped']:.0%} |")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    sweep = sub.add_parser("sweep")
    sweep.add_argument("--corpus", required=True)
    sweep.add_argument("--variants", required=True, help="comma-separated names of VARIANTS")
    sweep.add_argument("--out", required=True, help="output directory relative to the repository")
    sweep.add_argument("--spacing-s", type=float, default=2.0)
    summ = sub.add_parser("summarize")
    summ.add_argument("--out", required=True)
    summ.add_argument("--table", default=None)
    args = parser.parse_args()
    if args.command == "sweep":
        corpus = load_corpus(args.corpus)
        for variant in args.variants.split(","):
            if variant not in VARIANTS:
                parser.error(f"unknown variant {variant}")
            for cell in corpus["cells"]:
                print(f"== {variant} {cell['cell']}", flush=True)
                run(cell, variant, args.out, args.spacing_s)
    else:
        rows = summarize(args.out)
        text = table(rows)
        backends = {v: r for (v, k), r in rows.items() if k == "cold" and v.split("_")[0] in ("hpipm", "osqp", "daqp")}
        text += f"\n\nBackend by the rule (lowest cold p99 within 1 % of the best convergence): {choose_backend(backends)}\n"
        print(text)
        if args.table:
            Path(args.table).write_text(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
