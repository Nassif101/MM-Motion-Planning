#!/usr/bin/env python3
"""Phase 4B-B sweep driver over the replay bench (docs/superpowers/specs/2026-10-22-phase4b-b-performance-sweep-design.md).

Runs in the dev container (it calls `ros2 run mobile_manipulator_mpc mpc_replay`).

  fidelity  --corpus FILE --out DIR              replay the installed configuration on every corpus run with the
                                                 recorded misses; exit 1 if a run is below 95 % (spec Section 3.5)
  run       --corpus FILE --variants FILE --out DIR [--rate-hz R] [--repeats N]
                                                 every variant (an overlay on the installed wholebody_mpc.yaml) on
                                                 every corpus run; one JSON per run and variant under DIR/<variant>/
  shortlist --out DIR --frozen NAME              Section 5.4's rules over DIR's variants against the frozen variant

The corpus file lists runs relative to the repository: {"runs": [{"dir": ..., "cell": ..., "shortfall": bool,
"timing_heavy": bool}]}. A variants file: {"variants": [{"name": ..., "overlay": {...}}]}.
"""
import argparse
import copy
import json
import math
import statistics
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
FIDELITY_MIN = 0.95
# Sweeps overlay variants on the frozen B5 (tag b5-frozen-2026-10-22), not the installed wholebody_mpc.yaml, which is
# B5's current default (HPIPM since the Phase 4B-B re-freeze): recorded sweeps stay reproducible. --base overrides.
FROZEN_CONFIG = ROOT / "ros2_ws/src/mobile_manipulator_mpc/test/data/frozen_wholebody_mpc.yaml"
CLEARANCE_M = 0.045      # spec Section 5.4 rule 2
CLEARANCE_SHARE = 0.99
P99_MAX_S = 0.010         # rule 3 (50 Hz: half the period)
MAX_S = 0.020
PROGRESS_SHARE = 0.95     # rule 4 (on the planned progress rate over the slack gate's horizon)
CONTOUR_GROWTH = 1.10
P99_TIE_S = 0.00025       # rule 5: p99s this close rank by fewer switches


def merge_overlay(base, overlay):
    """`base` with `overlay`'s values, nested dictionaries merged key by key (neither argument changes)."""
    merged = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = merge_overlay(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def count_switches(overlay):
    """The overlay's leaf values: how many settings a variant changes."""
    return sum(count_switches(v) if isinstance(v, dict) else 1 for v in overlay.values())


def _shortfall_share(run):
    values = [v for v in run["per_cycle"]["true_clearance_m"] if v is not None and math.isfinite(v)]
    return sum(v >= CLEARANCE_M - 1e-12 for v in values) / len(values) if values else 1.0


def _p99_max(runs):
    return max(r["timing"]["total_s"]["p99"] for r in runs)


def _failed_rules(runs, frozen, fidelity_ok):
    failed = []
    failures = sum(r["solver_failures"] for r in runs)
    self_ok = all(r.get("self_min_m") is None or r["self_min_m"] >= 0.05 - 1e-9 for r in runs)
    if not fidelity_ok or failures > sum(r["solver_failures"] for r in frozen) or not self_ok:
        failed.append(1)
    if any(_shortfall_share(r) < CLEARANCE_SHARE for r in runs if r["flags"]["shortfall"]):
        failed.append(2)
    if any(r["timing"]["total_s"]["p99"] > P99_MAX_S or r["timing"]["total_s"]["max"] > MAX_S for r in runs):
        failed.append(3)
    def pooled(rs, key, how):
        values = [r[key] for r in rs if r.get(key) is not None]
        return how(values) if values else None
    progress, frozen_progress = (pooled(rs, "progress_median_mps", statistics.median) for rs in (runs, frozen))
    contour, frozen_contour = (pooled(rs, "contour_mean_m", statistics.mean) for rs in (runs, frozen))
    if ((progress is not None and frozen_progress is not None and progress < PROGRESS_SHARE * frozen_progress) or
            (contour is not None and frozen_contour is not None and
             contour > CONTOUR_GROWTH * frozen_contour + 1e-12)):
        failed.append(4)
    return failed


def _rank(entries):
    """By p99; entries within P99_TIE_S of the fastest of their group rank by fewer switches, then p99."""
    ordered, ranked = sorted(entries, key=lambda r: r["p99_max"]), []
    while ordered:
        group = [r for r in ordered if r["p99_max"] <= ordered[0]["p99_max"] + P99_TIE_S]
        ranked += sorted(group, key=lambda r: (r["switches"], r["p99_max"]))
        ordered = ordered[len(group):]
    return ranked


def shortlist(results, frozen, fidelity_ok=True):
    """Section 5.4: the variants passing rules 1-4 ranked by their worst run's p99, then by fewer switches; when
    none passes, every variant ranked by the number of rules it fails, then p99. Each entry: name, failed (rule
    numbers), p99_max, switches."""
    ranked = [{"name": v["name"], "failed": _failed_rules(v["runs"], frozen, fidelity_ok),
               "p99_max": _p99_max(v["runs"]), "switches": v["switches"]} for v in results]
    passing = [r for r in ranked if not r["failed"]]
    if passing:
        return _rank(passing)
    return [r for n in sorted({len(r["failed"]) for r in ranked}) for r in _rank([e for e in ranked if len(e["failed"]) == n])]


def summarize(v, frozen, fidelity_ok=True):
    """One table row of a variant over its runs: timing (ms), iterations, failures, holds, the shortfall runs'
    clearance share at 0.045 m and minimum (at the stages, and between them: share at 0.045 and 0.05 m, minimum), the planned progress rate and contour, and the rules it fails."""
    runs = v["runs"]
    timing = [r["timing"]["total_s"] for r in runs]
    shortfall = [r for r in runs if r["flags"]["shortfall"]]
    clearances = [c for r in shortfall for c in r["per_cycle"]["true_clearance_m"] if c is not None]
    # Phase 4B-C: the plan between its stages (the replay's substep clearance; absent in older outputs).
    between = [c for r in shortfall for c in r["per_cycle"].get("substep_clearance_m", []) if c is not None]
    share = lambda xs, m: round(sum(c >= m for c in xs) / len(xs), 4) if xs else None
    values = lambda key: [r[key] for r in runs if r.get(key) is not None]
    return {
        "name": v["name"], "switches": v["switches"],
        "p50_ms": round(1e3 * statistics.median(t["p50"] for t in timing), 2),
        "p99_ms": round(1e3 * statistics.median(t["p99"] for t in timing), 2),
        "p99_max_ms": round(1e3 * max(t["p99"] for t in timing), 2),
        "max_ms": round(1e3 * max(t["max"] for t in timing), 2),
        "iterations_max": max(r["iterations"]["max"] for r in runs) if all("iterations" in r for r in runs) else None,
        "solver_failures": sum(r["solver_failures"] for r in runs),
        "holds": sum(r.get("holding_cycles", 0) for r in runs),
        "shortfall_share": round(sum(c >= CLEARANCE_M for c in clearances) / len(clearances), 4) if clearances else None,
        "shortfall_min_m": round(min(clearances), 4) if clearances else None,
        "between_share": share(between, CLEARANCE_M),
        "between_share_050": share(between, 0.05),
        "between_min_m": round(min(between), 4) if between else None,
        "progress_mps": round(statistics.median(values("progress_median_mps")), 4) if values("progress_median_mps") else None,
        "contour_m": round(statistics.mean(values("contour_mean_m")), 4) if values("contour_mean_m") else None,
        "failed": _failed_rules(runs, frozen, fidelity_ok),
    }


COLUMNS = ["name", "switches", "p50_ms", "p99_ms", "p99_max_ms", "max_ms", "iterations_max", "solver_failures", "holds",
           "shortfall_share", "shortfall_min_m", "between_share", "between_share_050", "between_min_m", "progress_mps", "contour_m", "failed"]


def markdown_table(rows):
    """The rows as a Markdown table (failed: the Section 5.4 rule numbers)."""
    head = "| variant | " + " | ".join(COLUMNS[1:]) + " |"
    lines = [head, "|" + "---|" * len(COLUMNS)]
    for row in rows:
        cells = [str(row[c]) if not isinstance(row[c], list) else (",".join(map(str, row[c])) or "-") for c in COLUMNS]
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def replay(run_dir, config, output, extra):
    command = ["ros2", "run", "mobile_manipulator_mpc", "mpc_replay", "--run", str(ROOT / run_dir),
               "--config", str(config), "--output", str(output)] + extra
    result = subprocess.run(command, capture_output=True, text=True)
    sys.stderr.write(result.stderr)
    if result.returncode != 0:
        raise RuntimeError(f"mpc_replay failed on {run_dir} ({result.returncode})")
    report = json.loads(Path(output).read_text())
    return report


def grid_order(runs, variants):
    """Every variant on one run before the next run: host drift over the hours of a sweep spreads over all variants."""
    return [(r, v) for r in runs for v in variants]


def run_grid(corpus, variants, out, rate_hz, repeats=3, base_config=FROZEN_CONFIG):
    runs = json.loads(Path(corpus).read_text())["runs"]
    base = yaml.safe_load(Path(base_config).read_text())
    defined = yaml.safe_load(Path(variants).read_text())["variants"]
    configs = {}
    for v in defined:
        folder = Path(out) / v["name"]
        folder.mkdir(parents=True, exist_ok=True)
        configs[v["name"]] = folder / "config.yaml"
        configs[v["name"]].write_text(yaml.safe_dump(merge_overlay(base, v.get("overlay", {})), sort_keys=False))
        (folder / "variant.json").write_text(json.dumps({"name": v["name"], "overlay": v.get("overlay", {}),
                                                         "switches": count_switches(v.get("overlay", {}))}))
    for r, name in grid_order(runs, [v["name"] for v in defined]):
        output = Path(out) / name / (Path(r["dir"]).name + ".json")
        report = replay(r["dir"], configs[name], output, ["--rate-hz", str(rate_hz), "--repeats", str(repeats),
                                                          "--variant", name])
        report["flags"] = {"shortfall": r.get("shortfall", False), "timing_heavy": r.get("timing_heavy", False)}
        output.write_text(json.dumps(report))


def load_variant(folder):
    meta = json.loads((folder / "variant.json").read_text())
    runs = [json.loads(p.read_text()) for p in sorted(folder.glob("*.json")) if p.name != "variant.json"]
    return {"name": meta["name"], "switches": meta["switches"], "runs": runs}


def fidelity(corpus, out, base_config=FROZEN_CONFIG):
    runs = json.loads(Path(corpus).read_text())["runs"]
    Path(out).mkdir(parents=True, exist_ok=True)
    worst = []
    for r in runs:
        report = replay(r["dir"], base_config, Path(out) / (Path(r["dir"]).name + ".json"),
                        ["--recorded-misses", "--repeats", "1", "--variant", "fidelity"])
        f = report["fidelity"]
        print(f"{r['cell']:45s} {Path(r['dir']).name:50s} status {f['status_match']:.3f} "
              f"stage1 {f['stage1_match']:.3f} iterations {f['iterations_match']:.3f} ({f['compared']} ok cycles)")
        worst.append(min(f["status_match"], f["stage1_match"], f["iterations_match"]))
    return all(w >= FIDELITY_MIN for w in worst)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    f = sub.add_parser("fidelity")
    f.add_argument("--corpus", required=True)
    f.add_argument("--out", required=True)
    f.add_argument("--base", default=str(FROZEN_CONFIG),
                   help="the configuration the corpus was recorded with (default: the frozen B5)")
    r = sub.add_parser("run")
    r.add_argument("--base", default=str(FROZEN_CONFIG), help="the configuration the variants overlay (default: the frozen B5)")
    r.add_argument("--corpus", required=True)
    r.add_argument("--variants", required=True)
    r.add_argument("--out", required=True)
    r.add_argument("--rate-hz", type=float, default=50.0)
    r.add_argument("--repeats", type=int, default=3)
    t = sub.add_parser("table")
    t.add_argument("--out", required=True)
    t.add_argument("--frozen", required=True)
    s = sub.add_parser("shortlist")
    s.add_argument("--out", required=True)
    s.add_argument("--frozen", required=True)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.command == "fidelity":
        return 0 if fidelity(args.corpus, args.out, args.base) else 1
    if args.command == "run":
        run_grid(args.corpus, args.variants, args.out, args.rate_hz, args.repeats, args.base)
        return 0
    folders = sorted(p for p in Path(args.out).iterdir() if (p / "variant.json").exists())
    if args.command == "table":
        frozen_runs = load_variant(Path(args.out) / args.frozen)["runs"]
        print(markdown_table([summarize(load_variant(p), frozen_runs) for p in folders]))
        return 0
    frozen = load_variant(Path(args.out) / args.frozen)["runs"]
    print(json.dumps(shortlist([load_variant(p) for p in folders if p.name != args.frozen], frozen), indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
