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
CLEARANCE_M = 0.045      # spec Section 5.4 rule 2
CLEARANCE_SHARE = 0.99
P99_MAX_S = 0.010         # rule 3 (50 Hz: half the period)
MAX_S = 0.020
PROGRESS_SHARE = 0.95     # rule 4
CONTOUR_GROWTH = 1.10


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
    progress = statistics.median(r["progress_median_m"] for r in runs)
    contour = statistics.mean(r["contour_mean_m"] for r in runs)
    if (progress < PROGRESS_SHARE * statistics.median(r["progress_median_m"] for r in frozen) or
            contour > CONTOUR_GROWTH * statistics.mean(r["contour_mean_m"] for r in frozen) + 1e-12):
        failed.append(4)
    return failed


def shortlist(results, frozen, fidelity_ok=True):
    """Section 5.4: the variants passing rules 1-4 ranked by their worst run's p99, then by fewer switches; when
    none passes, every variant ranked by the number of rules it fails, then p99. Each entry: name, failed (rule
    numbers), p99_max, switches."""
    ranked = [{"name": v["name"], "failed": _failed_rules(v["runs"], frozen, fidelity_ok),
               "p99_max": _p99_max(v["runs"]), "switches": v["switches"]} for v in results]
    passing = [r for r in ranked if not r["failed"]]
    if passing:
        return sorted(passing, key=lambda r: (r["p99_max"], r["switches"]))
    return sorted(ranked, key=lambda r: (len(r["failed"]), r["p99_max"], r["switches"]))


def installed_config():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("mobile_manipulator_mpc")) / "config" / "wholebody_mpc.yaml"


def replay(run_dir, config, output, extra):
    command = ["ros2", "run", "mobile_manipulator_mpc", "mpc_replay", "--run", str(ROOT / run_dir),
               "--config", str(config), "--output", str(output)] + extra
    result = subprocess.run(command, capture_output=True, text=True)
    sys.stderr.write(result.stderr)
    if result.returncode != 0:
        raise RuntimeError(f"mpc_replay failed on {run_dir} ({result.returncode})")
    report = json.loads(Path(output).read_text())
    return report


def run_grid(corpus, variants, out, rate_hz, repeats=3):
    runs = json.loads(Path(corpus).read_text())["runs"]
    base = yaml.safe_load(installed_config().read_text())
    for v in yaml.safe_load(Path(variants).read_text())["variants"]:
        folder = Path(out) / v["name"]
        folder.mkdir(parents=True, exist_ok=True)
        config = folder / "config.yaml"
        config.write_text(yaml.safe_dump(merge_overlay(base, v.get("overlay", {})), sort_keys=False))
        (folder / "variant.json").write_text(json.dumps({"name": v["name"], "overlay": v.get("overlay", {}),
                                                         "switches": count_switches(v.get("overlay", {}))}))
        for r in runs:
            output = folder / (Path(r["dir"]).name + ".json")
            report = replay(r["dir"], config, output, ["--rate-hz", str(rate_hz), "--repeats", str(repeats),
                                                       "--variant", v["name"]])
            report["flags"] = {"shortfall": r.get("shortfall", False), "timing_heavy": r.get("timing_heavy", False)}
            output.write_text(json.dumps(report))


def load_variant(folder):
    meta = json.loads((folder / "variant.json").read_text())
    runs = [json.loads(p.read_text()) for p in sorted(folder.glob("*.json")) if p.name != "variant.json"]
    return {"name": meta["name"], "switches": meta["switches"], "runs": runs}


def fidelity(corpus, out):
    runs = json.loads(Path(corpus).read_text())["runs"]
    Path(out).mkdir(parents=True, exist_ok=True)
    worst = []
    for r in runs:
        report = replay(r["dir"], installed_config(), Path(out) / (Path(r["dir"]).name + ".json"),
                        ["--recorded-misses", "--repeats", "1", "--variant", "fidelity"])
        f = report["fidelity"]
        print(f"{r['cell']:45s} {Path(r['dir']).name:50s} status {f['status_match']:.3f} "
              f"stage1 {f['stage1_match']:.3f} iterations {f['iterations_match']:.3f} ({f['compared']} ok cycles)")
        worst.append(min(f["status_match"], f["stage1_match"], f["iterations_match"]))
    return all(w >= FIDELITY_MIN for w in worst)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    f = sub.add_parser("fidelity")
    f.add_argument("--corpus", required=True)
    f.add_argument("--out", required=True)
    r = sub.add_parser("run")
    r.add_argument("--corpus", required=True)
    r.add_argument("--variants", required=True)
    r.add_argument("--out", required=True)
    r.add_argument("--rate-hz", type=float, default=50.0)
    r.add_argument("--repeats", type=int, default=3)
    s = sub.add_parser("shortlist")
    s.add_argument("--out", required=True)
    s.add_argument("--frozen", required=True)
    args = parser.parse_args(argv)
    if args.command == "fidelity":
        return 0 if fidelity(args.corpus, args.out) else 1
    if args.command == "run":
        run_grid(args.corpus, args.variants, args.out, args.rate_hz, args.repeats)
        return 0
    folders = [p for p in Path(args.out).iterdir() if (p / "variant.json").exists()]
    frozen = load_variant(Path(args.out) / args.frozen)["runs"]
    print(json.dumps(shortlist([load_variant(p) for p in folders if p.name != args.frozen], frozen), indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
