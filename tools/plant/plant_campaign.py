"""Run campaigns on the MuJoCo plant in parallel (MuJoCo plant plan Task 8).

    python3 tools/plant/plant_campaign.py run --corpus FILE --mode ideal|realistic --seeds N --jobs J --out DIR
        [--kinds f2,b5] [--lidar] [--timeout-s T] [--container NAME]
    python3 tools/plant/plant_campaign.py table --out DIR

`run` starts the cells x seeds x kinds runs of the plant_campaign binary, J at a time (from the host through
`docker exec`, or directly inside the container with --container ""), each into DIR/<cell>-<kind>-<mode>-s<seed>/
(summary.json, run.mmlog). Seeds perturb the start pose (+-1 mm, +-1 mrad) and drive the plant's noise: judge results
on the distributions over seeds (tune-with-ensembles). `table` prints the runs and the per-cell ensemble.
"""
import argparse
import json
import statistics
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CONTAINER = "mm-motion-planning-ma-robot-sim-1"
WORKSPACE = "/workspaces/mm-motion-planning"
BINARY = "ros2_ws/install/mobile_manipulator_campaign/lib/mobile_manipulator_campaign/plant_campaign"


def load_corpus(path):
    return json.loads(Path(path).read_text())["cells"]


def runs_of(cells, seeds, mode, kinds=("f2", "b5"), lidar=False, timeout_s=None, profile=None, tag=None):
    """One run per cell, seed (1..seeds) and kind: its directory name and the binary's arguments (but --out). A
    `profile` variant (the grip ensemble) is passed to every run, its `tag` in the run names."""
    runs = []
    for cell in cells:
        for kind in kinds:
            for seed in range(1, seeds + 1):
                args = ["--scenario", cell["scenario"], "--mode", mode, "--seed", str(seed)]
                if cell.get("bag"):
                    args += ["--bag", cell["bag"]]
                if kind == "b5":
                    args.append("--b5-only")
                if lidar:
                    args.append("--lidar")
                if timeout_s:
                    args += ["--timeout-s", str(timeout_s)]
                if profile:
                    args += ["--profile", str(profile)]  # repository-relative: the runs start in the repository root
                name = f"{cell['cell']}-{kind}-{mode}" + (f"-{tag}" if tag else "") + f"-s{seed}"
                runs.append({"name": name, "cell": cell["cell"], "kind": kind, "seed": seed, "variant": tag or "",
                             "args": args})
    return runs


def pending(runs, out_dir):
    """The runs without a summary.json under out_dir (a campaign restarted where it stopped)."""
    return [r for r in runs if not (Path(out_dir) / r["name"] / "summary.json").exists()]


def partition(runs, jobs):
    """The runs dealt round-robin into `jobs` lists (sizes within one)."""
    parts = [[] for _ in range(max(1, jobs))]
    for i, run in enumerate(runs):
        parts[i % len(parts)].append(run)
    return parts


def command(run, out_dir, container=CONTAINER):
    """The command for one run: through docker exec from the host, or the binary itself inside the container."""
    if not container:
        return [str(ROOT / BINARY)] + run["args"] + ["--out", str(out_dir)]
    inner = " ".join([f"{WORKSPACE}/{BINARY}"] + run["args"] + ["--out", f"{WORKSPACE}/{out_dir}"])
    return ["docker", "exec", container, "bash", "-lc", f"cd {WORKSPACE} && source ros2_ws/install/setup.bash && {inner}"]


def table(records):
    lines = ["| Cell | Kind | Seed | Profile | Reached | Time (s) | Min clearance (m) | Plant min clearance (m) | Contacts | "
             "Following | RTF |", "|---|---|---:|---|---|---:|---:|---:|---:|---:|---:|"]
    fmt = lambda x: "-" if x is None else f"{x:.3f}"
    for r in records:
        following = r.get("following_cycles", 0) / max(r.get("b5_cycles", 1), 1)
        lines.append(f"| {r['cell']} | {r.get('kind', '-')} | {r.get('seed', '-')} | {r.get('plant_profile', '-')} | "
                     f"{'yes' if r['reached'] else 'no (' + ('stalled' if r.get('stalled') else 'timeout' if r.get('timed_out') else r.get('end_status', '?')) + ')'} | {r['time_s']:.1f} | "
                     f"{fmt(r.get('min_clearance_m'))} | {fmt(r.get('plant_min_clearance_m'))} | {r.get('plant_contacts', '-')} | "
                     f"{following:.0%} | {r.get('real_time_factor', 0.0):.1f} |")
    return "\n".join(lines)


def ensemble_table(records):
    """Per cell and kind over the seeds: runs, goals reached (rate) and the time's median and spread."""
    groups = {}
    for r in records:
        groups.setdefault((r["cell"], r.get("kind", "-")), []).append(r)
    lines = ["| Cell | Kind | Runs | Reached | Time median (s) | Time IQR (s) |", "|---|---|---:|---|---:|---:|"]
    for (cell, kind), rs in sorted(groups.items()):
        n, ok = len(rs), sum(r["reached"] for r in rs)
        times = sorted(r["time_s"] for r in rs if r["reached"])
        median = f"{statistics.median(times):.1f}" if times else "-"
        iqr = f"{times[(3 * len(times)) // 4] - times[len(times) // 4]:.1f}" if len(times) >= 4 else "-"
        lines.append(f"| {cell} | {kind} | {n} | {ok}/{n} ({100 * ok / n:.0f} %) | {median} | {iqr} |")
    return "\n".join(lines)


def run(args):
    out = Path(args.out)
    (ROOT / out).mkdir(parents=True, exist_ok=True)
    runs = runs_of(load_corpus(args.corpus), args.seeds, args.mode, tuple(args.kinds.split(",")), args.lidar, args.timeout_s,
                   args.profile, args.tag)
    left = pending(runs, ROOT / out)
    print(f"{len(runs)} runs, {len(runs) - len(left)} already done", flush=True)
    runs = left

    def one(r):
        log = ROOT / out / f"{r['name']}.log"
        with open(log, "w") as f:
            code = subprocess.run(command(r, out / r["name"], args.container), stdout=f, stderr=subprocess.STDOUT).returncode
        print(f"{r['name']}: {'ok' if code == 0 else f'exit {code} (see {log.relative_to(ROOT)})'}", flush=True)
        return code

    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        codes = list(pool.map(one, runs))
    return 0 if all(c == 0 for c in codes) else 1


def collect(out):
    return [json.loads(p.read_text()) for p in sorted((ROOT / out).glob("*/summary.json"))]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="step", required=True)
    r = sub.add_parser("run")
    r.add_argument("--corpus", required=True)
    r.add_argument("--mode", choices=["ideal", "realistic"], required=True)
    r.add_argument("--seeds", type=int, default=1)
    r.add_argument("--jobs", type=int, default=4)
    r.add_argument("--kinds", default="f2,b5")
    r.add_argument("--lidar", action="store_true")
    r.add_argument("--timeout-s", type=float)
    r.add_argument("--container", default=CONTAINER)
    r.add_argument("--profile", help="a robot profile variant (repository-relative), passed to every run")
    r.add_argument("--tag", help="the variant's tag in the run names")
    r.add_argument("--out", required=True)
    t = sub.add_parser("table")
    t.add_argument("--out", required=True)
    args = ap.parse_args()
    if args.step == "run":
        return run(args)
    records = collect(args.out)
    print(table(records) + "\n\n" + ensemble_table(records))
    return 0


if __name__ == "__main__":
    sys.exit(main())
