"""Run campaigns on the MuJoCo plant in parallel (MuJoCo plant plan Task 8).

    python3 tools/plant/plant_campaign.py run --corpus FILE --mode ideal|realistic --seeds N --jobs J --out DIR
        [--kinds f2,b5] [--lidar] [--timeout-s T] [--container NAME] [--profile P --tag T] [--b5-config F]
        [--start-nudge EPS]
    python3 tools/plant/plant_campaign.py table --out DIR

`run` starts the cells x seeds x kinds runs of the plant_campaign binary, J at a time (from the host through
`docker exec`, or directly inside the container with --container ""), each into DIR/<cell>-<kind>-<mode>-s<seed>/
(summary.json, run.mmlog). Seeds perturb the start pose (+-1 mm, +-1 mrad) and drive the plant's noise: judge results
on the distributions over seeds (tune-with-ensembles). `table` prints the runs and the per-cell ensemble.
"""
import argparse
import hashlib
import json
import shlex
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


def runs_of(cells, seeds, mode, kinds=("f2", "b5"), lidar=False, timeout_s=None, profile=None, tag=None,
            b5_config=None, start_nudge=None, seed_list=None, cells_only=None, diagnostics=False,
            tracking_model=None, tracking_adaptation=None, profile_timing=False, row_trace=False, medium_config=None,
            routes="recorded"):
    """One run per cell, seed (1..seeds) and kind: its directory name and the binary's arguments (but --out). A
    `profile` variant (the grip ensemble) or a `b5_config` variant (E1) is passed to every run, its `tag` in the run
    names; `start_nudge` (rad, E1's twins) is added to every run's start yaw and marks the runs `twin` (name suffix
    -n). E2: `seed_list` replaces 1..seeds, `cells_only` keeps those corpus cells (ValueError naming an unknown one),
    `diagnostics` records the gate and command diagnostics (--diagnostics). E2 Task 7: `tracking_model` (a
    repository-relative model file, or "none": the unicycle) and `tracking_adaptation` ("on" or "off") override the B5
    config's for every run; either one is a variant and needs a `tag`. E3 Task 1: `row_trace` records B5's row trace
    (--row-trace), a diagnostic like `diagnostics`. E3 Task 4: `medium_config` (a repository-relative medium.yaml
    variant) is passed to every run and needs a `tag`. Spec 2 Task 8: `routes` "lattice" plans every run's routes with
    the Smac Lattice port (--routes lattice): no bag, the full hierarchy only (no b5 kind), a cell's optional `goals`
    (a repository-relative goal script) passed with --goals, and "-lattice" in the run names."""
    if cells_only:
        unknown = sorted(set(cells_only) - {c["cell"] for c in cells})
        if unknown:
            raise ValueError(f"cells not in the corpus: {', '.join(unknown)}")
        cells = [c for c in cells if c["cell"] in cells_only]
    seed_values = list(seed_list) if seed_list else list(range(1, seeds + 1))
    if routes not in ("recorded", "lattice"):
        raise ValueError(f"routes {routes!r}: recorded|lattice")
    if routes == "lattice" and "b5" in kinds:
        raise ValueError("lattice routes run the full hierarchy only: use --kinds f2")
    if tag and "-" in tag:
        raise ValueError(f"tag {tag!r}: no hyphen (run names are <cell>-<kind>-<mode>-<tag>-s<seed>; use _ )")
    if b5_config:
        if not tag:
            raise ValueError("a --b5-config variant needs a --tag: its runs would take the baseline runs' names")
        b5_config = repository_relative(b5_config)
    if medium_config:
        if not tag:
            raise ValueError("a --medium-config variant needs a --tag: its runs would take the baseline runs' names")
        medium_config = repository_relative(medium_config)
    if tracking_adaptation is not None and tracking_adaptation not in ("on", "off", "gain"):
        raise ValueError(f"tracking_adaptation {tracking_adaptation!r}: on|off|gain (gain: the turn gain alone)")
    if tracking_model == "none" and tracking_adaptation in ("on", "gain"):
        raise ValueError("tracking_model none is the unicycle: it takes no tracking_adaptation on|gain (final review C2)")
    if ((tracking_model and tracking_model != "none") or tracking_adaptation) and not tag:  # none: the old default
        raise ValueError("a --tracking-model or --tracking-adaptation variant needs a --tag")
    if tracking_model and tracking_model != "none":
        tracking_model = repository_relative(tracking_model)
    runs = []
    for cell in cells:
        for kind in kinds:
            for seed in seed_values:
                args = ["--scenario", cell["scenario"], "--mode", mode, "--seed", str(seed)]
                if cell["cell"] != cell["scenario"]:
                    args += ["--cell", cell["cell"]]  # the summary names the corpus cell
                if routes == "lattice":
                    args += ["--routes", "lattice"]
                    if cell.get("goals"):
                        args += ["--goals", str(repository_relative(cell["goals"]))]
                elif cell.get("bag"):
                    args += ["--bag", cell["bag"]]
                if kind == "b5":
                    args.append("--b5-only")
                if lidar:
                    args.append("--lidar")
                if timeout_s:
                    args += ["--timeout-s", str(timeout_s)]
                if profile:
                    args += ["--profile", str(profile)]  # repository-relative: the runs start in the repository root
                if b5_config:
                    args += ["--b5-config", str(b5_config)]  # repository-relative, as the profile
                if start_nudge:
                    args += ["--start-nudge", repr(float(start_nudge))]
                if diagnostics:
                    args.append("--diagnostics")
                if row_trace:
                    args.append("--row-trace")
                if medium_config:
                    args += ["--medium-config", str(medium_config)]
                if tracking_model:
                    args += ["--tracking-model", str(tracking_model)]
                if tracking_adaptation:
                    args += ["--tracking-adaptation", tracking_adaptation]
                if profile_timing:
                    args.append("--profile-timing")  # E2 Task 9 (T7): wall times in the summary
                name = (f"{cell['cell']}-{kind}-{mode}" + ("-lattice" if routes == "lattice" else "") + (f"-{tag}" if tag else "")
                        + f"-s{seed}" + ("-n" if start_nudge else ""))
                runs.append({"name": name, "cell": cell["cell"], "kind": kind, "seed": seed, "variant": tag or "",
                             "args": args, "twin": bool(start_nudge), "b5_config": b5_config or None,
                             "tracking_model": tracking_model if tracking_model and tracking_model != "none" else None})
    return runs


def host_path(path):
    """A path a run recorded (relative to the repository root, or absolute in the container's or the host's view) on
    this side."""
    if path.startswith(WORKSPACE + "/"):
        return ROOT / path[len(WORKSPACE) + 1:]
    return Path(path) if Path(path).is_absolute() else ROOT / path


def repository_relative(path):
    """A path under the repository as the runs see it from the repository root (host and container alike); ValueError
    for one outside it, which the container does not mount."""
    path = Path(path)
    if not path.is_absolute():
        return path.as_posix()
    try:
        return path.resolve().relative_to(ROOT.resolve()).as_posix()
    except ValueError:
        raise ValueError(f"{path} is outside the repository, which is all the container mounts") from None


INSTALL = WORKSPACE + "/ros2_ws/install/"  # a Docker named volume: the host cannot read it
INSTALLED_B5_CONFIG = INSTALL + "mobile_manipulator_mpc/share/mobile_manipulator_mpc/config/wholebody_mpc.yaml"
INSTALLED_TRACKING_MODEL = INSTALL + "mobile_manipulator_mpc/share/mobile_manipulator_mpc/config/tracking_model_mujoco.yaml"


def installed_sha256(path, container=CONTAINER):
    """SHA-256 of a file in the container's install tree ("" when absent): sha256sum through docker exec (the host
    cannot read the named volume), or directly with container None (inside the container)."""
    if not container or ROOT.as_posix() == WORKSPACE:  # inside the container: the volume is mounted here
        return file_sha256(host_path(path))
    out = subprocess.run(["docker", "exec", container, "sha256sum", path], capture_output=True, text=True)
    return out.stdout.split()[0] if out.returncode == 0 and out.stdout else ""


def pending(runs, out_dir, container=CONTAINER):
    """The runs not yet done under out_dir (a campaign restarted where it stopped): no summary.json, or one written for
    other arguments (another bag, profile, timeout, lidar...: the summary records the arguments it ran with), or for
    another B5 (final review C1): the config file it loaded (an explicit variant, or the installed wholebody_mpc.yaml)
    changed since, or an explicit --medium-config file changed (or the summary predates its hash), or its tracking model
    is not the one the arguments now select. Without --tracking-model that is a
    model (the config's, else the installed MuJoCo default since CP4; a pre-CP4 unicycle summary is stale), with the
    loaded file unchanged; with none, no model. Installed files are hashed in `container` (I1), once each."""
    hashes = {}

    def sha(path):
        if path not in hashes:
            hashes[path] = installed_sha256(path, container) if path.startswith(INSTALL) else file_sha256(host_path(path))
        return hashes[path]

    def done(r):
        path = Path(out_dir) / r["name"] / "summary.json"
        try:
            summary = json.loads(path.read_text())
        except (OSError, ValueError):
            return False
        if summary.get("args") != r["args"]:
            return False
        config = (ROOT / r["b5_config"]).as_posix() if r.get("b5_config") else INSTALLED_B5_CONFIG
        if summary.get("b5_config_sha256") != (file_sha256(config) if r.get("b5_config") else sha(config)):
            return False
        if "--medium-config" in r["args"]:  # an explicit medium variant: the bytes the run used (E3 final review I2)
            medium = ROOT / r["args"][r["args"].index("--medium-config") + 1]
            if summary.get("medium_config_sha256") != file_sha256(medium):
                return False
        # the model is the one the run loaded, as the C++ loader resolved it (the summary's path; closure R2)
        model = summary.get("tracking_model")
        selected = r["args"][r["args"].index("--tracking-model") + 1] if "--tracking-model" in r["args"] else None
        if selected == "none":
            return not model
        return bool(model) and summary.get("tracking_model_sha256") == sha(model)
    return [r for r in runs if not done(r)]


def file_sha256(path):
    """SHA-256 of a file's bytes, lowercase hex ("" when it cannot be read), as the runs' summaries record it."""
    try:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    except OSError:
        return ""


def container_path(path):
    """A repository path (relative, or absolute under the repository) as the container sees it."""
    path = Path(path)
    if path.is_absolute():
        try:
            path = path.resolve().relative_to(ROOT)
        except ValueError:
            raise ValueError(f"{path} is outside the repository, which is all the container mounts") from None
    return f"{WORKSPACE}/{path.as_posix()}"


def command(run, out_dir, container=CONTAINER):
    """The command for one run: through docker exec from the host, or the binary itself inside the container."""
    if not container:
        return [str(ROOT / BINARY)] + run["args"] + ["--out", str(out_dir)]
    inner = shlex.join([f"{WORKSPACE}/{BINARY}"] + run["args"] + ["--out", container_path(out_dir)])
    return ["docker", "exec", container, "bash", "-lc", f"cd {WORKSPACE} && source ros2_ws/install/setup.bash && {inner}"]


def stop_command(out_dir, container=CONTAINER):
    """Stops this campaign's runs in the container (an interrupted host leaves them running otherwise: docker exec does
    not forward the interrupt), and only those: their --out lies under out_dir."""
    return ["docker", "exec", container, "pkill", "-INT", "-f", f"[p]lant_campaign .*--out {container_path(out_dir)}/"]


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


def run_all(runs, out_dir, jobs, container=CONTAINER):
    """Runs `runs` `jobs` at a time into out_dir/<name>/ (repository-relative or absolute), each run's output in
    out_dir/<name>.log; the number of failed runs. Ctrl-C stops this campaign's runs in the container too, then
    re-raises."""
    out = Path(out_dir)

    def one(r):
        log = ROOT / out / f"{r['name']}.log"
        with open(log, "w") as f:
            code = subprocess.run(command(r, out / r["name"], container), stdout=f, stderr=subprocess.STDOUT).returncode
        shown = log.relative_to(ROOT) if log.is_relative_to(ROOT) else log
        print(f"{r['name']}: {'ok' if code == 0 else f'exit {code} (see {shown})'}", flush=True)
        return code

    (ROOT / out).mkdir(parents=True, exist_ok=True)
    pool = ThreadPoolExecutor(max_workers=jobs)
    try:
        codes = list(pool.map(one, runs))
    except KeyboardInterrupt:
        pool.shutdown(wait=False, cancel_futures=True)
        if container:
            subprocess.run(stop_command(out, container), check=False)
        raise
    pool.shutdown()
    return sum(1 for c in codes if c != 0)


def run(args):
    out = Path(args.out)
    (ROOT / out).mkdir(parents=True, exist_ok=True)
    runs = runs_of(load_corpus(args.corpus), args.seeds, args.mode, tuple(args.kinds.split(",")), args.lidar, args.timeout_s,
                   args.profile, args.tag, args.b5_config, args.start_nudge,
                   seed_list=[int(v) for v in args.seed_list.split(",")] if args.seed_list else None,
                   cells_only=args.cells.split(",") if args.cells else None, diagnostics=args.diagnostics,
                   tracking_model=args.tracking_model, tracking_adaptation=args.tracking_adaptation,
                   profile_timing=args.profile_timing, row_trace=args.row_trace,
                   medium_config=args.medium_config, routes=args.routes)
    left = pending(runs, ROOT / out, args.container)
    print(f"{len(runs)} runs, {len(runs) - len(left)} already done", flush=True)
    try:
        failed = run_all(left, out, args.jobs, args.container)
    except KeyboardInterrupt:
        print("interrupted: this campaign's runs stopped; a rerun resumes", flush=True)
        return 130
    return 0 if failed == 0 else 1


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
    r.add_argument("--b5-config", help="a B5 config variant (repository-relative wholebody_mpc.yaml copy), every run (E1)")
    r.add_argument("--start-nudge", type=float, help="rad added to every run's start yaw: twin runs (E1), names end -n")
    r.add_argument("--seed-list", help="comma-separated seeds in place of 1..--seeds (E2)")
    r.add_argument("--cells", help="comma-separated corpus cells to run, the others left out (E2)")
    r.add_argument("--diagnostics", action="store_true", help="record the gate and command diagnostics (E2)")
    r.add_argument("--medium-config", help="a medium planner config variant (repository-relative), every run (E3)")
    r.add_argument("--row-trace", action="store_true", help="record B5's row trace, rows/rows_x/rows_u (E3)")
    r.add_argument("--tracking-model", help="B5's tracking model (repository-relative) or none, every run (E2)")
    r.add_argument("--tracking-adaptation", choices=["on", "off", "gain"],
                   help="B5's online correction, every run (E2; gain: the turn gain alone, Task 10c)")
    r.add_argument("--profile-timing", action="store_true", help="wall times (plant, scans, B5) in the summaries (E2)")
    r.add_argument("--routes", choices=["recorded", "lattice"], default="recorded",
                   help="lattice: plan the routes with the Smac Lattice port, no bags (spec 2 Task 8)")
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
