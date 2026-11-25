"""The medium diagnosis's replay matrix (plan docs/superpowers/plans/2026-11-22-medium-diagnosis.md, Task 5 and
Amendment 1): a failed run rerun byte for byte to its fork point, then ONE setting changed (plant_campaign --fork-at,
--fork-set), one cycle (--fork-cycles 1: does this plan become feasible?) or to the end (does the robot then pass?).

  python3 tools/plant/fork_replay.py run --points FORK_POINTS.json --runs BASELINE_DIR --classes C1,C2 \\
      --mode one|closed|control --out DIR [--one ONE_CYCLE_DIR] [--per-cell 5] [--jobs 4] [--extra RUN_DIR ...]
  python3 tools/plant/fork_replay.py report --points FORK_POINTS.json --runs BASELINE_DIR --classes C1,C2 \\
      --one DIR --closed DIR --out FILE.md

`one`: every point x ONE_CYCLE setting at the fork time. `closed`: every point x CLOSED_LOOP setting, plus each
one-cycle setting that made that point's plan feasible. `control`: the control fork (no setting) at the stop, closed
loop (the speed standstill's commands). Every fork run carries --prediction --seed-detail --diagnostics.
"""
import argparse
import json
import statistics
import sys
from collections import defaultdict
from pathlib import Path

import medium_diag as md
import plant_campaign as pc
import plant_stream as ps

# Amendment 1 (checkpoint A, user 2026-11-24): each setting alone against the control, or against its partner
ONE_CYCLE = [("control", {}), ("m040", {"medium_arm_margin": "0.04"}), ("m030", {"medium_arm_margin": "0.03"}),
             ("b300", {"iteration_budget": "300"}), ("it30", {"max_iter": "30"}),
             ("b300full", {"iteration_budget": "300", "full_every_cycle": "1"})]
CLOSED_LOOP = [("control", {}), ("inf6", {"b5_max_infeasible_s": "6"}), ("infoff", {"b5_max_infeasible_s": "-1"})]
PAIRED = {"b300full": "b300"}  # the seeds compared at budget 300, so that the budget does not starve them
DIAGNOSTIC_FLAGS = ["--prediction", "--seed-detail", "--diagnostics"]
INCUMBENTS = ("previous", "hold")


def select(points, classes, per_cell=5):
    """The fork points of `classes`, at most per_cell per cell, in seed order, cells in name order."""
    by_cell = defaultdict(list)
    for p in points:
        if p["class"] in classes:
            by_cell[p["cell"]].append(p)
    return [p for cell in sorted(by_cell) for p in sorted(by_cell[cell], key=lambda p: p["seed"])[:per_cell]]


def fork_runs(points, settings, runs_root, cycles, at="t"):
    """plant_campaign runs (plant_campaign.py's run dicts): each point's original arguments (its summary's), the
    diagnostic flags once, the fork at the point's `at` key ("t": the fork point, "stop_t": the stop) with the
    setting; named <run>-<tag>-1c (cycles 1) or -cl (closed loop)."""
    out = []
    for p in points:
        original = json.loads((Path(runs_root) / p["run"] / "summary.json").read_text())["args"]
        args = list(original) + [f for f in DIAGNOSTIC_FLAGS if f not in original]
        args += ["--fork-at", repr(round(float(p[at]), 6))]
        for tag, setting in settings:
            a = list(args)
            for key, value in setting.items():
                a += ["--fork-set", f"{key}={value}"]
            if cycles:
                a += ["--fork-cycles", str(cycles)]
            out.append({"name": f"{p['run']}-{tag}-{'1c' if cycles else 'cl'}", "point": p["run"], "tag": tag,
                        "args": a, "b5_config": None, "tracking_model": None})
    return out


def _summary(d):
    try:
        return json.loads((Path(d) / "summary.json").read_text())
    except (OSError, ValueError):
        return None


def one_cycle_results(points, settings, root):
    """Per point and setting, the forked plan: F2's status, whether its winner is feasible, the incumbent's collision
    violation and deepest row, the cycle's wall time. A cycle the 10 s deadline cut is excluded (not reproducible)."""
    out = []
    for p in points:
        for tag, _ in settings:
            d = Path(root) / f"{p['run']}-{tag}-1c"
            r = {"run": p["run"], "cell": p["cell"], "class": p["class"], "tag": tag, "feasible": None, "status": None,
                 "collision_m": None, "deepest": None, "wall_s": None, "excluded": None}
            s = _summary(d)
            if s is None or "fork" not in s or s["fork"].get("applied_t") is None:
                out.append({**r, "excluded": "missing"})
                continue
            fork = s["fork"]
            r["wall_s"] = fork["cycle_wall_s"][0] if fork["cycle_wall_s"] else None
            if fork.get("deadline_cut_cycles", 0) > 0:
                out.append({**r, "excluded": "deadline_cut"})
                continue
            cycles = [c for c in md.cycles(md.frames_of(d / "run.mmlog")) if c["t"] >= fork["applied_t"] - 1e-9]
            if not cycles:
                out.append({**r, "excluded": "no_plan"})
                continue
            c = cycles[0]
            seeds = {x["name"]: x for x in c["seeds"]}
            winner = seeds.get(c["seed"])
            inc = next((seeds[n] for n in INCUMBENTS if n in seeds), winner)
            r.update(status=c["status"], feasible=c["status"] == "ok" and bool(winner and winner["feasible"]))
            if inc:
                r["collision_m"] = inc["collision_m"]
                r["deepest"] = f"{inc['deepest_link']}/{inc['deepest_obstacle']}" if inc.get("deepest_obstacle") else ""
            out.append(r)
    return out


def closed_loop_followups(results):
    """Per point, the one-cycle settings (not the control) that made its plan feasible."""
    follow = defaultdict(list)
    for r in results:
        if r["tag"] != "control" and r["feasible"]:
            follow[r["run"]].append(r["tag"])
    return dict(follow)


def reproduces(original, forked):
    """A closed-loop control fork reproduces its original run: the same outcome, time, clearance and contacts."""
    return all(original.get(k) == forked.get(k) for k in ("reached", "time_s", "min_clearance_m", "plant_contacts"))


def closed_loop_results(points, tags, root, runs_root):
    """Per point and tag, the closed-loop fork's outcome; the control's reproduction of the original run."""
    out = []
    for p in points:
        original = _summary(Path(runs_root) / p["run"])
        for tag in tags.get(p["run"], []):
            s = _summary(Path(root) / f"{p['run']}-{tag}-cl")
            r = {"run": p["run"], "cell": p["cell"], "class": p["class"], "tag": tag}
            if s is None:
                out.append({**r, "excluded": "missing"})
                continue
            fork = s.get("fork", {})
            walls = fork.get("cycle_wall_s") or []
            out.append({**r, "excluded": "deadline_cut" if fork.get("deadline_cut_cycles", 0) else None,
                        "reached": pc.succeeded(s), "stalled": s.get("stalled"), "time_s": s.get("time_s"),
                        "min_clearance_m": s.get("min_clearance_m"), "plant_contacts": s.get("plant_contacts"),
                        "wall_median_s": statistics.median(walls) if walls else None,
                        "reproduces": reproduces(original, s) if tag == "control" and original else None})
    return out


def report(one, closed):
    """Markdown: the one-cycle matrix (feasible forks per setting and class), the closed-loop outcomes, the control's
    reproduction and the wall times against the 0.4 s period."""
    lines = ["# Medium diagnosis: the replay matrix", "", "## One cycle at the fork", "",
             "| Setting | Points | Feasible | Excluded | Incumbent collision median (mm) | Wall median (s) |",
             "|---|---:|---:|---:|---:|---:|"]
    tags = list(dict.fromkeys(r["tag"] for r in one))
    for tag in tags:
        rs = [r for r in one if r["tag"] == tag]
        used = [r for r in rs if not r["excluded"]]
        coll = [r["collision_m"] for r in used if r["collision_m"] is not None]
        walls = [r["wall_s"] for r in rs if r["wall_s"] is not None]
        lines.append(f"| {tag} | {len(rs)} | {sum(bool(r['feasible']) for r in used)}/{len(used)} | "
                     f"{len(rs) - len(used)} | {1000 * statistics.median(coll):.1f} | "
                     f"{statistics.median(walls):.3f} |" if coll and walls else
                     f"| {tag} | {len(rs)} | {sum(bool(r['feasible']) for r in used)}/{len(used)} | {len(rs) - len(used)} | - | - |")
    lines += ["", "## Closed loop from the fork", "",
              "| Setting | Runs | Reached | Stalled | Contacts > 0 | Min clearance median (m) | Wall median (s) |",
              "|---|---:|---:|---:|---:|---:|---:|"]
    for tag in dict.fromkeys(r["tag"] for r in closed):
        rs = [r for r in closed if r["tag"] == tag and not r["excluded"]]
        cl = [r["min_clearance_m"] for r in rs if r.get("min_clearance_m") is not None]
        wl = [r["wall_median_s"] for r in rs if r.get("wall_median_s") is not None]
        lines.append(f"| {tag} | {len(rs)} | {sum(bool(r['reached']) for r in rs)} | {sum(bool(r['stalled']) for r in rs)} | "
                     f"{sum(1 for r in rs if (r['plant_contacts'] or 0) > 0)} | "
                     f"{statistics.median(cl):.3f} |" if cl else f"| {tag} | {len(rs)} | - | - | - | - |")
        if wl:
            lines[-1] += f" {statistics.median(wl):.3f} |"
        elif cl:
            lines[-1] += " - |"
    controls = [r for r in closed if r["tag"] == "control" and r.get("reproduces") is not None]
    if controls:
        lines += ["", f"Control forks reproducing their original run: {sum(r['reproduces'] for r in controls)}/{len(controls)}."]
    return "\n".join(lines) + "\n"


def _pending(runs, out):
    """Runs without a summary for the same arguments under out."""
    def done(r):
        s = _summary(Path(out) / r["name"])
        return s is not None and s.get("args") == r["args"]
    return [r for r in runs if not done(r)]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="step", required=True)
    for name in ("run", "report"):
        a = sub.add_parser(name)
        a.add_argument("--points", required=True)
        a.add_argument("--runs", required=True)
        a.add_argument("--classes", required=True)
        a.add_argument("--per-cell", type=int, default=5)
        a.add_argument("--extra", nargs="*", default=[], help="fork points.json-style run names to add (any class)")
        a.add_argument("--extra-runs", help="the directory of the --extra runs (default: --runs)")
        a.add_argument("--out", required=True)
        if name == "run":
            a.add_argument("--mode", choices=["one", "closed", "control"], required=True)
            a.add_argument("--one", help="the one-cycle directory (closed: its feasible settings follow up)")
            a.add_argument("--jobs", type=int, default=4)
        else:
            a.add_argument("--one", required=True)
            a.add_argument("--closed", required=True)
    args = ap.parse_args(argv)
    all_points = json.loads(Path(args.points).read_text())
    pts = select(all_points, set(args.classes.split(",")), args.per_cell)
    if args.step == "run":
        if args.mode == "one":
            runs = fork_runs(pts, ONE_CYCLE, args.runs, cycles=1)
        elif args.mode == "closed":
            follow = closed_loop_followups(one_cycle_results(pts, ONE_CYCLE, args.one)) if args.one else {}
            runs = fork_runs(pts, CLOSED_LOOP, args.runs, cycles=0)
            for p in pts:
                extra = [(t, s) for t, s in ONE_CYCLE if t in follow.get(p["run"], [])]
                runs += fork_runs([p], extra, args.runs, cycles=0) if extra else []
        else:
            runs = fork_runs(pts, [("control", {})], args.runs, cycles=0, at="stop_t")
            extra_root = args.extra_runs or args.runs
            for spec in args.extra:  # RUN_NAME:STOP_T, e.g. the door run
                name, stop = spec.rsplit(":", 1)
                runs += fork_runs([{"run": name, "stop_t": float(stop)}], [("control", {})], extra_root, cycles=0, at="stop_t")
        left = _pending(runs, args.out)
        print(f"{len(runs)} fork runs, {len(runs) - len(left)} already done", flush=True)
        return 1 if pc.run_all(left, args.out, args.jobs) else 0
    one = one_cycle_results(pts, ONE_CYCLE, args.one)
    tags = {p["run"]: [t for t, _ in CLOSED_LOOP] + closed_loop_followups(one).get(p["run"], []) for p in pts}
    closed = closed_loop_results(pts, tags, args.closed, args.runs)
    Path(args.out).write_text(report(one, closed))
    Path(args.out).with_suffix(".json").write_text(json.dumps({"one": one, "closed": closed}, indent=1) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
