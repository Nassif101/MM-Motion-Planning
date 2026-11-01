#!/usr/bin/env python3
"""The Phase 5A medium-layer bench (plan Task 9, spec Section 6.1): run medium_bench over a corpus and variants, and
summarize the cycle times, the split, the SCP convergence and the plan status.

    python3 tools/medium_bench.py sweep --corpus FILE --variants NAME[,NAME] --out DIR [--spacing-s S]
    python3 tools/medium_bench.py summarize --out DIR [--table FILE]
    python3 tools/medium_bench.py closed --corpus FILE --out DIR [--b5-only] [--config FILE] [--iteration-budget N]
    python3 tools/medium_bench.py gate --corpus FILE --out DIR [--no-timing]
    python3 tools/medium_bench.py campaign --since UTC [--until UTC] [--runs-per-cell N] [--table FILE]

closed (plan Task 12) runs medium_bench closed on every cell (B5 at 50 Hz on the kinematic plant following the medium
layer; --b5-only the frozen B5 alone) into DIR/closed[_b5]/<cell>.json; gate prints each cell's outcome and checks the
kinematic gate (gate()), exit code 0 only if every criterion holds.

campaign (plan Task 13 Step 3) scores live campaign A runs (experiment_runs/<run>, summary.json utc in --since ..
--until, the cells of CAMPAIGN_A): it runs tools/medium_run_stats.py in the container for runs without
medium_stats.json, prints one row per run and checks spec Section 7.1 on the F2 runs (campaign()); B5's runs are
reported, not judged. Exit code 0 only if every criterion holds.

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


# The swing-side cells (spec Section 7.1's "Swing side" row): no switch after the first choice there.
SWING_CELLS = ("v2_gate_right_swing_nav", "v2_door_105_nav")
GATE_P99_S = 0.15


def gate(records, cells=None, timing=True):
    """The kinematic gate over closed-loop records (one per cell): (ok, failures). Every cell reaches its goal, executes
    no plan with collision slack, the medium layer's cycle p99 over every cycle of every cell is at most GATE_P99_S
    (`timing`; a run under an iteration budget judges behaviour only), and the swing-side cells switch side no more
    after the first choice; `cells` names the cells that must be there."""
    failures = []
    seen = {r["cell"] for r in records}
    for cell in cells or []:
        if cell not in seen:
            failures.append(f"{cell}: missing from the run")
    solve = []
    for r in records:
        if not r["reached"]:
            failures.append(f"{r['cell']}: did not reach its goal ({r.get('end_status')})")
        if r["executed_slack_plans"] > 0:
            failures.append(f"{r['cell']}: {r['executed_slack_plans']} executed plans with collision slack")
        if r["cell"] in SWING_CELLS and r["homotopy_switches_after_first"] > 0:
            failures.append(f"{r['cell']}: {r['homotopy_switches_after_first']} swing-side switch after the first choice")
        if not timing:
            continue
        solve += r["medium_solve_s"]
        cell_p99 = percentile(r["medium_solve_s"], 99) if r["medium_solve_s"] else 0.0
        if cell_p99 > GATE_P99_S:
            failures.append(f"{r['cell']}: medium p99 {cell_p99:.3f} s above {GATE_P99_S} s")
    if solve and percentile(solve, 99) > GATE_P99_S and not any("p99" in f for f in failures):
        failures.append(f"all cells: medium p99 {percentile(solve, 99):.3f} s above {GATE_P99_S} s")
    return not failures, failures


# Spec Section 7.1: campaign A's cells by must-fix row, and the criteria of every F2 run.
CAMPAIGN_A = {"v2_room_r4_nav": "door", "v2_store_room_nav": "door", "v2_gate_right_swing_nav": "swing",
              "v2_door_105_nav": "swing", "v2_low_beam_123_nav": "low", "v2_low_beam_130_nav": "low",
              "v2_door_then_beam_nav": "sequencing"}
LIVE_CLEARANCE_M = 0.045
LIVE_MPC_P99_S = 0.010
LIVE_MISS_SHARE = 0.01
LIVE_FALLBACK_SHARE = 0.01
LIVE_TRACKING_RAD = 0.10
SHOULDER = 1  # shoulder_lift_joint in the model's joint order


def live_record(summary, stats):
    """One live run (summary.json and medium_stats.json) as a flat record."""
    mpc = summary.get("mpc") or {}
    medium = summary.get("medium") or {}
    fallbacks = medium.get("fallback_cycles") or {}
    offset = mpc.get("arm_reference_offset_rad") or {}
    torque = (stats or {}).get("torque") or {}
    ratios = torque.get("ratio") or [None] * 6
    contact = bool((summary.get("contacts") or {}).get("contact"))
    return {
        "run": summary.get("utc"), "cell": summary["scenario"]["name"], "layer": medium.get("layer") or "none",
        "status": summary["task"].get("status"), "contact": contact,
        "success": summary["task"].get("status") == "succeeded" and not contact,
        "time_s": summary["task"].get("time_s"), "clearance_3d_m": summary.get("clearance_3d_m"),
        "clearance_self_m": summary.get("clearance_self_m"),
        "mpc_p99_s": (mpc.get("total_s") or {}).get("p99"), "mpc_cycles": mpc.get("cycles") or 0,
        "deadline_misses": mpc.get("deadline_misses") or 0,
        "following_cycles": medium.get("following_cycles"), "fallback_cycles": sum(fallbacks.values()),
        "fallback_reasons": fallbacks, "valid": medium.get("valid"),
        "arm_offset_max_rad": offset.get("max"), "arm_offset_p99_rad": offset.get("p99"),
        "medium_p99_s": ((stats or {}).get("total_s") or {}).get("p99"), "medium_cycles": (stats or {}).get("cycles"),
        "capped_cycles": (stats or {}).get("capped_cycles"), "side_sequence": (stats or {}).get("side_sequence") or [],
        "switches": (stats or {}).get("homotopy_switches_after_first"),
        "shoulder_torque_ratio": ratios[SHOULDER],
        "torque_ratio_max": max((r for r in ratios if r is not None), default=None),
    }


def run_criteria(r):
    """An F2 run's criteria of spec Section 7.1 (every F2 run): True passes, False fails, None not measured."""
    def at_most(value, bound):
        return None if value is None else value <= bound
    cycles = r["mpc_cycles"]
    return {
        "success": r["success"],
        "no contact": not r["contact"],
        "3D clearance": None if r["clearance_3d_m"] is None else r["clearance_3d_m"] >= LIVE_CLEARANCE_M,
        "MPC p99": at_most(r["mpc_p99_s"], LIVE_MPC_P99_S),
        "MPC misses": r["deadline_misses"] < LIVE_MISS_SHARE * cycles if cycles else None,
        "medium p99": at_most(r["medium_p99_s"], GATE_P99_S),
        "fallback share": r["fallback_cycles"] < LIVE_FALLBACK_SHARE * cycles if cycles else None,
        "arm tracking": at_most(r["arm_offset_max_rad"], LIVE_TRACKING_RAD),
        "followed the medium layer": r["valid"],
    }


def campaign(records, runs_per_cell=3):
    """Spec Section 7.1 over live records (live_record): (ok, failures). Per cell runs_per_cell F2 runs, all successes;
    the swing cells without a side switch after the first choice, the low-pass cells with the peak static shoulder
    torque within the limit; every F2 run meets run_criteria (a criterion not measured fails). B5's runs are not
    judged."""
    failures = []
    f2 = [r for r in records if r["layer"] != "none"]
    for cell, row in CAMPAIGN_A.items():
        runs = [r for r in f2 if r["cell"] == cell]
        if len(runs) != runs_per_cell:
            failures.append(f"{cell}: {len(runs)} F2 runs, {runs_per_cell} expected")
        wins = sum(r["success"] for r in runs)
        if wins < len(runs):
            failures.append(f"{cell}: {wins}/{len(runs)} F2 successes")
        for r in runs:
            if row == "swing" and r["switches"] != 0:
                failures.append(f"{cell} {r['run']}: {r['switches']} side switches after the first choice "
                                f"({' > '.join(r['side_sequence'])})")
            ratio = r["shoulder_torque_ratio"]
            if row == "low" and (ratio is None or ratio > 1.0):
                failures.append(f"{cell} {r['run']}: peak static shoulder torque at {ratio} of the limit")
    for r in f2:
        for name, ok in run_criteria(r).items():
            if name != "success" and ok is not True:
                failures.append(f"{r['cell']} {r['run']}: {name} " + ("not measured" if ok is None else "failed"))
    return not failures, failures


def campaign_table(records):
    def num(value, form):
        return "-" if value is None else format(value, form)
    lines = ["| Cell | Layer | Run | Result | Time (s) | 3D clearance (m) | MPC p99 (ms) | Misses | Medium p99 (s) | "
             "Capped | Fallback (cycles) | Arm offset max (rad) | Sides | Switches | Shoulder torque / limit |",
             "|---|---|---|---|---:|---:|---:|---:|---:|---:|---|---:|---|---:|---:|"]
    for r in sorted(records, key=lambda r: (list(CAMPAIGN_A).index(r["cell"]), r["layer"] == "none", r["run"])):
        result = "success" if r["success"] else ("contact" if r["contact"] else r["status"])
        reasons = ", ".join(f"{k} {v}" for k, v in sorted(r["fallback_reasons"].items()))
        fallback = "-" if r["layer"] == "none" else f"{r['fallback_cycles']}" + (f" ({reasons})" if reasons else "")
        lines.append(f"| {r['cell']} | {'F2' if r['layer'] != 'none' else 'B5'} | {r['run']} | {result} | "
                     f"{num(r['time_s'], '.1f')} | {num(r['clearance_3d_m'], '.4f')} | "
                     f"{num(r['mpc_p99_s'] and r['mpc_p99_s'] * 1e3, '.1f')} | {r['deadline_misses']} | "
                     f"{num(r['medium_p99_s'], '.3f')} | {num(r['capped_cycles'], 'd')} | {fallback} | "
                     f"{num(r['arm_offset_max_rad'], '.3f')} | {' > '.join(r['side_sequence']) or '-'} | "
                     f"{num(r['switches'], 'd')} | {num(r['shoulder_torque_ratio'], '.2f')} |")
    return "\n".join(lines)


def campaign_runs(since, until=None):
    """The run directories of the campaign cells with summary.json utc in since .. until (inclusive)."""
    runs = []
    for summary in sorted((ROOT / "experiment_runs").glob("*/summary.json")):
        s = json.loads(summary.read_text())
        if s.get("scenario", {}).get("name") in CAMPAIGN_A and s.get("utc", "") >= since and (
                until is None or s["utc"] <= until):
            runs.append(summary.parent)
    return runs


def run_stats(runs):
    """tools/medium_run_stats.py in the container for the runs without medium_stats.json."""
    missing = [f"{WORKSPACE}/{run.relative_to(ROOT)}" for run in runs if not (run / "medium_stats.json").exists()]
    if missing:
        script = ("source $ROS_WS/install/setup.bash 2>/dev/null; cd " + WORKSPACE + " && python3 tools/medium_run_stats.py "
                  + " ".join(missing))
        subprocess.run(["docker", "exec", CONTAINER, "bash", "-c", script], check=True)


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


def run_closed(cell, out_dir, b5_only=False, config=None, iteration_budget=0):
    kind = "closed_b5" if b5_only else "closed"
    out = Path(out_dir) / kind / f"{cell['cell']}.json"
    (ROOT / out.parent).mkdir(parents=True, exist_ok=True)
    command = [BENCH, "closed", "--scenario", cell["scenario"], "--out", f"{WORKSPACE}/{out}"]
    if not cell.get("synthetic"):
        command += ["--bag", f"{WORKSPACE}/{cell['bag']}"]
    if b5_only:
        command.append("--b5-only")
    if config:
        command += ["--config", f"{WORKSPACE}/{config}"]
    if iteration_budget:
        command += ["--iteration-budget", str(iteration_budget)]
    script = "source $ROS_WS/install/setup.bash 2>/dev/null; cd $ROS_WS && " + " ".join(command)
    subprocess.run(["docker", "exec", CONTAINER, "bash", "-c", script], check=True)


def closed_records(out_dir, kind="closed"):
    records = []
    for f in sorted((ROOT / out_dir / kind).glob("*.json")):
        record = json.loads(f.read_text())["records"][0]
        record["cell"] = f.stem
        records.append(record)
    return records


def closed_table(records):
    lines = ["| Cell | Reached | Time (s) | Executed slack plans | Min clearance (m) | Following | Fallbacks | "
             "Medium p99 (s) | Capped | Sides | Switches | Followed infeasible (cycles / plans) |",
             "|---|---|---:|---:|---:|---:|---|---:|---:|---|---:|---:|"]
    for r in records:
        p99 = percentile(r["medium_solve_s"], 99) if r["medium_solve_s"] else 0.0
        following = r["following_cycles"] / max(r["b5_cycles"], 1)
        clearance = "-" if r["min_clearance_m"] is None else f"{r['min_clearance_m']:.3f}"
        lines.append(f"| {r['cell']} | {'yes' if r['reached'] else 'no (' + r['end_status'] + ')'} | {r['time_s']:.1f} | "
                     f"{r['executed_slack_plans']} | {clearance} | {following:.0%} | {r['fallbacks']} | {p99:.3f} | "
                     f"{r['medium_capped']} | {' > '.join(r.get('side_sequence') or []) or '-'} | "
                     f"{r['homotopy_switches_after_first']} | {r.get('followed_infeasible_cycles', '-')} / "
                     f"{r.get('followed_infeasible_plans', '-')} |")
    return "\n".join(lines)


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
    closed = sub.add_parser("closed")
    closed.add_argument("--corpus", required=True)
    closed.add_argument("--out", required=True)
    closed.add_argument("--b5-only", action="store_true")
    closed.add_argument("--config", default=None, help="a medium.yaml variant inside the repository")
    closed.add_argument("--iteration-budget", type=int, default=0,
                        help="SCP iterations per medium cycle instead of the wall-clock cap (reproducible)")
    gated = sub.add_parser("gate")
    gated.add_argument("--corpus", required=True)
    gated.add_argument("--out", required=True)
    gated.add_argument("--no-timing", action="store_true", help="a budget run: no medium p99 criterion")
    camp = sub.add_parser("campaign")
    camp.add_argument("--since", required=True, help="first run's utc, e.g. 20261101T091500Z")
    camp.add_argument("--until", default=None)
    camp.add_argument("--runs-per-cell", type=int, default=3)
    camp.add_argument("--table", default=None)
    args = parser.parse_args()
    if args.command == "campaign":
        runs = campaign_runs(args.since, args.until)
        run_stats(runs)
        records = [live_record(json.loads((run / "summary.json").read_text()),
                               json.loads((run / "medium_stats.json").read_text())) for run in runs]
        ok, failures = campaign(records, args.runs_per_cell)
        text = campaign_table(records) + "\n\ncampaign: " + ("PASS" if ok else "FAIL\n  " + "\n  ".join(failures))
        print(text)
        if args.table:
            Path(args.table).write_text(text + "\n")
        return 0 if ok else 1
    if args.command == "closed":
        for cell in load_corpus(args.corpus)["cells"]:
            print(f"== closed {cell['cell']}", flush=True)
            run_closed(cell, args.out, args.b5_only, args.config, args.iteration_budget)
        return 0
    if args.command == "gate":
        records = closed_records(args.out)
        print(closed_table(records))
        ok, failures = gate(records, cells=[c["cell"] for c in load_corpus(args.corpus)["cells"]], timing=not args.no_timing)
        print("\ngate: " + ("PASS" if ok else "FAIL\n  " + "\n  ".join(failures)))
        return 0 if ok else 1
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
