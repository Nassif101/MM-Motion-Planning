"""Twin runs on the MuJoCo plant: where two runs that differ at rounding level part (E1 plan Task 4; spec
docs/superpowers/specs/2026-11-08-e1-gate-robustness-design.md Section 3.3).

    python3 tools/plant/divergence.py pair LOG_A LOG_B
    python3 tools/plant/divergence.py twin --corpus FILE --cell C --kind f2|b5 --seed S --out DIR [--b5-config F]
    python3 tools/plant/divergence.py ensemble --corpus FILE --seeds N --out DIR [--kinds f2,b5] [--b5-config F]
        [--profile P] [--tag T] [--jobs J]

`twin` writes DIR/<run>.pair.json. A twin is the same seed (the same start perturbation and noise stream) with EPS rad (--nudge, default 1e-12) added to
the start yaw. `pair` reports the first cycle at which a discrete decision differs, the state distance there and at the
end, both sides' context and outcomes. `ensemble` runs N twin pairs per cell and kind and writes DIR/flips.json and
DIR/flips.md: per cell the flip rate (the share of pairs whose outcomes differ: reached against not, contact against
none) and a histogram of the first diverging decision. Standard library only (the host's Python 3.9).
"""
import argparse
import json
import math
import sys
from collections import Counter, OrderedDict
from pathlib import Path

import plant_campaign as pc
import plant_stream as ps

# the discrete decisions compared cycle by cycle, in this order; plan_* are the latest Plans frame's fields
DECISIONS = ("status", "gate_unsafe", "dwell", "fallback", "plan_seed", "plan_homotopy", "plan_status", "plan_waiting")
# Hello fields that must agree between twins: the run, the layer (medium), the models and the settings (the nudge and
# the provenance may differ); config fields absent from older logs take their default (stall_s: no detector before
# 5bd4300, the baseline replicas)
PAIR_FIELDS = ("scenario", "bag", "seed", "mode", "medium", "profile_sha256", "robot_model_sha256", "site_model_sha256")
PAIR_CONFIG = {"medium_config": None, "timeout_s": None, "iteration_budget": None, "perturb_start": None, "lidar": None,
               "stall_s": 0.0}
NUDGE = 1e-12
GATE_TOLERANCE_M = 0.005  # the context's allowance: the base rows' tolerance (wholebody_mpc.yaml)
INSIDE_ALLOWANCE_M = 1e-4


def decision_stream(log_path):
    """(hello, records): one record per State frame with its decision fields, `gate`, the true pose (`pose`, 9 floats)
    and the latest Plans frame's fields (`plan_*`, absent before the first plan and in schema 1 for the trace fields)."""
    log = ps.RunLog(log_path)
    records, plan = [], {}
    for kind, header, blocks in log.frames():
        if kind == ps.PLANS:
            plan = {"plan_status": header.get("status"), "plan_homotopy": header.get("homotopy")}
            if "seed" in header:  # schema 2
                plan.update({"plan_seed": header["seed"], "plan_waiting": header.get("waiting"),
                             "plan_seeds": header.get("seeds")})
        elif kind == ps.STATE:
            gate = header.get("gate")
            rec = {"t": header.get("t"), "status": header.get("status"), "fallback": header.get("fallback"),
                   "gate": gate, "pose": blocks[0] if blocks else None}
            if gate:  # absent before 93a4d91 (the baseline replicas): skipped then, as a schema-1 trace field
                rec["gate_unsafe"] = int(gate[2])
            if "dwell" in header:
                rec["dwell"] = header["dwell"]
            rec.update(plan)
            records.append(rec)
    return log.header, records


def check_pair(a, b):
    """ValueError naming the Hello field in which two logs differ beyond a twin's nudge."""
    for field in PAIR_FIELDS:
        if a.get(field) != b.get(field):
            raise ValueError(f"not twins: {field} differs ({a.get(field)!r} against {b.get(field)!r})")
    ca, cb = a.get("config", {}), b.get("config", {})
    for field, default in PAIR_CONFIG.items():
        va, vb = ca.get(field, default), cb.get(field, default)
        if va != vb:
            raise ValueError(f"not twins: config.{field} differs ({va!r} against {vb!r})")
    sa, sb = ca.get("b5_config_sha256"), cb.get("b5_config_sha256")
    if sa is not None and sb is not None and sa != sb:
        raise ValueError(f"not twins: config.b5_config_sha256 differs ({sa} against {sb})")


def outcome(log_path):
    """reached / end_status / plant_contacts from the summary beside a log; None when the run did not finish."""
    try:
        s = json.loads((Path(log_path).parent / "summary.json").read_text())
    except (OSError, ValueError):
        return None
    return {"reached": s.get("reached"), "end_status": s.get("end_status"), "plant_contacts": s.get("plant_contacts", 0)}


def flipped(a, b):
    """The outcomes differ: one reached and the other did not, or one touched and the other did not."""
    if a is None or b is None:
        return False
    return bool(a.get("reached")) != bool(b.get("reached")) or (a.get("plant_contacts", 0) > 0) != (b.get("plant_contacts", 0) > 0)


def distance(ra, rb, cycle=None):
    pa, pb = ra.get("pose"), rb.get("pose")
    if not pa or not pb:
        return None
    yaw = abs(math.remainder(pa[2] - pb[2], 2 * math.pi))
    out = {"base_m": math.hypot(pa[0] - pb[0], pa[1] - pb[1]), "yaw_rad": yaw,
           "joints_rad": max(abs(x - y) for x, y in zip(pa[3:9], pb[3:9]))}
    if cycle is not None:
        out["cycle"] = cycle
    return out


def context(rec, hello):
    gate = rec.get("gate") or [0.0, 0.0, 0]
    rule = hello.get("config", {}).get("gate_rule", "increment")
    v = gate[1]
    allowance = GATE_TOLERANCE_M if not v > 0 else (
        max(GATE_TOLERANCE_M, v + INSIDE_ALLOWANCE_M) if rule == "absolute" else v + INSIDE_ALLOWANCE_M)
    return {"t": rec.get("t"), "status": rec.get("status"), "gate": gate, "gate_rule": rule,
            "base_allowance_m": allowance, "dwell": rec.get("dwell"), "fallback": rec.get("fallback"),
            "seed": rec.get("plan_seed"), "seeds": rec.get("plan_seeds"), "waiting": rec.get("plan_waiting")}


def compare(log_a, log_b):
    ha, a = decision_stream(log_a)
    hb, b = decision_stream(log_b)
    check_pair(ha, hb)
    have_a = {d for d in DECISIONS if any(d in r for r in a)}
    have_b = {d for d in DECISIONS if any(d in r for r in b)}
    common = [d for d in DECISIONS if d in have_a and d in have_b]
    out = {"first": None, "distance_at_first": None, "distance_end": None, "truncated": None,
           "skipped": sorted(have_a ^ have_b), "outcomes": {"a": outcome(log_a), "b": outcome(log_b)}}
    n = min(len(a), len(b))
    for k in range(n):
        for d in common:
            if a[k].get(d) != b[k].get(d):
                out["first"] = {"cycle": k, "t": a[k].get("t"), "t_b": b[k].get("t"), "decision": d,
                                "t_mismatch": a[k].get("t") != b[k].get("t"), "a": a[k].get(d), "b": b[k].get(d),
                                "context_a": context(a[k], ha), "context_b": context(b[k], hb)}
                out["distance_at_first"] = distance(a[k], b[k])
                break
        if out["first"]:
            break
    if len(a) != len(b):
        side, short = ("a", a) if len(a) < len(b) else ("b", b)
        if out["outcomes"][side] is None:  # the shorter run did not finish: a dead or interrupted run (incomplete)
            out["truncated"] = {"side": side, "cycle": len(short)}
        elif out["first"] is None:  # it finished earlier: the two part where it ended
            out["first"] = {"cycle": n, "decision": "end", "a": len(a), "b": len(b)}
    if n:  # the last cycle both runs have
        out["distance_end"] = distance(a[n - 1], b[n - 1], n - 1)
    return out


def complete(r):
    """A pair both of whose runs finished (a truncated or unfinished run is incomplete, outside the flip rate)."""
    return not r.get("truncated") and all(r.get("outcomes", {}).get(s, True) is not None for s in ("a", "b"))


def ensemble_report(results):
    """Per cell and kind: complete pairs, flips, flip rate and incomplete pairs; per kind the first-divergence histogram
    ("none" for pairs that never part); the pooled rate."""
    cells, hist = OrderedDict(), OrderedDict()
    for r in results:
        row = cells.setdefault((r["cell"], r["kind"]),
                               {"cell": r["cell"], "kind": r["kind"], "pairs": 0, "flips": 0, "incomplete": 0})
        if not complete(r):
            row["incomplete"] += 1
            continue
        row["pairs"] += 1
        row["flips"] += 1 if r["flipped"] else 0
        hist.setdefault(r["kind"], Counter())[r["first"]["decision"] if r["first"] else "none"] += 1
    for row in cells.values():
        row["flip_rate"] = row["flips"] / row["pairs"] if row["pairs"] else 0.0
    done = [r for r in results if complete(r)]
    flips = sum(1 for r in done if r["flipped"])
    return {"cells": list(cells.values()), "first_divergence": {k: dict(v) for k, v in hist.items()},
            "pooled": {"pairs": len(done), "flips": flips, "flip_rate": flips / len(done) if done else 0.0,
                       "incomplete": len(results) - len(done)}}


def ensemble_markdown(report):
    lines = ["| cell | kind | pairs | flips | flip rate | incomplete |", "|---|---|---|---|---|---|"]
    for row in report["cells"]:
        lines.append(f"| {row['cell']} | {row['kind']} | {row['pairs']} | {row['flips']} | {row['flip_rate']:.2f} | "
                     f"{row['incomplete']} |")
    p = report["pooled"]
    lines += ["", f"Pooled: {p['flips']} of {p['pairs']} complete pairs flipped ({p['flip_rate']:.2f}); "
                  f"{p['incomplete']} incomplete.", "",
              "| kind | first diverging decision | pairs |", "|---|---|---|"]
    for kind, counts in report["first_divergence"].items():
        for decision, count in sorted(counts.items(), key=lambda kv: -kv[1]):
            lines.append(f"| {kind} | {decision} | {count} |")
    return "\n".join(lines) + "\n"


def twin_runs(cells, seeds, args):
    """The plain runs and their twins, in the same order (runs_of's loops; the twin's name is the plain one's plus -n)."""
    kw = dict(mode=args.mode, kinds=tuple(args.kinds.split(",")), profile=args.profile, tag=args.tag,
              b5_config=args.b5_config)
    try:
        plain = pc.runs_of(cells, seeds, **kw)
        twins = pc.runs_of(cells, seeds, start_nudge=args.nudge, **kw)
    except ValueError as e:  # a variant without a tag, a path outside the repository
        raise SystemExit(str(e)) from None
    assert all(t["name"] == p["name"] + "-n" for p, t in zip(plain, twins))
    return plain, twins


def run_pairs(plain, twins, out, args):
    todo = pc.pending(plain + twins, pc.ROOT / out)
    print(f"{len(plain)} pairs, {len(plain) + len(twins) - len(todo)} runs already done", flush=True)
    failed = pc.run_all(todo, out, args.jobs, args.container)
    if failed:
        print(f"{failed} runs failed (see their .log under {out})", file=sys.stderr)
    results = []
    for p, t in zip(plain, twins):
        la, lb = pc.ROOT / out / p["name"] / "run.mmlog", pc.ROOT / out / t["name"] / "run.mmlog"
        try:
            r = compare(la, lb)
        except (OSError, ValueError) as e:
            print(f"{p['name']}: not compared ({e})", file=sys.stderr)
            continue
        r.update({"cell": p["cell"], "kind": p["kind"], "seed": p["seed"], "name": p["name"],
                  "flipped": flipped(r["outcomes"]["a"], r["outcomes"]["b"])})
        results.append(r)
    return results


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="step", required=True)
    p = sub.add_parser("pair")
    p.add_argument("log_a")
    p.add_argument("log_b")
    for name in ("twin", "ensemble"):
        s = sub.add_parser(name)
        s.add_argument("--corpus", required=True)
        s.add_argument("--out", required=True)
        s.add_argument("--mode", choices=["ideal", "realistic"], default="realistic")
        s.add_argument("--kinds", default="f2,b5")
        s.add_argument("--b5-config")
        s.add_argument("--profile")
        s.add_argument("--tag")
        s.add_argument("--nudge", type=float, default=NUDGE)
        s.add_argument("--jobs", type=int, default=4)
        s.add_argument("--container", default=pc.CONTAINER)
        if name == "twin":
            s.add_argument("--cell", required=True)
            s.add_argument("--kind", choices=["f2", "b5"], required=True)
            s.add_argument("--seed", type=int, required=True)
        else:
            s.add_argument("--seeds", type=int, required=True)
            s.add_argument("--cells", help="comma-separated corpus cells (default: all)")
    args = ap.parse_args(argv)
    if args.step == "pair":
        print(json.dumps(compare(args.log_a, args.log_b), indent=1))
        return 0
    cells = pc.load_corpus(args.corpus)
    out = Path(args.out)
    if args.step == "twin":
        cells = [c for c in cells if c["cell"] == args.cell]
        if not cells:
            raise SystemExit(f"{args.cell} is not in {args.corpus}")
        args.kinds = args.kind
        plain, twins = twin_runs(cells, args.seed, args)
        keep = [i for i, r in enumerate(plain) if r["seed"] == args.seed]
        results = run_pairs([plain[i] for i in keep], [twins[i] for i in keep], out, args)
        for r in results:  # the report beside the runs (stdout also carries the runs' progress lines)
            (pc.ROOT / out / f"{r['name']}.pair.json").write_text(json.dumps(r, indent=1))
            print(f"{r['name']}: report in {out / (r['name'] + '.pair.json')}")
            print(json.dumps(r, indent=1))
        return 0 if results else 1
    if args.cells:
        wanted = set(args.cells.split(","))
        cells = [c for c in cells if c["cell"] in wanted]
    plain, twins = twin_runs(cells, args.seeds, args)
    results = run_pairs(plain, twins, out, args)
    report = ensemble_report(results)
    (pc.ROOT / out / "flips.json").write_text(json.dumps({"report": report, "pairs": results}, indent=1))
    (pc.ROOT / out / "flips.md").write_text(ensemble_markdown(report))
    print(ensemble_markdown(report))
    return 0 if results and report["pooled"]["incomplete"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
