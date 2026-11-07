"""Gate R: the campaign replicas on the MuJoCo plant against Unity's campaigns (MuJoCo plant plan Task 11; R as
amended 2026-11-06, Section 6).

R passes when the planner ranking holds on the plant: F2 reaches the goal at least as often as B5 in every cell where it
does on Unity. Every per-cell difference from Unity is listed for investigation (and attribution to a physical or model
cause in replicas.md):
- outcome: the success rates differ by more than 0.25 (Unity has 3 runs a cell, the plant >= 20);
- clearance: the medians of the true 3D clearance differ by more than 0.010 m;
- contact: the plant touches where Unity never did.
Clearances and contacts are reported, not gated. Unity runs where the collision monitor acted are listed.

    python3 tools/plant/compare_campaigns.py --out experiment_runs/plant-replicas --report docs/experiments/mujoco-plant/replicas.md
"""
import argparse
import json
import re
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PHASE5A = ROOT / "docs/experiments/medium-horizon/phase5a"
RATE_GAP = 0.25
CLEARANCE_TOL_M = 0.010


def table_runs(table_md):
    """(cell, layer, run) of every run row of a campaign table (campaign A: Cell | Layer | Run; campaign B: # | Cell |
    Run, all F2)."""
    lines = [l for l in Path(table_md).read_text().splitlines() if l.startswith("|")]
    head = [h.strip() for h in lines[0].strip("|").split("|")]
    out = []
    for line in lines[2:]:
        cols = dict(zip(head, (c.strip() for c in line.strip("|").split("|"))))
        if re.match(r"\d{8}T\d{6}Z$", cols.get("Run", "")):
            out.append((cols["Cell"], cols.get("Layer", "F2"), cols["Run"]))
    return out


def unity_dirs(runs, root=ROOT / "experiment_runs"):
    """The run directories of (cell, layer, run) rows."""
    dirs = []
    for cell, _, run in runs:
        found = sorted(root.glob(f"{run}-{cell}-*"))
        if not found:
            raise FileNotFoundError(f"no run directory for {run} ({cell})")
        dirs.append(found[0])
    return dirs


def unity_records(run_dirs):
    """Unity runs as comparable records: cell, layer (F2 with the medium layer, else B5), reached, time, the true 3D
    clearance, contact, the collision monitor's actions."""
    out = []
    for d in run_dirs:
        s = json.loads((Path(d) / "summary.json").read_text())
        task = s.get("task", {})
        out.append({"cell": s["scenario"]["name"], "layer": "F2" if (s.get("medium") or {}).get("layer") else "B5",
                    "run": Path(d).name, "reached": task.get("status") == "succeeded", "time_s": task.get("time_s"),
                    "clearance_m": s.get("clearance_3d_m"), "contact": bool((s.get("contacts") or {}).get("contact")),
                    "cm_actions": [a["action"] for a in task.get("collision_monitor_activations", [])]})
    return out


def plant_records(out_dir):
    """The plant's runs (plant_campaign summaries; the layer, variant and seed from the run directory's name)."""
    out = []
    for p in sorted(Path(out_dir).glob("*/summary.json")):
        s = json.loads(p.read_text())
        m = re.match(r"(?P<cell>.+)-(?P<kind>f2|b5)-(?P<mode>ideal|realistic)(?:-(?P<variant>[^-]+))?-s(?P<seed>\d+)$", p.parent.name)
        if not m:
            continue
        out.append({"cell": m["cell"], "layer": m["kind"].upper(), "mode": m["mode"], "variant": m["variant"] or "",
                    "seed": int(m["seed"]), "run": p.parent.name, "reached": bool(s.get("reached")),
                    "timed_out": bool(s.get("timed_out")), "time_s": s.get("time_s"),
                    "clearance_m": s.get("plant_min_clearance_m"), "contact": (s.get("plant_contacts") or 0) > 0,
                    "cm_actions": []})
    return out


def _group(records):
    g = {}
    for r in records:
        g.setdefault(r["cell"], {}).setdefault(r["layer"], []).append(r)
    return g


def _stats(rs):
    clear = [r["clearance_m"] for r in rs if r["clearance_m"] is not None]
    times = [r["time_s"] for r in rs if r["reached"] and r["time_s"] is not None]
    return {"runs": len(rs), "reached": sum(r["reached"] for r in rs), "rate": sum(r["reached"] for r in rs) / len(rs),
            "clearance_median_m": statistics.median(clear) if clear else None,
            "time_median_s": statistics.median(times) if times else None, "contacts": sum(r["contact"] for r in rs)}


def agreement(unity, plant):
    """{cells: {cell: {unity: {layer: stats}, plant: {layer: stats}, ranking, unity_ranking}}, ranking_holds,
    to_investigate: [{cell, layer, what, detail}], collision_monitor_runs}."""
    gu, gp = _group(unity), _group(plant)
    cells, todo, holds = {}, [], True
    for cell in sorted(set(gu) & set(gp)):
        su = {layer: _stats(rs) for layer, rs in gu[cell].items()}
        sp = {layer: _stats(rs) for layer, rs in gp[cell].items()}
        row = {"unity": su, "plant": sp, "ranking": None, "unity_ranking": None}
        if {"F2", "B5"} <= set(sp) and {"F2", "B5"} <= set(su):
            row["unity_ranking"] = "F2 >= B5" if su["F2"]["rate"] >= su["B5"]["rate"] else "B5 > F2"
            row["ranking"] = "F2 >= B5" if sp["F2"]["rate"] >= sp["B5"]["rate"] else "B5 > F2"
            if row["unity_ranking"] == "F2 >= B5" and row["ranking"] != "F2 >= B5":
                holds = False
        for layer in sorted(set(su) & set(sp)):
            u, p = su[layer], sp[layer]
            if abs(u["rate"] - p["rate"]) > RATE_GAP + 1e-9:
                todo.append({"cell": cell, "layer": layer, "what": "outcome",
                             "detail": f"Unity {u['reached']}/{u['runs']}, plant {p['reached']}/{p['runs']}"})
            if u["clearance_median_m"] is not None and p["clearance_median_m"] is not None and \
                    abs(u["clearance_median_m"] - p["clearance_median_m"]) > CLEARANCE_TOL_M + 1e-9:
                todo.append({"cell": cell, "layer": layer, "what": "clearance",
                             "detail": f"median Unity {u['clearance_median_m']:.3f} m, plant {p['clearance_median_m']:.3f} m"})
            if p["contacts"] and not u["contacts"]:
                todo.append({"cell": cell, "layer": layer, "what": "contact",
                             "detail": f"plant {p['contacts']} of {p['runs']} runs touch, Unity none"})
        cells[cell] = row
    cm = [r["run"] for r in unity if r["cm_actions"]]
    return {"cells": cells, "ranking_holds": holds, "to_investigate": todo, "collision_monitor_runs": cm}


# --- the report -----------------------------------------------------------------------------------------------------

def unity_campaigns():
    """Unity's campaign A and campaign B V2 runs (both campaigns' runs of a cell together)."""
    runs = table_runs(PHASE5A / "campaign_a_table.md") + table_runs(PHASE5A / "campaign_b_table.md")
    records = unity_records(unity_dirs(runs))
    v2 = {c["cell"] for c in json.loads((ROOT / "docs/experiments/mujoco-plant/corpus_campaign_b.json").read_text())["cells"]}
    v2 |= {c["cell"] for c in json.loads((ROOT / "docs/experiments/mujoco-plant/corpus_campaign_a.json").read_text())["cells"]}
    return [r for r in records if r["cell"] in v2]


def _fmt(x, digits=3):
    return "-" if x is None else f"{x:.{digits}f}"


def agreement_table(a):
    lines = ["| Cell | Layer | Unity reached | Plant reached | Unity time (s) | Plant time (s) | Unity clearance (m) | "
             "Plant clearance (m) | Unity / plant contacts | Ranking Unity / plant |", "|---|---|---|---|---:|---:|---:|---:|---|---|"]
    for cell, row in a["cells"].items():
        for layer in sorted(set(row["unity"]) & set(row["plant"])):
            u, p = row["unity"][layer], row["plant"][layer]
            lines.append(f"| {cell} | {layer} | {u['reached']}/{u['runs']} | {p['reached']}/{p['runs']} | {_fmt(u['time_median_s'], 1)} | "
                         f"{_fmt(p['time_median_s'], 1)} | {_fmt(u['clearance_median_m'])} | {_fmt(p['clearance_median_m'])} | "
                         f"{u['contacts']} / {p['contacts']} | {row['unity_ranking'] or '-'} / {row['ranking'] or '-'} |")
    return lines


def grip_table(plant):
    """Per grip cell and layer: the reached rate at mu 0.4, 0.6 (the profile) and 0.8, and the ranking at each."""
    cells = [c["cell"] for c in json.loads((ROOT / "docs/experiments/mujoco-plant/corpus_grip.json").read_text())["cells"]]
    variants = (("mu0.4", "0.4"), ("", "0.6"), ("mu0.8", "0.8"))
    g = {}
    for r in plant:
        if r["mode"] == "realistic" and r["cell"] in cells:
            g.setdefault((r["cell"], r["layer"], r["variant"]), []).append(r)
    lines = ["| Cell | Layer | " + " | ".join(f"mu {mu}: reached, clearance (m)" for _, mu in variants) + " | Ranking (0.4 / 0.6 / 0.8) |",
             "|---|---|" + "---|" * len(variants) + "---|"]
    for cell in cells:
        rank = []
        for v, _ in variants:
            f2, b5 = g.get((cell, "F2", v), []), g.get((cell, "B5", v), [])
            rank.append("-" if not (f2 and b5) else ("F2 >= B5" if _stats(f2)["rate"] >= _stats(b5)["rate"] else "B5 > F2"))
        for layer in ("F2", "B5"):
            cells_ = []
            for v, _ in variants:
                rs = g.get((cell, layer, v), [])
                st = _stats(rs) if rs else None
                cells_.append("-" if not st else f"{st['reached']}/{st['runs']}, {_fmt(st['clearance_median_m'])}")
            lines.append(f"| {cell} | {layer} | " + " | ".join(cells_) + f" | {' / '.join(rank)} |")
    return lines


def report(out_dir, path):
    unity = unity_campaigns()
    plant = plant_records(out_dir)
    lines = ["# Gate R: campaigns A and B replicated on the MuJoCo plant", "",
             f"Generated by `tools/plant/compare_campaigns.py` from `{Path(out_dir).resolve().relative_to(ROOT)}` (MuJoCo plant "
             "plan Task 11; `replicas.sh`). Unity: Phase 5A campaigns A (7 cells x F2/B5 x 3 runs) and B's V2 cells (F2), "
             "2026-11-01, both campaigns' runs of a cell together. The plant: the recorded route of one Unity run per cell, "
             "20 perturbed seeds per cell and layer in realistic mode (the gate), 5 in ideal mode; F2 at the 2.5 Hz profile "
             "with its latency fixed (the iteration budget). Clearances: the true 3D minimum (Unity's clearance_3d_m, the "
             "plant's ground truth).", "",
             "R (amendment 2026-11-06, Section 6) passes when the plant keeps the planner ranking (F2 reaches the goal at "
             "least as often as B5) in every cell where Unity does; per-cell differences are listed and investigated "
             f"(success rates differing by more than {RATE_GAP}, median clearances by more than {CLEARANCE_TOL_M} m, plant "
             "contacts where Unity had none). Clearances and contacts are reported, not gated.", ""]
    verdicts = {}
    for mode in ("realistic", "ideal"):
        sel = [r for r in plant if r["mode"] == mode and r["variant"] == ""]
        a = agreement(unity, sel)
        verdicts[mode] = a
        lines += [f"## {mode.capitalize()} mode ({len(sel)} plant runs)", "",
                  f"**Ranking holds: {'yes' if a['ranking_holds'] else 'NO'}.** "
                  f"{len(a['to_investigate'])} differences to investigate.", ""] + agreement_table(a) + [""]
        if a["to_investigate"]:
            lines += ["| Cell | Layer | Difference | Detail |", "|---|---|---|---|"]
            lines += [f"| {i['cell']} | {i['layer']} | {i['what']} | {i['detail']} |" for i in a["to_investigate"]] + [""]
    lines += ["## Grip ensemble (CP3b)", "",
              "The tight cells at contact_mu 0.4 and 0.8 against the profile's 0.6 (the effective track kept), realistic, "
              "20 seeds each:", ""] + grip_table(plant) + [""]
    cm = verdicts["realistic"]["collision_monitor_runs"]
    lines += ["## Unity runs where the collision monitor acted", "", ", ".join(cm) if cm else "None.", ""]
    notes = Path(path).with_name("replicas_notes.md")
    if notes.exists():
        lines += [notes.read_text().rstrip(), ""]
    Path(path).write_text("\n".join(lines))
    (Path(out_dir) / "agreement.json").write_text(json.dumps(verdicts, indent=1, default=str) + "\n")
    return verdicts


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--report", required=True)
    args = ap.parse_args()
    v = report(ROOT / args.out, ROOT / args.report)
    for mode, a in v.items():
        print(mode, "ranking holds" if a["ranking_holds"] else "RANKING FLIPS", len(a["to_investigate"]), "to investigate")
    return 0


if __name__ == "__main__":
    sys.exit(main())
