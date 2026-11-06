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
