"""The medium-planner diagnosis's log reading (plan docs/superpowers/plans/2026-11-22-medium-diagnosis.md; spec
docs/superpowers/specs/2026-11-22-medium-diagnosis-design.md). Standard library only.

log_fingerprint: a run log's frames after the Hello, hashed, so that two builds' logs compare without the Hello's
provenance (the commit) and its config keys.

Task 2: per run, the medium cycles (Plans) and the plant states (State), where the base stopped (the session's
base progress rule), the fork point (the first cycle of the infeasible streak that runs to the stop, or the stop) and
the class of the layer that stopped; `report` writes a per-cell table and the fork points.

  python3 tools/plant/medium_diag.py report --runs DIR --out DIR2
  python3 tools/plant/medium_diag.py seeds --run RUN_DIR [--from T0] [--to T1]
"""
import argparse
import hashlib
import json
import math
import statistics
import struct
import sys
import zlib
from collections import Counter, defaultdict
from pathlib import Path

import plant_stream as ps


def _raw_frames(data):
    """The encoded frames of a run log's bytes, in order, across its zlib chunks; a truncated or corrupt last chunk ends
    them after the whole ones (as plant_stream.RunLog)."""
    if data[:8] != b"MMRUNLG1":
        raise ValueError("not a run log")
    at, buffer = 8, b""
    while at + 8 <= len(data):
        _, packed_len = struct.unpack_from("<II", data, at)
        at += 8
        if at + packed_len > len(data):
            return
        try:
            buffer += zlib.decompress(data[at:at + packed_len])
        except zlib.error:
            return
        at += packed_len
        pos = 0
        while pos + 4 <= len(buffer):
            (length,) = struct.unpack_from("<I", buffer, pos)
            if pos + 4 + length > len(buffer):
                break
            yield buffer[pos:pos + 4 + length]
            pos += 4 + length
        buffer = buffer[pos:]


def log_fingerprint(path):
    """sha256 (hex) over the encoded bytes of every frame after the Hello, in order."""
    digest = hashlib.sha256()
    frames = _raw_frames(open(path, "rb").read())
    next(frames, None)  # the Hello
    for frame in frames:
        digest.update(frame)
    return digest.hexdigest()


# The Plans seed row's fields: E1's five, then the medium diagnosis's (Task 1, --seed-detail; `interrupted` its review I1;
# `cost_terms`, an object of the solution's cost terms, Task 4b)
BASE_FIELDS = ("name", "merit", "feasible", "progress_m", "merged")
DETAIL_FIELDS = ("slack", "collision_m", "torque", "speed_mps", "hull", "iterations", "converged", "resumed", "homotopy",
                 "deepest_link", "deepest_obstacle", "deepest_distance", "deepest_t", "interrupted", "cost_terms")
FLAGS = {"feasible", "merged", "converged", "resumed", "interrupted"}

# Where the robot stopped: the base's progress rule of the session (session.cpp kStallBaseM, kStallYawRad) on the true
# pose; the joints are left out (Task 2 review C1: a parked base with a slowly drifting arm is stopped)
STALL_BASE_M, STALL_YAW_RAD = 0.05, 0.1
# The infeasible streak runs to the stop, give or take one medium period (B5 can stop a cycle before F2's status turns)
MEDIUM_PERIOD_S = 0.4
# A run whose base still made progress this close to its end timed out moving
MOVING_AT_END_S = 5.0


def frames_of(path):
    """A run log's frames, the Hello first, as (kind, header, blocks); ValueError when it has no readable Hello."""
    log = ps.RunLog(path)
    return [(ps.HELLO, log.header, [])] + list(log.frames())


def seed_row(row):
    """A Plans seed row as a dict; the detail fields None when the row has only the five base fields."""
    out = {}
    for i, key in enumerate(BASE_FIELDS + DETAIL_FIELDS):
        value = row[i] if i < len(row) else None
        out[key] = bool(value) if key in FLAGS and value is not None else value
    return out


def cycles(frames):
    """Per Plans frame: t, status, the winner, full_multistart, waiting and the seed rows (seed_row)."""
    return [{"t": h["t"], "status": h.get("status"), "seed": h.get("seed"), "full_multistart": h.get("full_multistart"),
             "waiting": h.get("waiting"), "seeds": [seed_row(r) for r in h.get("seeds", [])]}
            for kind, h, _ in frames if kind == ps.PLANS]


def states(frames):
    """Per State frame: t, the true pose (x, y, yaw, q), B5's status and fallback, the gate, the execution (lattice)."""
    out = []
    for kind, h, blocks in frames:
        if kind != ps.STATE:
            continue
        pose = blocks[0]
        out.append({"t": h["t"], "x": pose[0], "y": pose[1], "yaw": pose[2], "q": pose[3:9], "status": h.get("status"),
                    "fallback": h.get("fallback", ""), "gate": h.get("gate"), "execution": h.get("execution")})
    return out


def _moved(a, b, base_m, yaw_rad):
    yaw = math.remainder(a["yaw"] - b["yaw"], 2.0 * math.pi)
    return math.hypot(a["x"] - b["x"], a["y"] - b["y"]) > base_m or abs(yaw) > yaw_rad


def stop_state(states, base_m=STALL_BASE_M, yaw_rad=STALL_YAW_RAD):
    """The state at which the base last made progress (its reference moved by more than base_m or yaw_rad); planning
    holds of a lattice run are skipped as the session skips them. None without states."""
    ref = None
    for x in states:
        if x["execution"] == "holding":
            continue
        if ref is None or _moved(x, ref, base_m, yaw_rad):
            ref = x
    return ref


def stall_onset(states, base_m=STALL_BASE_M, yaw_rad=STALL_YAW_RAD):
    """The time the base last made progress (stop_state); None without states."""
    x = stop_state(states, base_m, yaw_rad)
    return x["t"] if x else None


def fork_point(cycles, states, reached):
    """None for a reached run. Else {"kind": "infeasible", "t"}: the first cycle of the streak of cycles that are not ok
    (at least one infeasible) running to the last cycle at or before the stop (+ MEDIUM_PERIOD_S): F2 stayed
    infeasible until the robot stopped (Task 2 review I1: not to the log's end, which runs on past the stall);
    otherwise {"kind": "stall_onset", "t": the stop}."""
    if reached:
        return None
    onset = stall_onset(states)
    upto = [c for c in cycles if onset is None or c["t"] <= onset + MEDIUM_PERIOD_S]
    start = None
    for c in reversed(upto):
        if c["status"] == "ok":
            break
        start = c
    if start is not None and any(c["status"] == "infeasible" for c in upto if c["t"] >= start["t"]):
        return {"kind": "infeasible", "t": start["t"]}
    return {"kind": "stall_onset", "t": onset}


def _share(items, test):
    items = list(items)
    return sum(1 for i in items if test(i)) / len(items) if items else None


def classify(cycles, states, fork):
    """The layer that stopped, from the cycles and states after the stop (majorities); B5's "infeasible" fallback (it
    drives the route on its own) is not a hold (Task 2 review I2).
    reached; timeout_moving (the base progressed within MOVING_AT_END_S of the end); F2 infeasible to the stop:
    infeasible_gate_hold (B5 holding) or infeasible_b5_fallback; F2 mostly ok after the stop: ok_plan_b5_held (B5
    holding), f2_waiting (the plans wait), ok_plan_b5_creep (B5 tracking, the base not moving); otherwise other_stall."""
    if fork is None:
        return "reached"
    stop = stop_state(states)
    if stop is None or states[-1]["t"] - stop["t"] < MOVING_AT_END_S:
        return "timeout_moving"
    after = [c for c in cycles if c["t"] >= stop["t"]]
    later = [x for x in states if x["t"] >= stop["t"]]
    held = (_share(later, lambda x: x["status"] == "holding") or 0.0) >= 0.5
    if fork["kind"] == "infeasible":
        return "infeasible_gate_hold" if held else "infeasible_b5_fallback"
    if (_share(after, lambda c: c["status"] == "ok") or 0.0) >= 0.5:
        if held:
            return "ok_plan_b5_held"
        if (_share(after, lambda c: bool(c["waiting"])) or 0.0) >= 0.5:
            return "f2_waiting"
        return "ok_plan_b5_creep"
    return "other_stall"


def diagnose_run(run_dir):
    """One run directory (run.mmlog, summary.json) as a record: run, cell, seed, reached, class, the fork's kind and t,
    where and when the base stopped and B5's status and fallback there (review M5), the share of cycles not ok before
    the fork. A missing or bad summary, one without `reached`, a log without a Hello or without States: unreadable."""
    run_dir = Path(run_dir)
    record = {"run": run_dir.name, "cell": None, "seed": None, "reached": False, "kind": None, "t": None,
              "stop_t": None, "stop_x": None, "stop_y": None, "b5_status": None, "b5_fallback": None, "not_ok_before": None}
    try:
        summary = json.loads((run_dir / "summary.json").read_text())
        record.update(cell=summary.get("cell"), seed=summary.get("seed"))
        if "reached" not in summary:
            raise ValueError("no reached")
        record["reached"] = bool(summary["reached"])
        frames = frames_of(run_dir / "run.mmlog")
    except (ValueError, OSError, StopIteration):
        return {**record, "class": "unreadable"}
    cs, xs = cycles(frames), states(frames)
    if not cs:
        return {**record, "class": "no_medium"}
    if not xs:
        return {**record, "class": "unreadable"}
    fork = fork_point(cs, xs, record["reached"])
    if fork:
        stop = stop_state(xs)
        record.update(kind=fork["kind"], t=fork["t"], stop_t=stop["t"], stop_x=stop["x"], stop_y=stop["y"],
                      b5_status=stop["status"], b5_fallback=stop["fallback"])
        record["not_ok_before"] = _share([c for c in cs if c["t"] < fork["t"]], lambda c: c["status"] != "ok")
    return {**record, "class": classify(cs, xs, fork)}


def diagnose_runs(root):
    """Every run directory under `root` (those without a summary are unreadable), in name order."""
    return [diagnose_run(d) for d in sorted(Path(root).iterdir()) if d.is_dir()]


def seed_table(cycles, t0, t1):
    """The cycles with t0 <= t < t1: t, status, the winner, full_multistart and the seeds tried by name (seed_row);
    a seed not tried in a cycle is absent from it."""
    return [{"t": c["t"], "status": c["status"], "winner": c["seed"], "full_multistart": c["full_multistart"],
             "seeds": {r["name"]: r for r in c["seeds"]}} for c in cycles if t0 <= c["t"] < t1]


def layer_at(cycles, states, t):
    """At time t, the last cycle and state at or before it: F2's status, whether its winner was feasible, B5's status,
    fallback and gate, and the base's position."""
    c = next((c for c in reversed(cycles) if c["t"] <= t + 1e-9), None)
    x = next((x for x in reversed(states) if x["t"] <= t + 1e-9), None)
    winner = next((r for r in c["seeds"] if r["name"] == c["seed"]), None) if c else None
    return {"t": t, "medium": c["status"] if c else None, "plan_feasible": winner["feasible"] if winner else None,
            "winner": c["seed"] if c else None, "b5_status": x["status"] if x else None,
            "fallback": x["fallback"] if x else None, "gate": x["gate"] if x else None,
            "x": x["x"] if x else None, "y": x["y"] if x else None}


def print_seed_table(table, out=sys.stdout):
    """One line per cycle: t, status, winner; then per seed: feasible, merit, collision_m, the deepest row."""
    for row in table:
        out.write(f"{row['t']:8.2f} {row['status']:<11} winner {row['winner']}{'  (full)' if row['full_multistart'] else ''}\n")
        for name, r in row["seeds"].items():
            detail = ""
            if r["collision_m"] is not None:
                detail = (f" coll {r['collision_m']:.3f} torque {r['torque']:.3f} it {r['iterations']}"
                          f"{' cut' if r['interrupted'] else ''} {r['homotopy']}")
                if r["deepest_obstacle"]:
                    detail += f" deepest {r['deepest_link']}/{r['deepest_obstacle']} {r['deepest_distance']:.3f} m at {r['deepest_t']:.2f} s"
            merit = "None" if r["merit"] is None else f"{r['merit']:.3f}"
            out.write(f"           {name:<22} {'feas' if r['feasible'] else 'INF '} merit {merit} prog {r['progress_m']:.2f}{detail}\n")


def write_report(records, out):
    """out/report.md (per cell: runs, reached, the classes, the median share of cycles not ok before the fork) and
    out/fork_points.json (the failed runs with a fork point)."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    by_cell = defaultdict(list)
    for r in records:
        by_cell[r["cell"]].append(r)
    lines = ["# Medium diagnosis: runs per cell", "",
             "| cell | runs | reached | classes | median share of cycles not ok before the fork |", "|---|---|---|---|---|"]
    for cell in sorted(by_cell, key=str):
        rs = by_cell[cell]
        classes = Counter(r["class"] for r in rs if r["class"] != "reached")
        shares = [r["not_ok_before"] for r in rs if r["not_ok_before"] is not None]
        median = f"{statistics.median(shares):.2f}" if shares else "-"
        text = ", ".join(f"{k} {v}" for k, v in sorted(classes.items())) or "-"
        lines.append(f"| {cell} | {len(rs)} | {sum(r['reached'] for r in rs)} | {text} | {median} |")
    (out / "report.md").write_text("\n".join(lines) + "\n")
    keys = ("run", "cell", "seed", "kind", "t", "class", "stop_t", "stop_x", "stop_y", "b5_status", "b5_fallback")
    points = [{k: r[k] for k in keys} for r in records if r["kind"] is not None]
    (out / "fork_points.json").write_text(json.dumps(points, indent=1) + "\n")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="step", required=True)
    r = sub.add_parser("report")
    r.add_argument("--runs", required=True)
    r.add_argument("--out", required=True)
    t = sub.add_parser("seeds", help="the seed table of one run between two times")
    t.add_argument("--run", required=True)
    t.add_argument("--from", dest="t0", type=float, default=0.0)
    t.add_argument("--to", dest="t1", type=float, default=float("inf"))
    args = ap.parse_args(argv)
    if args.step == "report":
        write_report(diagnose_runs(args.runs), args.out)
    elif args.step == "seeds":
        print_seed_table(seed_table(cycles(frames_of(Path(args.run) / "run.mmlog")), args.t0, args.t1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
