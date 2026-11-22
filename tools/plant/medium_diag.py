"""The medium-planner diagnosis's log reading (plan docs/superpowers/plans/2026-11-22-medium-diagnosis.md; spec
docs/superpowers/specs/2026-11-22-medium-diagnosis-design.md). Standard library only.

log_fingerprint: a run log's frames after the Hello, hashed, so that two builds' logs compare without the Hello's
provenance (the commit) and its config keys.

Task 2: per run, the medium cycles (Plans) and the plant states (State), where the robot stopped (the session's
progress rule), the fork point (the first cycle of the final infeasible streak, or the stall's onset) and a stall
class; `report` writes a per-cell table and the fork points.

  python3 tools/plant/medium_diag.py report --runs DIR --out DIR2
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


# The Plans seed row's fields: E1's five, then the medium diagnosis's (Task 1, --seed-detail; `interrupted` its review I1)
BASE_FIELDS = ("name", "merit", "feasible", "progress_m", "merged")
DETAIL_FIELDS = ("slack", "collision_m", "torque", "speed_mps", "hull", "iterations", "converged", "resumed", "homotopy",
                 "deepest_link", "deepest_obstacle", "deepest_distance", "deepest_t", "interrupted")
FLAGS = {"feasible", "merged", "converged", "resumed", "interrupted"}

# The session's progress rule (session.cpp kStallBaseM, kStallYawRad, kStallJointRad)
STALL_BASE_M, STALL_YAW_RAD, STALL_JOINT_RAD = 0.05, 0.1, 0.05
# An infeasible streak that starts this long after the stall's onset did not cause it (the robot had stopped first)
STREAK_LAG_S = 2.0
# A run that still made progress this close to its end timed out moving
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


def _moved(a, b, base_m, yaw_rad, joint_rad):
    yaw = math.remainder(a["yaw"] - b["yaw"], 2.0 * math.pi)
    return (math.hypot(a["x"] - b["x"], a["y"] - b["y"]) > base_m or abs(yaw) > yaw_rad
            or max(abs(p - q) for p, q in zip(a["q"], b["q"])) > joint_rad)


def stall_onset(states, base_m=STALL_BASE_M, yaw_rad=STALL_YAW_RAD, joint_rad=STALL_JOINT_RAD):
    """The last time the session's progress rule saw progress (its reference moved); planning holds of a lattice run
    are skipped as the session skips them. None without states."""
    ref, onset = None, None
    for x in states:
        if x["execution"] == "holding":
            continue
        if ref is None or _moved(x, ref, base_m, yaw_rad, joint_rad):
            ref, onset = x, x["t"]
    return onset


def fork_point(cycles, states, reached):
    """None for a reached run. Else {"kind": "infeasible", "t"}: the first cycle of the final streak of cycles that are
    not ok (at least one infeasible) running to the last cycle, when it starts no later than STREAK_LAG_S after the
    stall's onset; otherwise {"kind": "stall_onset", "t": the onset}."""
    if reached:
        return None
    onset = stall_onset(states)
    start = None
    for c in reversed(cycles):
        if c["status"] == "ok":
            break
        start = c
    streak = [c for c in cycles if start is not None and c["t"] >= start["t"]]
    if start is not None and any(c["status"] == "infeasible" for c in streak) and (
            onset is None or start["t"] <= onset + STREAK_LAG_S):
        return {"kind": "infeasible", "t": start["t"]}
    return {"kind": "stall_onset", "t": onset}


def _share(items, test):
    items = list(items)
    return sum(1 for i in items if test(i)) / len(items) if items else None


def classify(cycles, states, fork):
    """reached (no fork), infeasible_stall, timeout_moving (progress within MOVING_AT_END_S of the end), b5_refused
    (after the onset F2's cycles mostly ok while B5 mostly held or fell back), f2_waiting (mostly ok plans that wait),
    slack_stall (otherwise: F2 mostly not ok but not persistently)."""
    if fork is None:
        return "reached"
    if fork["kind"] == "infeasible":
        return "infeasible_stall"
    end = states[-1]["t"] if states else None
    if fork["t"] is None or (end is not None and end - fork["t"] < MOVING_AT_END_S):
        return "timeout_moving"
    after = [c for c in cycles if c["t"] >= fork["t"]]
    ok = _share(after, lambda c: c["status"] == "ok") or 0.0
    held = _share([x for x in states if x["t"] >= fork["t"]],
                  lambda x: x["status"] != "ok" or bool(x["fallback"])) or 0.0
    if ok >= 0.5 and held >= 0.5:
        return "b5_refused"
    if ok >= 0.5 and (_share(after, lambda c: bool(c["waiting"])) or 0.0) >= 0.5:
        return "f2_waiting"
    return "slack_stall"


def diagnose_run(run_dir):
    """One run directory (run.mmlog, summary.json) as a record: run, cell, seed, reached, class, fork kind and t, and
    the share of cycles not ok before the fork."""
    run_dir = Path(run_dir)
    summary = json.loads((run_dir / "summary.json").read_text())
    record = {"run": run_dir.name, "cell": summary.get("cell"), "seed": summary.get("seed"),
              "reached": bool(summary.get("reached")), "kind": None, "t": None, "not_ok_before": None}
    try:
        frames = frames_of(run_dir / "run.mmlog")
    except (ValueError, OSError, StopIteration):
        return {**record, "class": "unreadable"}
    cs, xs = cycles(frames), states(frames)
    if not cs:
        return {**record, "class": "no_medium"}
    fork = fork_point(cs, xs, record["reached"])
    if fork:
        record.update(kind=fork["kind"], t=fork["t"])
        record["not_ok_before"] = _share([c for c in cs if fork["t"] is None or c["t"] < fork["t"]],
                                         lambda c: c["status"] != "ok")
    return {**record, "class": classify(cs, xs, fork)}


def diagnose_runs(root):
    """Every run directory under `root` that has a summary.json, in name order."""
    return [diagnose_run(d) for d in sorted(Path(root).iterdir()) if (d / "summary.json").exists()]


def write_report(records, out):
    """out/report.md (per cell: runs, reached, the classes, the median share of cycles not ok before the fork) and
    out/fork_points.json (the failed runs with a fork point)."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    by_cell = defaultdict(list)
    for r in records:
        by_cell[r["cell"]].append(r)
    lines = ["# Medium diagnosis: runs per cell", "",
             "| cell | runs | reached | classes | median share not ok before the fork |", "|---|---|---|---|---|"]
    for cell in sorted(by_cell, key=str):
        rs = by_cell[cell]
        classes = Counter(r["class"] for r in rs if r["class"] != "reached")
        shares = [r["not_ok_before"] for r in rs if r["not_ok_before"] is not None]
        median = f"{statistics.median(shares):.2f}" if shares else "-"
        text = ", ".join(f"{k} {v}" for k, v in sorted(classes.items())) or "-"
        lines.append(f"| {cell} | {len(rs)} | {sum(r['reached'] for r in rs)} | {text} | {median} |")
    (out / "report.md").write_text("\n".join(lines) + "\n")
    points = [{"run": r["run"], "cell": r["cell"], "seed": r["seed"], "kind": r["kind"], "t": r["t"], "class": r["class"]}
              for r in records if r["kind"] is not None]
    (out / "fork_points.json").write_text(json.dumps(points, indent=1) + "\n")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="step", required=True)
    r = sub.add_parser("report")
    r.add_argument("--runs", required=True)
    r.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    if args.step == "report":
        write_report(diagnose_runs(args.runs), args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
