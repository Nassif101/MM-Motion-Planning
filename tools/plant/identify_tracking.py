"""Identify the MuJoCo plant's tracking model for B5's prediction (E2 plan Task 6, amended at CP1; spec
docs/superpowers/specs/2026-11-11-e2-tracking-prediction-design.md Section 3.2 and Amendment 1).

The map (tracking_model.hpp): yaw rate = g_inf w - (g_inf - g_0) c tanh(w / c); forward speed = v (1 - s sabs(yaw
rate)); lateral speed = k x forward speed x yaw rate; and the arm's command delay. The fit is per 20 ms interval on
the measured body speeds (finite differences of the logged pose, model_rollout's time convention: record j's command
drove the interval into j). For each c on a grid the turn map is linear in (g_inf, g_0); s and k are then
single-coefficient regressions; the arm delay is the pipeline alignment's delay plus time constant. Every parameter is
kept inside load_tracking_model's ranges. The held-out check rolls B5's own model (model_step) out with and without
the map over non-overlapping 0.1 s windows of the held-out segments.

    python3 tools/plant/identify_tracking.py fit --source excitation|closed-loop [--logs LOG...] --terms a,b,...
        --out FILE --report DIR [--poses carry,home,...]
    python3 tools/plant/identify_tracking.py check --model FILE --logs LOG... --report DIR
    python3 tools/plant/identify_tracking.py grip --terms a,b,... --report DIR

Runs in the container (plant_replay, numpy through calibrate_plant).
"""
import argparse
import datetime as dt
import json
import math
import sys
from pathlib import Path

import model_rollout as mr

ROOT = Path(__file__).resolve().parents[2]
MODEL_KEYS = ("turn_delta_inf", "turn_delta_zero", "turn_scale", "speed_turn", "sideslip", "base_delay_s", "arm_delay_s")
DEFAULTS = {"turn_delta_inf": 0.0, "turn_delta_zero": 0.0, "turn_scale": 0.05, "speed_turn": 0.0, "sideslip": 0.0,
            "base_delay_s": 0.0, "arm_delay_s": 0.0}
RANGES = {"turn_delta_inf": (-0.5, 0.5), "turn_delta_zero": (-0.99, 0.5), "turn_scale": (0.001, 1.0),
          "speed_turn": (0.0, 1.0), "sideslip": (-0.5, 0.5), "base_delay_s": (0.0, 0.2), "arm_delay_s": (0.0, 0.2)}
TURN_SCALES = [round(0.005 + 0.0025 * k, 6) for k in range(39)]  # 0.005 .. 0.1 rad/s
POSES = ("carry", "home", "vertical_carry", "level_extension", "fold")
GRID_V = (0.0, 0.1, 0.2, 0.3)
GRID_W = (0.0, 0.05, -0.05, 0.1, -0.1, 0.15, -0.15, 0.2, -0.2, 0.3, -0.3, 0.4, -0.4, 0.6, -0.6)
ENVELOPE = 1.12  # |v| + 1.5 |omega| (commission_plant: the rim speed limit)
GRIP_PROFILES = {"0.4": "experiment_runs/e1/profiles/profile_mu0.4.yaml", "0.8": "experiment_runs/e1/profiles/profile_mu0.8.yaml"}


def clip(name, value):
    low, high = RANGES[name]
    return min(high, max(low, value))


def turn_map(w, g_inf, g_0, c):
    return g_inf * w - (g_inf - g_0) * c * math.tanh(w / c)


def sabs(x):
    return math.sqrt(x * x + 1e-4) - 0.01


def map_parameters(p):
    return [p["turn_delta_inf"], p["turn_delta_zero"], p["turn_scale"], p["speed_turn"], p["sideslip"]]


def intervals(segments, role):
    """(v_cmd, w_cmd, forward, lateral, yaw rate) per 20 ms interval of the segments with this role."""
    out = []
    for seg in segments:
        if seg["role"] != role:
            continue
        recs = seg["records"]
        for j in range(1, len(recs)):
            step = recs[j]["t"] - recs[j - 1]["t"]
            if not 0.0 < step <= mr.MAX_GAP_S:
                continue
            fwd, lat, yaw_rate = mr._body(recs[j - 1]["pose"], recs[j]["pose"], step)
            out.append((recs[j]["cmd"][0], recs[j]["cmd"][1], fwd, lat, yaw_rate))
    return out


def _fit_turn(data, terms, p):
    """g_inf, g_0 (linear) for each c on the grid; the c with the least squared yaw-rate error."""
    free_inf, free_zero = "turn_delta_inf" in terms, "turn_delta_zero" in terms
    scales = TURN_SCALES if "turn_scale" in terms else [p["turn_scale"]]
    best = None
    for c in scales:
        a = [w - c * math.tanh(w / c) for _, w, _, _, _ in data]
        b = [c * math.tanh(w / c) for _, w, _, _, _ in data]
        y = [yr for *_, yr in data]
        if free_inf and free_zero:
            saa, sbb, sab = sum(x * x for x in a), sum(x * x for x in b), sum(x * z for x, z in zip(a, b))
            say, sby = sum(x * z for x, z in zip(a, y)), sum(x * z for x, z in zip(b, y))
            det = saa * sbb - sab * sab
            if abs(det) < 1e-18:
                continue
            g_inf, g_0 = (say * sbb - sby * sab) / det, (sby * saa - say * sab) / det
        elif free_zero:  # g_inf fixed: y - g_inf a = g_0 b
            g_inf = 1.0 + p["turn_delta_inf"]
            sbb = sum(x * x for x in b)
            g_0 = sum(x * (z - g_inf * u) for x, z, u in zip(b, y, a)) / sbb if sbb > 0 else 1.0
        else:
            g_inf = sum(x * z for x, z in zip(a, y)) / max(1e-18, sum(x * x for x in a))
            g_0 = 1.0 + p["turn_delta_zero"]
        g_inf = 1.0 + clip("turn_delta_inf", g_inf - 1.0)
        g_0 = 1.0 + clip("turn_delta_zero", g_0 - 1.0)
        sse = sum((g_inf * u + g_0 * x - z) ** 2 for u, x, z in zip(a, b, y))
        if best is None or sse < best[0]:
            best = (sse, g_inf, g_0, c)
    if best is not None:
        _, g_inf, g_0, c = best
        if free_inf:
            p["turn_delta_inf"] = g_inf - 1.0
        if free_zero:
            p["turn_delta_zero"] = g_0 - 1.0
        if "turn_scale" in terms:
            p["turn_scale"] = c


def fit(segments, terms, held_out_check=False, container=mr.CONTAINER):
    """The parameters of `terms` (TrackingModel's field names) from the `fit` segments, the others at their defaults;
    with held_out_check the one-stage error of B5's model with and without them on the `held_out` segments."""
    unknown = set(terms) - set(MODEL_KEYS)
    if unknown:
        raise ValueError(f"unknown terms: {sorted(unknown)}")
    p = dict(DEFAULTS)
    data = [d for d in intervals(segments, "fit") if d[0] != 0.0 or d[1] != 0.0]
    if any(t in terms for t in ("turn_delta_inf", "turn_delta_zero", "turn_scale")):
        _fit_turn(data, terms, p)
    g_inf, g_0, c = 1.0 + p["turn_delta_inf"], 1.0 + p["turn_delta_zero"], p["turn_scale"]
    wf = [turn_map(w, g_inf, g_0, c) if p["turn_delta_inf"] or p["turn_delta_zero"] else w for _, w, *_ in data]
    if "speed_turn" in terms:  # forward - v = -s v sabs(wf)
        x = [v * sabs(w) for (v, *_), w in zip(data, wf)]
        y = [fwd - v for (v, _, fwd, _, _) in data]
        sxx = sum(a * a for a in x)
        p["speed_turn"] = clip("speed_turn", -sum(a * b for a, b in zip(x, y)) / sxx) if sxx > 0 else 0.0
    vf = [v * (1 - p["speed_turn"] * sabs(w)) for (v, *_), w in zip(data, wf)]
    if "sideslip" in terms:  # lateral = k vf wf
        x = [a * b for a, b in zip(vf, wf)]
        sxx = sum(a * a for a in x)
        p["sideslip"] = clip("sideslip", sum(a * d[3] for a, d in zip(x, data)) / sxx) if sxx > 0 else 0.0
    if "arm_delay_s" in terms or "base_delay_s" in terms:
        fit_recs = [dict(r, goal=(n, r["goal"])) for n, seg in enumerate(segments) if seg["role"] == "fit"
                    for r in seg["records"]]
        al = mr.alignment(fit_recs)
        if "arm_delay_s" in terms:
            lags = [al[f"q{i}"]["pipeline"]["delay"] + al[f"q{i}"]["pipeline"]["tau"] for i in range(6)
                    if al[f"q{i}"]["pipeline"]]
            p["arm_delay_s"] = clip("arm_delay_s", round(sum(lags) / len(lags) / 0.005) * 0.005) if lags else 0.0
        if "base_delay_s" in terms:
            lags = [al[k]["delay"] for k in ("v", "omega") if al[k]]
            p["base_delay_s"] = clip("base_delay_s", sum(lags) / len(lags)) if lags else 0.0
    result = {"parameters": p, "terms": list(terms), "intervals": len(data)}
    if held_out_check:
        result["held_out"] = held_out(segments, p, container)
    return result


def _rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else None


def one_stage_errors(recs_list, params, container=mr.CONTAINER, keep=None):
    """RMS of the one-stage (0.1 s) error per axis, B5's model with and without the map, over non-overlapping 0.1 s
    windows (`keep(recs, i0, i1)` filters them)."""
    wins = [(recs, i0, i1) for recs in recs_list for i0, i1 in mr.windows(recs, 0.1, stride=5)
            if keep is None or keep(recs, i0, i1)]
    out = {"windows": len(wins)}
    with mr.container_step(container=container) as plain, \
            mr.container_step(container=container, params=map_parameters(params)) as mapped:
        for name, step in (("before", plain), ("after", mapped)):
            errs = []
            for recs in recs_list:
                own = [(i0, i1) for r, i0, i1 in wins if r is recs]
                if own:
                    preds = mr.rollout_many(recs, own, "cmd", step)
                    errs += [mr.error(recs, i0, i1, x) for (i0, i1), x in zip(own, preds)]
            out[name] = {k: _rms([e[k] for e in errs]) for k in ("forward", "lateral", "yaw")}
            out[name].update({f"q{i}": _rms([e["q"][i] for e in errs]) for i in range(6)})
    return out


def held_out(segments, params, container=mr.CONTAINER):
    return one_stage_errors([s["records"] for s in segments if s["role"] == "held_out"], params, container)


def write_model(path, params, provenance):
    """The tracking model file: a header with its provenance, then TrackingModel's keys in order (6 significant
    digits). Deterministic for the same parameters and provenance."""
    lines = ["# The MuJoCo plant's tracking model for B5's prediction (E2, tools/plant/identify_tracking.py).",
             f"# Identified {provenance.get('date', '')} from {provenance.get('source', '')}; terms "
             f"{', '.join(provenance.get('terms', []))}.",
             "# Held-out one-stage rms (m, rad) before -> after: "
             + ", ".join(f"{k} {v[0]:.4g} -> {v[1]:.4g}" for k, v in sorted(provenance.get("held_out", {}).items())),
             "# tracking_model.hpp: yaw rate = g_inf w - (g_inf - g_0) c tanh(w / c), g = 1 + delta; forward speed ="
             " v (1 - speed_turn sabs(yaw rate)); lateral = sideslip x forward x yaw rate."]
    if provenance.get("profile_sha256"):
        lines.append(f"# Plant profile sha256 {provenance['profile_sha256']}.")
    lines += [f"{k}: {params[k]:.6g}" for k in MODEL_KEYS]
    Path(path).write_text("\n".join(lines) + "\n")


# ---- the excitation ------------------------------------------------------------------------------------------------

def excitation(poses=POSES):
    """Per pose: the steady twists of the grid inside the envelope (6 s each from rest), Task 4's steps and ramps, and
    the turns entered and left straight (3 s, 6 s, 3 s). Roles alternate fit / held_out over the whole list."""
    segs = []
    for pose in poses:
        q = mr.arm_pose(pose)
        for v in GRID_V:
            for w in GRID_W:
                if (v, w) == (0.0, 0.0) or abs(v) + 1.5 * abs(w) > ENVELOPE + 1e-9:
                    continue
                segs.append({"name": f"steady_v{v:g}_w{w:g}_{pose}", "pose": pose,
                             "input": mr._replay_input(lambda t, v=v, w=w: (v, w), 6.0, q)})
        for seg in mr.excitation_inputs(q):
            if seg["name"].startswith("turn"):
                v, w = (float(s[1:]) for s in seg["name"].split("_")[1:3])
                seg = {"name": seg["name"],
                       "input": mr._replay_input(lambda t, v=v, w=w: (v, w if 3.0 <= t < 9.0 else 0.0), 12.0, q)}
            segs.append({"name": f"{seg['name']}_{pose}", "pose": pose, "input": seg["input"]})
    for k, s in enumerate(segs):
        s["role"] = "fit" if k % 2 == 0 else "held_out"
    return segs


def _replay(job):
    seg, profile = job
    import calibrate_plant as cp
    out = cp.plant_replay(seg["input"], {}, profile=profile or mr.PLANT_PROFILE)
    out = {k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in out.items()}
    return mr.excitation_records(seg, out)


def run_excitation(segs, profile=None, jobs=4):
    from multiprocessing import Pool
    with Pool(jobs) as pool:
        records = pool.map(_replay, [(s, profile) for s in segs])
    for s, recs in zip(segs, records):
        s["records"] = recs
    return segs


def closed_loop_segments(logs):
    """The closed-loop logs as segments, alternating fit / held_out by log."""
    return [{"name": Path(log).parent.name, "role": "fit" if k % 2 == 0 else "held_out", "records": mr.records(log)[1]}
            for k, log in enumerate(logs)]


def per_pose(segs, params):
    """The held-out one-stage rms per pose (the map identified on all poses)."""
    out = {}
    for pose in sorted({s.get("pose") for s in segs if s.get("pose")}):
        out[pose] = one_stage_errors([s["records"] for s in segs if s.get("pose") == pose and s["role"] == "held_out"],
                                     params, container=None)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("fit")
    f.add_argument("--source", choices=["excitation", "closed-loop"], required=True)
    f.add_argument("--logs", nargs="*", default=[])
    f.add_argument("--terms", required=True)
    f.add_argument("--out", required=True)
    f.add_argument("--report", required=True)
    f.add_argument("--poses", default=",".join(POSES))
    f.add_argument("--profile")
    f.add_argument("--date", default=dt.date.today().isoformat())
    c = sub.add_parser("check")
    c.add_argument("--model", required=True)
    c.add_argument("--logs", nargs="+", required=True)
    c.add_argument("--report", required=True)
    g = sub.add_parser("grip")
    g.add_argument("--terms", required=True)
    g.add_argument("--report", required=True)
    args = ap.parse_args(argv)
    report = Path(args.report)
    report.mkdir(parents=True, exist_ok=True)
    if args.cmd == "fit":
        terms = args.terms.split(",")
        if args.source == "excitation":
            segs = run_excitation(excitation(tuple(args.poses.split(","))), profile=args.profile)
        else:
            segs = closed_loop_segments(args.logs)
        result = fit(segs, terms, held_out_check=True, container=None)
        if args.source == "excitation":
            result["per_pose"] = per_pose(segs, result["parameters"])
        ho = result["held_out"]
        write_model(args.out, result["parameters"],
                    {"date": args.date, "source": f"{args.source} ({len(segs)} segments)", "terms": terms,
                     "held_out": {k: (ho["before"][k], ho["after"][k]) for k in ("forward", "lateral", "yaw")},
                     "profile_sha256": mr_sha(args.profile)})
        (report / f"fit_{args.source}.json").write_text(json.dumps(result, indent=1))
    elif args.cmd == "check":
        params = read_model(args.model)
        recs_list = [mr.records(log)[1] for log in args.logs]
        deep = lambda recs, i0, i1: (recs[i0 + 1]["depth"] or 0.0) >= 0.005  # noqa: E731 - the ratchet's windows
        result = {"all": one_stage_errors(recs_list, params, container=None),
                  "deep": one_stage_errors(recs_list, params, container=None, keep=deep), "parameters": params}
        (report / "check.json").write_text(json.dumps(result, indent=1))
    else:
        terms = args.terms.split(",")
        result = {}
        for mu, profile in GRIP_PROFILES.items():
            segs = run_excitation(excitation(("carry",)), profile=str(ROOT / profile))
            result[mu] = fit(segs, terms, held_out_check=True, container=None)
        (report / "grip.json").write_text(json.dumps(result, indent=1))
    return 0


def read_model(path):
    p = dict(DEFAULTS)
    for line in Path(path).read_text().splitlines():
        if line.strip() and not line.startswith("#"):
            key, value = line.split(":", 1)
            if key.strip() not in MODEL_KEYS:
                raise ValueError(f"{path}: unknown key {key.strip()!r}")
            p[key.strip()] = float(value)
    return p


def mr_sha(profile):
    import hashlib
    path = Path(profile) if profile else mr.PLANT_PROFILE
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return ""


if __name__ == "__main__":
    sys.exit(main())
