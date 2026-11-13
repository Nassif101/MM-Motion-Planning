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


def _bounded_gains(a, b, y, free_inf, free_zero, g_inf0, g_00):
    """The least-squares (g_inf, g_0) of y = g_inf a + g_0 b within the loader's ranges (Task 6 review F1): a fixed gain
    is subtracted before solving; with both free the interior solution if admissible, else the best of the boundary
    solutions (one gain on a bound, the other refitted and clipped) and the corners. Returns (sse, g_inf, g_0)."""
    lo_i, hi_i = (1 + RANGES["turn_delta_inf"][0], 1 + RANGES["turn_delta_inf"][1])
    lo_z, hi_z = (1 + RANGES["turn_delta_zero"][0], 1 + RANGES["turn_delta_zero"][1])
    saa, sbb = sum(x * x for x in a), sum(x * x for x in b)
    sab = sum(x * z for x, z in zip(a, b))
    say, sby = sum(x * z for x, z in zip(a, y)), sum(x * z for x, z in zip(b, y))
    syy = sum(z * z for z in y)

    def sse(gi, gz):
        return syy - 2 * gi * say - 2 * gz * sby + gi * gi * saa + 2 * gi * gz * sab + gz * gz * sbb

    def best_inf(gz):  # g_inf given g_0, clipped (exact for a convex one-dimensional quadratic)
        return min(hi_i, max(lo_i, (say - gz * sab) / saa)) if saa > 0 else g_inf0

    def best_zero(gi):
        return min(hi_z, max(lo_z, (sby - gi * sab) / sbb)) if sbb > 0 else g_00
    if free_inf and free_zero:
        det = saa * sbb - sab * sab
        candidates = []
        if abs(det) > 1e-18:
            gi, gz = (say * sbb - sby * sab) / det, (sby * saa - say * sab) / det
            if lo_i <= gi <= hi_i and lo_z <= gz <= hi_z:
                candidates.append((gi, gz))
        if not candidates:
            for gz in (lo_z, hi_z):
                candidates.append((best_inf(gz), gz))
            for gi in (lo_i, hi_i):
                candidates.append((gi, best_zero(gi)))
        gi, gz = min(candidates, key=lambda c: sse(*c))
    elif free_inf:
        gi, gz = best_inf(g_00), g_00
    elif free_zero:
        gi, gz = g_inf0, best_zero(g_inf0)
    else:
        gi, gz = g_inf0, g_00
    return sse(gi, gz), gi, gz


def _fit_turn(data, terms, p):
    """For each turn scale on the grid (or the current one), the bounded gains; the scale whose returned model has the
    least squared yaw-rate error."""
    free_inf, free_zero = "turn_delta_inf" in terms, "turn_delta_zero" in terms
    scales = TURN_SCALES if "turn_scale" in terms else [p["turn_scale"]]
    y = [yr for *_, yr in data]
    best = None
    for c in scales:
        a = [w - c * math.tanh(w / c) for _, w, _, _, _ in data]
        b = [c * math.tanh(w / c) for _, w, _, _, _ in data]
        sse, g_inf, g_0 = _bounded_gains(a, b, y, free_inf, free_zero, 1.0 + p["turn_delta_inf"], 1.0 + p["turn_delta_zero"])
        if best is None or sse < best[0] - 1e-15:
            best = (sse, g_inf, g_0, c)
    if best is not None:
        _, g_inf, g_0, c = best
        p["turn_delta_inf"], p["turn_delta_zero"] = g_inf - 1.0, g_0 - 1.0
        if "turn_scale" in terms and (free_inf or free_zero):
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


def yaml_number(value):
    """A YAML number the shared resolver reads as one (Task 6 review F3): 6 significant digits, an exponent's mantissa
    always with a decimal point (1e-06 would be read as a string)."""
    text = f"{value:.6g}"
    if "e" in text:
        mantissa, exponent = text.split("e")
        if "." not in mantissa:
            mantissa += ".0"
        text = f"{mantissa}e{exponent}"
    return text


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
    lines += [f"{k}: {yaml_number(params[k])}" for k in MODEL_KEYS]
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


def case_of(name):
    """The physical case of a run directory <cell>-<kind>-<mode>[-<tag>]-s<seed>[-n]: <cell>-<kind>-s<seed>[-n] (the
    tag names a config, the mode is the same throughout)."""
    parts = name.split("-")
    seed = next(p for p in reversed(parts) if p.startswith("s") and p[1:].isdigit())
    return f"{parts[0]}-{parts[1]}-{seed}" + ("-n" if parts[-1] == "n" else "")


def role_of(case):
    """A case's role, the same whatever other runs are present (Task 6 review F2): fit or held_out by a stable hash."""
    import zlib
    return "fit" if zlib.crc32(case.encode()) % 2 == 0 else "held_out"


def closed_loop_segments(logs):
    """The closed-loop logs as segments, one per physical case: a second log of a case with identical records (the
    same run under another config) is dropped; the role follows the case (role_of)."""
    segs, seen = [], {}
    for log in logs:
        name = Path(log).parent.name
        case = case_of(name)
        recs = mr.records(log)[1]
        key = [(r["t"], tuple(r["cmd"]), tuple(r["pose"])) for r in recs]
        if case in seen and seen[case] == key:
            continue
        seen.setdefault(case, key)
        segs.append({"name": name, "case": case, "role": role_of(case), "records": recs})
    return segs


def logs_profile(logs):
    """The plant profile the logs were made with (their hello's profile_sha256); ValueError for mixed profiles
    (Task 6 review F4: a fit names the plant that made its data)."""
    import plant_stream as ps
    found = {ps.RunLog(log).header.get("profile_sha256", "") for log in logs}
    if len(found) != 1:
        raise ValueError(f"the logs come from different plant profiles: {sorted(found)}")
    return found.pop()


def save_segments(path, segs, meta):
    """The excitation's (or any) segments with their roles and records, gzip JSON (Task 6 review F5)."""
    import gzip
    keep = ("name", "role", "pose", "case", "records")
    with gzip.open(path, "wt") as out:
        json.dump({"meta": meta, "segments": [{k: s[k] for k in keep if k in s} for s in segs]}, out)


def load_segments(path):
    import gzip
    with gzip.open(path, "rt") as stream:
        data = json.load(stream)
    return data["segments"], data["meta"]


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
    f.add_argument("--from-records", help="excitation: refit from a saved excitation_records.json.gz")
    c = sub.add_parser("check")
    c.add_argument("--model", required=True)
    c.add_argument("--logs", nargs="+", required=True)
    c.add_argument("--report", required=True)
    hz = sub.add_parser("horizon", help="T11 at 1 s and 2 s: unicycle, prior, online correction, per slice")
    hz.add_argument("--model", required=True)
    hz.add_argument("--logs", nargs="+", required=True)
    hz.add_argument("--report", required=True)
    hz.add_argument("--counterfactual", action="store_true", help="score a model the logs did not use (labelled)")
    cl = sub.add_parser("closed-loop", help="T1 on closed-loop logs: one-stage errors, depth direction, estimator replay")
    cl.add_argument("--model", required=True)
    cl.add_argument("--logs", nargs="+", required=True)
    cl.add_argument("--report", required=True)
    cl.add_argument("--counterfactual", action="store_true", help="score a model the logs did not use (labelled)")
    g = sub.add_parser("grip")
    g.add_argument("--terms", required=True)
    g.add_argument("--report", required=True)
    args = ap.parse_args(argv)
    report = Path(args.report)
    report.mkdir(parents=True, exist_ok=True)
    if args.cmd == "fit":
        terms = args.terms.split(",")
        if args.source == "excitation":
            if args.from_records:
                segs, meta = load_segments(args.from_records)
                profile_sha = meta.get("profile_sha256", "")
            else:
                segs = run_excitation(excitation(tuple(args.poses.split(","))), profile=args.profile)
                profile_sha = mr_sha(args.profile)
                save_segments(report / "excitation_records.json.gz", segs,
                              {"profile_sha256": profile_sha, "source": "identify_tracking excitation",
                               "poses": args.poses, "date": args.date})
        else:
            segs = closed_loop_segments(args.logs)
            profile_sha = logs_profile(args.logs)
            if args.profile and mr_sha(args.profile) != profile_sha:
                raise ValueError(f"--profile {args.profile} is not the profile the logs were made with")
        result = fit(segs, terms, held_out_check=True, container=None)
        if args.source == "excitation":
            result["per_pose"] = per_pose(segs, result["parameters"])
        ho = result["held_out"]
        write_model(args.out, result["parameters"],
                    {"date": args.date, "source": f"{args.source} ({len(segs)} segments)", "terms": terms,
                     "held_out": {k: (ho["before"][k], ho["after"][k]) for k in ("forward", "lateral", "yaw")},
                     "profile_sha256": profile_sha})
        result["manifest"] = [{"name": s["name"], "case": s.get("case"), "pose": s.get("pose"), "role": s["role"]}
                              for s in segs]
        result["profile_sha256"] = profile_sha
        (report / f"fit_{args.source}.json").write_text(json.dumps(result, indent=1))
    elif args.cmd == "check":
        params = read_model(args.model)
        recs_list = [mr.records(log)[1] for log in args.logs]
        deep = lambda recs, i0, i1: (recs[i0 + 1]["depth"] or 0.0) >= 0.005  # noqa: E731 - the ratchet's windows
        result = {"all": one_stage_errors(recs_list, params, container=None),
                  "deep": one_stage_errors(recs_list, params, container=None, keep=deep), "parameters": params}
        (report / "check.json").write_text(json.dumps(result, indent=1))
    elif args.cmd == "horizon":
        result = horizon(args.logs, args.model, container=None, counterfactual=args.counterfactual)
        (report / "horizon.json").write_text(json.dumps(result, indent=1, sort_keys=True))
    elif args.cmd == "closed-loop":  # E2 Task 9 (T1)
        prior = read_model(args.model)
        logged = [mr.records(log) for log in args.logs]
        recs_list = [recs for _, recs in logged]
        meta = model_provenance([hello for hello, _ in logged], args.model, args.counterfactual)
        result = {"_meta": meta, "one_stage": closed_loop_errors(recs_list, prior, container=None),
                  "depth_direction": depth_direction(recs_list, prior, container=None),
                  "estimator_replay_max_diff": max((d for d in (estimate_consistency(recs, args.model, container=None,
                                                                                     start=hello.get("start"))
                                                                for hello, recs in logged) if d is not None),
                                                   default=None)}
        (report / "closed_loop.json").write_text(json.dumps(result, indent=1, sort_keys=True))
    else:
        terms = args.terms.split(",")
        result = {}
        for mu, profile in GRIP_PROFILES.items():
            segs = run_excitation(excitation(("carry",)), profile=str(ROOT / profile))
            result[mu] = fit(segs, terms, held_out_check=True, container=None)
        (report / "grip.json").write_text(json.dumps(result, indent=1))
    return 0


# ---- the horizon check (T11, after the Task 6b review) ---------------------------------------------------------------

TRACKING_REPLAY = "ros2_ws/install/mobile_manipulator_mpc/lib/mobile_manipulator_mpc/tracking_replay"
MOVING = 0.02  # m/s or rad/s: a command above this in either is motion


# ---- E2 Task 9: the closed-loop check (T1) ----------------------------------------------------------------------

CLOSED_LOOP_MODELS = ("unicycle", "prior", "b5")


def b5_parameters(recs, i0, prior):
    """The map parameters B5 planned the transition from record i0 with: record i0 + 1's diag_track (the cycle's g_inf,
    g_0 and k; the time convention: a frame carries what the cycle one period earlier used) with the prior's turn
    scale and speed term; the prior's own where that record carries none."""
    track = recs[i0 + 1].get("track") if i0 + 1 < len(recs) else None
    if not track:
        return map_parameters(prior)
    return [track[0] - 1.0, track[1] - 1.0, prior["turn_scale"], prior["speed_turn"], track[2]]


def _model_parameters(recs, wins, prior):
    """Per window, the map parameters of each model the check compares."""
    identity = [0.0, 0.0, prior["turn_scale"], 0.0, 0.0]
    return {"unicycle": [identity] * len(wins), "prior": [map_parameters(prior)] * len(wins),
            "b5": [b5_parameters(recs, i0, prior) for i0, _ in wins]}


def closed_loop_errors(recs_list, prior, container=mr.CONTAINER):
    """T1: the one-stage (0.1 s, non-overlapping windows) error rms per axis on closed-loop records, for the unicycle,
    the fixed prior and B5's own per-cycle model (b5_parameters)."""
    errs = {k: [] for k in CLOSED_LOOP_MODELS}
    out = {"windows": 0}
    with mr.container_step(container=container) as step:
        for recs in recs_list:
            wins = mr.windows(recs, 0.1, stride=5)
            out["windows"] += len(wins)
            for name, params in _model_parameters(recs, wins, prior).items():
                for (i0, i1), x in zip(wins, _rollout_with_params(recs, wins, params, step)):
                    errs[name].append(mr.error(recs, i0, i1, x))
    for name in CLOSED_LOOP_MODELS:
        out[name] = {k: _rms([e[k] for e in errs[name]]) for k in ("forward", "lateral", "yaw")}
    return out


def depth_direction(recs_list, prior, container=mr.CONTAINER, depth_min=0.005):
    """T1's ratchet part: over the 0.1 s windows that start with the deepest gated pair at least depth_min deep and keep
    that pair without a feature switch (mr.gradient_projection), the share whose predicted depth change (each model's
    motion projected on the pair's gradient at the start) has the sign of the measured depth change. Windows with no
    measured change are left out."""
    agree = {k: 0 for k in CLOSED_LOOP_MODELS}
    n = 0
    with mr.container_step(container=container) as step:
        for recs in recs_list:
            rows = [r for r in mr.gradient_projection(recs, depth_min=depth_min)["per_window"]
                    if not r["feature_switch"] and abs(r["depth_change"]) > 1e-9]
            if not rows:
                continue
            wins = [(r["i0"], r["i1"]) for r in rows]
            n += len(rows)
            for name, params in _model_parameters(recs, wins, prior).items():
                for (i0, _), row, x in zip(wins, rows, _rollout_with_params(recs, wins, params, step)):
                    predicted = mr._split(recs[i0 + 1]["grad"], recs[i0]["pose"], x)["total"]
                    agree[name] += predicted * row["depth_change"] > 0.0
    return {"windows": n, **{k: (agree[k] / n if n else None) for k in CLOSED_LOOP_MODELS}}


def estimate_consistency(recs, model_path, container=mr.CONTAINER, start=None):
    """The largest difference between B5's logged estimate (record j's diag_track: the cycle at record j - 1, after
    the transitions up to it) and the C++ estimator replayed offline over the logged transitions; None without
    diag_track."""
    if not any(r.get("track") for r in recs):
        return None
    est = replay_estimates(recs, model_path, container, start=start)
    return max((abs(a - b) for j in range(1, len(recs)) if recs[j].get("track")
                for a, b in zip(recs[j]["track"], est[j - 1])), default=0.0)


CYCLE_S = 0.02  # the campaign's B5 period: the start pose is one cycle before the first State frame


def replay_estimates(recs, model_path, container=mr.CONTAINER, start=None):
    """The online correction's estimate (g_inf, g_0, k) in force at each record: the C++ estimator from the prior
    `model_path`, fed the transitions up to that record (causal). `start` (x, y, yaw; the hello's): the pose one cycle
    before record 0. A log's first State frame comes after B5's first cycle, so B5 also observed the transition from
    it into record 0, under record 0's command; with `start` the replay includes it and matches B5's own estimate
    (diag_track) to the log's float precision (Task 9)."""
    import subprocess
    lines = ["show"]
    if start is not None and recs:
        r = recs[0]
        lines.append(" ".join(repr(float(v)) for v in [CYCLE_S, *start[:3], *r["pose"][:3], r["cmd"][0], r["cmd"][1]]))
    for j in range(1, len(recs)):
        a, b = recs[j - 1], recs[j]
        lines.append(" ".join(repr(float(v)) for v in [b["t"] - a["t"], *a["pose"][:3], *b["pose"][:3], b["cmd"][0], b["cmd"][1]]))
    binary = f"{mr.WORKSPACE}/{TRACKING_REPLAY}" if container else str(ROOT / TRACKING_REPLAY)
    model = mr.container_path(model_path) if container else str(model_path)
    inner = f"exec {binary} --model {model}"
    argv = ["docker", "exec", "-i", container, "bash", "-lc", inner] if container else ["bash", "-lc", inner]
    out = subprocess.run(argv, input="\n".join(lines) + "\n", capture_output=True, text=True, check=True).stdout
    est = [[float(v) for v in line.split()] for line in out.splitlines()]
    return est[1:] if start is not None and recs else est  # the prior before the start transition is no record's


def _moving(rec):
    return abs(rec["cmd"][0]) > MOVING or abs(rec["cmd"][1]) > MOVING


def horizon_slices(recs, rest_s=0.5, first_s=1.0, after_s=2.0, from_reset=True):
    """Start records of the horizon check by slice (the review's): `moving` (the command in force is motion);
    `launch` (within first_s of the log's first motion, when the log starts at a reset, at rest: the campaign's; Task 9
    review I1), `restart` (within first_s of later motion starting after rest_s at rest), `first_second` (both);
    `after_in_place` (the command a turn above 0.1 rad/s, with an in-place slow turn (|v| < 0.02, 0.02 < |w| < 0.08)
    within the previous after_s). Without `from_reset` the first motion counts only as a restart would."""
    n_rest, n_first, n_after = int(round(rest_s / 0.02)), int(round(first_s / 0.02)), int(round(after_s / 0.02))
    moving, launch, restart, after = [], [], [], []
    start, kind, seen = None, None, False
    for i, r in enumerate(recs):
        if not _moving(r):
            continue
        moving.append(i)
        if not seen and from_reset:
            start, kind = i, "launch"
        elif i >= n_rest and all(not _moving(recs[k]) for k in range(i - n_rest, i)):
            start, kind = i, "restart"
        seen = True
        if start is not None and i < start + n_first:
            (launch if kind == "launch" else restart).append(i)
        if abs(r["cmd"][1]) > 0.1 and any(abs(recs[k]["cmd"][0]) < MOVING and MOVING < abs(recs[k]["cmd"][1]) < 0.08
                                         for k in range(max(0, i - n_after), i)):
            after.append(i)
    return {"moving": moving, "launch": launch, "restart": restart, "first_second": sorted(launch + restart),
            "after_in_place": after}


def model_provenance(hellos, model_path, counterfactual=False):
    """The model file a closed-loop analysis scores "B5's own model" and replays the estimator with must be the one the
    runs used: every log's hello records its SHA-256 (Task 9 review I4). Another file is a counterfactual, refused
    unless asked for; the result says which, for the saved report."""
    import hashlib
    sha = hashlib.sha256(Path(model_path).read_bytes()).hexdigest()
    used = sorted({str(h.get("config", {}).get("tracking_model_sha256")) for h in hellos})
    if used != [sha] and not counterfactual:
        raise ValueError(f"the logs used tracking models {used}, {model_path} is {sha}: not the runs' model; "
                         "score another model only as a counterfactual (--counterfactual)")
    return {"model": str(model_path), "model_sha256": sha, "counterfactual": used != [sha]}


def _rollout_with_params(recs, wins, params, step):
    """The model's state at each window's end from pose[i0], with that window's map parameters (5) appended to every
    step's control: model_step's 23-number lines."""
    states = [list(recs[i0]["pose"]) for i0, _ in wins]
    longest = max((i1 - i0 for i0, i1 in wins), default=0)
    for k in range(1, longest + 1):
        live = [w for w, (i0, i1) in enumerate(wins) if i0 + k <= i1]
        batch = [(recs[wins[w][0] + k]["t"] - recs[wins[w][0] + k - 1]["t"], states[w],
                  list(recs[wins[w][0] + k]["cmd"]) + list(params[w])) for w in live]
        for w, x in zip(live, step(batch)):
            states[w] = list(x)
    return states


def horizon(logs, model_path, horizons=(1.0, 2.0), container=mr.CONTAINER, counterfactual=False):
    """T11 at B5's horizon: from each 0.1 s start, B5's model rolled over the next 1 s and 2 s of logged commands with
    the unicycle, the fixed prior, and the online correction's estimate in force at the start; rms of the yaw and
    body-lateral error at the end, per slice."""
    prior = read_model(model_path)
    fixed = map_parameters(prior)
    acc, runs = {}, {}
    logged = [(log, *mr.records(log)) for log in logs]
    meta = model_provenance([hello for _, hello, _ in logged], model_path, counterfactual)
    with mr.container_step(container=container) as step:
        for log, hello, recs in logged:
            est = replay_estimates(recs, model_path, container, start=hello.get("start"))
            slices = horizon_slices(recs, from_reset=bool(recs) and recs[0]["t"] <= CYCLE_S + 1e-9)
            slices["all"] = list(range(len(recs)))
            cell = Path(log).parent.name.split("-")[0]
            for h in horizons:
                wins = mr.windows(recs, h, stride=5)
                variants = {"unicycle": [[0.0, 0.0, prior["turn_scale"], 0.0, 0.0]] * len(wins), "prior": [fixed] * len(wins),
                            "adapted": [[e[0] - 1.0, e[1] - 1.0, prior["turn_scale"], prior["speed_turn"], e[2]]
                                        for e in (est[i0] for i0, _ in wins)]}
                for name, params in variants.items():
                    ends = _rollout_with_params(recs, wins, params, step)
                    for (i0, i1), x in zip(wins, ends):
                        e = mr.error(recs, i0, i1, x)
                        for sl, members in slices.items():
                            if i0 in members or sl == "all":
                                for key in (f"{h:g}|{sl}|{name}", f"{h:g}|{sl}:{cell}|{name}"):
                                    a = acc.setdefault(key, [0, 0.0, 0.0])
                                    a[0] += 1
                                    a[1] += e["yaw"] ** 2
                                    a[2] += e["lateral"] ** 2
                                    runs.setdefault(key, set()).add(str(log))
    # n: overlapping starts (not independent samples); runs: the logs they come from (Task 9 review)
    out = {k: {"n": n, "runs": len(runs[k]), "yaw": math.sqrt(y / n), "lateral": math.sqrt(l / n)}
           for k, (n, y, l) in acc.items()}
    out["_meta"] = meta
    return out


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
