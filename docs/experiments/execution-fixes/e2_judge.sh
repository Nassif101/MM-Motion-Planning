#!/usr/bin/env bash
# E2 Task 10: judge an ensemble variant (e2_ensembles.sh variant CONFIG TAG) against E1's baselines, T2-T4.
#
#   bash docs/experiments/execution-fixes/e2_judge.sh TAG
#
# Reports into experiment_runs/e2/<TAG>_judge/ (git-ignored):
#   <set>_vs_absinf.md, <set>_vs_abs.md   compare_variants.py regressions (clearance: E2's T2, 1 mm) per run set (twins_a, twins_chicane,
#                                         campaign_b; grip at mu 0.4 and 0.8 with their tags), against
#                                         experiment_runs/e1/absinf (F2's baseline) and experiment_runs/e1/abs (B5's)
#   locks_<set>.md                        compare_variants.py locks of the variant's runs
#   handoff.json                          T4: store_room B5 alone reached; gate_right_swing F2 runs that start left
#                                         and reach; runs with a deep hold per set and kind (baseline: absinf for
#                                         F2, abs for B5), with one-sided Fisher p: `deep_holds` = a deep hold
#                                         anywhere in the run (the plan's measure; Task 10 review I2),
#                                         `deep_holds_terminal` = ending in one
# Exit status: compare_variants' (1 on a regression or a step lock).
set -uo pipefail
cd "$(dirname "$0")/../../.."
tag="${1:?tag}"
variant=experiment_runs/e2/${tag}
out=experiment_runs/e2/${tag}_judge
mkdir -p "${out}"
failed=0

for set in twins_a twins_chicane campaign_b; do
  for base in absinf abs; do
    python3 tools/plant/compare_variants.py regressions "experiment_runs/e1/${base}/${set}" "${variant}/${set}" \
      --baseline-tag "${base}" --tag "${tag}" --clearance-drop-m 0.001 --report "${out}/${set}_vs_${base}.md" || failed=1
  done
  python3 tools/plant/compare_variants.py locks "${variant}/${set}" --tag "${tag}" --report "${out}/locks_${set}.md" || failed=1
done
for mu in 0.4 0.8; do
  for base in absinf abs; do
    python3 tools/plant/compare_variants.py regressions "experiment_runs/e1/${base}/grip" "${variant}/grip" \
      --baseline-tag "${base}_mu${mu}" --tag "${tag}_mu${mu}" --clearance-drop-m 0.001 --report "${out}/grip_mu${mu}_vs_${base}.md" || failed=1
  done
  python3 tools/plant/compare_variants.py locks "${variant}/grip" --tag "${tag}_mu${mu}" \
    --report "${out}/locks_grip_mu${mu}.md" || failed=1
done

python3 - "${tag}" "${out}" <<'PY'
import json, sys
from pathlib import Path
sys.path.insert(0, "tools/plant")
import compare_variants as cv

tag, out = sys.argv[1], Path(sys.argv[2])
variant = Path("experiment_runs/e2") / tag
base_tag = {"f2": "absinf", "b5": "abs"}


def plain(directory, run_tag, cell, kind):
    groups, _ = cv.runs(directory, run_tag)
    return groups.get((cell, kind, "realistic"), [])


result = {}
# store_room for B5 alone (T4: 0 of 20 in E1)
b, v = plain("experiment_runs/e1/abs/twins_a", "abs", "v2_store_room_nav", "b5"), plain(variant / "twins_a", tag, "v2_store_room_nav", "b5")
result["store_room_b5"] = {"baseline": [sum(s["reached"] for s in b), len(b)], "variant": [sum(s["reached"] for s in v), len(v)],
                           "p_variant_better": cv.fisher_greater(sum(s["reached"] for s in v), len(v), sum(s["reached"] for s in b), len(b))}
# gate_right_swing F2: reached per first swing side ("none": no medium plan chose a side; E1's baseline records no
# side for its failing runs, seeds 4, 7, 11 and 19, which E1's frozen twins showed starting left)
gate = {}
for name, d, t in (("baseline", "experiment_runs/e1/absinf/twins_a", "absinf"), ("variant", variant / "twins_a", tag)):
    runs = plain(d, t, "v2_gate_right_swing_nav", "f2")
    sides = {}
    for s in runs:
        side = s.get("first_side") or "none"
        sides.setdefault(side, [0, 0])
        sides[side][0] += bool(s["reached"])
        sides[side][1] += 1
    gate[name] = {"reached": [sum(s["reached"] for s in runs), len(runs)], "reached_by_first_side": sides,
                  "failed_seeds": sorted(s["seed"] for s in runs if not s["reached"])}
result["gate_right_swing_f2"] = gate
# runs ending in a deep hold, per set and kind
deep, terminal = {}, {}
sets = [("twins_a", "", ""), ("twins_chicane", "", ""), ("campaign_b", "", ""), ("grip", "_mu0.4", "_mu0.4"), ("grip", "_mu0.8", "_mu0.8")]
for s, bsuf, vsuf in sets:
    for kind in ("f2", "b5"):
        for target, held_runs in ((deep, lambda d, t: cv.deep_episodes(d, tag=t)),
                                  (terminal, lambda d, t: [r["run"] for r in cv.locks(d, tag=t) if r["kind"] == "deep"])):
            counts = {}
            for name, d, t in (("baseline", Path("experiment_runs/e1") / base_tag[kind] / s, base_tag[kind] + bsuf),
                               ("variant", variant / s, tag + vsuf)):
                groups, _ = cv.runs(d, t)
                n = sum(len(v) for (cell, k, mode), v in groups.items() if k == kind)
                held = [r for r in held_runs(d, t) if f"-{kind}-" in r]
                counts[name] = [len(held), n]
            if counts["baseline"][1] or counts["variant"][1]:
                kb, nb = counts["baseline"]
                kv, nv = counts["variant"]
                # fewer deep holds in the variant: P that the baseline has at least kb with the pooled count fixed
                counts["p_variant_fewer"] = cv.fisher_greater(kb, nb, kv, nv) if nb and nv else None
                target[f"{s}{vsuf}:{kind}"] = counts
result["deep_holds"] = deep
result["deep_holds_terminal"] = terminal
(out / "handoff.json").write_text(json.dumps(result, indent=1))
print(json.dumps(result, indent=1))
PY
exit "${failed}"
