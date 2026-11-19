#!/usr/bin/env bash
# Spec 2 Task 13 (docs/superpowers/plans/2026-11-18-plant-route-planner.md): the V2 corpus under both route
# conditions, kept apart. Campaign B's 19 V2 cells (docs/experiments/mujoco-plant/corpus_campaign_b.json), realistic
# plant, F2 above B5, the same seeds, once with the recorded Nav2 routes (the cells' bags) and once with routes the
# Smac Lattice port plans in the session (--routes lattice; run names carry "-lattice").
# A rerun skips finished runs. Never overlap with Unity Play or another timed ensemble (fanless host); power, lid open.
#
#   caffeinate -i bash docs/experiments/plant-routing/task13_conditions.sh
#
# SEEDS=n (default 5), JOBS=n (default: the performance cores), OUT=dir (default experiment_runs/plant-routing-task13;
# a rerun after a code or config change goes to a NEW directory, never into an old one).
set -euo pipefail
cd "$(dirname "$0")/../../.."

jobs="${JOBS:-$(sysctl -n hw.perflevel0.physicalcpu 2>/dev/null || nproc)}"
seeds="${SEEDS:-5}"
out="${OUT:-experiment_runs/plant-routing-task13}"
corpus=docs/experiments/mujoco-plant/corpus_campaign_b.json
failed=0

for routes in recorded lattice; do
  echo "== routes ${routes} =="
  python3 tools/plant/plant_campaign.py run --corpus "${corpus}" --mode realistic --seeds "${seeds}" --kinds f2 \
    --jobs "${jobs}" --routes "${routes}" --out "${out}" || failed=1
done
python3 tools/plant/plant_campaign.py table --out "${out}" > "${out}/table.md" || failed=1
exit "${failed}"
