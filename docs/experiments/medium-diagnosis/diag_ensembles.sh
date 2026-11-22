#!/usr/bin/env bash
# The medium diagnosis's ensembles on the MuJoCo plant (docs/superpowers/plans/2026-11-22-medium-diagnosis.md). A rerun
# skips finished runs. Never overlap with Unity Play or another timed ensemble (fanless host); power, lid open.
#
#   caffeinate -i bash docs/experiments/medium-diagnosis/diag_ensembles.sh baseline
#
# baseline  Task 2: E3's eight gap cells (corpus_e3_tight.json, their own scenario routes, as E3 ran them) and the two
#           cells of corpus_diag.json (R4's door, the 1.23 m low beam) on lattice routes; F2, realistic, the installed
#           0.05 m margins, seeds 1-20 (SEEDS=n overrides), with --prediction and --seed-detail, into
#           experiment_runs/medium-diag/baseline (OUT=dir overrides: a rerun after a changed cell or config goes to a
#           NEW directory)
# door      Task 3 (A3): corpus_door.json (R4's goal after a stop in the hallway: at C1's mouth, west, east), lattice
#           routes, the same flags, into experiment_runs/medium-diag/door
set -euo pipefail
cd "$(dirname "$0")/../../.."
stage="${1:?stage: baseline | door}"
jobs="${JOBS:-$(sysctl -n hw.perflevel0.physicalcpu 2>/dev/null || nproc)}"
seeds="${SEEDS:-20}"
here=docs/experiments/medium-diagnosis
case "${stage}" in
  baseline)
    out="${OUT:-experiment_runs/medium-diag/baseline}"
    common=(--mode realistic --kinds f2 --seeds "${seeds}" --jobs "${jobs}" --prediction --seed-detail --out "${out}")
    echo "== baseline: E3's gap cells =="
    python3 tools/plant/plant_campaign.py run --corpus docs/experiments/execution-fixes/corpus_e3_tight.json \
      --cells e3_gap_straight_1,e3_gap_straight_2,e3_gap_straight_3,e3_gap_straight_4,e3_gap_turn_1,e3_gap_turn_2,e3_gap_turn_3,e3_gap_turn_4 \
      --tag m050 "${common[@]}"
    echo "== baseline: R4's door and the low beam, lattice routes =="
    python3 tools/plant/plant_campaign.py run --corpus "${here}/corpus_diag.json" --routes lattice "${common[@]}" ;;
  door)
    out="${OUT:-experiment_runs/medium-diag/door}"
    python3 tools/plant/plant_campaign.py run --corpus "${here}/corpus_door.json" --routes lattice --mode realistic \
      --kinds f2 --seeds "${seeds}" --jobs "${jobs}" --prediction --seed-detail --out "${out}" ;;
  *) echo "stage: baseline | door" >&2; exit 2 ;;
esac
