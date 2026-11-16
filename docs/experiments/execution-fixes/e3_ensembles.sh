#!/usr/bin/env bash
# E3 ensembles on the MuJoCo plant (docs/superpowers/plans/2026-11-15-e3-learned-margin.md Task 4; spec Amendment 1).
# A rerun skips finished runs. Never overlap with Unity Play or another timed ensemble (fanless host); power, lid open.
#
#   caffeinate -i bash docs/experiments/execution-fixes/e3_ensembles.sh sweep realistic|ideal
#
# sweep  the fixed-margin sweep on the tightness cells (corpus_e3_tight.json): arm and panel margin 0.05, 0.04, 0.03,
#        0.02 m in B5 and the medium planner alike, F2, seeds 1-20 (SEEDS=n overrides), into experiment_runs/e3/sweep
#        (OUT=dir overrides: a rerun after a changed cell or config goes to a NEW directory, never into an old one).
#        realistic  tags m050 m040 m030 m020; the row trace on m050 and m030; the plant runs' default tracking model
#        ideal      the plant executing the commands as sent, B5 with the unicycle (--tracking-model none); tags i050
#                   i040 i030 i020; the row trace on i050 (the check that the tracking error is near zero there)
set -euo pipefail
cd "$(dirname "$0")/../../.."

stage="${1:?stage: sweep}"
jobs="${JOBS:-$(sysctl -n hw.perflevel0.physicalcpu 2>/dev/null || nproc)}"
seeds="${SEEDS:-20}"
configs=docs/experiments/execution-fixes/configs
corpus=docs/experiments/execution-fixes/corpus_e3_tight.json

case "${stage}" in
  sweep)
    leg="${2:?leg: realistic | ideal}"
    out="${OUT:-experiment_runs/e3/sweep}"
    for mm in 050 040 030 020; do
      args=(--corpus "${corpus}" --seeds "${seeds}" --jobs "${jobs}" --kinds f2 --out "${out}")
      # the installed configs are the 0.05 m margin; the others are their copies with the one value changed
      if [ "${mm}" != 050 ]; then
        args+=(--b5-config "${configs}/b5_margin_${mm}.yaml" --medium-config "${configs}/medium_margin_${mm}.yaml")
      fi
      case "${leg}" in
        realistic)
          args+=(--mode realistic --tag "m${mm}")
          if [ "${mm}" = 050 ] || [ "${mm}" = 030 ]; then args+=(--row-trace); fi ;;
        ideal)
          args+=(--mode ideal --tracking-model none --tag "i${mm}")
          if [ "${mm}" = 050 ]; then args+=(--row-trace); fi ;;
        *) echo "leg: realistic | ideal" >&2; exit 2 ;;
      esac
      echo "== sweep ${leg} margin 0.${mm} m =="
      python3 tools/plant/plant_campaign.py run "${args[@]}"
    done ;;
  *) echo "stage: sweep" >&2; exit 2 ;;
esac
