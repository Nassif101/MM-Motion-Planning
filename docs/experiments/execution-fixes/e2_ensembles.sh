#!/usr/bin/env bash
# E2 ensembles on the MuJoCo plant: e1_ensembles.sh's stages with the variants written under experiment_runs/e2/<TAG>
# (plan Tasks 4b and 10). The baselines stay experiment_runs/e1/absinf (F2) and experiment_runs/e1/abs (B5 alone).
#
#   caffeinate -i bash docs/experiments/execution-fixes/e2_ensembles.sh variant CONFIG TAG
#
# Task 4b's T10: variant docs/experiments/execution-fixes/configs/b5_follow_plan.yaml follow (about 1 h 50 at 4 jobs).
set -euo pipefail
cd "$(dirname "$0")/../../.."
OUT_ROOT=experiment_runs/e2 exec bash docs/experiments/execution-fixes/e1_ensembles.sh "$@"
