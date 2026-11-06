#!/usr/bin/env bash
# MuJoCo plant plan Task 11 (gate R, amended 2026-11-06 Section 6): campaigns A and B replicated on the plant.
# - realistic mode (the gate): campaign A (7 cells x F2/B5) and campaign B's 19 V2 cells (F2), 20 perturbed seeds a
#   cell and layer;
# - ideal mode (informational): the same with 5 seeds;
# - the grip ensemble (CP3b): door_105, low_beam_123, chicane, store_room, F2/B5 at contact_mu 0.4 and 0.8, 20 seeds.
# Runs go to experiment_runs/plant-replicas; a rerun skips the finished ones. Jobs: the host's performance cores
# minus one (override with JOBS=n). Interrupting stops the container's runs too. Then:
#   python3 tools/plant/compare_campaigns.py --out experiment_runs/plant-replicas --report docs/experiments/mujoco-plant/replicas.md
#
#   caffeinate -i bash docs/experiments/mujoco-plant/replicas.sh
set -euo pipefail
cd "$(dirname "$0")/../../.."

container="mm-motion-planning-ma-robot-sim-1"
jobs="${JOBS:-$(( $(sysctl -n hw.perflevel0.physicalcpu 2>/dev/null || nproc) - 1 ))}"
out=experiment_runs/plant-replicas
docs=docs/experiments/mujoco-plant
trap 'docker exec "${container}" pkill -INT -f "[p]lant_campaign --scenario" || true; exit 130' INT TERM

run() { python3 tools/plant/plant_campaign.py run --jobs "${jobs}" --out "${out}" "$@"; }
echo "jobs ${jobs}"
run --corpus "${docs}/corpus_campaign_a.json" --mode realistic --seeds 20
run --corpus "${docs}/corpus_campaign_b.json" --mode realistic --seeds 20 --kinds f2
mkdir -p "${out}/profiles"
for mu in 0.4 0.8; do
  # the variant lives elsewhere: its generated models by absolute container path (the profile's are beside it)
  robots=/workspaces/mm-motion-planning/ros2_ws/src/mobile_manipulator_plant/robots/mobile_manipulator
  sed -e "s/^  contact: {mu: 0.6, /  contact: {mu: ${mu}, /" -e "s|^model: robot.xml |model: ${robots}/robot.xml |" \
      -e "s|^model_wheels: robot_wheels.xml |model_wheels: ${robots}/robot_wheels.xml |" \
      ros2_ws/src/mobile_manipulator_plant/robots/mobile_manipulator/profile.yaml > "${out}/profiles/profile_mu${mu}.yaml"
  grep -q "contact: {mu: ${mu}," "${out}/profiles/profile_mu${mu}.yaml"
  run --corpus "${docs}/corpus_grip.json" --mode realistic --seeds 20 --profile "${out}/profiles/profile_mu${mu}.yaml" --tag "mu${mu}"
done
run --corpus "${docs}/corpus_campaign_a.json" --mode ideal --seeds 5
run --corpus "${docs}/corpus_campaign_b.json" --mode ideal --seeds 5 --kinds f2
echo "done: $(ls -d ${out}/*/summary.json 2>/dev/null | wc -l) runs"
