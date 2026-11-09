#!/usr/bin/env bash
# E1 (gate and decision robustness) ensembles on the MuJoCo plant, realistic mode, 20 seeds, recorded routes
# (docs/superpowers/plans/2026-11-08-e1-gate-robustness.md Tasks 6, 7, 10). A rerun skips finished runs.
#
#   caffeinate -i bash docs/experiments/execution-fixes/e1_ensembles.sh STAGE [VARIANT_CONFIG TAG]
#
# Stages:
#   equivalence  the frozen code against the baseline replicas (A5): three cells rerun without the stall detector (the
#                replicas predate it) with configs/b5_frozen.yaml (the frozen gate and medium mode under the adopted
#                yaml) and compared frame by frame (divergence.py pair); EQ_OUT=dir overrides experiment_runs/e1/equivalence
#   twins        twin ensembles (A2's flip rate): campaign A's 7 cells and the grip corpus's chicane, F2 and B5
#   a3           the plain runs A3 needs beside the twins' plain ones: campaign B (F2) and the grip corpus at mu 0.4
#                and 0.8 (the replicas ran without the stall detector, so A3 compares with these, not with them)
#   variant      twins + a3 with a B5 config variant: VARIANT_CONFIG (repository-relative) and TAG (no hyphen)
# Without a variant the runs went to the frozen baseline in experiment_runs/e1/baseline (complete, 2026-11-08); since
# E1's acceptance the installed wholebody_mpc.yaml is the adopted B5, so `twins` and `a3` without a variant refuse to run
# (they would mix adopted runs into the frozen set): a new frozen set is `variant
# docs/experiments/execution-fixes/configs/b5_frozen.yaml frozen`. A variant goes to experiment_runs/e1/<TAG>. Jobs: every performance core (JOBS=n overrides). Never overlap with Unity Play or another
# timed ensemble (fanless host).
set -euo pipefail
cd "$(dirname "$0")/../../.."

stage="${1:?stage: equivalence | twins | a3 | variant}"
container="mm-motion-planning-ma-robot-sim-1"
jobs="${JOBS:-$(sysctl -n hw.perflevel0.physicalcpu 2>/dev/null || nproc)}"
kinds="${KINDS:-f2,b5}"  # KINDS=f2 for a variant that only changes the medium mode
plant=docs/experiments/mujoco-plant
seeds=20
variant=() tag="" out=experiment_runs/e1/baseline
if [ "${stage}" = variant ]; then
  config="${2:?variant config}" tag="${3:?variant tag}"
  variant=(--b5-config "${config}" --tag "${tag}")
  out="experiment_runs/e1/${tag}"
fi
failed=0
if [ -z "${tag}" ] && { [ "${stage}" = twins ] || [ "${stage}" = a3 ]; }; then
  echo "stage ${stage} without a variant would run the adopted B5 into the frozen baseline (${out}); use" >&2
  echo "  variant docs/experiments/execution-fixes/configs/b5_frozen.yaml frozen" >&2
  exit 2
fi
echo "stage ${stage}, jobs ${jobs}, out ${out}"

profiles() {  # the grip variants of the installed profile (as replicas.sh makes them)
  mkdir -p experiment_runs/e1/profiles
  for mu in 0.4 0.8; do
    docker exec "${container}" bash -c "installed=\$ROS_WS/install/mobile_manipulator_plant/share/mobile_manipulator_plant/robots/mobile_manipulator; \
      sed -e 's/^  contact: {mu: 0.6, /  contact: {mu: ${mu}, /' -e \"s|^model: robot.xml|model: \$installed/robot.xml|\" \
          -e \"s|^model_wheels: robot_wheels.xml|model_wheels: \$installed/robot_wheels.xml|\" \
          \$installed/profile.yaml > /workspaces/mm-motion-planning/experiment_runs/e1/profiles/profile_mu${mu}.yaml"
    grep -q "contact: {mu: ${mu}," "experiment_runs/e1/profiles/profile_mu${mu}.yaml"
  done
}

twins() {
  python3 tools/plant/divergence.py ensemble --corpus "${plant}/corpus_campaign_a.json" --seeds "${seeds}" --jobs "${jobs}" --kinds "${kinds}" \
    --out "${out}/twins_a" ${variant[@]+"${variant[@]}"} || failed=1
  python3 tools/plant/divergence.py ensemble --corpus "${plant}/corpus_grip.json" --cells v2_chicane_nav --seeds "${seeds}" \
    --jobs "${jobs}" --kinds "${kinds}" --out "${out}/twins_chicane" ${variant[@]+"${variant[@]}"} || failed=1
}

a3() {
  python3 tools/plant/plant_campaign.py run --corpus "${plant}/corpus_campaign_b.json" --mode realistic --seeds "${seeds}" \
    --kinds f2 --jobs "${jobs}" --out "${out}/campaign_b" ${variant[@]+"${variant[@]}"} || failed=1
  profiles
  for mu in 0.4 0.8; do
    local mtag="mu${mu}"
    local args=(--profile "experiment_runs/e1/profiles/profile_mu${mu}.yaml" --tag "${mtag}")
    if [ -n "${tag}" ]; then args=(--profile "experiment_runs/e1/profiles/profile_mu${mu}.yaml" --b5-config "${config}" --tag "${tag}_${mtag}"); fi
    python3 tools/plant/plant_campaign.py run --corpus "${plant}/corpus_grip.json" --mode realistic --seeds "${seeds}" \
      --kinds "${kinds}" --jobs "${jobs}" --out "${out}/grip" "${args[@]}" || failed=1
  done
}

equivalence() {  # the replicas' runs at the current code, no stall detector (stall_s 0) as the replicas ran
  local eq=${EQ_OUT:-experiment_runs/e1/equivalence} base=experiment_runs/plant-replicas
  local binary=/workspaces/mm-motion-planning/ros2_ws/install/mobile_manipulator_campaign/lib/mobile_manipulator_campaign/plant_campaign
  local cells=("v2_store_room_nav b5 1 ${plant}/corpus_campaign_a.json" "v2_gate_right_swing_nav f2 4 ${plant}/corpus_campaign_a.json"
               "v2_door_105_mission f2 20 ${plant}/corpus_campaign_b.json")
  for entry in "${cells[@]}"; do
    read -r cell kind seed corpus <<< "${entry}"
    local bag; bag=$(python3 -c "import json,sys; print([c for c in json.load(open('${corpus}'))['cells'] if c['cell']=='${cell}'][0]['bag'])")
    local name="${cell}-${kind}-realistic-s${seed}"
    local only=(); [ "${kind}" = b5 ] && only=(--b5-only)
    if [ ! -f "${eq}/${name}/summary.json" ]; then
      docker exec "${container}" bash -lc "cd /workspaces/mm-motion-planning && source ros2_ws/install/setup.bash && \
        ${binary} --scenario ${cell} --mode realistic --seed ${seed} --bag ${bag} ${only[*]+${only[*]}} --stall-s 0 \
        --b5-config docs/experiments/execution-fixes/configs/b5_frozen.yaml --out ${eq}/${name}" || failed=1
    fi
    python3 tools/plant/divergence.py pair "${base}/${name}/run.mmlog" "${eq}/${name}/run.mmlog" > "${eq}/${name}.pair.json" \
      || { failed=1; continue; }
    python3 - "${eq}/${name}.pair.json" <<'EOF' || failed=1
import json, sys
r = json.load(open(sys.argv[1]))
same = r["first"] is None and r["truncated"] is None and r["distance_end"] and r["distance_end"]["base_m"] == 0.0 \
    and r["outcomes"]["a"] == r["outcomes"]["b"]
print(f"{sys.argv[1]}: {'EQUIVALENT' if same else 'DIFFERS'}; first {r['first']}; end {r['distance_end']}; "
      f"skipped {r['skipped']}; outcomes {r['outcomes']}")
sys.exit(0 if same else 1)
EOF
  done
}

case "${stage}" in
  equivalence) equivalence ;;
  twins) twins ;;
  a3) a3 ;;
  variant) twins; a3 ;;
  *) echo "unknown stage ${stage}"; exit 2 ;;
esac
[ "${failed}" -eq 0 ] || { echo "stage ${stage}: some runs or checks failed"; exit 1; }
echo "stage ${stage}: done"
