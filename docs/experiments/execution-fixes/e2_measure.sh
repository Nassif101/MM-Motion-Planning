#!/usr/bin/env bash
# E2 stage 1 measurement runs on the MuJoCo plant, realistic mode (docs/superpowers/plans/2026-11-11-e2-tracking-
# prediction.md Task 4). Every closed-loop run records the diagnostics (--diagnostics) and uses its baseline's B5
# config, so it reproduces that run's frames; a rerun skips finished runs.
#
#   caffeinate -i bash docs/experiments/execution-fixes/e2_measure.sh STAGE
#
# Stages (outputs under experiment_runs/e2/measure/, git-ignored):
#   ratchet     E1's five depth-ratchet runs (experiment_runs/e1/ratchet), each compared with its original frame by
#               frame (model_rollout.py same-run: identical apart from the diagnostics, or the stage fails)
#   store_room  B5 alone, store_room seeds 1-5 (baseline experiment_runs/e1/abs: 0 of 20 reach), compared likewise
#   gate        F2 gate_right_swing: the failing seeds 4, 7, 11, 19 and the passing 1, 2, 3, 5 of experiment_runs/e1/absinf
#               (the swing side is not in the logs; E1's left-start failures were seeds 4, 7, 19), compared likewise
#   excite      the open-loop excitation through plant_replay and its reports (model_rollout.py excite)
#   arm_ff      T8: the arm's tracking with the motion feedforward on and off (model_rollout.py arm-ff)
#   report      the rollout reports over ratchet, store_room and gate (model_rollout.py report)
#   all         every stage in this order
# Jobs: every performance core (JOBS=n overrides). Never overlap with Unity Play or another timed run (fanless host);
# power and lid open (a host sleep stalls the runs).
set -euo pipefail
cd "$(dirname "$0")/../../.."

stage="${1:?stage: ratchet | store_room | gate | excite | arm_ff | report | all}"
container="mm-motion-planning-ma-robot-sim-1"
workspace=/workspaces/mm-motion-planning
jobs="${JOBS:-$(sysctl -n hw.perflevel0.physicalcpu 2>/dev/null || nproc)}"
plant=docs/experiments/mujoco-plant
configs=docs/experiments/execution-fixes/configs
out=experiment_runs/e2/measure
failed=0

campaign() {  # corpus, out dir, then plant_campaign.py run options
  local corpus="$1" dir="$2"
  shift 2
  python3 tools/plant/plant_campaign.py run --corpus "${plant}/${corpus}" --mode realistic --jobs "${jobs}" --diagnostics \
    --out "${dir}" "$@" || failed=1
}

same() {  # a rerun against its original
  if ! python3 tools/plant/model_rollout.py same-run "$1/run.mmlog" "$2/run.mmlog"; then
    echo "  $1 does not reproduce $2" >&2
    failed=1
  fi
}

ratchet() {
  local dir="${out}/ratchet" e1=experiment_runs/e1/ratchet
  campaign corpus_campaign_b.json "${dir}" --cells v2_door_105_mission --kinds f2 --seed-list 20 \
    --b5-config "${configs}/b5_frozen.yaml" --tag frozen
  campaign corpus_campaign_b.json "${dir}" --cells v2_door_105_mission --kinds f2 --seed-list 20 \
    --b5-config "${configs}/b5_absolute.yaml" --tag abs
  campaign corpus_campaign_a.json "${dir}" --cells v2_store_room_nav --kinds f2 --seed-list 18 \
    --b5-config "${configs}/b5_absolute.yaml" --tag abs
  campaign corpus_campaign_a.json "${dir}" --cells v2_door_then_beam_nav --kinds f2 --seed-list 5,15 --start-nudge 1e-12 \
    --b5-config "${configs}/b5_frozen.yaml" --tag frozen
  same "${dir}/v2_door_105_mission-f2-realistic-frozen-s20" "${e1}/v2_door_105_mission-f2-realistic-s20"
  same "${dir}/v2_door_105_mission-f2-realistic-abs-s20" "${e1}/v2_door_105_mission-f2-realistic-abs-s20"
  same "${dir}/v2_store_room_nav-f2-realistic-abs-s18" "${e1}/v2_store_room_nav-f2-realistic-abs-s18"
  same "${dir}/v2_door_then_beam_nav-f2-realistic-frozen-s5-n" "${e1}/v2_door_then_beam_nav-f2-realistic-s5-n"
  same "${dir}/v2_door_then_beam_nav-f2-realistic-frozen-s15-n" "${e1}/v2_door_then_beam_nav-f2-realistic-s15-n"
}

store_room() {
  local dir="${out}/store_room"
  campaign corpus_campaign_a.json "${dir}" --cells v2_store_room_nav --kinds b5 --seed-list 1,2,3,4,5 \
    --b5-config "${configs}/b5_absolute.yaml" --tag abs
  for s in 1 2 3 4 5; do
    same "${dir}/v2_store_room_nav-b5-realistic-abs-s${s}" "experiment_runs/e1/abs/twins_a/v2_store_room_nav-b5-realistic-abs-s${s}"
  done
}

gate() {
  local dir="${out}/gate"
  campaign corpus_campaign_a.json "${dir}" --cells v2_gate_right_swing_nav --kinds f2 --seed-list 4,7,11,19,1,2,3,5 \
    --b5-config "${configs}/b5_abs_inf.yaml" --tag absinf
  for s in 4 7 11 19 1 2 3 5; do
    same "${dir}/v2_gate_right_swing_nav-f2-realistic-absinf-s${s}" \
      "experiment_runs/e1/absinf/twins_a/v2_gate_right_swing_nav-f2-realistic-absinf-s${s}"
  done
}

in_container() {
  docker exec "${container}" bash -lc "source /opt/ros/jazzy/setup.bash && source ${workspace}/ros2_ws/install/setup.bash \
    && cd ${workspace}/tools/plant && $*" || failed=1
}

excite() { in_container python3 model_rollout.py excite --out "${workspace}/${out}/excite"; }
arm_ff() { in_container python3 model_rollout.py arm-ff --out "${workspace}/${out}/arm_ff"; }

report() {
  local logs
  logs=$(cd "${out}" && ls -d ratchet/*/run.mmlog store_room/*/run.mmlog gate/*/run.mmlog | sed "s|^|${workspace}/${out}/|" | tr '\n' ' ')
  in_container python3 model_rollout.py report ${logs} --out "${workspace}/${out}/report" --container none
}

echo "stage ${stage}, jobs ${jobs}, out ${out}"
case "${stage}" in
  ratchet | store_room | gate | excite | arm_ff | report) "${stage}" ;;
  all) ratchet; store_room; gate; excite; arm_ff; report ;;
  *) echo "unknown stage ${stage}" >&2; exit 2 ;;
esac
exit "${failed}"
