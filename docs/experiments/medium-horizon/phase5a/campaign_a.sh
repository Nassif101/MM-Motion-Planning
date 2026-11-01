#!/usr/bin/env bash
# Phase 5A campaign A (plan Task 13, spec Section 7.1): the seven acceptance cells, F2 (the medium layer above B5,
# --medium bezier) against B5 alone (--medium none), three runs each, B5 at 50 Hz with the base friction feedforward on
# (the runner's default). Interleaved: every round runs every cell once in each mode, the mode that goes first alternating
# by round, so slow drift (thermals, a long-running stack) does not fall on one mode. One run at a time; nothing else
# may load the host (a fanless MacBook Air: never overlap Play with other work). Docker and the Unity editor with the
# project must be up; Play is stopped at the end.
#
#   caffeinate -i bash docs/experiments/medium-horizon/phase5a/campaign_a.sh [rounds [skip]]   # about 1.75 h for 3
#
# skip: the number of runs (in the campaign's order) already recorded, to resume an interrupted campaign. A run whose
# runner fails before the drive (an infrastructure failure, such as the arm left where MoveIt cannot plan by a run that
# stalled in a door) is retried once in a new Play epoch, and the run after a failure starts a new epoch, as the Phase
# 4B driver did; a drive that fails is a result, not retried.
set -euo pipefail
cd "$(dirname "$0")/../../../.."

rounds="${1:-3}"
skip="${2:-0}"
cells=(v2_room_r4_nav v2_store_room_nav v2_gate_right_swing_nav v2_door_105_nav v2_low_beam_123_nav v2_low_beam_130_nav
       v2_door_then_beam_nav)

container="mm-motion-planning-ma-robot-sim-1"
# The description launch (robot_state_publisher and the static map -> odom) must run in the container: the runner
# expects it (tools/demo.py up starts it); without it no costmap gets its map -> base_footprint transform.
if ! docker exec "${container}" pgrep -f "[s]imulation.launch.py" >/dev/null; then
  echo "starting the description launch"
  docker exec "${container}" bash -c 'source "$ROS_WS/install/setup.bash" && setsid nohup ros2 launch \
    mobile_manipulator_description simulation.launch.py > /tmp/mm_description.log 2>&1 & echo $! > /tmp/mm_description.pid'
  sleep 6
fi

epoch=(--new-epoch)  # the first run restarts Play and arm control; the others continue in that Play epoch
index=0
attempt() {
  python3 tools/run_nav_scenario.py "$1" --controller wholebody --mpc-rate-hz 50 --medium "$2" ${epoch[@]+"${epoch[@]}"}
}
run() {
  local cell="$1" medium="$2"
  index=$((index + 1))
  if (( index <= skip )); then return; fi
  echo "== round ${round} (run ${index}): ${cell} --medium ${medium}"
  if attempt "${cell}" "${medium}"; then epoch=(); return; fi
  echo "!! ${cell} --medium ${medium} failed before its drive (round ${round}); retrying in a new Play epoch"
  epoch=(--new-epoch)
  if attempt "${cell}" "${medium}"; then epoch=(); return; fi
  echo "!! ${cell} --medium ${medium} failed again (round ${round}); continuing in a new Play epoch"
}

for round in $(seq 1 "${rounds}"); do
  if (( round % 2 == 1 )); then order=(bezier none); else order=(none bezier); fi
  for cell in "${cells[@]}"; do
    for medium in "${order[@]}"; do run "${cell}" "${medium}"; done
  done
done
unity command editor_stop >/dev/null 2>&1 || true
echo "campaign A: ${rounds} rounds done"
