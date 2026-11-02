#!/usr/bin/env bash
# Phase 5A recheck after the final review (I1: a same-goal Nav2 replan carries the published medium plan on to the new
# route_id at once): store_room x3 (its door S wedge followed route_id fallbacks), room_r4 and door_105 x1 as
# regression cells; F2 above B5 at 50 Hz, the 2.5 Hz profile, as campaign B. One run at a time, each in a new Play
# epoch with Play stopped for cool_s (default 120 s) after it; a run whose runner fails before its drive is retried
# once. Docker and the Unity editor with the project must be up; Play is stopped at the end.
#
#   caffeinate -i bash docs/experiments/medium-horizon/phase5a/recheck_replans.sh [skip [cool_s]]   # about 25 min
set -euo pipefail
cd "$(dirname "$0")/../../../.."

skip="${1:-0}"
cool_s="${2:-120}"
runs=(v2_store_room_nav v2_store_room_nav v2_store_room_nav v2_room_r4_nav v2_door_105_nav)

container="mm-motion-planning-ma-robot-sim-1"
# The description launch (robot_state_publisher and the static map -> odom) must run in the container.
if ! docker exec "${container}" pgrep -f "[s]imulation.launch.py" >/dev/null; then
  echo "starting the description launch"
  docker exec "${container}" bash -c 'source "$ROS_WS/install/setup.bash" && setsid nohup ros2 launch \
    mobile_manipulator_description simulation.launch.py > /tmp/mm_description.log 2>&1 & echo $! > /tmp/mm_description.pid'
  sleep 6
fi

attempt() {
  python3 tools/run_nav_scenario.py "$1" --controller wholebody --mpc-rate-hz 50 --medium bezier --new-epoch
}
index=0
for cell in "${runs[@]}"; do
  index=$((index + 1))
  if (( index <= skip )); then continue; fi
  echo "== run ${index}/${#runs[@]}: ${cell} $(date -u +%H:%M:%SZ)"
  if ! attempt "${cell}"; then
    echo "!! ${cell} failed before its drive (run ${index}); retrying"
    attempt "${cell}" || echo "!! ${cell} failed again (run ${index}); continuing"
  fi
  unity command editor_stop >/dev/null 2>&1 || true
  sleep "${cool_s}"
done
unity command editor_stop >/dev/null 2>&1 || true
echo "recheck: ${#runs[@]} runs done"
