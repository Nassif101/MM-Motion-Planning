#!/usr/bin/env bash
# Phase 5A campaign B (plan Task 14, spec Section 7.2): F2 (the medium layer above B5, --medium bezier) on campaign 2's
# exit set (the 34 runs of docs/experiments/wholebody-mpc/phase4b-b.md, 2026-10-24), B5 at 50 Hz with the base friction
# feedforward on (the runner's default since Phase 5A Task 1: the base change against campaign 2). Campaign 2's three
# scripted-reference runs of constrained_reconfiguration_mission run unscripted: the medium layer gives B5 its references
# (it refuses scripted ones), so that cell has five runs. One run at a time, each in a new Play epoch with Play stopped
# for cool_s (default 120 s) after it (the fanless MacBook Air); a run whose runner fails before its drive is retried
# once. Docker and the Unity editor with the project must be up; Play is stopped at the end.
#
#   caffeinate -i bash docs/experiments/medium-horizon/phase5a/campaign_b.sh [skip [cool_s]]   # about 3 h
set -euo pipefail
cd "$(dirname "$0")/../../../.."

skip="${1:-0}"
cool_s="${2:-120}"
runs=(constrained_reconfiguration_mission constrained_reconfiguration_mission constrained_reconfiguration_mission
      constrained_reconfiguration_mission constrained_reconfiguration_mission
      narrow_gate_home_nav narrow_gate_home_nav overhead_course_nav overhead_course_nav
      overhead_course_worker_nav overhead_course_worker_nav
      v2_chicane_nav v2_door_105_mission v2_door_105_nav v2_door_130_nav v2_gate_right_swing_nav v2_goal_near_skip_nav
      v2_open_yard_nav v2_room_r1_nav v2_room_r4_nav v2_room_r4_nav v2_room_r4_nav v2_scaffold_tunnel_mission
      v2_scaffold_tunnel_nav v2_services_deadend_nav v2_spine_nav v2_store_room_nav v2_store_room_nav v2_store_room_nav
      v2_through_building_mission v2_worker_crossing_nav v2_worker_late_crossing_nav v2_worker_stops_in_lane_nav
      v2_yard_store_nav)

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
echo "campaign B: ${#runs[@]} runs done"
