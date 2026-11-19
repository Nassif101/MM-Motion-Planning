#!/usr/bin/env bash
# Spec 2 Task 14 (docs/superpowers/plans/2026-11-18-plant-route-planner.md): the route planner's wall-time cost on
# this host. One job at a time, Unity closed, power and lid open:
#
#   caffeinate -i bash docs/experiments/plant-routing/task14_measure.sh
#
# headless  unpaced lattice runs with --profile-timing, one at a time: the cells with the longest searches
#           (services_deadend, store_room), a short one (open_yard), lidar off and on. The summaries' `timing`
#           has the planner setup (routing_setup_ms), each search tick (route_search_ms), the plant step and
#           B5's cycle; `real_time_factor` includes the search work.
# paced     a generated-route demo server run at 1x real time for 60 s of simulation, lidar on, with a goal that
#           needs the longest search: counts `behind` events and the achieved simulation speed.
#
# OUT=dir (default experiment_runs/plant-routing-task14; a rerun after a code change goes to a NEW directory).
set -euo pipefail
cd "$(dirname "$0")/../../.."

out="${OUT:-experiment_runs/plant-routing-task14}"
container=mm-motion-planning-ma-robot-sim-1
ws=/workspaces/mm-motion-planning
bin="${ws}/ros2_ws/install/mobile_manipulator_campaign/lib/mobile_manipulator_campaign"
mkdir -p "${out}"

if pgrep -x Unity >/dev/null; then echo "Unity is running: close it before timed runs" >&2; exit 2; fi

for cell in v2_services_deadend_nav v2_store_room_nav v2_open_yard_nav; do
  for lidar in off on; do
    dir="${out}/${cell}-lidar_${lidar}"
    [ -f "${dir}/summary.json" ] && continue
    extra=()
    [ "${lidar}" = on ] && extra+=(--lidar)
    echo "== ${cell} lidar ${lidar} =="
    docker exec "${container}" bash -lc "cd ${ws} && source ros2_ws/install/setup.bash && \
      ${bin}/plant_campaign --scenario ${cell} --mode realistic --seed 1 --routes lattice --profile-timing \
      ${extra[*]:-} --out ${ws}/${dir}" > "${dir}.log" 2>&1
  done
done

if [ ! -f "${out}/paced.json" ]; then
  echo "== paced demo server, 1x, lidar on =="
  docker exec -d "${container}" bash -lc "cd ${ws} && source ros2_ws/install/setup.bash && \
    exec ${bin}/plant_demo_server --mode realistic --routes lattice --lidar > /tmp/task14_demo_server.log 2>&1"
  sleep 3
  python3 - "${out}/paced.json" <<'EOF'
import json, sys, time
sys.path.insert(0, "tools/plant")
import plant_stream as ps
c = ps.Client("localhost", 7700, timeout=30.0)
c.command("reset", scenario="v2_services_deadend_nav", seed=1)
c.command("submit_goal", target=[16.3, -16.3, 1.570796], client_request=1)  # the scenario's goal
c.command("run", speed=1.0)
behind, t_sim, wall0, kinds = [], 0.0, time.monotonic(), {}
while t_sim < 60.0:
    kind, header, _ = c.next(timeout=30.0)
    kinds[kind] = kinds.get(kind, 0) + 1
    if kind == ps.STATE:
        t_sim = header.get("t", t_sim)
    elif kind == ps.EVENT and header.get("event") == "behind":
        behind.append(header)
wall = time.monotonic() - wall0
c.command("pause")
c.close()
json.dump({"simulated_s": t_sim, "wall_s": wall, "achieved_speed": t_sim / wall, "behind_events": behind,
           "frames": {str(k): v for k, v in kinds.items()}}, open(sys.argv[1], "w"), indent=1)
print(f"paced: {t_sim:.1f} s simulated in {wall:.1f} s wall ({t_sim / wall:.3f}x), {len(behind)} behind events")
EOF
  docker exec "${container}" pkill -INT -f "[p]lant_demo_server --mode realistic --routes lattice" || true
fi
