#!/usr/bin/env bash
# E2 Task 6: the identification runs, inside the container:
#   docker exec mm-motion-planning-ma-robot-sim-1 bash /workspaces/mm-motion-planning/docs/experiments/execution-fixes/e2_identify.sh
# Outputs in experiment_runs/e2/ident (git-ignored); the candidate model is copied to
# ros2_ws/src/mobile_manipulator_mpc/config/tracking_model_mujoco.yaml by hand after the CP2 ruling.
R=/workspaces/mm-motion-planning
source /opt/ros/jazzy/setup.bash && source $R/ros2_ws/install/setup.bash
set -euo pipefail
cd $R/tools/plant
OUT=$R/experiment_runs/e2/ident
mkdir -p $OUT
TERMS=turn_delta_inf,turn_delta_zero,turn_scale,speed_turn,sideslip,arm_delay_s
LOGS=$(ls $R/experiment_runs/e2/measure/{ratchet,store_room,gate}/*/run.mmlog)
echo "closed-loop fit"; python3 identify_tracking.py fit --source closed-loop --logs $LOGS --terms $TERMS --out $OUT/tracking_closed_loop.yaml --report $OUT --date 2026-11-12
echo "excitation fit"; python3 identify_tracking.py fit --source excitation --terms $TERMS --out $OUT/tracking_excitation.yaml --report $OUT --date 2026-11-12
echo "excitation fit again (determinism)"; python3 identify_tracking.py fit --source excitation --terms $TERMS --out $OUT/tracking_excitation_again.yaml --report $OUT/again --date 2026-11-12
cmp $OUT/tracking_excitation.yaml $OUT/tracking_excitation_again.yaml && echo "deterministic"
echo "grip"; python3 identify_tracking.py grip --terms turn_delta_inf,turn_delta_zero,turn_scale,speed_turn,sideslip --report $OUT
for m in closed_loop excitation; do echo "check $m"; python3 identify_tracking.py check --model $OUT/tracking_$m.yaml --logs $LOGS --report $OUT/check_$m; done
echo done
