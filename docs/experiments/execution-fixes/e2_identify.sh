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
# arm_delay_s left out since the Task 8 review (C1, user ruling 2026-11-13): B5's arm state is the streamed reference, so
# the joints' lag behind it is not a delay of B5's prediction. The 0.04 s fit stays in e2.md as a measurement.
TERMS=turn_delta_inf,turn_delta_zero,turn_scale,speed_turn,sideslip
LOGS=$(ls $R/experiment_runs/e2/measure/{ratchet,store_room,gate}/*/run.mmlog)
echo "closed-loop fit"; python3 identify_tracking.py fit --source closed-loop --logs $LOGS --terms $TERMS --out $OUT/tracking_closed_loop.yaml --report $OUT --date 2026-11-12
echo "excitation fit (records saved)"; python3 identify_tracking.py fit --source excitation --terms $TERMS --out $OUT/tracking_excitation.yaml --report $OUT --date 2026-11-12
echo "excitation refit from the saved records (determinism)"; python3 identify_tracking.py fit --source excitation --from-records $OUT/excitation_records.json.gz --terms $TERMS --out $OUT/tracking_excitation_again.yaml --report $OUT/again --date 2026-11-12
cmp $OUT/tracking_excitation.yaml $OUT/tracking_excitation_again.yaml && echo "deterministic"
for mu in 0.4 0.8; do
  GLOGS=$(ls $R/experiment_runs/e2/grip/mu$mu/*/run.mmlog)
  echo "closed-loop fit mu $mu"; python3 identify_tracking.py fit --source closed-loop --logs $GLOGS --terms $TERMS --out $OUT/tracking_closed_loop_mu$mu.yaml --report $OUT/mu$mu --date 2026-11-12
  echo "check the mu 0.6 model on mu $mu"; python3 identify_tracking.py check --model $OUT/tracking_closed_loop.yaml --logs $GLOGS --report $OUT/check_mu0.6_model_on_mu$mu
done
for m in closed_loop excitation; do echo "check $m"; python3 identify_tracking.py check --model $OUT/tracking_$m.yaml --logs $LOGS --report $OUT/check_$m; done
echo done
