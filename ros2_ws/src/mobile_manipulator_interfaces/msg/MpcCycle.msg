# Diagnostics of one whole-body MPC cycle (wholebody_mpc_node, baseline B5).
std_msgs/Header header
# ok, fallback, braking, failed, goal_reached, stale_state or holding (mobile_manipulator_mpc::CycleStatus;
# holding: the solved plan failed the slack gate and was not executed, the robot brakes and re-plans;
# a solver failure during a hold also reports holding, with zero slack, and counts as a failure).
string status
# Seconds. linearize: advancing the nominal plan; collision: distance rows and the home clearance;
# setup: the QP build; solve: the solver's update and solve; total: the node's whole cycle (goal
# handling, state read, controller step, publishing); state_to_command: from reading the state to
# publishing the base command.
float64 linearize_s
float64 collision_s
float64 setup_s
float64 solve_s
float64 total_s
float64 state_to_command_s
int32 iterations
# Collision rows used this cycle, and pairs within the activation distance beyond the per-stage budget.
int32 active_pairs
int32 dropped_pairs
# Largest collision-row slack of the solution (m).
float64 max_slack
# Slack gate of a solved cycle with the gate on (m; 0 and false otherwise: with the gate off no pair
# distance is computed, as in the recorded B5's cycle): of the rows within the gate's horizon,
# the slack of the one that exceeds its allowance the most, and its pair's current violation
# (max(0, margin - its distance at the measured state)); with no slack beyond its allowance, the
# largest slack there and 0. The allowance is the tolerance for a pair outside the margin (violation
# 0) and the violation for a pair inside it (no deeper). gate_unsafe: some slack exceeds its allowance
# (the cycle holds).
float64 gate_slack
float64 gate_violation
bool gate_unsafe
# Translational manipulability sqrt(det(J J^T)) of tool0 at the measured state.
float64 manipulability
# Posture preference blend alpha in [0, 1] (0 prefers home, 1 vertical carry).
float64 tightness
# total_s exceeded the cycle period (counted as a controller failure).
bool deadline_missed
# Arm reference (ruling R24): the largest |reference - measured| over the six joints (rad; the
# JTC tracking error of the reference the stream continues), and whether this cycle re-anchored
# the MPC's arm state on the measured joints (that offset above 0.15 rad with fresh joint states).
float64 arm_reference_offset
bool arm_reanchored
