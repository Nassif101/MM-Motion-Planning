# Diagnostics of one whole-body MPC cycle (wholebody_mpc_node, baseline B5).
std_msgs/Header header
# ok, fallback, braking, failed, goal_reached or stale_state (mobile_manipulator_mpc::CycleStatus).
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
