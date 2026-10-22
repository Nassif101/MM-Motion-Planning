# Diagnostics of one whole-body MPC cycle (wholebody_mpc_node, baseline B5).
std_msgs/Header header
# ok, fallback, braking, failed, goal_reached, stale_state, holding or blocked (mobile_manipulator_mpc::CycleStatus;
# blocked: the collision monitor passes nothing while the MPC commands (BlockedDetector), the robot holds;
# holding: the solved plan failed the slack gate and was not executed, the robot brakes and re-plans;
# a solver failure during a hold also reports holding, with zero slack, and counts as a failure).
string status
# Seconds. linearize: advancing the nominal plan; collision: distance rows and the comfort pose's
# clearance (the tightness input);
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
# Posture preference blend alpha in [0, 1] (0 prefers the comfort pose, comfort_pose: carry or home;
# 1 vertical carry).
float64 tightness
# The side picker (Phase 4B-A): none, comfort, left or right (the tight pose the blend pulls towards; comfort:
# no tight pose beats the comfort pose at the first conflict), and the last choosing cycle's scores in metres
# (comfort, then each tight pose in the first-conflict window; when both tight poses were eligible and the route
# probe's tight segment had counted samples (Phase 4B-A Task 11), then each tight pose's window score capped by its
# route-segment minimum and each one's segment mean; empty until a choice, cleared when the choice is released).
string posture_choice
float64[] probe_scores
# total_s exceeded the cycle period (counted as a controller failure).
bool deadline_missed
# Arm reference (ruling R24): the largest |reference - measured| over the six joints (rad; the
# JTC tracking error of the reference the stream continues), and whether this cycle re-anchored
# the MPC's arm state on the measured joints (that offset above 0.15 rad with fresh joint states).
float64 arm_reference_offset
bool arm_reanchored
# Phase 4B-B (spec Section 4.8). iter2_used: a second QP iteration ran and its iterate was kept; iter2_rejected: it
# ran and the step check kept the first. prep_s: the preparation this cycle used (preparation/feedback split, made
# after the previous command), feedback_s: the in-cycle part (both 0 without the split). self_pairs: the
# panel-against-arm-link rows. timer_lateness_s: the cycle's start after its scheduled time (period multiples; a
# cycle a period or more late re-anchors the schedule); cycle_cpu_s: the cycle's thread CPU time (a cycle whose
# total_s far exceeds it was descheduled).
bool iter2_used
bool iter2_rejected
float64 prep_s
float64 feedback_s
int32 self_pairs
float64 timer_lateness_s
float64 cycle_cpu_s
# The deadline cap ended or skipped this cycle's solve (a solver failure, not a missed deadline).
bool time_limited
