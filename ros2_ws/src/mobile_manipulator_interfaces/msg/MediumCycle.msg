# One medium-horizon cycle (medium_horizon_node, Phase 5A): timing and per-seed SCP statistics.
std_msgs/Header header
uint64 plan_id
# Wall time of the whole cycle, of the collision/torque linearizations and of the QP solves (s).
float64 total_s
float64 linearize_s
float64 qp_s
# A full multi-start this cycle (new route, draggable change, or the period), and whether the solve cap cut it short.
bool full_multistart
bool capped
# Per seed solved this cycle, index for index.
string[] seeds
float64[] seed_costs
float64[] seed_slack
float64[] seed_solve_s
int32[] seed_iterations
bool[] seed_converged
# The published homotopy and whether it changed this cycle.
string homotopy
bool homotopy_switched
