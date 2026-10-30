# A medium-horizon plan (medium_horizon_node, Phase 5A): B5's arc-length references in reference_source: medium.
# header.frame_id is map; header.stamp is the plan's start time (the state it was planned from).
std_msgs/Header header
# The route (WholeBodyRoute.route_id) the plan was built on, and the plan's own counter.
uint64 route_id
uint64 plan_id
# ok, infeasible (the lowest-slack solution of a plan with no feasible seed) or stale_reused (the solve cap was hit:
# the previous samples, re-sent).
string status
# The seed the published solution came from and its homotopy label (comfort, left, right, lowered, joined with +).
string seed
string homotopy
float64 cost
# Compute time of this plan (s).
float64 solve_s
# Arc length on the route (WholeBodyRoute.path) of the last sample's projection: B5 continues on the route from here.
float64 splice_s
MediumPlanSample[] samples
