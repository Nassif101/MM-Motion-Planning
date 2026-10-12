# The whole-body MPC's predicted plan of one cycle (wholebody_mpc_node, baseline B5).
# header.frame_id is map; header.stamp is the cycle time (the time of stage 0).
std_msgs/Header header
# The arm joints of `positions`, in qualified_payload.json joint_order.
string[] joint_names
# Per stage (N + 1 stages, stage 0 = the measured state): seconds after header.stamp.
float64[] time_from_start
# Per stage: base_footprint pose in the map frame.
geometry_msgs/Pose2D[] base
# Row-major stage x joint: positions[k * joint_names.size() + j] is joint j at stage k (rad).
float64[] positions
