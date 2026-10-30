# The route B5 is following (wholebody_mpc_node, Phase 5A): published on every FollowPath goal and every path update
# within one, transient local, so the medium layer plans against exactly this route.
# header.frame_id is map; header.stamp is when B5 accepted the route.
std_msgs/Header header
# Increases by one with every new route (a new goal or an in-place update).
uint64 route_id
# The route's points (positions only: every orientation is identity; its heading is the segment tangent). For a
# one-point FollowPath path the robot's position is prepended, as B5 does.
nav_msgs/Path path
# The goal pose (the path's last pose): x, y, yaw in map.
geometry_msgs/Pose2D goal
