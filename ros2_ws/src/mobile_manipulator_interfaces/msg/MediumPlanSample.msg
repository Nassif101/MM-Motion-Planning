# One sample of a medium-horizon plan (MediumPlan), every sample_ds of arc length or sample_dyaw of yaw change,
# whichever comes first. The first sample is the planned-from state at MediumPlan.header.stamp (s = 0). A turn on the
# spot gives several samples at the same s; a lookup at that s takes the last of them.
# Arc length along the planned base path from its first sample (m).
float64 s
# Base pose in map (m, m, rad) and speed along the path (m/s).
float64 x
float64 y
float64 yaw
float64 speed
# Posture target of the six arm joints in qualified_payload.json joint_order (rad): the planned joints, the others
# at the comfort pose.
float64[6] q
