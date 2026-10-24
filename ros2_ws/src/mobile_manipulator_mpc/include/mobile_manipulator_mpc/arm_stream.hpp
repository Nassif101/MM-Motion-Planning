#pragma once
// The arm stream as the JTC samples it (the node's reference, the replay bench's previous arm command).
#include <Eigen/Core>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace mobile_manipulator_mpc
{
// Position and velocity of a sent trajectory `t` seconds after it was sent, as the JTC samples it from positions
// and velocities: a cubic Hermite spline between points; the first point before it (a first point at time 0 is
// never before), at rest at the last point after it. Joints in the trajectory's order (the model's).
// The start time (s, the node's clock) of a streamed trajectory led by `lead_s` (Phase 4B-B option A, ADR 0014): `now_s
// - lead_s`, or 0 (start on receipt) without a lead or before the clock passes it. The node's own sampling of what it
// sent keeps counting from the send time: the plan's time, where the arm is once the lead covers its delay.
inline double stream_start_seconds(double now_s, double lead_s)
{
  return lead_s > 0.0 && now_s - lead_s > 0.0 ? now_s - lead_s : 0.0;
}

void sample_trajectory(const trajectory_msgs::msg::JointTrajectory & trajectory, double t,
                       Eigen::Matrix<double, 6, 1> & q, Eigen::Matrix<double, 6, 1> & v);
}  // namespace mobile_manipulator_mpc
