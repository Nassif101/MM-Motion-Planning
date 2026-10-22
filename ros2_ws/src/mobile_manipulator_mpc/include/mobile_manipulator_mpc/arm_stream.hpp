#pragma once
// The arm stream as the JTC samples it (the node's reference, the replay bench's previous arm command).
#include <Eigen/Core>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace mobile_manipulator_mpc
{
// Position and velocity of a sent trajectory `t` seconds after it was sent, as the JTC samples it from positions
// and velocities: a cubic Hermite spline between points; the first point before it (a first point at time 0 is
// never before), at rest at the last point after it. Joints in the trajectory's order (the model's).
void sample_trajectory(const trajectory_msgs::msg::JointTrajectory & trajectory, double t,
                       Eigen::Matrix<double, 6, 1> & q, Eigen::Matrix<double, 6, 1> & v);
}  // namespace mobile_manipulator_mpc
