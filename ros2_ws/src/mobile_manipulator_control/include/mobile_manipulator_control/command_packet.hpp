#pragma once
// The arm command to Unity (Phase 4B-B, ADR 0014): one trajectory point holding the JTC's commanded positions,
// velocities and accelerations, stamped with the controller's time. Unity treats it as an instantaneous target.
#include <builtin_interfaces/msg/time.hpp>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>

namespace mobile_manipulator_control
{
inline trajectory_msgs::msg::JointTrajectory command_packet(
  const builtin_interfaces::msg::Time & stamp, const std::vector<std::string> & names,
  const std::vector<double> & q, const std::vector<double> & v, const std::vector<double> & a)
{
  trajectory_msgs::msg::JointTrajectory packet;
  packet.header.stamp = stamp;
  packet.joint_names = names;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = q;
  point.velocities = v;
  point.accelerations = a;
  packet.points.push_back(point);
  return packet;
}
}  // namespace mobile_manipulator_control
