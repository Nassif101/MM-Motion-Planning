#include "mobile_manipulator_manipulation/reconfigure_logic.hpp"

#include <algorithm>
#include <cmath>

#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace mobile_manipulator_manipulation
{
namespace
{
// Samples may be this much sparser than the window edge (odom arrives at ~50 Hz).
constexpr double kMaxGap = 0.1;
// A sample this much older than the newest one starts a new epoch (teleport, Play restart).
constexpr double kEpochJump = 0.5;
}  // namespace

moveit_msgs::msg::Constraints panel_goal_constraints(const geometry_msgs::msg::PoseStamped & pose,
                                                     const geometry_msgs::msg::Vector3 & position_tolerance,
                                                     const geometry_msgs::msg::Vector3 & orientation_tolerance)
{
  moveit_msgs::msg::Constraints constraints;
  constraints.name = "panel_pose";

  moveit_msgs::msg::PositionConstraint position;
  position.header = pose.header;
  position.link_name = kPanelFrame;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {2.0 * position_tolerance.x, 2.0 * position_tolerance.y, 2.0 * position_tolerance.z};
  geometry_msgs::msg::Pose centre;
  centre.position = pose.pose.position;
  centre.orientation.w = 1.0;
  position.constraint_region.primitives.push_back(box);
  position.constraint_region.primitive_poses.push_back(centre);
  position.weight = 1.0;
  constraints.position_constraints.push_back(position);

  moveit_msgs::msg::OrientationConstraint orientation;
  orientation.header = pose.header;
  orientation.link_name = kPanelFrame;
  orientation.orientation = pose.pose.orientation;
  orientation.parameterization = moveit_msgs::msg::OrientationConstraint::ROTATION_VECTOR;
  orientation.absolute_x_axis_tolerance = orientation_tolerance.x;
  orientation.absolute_y_axis_tolerance = orientation_tolerance.y;
  orientation.absolute_z_axis_tolerance = orientation_tolerance.z;
  orientation.weight = 1.0;
  constraints.orientation_constraints.push_back(orientation);
  return constraints;
}

bool panel_goal_met(const Eigen::Isometry3d & reached, const geometry_msgs::msg::PoseStamped & goal,
                    const geometry_msgs::msg::Vector3 & position_tolerance,
                    const geometry_msgs::msg::Vector3 & orientation_tolerance)
{
  constexpr double kSlack = 1e-6;
  const auto & p = goal.pose.position;
  const Eigen::Vector3d offset = reached.translation() - Eigen::Vector3d(p.x, p.y, p.z);
  if (std::abs(offset.x()) > position_tolerance.x + kSlack || std::abs(offset.y()) > position_tolerance.y + kSlack ||
      std::abs(offset.z()) > position_tolerance.z + kSlack) {
    return false;
  }
  const auto & q = goal.pose.orientation;
  const Eigen::Quaterniond target(q.w, q.x, q.y, q.z);
  const Eigen::AngleAxisd error(target.normalized().inverse() * Eigen::Quaterniond(reached.linear()));
  const Eigen::Vector3d rotation = error.angle() * error.axis();
  return std::abs(rotation.x()) <= orientation_tolerance.x + kSlack &&
         std::abs(rotation.y()) <= orientation_tolerance.y + kSlack &&
         std::abs(rotation.z()) <= orientation_tolerance.z + kSlack;
}

void BaseMotionWindow::add(double t, double linear, double angular)
{
  // A large jump back is a new simulation epoch; a small one is a sample handled late.
  if (!samples_.empty() && t < samples_.back().t - kEpochJump) samples_.clear();
  const Sample sample{t, std::abs(linear), std::abs(angular)};
  samples_.insert(std::upper_bound(samples_.begin(), samples_.end(), t,
                                   [](double time, const Sample & s) { return time < s.t; }),
                  sample);
  // Keep a little more than any window a caller is likely to ask for.
  while (samples_.size() > 1 && samples_.front().t < t - 5.0) samples_.pop_front();
}

bool BaseMotionWindow::stopped(double now, double window_s, double v_max, double w_max) const
{
  if (samples_.empty() || now - samples_.back().t > kMaxGap) return false;
  // No blind gap: samples from the window start to the newest, none further apart than kMaxGap.
  double previous = now - window_s;
  for (const auto & sample : samples_) {
    if (sample.t < now - window_s) continue;
    if (sample.t - previous > kMaxGap) return false;
    if (sample.linear >= v_max || sample.angular >= w_max) return false;
    previous = sample.t;
  }
  return previous > now - window_s;
}

bool execution_overdue(double sim_elapsed_s, double wall_elapsed_s, double budget_s)
{
  constexpr double kWallFactor = 4.0;
  return sim_elapsed_s > budget_s || wall_elapsed_s > kWallFactor * budget_s;
}

void HoldErrorTracker::add(const std::vector<double> & measured)
{
  for (size_t i = 0; i < target_.size() && i < measured.size(); ++i) {
    max_error_ = std::max(max_error_, std::abs(measured[i] - target_[i]));
  }
  samples_ += 1;
}

double HoldErrorTracker::value() const { return samples_ ? max_error_ : std::nan(""); }

bool replan_after(int moveit_error_code)
{
  using Codes = moveit_msgs::msg::MoveItErrorCodes;
  return moveit_error_code == Codes::FAILURE || moveit_error_code == Codes::PLANNING_FAILED ||
         moveit_error_code == Codes::INVALID_MOTION_PLAN;
}

bool fresh_after(const builtin_interfaces::msg::Time & sample, const builtin_interfaces::msg::Time & trajectory_end)
{
  return sample.sec > trajectory_end.sec ||
         (sample.sec == trajectory_end.sec && sample.nanosec >= trajectory_end.nanosec);
}

double joint_path_length(const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  double length = 0.0;
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & a = trajectory.points[i - 1].positions;
    const auto & b = trajectory.points[i].positions;
    for (size_t j = 0; j < a.size() && j < b.size(); ++j) length += std::abs(b[j] - a[j]);
  }
  return length;
}
}  // namespace mobile_manipulator_manipulation
