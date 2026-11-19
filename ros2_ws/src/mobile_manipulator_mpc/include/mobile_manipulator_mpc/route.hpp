#pragma once
// A navigation polyline parameterised by arc length, or an explicit stationary position/yaw
// reference (no ROS graph). The MPC projects predicted base positions onto this reference.
#include <vector>

#include <Eigen/Core>

namespace mobile_manipulator_mpc
{
class Route
{
public:
  // Consecutive duplicate points are dropped; throws std::invalid_argument for fewer than two
  // distinct points.
  explicit Route(std::vector<Eigen::Vector2d> points);

  // A heading-only destination. No artificial translational segment is introduced. Throws
  // std::invalid_argument for a non-finite anchor/yaw. The target yaw is wrapped to [-pi, pi].
  static Route stationary(const Eigen::Vector2d & anchor, double target_yaw);
  bool is_stationary() const { return points_.size() == 1; }

  double length() const { return s_.back(); }

  struct Projection
  {
    double s;               // arc length of the nearest route point
    Eigen::Vector2d point;  // the nearest route point
    double heading;         // segment direction, or stationary target yaw (rad)
    double lateral;         // signed offset of p, positive left of the path direction
  };

  // Nearest point of the segments within `window` metres of arc length around s_hint, so a
  // route that passes near itself is followed on the branch the robot is on.
  Projection project(const Eigen::Vector2d & p, double s_hint, double window = 1.0) const;
  // Nearest point with s in [s_lo, s_hi] (clamped to the route).
  Projection project_range(const Eigen::Vector2d & p, double s_lo, double s_hi) const;

  // Point at arc length s (clamped to [0, length]); constant for a stationary route.
  Eigen::Vector2d at(double s) const;
  // Heading of the segment containing s; at a corner the outgoing segment's, at the end the
  // last segment's. A stationary route always returns its explicit target yaw.
  double heading_at(double s) const;
  // The points (consecutive duplicates dropped).
  const std::vector<Eigen::Vector2d> & points() const { return points_; }

private:
  Route(const Eigen::Vector2d & anchor, double target_yaw);
  size_t segment_of(double s) const;

  std::vector<Eigen::Vector2d> points_;
  std::vector<double> s_;  // arc length at each point
  double stationary_yaw_ = 0.0;
};
}  // namespace mobile_manipulator_mpc
