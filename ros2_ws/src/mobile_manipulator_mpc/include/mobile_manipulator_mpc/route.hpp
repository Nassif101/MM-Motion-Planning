#pragma once
// The Nav2 path as a polyline parameterised by arc length (no ROS graph). The MPC projects
// its predicted base positions onto it to track progress and lateral offset.
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

  double length() const { return s_.back(); }

  struct Projection
  {
    double s;               // arc length of the nearest route point
    Eigen::Vector2d point;  // the nearest route point
    double heading;         // direction of the route segment there (rad)
    double lateral;         // signed offset of p, positive left of the path direction
  };

  // Nearest point of the segments within `window` metres of arc length around s_hint, so a
  // route that passes near itself is followed on the branch the robot is on.
  Projection project(const Eigen::Vector2d & p, double s_hint, double window = 1.0) const;
  // Nearest point with s in [s_lo, s_hi] (clamped to the route).
  Projection project_range(const Eigen::Vector2d & p, double s_lo, double s_hi) const;

  // Point at arc length s (clamped to [0, length]).
  Eigen::Vector2d at(double s) const;
  // Heading of the segment containing s; at a corner the outgoing segment's, at the end the
  // last segment's.
  double heading_at(double s) const;

private:
  size_t segment_of(double s) const;

  std::vector<Eigen::Vector2d> points_;
  std::vector<double> s_;  // arc length at each point
};
}  // namespace mobile_manipulator_mpc
