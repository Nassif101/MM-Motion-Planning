#include "mobile_manipulator_mpc/arm_stream.hpp"

#include <algorithm>

#include <rclcpp/duration.hpp>

namespace mobile_manipulator_mpc
{
void sample_trajectory(const trajectory_msgs::msg::JointTrajectory & trajectory, double t,
                       Eigen::Matrix<double, 6, 1> & q, Eigen::Matrix<double, 6, 1> & v)
{
  using Joints = Eigen::Matrix<double, 6, 1>;
  const auto & points = trajectory.points;
  const auto time = [](const trajectory_msgs::msg::JointTrajectoryPoint & p) {
    return rclcpp::Duration(p.time_from_start).seconds();
  };
  const auto at = [](const std::vector<double> & values) { return Eigen::Map<const Joints>(values.data()); };
  if (t <= time(points.front())) {
    q = at(points.front().positions);
    v = at(points.front().velocities);
    return;
  }
  size_t i = 0;
  while (i + 1 < points.size() && time(points[i + 1]) < t) ++i;
  if (i + 1 >= points.size()) {
    q = at(points.back().positions);
    v = Joints::Zero();
    return;
  }
  const double t0 = time(points[i]), h = time(points[i + 1]) - t0;
  if (!(h > 0.0)) {
    q = at(points[i + 1].positions);
    v = at(points[i + 1].velocities);
    return;
  }
  const double s = std::clamp((t - t0) / h, 0.0, 1.0), s2 = s * s, s3 = s2 * s;
  const Joints p0 = at(points[i].positions), p1 = at(points[i + 1].positions);
  const Joints m0 = h * at(points[i].velocities), m1 = h * at(points[i + 1].velocities);
  q = (2 * s3 - 3 * s2 + 1) * p0 + (s3 - 2 * s2 + s) * m0 + (-2 * s3 + 3 * s2) * p1 + (s3 - s2) * m1;
  v = ((6 * s2 - 6 * s) * p0 + (3 * s2 - 4 * s + 1) * m0 + (-6 * s2 + 6 * s) * p1 + (3 * s2 - 2 * s) * m1) / h;
}
}  // namespace mobile_manipulator_mpc
