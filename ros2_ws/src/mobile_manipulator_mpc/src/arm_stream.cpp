#include "mobile_manipulator_mpc/arm_stream.hpp"

#include <algorithm>

#include <rclcpp/duration.hpp>

namespace mobile_manipulator_mpc
{
std::vector<StreamPoint> plan_stream_points(const std::vector<Eigen::Matrix<double, 6, 1>> & q_stage,
                                            const std::vector<Eigen::Matrix<double, 6, 1>> & u_stage, double dt,
                                            double offset, double min_lead_s, double dense_s, double dense_dt_s)
{
  using Joints = Eigen::Matrix<double, 6, 1>;
  const int n = static_cast<int>(u_stage.size());
  std::vector<StreamPoint> out;
  out.reserve(static_cast<size_t>(n) + (dense_dt_s > 0.0 ? static_cast<size_t>(dense_s / dense_dt_s) + 1 : 0));
  int first = 1;
  while (first < n && first * dt - offset <= min_lead_s) ++first;
  // The first stage point stays as without dense points: point 0 carries the reference velocity, and a first segment
  // shortened to dense_dt would sharpen that kink (Phase 4B-C Task 2 review).
  const double first_t = std::max(first * dt - offset, min_lead_s);
  double dense_end = -1.0;
  if (dense_s > 0.0 && dense_dt_s > 0.0) {
    for (int i = 1; i * dense_dt_s <= dense_s + 1e-9; ++i) {
      const double tau = i * dense_dt_s, plan_t = tau + offset;
      if (tau <= first_t + 0.5 * dense_dt_s) continue;
      if (plan_t > n * dt + 1e-9) break;
      const int k = std::clamp(static_cast<int>(std::floor(plan_t / dt + 1e-9)), 0, n - 1);
      const auto ks = static_cast<size_t>(k);
      out.push_back({tau, Joints(q_stage[ks] + u_stage[ks] * (plan_t - k * dt)), u_stage[ks]});
      dense_end = tau;
    }
  }
  // The first stage point goes first (dense points start after it).
  const auto first_point = [&]() {
    const auto ks = static_cast<size_t>(first);
    const Joints v = first < n ? Joints(0.5 * (u_stage[ks - 1] + u_stage[ks])) : u_stage[static_cast<size_t>(n - 1)];
    return StreamPoint{first_t, q_stage[ks], v};
  };
  if (dense_end > 0.0) {
    out.insert(out.begin(), first_point());
  }
  for (int k = first; k <= n; ++k) {
    const double t = std::max(k * dt - offset, min_lead_s);
    // Stage points inside the dense part (or within half a dense step after it) are replaced.
    if (dense_end > 0.0 && (k == first || t <= dense_end + 0.5 * dense_dt_s)) continue;
    const auto ks = static_cast<size_t>(k);
    const Joints v = k < n ? Joints(0.5 * (u_stage[ks - 1] + u_stage[ks])) : u_stage[static_cast<size_t>(n - 1)];
    out.push_back({t, q_stage[ks], v});
  }
  return out;
}

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
