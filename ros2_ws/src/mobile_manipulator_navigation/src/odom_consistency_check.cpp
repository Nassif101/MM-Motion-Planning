// Check Unity /odom against the ground-truth odom -> base_footprint TF.
//
// Read-only. Records both streams for a fixed simulated duration (drive the base meanwhile,
// e.g. with base_step_test) and reports: rate, frames, stamps on physics ticks, pose
// equality for identical stamps, and twist agreement with centred TF differences expressed
// in base_footprint.
//
// Usage: odom_consistency_check [--seconds 30 (simulated)] [--output FILE]
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "mobile_manipulator_control/cli.hpp"
#include "mobile_manipulator_navigation/scenario_metrics.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace fs = std::filesystem;
namespace mmn = mobile_manipulator_navigation;
using mmn::Json;

namespace
{
constexpr int64_t kTickNs = 20'000'000;

template <typename Stamp>
int64_t stamp_ns(const Stamp & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1'000'000'000 + stamp.nanosec;
}

template <typename Q>
double yaw(const Q & q)
{
  return mmn::yaw_of(q.x, q.y, q.z, q.w);
}

Json quantiles(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  const auto pick = [&values](double q) {
    return mmn::round_digits(values[std::min(values.size() - 1, static_cast<size_t>(q * values.size()))], 4);
  };
  return {{"n", values.size()}, {"p50", pick(0.5)}, {"p95", pick(0.95)}, {"p99", pick(0.99)},
          {"max", mmn::round_digits(values.back(), 4)}};
}

// Body-frame twist from the TF poses a and b (dt apart), with the heading given.
std::pair<double, double> body_velocity(const geometry_msgs::msg::Transform & a, const geometry_msgs::msg::Transform & b,
                                        double dt, double heading)
{
  const double vx = (b.translation.x - a.translation.x) / dt, vy = (b.translation.y - a.translation.y) / dt;
  return {std::cos(heading) * vx + std::sin(heading) * vy, -std::sin(heading) * vx + std::cos(heading) * vy};
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const mobile_manipulator_control::Args args(argc, argv);
  const double seconds = args.number("seconds", 30.0);

  auto node = std::make_shared<rclcpp::Node>(
    "odom_consistency_check", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));
  std::map<int64_t, nav_msgs::msg::Odometry> odom;
  std::map<int64_t, geometry_msgs::msg::TransformStamped> tf;
  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 100, [&odom](nav_msgs::msg::Odometry::ConstSharedPtr m) { odom.emplace(stamp_ns(m->header.stamp), *m); });
  auto tf_sub = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 100, [&tf](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {
      for (const auto & t : m->transforms) {
        if (t.child_frame_id == "base_footprint") tf.emplace(stamp_ns(t.header.stamp), t);
      }
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (node->get_clock()->now().nanoseconds() == 0) executor.spin_once(std::chrono::milliseconds(100));
  const int64_t end = node->get_clock()->now().nanoseconds() + static_cast<int64_t>(seconds * 1e9);
  while (node->get_clock()->now().nanoseconds() < end) executor.spin_once(std::chrono::milliseconds(50));
  rclcpp::shutdown();

  std::vector<int64_t> stamps;
  for (const auto & [stamp, message] : odom) {
    if (tf.count(stamp)) stamps.push_back(stamp);
  }
  if (stamps.empty()) {
    std::cerr << "No /odom and TF samples with matching stamps" << std::endl;
    return 1;
  }
  double pose_error = 0.0, yaw_error = 0.0;
  for (const auto s : stamps) {
    const auto & p = odom.at(s).pose.pose.position;
    const auto & t = tf.at(s).transform.translation;
    pose_error = std::max(pose_error, std::sqrt((p.x - t.x) * (p.x - t.x) + (p.y - t.y) * (p.y - t.y) + (p.z - t.z) * (p.z - t.z)));
    yaw_error = std::max(yaw_error, std::abs(std::remainder(yaw(odom.at(s).pose.pose.orientation) -
                                                            yaw(tf.at(s).transform.rotation), 2 * M_PI)));
  }
  std::vector<double> linear_errors, angular_errors, steady_linear, steady_angular, tick_linear, tick_angular;
  double peak_v = 0.0, peak_w = 0.0;
  for (size_t i = 0; i + 4 < stamps.size(); ++i) {
    const auto before = stamps[i], current = stamps[i + 2], after = stamps[i + 4];
    const auto & a = tf.at(before).transform;
    const auto & b = tf.at(after).transform;
    const double dt = (after - before) * 1e-9;
    const auto [vx, vy] = body_velocity(a, b, dt, yaw(tf.at(current).transform.rotation));
    const double w = std::remainder(yaw(b.rotation) - yaw(a.rotation), 2 * M_PI) / dt;
    const auto & twist = odom.at(current).twist.twist;
    linear_errors.push_back(std::hypot(vx - twist.linear.x, vy - twist.linear.y));
    angular_errors.push_back(std::abs(w - twist.angular.z));
    // Steady: the 80 ms window's endpoint twists agree, so the centred difference is unbiased.
    const auto & e0 = odom.at(before).twist.twist;
    const auto & e1 = odom.at(after).twist.twist;
    if (std::abs(e0.linear.x - e1.linear.x) < 0.01 && std::abs(e0.angular.z - e1.angular.z) < 0.01) {
      steady_linear.push_back(linear_errors.back());
      steady_angular.push_back(angular_errors.back());
    }
    peak_v = std::max(peak_v, std::abs(twist.linear.x));
    peak_w = std::max(peak_w, std::abs(twist.angular.z));
  }
  // PhysX integrates semi-implicitly (x[k] = x[k-1] + v[k] dt), so a one-tick TF difference
  // must reproduce the reported twist at the later tick if /odom is exact.
  for (size_t i = 1; i < stamps.size(); ++i) {
    if (stamps[i] - stamps[i - 1] != kTickNs) continue;
    const auto & a = tf.at(stamps[i - 1]).transform;
    const auto & b = tf.at(stamps[i]).transform;
    const double dt = kTickNs * 1e-9;
    const auto [vx, vy] = body_velocity(a, b, dt, yaw(b.rotation));
    const double w = std::remainder(yaw(b.rotation) - yaw(a.rotation), 2 * M_PI) / dt;
    const auto & twist = odom.at(stamps[i]).twist.twist;
    tick_linear.push_back(std::hypot(vx - twist.linear.x, vy - twist.linear.y));
    tick_angular.push_back(std::abs(w - twist.angular.z));
  }

  const auto & sample = odom.at(stamps.back());
  bool on_ticks = true;
  for (const auto & [stamp, message] : odom) on_ticks &= stamp % kTickNs == 0;
  const auto q = [](const std::vector<double> & v) { return v.empty() ? Json(nullptr) : quantiles(v); };
  const Json report = {
    {"odom_messages", odom.size()}, {"tf_samples", tf.size()}, {"paired", stamps.size()},
    {"odom_rate_hz", mmn::round_digits(odom.size() / seconds, 2)},
    {"frames", {sample.header.frame_id, sample.child_frame_id}},
    {"all_stamps_on_physics_ticks", on_ticks},
    {"max_pose_error_m", pose_error}, {"max_yaw_error_rad", yaw_error},
    {"linear_twist_error_mps", q(linear_errors)}, {"angular_twist_error_radps", q(angular_errors)},
    {"steady_linear_twist_error_mps", q(steady_linear)}, {"steady_angular_twist_error_radps", q(steady_angular)},
    {"one_tick_linear_twist_error_mps", q(tick_linear)}, {"one_tick_angular_twist_error_radps", q(tick_angular)},
    {"peak_linear_mps", mmn::round_digits(peak_v, 3)}, {"peak_angular_radps", mmn::round_digits(peak_w, 3)},
  };
  std::cout << report.dump(2) << std::endl;
  if (args.has("output")) {
    const fs::path output = args.get("output");
    if (output.has_parent_path()) fs::create_directories(output.parent_path());
    std::ofstream(output) << report.dump(2) << "\n";
  }
  return 0;
}
