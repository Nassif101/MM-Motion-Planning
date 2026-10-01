// Measure Unity -> ROS transport load and small-message jitter with the lidar active.
//
// Read-only. For a wall-clock window it records /livox/lidar size and rate, wall-clock
// inter-arrival statistics of /clock, /tf, and /arm/state, and the ROS-TCP endpoint's CPU
// use (from /proc). Receiver-side observations, not one-way latency.
//
// Usage: sensor_transport_probe --label NAME [--seconds 30] [--output FILE]
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <optional>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "mobile_manipulator_control/cli.hpp"
#include "mobile_manipulator_navigation/scenario_metrics.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace fs = std::filesystem;
namespace mmn = mobile_manipulator_navigation;
using mmn::Json;
using Clock = std::chrono::steady_clock;

namespace
{
std::string read_file(const fs::path & path)
{
  std::ifstream stream(path);
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

// The first process whose command line contains the endpoint's install path (as pgrep -f).
std::optional<int> endpoint_pid()
{
  std::optional<int> found;
  for (const auto & entry : fs::directory_iterator("/proc")) {
    const std::string name = entry.path().filename();
    if (name.find_first_not_of("0123456789") != std::string::npos) continue;
    std::string cmdline = read_file(entry.path() / "cmdline");
    std::replace(cmdline.begin(), cmdline.end(), '\0', ' ');
    if (cmdline.find("lib/mobile_manipulator_control/unity_control_endpoint") == std::string::npos) continue;
    const int pid = std::stoi(name);
    if (!found || pid < *found) found = pid;
  }
  return found;
}

double cpu_seconds(int pid)
{
  const std::string stat = read_file("/proc/" + std::to_string(pid) + "/stat");
  std::istringstream fields(stat.substr(stat.rfind(')') + 1));
  std::vector<std::string> values{std::istream_iterator<std::string>(fields), {}};
  return (std::stod(values[11]) + std::stod(values[12])) / sysconf(_SC_CLK_TCK);
}

Json interval_stats(const std::vector<double> & arrivals)
{
  std::vector<double> gaps;
  for (size_t i = 1; i < arrivals.size(); ++i) gaps.push_back(arrivals[i] - arrivals[i - 1]);
  std::vector<double> ordered = gaps;
  std::sort(ordered.begin(), ordered.end());
  const auto pick = [&ordered](double q) {
    return mmn::round_digits(ordered[std::min(ordered.size() - 1, static_cast<size_t>(q * ordered.size()))] * 1000, 2);
  };
  const double mean = std::accumulate(gaps.begin(), gaps.end(), 0.0) / gaps.size();
  double variance = 0.0;
  for (const double g : gaps) variance += (g - mean) * (g - mean);
  return {{"count", arrivals.size()},
          {"rate_hz", mmn::round_digits((arrivals.size() - 1) / (arrivals.back() - arrivals.front()), 2)},
          {"gap_ms", {{"p50", pick(0.5)}, {"p95", pick(0.95)}, {"p99", pick(0.99)},
                      {"max", mmn::round_digits(ordered.back() * 1000, 2)},
                      {"stdev", mmn::round_digits(std::sqrt(variance / gaps.size()) * 1000, 2)}}}};
}

double wall_now() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const mobile_manipulator_control::Args args(argc, argv);
  const double window = args.number("seconds", 30.0);
  const std::string label = args.get("label");
  const auto pid = endpoint_pid();

  auto node = std::make_shared<rclcpp::Node>("sensor_transport_probe");
  std::map<std::string, std::vector<double>> arrivals = {{"/clock", {}}, {"/tf", {}}, {"/arm/state", {}}, {"/livox/lidar", {}}};
  std::vector<double> cloud_bytes, cloud_points;
  auto clock_sub = node->create_subscription<rosgraph_msgs::msg::Clock>(
    "/clock", 1000, [&](rosgraph_msgs::msg::Clock::ConstSharedPtr) { arrivals["/clock"].push_back(wall_now()); });
  auto tf_sub = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 1000, [&](tf2_msgs::msg::TFMessage::ConstSharedPtr) { arrivals["/tf"].push_back(wall_now()); });
  auto arm_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/arm/state", 1000, [&](sensor_msgs::msg::JointState::ConstSharedPtr) { arrivals["/arm/state"].push_back(wall_now()); });
  auto cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/livox/lidar", rclcpp::SensorDataQoS(), [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
      arrivals["/livox/lidar"].push_back(wall_now());
      cloud_bytes.push_back(m->data.size());
      cloud_points.push_back(static_cast<double>(m->width) * m->height);
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  const std::optional<double> start_cpu = pid ? std::optional<double>(cpu_seconds(*pid)) : std::nullopt;
  const auto start = Clock::now();
  while (std::chrono::duration<double>(Clock::now() - start).count() < window) {
    executor.spin_once(std::chrono::milliseconds(50));
  }
  const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
  const std::optional<double> end_cpu = pid ? std::optional<double>(cpu_seconds(*pid)) : std::nullopt;
  rclcpp::shutdown();

  Json lidar = nullptr;
  if (!cloud_bytes.empty()) {
    const double bytes = std::accumulate(cloud_bytes.begin(), cloud_bytes.end(), 0.0);
    lidar = {{"mean_points", std::lround(std::accumulate(cloud_points.begin(), cloud_points.end(), 0.0) / cloud_points.size())},
             {"mean_bytes", std::lround(bytes / cloud_bytes.size())},
             {"bandwidth_mb_s", mmn::round_digits(bytes / elapsed / 1e6, 3)}};
  }
  Json topics = Json::object();
  for (const std::string topic : {"/clock", "/tf", "/arm/state", "/livox/lidar"}) {
    if (arrivals[topic].size() > 2) topics[topic] = interval_stats(arrivals[topic]);
  }
  const Json report = {{"label", label}, {"window_s", mmn::round_digits(elapsed, 2)}, {"lidar", lidar},
                       {"endpoint_cpu_percent", pid ? Json(mmn::round_digits(100 * (*end_cpu - *start_cpu) / elapsed, 1)) : Json(nullptr)},
                       {"topics", topics}};
  std::cout << report.dump(2) << std::endl;
  if (args.has("output")) {
    const fs::path output = args.get("output");
    if (output.has_parent_path()) fs::create_directories(output.parent_path());
    std::ofstream(output) << report.dump(2) << "\n";
  }
  return 0;
}
