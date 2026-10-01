// Bounded arm-specific transport benchmark; simulation stamps are not one-way network latency.
//
// Usage: measure_transport --output FILE [--seconds 1..60 (default 8)]
// For /arm/command, /arm/state and /clock: message count, wall-clock and simulation rates,
// median/maximum arrival and stamp intervals, non-increasing stamps, and how far the latest
// /clock was ahead of each message's stamp when it arrived.
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <thread>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "mobile_manipulator_control/cli.hpp"

using Json = nlohmann::ordered_json;
using Clock = std::chrono::steady_clock;

namespace
{
struct Sample
{
  double wall, stamp;
  std::optional<double> clock;  // latest /clock when the message arrived
};

double median(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  const size_t n = values.size();
  return n % 2 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2;
}

double wall_now()
{
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const mobile_manipulator_control::Args args(argc, argv);
  const double seconds = args.number("seconds", 8.0);
  if (!(seconds >= 1 && seconds <= 60)) {
    std::cerr << "Benchmark duration must be 1..60 seconds" << std::endl;
    return 2;
  }
  const std::string output = args.get("output");

  auto node = std::make_shared<rclcpp::Node>("arm_transport_benchmark");
  const std::vector<std::string> topics = {"/arm/command", "/arm/state", "/clock"};
  std::map<std::string, std::vector<Sample>> samples;
  std::optional<double> latest_clock;
  // The observer must retain bursts rather than manufacture a lower measured rate.
  auto clock_sub = node->create_subscription<rosgraph_msgs::msg::Clock>(
    "/clock", 1000, [&](rosgraph_msgs::msg::Clock::ConstSharedPtr m) {
      latest_clock = m->clock.sec + m->clock.nanosec * 1e-9;
      samples["/clock"].push_back({wall_now(), *latest_clock, latest_clock});
    });
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> joint_subs;
  for (const std::string topic : {"/arm/command", "/arm/state"}) {
    joint_subs.push_back(node->create_subscription<sensor_msgs::msg::JointState>(
      topic, 1000, [&, topic](sensor_msgs::msg::JointState::ConstSharedPtr m) {
        samples[topic].push_back({wall_now(), m->header.stamp.sec + m->header.stamp.nanosec * 1e-9, latest_clock});
      }));
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto stop = Clock::now() + std::chrono::duration<double>(seconds);
  while (Clock::now() < stop) executor.spin_once(std::chrono::milliseconds(100));

  Json result = Json::object();
  for (const auto & topic : topics) {
    const auto & points = samples[topic];
    if (points.size() < 2) {
      std::cerr << "No transport samples: " << topic << std::endl;
      rclcpp::shutdown();
      return 1;
    }
    std::vector<double> intervals, stamp_intervals, lag;
    int nonincreasing = 0;
    for (size_t i = 1; i < points.size(); ++i) {
      intervals.push_back(points[i].wall - points[i - 1].wall);
      stamp_intervals.push_back(points[i].stamp - points[i - 1].stamp);
      nonincreasing += stamp_intervals.back() <= 0;
    }
    for (const auto & p : points) {
      if (p.clock) lag.push_back(*p.clock - p.stamp);
    }
    const double n = points.size() - 1.0;
    result[topic] = {
      {"count", points.size()},
      {"wall_hz", n / (points.back().wall - points.front().wall)},
      {"simulation_hz", n / (points.back().stamp - points.front().stamp)},
      {"median_wall_interval", median(intervals)},
      {"max_wall_interval", *std::max_element(intervals.begin(), intervals.end())},
      {"median_stamp_interval", median(stamp_intervals)},
      {"max_stamp_interval", *std::max_element(stamp_intervals.begin(), stamp_intervals.end())},
      {"nonincreasing_stamps", nonincreasing},
      {"max_observed_clock_minus_stamp", lag.empty() ? 0.0 : *std::max_element(lag.begin(), lag.end())},
      {"median_observed_clock_minus_stamp", lag.empty() ? Json(nullptr) : Json(median(lag))},
    };
  }
  std::ofstream(output) << result.dump(2) << "\n";
  std::cout << result.dump() << std::endl;
  rclcpp::shutdown();
  return 0;
}
