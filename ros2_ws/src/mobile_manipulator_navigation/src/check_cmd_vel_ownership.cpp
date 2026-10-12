// Fail unless /cmd_vel has at most one publisher, of the expected node and type (ADR 0006).
//
// Read-only graph check to run before experiments. Unity's in-editor keyboard teleop is
// not visible to ROS and must be disabled separately.
//
// Usage: check_cmd_vel_ownership [--expect NODE] [--expect-upstream NODE] [--discovery-seconds S]
//   --expect: node name that must be the only publisher (e.g. collision_monitor); omit to
//             require no publisher. --expect-upstream: node name that must be the only
//             publisher of /cmd_vel_smoothed, the collision monitor's input (velocity_smoother
//             for B1-B4, wholebody_mpc for B5); omit to leave it unchecked.
//             --discovery-seconds: upper bound (default 8); returns early once the graph is
//             stable for 2 s.
#include <algorithm>
#include <chrono>
#include <iostream>
#include <optional>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "mobile_manipulator_control/cli.hpp"

namespace mmn = mobile_manipulator_control;
using Clock = std::chrono::steady_clock;

namespace
{
const char * kTopic = "/cmd_vel";
const char * kUpstreamTopic = "/cmd_vel_smoothed";
const char * kType = "geometry_msgs/msg/Twist";

std::vector<std::string> node_names(const std::vector<rclcpp::TopicEndpointInfo> & endpoints, bool sort)
{
  std::vector<std::string> names;
  for (const auto & endpoint : endpoints) names.push_back(endpoint.node_name());
  if (sort) std::sort(names.begin(), names.end());
  return names;
}

// Python list text, as the previous tool printed it.
std::string list_text(const std::vector<std::string> & names)
{
  std::string text = "[";
  for (size_t i = 0; i < names.size(); ++i) text += (i ? ", '" : "'") + names[i] + "'";
  return text + "]";
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const mmn::Args args(argc, argv);
  const bool expect_publisher = args.has("expect");
  const std::string expected = expect_publisher ? args.get("expect") : "";
  const auto discovery = std::chrono::duration<double>(args.number("discovery-seconds", 8.0));
  const bool check_upstream = args.has("expect-upstream");
  const std::string upstream = check_upstream ? args.get("expect-upstream") : "";
  auto node = std::make_shared<rclcpp::Node>("check_cmd_vel_ownership");

  // A fresh node discovers large graphs gradually; wait until the endpoint lists of the
  // topic stop changing (and at least one endpoint is known) rather than a fixed delay.
  const auto deadline = Clock::now() + discovery;
  std::optional<std::pair<std::vector<std::string>, std::vector<std::string>>> snapshot;
  std::vector<std::string> upstream_snapshot;
  auto stable_since = Clock::now();
  while (Clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const auto current = std::make_pair(node_names(node->get_publishers_info_by_topic(kTopic), true),
                                        node_names(node->get_subscriptions_info_by_topic(kTopic), true));
    const auto upstream_now = node_names(node->get_publishers_info_by_topic(kUpstreamTopic), true);
    if (!snapshot || current != *snapshot || upstream_now != upstream_snapshot) {
      snapshot = current;
      upstream_snapshot = upstream_now;
      stable_since = Clock::now();
    } else if (!(current.first.empty() && current.second.empty()) &&
               Clock::now() - stable_since >= std::chrono::seconds(2)) {
      break;
    }
  }
  const auto publishers = node->get_publishers_info_by_topic(kTopic);
  const auto subscribers = node->get_subscriptions_info_by_topic(kTopic);
  const auto upstream_publishers = node->get_publishers_info_by_topic(kUpstreamTopic);
  rclcpp::shutdown();

  const auto names = node_names(publishers, false);
  std::vector<std::string> problems;
  for (const auto * endpoints : {&publishers, &subscribers}) {
    for (const auto & info : *endpoints) {
      if (info.topic_type() != kType) {
        problems.push_back(info.node_name() + " uses " + info.topic_type() + ", expected " + kType);
      }
    }
  }
  if (!expect_publisher && !names.empty()) problems.push_back("expected no publisher, found " + list_text(names));
  if (expect_publisher && names != std::vector<std::string>{expected}) {
    problems.push_back("expected only " + expected + ", found " + (names.empty() ? "none" : list_text(names)));
  }

  if (check_upstream) {
    const auto upstream_names = node_names(upstream_publishers, false);
    for (const auto & info : upstream_publishers) {
      if (info.topic_type() != kType) {
        problems.push_back(info.node_name() + " uses " + info.topic_type() + " on " + kUpstreamTopic + ", expected " + kType);
      }
    }
    if (upstream_names != std::vector<std::string>{upstream}) {
      problems.push_back(std::string("expected only ") + upstream + " on " + kUpstreamTopic + ", found " +
                         (upstream_names.empty() ? "none" : list_text(upstream_names)));
    }
    std::cout << kUpstreamTopic << ": publishers=" << list_text(upstream_names) << std::endl;
  }
  std::cout << kTopic << ": publishers=" << list_text(names)
            << " subscribers=" << list_text(node_names(subscribers, false)) << std::endl;
  for (const auto & problem : problems) std::cout << "FAIL: " << problem << std::endl;
  return problems.empty() ? 0 : 1;
}
