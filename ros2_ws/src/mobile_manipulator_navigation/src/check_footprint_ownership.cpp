// Fail unless the footprint and collision-monitor zone topics are published only by the owner
// footprint_mode requires (mission.hpp ownership_problems; ADR 0010).
//
// Read-only graph check to run after launching a scenario's navigation and manipulation.
//
// Usage: check_footprint_ownership --mode static|profiles|dynamic [--discovery-seconds S]
//   Waits until the publisher lists stop changing for 2 s (at most S, default 8 s).
#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "mobile_manipulator_control/cli.hpp"
#include "mobile_manipulator_navigation/mission.hpp"

namespace mmc = mobile_manipulator_control;
namespace mmn = mobile_manipulator_navigation;
using Clock = std::chrono::steady_clock;

namespace
{
std::map<std::string, std::vector<std::string>> publishers(const rclcpp::Node::SharedPtr & node)
{
  std::map<std::string, std::vector<std::string>> result;
  for (const auto & topic : mmn::kFootprintTopics) {
    auto & names = result[topic];
    for (const auto & info : node->get_publishers_info_by_topic(topic)) names.push_back(info.node_name());
    std::sort(names.begin(), names.end());
  }
  return result;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const mmc::Args args(argc, argv);
  const std::string mode = args.get("mode");
  const auto discovery = std::chrono::duration<double>(args.number("discovery-seconds", 8.0));
  auto node = std::make_shared<rclcpp::Node>("check_footprint_ownership");

  const auto deadline = Clock::now() + discovery;
  auto snapshot = publishers(node);
  auto stable_since = Clock::now();
  while (Clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const auto current = publishers(node);
    if (current != snapshot) {
      snapshot = current;
      stable_since = Clock::now();
    } else if (mode != "static" && mmn::ownership_problems(mode, current).empty() &&
               Clock::now() - stable_since >= std::chrono::seconds(2)) {
      break;
    }
  }
  rclcpp::shutdown();

  for (const auto & [topic, names] : snapshot) {
    std::cout << topic << ": publishers=[";
    for (size_t i = 0; i < names.size(); ++i) std::cout << (i ? ", " : "") << names[i];
    std::cout << "]" << std::endl;
  }
  const auto problems = mmn::ownership_problems(mode, snapshot);
  for (const auto & problem : problems) std::cout << "FAIL: " << problem << std::endl;
  return problems.empty() ? 0 : 1;
}
