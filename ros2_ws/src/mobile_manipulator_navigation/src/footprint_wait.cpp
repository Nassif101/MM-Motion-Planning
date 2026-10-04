// Wait until both costmaps use the expected footprint before a plan-only or navigate task.
//
// static: the costmaps publish the launch profile (--profile NAME); dynamic: they publish the
// latest /dynamic_footprint/footprint. Two matching messages received after the start must
// arrive on every costmap (FootprintRefresh: a full update cycle on the footprint), and with
// --zones every collision-monitor zone input must match too (the profile's padded
// rectangles, or the dynamic footprint's outward offsets).
//
// --global-only: plan-only runs have no local costmap.
//
// Usage: footprint_wait --mode static|dynamic [--profile NAME] [--zones] [--global-only] [--timeout S]
// Exit codes: 0 applied, 1 timeout or bad arguments.
#include <chrono>
#include <iostream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mobile_manipulator_control/cli.hpp"
#include "mobile_manipulator_navigation/footprint_watch.hpp"
#include "mobile_manipulator_navigation/navigate_run.hpp"

namespace mmn = mobile_manipulator_navigation;
using Clock = std::chrono::steady_clock;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const mobile_manipulator_control::Args args(argc, argv);
  const std::string mode = args.get("mode");
  const bool zones_too = args.has("zones");
  const double timeout = args.number("timeout", 30.0);
  if (mode != "static" && mode != "dynamic") {
    std::cerr << "--mode must be static or dynamic" << std::endl;
    return 1;
  }
  const mmn::ScenarioConfig config(ament_index_cpp::get_package_share_directory("mobile_manipulator_navigation"));
  mmn::Polygon profile;
  if (mode == "static") {
    if (!args.has("profile")) {
      std::cerr << "--mode static needs --profile NAME" << std::endl;
      return 1;
    }
    profile = mmn::polygon_of(config.load("footprint_profiles.yaml").at("profiles").at(args.get("profile")).at("polygon"));
  }
  const auto zones = zones_too ? mmn::monitor_zones(config.load("nav2_navigation.yaml")) : std::vector<mmn::MonitorZone>{};
  auto pose = std::make_shared<mmn::Recorder>("footprint_wait_pose");
  const auto costmaps = args.has("global-only") ? std::vector<std::string>{"/global_costmap/published_footprint"}
                                                : mmn::kCostmapFootprintTopics;
  auto watch = std::make_shared<mmn::FootprintWatch>(zones, "footprint_wait", costmaps);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pose);
  executor.add_node(watch);

  mmn::FootprintRefresh refresh(mmn::FootprintWatch::wall_seconds());
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout);
  while (Clock::now() < deadline) {
    executor.spin_once(std::chrono::milliseconds(20));
    bool applied = false;
    if (mode == "static") {
      applied = watch->all_match(profile, pose->pose(), refresh.since());
    } else if (const auto footprint = watch->dynamic_footprint()) {
      applied = watch->all_match_dynamic(*footprint, pose->pose(), refresh.since());
    }
    if (refresh.observe(applied, mmn::FootprintWatch::wall_seconds())) {
      std::cout << "footprint applied (" << mode << ")" << std::endl;
      rclcpp::shutdown();
      return 0;
    }
  }
  std::cout << "FAIL: the costmaps " << (zones_too ? "and zones " : "") << "do not show the "
            << (mode == "static" ? "profile " + args.get("profile") : std::string("dynamic footprint"))
            << (refresh.shown() ? " (shown, but no full update cycle)" : "") << std::endl;
  rclcpp::shutdown();
  return 1;
}
