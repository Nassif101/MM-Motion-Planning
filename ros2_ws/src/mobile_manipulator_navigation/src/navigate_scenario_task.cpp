// Navigate-to-pose scenario task: preflight, one NavigateToPose goal, and run metrics.
//
// Used by tools/run_nav_scenario.py with navigation.launch.py active. Preflight refuses to
// send a goal unless the placed pose matches the scenario start, the base is stationary on
// /odom, the local costmap is publishing, and the collision monitor is the only /cmd_vel
// publisher. All timing is simulation time unless named wall_*.
//
// Metrics: success and error code, time to goal, executed path length, final position and
// heading error, cross-track error to the latest /plan, minimum clearance between the
// posed footprint polygon and occupied static-map cells (and, separately, the scenario's
// unmapped obstacle boxes and, from Unity's ground truth on /scenario/movers, its movers),
// recoveries, collision-monitor activations, /cmd_vel
// acceleration and jerk, filtered-lidar gaps over 0.5 s, local costmap publish interval,
// and CPU/memory of the navigation processes.
//
// Usage: navigate_scenario_task --start X Y YAW --goal X Y YAW --footprint-profile NAME
//          --timeout SIM_S --output FILE [--start-tolerance M] [--obstacles JSON] [--movers JSON]
// Exit codes: 0 succeeded, 3 the goal ran but did not succeed, 1 preflight or setup failure.
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mobile_manipulator_control/cli.hpp"
#include "mobile_manipulator_navigation/navigate_run.hpp"

namespace mmn = mobile_manipulator_navigation;
using mmn::Json;

namespace
{
int fail(const std::string & message)
{
  std::cerr << message << std::endl;
  rclcpp::shutdown();
  return 1;
}

mmn::Pose2 pose3(const std::vector<double> & values) { return {values.at(0), values.at(1), values.at(2)}; }
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const mobile_manipulator_control::Args args(argc, argv);
  const auto start = pose3(args.numbers("start", 3)), goal = pose3(args.numbers("goal", 3));
  const double timeout = std::stod(args.get("timeout"));
  const double start_tolerance = args.number("start-tolerance", 0.10);
  const Json obstacles = Json::parse(args.get("obstacles", "[]"));
  const Json movers = Json::parse(args.get("movers", "[]"));
  const std::filesystem::path output = args.get("output");

  const auto share = ament_index_cpp::get_package_share_directory("mobile_manipulator_navigation");
  const auto polygon = mmn::load_yaml_file(share + "/config/footprint_profiles.yaml")
                         .at("profiles").at(args.get("footprint-profile")).at("polygon");
  const auto tolerance = mmn::goal_tolerance(mmn::load_yaml_file(share + "/config/nav2_navigation.yaml"));

  auto node = std::make_shared<mmn::Recorder>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto client = rclcpp_action::create_client<mmn::NavigateToPose>(node, "navigate_to_pose");

  // ---- Preflight (no goal is sent unless every check passes) ----
  const auto problems = mmn::navigation_preflight(*node, executor, client, start, start_tolerance);
  if (!problems.empty()) {
    mmn::write_report(output, {{"preflight_failed", problems}});
    std::string text = "Preflight failed: ";
    for (size_t i = 0; i < problems.size(); ++i) text += (i ? "; " : "") + problems[i];
    return fail(text);
  }

  Json report;
  try {
    report = mmn::run_navigate(*node, executor, client, start, goal, polygon, timeout, obstacles, movers, tolerance);
  } catch (const std::exception & error) {
    return fail(error.what());
  }
  mmn::write_report(output, report);
  Json summary = report;
  for (const char * key : {"trajectory", "cpu_percent_of_core", "max_rss_mb"}) summary.erase(key);
  std::cout << summary.dump() << std::endl;
  rclcpp::shutdown();
  return report["status"] == "succeeded" ? 0 : 3;
}
