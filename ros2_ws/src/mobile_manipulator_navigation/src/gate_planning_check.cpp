// Query each global planner across the ConstructionSiteV1 gates and report the route.
//
// Read-only: sends ComputePathToPose goals with explicit start poses and never commands
// motion. Requires global_planning.launch.py active (complete map -> base_footprint TF).
// Gate geometry comes from ConstructionSiteTools (Unity) converted with ROS x = Unity z,
// ROS y = -Unity x.
//
// Usage: gate_planning_check --label NAME [--planners GridBased Lattice] [--output FILE]
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "mobile_manipulator_control/cli.hpp"
#include "mobile_manipulator_navigation/scenario_metrics.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace fs = std::filesystem;
namespace mmn = mobile_manipulator_navigation;
using mmn::Json;
using ComputePath = nav2_msgs::action::ComputePathToPose;

namespace
{
struct Gate
{
  std::string name;
  mmn::Point2 start, goal;
  double heading;
  int axis;  // 0: the gate line is x = line; 1: y = line
  double line, low, high;
};

const std::vector<Gate> kGates = {
  // ManipulationRequiredGate_1p05m: posts at Unity z=-7.225, opening Unity x 7.20..8.25.
  {"gate_1p05", {-5.3, -7.725}, {-9.0, -7.725}, M_PI, 0, -7.225, -8.25, -7.20},
  // ControlledGate_1p35m: posts at Unity x=0, opening Unity z -8.00..-6.65.
  {"gate_1p35", {-7.325, 2.5}, {-7.325, -2.5}, -M_PI / 2, 1, 0.0, -8.0, -6.65},
};

geometry_msgs::msg::PoseStamped pose(const rclcpp::Node::SharedPtr & node, const mmn::Point2 & xy, double heading)
{
  geometry_msgs::msg::PoseStamped message;
  message.header.frame_id = "map";
  message.header.stamp = node->get_clock()->now();
  message.pose.position.x = xy[0];
  message.pose.position.y = xy[1];
  message.pose.orientation.z = std::sin(heading / 2);
  message.pose.orientation.w = std::cos(heading / 2);
  return message;
}

// Coordinate where the path crosses the gate line, if it does, and whether it is in the opening.
std::pair<std::optional<double>, bool> crossing(const std::vector<mmn::Point2> & points, const Gate & gate)
{
  const int i = gate.axis, o = 1 - gate.axis;
  for (size_t k = 1; k < points.size(); ++k) {
    const auto & a = points[k - 1];
    const auto & b = points[k];
    if ((a[i] - gate.line) * (b[i] - gate.line) <= 0 && a[i] != b[i]) {
      const double s = (gate.line - a[i]) / (b[i] - a[i]);
      const double other = a[o] + s * (b[o] - a[o]);
      return {other, gate.low <= other && other <= gate.high};
    }
  }
  return {std::nullopt, false};
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const mobile_manipulator_control::Args args(argc, argv);
  const std::string label = args.get("label");
  const std::vector<std::string> planners =
    args.has("planners") ? args.values("planners") : std::vector<std::string>{"GridBased", "Lattice"};

  auto node = std::make_shared<rclcpp::Node>("gate_planning_check");
  auto client = rclcpp_action::create_client<ComputePath>(node, "compute_path_to_pose");
  if (!client->wait_for_action_server(std::chrono::seconds(10))) {
    std::cerr << "compute_path_to_pose is not available" << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  Json results = {{"label", label}, {"cases", Json::object()}};
  for (const auto & gate : kGates) {
    for (const auto & planner : planners) {
      ComputePath::Goal request;
      request.start = pose(node, gate.start, gate.heading);
      request.goal = pose(node, gate.goal, gate.heading);
      request.planner_id = planner;
      request.use_start = true;
      auto handle_future = client->async_send_goal(request);
      rclcpp::spin_until_future_complete(node, handle_future);
      auto result_future = client->async_get_result(handle_future.get());
      rclcpp::spin_until_future_complete(node, result_future);
      const auto result = result_future.get().result;
      std::vector<mmn::Point2> points;
      for (const auto & p : result->path.poses) points.push_back({p.pose.position.x, p.pose.position.y});
      const auto [where, through] = points.empty() ? std::make_pair(std::optional<double>(), false) : crossing(points, gate);
      const Json entry = {
        {"error_code", result->error_code},
        {"planning_time_s", result->planning_time.sec + result->planning_time.nanosec * 1e-9},
        {"poses", points.size()},
        {"length_m", mmn::round_digits(mmn::polyline_length(points), 3)},
        {"straight_line_m", mmn::round_digits(std::hypot(gate.goal[0] - gate.start[0], gate.goal[1] - gate.start[1]), 3)},
        {"through_gate", through},
        {"gate_line_crossing", where ? Json(mmn::round_digits(*where, 3)) : Json(nullptr)},
      };
      results["cases"][gate.name + "/" + planner] = entry;
      char prefix[64];
      std::snprintf(prefix, sizeof(prefix), "%-15s %-10s %-10s ", label.c_str(), gate.name.c_str(), planner.c_str());
      std::cout << prefix << entry.dump() << std::endl;
    }
  }
  if (args.has("output")) {
    const fs::path output = args.get("output");
    if (output.has_parent_path()) fs::create_directories(output.parent_path());
    std::ofstream(output) << results.dump(2) << "\n";
  }
  rclcpp::shutdown();
  return 0;
}
