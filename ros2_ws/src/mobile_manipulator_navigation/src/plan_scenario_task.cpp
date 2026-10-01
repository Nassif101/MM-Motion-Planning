// Planner-only scenario task: verify the placed start pose, then query each global planner.
//
// Read-only (ComputePathToPose; no motion). Used by tools/run_nav_scenario.py. Reports
// success, planner-reported and wall-clock latency, path length, and minimum clearance
// between path poses and occupied /map cells (the path reference point, not the footprint).
//
// Usage: plan_scenario_task --start X Y YAW --goal X Y YAW --planners NAME... --output FILE
//                           [--repeats N] [--start-tolerance M]
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mobile_manipulator_control/cli.hpp"
#include "mobile_manipulator_navigation/scenario_metrics.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace mmn = mobile_manipulator_navigation;
using ComputePath = nav2_msgs::action::ComputePathToPose;
using Clock = std::chrono::steady_clock;

namespace
{
geometry_msgs::msg::PoseStamped pose(const std::vector<double> & xyyaw, const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::PoseStamped message;
  message.header.frame_id = "map";
  message.header.stamp = stamp;
  message.pose.position.x = xyyaw[0];
  message.pose.position.y = xyyaw[1];
  message.pose.orientation.z = std::sin(xyyaw[2] / 2);
  message.pose.orientation.w = std::cos(xyyaw[2] / 2);
  return message;
}

int fail(const std::string & message)
{
  std::cerr << message << std::endl;
  rclcpp::shutdown();
  return 1;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const mobile_manipulator_control::Args args(argc, argv);
  const auto start = args.numbers("start", 3), goal = args.numbers("goal", 3);
  const auto planners = args.values("planners");
  const int repeats = static_cast<int>(args.number("repeats", 3));
  const double start_tolerance = args.number("start-tolerance", 0.10);
  const std::filesystem::path output = args.get("output");

  auto node = std::make_shared<rclcpp::Node>(
    "plan_scenario_task", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));
  tf2_ros::Buffer buffer(node->get_clock());
  tf2_ros::TransformListener listener(buffer, node);
  nav_msgs::msg::OccupancyGrid::SharedPtr map;
  auto map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local().reliable(),
    [&map](nav_msgs::msg::OccupancyGrid::SharedPtr message) { map = message; });
  auto client = rclcpp_action::create_client<ComputePath>(node, "compute_path_to_pose");

  const auto deadline = Clock::now() + std::chrono::seconds(20);
  while (Clock::now() < deadline && !(map && buffer.canTransform("map", "base_footprint", tf2::TimePointZero))) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!map) return fail("No /map received");
  geometry_msgs::msg::Transform placed;
  try {
    placed = buffer.lookupTransform("map", "base_footprint", tf2::TimePointZero).transform;
  } catch (const tf2::TransformException & error) {
    return fail(std::string("No map -> base_footprint transform: ") + error.what());
  }
  const double offset = std::hypot(placed.translation.x - start[0], placed.translation.y - start[1]);
  if (offset > start_tolerance) {
    char text[96];
    std::snprintf(text, sizeof(text), "Robot is %.3f m from the scenario start; placement failed", offset);
    return fail(text);
  }
  if (!client->wait_for_action_server(std::chrono::seconds(10))) return fail("compute_path_to_pose is not available");
  const auto obstacles = mmn::occupied_points(map->data, map->info.width, map->info.height, map->info.resolution,
                                              map->info.origin.position.x, map->info.origin.position.y);

  mmn::Json report = {{"start", start}, {"goal", goal},
                      {"placed_start_offset_m", mmn::round_digits(offset, 3)}, {"planners", mmn::Json::object()}};
  for (const auto & planner : planners) {
    mmn::Json runs = mmn::Json::array();
    int succeeded = 0;
    for (int i = 0; i < repeats; ++i) {
      ComputePath::Goal request;
      request.start = pose(start, node->get_clock()->now());
      request.goal = pose(goal, node->get_clock()->now());
      request.planner_id = planner;
      request.use_start = true;
      const auto began = Clock::now();
      auto handle_future = client->async_send_goal(request);
      rclcpp::spin_until_future_complete(node, handle_future);
      const auto handle = handle_future.get();
      if (!handle) return fail("ComputePathToPose goal rejected");
      auto result_future = client->async_get_result(handle);
      rclcpp::spin_until_future_complete(node, result_future);
      const double wall = std::chrono::duration<double>(Clock::now() - began).count();
      const auto result = result_future.get().result;
      std::vector<mmn::Point2> path;
      for (const auto & p : result->path.poses) path.push_back({p.pose.position.x, p.pose.position.y});
      const auto clearance = mmn::min_clearance(path, obstacles);
      succeeded += result->error_code == 0;
      runs.push_back({
        {"error_code", result->error_code},
        {"planner_time_s", result->planning_time.sec + result->planning_time.nanosec * 1e-9},
        {"wall_latency_s", mmn::round_digits(wall, 4)},
        {"length_m", mmn::round_digits(mmn::polyline_length(path), 3)},
        {"poses", path.size()},
        {"min_static_clearance_m", clearance ? mmn::Json(*clearance) : mmn::Json(nullptr)},
      });
    }
    report["planners"][planner] = {{"success_rate", static_cast<double>(succeeded) / runs.size()}, {"runs", runs}};
    std::cout << planner << " " << report["planners"][planner].dump() << std::endl;
  }

  std::filesystem::create_directories(output.parent_path());
  std::ofstream(output) << report.dump(2) << "\n";
  rclcpp::shutdown();
  return 0;
}
