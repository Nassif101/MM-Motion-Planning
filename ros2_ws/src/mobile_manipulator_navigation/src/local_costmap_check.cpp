// Read-only checks of /local_costmap/costmap for the perception contract.
//
// snapshot: count lethal cells and report the nearest one to the robot.
// wait:     wait until the cell at (x, y) in the costmap frame becomes lethal
//           (--state lethal) or free (--state free) and print the simulated stamp of
//           the first costmap update that shows it, for latency against a Unity event.
// count:    count lethal (100) cells within --half metres of (x, y) in the next update.
//
// Usage: local_costmap_check snapshot|wait|count [--x X --y Y] [--state lethal|free]
//                            [--half 0.35] [--timeout 10 (wall seconds)]
// Prints one JSON line; exits 1 on timeout.
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mobile_manipulator_control/cli.hpp"
#include "mobile_manipulator_navigation/scenario_metrics.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace mmn = mobile_manipulator_navigation;
using mmn::Json;
using Clock = std::chrono::steady_clock;

namespace
{
constexpr int kLethal = 99;  // costmap_2d publishes inscribed (99) and lethal (100) as occupied

std::optional<int> cell_value(const nav_msgs::msg::OccupancyGrid & grid, double x, double y)
{
  const auto & info = grid.info;
  const int col = static_cast<int>((x - info.origin.position.x) / info.resolution);
  const int row = static_cast<int>((y - info.origin.position.y) / info.resolution);
  if (!(col >= 0 && col < static_cast<int>(info.width) && row >= 0 && row < static_cast<int>(info.height))) {
    return std::nullopt;
  }
  return grid.data[row * info.width + col];
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const mobile_manipulator_control::Args args(argc, argv);
  if (args.positional().size() != 1) {
    std::cerr << "usage: local_costmap_check snapshot|wait|count [options]" << std::endl;
    return 2;
  }
  const std::string mode = args.positional()[0];
  if (mode != "snapshot" && mode != "wait" && mode != "count") {
    std::cerr << "mode must be snapshot, wait or count" << std::endl;
    return 2;
  }
  const std::string state = args.get("state", "lethal");
  const double half = args.number("half", 0.35), timeout = args.number("timeout", 10.0);
  const double x = args.number("x", std::nan("")), y = args.number("y", std::nan(""));

  auto node = std::make_shared<rclcpp::Node>(
    "local_costmap_check", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));
  tf2_ros::Buffer buffer(node->get_clock());
  tf2_ros::TransformListener listener(buffer, node);
  std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> grids;
  auto subscription = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap/costmap", 10, [&grids](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { grids.push_back(m); });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout);
  std::optional<Json> report;
  while (Clock::now() < deadline && !report) {
    executor.spin_once(std::chrono::milliseconds(50));
    if (grids.empty()) continue;
    const auto & grid = *grids.back();
    const double stamp = grid.header.stamp.sec + grid.header.stamp.nanosec * 1e-9;
    const auto & info = grid.info;
    if (mode == "count") {
      int near = 0;
      for (uint32_t row = 0; row < info.height; ++row) {
        for (uint32_t col = 0; col < info.width; ++col) {
          if (grid.data[row * info.width + col] != 100) continue;
          const double cx = info.origin.position.x + (col + 0.5) * info.resolution;
          const double cy = info.origin.position.y + (row + 0.5) * info.resolution;
          near += std::abs(cx - x) <= half && std::abs(cy - y) <= half;
        }
      }
      report = Json{{"costmap_stamp", stamp}, {"lethal_cells", near}};
      continue;
    }
    if (mode == "wait") {
      const auto value = cell_value(grid, x, y);
      if (value && (state == "lethal" ? *value >= kLethal : *value < kLethal)) {
        report = Json{{"state", state}, {"cell_value", *value}, {"costmap_stamp", stamp}};
      }
      grids.clear();
      continue;
    }
    if (!buffer._frameExists(grid.header.frame_id) || !buffer._frameExists("base_footprint") ||
        !buffer.canTransform(grid.header.frame_id, "base_footprint", tf2::TimePointZero)) {
      continue;
    }
    const auto robot = buffer.lookupTransform(grid.header.frame_id, "base_footprint", tf2::TimePointZero).transform;
    long lethal = 0, nonzero = 0;
    double nearest = std::numeric_limits<double>::infinity();
    for (uint32_t row = 0; row < info.height; ++row) {
      for (uint32_t col = 0; col < info.width; ++col) {
        const int value = grid.data[row * info.width + col];
        nonzero += value > 0;
        if (value < kLethal) continue;
        ++lethal;
        const double cx = info.origin.position.x + (col + 0.5) * info.resolution;
        const double cy = info.origin.position.y + (row + 0.5) * info.resolution;
        nearest = std::min(nearest, std::hypot(cx - robot.translation.x, cy - robot.translation.y));
      }
    }
    report = Json{{"costmap_stamp", stamp},
                  {"frame", grid.header.frame_id},
                  {"size_cells", {info.width, info.height}},
                  {"lethal_cells", lethal},
                  {"nearest_lethal_m", lethal ? Json(mmn::round_digits(nearest, 3)) : Json(nullptr)},
                  {"nonzero_cost_cells", nonzero}};
  }
  rclcpp::shutdown();
  const Json output = report ? *report : Json{{"timeout", true}, {"mode", mode}};
  std::cout << output.dump() << std::endl;
  return report ? 0 : 1;
}
