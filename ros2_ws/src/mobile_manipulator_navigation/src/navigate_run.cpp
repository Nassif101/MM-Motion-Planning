#include "mobile_manipulator_navigation/navigate_run.hpp"

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <sstream>
#include <thread>

#include <action_msgs/msg/goal_status.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/msg/collision_monitor_state.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>



namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
using GoalHandle = rclcpp_action::ClientGoalHandle<mobile_manipulator_navigation::NavigateToPose>;

namespace mobile_manipulator_navigation
{
const std::vector<std::string> kNavProcesses = {"controller_server", "planner_server", "bt_navigator",
                                                "velocity_smoother", "collision_monitor", "behavior_server",
                                                "map_server", "livox_robot_filter"};

namespace
{
double since(Clock::time_point start)
{
  return std::chrono::duration<double>(Clock::now() - start).count();
}

std::string read_file(const fs::path & path)
{
  std::ifstream stream(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

}  // namespace

std::map<std::string, Usage> proc_cpu_mem(const std::vector<std::string> & names)
{
  const double tick = sysconf(_SC_CLK_TCK), page = sysconf(_SC_PAGE_SIZE);
  std::map<std::string, Usage> usage;
  for (const auto & entry : fs::directory_iterator("/proc")) {
    const std::string pid = entry.path().filename();
    if (pid.find_first_not_of("0123456789") != std::string::npos) continue;
    try {
      std::string cmdline = read_file(entry.path() / "cmdline");
      std::replace(cmdline.begin(), cmdline.end(), '\0', ' ');
      if (cmdline.find("ros2 launch") != std::string::npos) continue;
      const auto name = std::find_if(names.begin(), names.end(),
                                     [&](const std::string & n) { return cmdline.find(n) != std::string::npos; });
      if (name == names.end()) continue;
      // Launch-named Nav2 nodes carry __node:=; these run as plain executables.
      static const std::vector<std::string> plain = {"livox_robot_filter", "move_group", "reconfigure_panel_server"};
      if (cmdline.find("__node:=") == std::string::npos &&
          std::find(plain.begin(), plain.end(), *name) == plain.end()) {
        continue;
      }
      const std::string stat = read_file(entry.path() / "stat");
      std::istringstream fields(stat.substr(stat.rfind(')') + 1));
      std::vector<std::string> values{std::istream_iterator<std::string>(fields), {}};
      std::istringstream statm(read_file(entry.path() / "statm"));
      long size = 0, resident = 0;
      statm >> size >> resident;
      if (values.size() < 13 || !statm) continue;
      auto & total = usage[*name];
      total.cpu_s += (std::stod(values[11]) + std::stod(values[12])) / tick;
      total.rss_mb = std::max(total.rss_mb, resident * page / 1e6);
    } catch (const std::exception &) {
      continue;  // the process ended while being read
    }
  }
  return usage;
}

namespace
{
Json summary_json(const std::vector<double> & values)
{
  const auto summary = summarize(values);
  if (!summary) return nullptr;
  return {{"mean", summary->mean}, {"p95", summary->p95}, {"max", summary->max}};
}

}  // namespace

void write_report(const fs::path & output, const Json & report)
{
  fs::create_directories(output.parent_path());
  std::ofstream(output) << report.dump(2) << "\n";
}


namespace
{
std::string status_name(std::optional<rclcpp_action::ResultCode> code)
{
  if (!code) return "None";
  switch (*code) {
    case rclcpp_action::ResultCode::SUCCEEDED: return "succeeded";
    case rclcpp_action::ResultCode::ABORTED: return "aborted";
    case rclcpp_action::ResultCode::CANCELED: return "canceled";
    default: return std::to_string(static_cast<int>(*code));
  }
}
}  // namespace

std::vector<std::string> navigation_preflight(Recorder & node, rclcpp::Executor & executor,
                                              const NavigateClient::SharedPtr & client, const Pose2 & start,
                                              double start_tolerance)
{
  // ---- Preflight (no goal is sent unless every check passes) ----
  auto deadline = Clock::now() + std::chrono::seconds(30);
  while (Clock::now() < deadline && !(node.map && node.odom && node.pose() && node.costmap_stamps.size() >= 2)) {
    executor.spin_once(std::chrono::milliseconds(100));
  }
  std::vector<std::string> problems;
  const auto placed = node.pose();
  if (!node.map || !node.odom || !placed || node.costmap_stamps.size() < 2) {
    problems.push_back("map, odom, TF, or local costmap not available");
  } else {
    const double offset = std::hypot((*placed)[0] - start[0], (*placed)[1] - start[1]);
    if (offset > start_tolerance) {
      char text[64];
      std::snprintf(text, sizeof(text), "robot is %.3f m from the scenario start", offset);
      problems.push_back(text);
    }
    const auto & twist = node.odom->twist.twist;
    if (std::abs(twist.linear.x) > 0.02 || std::abs(twist.angular.z) > 0.02) problems.push_back("base is not stationary");
  }
  // A fresh node discovers the ~20-node graph gradually: wait until the /cmd_vel
  // publisher list is non-empty and unchanged for 2 s (at most 10 s).
  std::optional<std::vector<std::string>> publishers;
  auto stable_since = Clock::now();
  deadline = Clock::now() + std::chrono::seconds(10);
  while (Clock::now() < deadline) {
    executor.spin_once(std::chrono::milliseconds(100));
    std::vector<std::string> current;
    for (const auto & info : node.get_publishers_info_by_topic("/cmd_vel")) current.push_back(info.node_name());
    std::sort(current.begin(), current.end());
    if (!publishers || current != *publishers) {
      publishers = current;
      stable_since = Clock::now();
    } else if (!publishers->empty() && since(stable_since) >= 2.0) {
      break;
    }
  }
  if (!publishers || *publishers != std::vector<std::string>{"collision_monitor"}) {
    problems.push_back("/cmd_vel publishers are " + Json(publishers.value_or(std::vector<std::string>{})).dump() +
                       ", expected collision_monitor");
  }
  if (!client->wait_for_action_server(std::chrono::seconds(10))) problems.push_back("navigate_to_pose is not available");
  return problems;
}

Json run_navigate(Recorder & node, rclcpp::Executor & executor, const NavigateClient::SharedPtr & client,
                  const Pose2 & start, const Pose2 & goal_pose, const Json & polygon, double timeout,
                  const Json & obstacles, Json movers, const std::vector<std::string> & processes)
{
  // Unity publishes mover positions sorted by name; sizes come from the scenario.
  std::sort(movers.begin(), movers.end(), [](const Json & a, const Json & b) { return a.at("name") < b.at("name"); });
  const double x_first = polygon[0][0].get<double>(), y_first = polygon[0][1].get<double>();
  Bounds bounds{x_first, x_first, y_first, y_first};
  for (const auto & p : polygon) {
    bounds = {std::min(bounds.x0, p[0].get<double>()), std::max(bounds.x1, p[0].get<double>()),
              std::min(bounds.y0, p[1].get<double>()), std::max(bounds.y1, p[1].get<double>())};
  }
  const auto & grid = *node.map;
  const auto occupied = occupied_points(grid.data, grid.info.width, grid.info.height, grid.info.resolution,
                                             grid.info.origin.position.x, grid.info.origin.position.y);
  std::vector<Point2> obstacle_points;
  for (const auto & o : obstacles) {
    const auto outline = box_outline(o.at("x").get<double>(), o.at("y").get<double>(),
                                          o.at("size_x").get<double>(), o.at("size_y").get<double>());
    obstacle_points.insert(obstacle_points.end(), outline.begin(), outline.end());
  }

  // ---- Goal ----
  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node.get_clock()->now();
  goal.pose.pose.position.x = goal_pose[0];
  goal.pose.pose.position.y = goal_pose[1];
  goal.pose.pose.orientation.z = std::sin(goal_pose[2] / 2);
  goal.pose.pose.orientation.w = std::cos(goal_pose[2] / 2);
  int recoveries = 0;
  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.feedback_callback = [&recoveries](GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> f) {
    recoveries = f->number_of_recoveries;
  };

  const auto cpu_before = proc_cpu_mem(processes);
  const auto wall_before = Clock::now();
  const double started = node.now_s();
  node.cmd.clear();
  node.lidar.clear();
  node.costmap_stamps.clear();
  node.monitor.clear();
  auto handle_future = client->async_send_goal(goal, options);
  executor.spin_until_future_complete(handle_future);
  const auto handle = handle_future.get();
  if (!handle) throw std::runtime_error("NavigateToPose goal rejected");
  auto result_future = client->async_get_result(handle);
  const auto done = [&result_future] {
    return result_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  };

  std::vector<std::array<double, 4>> trajectory;  // sim time, x, y, yaw
  std::vector<double> cross_track, clearance, obstacle_clearance, mover_clearance;
  double next_sample = started;
  while (!done()) {
    executor.spin_once(std::chrono::milliseconds(20));
    const double now = node.now_s();
    if (now - started > timeout) {
      client->async_cancel_goal(handle);
      break;
    }
    if (now < next_sample) continue;
    next_sample = now + 0.05;
    const auto pose = node.pose();
    if (!pose) continue;
    const auto [x, y, yaw] = *pose;
    trajectory.push_back({now, x, y, yaw});
    // Distance to the path polyline, not to its nearest discrete pose (which overstated
    // the error by up to half the pose spacing before 2026-09-28).
    if (const auto d = mobile_manipulator_navigation::cross_track(x, y, node.plan)) cross_track.push_back(*d);
    if (const auto d = footprint_clearance(occupied, x, y, yaw, bounds, 8.0)) clearance.push_back(*d);
    if (const auto d = footprint_clearance(obstacle_points, x, y, yaw, bounds)) obstacle_clearance.push_back(*d);
    if (!movers.empty() && node.movers && node.movers->poses.size() == movers.size()) {
      std::vector<Point2> mover_points;
      for (size_t i = 0; i < movers.size(); ++i) {
        const auto & centre = node.movers->poses[i].position;
        const auto outline = box_outline(centre.x, centre.y, movers[i].at("size_x").get<double>(),
                                              movers[i].at("size_y").get<double>());
        mover_points.insert(mover_points.end(), outline.begin(), outline.end());
      }
      if (const auto d = footprint_clearance(mover_points, x, y, yaw, bounds)) mover_clearance.push_back(*d);
    }
  }
  const auto wait_until = Clock::now() + std::chrono::seconds(10);
  while (!done() && Clock::now() < wait_until) executor.spin_once(std::chrono::milliseconds(50));
  const double finished = node.now_s();
  const auto cpu_after = proc_cpu_mem(processes);
  const double wall_elapsed = since(wall_before);

  std::optional<rclcpp_action::ResultCode> status;
  Json error_code = nullptr;
  if (done()) {
    const auto wrapped = result_future.get();
    status = wrapped.code;
    if (wrapped.result) error_code = wrapped.result->error_code;
  }
  const auto final = node.pose();
  std::vector<Point2> points;
  for (const auto & sample : trajectory) points.push_back({sample[1], sample[2]});
  const double length = polyline_length(points);

  const auto & commands = node.cmd;
  std::vector<std::array<double, 3>> accel;  // time, linear, angular
  std::vector<std::array<double, 2>> jerk;
  for (size_t i = 1; i < commands.size(); ++i) {
    const auto & [t0, v0, w0] = commands[i - 1];
    const auto & [t1, v1, w1] = commands[i];
    if (t1 > t0) accel.push_back({t1, (v1 - v0) / (t1 - t0), (w1 - w0) / (t1 - t0)});
  }
  for (size_t i = 1; i < accel.size(); ++i) {
    const auto & [t0, a0, b0] = accel[i - 1];
    const auto & [t1, a1, b1] = accel[i];
    if (t1 > t0) jerk.push_back({(a1 - a0) / (t1 - t0), (b1 - b0) / (t1 - t0)});
  }
  Json transitions = Json::array();
  std::optional<std::pair<std::string, std::string>> previous;
  for (const auto & sample : node.monitor) {
    const std::pair<std::string, std::string> state{sample.action, sample.polygon};
    if (state != previous && sample.action != "none") {
      transitions.push_back({{"t", round_digits(sample.t - started, 2)}, {"action", sample.action},
                             {"polygon", sample.polygon}});
    }
    previous = state;
  }
  std::vector<double> gaps, costmap_gaps;
  for (size_t i = 1; i < node.lidar.size(); ++i) gaps.push_back(node.lidar[i] - node.lidar[i - 1]);
  for (size_t i = 1; i < node.costmap_stamps.size(); ++i) {
    costmap_gaps.push_back(node.costmap_stamps[i] - node.costmap_stamps[i - 1]);
  }
  const auto absolute = [](const auto & rows, size_t column) {
    std::vector<double> values;
    for (const auto & row : rows) values.push_back(std::abs(row[column]));
    return values;
  };
  const auto minimum = [](const std::vector<double> & values) -> Json {
    if (values.empty()) return nullptr;
    return round_digits(*std::min_element(values.begin(), values.end()), 3);
  };

  Json cpu_percent = Json::object(), max_rss = Json::object();
  for (const auto & [name, usage] : cpu_after) {
    const double before = cpu_before.count(name) ? cpu_before.at(name).cpu_s : 0.0;
    cpu_percent[name] = round_digits(100 * (usage.cpu_s - before) / wall_elapsed, 1);
    max_rss[name] = round_digits(usage.rss_mb, 1);
  }
  Json sampled = Json::array();
  for (size_t i = 0; i < trajectory.size(); i += 4) {
    Json row = Json::array();
    for (const double v : trajectory[i]) row.push_back(round_digits(v, 3));
    sampled.push_back(row);
  }
  int long_gaps = 0;
  for (const double g : gaps) long_gaps += g > 0.5;

  Json report = {
    {"status", status_name(status)},
    {"error_code", error_code},
    {"timed_out", finished - started > timeout},
    {"time_s", round_digits(finished - started, 2)},
    {"wall_s", round_digits(wall_elapsed, 2)},
    {"path_length_m", round_digits(length, 3)},
    {"straight_line_m", round_digits(std::hypot(start[0] - goal_pose[0], start[1] - goal_pose[1]), 3)},
    {"final_position_error_m",
     final ? Json(round_digits(std::hypot((*final)[0] - goal_pose[0], (*final)[1] - goal_pose[1]), 3)) : Json(nullptr)},
    {"final_yaw_error_rad",
     final ? Json(round_digits(std::abs(std::remainder((*final)[2] - goal_pose[2], 2 * M_PI)), 3)) : Json(nullptr)},
    {"cross_track_m", summary_json(cross_track)},
    {"min_footprint_clearance_to_static_map_m", minimum(clearance)},
    {"min_footprint_clearance_to_obstacles_m", minimum(obstacle_clearance)},
    {"min_footprint_clearance_to_movers_m", minimum(mover_clearance)},
    {"recoveries", recoveries},
    {"collision_monitor_activations", transitions},
    {"cmd_vel", {{"messages", commands.size()},
                 {"abs_linear_accel", summary_json(absolute(accel, 1))},
                 {"abs_angular_accel", summary_json(absolute(accel, 2))},
                 {"abs_linear_jerk", summary_json(absolute(jerk, 0))},
                 {"abs_angular_jerk", summary_json(absolute(jerk, 1))}}},
    {"lidar_gaps_over_0p5s", long_gaps},
    {"max_lidar_gap_s", gaps.empty() ? Json(nullptr) : Json(round_digits(*std::max_element(gaps.begin(), gaps.end()), 3))},
    {"local_costmap_publish_interval_s", summary_json(costmap_gaps)},
    {"cpu_percent_of_core", cpu_percent},
    {"max_rss_mb", max_rss},
    {"trajectory", sampled},
  };
  return report;
}
}  // namespace mobile_manipulator_navigation
