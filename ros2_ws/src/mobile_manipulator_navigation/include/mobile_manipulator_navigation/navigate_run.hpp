#pragma once
// One NavigateToPose drive with the scenario metrics, shared by navigate_scenario_task and
// mission_scenario_task: the recorder node, the preflight checks, and the measured drive.
// All timing is simulation time unless named wall_*.
#include <array>
#include <chrono>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

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

#include "mobile_manipulator_navigation/scenario_metrics.hpp"
#include "mobile_manipulator_navigation/scenario_spec.hpp"
#include "mobile_manipulator_navigation/telemetry.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace mobile_manipulator_navigation
{
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateClient = rclcpp_action::Client<NavigateToPose>;

template <typename Stamp>
double stamp_seconds(const Stamp & stamp)
{
  return stamp.sec + stamp.nanosec * 1e-9;
}

// Processes whose CPU and memory the scenario tasks report.
extern const std::vector<std::string> kNavProcesses;

struct Usage
{
  double cpu_s = 0.0, rss_mb = 0.0;
};

// CPU seconds and peak RSS (MB) per process name, from /proc.
std::map<std::string, Usage> proc_cpu_mem(const std::vector<std::string> & names = kNavProcesses);

void write_report(const std::filesystem::path & output, const Json & report);

// {"cpu_percent_of_core": {name: %}, "max_rss_mb": {name: MB}} between two proc_cpu_mem
// samples taken wall_s apart.
Json usage_report(const std::map<std::string, Usage> & before, const std::map<std::string, Usage> & after,
                  double wall_s);

class Recorder : public rclcpp::Node
{
public:
  explicit Recorder(const std::string & name = "navigate_scenario_task")
  : Node(name, rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}})),
    buffer_(get_clock()), listener_(buffer_)
  {
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) { map = m; });
    plan_sub_ = create_subscription<nav_msgs::msg::Path>("/plan", 10, [this](nav_msgs::msg::Path::SharedPtr m) {
      plan.clear();
      for (const auto & p : m->poses) plan.push_back({p.pose.position.x, p.pose.position.y, 0.0});
    });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 50, [this](nav_msgs::msg::Odometry::SharedPtr m) { odom = m; });
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 50, [this](geometry_msgs::msg::Twist::SharedPtr m) { cmd.push_back({now_s(), m->linear.x, m->angular.z}); });
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/points_filtered", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr m) { lidar.push_back(stamp_seconds(m->header.stamp)); });
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap", 10,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) { costmap_stamps.push_back(stamp_seconds(m->header.stamp)); });
    movers_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/scenario/movers", 10, [this](geometry_msgs::msg::PoseArray::SharedPtr m) { movers = m; });
    monitor_sub_ = create_subscription<nav2_msgs::msg::CollisionMonitorState>(
      "/collision_monitor_state", 10, [this](nav2_msgs::msg::CollisionMonitorState::SharedPtr m) {
        monitor.push_back({now_s(), monitor_action_name(m->action_type), m->polygon_name});
      });
  }

  double now_s() { return get_clock()->now().seconds(); }

  std::optional<std::array<double, 3>> pose()
  {
    try {
      // Checking the frames first avoids tf2 warnings before the first transforms arrive.
      if (!buffer_._frameExists("map") || !buffer_._frameExists("base_footprint") ||
          !buffer_.canTransform("map", "base_footprint", tf2::TimePointZero)) {
        return std::nullopt;
      }
      const auto t = buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero).transform;
      return std::array<double, 3>{t.translation.x, t.translation.y,
                                   yaw_of(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)};
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr map;
  nav_msgs::msg::Odometry::SharedPtr odom;
  geometry_msgs::msg::PoseArray::SharedPtr movers;  // box centres, sorted by mover name
  std::vector<PathPoint> plan;
  std::vector<std::array<double, 3>> cmd;  // sim time, linear x, angular z
  std::vector<double> lidar, costmap_stamps;
  struct MonitorSample
  {
    double t;
    std::string action, polygon;
  };
  std::vector<MonitorSample> monitor;

private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_, costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<nav2_msgs::msg::CollisionMonitorState>::SharedPtr monitor_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr movers_sub_;
};

// Why a drive may not start (empty when it may): map, odom, TF and local costmap present,
// the robot within start_tolerance of `start`, the base stationary, collision_monitor the
// only /cmd_vel publisher, and the navigate_to_pose server available.
std::vector<std::string> navigation_preflight(Recorder & node, rclcpp::Executor & executor,
                                              const NavigateClient::SharedPtr & client, const Pose2 & start,
                                              double start_tolerance);

// One NavigateToPose goal from `start` to `goal` with the footprint `polygon`, measured as
// the navigate task reports it (status, path, clearances, cmd_vel, lidar, CPU, trajectory).
Json run_navigate(Recorder & node, rclcpp::Executor & executor, const NavigateClient::SharedPtr & client,
                  const Pose2 & start, const Pose2 & goal, const Json & polygon, double timeout,
                  const Json & obstacles, Json movers,
                  const std::vector<std::string> & processes = kNavProcesses);
}  // namespace mobile_manipulator_navigation
