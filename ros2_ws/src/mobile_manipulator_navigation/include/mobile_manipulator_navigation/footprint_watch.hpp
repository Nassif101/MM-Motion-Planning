#pragma once
// Latest footprint each costmap publishes, the collision-monitor zone inputs, and the B4
// dynamic footprint, for checking that a footprint change has reached Nav2 before a drive.
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mobile_manipulator_navigation/mission.hpp"

namespace mobile_manipulator_navigation
{
inline const std::vector<std::string> kCostmapFootprintTopics = {"/local_costmap/published_footprint",
                                                                 "/global_costmap/published_footprint"};
// Costmap footprint_padding in nav2_global_planning.yaml and nav2_local_costmap.yaml.
constexpr double kCostmapFootprintPadding = 0.01;

// The collision monitor only publishes its own zones while velocity commands flow, so the
// zones are read from the latched topics it takes them from.
class FootprintWatch : public rclcpp::Node
{
public:
  // `costmap_topics`: the published_footprint topics that must show the footprint.
  explicit FootprintWatch(std::vector<MonitorZone> zones, const std::string & name = "footprint_watch",
                          std::vector<std::string> costmap_topics = kCostmapFootprintTopics);

  // Whether the costmaps publish `profile` (messages received at or after `since`, wall
  // seconds) and every zone input is `profile` grown by its margin as a padded rectangle
  // (named profiles, footprints_applied).
  bool all_match(const Polygon & profile, const std::optional<Pose2> & robot, double since = 0.0);
  // The same for a dynamic footprint, whose zones are offset_outward(footprint, margin).
  bool all_match_dynamic(const Polygon & footprint, const std::optional<Pose2> & robot, double since = 0.0);
  // Latest /dynamic_footprint/footprint (latched), if any.
  std::optional<Polygon> dynamic_footprint();

  static double wall_seconds();

private:
  std::vector<MonitorZone> zones_;
  std::vector<std::string> costmap_topics_;
  std::mutex mutex_;
  std::map<std::string, PublishedPolygon> latest_;
  std::optional<Polygon> dynamic_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr> subs_;
};
}  // namespace mobile_manipulator_navigation
