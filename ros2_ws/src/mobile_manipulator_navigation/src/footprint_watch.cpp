#include "mobile_manipulator_navigation/footprint_watch.hpp"

#include <chrono>

#include "mobile_manipulator_geometry/polygon_ops.hpp"

namespace mobile_manipulator_navigation
{
FootprintWatch::FootprintWatch(std::vector<MonitorZone> zones, const std::string & name,
                               std::vector<std::string> costmap_topics)
: Node(name, rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}})), zones_(std::move(zones)),
  costmap_topics_(std::move(costmap_topics))
{
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  std::vector<std::pair<std::string, rclcpp::QoS>> topics;
  for (const auto & topic : costmap_topics_) topics.push_back({topic, rclcpp::QoS(10)});
  for (const auto & zone : zones_) topics.push_back({zone.polygon_topic, latched});
  for (const auto & [topic, qos] : topics) {
    subs_.push_back(create_subscription<geometry_msgs::msg::PolygonStamped>(
      topic, qos, [this, topic = topic](const geometry_msgs::msg::PolygonStamped & m) {
        Polygon polygon;
        for (const auto & p : m.polygon.points) polygon.push_back({p.x, p.y});
        std::lock_guard<std::mutex> lock(mutex_);
        latest_[topic] = {polygon, m.header.frame_id, wall_seconds()};
      }));
  }
  subs_.push_back(create_subscription<geometry_msgs::msg::PolygonStamped>(
    "/dynamic_footprint/footprint", latched, [this](const geometry_msgs::msg::PolygonStamped & m) {
      Polygon polygon;
      for (const auto & p : m.polygon.points) polygon.push_back({p.x, p.y});
      std::lock_guard<std::mutex> lock(mutex_);
      dynamic_ = polygon;
    }));
}

bool FootprintWatch::all_match(const Polygon & profile, const std::optional<Pose2> & robot, double since)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return robot && footprints_applied(latest_, costmap_topics_, zones_, profile, *robot, since,
                                     kCostmapFootprintPadding);
}

bool FootprintWatch::all_match_dynamic(const Polygon & footprint, const std::optional<Pose2> & robot, double since)
{
  std::vector<Polygon> zone_polygons;
  for (const auto & zone : zones_) zone_polygons.push_back(mobile_manipulator_geometry::offset_outward(footprint, zone.margin_m));
  std::lock_guard<std::mutex> lock(mutex_);
  return robot && footprints_applied(latest_, costmap_topics_, zones_, footprint, zone_polygons, *robot, since,
                                     kCostmapFootprintPadding);
}

std::optional<Polygon> FootprintWatch::dynamic_footprint()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return dynamic_;
}

double FootprintWatch::wall_seconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace mobile_manipulator_navigation
