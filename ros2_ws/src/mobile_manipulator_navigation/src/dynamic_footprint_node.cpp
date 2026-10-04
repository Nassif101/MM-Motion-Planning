// Configuration-dependent Nav2 footprint for baseline B4 (Sagar et al., CoDIT 2026; ADR 0010).
//
// At rate_hz it projects base, arm and the attached panel for the latest /joint_states,
// hulls and offsets the projection (DynamicFootprint, config/dynamic_footprint.yaml) and,
// when the padded hull has moved by more than change_threshold_m, publishes it to both
// costmaps' footprint topics, each collision-monitor zone input (hull + zone margin), and
// /dynamic_footprint/footprint. In footprint_mode:=dynamic it is the only publisher of those
// topics (check_footprint_ownership). It never shrinks the footprint without fresh joint data.
//
// Parameters: model ("" = the config's), stats_file (written on shutdown, "" = none).
#include <algorithm>
#include <chrono>
#include <fstream>
#include <numeric>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "mobile_manipulator_navigation/dynamic_footprint.hpp"
#include "mobile_manipulator_navigation/footprint_config.hpp"
#include "mobile_manipulator_navigation/mission.hpp"

namespace mmg = mobile_manipulator_geometry;
namespace mmn = mobile_manipulator_navigation;
using namespace std::chrono_literals;

namespace
{
const std::array<const char *, 2> kCostmaps = {"/global_costmap", "/local_costmap"};

std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot read " + path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

double steady_seconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

geometry_msgs::msg::Polygon polygon_msg(const mmn::Polygon & polygon)
{
  geometry_msgs::msg::Polygon message;
  for (const auto & [x, y] : polygon) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(x);
    p.y = static_cast<float>(y);
    message.points.push_back(p);
  }
  return message;
}
}  // namespace

class DynamicFootprintNode : public rclcpp::Node
{
public:
  DynamicFootprintNode()
  : Node("dynamic_footprint_node")
  {
    const auto share = [](const std::string & p) { return ament_index_cpp::get_package_share_directory(p); };
    const mmn::ScenarioConfig config(share("mobile_manipulator_navigation"));
    const auto parameters = config.load("dynamic_footprint.yaml").at("dynamic_footprint_node").at("ros__parameters");
    model_name_ = declare_parameter("model", std::string(""));
    if (model_name_.empty()) model_name_ = parameters.at("model").get<std::string>();
    stats_file_ = declare_parameter("stats_file", std::string(""));

    const auto payload = mmn::Json::parse(read_file(share("mobile_manipulator_control") + "/config/qualified_payload.json"));
    const auto projector = std::make_shared<const mmg::FootprintProjector>(
      read_file(share("mobile_manipulator_description") + "/urdf/mobile_manipulator.urdf"), mmg::payload_from_json(payload));
    zones_ = mmn::monitor_zones(config.load("nav2_navigation.yaml"));
    std::vector<double> margins;
    for (const auto & zone : zones_) margins.push_back(zone.margin_m);
    inflation_radius_ = config.load("nav2_global_planning.yaml").at("global_costmap").at("global_costmap")
                          .at("ros__parameters").at("inflation_layer").at("inflation_radius").get<double>();
    nav2_padding_ = config.load("nav2_local_costmap.yaml").at("local_costmap").at("local_costmap")
                      .at("ros__parameters").at("footprint_padding").get<double>();
    footprint_ = std::make_unique<mmn::DynamicFootprint>(
      mmn::footprint_model(model_name_, parameters, projector), payload.at("joint_order").get<std::vector<std::string>>(),
      margins, parameters.at("padding_m").get<double>(), parameters.at("change_threshold_m").get<double>(),
      parameters.at("stale_after_s").get<double>(), inflation_radius_);

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    for (const auto * costmap : kCostmaps) {
      footprint_pubs_.push_back(create_publisher<geometry_msgs::msg::Polygon>(std::string(costmap) + "/footprint", latched));
    }
    for (const auto & zone : zones_) {
      zone_pubs_.push_back(create_publisher<geometry_msgs::msg::PolygonStamped>(zone.polygon_topic, latched));
    }
    state_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>("/dynamic_footprint/footprint", latched);

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 50, [this](const sensor_msgs::msg::JointState & m) {
        footprint_->joints(steady_seconds(), m.name, m.position);
        joint_stamp_ = m.header.stamp;
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 20, [this](const nav_msgs::msg::Odometry & m) {
        still_.add(steady_seconds(), std::hypot(m.twist.twist.linear.x, m.twist.twist.linear.y), m.twist.twist.angular.z);
        const auto & q = m.pose.pose.orientation;
        pose_ = {m.pose.pose.position.x, m.pose.pose.position.y,
                 std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
      });
    for (size_t i = 0; i < kCostmaps.size(); ++i) {
      published_subs_.push_back(create_subscription<geometry_msgs::msg::PolygonStamped>(
        std::string(kCostmaps[i]) + "/published_footprint", 10, [this, i](const geometry_msgs::msg::PolygonStamped & m) {
          mmn::Polygon polygon;
          for (const auto & p : m.polygon.points) polygon.push_back({p.x, p.y});
          published_[i] = {polygon, m.header.frame_id, steady_seconds()};
        }));
    }
    const double rate = parameters.at("rate_hz").get<double>();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "Dynamic footprint (%s model) at %.0f Hz", model_name_.c_str(), rate);
  }

  void write_stats() const
  {
    if (stats_file_.empty()) return;
    auto times = compute_us_;
    std::sort(times.begin(), times.end());
    mmn::Json stats{{"model", model_name_}, {"ticks", times.size()}, {"publishes", publish_times_.size()},
                    {"publish_times_s", publish_times_}, {"points", points_},
                    {"beyond_inflation_ticks", beyond_inflation_ticks_}};
    if (!times.empty()) {
      stats["compute_us"] = {{"mean", std::accumulate(times.begin(), times.end(), 0.0) / times.size()},
                             {"min", times.front()}, {"max", times.back()},
                             {"p99", times[std::min(times.size() - 1, static_cast<size_t>(0.99 * times.size()))]}};
      stats["vertices"] = {{"min", vertices_min_}, {"max", vertices_max_}};
    }
    std::ofstream(stats_file_) << stats.dump(2) << "\n";
  }

private:
  void tick()
  {
    const double now = steady_seconds();
    const auto health = footprint_->health(now);
    if (health != mmn::DynamicFootprint::Health::Fresh) {
      if (health != mmn::DynamicFootprint::Health::NoData) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s joint state; keeping the last footprint",
                             health == mmn::DynamicFootprint::Health::Stale ? "Stale" : "Incomplete");
      }
      return;
    }
    const auto update = footprint_->tick(now);
    const auto & stats = footprint_->last_stats();
    compute_us_.push_back(stats.compute_s * 1e6);
    points_ = stats.points;
    vertices_min_ = std::min(vertices_min_, stats.vertices);
    vertices_max_ = std::max(vertices_max_, stats.vertices);
    if (stats.beyond_inflation) {
      ++beyond_inflation_ticks_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Footprint reaches beyond the %.2f m inflation radius; Smac collision checks may "
                           "misjudge it", inflation_radius_);
    }
    if (update) {
      publish(*update);
      return;
    }
    repair(now);
  }

  void publish(const mmn::FootprintUpdate & update)
  {
    const auto polygon = polygon_msg(update.footprint);
    for (const auto & pub : footprint_pubs_) pub->publish(polygon);
    for (size_t i = 0; i < zone_pubs_.size(); ++i) {
      geometry_msgs::msg::PolygonStamped zone;
      zone.header.frame_id = "base_footprint";
      zone.header.stamp = now();
      zone.polygon = polygon_msg(update.zones[i]);
      zone_pubs_[i]->publish(zone);
    }
    geometry_msgs::msg::PolygonStamped state;
    state.header.frame_id = "base_footprint";
    state.header.stamp = joint_stamp_;
    state.polygon = polygon;
    state_pub_->publish(state);
    last_publish_ = steady_seconds();
    publish_times_.push_back(now().seconds());
  }

  // Put the footprint back on a costmap that shows another one for 2 s with the base at rest
  // (a relaunched Nav2 subscribes after the latched message and keeps its launch footprint).
  void repair(double now)
  {
    const auto & current = footprint_->current();
    if (!current) return;
    const bool judged = still_.still_for(now) >= 0.5 && now - last_publish_ >= 2.0;
    for (size_t i = 0; i < kCostmaps.size(); ++i) {
      const auto & seen = published_[i];
      const bool matches = !judged || seen.received < now - 2.0 ||
                           mmn::footprint_matches(mmn::to_base_frame(seen.polygon, pose_), *current, nav2_padding_);
      if (drift_[i].republish(matches, now)) {
        RCLCPP_WARN(get_logger(), "%s/published_footprint does not show the dynamic footprint; republishing it",
                    kCostmaps[i]);
        footprint_pubs_[i]->publish(polygon_msg(*current));
      }
    }
  }

  std::string model_name_, stats_file_;
  std::unique_ptr<mmn::DynamicFootprint> footprint_;
  std::vector<mmn::MonitorZone> zones_;
  double inflation_radius_ = 1.0, nav2_padding_ = 0.01;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr> footprint_pubs_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr> zone_pubs_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr> published_subs_;
  rclcpp::TimerBase::SharedPtr timer_;
  builtin_interfaces::msg::Time joint_stamp_;
  mmn::Stillness still_{0.01, 0.02};
  mmn::Pose2 pose_{0.0, 0.0, 0.0};
  std::array<mmn::PublishedPolygon, 2> published_{};
  std::array<mmn::FootprintDriftGuard, 2> drift_;
  double last_publish_ = 0.0;
  std::vector<double> compute_us_, publish_times_;
  size_t points_ = 0, vertices_min_ = SIZE_MAX, vertices_max_ = 0, beyond_inflation_ticks_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamicFootprintNode>();
  rclcpp::spin(node);
  node->write_stats();
  rclcpp::shutdown();
  return 0;
}
