// Configuration-dependent Nav2 footprint for baseline B4 (Sagar et al., CoDIT 2026; ADR 0010).
//
// At rate_hz it projects base, arm and the attached panel for the latest /joint_states,
// hulls and offsets the projection (DynamicFootprint, config/dynamic_footprint.yaml) and,
// when the padded hull has moved by more than change_threshold_m (while /cmd_vel is non-zero: only
// when it grows beyond the current hull by more than that, then the union), publishes it to both
// costmaps' footprint topics, each collision-monitor zone input (hull + zone margin), and
// /dynamic_footprint/footprint. In footprint_mode:=dynamic it is the only publisher of those
// topics (check_footprint_ownership). It never shrinks the footprint without fresh joint data.
//
// footprint_mode:=wholebody (baseline B5, whole-body MPC): both costmaps and the stop zone get the
// base-only footprint (published once); the slowdown zone follows the hull swept by the
// configurations of the latest wholebody_mpc/plan, each placed at its predicted base pose relative
// to the current base pose (TF map -> base_footprint), and falls back to the hull of the current
// configuration while no plan has arrived for 0.5 s.
//
// Parameters: model ("" = the config's), stats_file (written on shutdown, "" = none),
// footprint_mode (dynamic, the default, or wholebody).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <numeric>
#include <set>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mobile_manipulator_interfaces/msg/whole_body_plan.hpp"

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

// The ground points of the base links only (the arm links and the panel excluded), whatever the joints.
class BaseLinksModel : public mmg::FootprintModel
{
public:
  BaseLinksModel(const std::shared_ptr<const mmg::FootprintProjector> & projector, const std::set<std::string> & links)
  : points_(projector->projected_points({}, {16, links, false})) {}
  std::vector<mmg::Point2> points(const mmg::JointMap &) const override { return points_; }

private:
  std::vector<mmg::Point2> points_;
};

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
    const auto mode = declare_parameter("footprint_mode", std::string("dynamic"));
    if (mode != "dynamic" && mode != "wholebody") {
      throw std::invalid_argument("footprint_mode must be dynamic or wholebody, not '" + mode + "'");
    }
    wholebody_ = mode == "wholebody";

    const auto payload = mmn::Json::parse(read_file(share("mobile_manipulator_control") + "/config/qualified_payload.json"));
    const auto projector = std::make_shared<const mmg::FootprintProjector>(
      read_file(share("mobile_manipulator_description") + "/urdf/mobile_manipulator.urdf"), mmg::payload_from_json(payload));
    zones_ = mmn::monitor_zones(config.load("nav2_navigation.yaml"));
    std::vector<double> margins;
    // footprint_mode wholebody (B5): the zones' wholebody margins (the StopZone is the base hull + 0.02 m).
    for (const auto & zone : zones_) margins.push_back(wholebody_ ? zone.wholebody_margin_m : zone.margin_m);
    inflation_radius_ = config.load("nav2_global_planning.yaml").at("global_costmap").at("global_costmap")
                          .at("ros__parameters").at("inflation_layer").at("inflation_radius").get<double>();
    nav2_padding_ = config.load("nav2_local_costmap.yaml").at("local_costmap").at("local_costmap")
                      .at("ros__parameters").at("footprint_padding").get<double>();
    footprint_ = std::make_unique<mmn::DynamicFootprint>(
      mmn::footprint_model(model_name_, parameters, projector), payload.at("joint_order").get<std::vector<std::string>>(),
      margins, parameters.at("padding_m").get<double>(), parameters.at("change_threshold_m").get<double>(),
      parameters.at("stale_after_s").get<double>(), inflation_radius_, nav2_padding_);

    if (wholebody_) {
      size_t slowdown = zones_.size();
      for (size_t i = 0; i < zones_.size(); ++i) {
        if (zones_[i].name == "SlowdownZone") slowdown = i;
      }
      if (slowdown == zones_.size()) throw std::runtime_error("nav2_navigation.yaml has no SlowdownZone");
      std::set<std::string> base_links;
      for (const auto & link : parameters.at("disc").at("base_links")) base_links.insert(link.get<std::string>());
      footprint_->use_wholebody(std::make_shared<BaseLinksModel>(projector, base_links), slowdown);
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
      plan_sub_ = create_subscription<mobile_manipulator_interfaces::msg::WholeBodyPlan>(
        "wholebody_mpc/plan", 5, [this](const mobile_manipulator_interfaces::msg::WholeBodyPlan & m) { on_plan(m); });
    }

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
        const double speed = std::hypot(m.twist.twist.linear.x, m.twist.twist.linear.y);
        still_.add(steady_seconds(), speed, m.twist.twist.angular.z);
        const auto & q = m.pose.pose.orientation;
        pose_ = {m.pose.pose.position.x, m.pose.pose.position.y,
                 std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
      });
    // "Base moving" is the base command, not /odom: arm motion alone rocks the simulated base past
    // the odometry thresholds, which would switch the rule to grow-only during reconfigurations.
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 20, [this](const geometry_msgs::msg::Twist & m) {
        footprint_->base_motion(steady_seconds(), std::hypot(m.linear.x, m.linear.y), m.angular.z);
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
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] {
      // A footprint that cannot be computed keeps the last one; the node must not die.
      try {
        tick();
      } catch (const std::exception & error) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Footprint update failed (keeping the last one): %s",
                              error.what());
      }
    });
    RCLCPP_INFO(get_logger(), "%s footprint (%s model) at %.0f Hz", wholebody_ ? "Whole-body" : "Dynamic",
                model_name_.c_str(), rate);
  }

  void write_stats() const
  {
    if (stats_file_.empty()) return;
    auto times = compute_us_;
    std::sort(times.begin(), times.end());
    mmn::Json stats{{"model", model_name_}, {"ticks", times.size()}, {"publishes", publish_times_.size()},
                    {"publish_times_s", publish_times_}, {"publish_reasons", publish_reasons_}, {"points", points_},
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
    const auto update = wholebody_ ? footprint_->tick_wholebody(now) : footprint_->tick(now);
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

  // The latest plan's configurations, placed relative to the current base pose.
  void on_plan(const mobile_manipulator_interfaces::msg::WholeBodyPlan & plan)
  {
    const size_t joints = plan.joint_names.size();
    const size_t stages = plan.base.size();
    if (joints == 0 || stages == 0 || plan.positions.size() != stages * joints) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring a malformed wholebody_mpc/plan");
      return;
    }
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No map -> base_footprint for the plan: %s", error.what());
      return;
    }
    const auto & q = transform.transform.rotation;
    const double x = transform.transform.translation.x, y = transform.transform.translation.y;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double c = std::cos(yaw), s = std::sin(yaw);
    std::vector<std::pair<mmn::Pose2, mmg::JointMap>> predicted;
    for (size_t k = 0; k < stages; ++k) {
      const double dx = plan.base[k].x - x, dy = plan.base[k].y - y;
      const double dyaw = std::remainder(plan.base[k].theta - yaw, 2.0 * M_PI);
      mmg::JointMap map = footprint_->positions();  // joints the plan does not name stay as measured
      for (size_t j = 0; j < joints; ++j) map[plan.joint_names[j]] = plan.positions[k * joints + j];
      predicted.push_back({{c * dx + s * dy, -s * dx + c * dy, dyaw}, std::move(map)});
    }
    footprint_->plan(steady_seconds(), std::move(predicted));
  }

  void publish(const mmn::FootprintUpdate & update)
  {
    const auto polygon = polygon_msg(update.footprint);
    if (update.footprint_changed) {
      for (const auto & pub : footprint_pubs_) pub->publish(polygon);
    }
    for (size_t i = 0; i < zone_pubs_.size(); ++i) {
      if (i < update.zone_changed.size() && !update.zone_changed[i]) continue;
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
    publish_reasons_.emplace_back(mmn::to_string(update.reason));
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
  bool wholebody_ = false;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<mobile_manipulator_interfaces::msg::WholeBodyPlan>::SharedPtr plan_sub_;
  std::unique_ptr<mmn::DynamicFootprint> footprint_;
  std::vector<mmn::MonitorZone> zones_;
  double inflation_radius_ = 1.0, nav2_padding_ = 0.01;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr> footprint_pubs_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr> zone_pubs_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr> published_subs_;
  rclcpp::TimerBase::SharedPtr timer_;
  builtin_interfaces::msg::Time joint_stamp_;
  mmn::Stillness still_{0.01, 0.02};
  mmn::Pose2 pose_{0.0, 0.0, 0.0};
  std::array<mmn::PublishedPolygon, 2> published_{};
  std::array<mmn::FootprintDriftGuard, 2> drift_;
  double last_publish_ = 0.0;
  std::vector<double> compute_us_, publish_times_;
  std::vector<std::string> publish_reasons_;
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
