// Aggregate navigation state into one small message for the Unity telemetry window.
//
// Publishes /mm/telemetry (std_msgs/String, JSON) at 5 Hz and /mm/telemetry/path
// (nav_msgs/Path, transient local) downsampled to 0.1 m only when the global plan changes.
// Everything Unity already knows (base motion, arm state, contacts, clock) stays in Unity;
// this node only forwards ROS-side navigation state, keeping the Unity link to about
// 2-3 KB/s. Set the `scenario` parameter to label runs.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <action_msgs/msg/goal_status_array.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/msg/collision_monitor_state.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mobile_manipulator_navigation/telemetry.hpp"

namespace mmn = mobile_manipulator_navigation;
using json = nlohmann::ordered_json;
using namespace std::chrono_literals;
using Feedback = nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage;

namespace
{
template <typename Stamp>
double stamp_seconds(const Stamp & stamp)
{
  return stamp.sec + stamp.nanosec * 1e-9;
}

std::string format(const char * pattern, double value)
{
  char text[64];
  std::snprintf(text, sizeof(text), pattern, value);
  return text;
}

struct Goal
{
  std::string status;
  int recoveries = 0;
  // Unity's JsonUtility has no null: unavailable numbers are sent as -1.
  double distance_remaining_m = -1, eta_s = -1, elapsed_s = 0;
  std::array<double, 2> pose = {0.0, 0.0};
};
}  // namespace

class NavTelemetry : public rclcpp::Node
{
public:
  NavTelemetry()
  : Node("nav_telemetry", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}})),
    scenario_(declare_parameter("scenario", std::string())),
    buffer_(get_clock()), listener_(buffer_), lidar_(0.5)
  {
    for (const auto & [name, fallback] : std::vector<std::pair<std::string, std::string>>{
           {"footprint_profile", ""}, {"behavior_tree", ""}, {"planner", "Lattice"}, {"controller", "RPP"}}) {
      label_names_.push_back(name);
      labels_[name] = declare_parameter(name, fallback);
    }
    parameters_ = add_post_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & changed) {
      for (const auto & parameter : changed) {
        if (parameter.get_name() == "scenario") {
          scenario_ = parameter.as_string();
          events_.add(now_s(), "scenario " + scenario_);
        } else if (labels_.count(parameter.get_name())) {
          labels_[parameter.get_name()] = parameter.as_string();
        }
      }
    });

    telemetry_pub_ = create_publisher<std_msgs::msg::String>("/mm/telemetry", 1);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/mm/telemetry/path", rclcpp::QoS(1).transient_local().reliable());
    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/plan", 10, [this](nav_msgs::msg::Path::ConstSharedPtr m) { on_plan(*m); });
    status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status", 10,
      [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr m) { on_status(*m); });
    feedback_sub_ = create_subscription<Feedback>(
      "/navigate_to_pose/_action/feedback", 10, [this](Feedback::ConstSharedPtr m) { on_feedback(*m); });
    monitor_sub_ = create_subscription<nav2_msgs::msg::CollisionMonitorState>(
      "/collision_monitor_state", 10,
      [this](nav2_msgs::msg::CollisionMonitorState::ConstSharedPtr m) { on_monitor(*m); });
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/points_filtered", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { on_lidar(*m); });
    publish_timer_ = rclcpp::create_timer(this, get_clock(), 200ms, [this] { publish(); });
    // The ROS-TCP endpoint subscribes with volatile QoS, so a latched path is not
    // replayed to a reconnecting Unity; resend the (small) current path every 5 s.
    path_timer_ = rclcpp::create_timer(this, get_clock(), 5s, [this] {
      if (last_path_) path_pub_->publish(*last_path_);
    });
  }

private:
  double now_s() { return get_clock()->now().seconds(); }

  void on_plan(const nav_msgs::msg::Path & message)
  {
    std::vector<mmn::PathPoint> points;
    points.reserve(message.poses.size());
    for (const auto & pose : message.poses) {
      const auto & p = pose.pose;
      points.push_back({p.position.x, p.position.y, 2 * std::atan2(p.orientation.z, p.orientation.w)});
    }
    if (points == path_) return;
    path_ = std::move(points);
    path_stamp_ = now_s();
    events_.add(now_s(), format("new plan %.1f m", mmn::path_length(path_)));
    nav_msgs::msg::Path out;
    out.header = message.header;
    for (const size_t index : mmn::downsample(path_, 0.1)) out.poses.push_back(message.poses[index]);
    last_path_ = out;
    path_pub_->publish(out);
  }

  void on_status(const action_msgs::msg::GoalStatusArray & message)
  {
    if (message.status_list.empty()) return;
    const auto & latest = message.status_list.back();
    const auto & goal_id = latest.goal_info.goal_id.uuid;
    const std::string status = mmn::goal_status_name(latest.status);
    if (!goal_id_ || *goal_id_ != goal_id) {
      goal_id_ = goal_id;
      ++goal_seq_;
      cross_track_max_ = 0.0;
      goal_ = Goal{status};
      events_.add(now_s(), "goal received");
    }
    if (status != goal_.status) events_.add(now_s(), "goal " + status);
    goal_.status = status;
  }

  void on_feedback(const Feedback & message)
  {
    const auto & feedback = message.feedback;
    const int recoveries = feedback.number_of_recoveries;
    if (recoveries > goal_.recoveries) events_.add(now_s(), "recovery " + std::to_string(recoveries));
    const auto & position = feedback.current_pose.pose.position;
    goal_.recoveries = recoveries;
    goal_.distance_remaining_m = mmn::round_to(feedback.distance_remaining, 2);
    goal_.eta_s = mmn::round_to(stamp_seconds(feedback.estimated_time_remaining), 1);
    goal_.elapsed_s = mmn::round_to(stamp_seconds(feedback.navigation_time), 1);
    goal_.pose = {mmn::round_to(position.x, 2), mmn::round_to(position.y, 2)};
  }

  void on_monitor(const nav2_msgs::msg::CollisionMonitorState & message)
  {
    const std::pair<std::string, std::string> state{
      mmn::monitor_action_name(message.action_type), message.polygon_name};
    if (state != monitor_ && state.first != "none") {
      events_.add(now_s(), "monitor " + state.first + " (" + state.second + ")");
    }
    monitor_ = state;
  }

  void on_lidar(const sensor_msgs::msg::PointCloud2 & message)
  {
    if (const auto gap = lidar_.add(stamp_seconds(message.header.stamp))) {
      events_.add(now_s(), format("lidar gap %.2f s", *gap));
    }
  }

  void publish()
  {
    std::optional<double> deviation;
    try {
      const auto t = buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero).transform;
      if (!path_.empty() && goal_.status == "executing") {
        deviation = mmn::cross_track(t.translation.x, t.translation.y, path_);
        cross_track_max_ = std::max(cross_track_max_, *deviation);
      }
    } catch (const tf2::TransformException &) {
    }
    const double now = now_s();
    json message = {{"t", mmn::round_to(now, 2)}, {"scenario", scenario_}};
    for (const auto & name : label_names_) message[name] = labels_[name];
    message["goal_seq"] = goal_seq_;
    message["goal"] = {{"status", goal_.status}, {"recoveries", goal_.recoveries},
                       {"distance_remaining_m", goal_.distance_remaining_m}, {"eta_s", goal_.eta_s},
                       {"elapsed_s", goal_.elapsed_s}, {"pose", goal_.pose}};
    message["path"] = {
      {"length_m", mmn::round_to(mmn::path_length(path_), 2)}, {"points", path_.size()},
      {"age_s", path_stamp_ ? mmn::round_to(now - *path_stamp_, 1) : -1.0},
      {"cross_track_m", deviation ? mmn::round_to(*deviation, 3) : -1.0},
      {"cross_track_max_m", mmn::round_to(cross_track_max_, 3)}};
    message["monitor"] = {{"action", monitor_.first}, {"polygon", monitor_.second}};
    message["lidar"] = {{"rate_hz", mmn::round_to(lidar_.rate(), 1)}, {"gaps_over_0p5s", lidar_.long_gaps()},
                        {"max_gap_s", mmn::round_to(lidar_.max_gap(), 2)}};
    json events = json::array();
    for (const auto & event : events_.events()) events.push_back({{"t", event.t}, {"text", event.text}});
    message["events"] = std::move(events);
    std_msgs::msg::String out;
    out.data = message.dump(-1, ' ', false, json::error_handler_t::replace);
    telemetry_pub_->publish(out);
  }

  std::string scenario_;
  std::vector<std::string> label_names_;
  std::map<std::string, std::string> labels_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  mmn::EventLog events_;
  mmn::GapCounter lidar_;
  std::vector<mmn::PathPoint> path_;
  std::optional<double> path_stamp_;
  std::optional<nav_msgs::msg::Path> last_path_;
  double cross_track_max_ = 0.0;
  Goal goal_{"idle"};
  std::optional<std::array<uint8_t, 16>> goal_id_;
  int goal_seq_ = 0;
  std::pair<std::string, std::string> monitor_{"none", ""};

  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr parameters_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::Subscription<Feedback>::SharedPtr feedback_sub_;
  rclcpp::Subscription<nav2_msgs::msg::CollisionMonitorState>::SharedPtr monitor_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_, path_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavTelemetry>());
  rclcpp::shutdown();
  return 0;
}
