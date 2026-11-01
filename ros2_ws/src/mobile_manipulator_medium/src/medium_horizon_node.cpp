// medium_horizon_node (Phase 5A spec Section 2, plan Task 11): the medium layer above B5. A single-threaded timer at
// rate_hz plans on B5's announced route (wholebody_mpc/route, latched) from the measured state while B5 tracks a goal
// (its wholebody_mpc/cycle messages arrive), and publishes the plan (/medium/plan, B5's references in
// reference_source: medium), the cycle's statistics (/medium/cycle) and RViz markers (/medium/markers: the planned
// base path and the panel along it). A full multi-start runs on a new goal (MediumPlanner), on a request when a
// draggable box moves more than kDraggableMoveM or kDraggableTurnRad, and as configured (seeds_per_cycle 0: periodic).
// The solve cap keeps a cycle below the period.
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mobile_manipulator_geometry/footprint_projection.hpp"
#include "mobile_manipulator_geometry/robot_model.hpp"
#include "mobile_manipulator_interfaces/msg/medium_cycle.hpp"
#include "mobile_manipulator_interfaces/msg/medium_plan.hpp"
#include "mobile_manipulator_interfaces/msg/mpc_cycle.hpp"
#include "mobile_manipulator_interfaces/msg/whole_body_route.hpp"
#include "mobile_manipulator_medium/planner.hpp"
#include "mobile_manipulator_mpc/ros_state.hpp"
#include "mobile_manipulator_mpc/scene.hpp"
#include "mobile_manipulator_mpc/statics.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace
{
namespace mmd = mobile_manipulator_medium;
namespace mmm = mobile_manipulator_mpc;
namespace mmn = mobile_manipulator_navigation;
namespace mmi = mobile_manipulator_interfaces;
using Steady = std::chrono::steady_clock;

constexpr double kTrackingS = 0.5;         // B5 counts as tracking while its last cycle message is this recent
constexpr double kDraggableMoveM = 0.05;   // a draggable moved this far (or turned kDraggableTurnRad): full multi-start
constexpr double kDraggableTurnRad = 0.05;
constexpr double kPanelMarkerS = 1.0;      // a panel marker every this along the plan (s of the horizon)

std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot read " + path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

std::string share(const std::string & package) { return ament_index_cpp::get_package_share_directory(package); }
double seconds(Steady::time_point t) { return std::chrono::duration<double>(t.time_since_epoch()).count(); }

builtin_interfaces::msg::Time stamp_of(double t)
{
  builtin_interfaces::msg::Time out;
  out.sec = static_cast<int32_t>(std::floor(t));
  out.nanosec = static_cast<uint32_t>(std::round((t - std::floor(t)) * 1e9));
  if (out.nanosec >= 1000000000u) {
    ++out.sec;
    out.nanosec -= 1000000000u;
  }
  return out;
}

class MediumHorizonNode : public rclcpp::Node
{
public:
  MediumHorizonNode() : Node("medium_horizon_node")
  {
    const std::string navigation = share("mobile_manipulator_navigation");
    const std::string config_file =
      declare_parameter("config_file", share("mobile_manipulator_medium") + "/config/medium.yaml");
    const std::string scenario = declare_parameter("scenario", std::string(""));
    const std::string site = declare_parameter("site", std::string(""));
    const std::string boxes_file = declare_parameter("boxes_file", std::string(""));
    const std::string draggables_topic = declare_parameter("draggables_topic", std::string(""));
    declare_parameter("movers_topic", std::string(""));  // spec 3.6: movers stay B5's in 5A (declared, unused)
    const std::string envelope_file =
      declare_parameter("envelope_file", navigation + "/config/nav_operating_envelope.yaml");
    const std::string comfort_pose = declare_parameter("comfort_pose", std::string("carry"));

    mmd::MediumConfig config = mmd::load_medium_config(config_file);
    rate_hz_ = declare_parameter("rate_hz", config.rate_hz);
    if (!(rate_hz_ > 0.0)) throw std::invalid_argument("rate_hz must be positive");

    const std::string urdf = read_file(share("mobile_manipulator_description") + "/urdf/mobile_manipulator.urdf");
    const mmn::Json payload =
      mmn::Json::parse(read_file(share("mobile_manipulator_control") + "/config/qualified_payload.json"));
    const auto primitives = mobile_manipulator_geometry::load_primitives(
      urdf, mobile_manipulator_geometry::payload_from_json(payload));
    panel_half_ = primitives.back().dims;
    std::map<std::string, mmd::Joints> poses;
    for (const auto & item : payload.at("poses_rad").items()) {
      mmd::Joints q;
      for (int i = 0; i < 6; ++i) q(i) = item.value().at(i).get<double>();
      poses[item.key()] = q;
    }
    if (!poses.count(comfort_pose)) throw std::invalid_argument("comfort_pose " + comfort_pose + " is not qualified");
    mmd::MediumProblem problem(config, mmm::load_limits(envelope_file, urdf), primitives, mmm::CollisionConfig{},
                               std::make_shared<const mmm::GeneratedStaticTorque>(), poses.at(comfort_pose));
    planner_.emplace(std::move(problem),
                     mmd::make_dense_solver(config.qp_backend, mmd::DenseQpOptions{config.qp_tolerance}), poses,
                     panel_half_);

    const mmm::Scene scene = mmm::load_scene(navigation, site, scenario, boxes_file);
    world_.set_static(scene.boxes);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 50, [this](const sensor_msgs::msg::JointState & m) { state_.on_joints(m, seconds(Steady::now())); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 20, [this](const nav_msgs::msg::Odometry & m) { state_.on_odom(m, seconds(Steady::now())); });
    arm_sub_ = create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "/arm_controller/controller_state", 10,
      [this](const control_msgs::msg::JointTrajectoryControllerState &) { state_.on_arm_state(seconds(Steady::now())); });
    route_sub_ = create_subscription<mmi::msg::WholeBodyRoute>(
      "/wholebody_mpc/route", rclcpp::QoS(1).transient_local(),
      [this](const mmi::msg::WholeBodyRoute & m) { on_route(m); });
    b5_cycle_sub_ = create_subscription<mmi::msg::MpcCycle>(
      "/wholebody_mpc/cycle", 10, [this](const mmi::msg::MpcCycle &) { b5_cycle_at_ = seconds(Steady::now()); });
    if (!draggables_topic.empty()) {
      draggables_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        draggables_topic, 10, [this](const visualization_msgs::msg::MarkerArray & m) { on_draggables(m); });
      RCLCPP_INFO(get_logger(), "Draggable obstacles from %s (quasi-static)", draggables_topic.c_str());
    }
    plan_pub_ = create_publisher<mmi::msg::MediumPlan>("/medium/plan", 10);
    cycle_pub_ = create_publisher<mmi::msg::MediumCycle>("/medium/cycle", 10);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/medium/markers", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_), [this] { cycle(); });
    RCLCPP_INFO(get_logger(), "medium layer at %.1f Hz: %zu joints, %d segments, %s, %zu static boxes", rate_hz_,
                config.arm_joints.size(), config.bezier.segments, config.qp_backend.c_str(), scene.boxes.size());
  }

private:
  // A route announcement: a new one by (B5's acceptance stamp, route_id) (route_id restarts with B5).
  void on_route(const mmi::msg::WholeBodyRoute & m)
  {
    const std::pair<double, uint64_t> key{rclcpp::Time(m.header.stamp).seconds(), m.route_id};
    if (route_key_ && *route_key_ == key) return;
    std::vector<Eigen::Vector2d> points;
    for (const auto & p : m.path.poses) points.emplace_back(p.pose.position.x, p.pose.position.y);
    try {
      planner_->set_route(mmm::Route(std::move(points)), {m.goal.x, m.goal.y, m.goal.theta}, m.route_id);
      route_key_ = key;
    } catch (const std::invalid_argument & e) {
      RCLCPP_WARN(get_logger(), "route %lu ignored: %s", static_cast<unsigned long>(m.route_id), e.what());
      return;
    }
    // plan for it now, the next cycle a period later (B5 falls back on another route_id until a plan for this one
    // arrives: at 2.5 Hz waiting for the timer cost up to a period plus the cap, user decision 2026-11-01)
    cycle();
    if (timer_) timer_->reset();
  }

  void on_draggables(const visualization_msgs::msg::MarkerArray & m)
  {
    for (const auto & marker : m.markers) {
      if (marker.action != visualization_msgs::msg::Marker::ADD) continue;
      const auto & p = marker.pose;
      const Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
      const mmm::YawedBox box = mmm::yawed_box(Eigen::Vector3d(p.position.x, p.position.y, p.position.z), q,
                                               Eigen::Vector3d(marker.scale.x, marker.scale.y, marker.scale.z));
      const std::string name = "draggable/" + marker.ns + "/" + std::to_string(marker.id);
      world_.update_box(name, box.center, box.yaw, box.size, rclcpp::Time(marker.header.stamp).seconds());
      // a box seen for the first time is part of the world from now on (its first pose requests nothing: the demo's
      // boxes stream from the start); one that moved beyond the thresholds since it last requested one does
      const Eigen::Vector3d pose(box.center.x(), box.center.y(), box.yaw);
      const auto it = draggable_poses_.find(name);
      if (it == draggable_poses_.end()) {
        draggable_poses_.emplace(name, pose);
      } else if ((it->second.head<2>() - pose.head<2>()).norm() > kDraggableMoveM ||
                 std::abs(std::remainder(it->second(2) - pose(2), 2.0 * M_PI)) > kDraggableTurnRad) {
        it->second = pose;
        planner_->request_full_multistart();
      }
    }
  }

  void cycle()
  {
    const double steady = seconds(Steady::now());
    if (!planner_->has_route() || std::isnan(b5_cycle_at_) || steady - b5_cycle_at_ > kTrackingS) return;
    const double now = get_clock()->now().seconds();
    const mmm::MeasuredState state = state_.measure(*tf_buffer_, now, steady);
    if (!state.complete || !state.fresh) return;
    Eigen::Matrix<double, 9, 1> rate;
    rate << state.v * std::cos(state.x(2)), state.v * std::sin(state.x(2)), state.omega, state.qd;
    const mmd::PlanOutput & out = planner_->plan(now, state.x, rate, world_);
    publish(out);
  }

  void publish(const mmd::PlanOutput & out)
  {
    mmi::msg::MediumPlan plan;
    plan.header.frame_id = "map";
    plan.header.stamp = stamp_of(out.planned_at);
    plan.route_id = out.route_id;
    plan.plan_id = out.plan_id;
    plan.status = out.status;
    plan.seed = out.seed;
    plan.homotopy = out.homotopy;
    plan.cost = out.cost;
    plan.solve_s = out.solve_s;
    plan.splice_s = out.splice_s;
    plan.samples.resize(out.samples.size());
    for (size_t i = 0; i < out.samples.size(); ++i) {
      const auto & s = out.samples[i];
      auto & m = plan.samples[i];
      m.s = s.s;
      m.x = s.x;
      m.y = s.y;
      m.yaw = s.yaw;
      m.speed = s.speed;
      for (int j = 0; j < 6; ++j) m.q[static_cast<size_t>(j)] = s.q(j);
    }
    plan_pub_->publish(plan);

    mmi::msg::MediumCycle cycle;
    cycle.header = plan.header;
    cycle.plan_id = out.plan_id;
    cycle.total_s = out.solve_s;
    cycle.linearize_s = out.linearize_s;
    cycle.qp_s = out.qp_s;
    cycle.full_multistart = out.full_multistart;
    cycle.capped = out.capped;
    for (const auto & s : out.seeds) {
      cycle.seeds.push_back(s.name);
      cycle.seed_costs.push_back(s.merit);
      cycle.seed_slack.push_back(s.slack);
      cycle.seed_solve_s.push_back(s.solve_s);
      cycle.seed_iterations.push_back(s.iterations);
      cycle.seed_converged.push_back(s.converged);
    }
    cycle.homotopy = out.homotopy;
    cycle.homotopy_switched = out.homotopy_switched;
    cycle_pub_->publish(cycle);
    publish_markers(plan, out);
  }

  // The planned base path (a line, coloured by status) and the panel every kPanelMarkerS along it.
  void publish_markers(const mmi::msg::MediumPlan & plan, const mmd::PlanOutput & out)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    visualization_msgs::msg::Marker path;
    path.header = plan.header;
    path.ns = "medium_path";
    path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path.scale.x = 0.03;
    path.color.a = 1.0;
    path.color.g = out.status == "ok" ? 0.8 : 0.3;
    path.color.r = out.status == "ok" ? 0.1 : 0.9;
    path.pose.orientation.w = 1.0;
    for (const auto & s : out.samples) {
      geometry_msgs::msg::Point p;
      p.x = s.x;
      p.y = s.y;
      p.z = 0.05;
      path.points.push_back(p);
    }
    markers.markers.push_back(path);
    const auto * t = planner_->last_trajectory();
    if (t != nullptr) {
      const auto & problem = planner_->problem();
      int id = 0;
      double next = 0.0;
      for (int k = 0; k < problem.samples(); ++k) {
        if (problem.sample_time(k) + 1e-9 < next) continue;
        next += kPanelMarkerS;
        const Eigen::Isometry3d panel = mmm::link_pose(mmm::link_index("panel"), problem.state_at(*t, k));
        visualization_msgs::msg::Marker box;
        box.header = plan.header;
        box.ns = "medium_panel";
        box.id = id++;
        box.type = visualization_msgs::msg::Marker::CUBE;
        box.pose.position.x = panel.translation().x();
        box.pose.position.y = panel.translation().y();
        box.pose.position.z = panel.translation().z();
        const Eigen::Quaterniond q(panel.linear());
        box.pose.orientation.w = q.w();
        box.pose.orientation.x = q.x();
        box.pose.orientation.y = q.y();
        box.pose.orientation.z = q.z();
        box.scale.x = 2.0 * panel_half_.x();
        box.scale.y = 2.0 * panel_half_.y();
        box.scale.z = 2.0 * panel_half_.z();
        box.color.a = 0.35;
        box.color.b = 0.9;
        box.color.g = 0.6;
        markers.markers.push_back(box);
      }
    }
    markers_pub_->publish(markers);
  }

  double rate_hz_ = 5.0;
  std::optional<mmd::MediumPlanner> planner_;
  Eigen::Vector3d panel_half_ = Eigen::Vector3d::Zero();
  mmm::World world_;
  mmm::StateTracker state_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::optional<std::pair<double, uint64_t>> route_key_;
  double b5_cycle_at_ = std::nan("");
  std::map<std::string, Eigen::Vector3d> draggable_poses_;  // the last pose that triggered a full multi-start
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr arm_sub_;
  rclcpp::Subscription<mmi::msg::WholeBodyRoute>::SharedPtr route_sub_;
  rclcpp::Subscription<mmi::msg::MpcCycle>::SharedPtr b5_cycle_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr draggables_sub_;
  rclcpp::Publisher<mmi::msg::MediumPlan>::SharedPtr plan_pub_;
  rclcpp::Publisher<mmi::msg::MediumCycle>::SharedPtr cycle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<MediumHorizonNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("medium_horizon_node"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
