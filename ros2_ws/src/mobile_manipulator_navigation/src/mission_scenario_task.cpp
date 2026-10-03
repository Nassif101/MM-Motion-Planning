// Mission scenario task (B3): ordered NavigateToPose drives and ReconfigurePanel steps.
//
// Used by tools/run_nav_scenario.py with navigation.launch.py (footprint_profile = the
// scenario's starting profile) and manipulation.launch.py active. Runs the navigate task's
// preflight once, then each step in order: a drive is measured exactly as
// navigate_scenario_task measures it; a reconfiguration records the ReconfigurePanel result
// and then waits until both costmaps publish the new (padded) footprint and have completed an
// update cycle on it before the next drive (costmap footprints and the collision monitor's
// zones, which navigation.launch.py must take from ReconfigurePanel:
// dynamic_monitor_zones:=true). The mission stops at the first step that does not succeed.
// All timing is simulation time unless named wall_*.
//
// Usage: mission_scenario_task --scenario NAME --output FILE [--start-tolerance M]
// Exit codes: 0 succeeded, 3 a step ran but did not succeed, 1 preflight or setup failure.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "mobile_manipulator_control/cli.hpp"
#include "mobile_manipulator_interfaces/action/reconfigure_panel.hpp"
#include "mobile_manipulator_navigation/mission.hpp"
#include "mobile_manipulator_navigation/navigate_run.hpp"

namespace mmn = mobile_manipulator_navigation;
using mmn::Json;
using Reconfigure = mobile_manipulator_interfaces::action::ReconfigurePanel;
using Clock = std::chrono::steady_clock;

namespace
{
const std::vector<std::string> kCostmapTopics = {"/local_costmap/published_footprint",
                                                 "/global_costmap/published_footprint"};
// Rest required before a reconfiguration: the server's 0.5 s window plus a margin.
constexpr double kSettleSeconds = 0.75;
// Costmap footprint_padding in nav2_global_planning.yaml and nav2_local_costmap.yaml.
constexpr double kFootprintPadding = 0.01;
const std::vector<std::string> kMissionProcesses = [] {
  auto names = mmn::kNavProcesses;
  names.insert(names.end(), {"move_group", "reconfigure_panel_server"});
  return names;
}();

int fail(const std::string & message)
{
  std::cerr << message << std::endl;
  rclcpp::shutdown();
  return 1;
}

const char * code_name(uint8_t code)
{
  static const std::vector<const char *> names = {
    "SUCCESS", "BASE_NOT_STOPPED", "ARM_NOT_ACTIVE", "UNKNOWN_PROFILE", "NO_IK", "PLANNING_FAILED",
    "PROFILE_TOO_SMALL", "EXECUTION_FAILED", "ARM_FAULT", "PROFILE_VIOLATED_AFTER_EXECUTION", "CANCELED"};
  return code < names.size() ? names[code] : "UNKNOWN";
}

geometry_msgs::msg::Vector3 vector3(const Json & values)
{
  geometry_msgs::msg::Vector3 v;
  v.x = values.at(0).get<double>();
  v.y = values.at(1).get<double>();
  v.z = values.at(2).get<double>();
  return v;
}

Reconfigure::Goal reconfigure_goal(const Json & step)
{
  Reconfigure::Goal goal;
  goal.footprint_profile = step.at("footprint_profile").get<std::string>();
  if (step.contains("named_state")) {
    goal.target_type = Reconfigure::Goal::NAMED_STATE;
    goal.named_state = step.at("named_state").get<std::string>();
    return goal;
  }
  const auto & pose = step.at("panel_pose");
  goal.target_type = Reconfigure::Goal::PANEL_POSE;
  goal.panel_pose.header.frame_id = "base_footprint";
  goal.panel_pose.pose.position.x = pose.at("xyz").at(0).get<double>();
  goal.panel_pose.pose.position.y = pose.at("xyz").at(1).get<double>();
  goal.panel_pose.pose.position.z = pose.at("xyz").at(2).get<double>();
  tf2::Quaternion q;
  q.setRPY(pose.at("rpy").at(0).get<double>(), pose.at("rpy").at(1).get<double>(), pose.at("rpy").at(2).get<double>());
  goal.panel_pose.pose.orientation.x = q.x();
  goal.panel_pose.pose.orientation.y = q.y();
  goal.panel_pose.pose.orientation.z = q.z();
  goal.panel_pose.pose.orientation.w = q.w();
  goal.position_tolerance = vector3(step.at("position_tolerance"));
  goal.orientation_tolerance = vector3(step.at("orientation_tolerance"));
  return goal;
}

Json result_json(const Reconfigure::Result & r)
{
  const auto number = [](double v) { return std::isfinite(v) ? Json(mmn::round_digits(v, 4)) : Json(nullptr); };
  return {{"error_code", code_name(r.error_code)}, {"message", r.message},
          {"applied_footprint_profile", r.applied_footprint_profile}, {"profile_violated", r.profile_violated},
          {"reached_joint_positions", r.reached_joint_positions},
          {"planning_requests", r.planning_requests},
          {"reached_panel_pose", {{"xyz", {r.reached_panel_pose.position.x, r.reached_panel_pose.position.y,
                                           r.reached_panel_pose.position.z}},
                                  {"quaternion_xyzw", {r.reached_panel_pose.orientation.x, r.reached_panel_pose.orientation.y,
                                                       r.reached_panel_pose.orientation.z, r.reached_panel_pose.orientation.w}}}},
          {"planned_containment_margin_m", number(r.planned_containment_margin_m)},
          {"measured_containment_margin_m", number(r.measured_containment_margin_m)},
          {"planning_time_s", number(r.planning_time_s)}, {"execution_time_s", number(r.execution_time_s)},
          {"footprint_switch_time_s", number(r.footprint_switch_time_s)},
          {"trajectory_duration_s", number(r.trajectory_duration_s)},
          {"joint_path_length_rad", number(r.joint_path_length_rad)},
          {"min_planned_clearance_m", number(r.min_planned_clearance_m)},
          {"max_path_error_rad", number(r.max_path_error_rad)}, {"hold_error_rad", number(r.hold_error_rad)}};
}

// Latest footprint each costmap publishes and the stop/slowdown zones on the latched topics
// the collision monitor takes them from (it only publishes its own zones while velocity
// commands flow, so they cannot be read back at standstill): all of them must reflect the
// active footprint profile before the next drive.
class FootprintWatch : public rclcpp::Node
{
public:
  explicit FootprintWatch(std::vector<mmn::MonitorZone> zones)
  : Node("mission_footprint_watch", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}})),
    zones_(std::move(zones))
  {
    std::vector<std::pair<std::string, rclcpp::QoS>> topics;
    for (const auto & topic : kCostmapTopics) topics.push_back({topic, rclcpp::QoS(10)});
    for (const auto & zone : zones_) topics.push_back({zone.polygon_topic, rclcpp::QoS(1).reliable().transient_local()});
    for (const auto & [topic, qos] : topics) {
      subs_.push_back(create_subscription<geometry_msgs::msg::PolygonStamped>(
        topic, qos, [this, topic = topic](const geometry_msgs::msg::PolygonStamped & m) {
          mmn::Polygon polygon;
          for (const auto & p : m.polygon.points) polygon.push_back({p.x, p.y});
          std::lock_guard<std::mutex> lock(mutex_);
          latest_[topic] = {polygon, m.header.frame_id, wall_seconds()};
        }));
    }
  }

  // Whether the costmaps publish `profile` (messages received at or after `since`, wall
  // seconds) and every zone input is `profile` grown by its margin (mmn::footprints_applied).
  bool all_match(const mmn::Polygon & profile, const std::optional<mmn::Pose2> & robot, double since = 0.0)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return robot && mmn::footprints_applied(latest_, kCostmapTopics, zones_, profile, *robot, since,
                                            kFootprintPadding);
  }

  static double wall_seconds()
  {
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
  }

private:
  std::vector<mmn::MonitorZone> zones_;
  std::mutex mutex_;
  std::map<std::string, mmn::PublishedPolygon> latest_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr> subs_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const mobile_manipulator_control::Args args(argc, argv);
  const std::string name = args.get("scenario");
  const std::filesystem::path output = args.get("output");
  const double start_tolerance = args.number("start-tolerance", 0.10);

  const auto share = ament_index_cpp::get_package_share_directory("mobile_manipulator_navigation");
  const mmn::ScenarioConfig config(share);
  Json scenario;
  try {
    scenario = config.resolve(name).at("scenario");
  } catch (const std::exception & error) {
    return fail(error.what());
  }
  if (scenario.at("task") != "mission") return fail(name + " is not a mission scenario");
  const auto steps = mmn::parse_mission(scenario);
  const Json profiles = config.load("footprint_profiles.yaml").at("profiles");
  const double timeout = scenario.at("timeout_s").get<double>();
  const Json obstacles = scenario.value("obstacles", Json::array());
  const Json movers = scenario.value("movers", Json::array());

  auto node = std::make_shared<mmn::Recorder>("mission_scenario_task");
  auto watch = std::make_shared<FootprintWatch>(mmn::monitor_zones(config.load("nav2_navigation.yaml")));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(watch);
  auto navigate = rclcpp_action::create_client<mmn::NavigateToPose>(node, "navigate_to_pose");
  auto reconfigure = rclcpp_action::create_client<Reconfigure>(node, "/reconfigure_panel");

  // ---- Preflight (no step runs unless every check passes) ----
  auto problems = mmn::navigation_preflight(*node, executor, navigate, mmn::pose_of(scenario.at("start")),
                                            start_tolerance);
  if (!reconfigure->wait_for_action_server(std::chrono::seconds(30))) {
    problems.push_back("reconfigure_panel is not available");
  }
  std::string profile = scenario.at("footprint_profile").get<std::string>();
  const auto wall_watch = Clock::now();
  while (problems.empty() && !watch->all_match(mmn::polygon_of(profiles.at(profile).at("polygon")), node->pose())) {
    executor.spin_once(std::chrono::milliseconds(100));
    if (Clock::now() - wall_watch > std::chrono::seconds(10)) {
      problems.push_back("costmaps and collision monitor zones do not show the starting profile " + profile +
                         " (launch navigation with dynamic_monitor_zones:=true)");
    }
  }
  if (!problems.empty()) {
    mmn::write_report(output, {{"preflight_failed", problems}});
    std::string text = "Preflight failed: ";
    for (size_t i = 0; i < problems.size(); ++i) text += (i ? "; " : "") + problems[i];
    return fail(text);
  }

  const double mission_start = node->now_s();
  const auto wall_start = Clock::now();
  Json reports = Json::array();
  std::string status = "succeeded";
  Json failed_step = nullptr;
  double drive_time = 0.0, reconfigure_time = 0.0;
  for (size_t i = 0; i < steps.size() && status == "succeeded"; ++i) {
    const auto & step = steps[i];
    const double step_start = node->now_s();
    Json report;
    if (step.kind == mmn::MissionStep::Kind::Navigate) {
      const auto here = node->pose();
      const mmn::Pose2 from = here ? *here : mmn::Pose2{step.pose};
      try {
        report = mmn::run_navigate(*node, executor, navigate, from, step.pose,
                                   profiles.at(profile).at("polygon"), timeout, obstacles, movers, kMissionProcesses);
      } catch (const std::exception & error) {
        report = {{"status", "rejected"}, {"error", error.what()}};
      }
      report["type"] = "navigate";
      report["goal"] = Json::array({step.pose[0], step.pose[1], step.pose[2]});
      report["footprint_profile"] = profile;
      drive_time += node->now_s() - step_start;
      if (report.at("status") != "succeeded") status = "failed";
    } else {
      // Nav2 reports success while the base is still settling; ReconfigurePanel refuses to
      // move the arm until the base has been at rest for 0.5 s (same speed limits).
      mmn::Stillness still(0.01, 0.02);
      double last_odom = -1.0;
      const auto settle_deadline = Clock::now() + std::chrono::seconds(15);
      while (still.still_for(node->now_s()) < kSettleSeconds && Clock::now() < settle_deadline) {
        executor.spin_once(std::chrono::milliseconds(20));
        if (node->odom && mmn::stamp_seconds(node->odom->header.stamp) != last_odom) {
          last_odom = mmn::stamp_seconds(node->odom->header.stamp);
          const auto & twist = node->odom->twist.twist;
          still.add(last_odom, std::hypot(twist.linear.x, twist.linear.y), twist.angular.z);
        }
      }
      const double settle_s = node->now_s() - step_start;
      // Spec 5: move_group CPU and memory per reconfiguration (drives measure them too).
      const auto usage_before = mmn::proc_cpu_mem(kMissionProcesses);
      const auto wall_before = Clock::now();
      auto sent = reconfigure->async_send_goal(reconfigure_goal(step.reconfigure));
      Json outcome;
      if (executor.spin_until_future_complete(sent, std::chrono::seconds(10)) != rclcpp::FutureReturnCode::SUCCESS ||
          !sent.get()) {
        outcome = {{"error_code", "NOT_ACCEPTED"}};
      } else {
        auto result = reconfigure->async_get_result(sent.get());
        if (executor.spin_until_future_complete(result, std::chrono::seconds(120)) != rclcpp::FutureReturnCode::SUCCESS) {
          reconfigure->async_cancel_goal(sent.get());
          outcome = {{"error_code", "NO_RESULT"}};
        } else {
          outcome = result_json(*result.get().result);
        }
      }
      const auto usage = mmn::usage_report(usage_before, mmn::proc_cpu_mem(kMissionProcesses),
                                           std::chrono::duration<double>(Clock::now() - wall_before).count());
      report = {{"type", "reconfigure"}, {"request", step.reconfigure}, {"result", outcome},
                {"settle_s", mmn::round_digits(settle_s, 2)},
                {"cpu_percent_of_core", usage.at("cpu_percent_of_core")}, {"max_rss_mb", usage.at("max_rss_mb")}};
      if (outcome.at("error_code") != "SUCCESS") {
        status = "failed";
      } else {
        profile = step.reconfigure.at("footprint_profile").get<std::string>();
        // Review Focus 4: the next drive must use the new footprint in both costmaps, and the
        // costmaps must have completed an update cycle on it (mmn::FootprintRefresh).
        const auto expected = mmn::polygon_of(profiles.at(profile).at("polygon"));
        mmn::FootprintRefresh refresh(FootprintWatch::wall_seconds());  // costmap messages must be newer
        const auto deadline = Clock::now() + std::chrono::seconds(5);
        const double switch_start = node->now_s();
        Json shown_after = nullptr;  // until the costmaps first show it
        while (Clock::now() < deadline) {
          const bool was_shown = refresh.shown();
          if (refresh.observe(watch->all_match(expected, node->pose(), refresh.since()), FootprintWatch::wall_seconds())) {
            break;
          }
          if (refresh.shown() && !was_shown) shown_after = mmn::round_digits(node->now_s() - switch_start, 3);
          executor.spin_once(std::chrono::milliseconds(20));
        }
        report["costmaps_applied_profile"] = refresh.refreshed();
        report["costmap_update_s"] = shown_after;
        report["costmap_refresh_s"] = mmn::round_digits(node->now_s() - switch_start, 3);
        if (!refresh.refreshed()) status = "failed";
      }
      reconfigure_time += node->now_s() - step_start;
    }
    report["step"] = static_cast<int>(i + 1);
    report["time_s"] = mmn::round_digits(node->now_s() - step_start, 2);
    if (status != "succeeded") failed_step = static_cast<int>(i + 1);
    reports.push_back(report);
    Json brief = {{"step", i + 1}, {"type", report["type"]}};
    if (report.contains("status")) brief["status"] = report["status"];
    if (report.contains("result")) brief["result"] = report["result"].value("error_code", "");
    std::cerr << brief.dump() << std::endl;
  }

  Json mission = {{"scenario", name},
                  {"status", status},
                  {"failed_step", failed_step},
                  {"total_time_s", mmn::round_digits(node->now_s() - mission_start, 2)},
                  {"wall_s", mmn::round_digits(std::chrono::duration<double>(Clock::now() - wall_start).count(), 2)},
                  {"drive_time_s", mmn::round_digits(drive_time, 2)},
                  {"reconfigure_time_s", mmn::round_digits(reconfigure_time, 2)},
                  {"final_footprint_profile", profile},
                  {"steps", reports}};
  mmn::write_report(output, mission);
  Json summary = mission;
  summary.erase("steps");
  std::cout << summary.dump() << std::endl;
  rclcpp::shutdown();
  return status == "succeeded" ? 0 : 3;
}
