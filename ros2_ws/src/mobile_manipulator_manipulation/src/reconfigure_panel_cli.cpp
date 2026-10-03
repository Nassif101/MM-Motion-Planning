// Send one ReconfigurePanel goal and write the result as JSON.
//
// Usage: reconfigure_panel (--named STATE | --panel-pose X Y Z R P Y
//                           --position-tolerance DX DY DZ --orientation-tolerance RX RY RZ)
//                          --profile NAME [--planning-time S] [--attempts N] [--output FILE]
// The panel pose is the panel centre in base_footprint (RPY as printed by panel_pose).
// Exit codes: 0 SUCCESS, 3 the goal ran but did not succeed, 1 usage or connection failure.
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "mobile_manipulator_interfaces/action/reconfigure_panel.hpp"

using Reconfigure = mobile_manipulator_interfaces::action::ReconfigurePanel;
using namespace std::chrono_literals;

namespace
{
int usage()
{
  std::cerr << "usage: reconfigure_panel (--named STATE | --panel-pose X Y Z R P Y --position-tolerance DX DY DZ "
               "--orientation-tolerance RX RY RZ) --profile NAME [--planning-time S] [--attempts N] [--output FILE]\n";
  return 1;
}

const char * code_name(uint8_t code)
{
  static const std::vector<const char *> names = {
    "SUCCESS", "BASE_NOT_STOPPED", "ARM_NOT_ACTIVE", "UNKNOWN_PROFILE", "NO_IK", "PLANNING_FAILED",
    "PROFILE_TOO_SMALL", "EXECUTION_FAILED", "ARM_FAULT", "PROFILE_VIOLATED_AFTER_EXECUTION", "CANCELED"};
  return code < names.size() ? names[code] : "UNKNOWN";
}
}  // namespace

int main(int argc, char ** argv)
{
  const auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  std::map<std::string, std::vector<double>> numbers;
  std::string named, profile, output;
  for (size_t i = 1; i < args.size(); ++i) {
    const auto & key = args[i];
    const auto take = [&](size_t n) {
      std::vector<double> values;
      for (size_t k = 0; k < n; ++k) {
        if (++i >= args.size()) throw std::invalid_argument(key + " needs " + std::to_string(n) + " values");
        values.push_back(std::stod(args[i]));
      }
      numbers[key] = values;
    };
    try {
      if (key == "--named" && i + 1 < args.size()) named = args[++i];
      else if (key == "--profile" && i + 1 < args.size()) profile = args[++i];
      else if (key == "--output" && i + 1 < args.size()) output = args[++i];
      else if (key == "--panel-pose") take(6);
      else if (key == "--position-tolerance" || key == "--orientation-tolerance") take(3);
      else if (key == "--planning-time" || key == "--attempts") take(1);
      else return usage();
    } catch (const std::exception &) {
      return usage();
    }
  }
  const bool panel = numbers.count("--panel-pose");
  if (profile.empty() || panel == !named.empty() ||
      (panel && (!numbers.count("--position-tolerance") || !numbers.count("--orientation-tolerance")))) {
    return usage();
  }

  Reconfigure::Goal goal;
  goal.footprint_profile = profile;
  if (numbers.count("--planning-time")) goal.planning_time_s = numbers["--planning-time"][0];
  if (numbers.count("--attempts")) goal.planning_attempts = static_cast<int32_t>(numbers["--attempts"][0]);
  if (panel) {
    const auto & p = numbers["--panel-pose"];
    goal.target_type = Reconfigure::Goal::PANEL_POSE;
    goal.panel_pose.header.frame_id = "base_footprint";
    goal.panel_pose.pose.position.x = p[0];
    goal.panel_pose.pose.position.y = p[1];
    goal.panel_pose.pose.position.z = p[2];
    tf2::Quaternion q;
    q.setRPY(p[3], p[4], p[5]);
    goal.panel_pose.pose.orientation.x = q.x();
    goal.panel_pose.pose.orientation.y = q.y();
    goal.panel_pose.pose.orientation.z = q.z();
    goal.panel_pose.pose.orientation.w = q.w();
    const auto & t = numbers["--position-tolerance"];
    goal.position_tolerance.x = t[0];
    goal.position_tolerance.y = t[1];
    goal.position_tolerance.z = t[2];
    const auto & o = numbers["--orientation-tolerance"];
    goal.orientation_tolerance.x = o[0];
    goal.orientation_tolerance.y = o[1];
    goal.orientation_tolerance.z = o[2];
  } else {
    goal.target_type = Reconfigure::Goal::NAMED_STATE;
    goal.named_state = named;
  }

  const auto node = std::make_shared<rclcpp::Node>("reconfigure_panel_cli");
  const auto client = rclcpp_action::create_client<Reconfigure>(node, "/reconfigure_panel");
  if (!client->wait_for_action_server(30s)) {
    std::cerr << "/reconfigure_panel is not available\n";
    rclcpp::shutdown();
    return 1;
  }
  auto sent = client->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(node, sent, 10s) != rclcpp::FutureReturnCode::SUCCESS || !sent.get()) {
    std::cerr << "goal was not accepted\n";
    rclcpp::shutdown();
    return 1;
  }
  auto result_future = client->async_get_result(sent.get());
  // The server bounds every phase well below this; a missing result means it is gone.
  if (rclcpp::spin_until_future_complete(node, result_future, 300s) != rclcpp::FutureReturnCode::SUCCESS) {
    std::cerr << "no result within 300 s\n";
    client->async_cancel_goal(sent.get());
    rclcpp::shutdown();
    return 1;
  }
  const auto result = result_future.get().result;
  if (!result) {
    std::cerr << "the server returned no result\n";
    rclcpp::shutdown();
    return 1;
  }

  nlohmann::ordered_json json;
  json["error_code"] = code_name(result->error_code);
  json["message"] = result->message;
  json["applied_footprint_profile"] = result->applied_footprint_profile;
  json["profile_violated"] = result->profile_violated;
  json["reached_joint_positions"] = result->reached_joint_positions;
  const auto & r = result->reached_panel_pose;
  json["reached_panel_pose"] = {{"xyz", {r.position.x, r.position.y, r.position.z}},
                                {"quaternion_xyzw", {r.orientation.x, r.orientation.y, r.orientation.z, r.orientation.w}}};
  for (const auto & [key, value] : std::vector<std::pair<const char *, double>>{
         {"planned_containment_margin_m", result->planned_containment_margin_m},
         {"measured_containment_margin_m", result->measured_containment_margin_m},
         {"planning_time_s", result->planning_time_s}, {"execution_time_s", result->execution_time_s},
         {"footprint_switch_time_s", result->footprint_switch_time_s},
         {"trajectory_duration_s", result->trajectory_duration_s},
         {"joint_path_length_rad", result->joint_path_length_rad},
         {"min_planned_clearance_m", result->min_planned_clearance_m},
         {"max_path_error_rad", result->max_path_error_rad}, {"hold_error_rad", result->hold_error_rad}}) {
    json[key] = std::isfinite(value) ? nlohmann::ordered_json(value) : nlohmann::ordered_json(nullptr);
  }
  const auto text = json.dump(2);
  if (!output.empty()) std::ofstream(output) << text << "\n";
  std::cout << text << "\n";
  rclcpp::shutdown();
  return result->error_code == Reconfigure::Result::SUCCESS ? 0 : 3;
}
