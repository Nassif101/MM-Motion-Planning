// Check the qualified (non-planned) arm transition against the current MoveIt scene.
//
// The commissioning transitions move in a straight joint-space line through home (the
// synchronized cubic only changes the timing). This samples that path from one qualified
// pose to another and asks move_group's /check_state_validity about each sample, to show
// whether a scenario's obstacle really blocks the naive transition that MoveIt must avoid.
// Read-only: nothing moves.
//
// Usage: transition_validity_check --from POSE --to POSE [--samples N]
//   (POSE from qualified_payload.json poses_rad)
// Exit codes: 0 every sample valid, 3 some sample in collision, 1 usage or service failure.
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  const auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  std::string from, to;
  int samples = 50;
  for (size_t i = 1; i + 1 < args.size(); i += 2) {
    if (args[i] == "--from") from = args[i + 1];
    else if (args[i] == "--to") to = args[i + 1];
    else if (args[i] == "--samples") samples = std::stoi(args[i + 1]);
  }
  std::ifstream stream(ament_index_cpp::get_package_share_directory("mobile_manipulator_control") +
                       "/config/qualified_payload.json");
  const auto payload = nlohmann::json::parse(stream);
  if (from.empty() || to.empty() || !payload["poses_rad"].contains(from) || !payload["poses_rad"].contains(to) ||
      samples < 2) {
    std::cerr << "usage: transition_validity_check --from POSE --to POSE [--samples N]\n";
    return 1;
  }
  const auto names = payload["joint_order"].get<std::vector<std::string>>();
  std::vector<std::vector<double>> waypoints = {payload["poses_rad"][from].get<std::vector<double>>()};
  if (from != "home" && to != "home") waypoints.push_back(payload["poses_rad"]["home"].get<std::vector<double>>());
  waypoints.push_back(payload["poses_rad"][to].get<std::vector<double>>());

  const auto node = std::make_shared<rclcpp::Node>("transition_validity_check");
  const auto client = node->create_client<moveit_msgs::srv::GetStateValidity>("/check_state_validity");
  if (!client->wait_for_service(30s)) {
    std::cerr << "/check_state_validity is not available\n";
    rclcpp::shutdown();
    return 1;
  }
  int invalid = 0;
  nlohmann::json first_invalid = nullptr;
  for (size_t leg = 0; leg + 1 < waypoints.size(); ++leg) {
    for (int k = 0; k < samples; ++k) {
      const double s = static_cast<double>(k) / (samples - 1);
      auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
      request->group_name = "arm";
      request->robot_state.is_diff = true;
      request->robot_state.joint_state.name = names;
      for (size_t j = 0; j < names.size(); ++j) {
        request->robot_state.joint_state.position.push_back(
          waypoints[leg][j] + s * (waypoints[leg + 1][j] - waypoints[leg][j]));
      }
      auto future = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, future, 10s) != rclcpp::FutureReturnCode::SUCCESS) {
        std::cerr << "/check_state_validity did not answer\n";
        rclcpp::shutdown();
        return 1;
      }
      const auto response = future.get();
      if (response->valid) continue;
      ++invalid;
      if (first_invalid.is_null()) {
        nlohmann::json contacts = nlohmann::json::array();
        for (const auto & c : response->contacts) contacts.push_back({c.contact_body_1, c.contact_body_2});
        first_invalid = {{"leg", leg + 1}, {"fraction", s},
                         {"positions", request->robot_state.joint_state.position}, {"contacts", contacts}};
      }
    }
  }
  const nlohmann::json report = {{"from", from}, {"to", to}, {"samples_per_leg", samples},
                                 {"legs", waypoints.size() - 1}, {"invalid_samples", invalid},
                                 {"first_invalid", first_invalid}};
  std::cout << report.dump() << std::endl;
  rclcpp::shutdown();
  return invalid ? 3 : 0;
}
