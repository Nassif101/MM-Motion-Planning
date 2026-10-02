// Load the known B3 collision world into move_group once, then exit.
//
// Publishes the exported static boxes, optionally the scenario's unmapped obstacle boxes
// (off in Octomap mode, where MoveIt must perceive them as Nav2 does), the raised floor,
// and the panel attached to tool0. The floor ACM entries are merged into the matrix read
// from move_group, because a diff carrying an ACM replaces the whole matrix.
//
// Parameters: boxes_file, payload_file, scenario ("" = none), include_scenario_obstacles.
// Exit codes: 0 applied, 1 failure.
#include <chrono>
#include <fstream>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mobile_manipulator_manipulation/scene_diff.hpp"
#include "mobile_manipulator_navigation/footprint_projection.hpp"
#include "mobile_manipulator_navigation/scenario_spec.hpp"

namespace mmm = mobile_manipulator_manipulation;
namespace mmn = mobile_manipulator_navigation;
using namespace std::chrono_literals;

namespace
{
std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot read " + path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

template<typename Service>
typename Service::Response::SharedPtr call(const rclcpp::Node::SharedPtr & node, const std::string & name,
                                           typename Service::Request::SharedPtr request)
{
  const auto client = node->create_client<Service>(name);
  if (!client->wait_for_service(60s)) throw std::runtime_error(name + " is not available");
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, 30s) != rclcpp::FutureReturnCode::SUCCESS) {
    throw std::runtime_error(name + " did not answer");
  }
  return future.get();
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("planning_scene_loader");
  const auto navigation = ament_index_cpp::get_package_share_directory("mobile_manipulator_navigation");
  const auto control = ament_index_cpp::get_package_share_directory("mobile_manipulator_control");
  const auto boxes_file = node->declare_parameter(
    "boxes_file", navigation + "/maps/construction_site.boxes.yaml");
  const auto payload_file = node->declare_parameter("payload_file", control + "/config/qualified_payload.json");
  const auto scenario = node->declare_parameter("scenario", std::string());
  const auto include_obstacles = node->declare_parameter("include_scenario_obstacles", true);
  int status = 0;
  try {
    mmm::SceneInputs inputs;
    inputs.static_boxes = mmm::load_boxes(boxes_file);
    inputs.panel = mmn::payload_from_json(mmn::Json::parse(read_file(payload_file)));
    if (!scenario.empty() && include_obstacles) {
      const auto spec = mmn::ScenarioConfig(navigation).load("scenarios.yaml").at("scenarios").at(scenario);
      for (const auto & obstacle : spec.value("obstacles", mmn::Json::array())) {
        inputs.scenario_boxes.push_back(mmm::scenario_box(obstacle));
      }
    }
    auto diff = mmm::build_scene_diff(inputs);

    auto get = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    get->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    const auto current = call<moveit_msgs::srv::GetPlanningScene>(node, "/get_planning_scene", get);
    diff.allowed_collision_matrix = mmm::allow_floor_contacts(current->scene.allowed_collision_matrix);

    auto apply = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    apply->scene = diff;
    if (!call<moveit_msgs::srv::ApplyPlanningScene>(node, "/apply_planning_scene", apply)->success) {
      throw std::runtime_error("move_group rejected the planning scene");
    }
    RCLCPP_INFO(node->get_logger(), "Loaded %zu static and %zu scenario boxes, the floor (top %.2f m) "
                "and the attached panel", inputs.static_boxes.size(), inputs.scenario_boxes.size(),
                inputs.floor_top_m);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
