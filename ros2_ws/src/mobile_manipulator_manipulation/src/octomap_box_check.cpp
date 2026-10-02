// Count occupied lidar-Octomap voxels inside each obstacle box of a scenario.
//
// Reads move_group's Octomap (/get_planning_scene, map frame) and the scenario's boxes from
// scenarios.yaml; shows whether the Octomap scene source perceived each unmapped box (for
// example one inside the lidar's near-field blind zone). Read-only.
//
// Usage: octomap_box_check --scenario NAME
// Prints {"scenario": NAME, "octomap_voxels": N, "boxes": {name: {"voxels": n,
// "max_z": height of the highest voxel touching the box}}, "z_bands": {band bottom:
// {"voxels", "x": [min, max], "y": [min, max]}} per 0.25 m height band}; exit 1 on failure.
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <nlohmann/json.hpp>
#include <octomap_msgs/conversions.h>
#include <rclcpp/rclcpp.hpp>

#include "mobile_manipulator_manipulation/octomap_box.hpp"
#include "mobile_manipulator_navigation/scenario_spec.hpp"

namespace mmm = mobile_manipulator_manipulation;
namespace mmn = mobile_manipulator_navigation;
using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  const auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  if (args.size() != 3 || args[1] != "--scenario") {
    std::cerr << "usage: octomap_box_check --scenario NAME\n";
    return 1;
  }
  const std::string name = args[2];
  const auto node = std::make_shared<rclcpp::Node>("octomap_box_check");
  try {
    const auto scenario = mmn::ScenarioConfig(ament_index_cpp::get_package_share_directory(
      "mobile_manipulator_navigation")).load("scenarios.yaml").at("scenarios").at(name);
    const auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    if (!client->wait_for_service(30s)) throw std::runtime_error("/get_planning_scene is not available");
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = moveit_msgs::msg::PlanningSceneComponents::OCTOMAP;
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future, 10s) != rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("/get_planning_scene did not answer");
    }
    const auto response = future.get();
    const auto & octomap = response->scene.world.octomap;
    if (octomap.header.frame_id != "map") throw std::runtime_error("Octomap is not in map: " + octomap.header.frame_id);
    std::unique_ptr<octomap::AbstractOcTree> abstract(octomap_msgs::msgToMap(octomap.octomap));
    auto * tree = dynamic_cast<octomap::OcTree *>(abstract.get());
    nlohmann::json boxes = nlohmann::json::object();
    std::size_t total = 0;
    struct Band
    {
      std::size_t voxels = 0;
      double x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9;
    };
    std::map<int, Band> bands;  // 0.25 m height bands
    if (tree) {
      for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it) {
        if (!tree->isNodeOccupied(*it)) continue;
        total += 1;
        auto & band = bands[static_cast<int>(std::floor(it.getZ() / 0.25))];
        band.voxels += 1;
        band.x0 = std::min(band.x0, static_cast<double>(it.getX()));
        band.x1 = std::max(band.x1, static_cast<double>(it.getX()));
        band.y0 = std::min(band.y0, static_cast<double>(it.getY()));
        band.y1 = std::max(band.y1, static_cast<double>(it.getY()));
      }
      for (const auto & obstacle : scenario.value("obstacles", mmn::Json::array())) {
        const auto box = mmm::scenario_box(obstacle);
        boxes[box.name] = {{"voxels", mmm::occupied_voxels_in(*tree, box)},
                           {"max_z", mmm::highest_voxel_in(*tree, box)}};
      }
    }
    nlohmann::json histogram = nlohmann::json::object();
    for (const auto & [index, band] : bands) {
      char key[16];
      std::snprintf(key, sizeof(key), "%.2f", index * 0.25);
      histogram[key] = {{"voxels", band.voxels}, {"x", {band.x0, band.x1}}, {"y", {band.y0, band.y1}}};
    }
    std::cout << nlohmann::json{{"scenario", name}, {"octomap_voxels", total}, {"boxes", boxes},
                                {"z_bands", histogram}}.dump() << std::endl;
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
