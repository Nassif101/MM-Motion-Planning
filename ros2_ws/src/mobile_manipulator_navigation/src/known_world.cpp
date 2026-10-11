#include "mobile_manipulator_navigation/known_world.hpp"

#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace mobile_manipulator_navigation
{
Box scenario_box(const Json & obstacle)
{
  const double height = obstacle.at("height").get<double>();
  const double bottom = obstacle.value("z", 0.0);
  return {obstacle.at("name").get<std::string>(),
          {obstacle.at("x").get<double>(), obstacle.at("y").get<double>(), bottom + height / 2.0},
          {obstacle.at("size_x").get<double>(), obstacle.at("size_y").get<double>(), height}};
}

std::vector<Box> load_boxes(const std::string & yaml_path)
{
  const auto root = YAML::LoadFile(yaml_path);
  if (root["frame_id"].as<std::string>() != "map") throw std::invalid_argument("planning boxes must be in map");
  std::vector<Box> boxes;
  for (const auto & box : root["boxes"]) {
    const auto c = box["center"], s = box["size"];
    boxes.push_back({box["name"].as<std::string>(),
                     {c[0].as<double>(), c[1].as<double>(), c[2].as<double>()},
                     {s[0].as<double>(), s[1].as<double>(), s[2].as<double>()},
                     box["yaw"] ? box["yaw"].as<double>() : 0.0});
  }
  return boxes;
}

std::vector<Box> known_boxes(const std::string & boxes_file, const Json * scenario)
{
  auto boxes = load_boxes(boxes_file);
  if (scenario != nullptr) {
    for (const auto & obstacle : scenario->value("obstacles", Json::array())) boxes.push_back(scenario_box(obstacle));
  }
  return boxes;
}
}  // namespace mobile_manipulator_navigation
