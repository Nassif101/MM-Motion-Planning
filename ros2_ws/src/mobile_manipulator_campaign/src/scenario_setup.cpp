#include "mobile_manipulator_campaign/scenario_setup.hpp"

namespace mobile_manipulator_campaign
{
mobile_manipulator_plant::ScenarioSetup scenario_setup(const mobile_manipulator_mpc::Scene & scene, const std::string & name)
{
  mobile_manipulator_plant::ScenarioSetup s;
  s.name = name;
  for (const auto & box : scene.boxes) s.boxes.push_back({box.name, box.center, box.size, box.yaw});
  if (!scene.scenario) return s;
  for (const auto & m : scene.scenario->value("movers", mobile_manipulator_navigation::Json::array())) {
    mobile_manipulator_plant::MoverSpec spec;
    spec.name = m.at("name").get<std::string>();
    spec.start = {m.at("start").at(0).get<double>(), m.at("start").at(1).get<double>()};
    spec.end = {m.at("end").at(0).get<double>(), m.at("end").at(1).get<double>()};
    spec.size = {m.at("size_x").get<double>(), m.at("size_y").get<double>(), m.at("height").get<double>()};
    spec.speed_mps = m.at("speed_mps").get<double>();
    spec.trigger_distance_m = m.at("trigger_distance_m").get<double>();
    spec.crossings = m.at("crossings").get<int>();
    s.movers.push_back(spec);
  }
  return s;
}
}  // namespace mobile_manipulator_campaign
