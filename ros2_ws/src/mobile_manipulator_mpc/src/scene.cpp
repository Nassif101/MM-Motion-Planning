#include "mobile_manipulator_mpc/scene.hpp"

#include <algorithm>

namespace mobile_manipulator_mpc
{
namespace mmn = mobile_manipulator_navigation;

Scene load_scene(const std::string & navigation_share, const std::string & site_param,
                 const std::string & scenario_name, const std::string & boxes_param)
{
  Scene scene;
  std::optional<std::string> scenario_site;
  if (!scenario_name.empty()) {
    const auto resolved = mmn::ScenarioConfig(navigation_share).resolve(scenario_name);
    scene.scenario = resolved.at("scenario");
    scenario_site = resolved.at("site").get<std::string>();
  }
  scene.site = mmn::effective_site(site_param, scenario_site);
  scene.boxes_file = boxes_param.empty() ? mmn::site_boxes_file(navigation_share, scene.site) : boxes_param;
  scene.boxes = mmn::known_boxes(scene.boxes_file, scene.scenario ? &*scene.scenario : nullptr);
  if (scene.scenario) {
    for (const auto & mover : scene.scenario->value("movers", mmn::Json::array())) {
      scene.movers.push_back({mover.at("name").get<std::string>(),
                              Eigen::Vector3d(mover.at("size_x").get<double>(), mover.at("size_y").get<double>(),
                                              mover.at("height").get<double>())});
    }
    std::sort(scene.movers.begin(), scene.movers.end(),
              [](const MoverSpec & a, const MoverSpec & b) { return a.name < b.name; });
  }
  return scene;
}
}  // namespace mobile_manipulator_mpc
