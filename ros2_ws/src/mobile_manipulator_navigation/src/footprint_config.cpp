#include "mobile_manipulator_navigation/footprint_config.hpp"

#include <stdexcept>

namespace mmg = mobile_manipulator_geometry;

namespace mobile_manipulator_navigation
{
mmg::DiscModelConfig disc_config_of(const Json & disc)
{
  mmg::DiscModelConfig config;
  const Json radii = disc.contains("radii") ? disc.at("radii") : Json::object();
  for (const auto & link : disc.at("links")) {
    const auto name = link.get<std::string>();
    config.links.push_back({name, radii.contains(name) ? std::optional<double>(radii.at(name).get<double>())
                                                       : std::nullopt});
  }
  config.disc_samples = disc.at("disc_samples").get<int>();
  config.segment_samples = disc.at("segment_samples").get<int>();
  for (const auto & link : disc.at("base_links")) config.base_links.insert(link.get<std::string>());
  return config;
}

std::shared_ptr<const mmg::FootprintModel> footprint_model(const std::string & name, const Json & parameters,
                                                           std::shared_ptr<const mmg::FootprintProjector> projector)
{
  if (name == "mesh") {
    return std::make_shared<const mmg::MeshHullModel>(projector, parameters.at("mesh").at("cylinder_sides").get<int>());
  }
  if (name == "disc") return std::make_shared<const mmg::DiscHullModel>(projector, disc_config_of(parameters.at("disc")));
  throw std::invalid_argument("unknown footprint model '" + name + "' (mesh or disc)");
}
}  // namespace mobile_manipulator_navigation
