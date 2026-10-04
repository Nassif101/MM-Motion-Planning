#pragma once
// Footprint models from config/dynamic_footprint.yaml (no ROS graph).
#include <memory>

#include "mobile_manipulator_geometry/footprint_model.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace mobile_manipulator_navigation
{
// The `disc` mapping of the node parameters: links (optional radius_m overrides as
// {link: radius} under `radii`), disc_samples, segment_samples, base_links.
mobile_manipulator_geometry::DiscModelConfig disc_config_of(const Json & disc);

// The model named by `name` ("mesh" or "disc") with the parameters of dynamic_footprint.yaml
// (`ros__parameters` of dynamic_footprint_node). Throws std::invalid_argument otherwise.
std::shared_ptr<const mobile_manipulator_geometry::FootprintModel> footprint_model(
  const std::string & name, const Json & parameters,
  std::shared_ptr<const mobile_manipulator_geometry::FootprintProjector> projector);
}  // namespace mobile_manipulator_navigation
