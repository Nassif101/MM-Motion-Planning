#pragma once

#include <memory>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "mobile_manipulator_navigation/known_world.hpp"
#include "mobile_manipulator_routing/footprint.hpp"
#include "mobile_manipulator_routing/primitives.hpp"

namespace mobile_manipulator_campaign
{
struct RoutingInputs
{
  std::shared_ptr<const mobile_manipulator_routing::GridSnapshot> grid;
  std::shared_ptr<const mobile_manipulator_routing::PrimitiveSet> primitives;
  mobile_manipulator_routing::Polygon footprint;
  nlohmann::json metadata;
};

// Only explicitly declared known additions enter the route map. In particular, do not pass
// every MuJoCo/scenario body here: unmapped geometry must arrive through observations.
RoutingInputs load_routing_inputs(const std::string & navigation_root, const std::string & urdf,
  const nlohmann::ordered_json & payload, const std::string & map_path = "",
  const std::string & primitive_path = "",
  const std::vector<mobile_manipulator_navigation::Box> & known_additions = {});
}  // namespace mobile_manipulator_campaign
