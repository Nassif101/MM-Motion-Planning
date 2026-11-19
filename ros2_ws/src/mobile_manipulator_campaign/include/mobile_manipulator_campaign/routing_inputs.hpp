#pragma once

#include <memory>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "mobile_manipulator_navigation/known_world.hpp"
#include "mobile_manipulator_routing/footprint.hpp"
#include "mobile_manipulator_routing/primitives.hpp"
#include "mobile_manipulator_routing/smac/costmap.hpp"
#include "mobile_manipulator_routing/smac/planner.hpp"

namespace mobile_manipulator_campaign
{
struct RoutingInputs
{
  std::shared_ptr<const mobile_manipulator_routing::GridSnapshot> grid;
  std::shared_ptr<const mobile_manipulator_routing::PrimitiveSet> primitives;
  // The validation pass's footprint: the hull with the geometric padding only, the same polygon as
  // costmap_footprint (spec 2 Amendment 3; the costmap's footprint_padding applies to Smac's own pose checks).
  mobile_manipulator_routing::Polygon footprint;
  // The polygon the Nav2 global costmap was configured with (geometric padding only); the Smac port pads it by the
  // costmap's footprint_padding itself, as Costmap2DROS does (spec 2 Amendment 1).
  mobile_manipulator_routing::Polygon costmap_footprint;
  mobile_manipulator_routing::smac::CostmapSettings costmap;  // footprint_padding and the inflation layer's settings
  std::string primitive_path;
  // The Lattice block of nav2_global_planning.yaml, with lattice_filepath set to `primitive_path`. Its
  // max_planning_time is not used: the port's limit is max_iterations (A1.2 item 1).
  mobile_manipulator_routing::smac::LatticeSettings lattice;
  nlohmann::json metadata;
};

// Only explicitly declared known additions enter the route map. In particular, do not pass
// every MuJoCo/scenario body here: unmapped geometry must arrive through observations.
RoutingInputs load_routing_inputs(const std::string & navigation_root, const std::string & urdf,
  const nlohmann::ordered_json & payload, const std::string & map_path = "",
  const std::string & primitive_path = "",
  const std::vector<mobile_manipulator_navigation::Box> & known_additions = {});
}  // namespace mobile_manipulator_campaign
