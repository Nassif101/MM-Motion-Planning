#pragma once
// The MPC's known world for a run: the site's boxes plus the scenario's obstacles, and the scenario's movers (by
// name, sorted, as /scenario/movers orders them). The node and the replay bench (Phase 4B-B) build it the same way.
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "mobile_manipulator_navigation/known_world.hpp"
#include "mobile_manipulator_navigation/scenario_spec.hpp"

namespace mobile_manipulator_mpc
{
struct MoverSpec
{
  std::string name;
  Eigen::Vector3d size;  // full size; the box stands on the ground
};

struct Scene
{
  std::string site, boxes_file;
  std::vector<mobile_manipulator_navigation::Box> boxes;
  std::vector<MoverSpec> movers;  // sorted by name
  std::optional<mobile_manipulator_navigation::Json> scenario;  // empty: no scenario (the map's boxes only)
};

// `navigation_share` is mobile_manipulator_navigation's share directory; an empty `site_param` takes the scenario's
// site, an empty `boxes_param` the site's boxes file. Throws as the navigation helpers do for an unknown scenario.
Scene load_scene(const std::string & navigation_share, const std::string & site_param,
                 const std::string & scenario_name, const std::string & boxes_param);
}  // namespace mobile_manipulator_mpc
