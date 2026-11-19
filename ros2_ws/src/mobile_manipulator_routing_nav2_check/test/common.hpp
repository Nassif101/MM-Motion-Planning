// Shared fixtures of the Smac-port checks: the V2 map's port costmap, copied into a nav2_costmap_2d::Costmap2D for
// the originals (S1 showed the two costmaps are byte-identical).
#pragma once
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "mobile_manipulator_routing/map_io.hpp"
#include "mobile_manipulator_routing/smac/costmap.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace check
{
namespace sm = mobile_manipulator_routing::smac;

inline const sm::Footprint & home_footprint()
{
  static const sm::Footprint f{{0.54, 0.62}, {0.54, -0.62}, {-0.70, -0.62}, {-0.70, 0.62}};
  return f;
}

inline sm::LayeredCostmap v2_costmap(const sm::Footprint & footprint = home_footprint())
{
  const auto grid = mobile_manipulator_routing::load_map(std::string(NAVIGATION_SOURCE) +
                                                         "/maps/construction_site_v2.yaml");
  return sm::build_costmap(grid, footprint, {});
}

inline std::shared_ptr<nav2_costmap_2d::Costmap2D> to_nav2(const sm::Costmap & port)
{
  auto out = std::make_shared<nav2_costmap_2d::Costmap2D>(port.getSizeInCellsX(), port.getSizeInCellsY(),
                                                          port.getResolution(), port.getOriginX(),
                                                          port.getOriginY());
  for (unsigned int y = 0; y < port.getSizeInCellsY(); ++y)
    for (unsigned int x = 0; x < port.getSizeInCellsX(); ++x) out->setCost(x, y, port.getCost(x, y));
  return out;
}

inline std::vector<geometry_msgs::msg::Point> to_msg(const sm::Footprint & footprint)
{
  std::vector<geometry_msgs::msg::Point> out;
  for (const auto & p : footprint) {
    geometry_msgs::msg::Point q;
    q.x = p.x;
    q.y = p.y;
    out.push_back(q);
  }
  return out;
}
}  // namespace check
