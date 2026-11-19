// Copyright (c) 2008, 2013, Willow Garage, Inc.; Copyright (c) 2015, Fetch Robotics, Inc.
// Licensed under the BSD 3-Clause License (nav2_costmap_2d); see third_party/NOTICE.md.
//
// Ported from navigation2 1.3.13 (f4108e5b), nav2_costmap_2d: costmap_2d.cpp (index and world/map conversions),
// cost_values.hpp, footprint.cpp (padFootprint, calculateMinAndMaxDistances), costmap_math.cpp (distanceToLine),
// plugins/static_layer.cpp (interpretValue) and plugins/inflation_layer.cpp (computeCaches, updateCosts, enqueue).
// Spec 2 Amendment 1 / ADR 0019. The layered costmap of the reference configuration is reproduced for a full update:
// static layer (track_unknown_space false, trinary), then the inflation layer (inflate_unknown and
// inflate_around_unknown false).
#pragma once
#include <cstdint>
#include <utility>
#include <vector>
#include "mobile_manipulator_routing/grid.hpp"
#include "mobile_manipulator_routing/types.hpp"

namespace mobile_manipulator_routing::smac
{
constexpr unsigned char kNoInformation = 255;
constexpr unsigned char kLethalObstacle = 254;
constexpr unsigned char kInscribedInflatedObstacle = 253;
constexpr unsigned char kMaxNonObstacle = 252;
constexpr unsigned char kFreeSpace = 0;

struct Point { double x = 0.0, y = 0.0; };
using Footprint = std::vector<Point>;

// nav2_costmap_2d::Costmap2D's storage and conversions (the parts Smac uses).
class Costmap
{
public:
  Costmap() = default;
  Costmap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y,
          unsigned char default_value = kFreeSpace);
  unsigned int getSizeInCellsX() const { return size_x_; }
  unsigned int getSizeInCellsY() const { return size_y_; }
  double getResolution() const { return resolution_; }
  double getOriginX() const { return origin_x_; }
  double getOriginY() const { return origin_y_; }
  unsigned int getIndex(unsigned int mx, unsigned int my) const { return my * size_x_ + mx; }
  unsigned char getCost(unsigned int mx, unsigned int my) const { return costmap_[getIndex(mx, my)]; }
  unsigned char getCost(unsigned int index) const { return costmap_[index]; }
  void setCost(unsigned int mx, unsigned int my, unsigned char cost) { costmap_[getIndex(mx, my)] = cost; }
  unsigned char * getCharMap() { return costmap_.data(); }
  const std::vector<unsigned char> & data() const { return costmap_; }
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const;
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;
  bool worldToMapContinuous(double wx, double wy, float & mx, float & my) const;
  unsigned int cellDistance(double world_dist) const;

private:
  unsigned int size_x_ = 0, size_y_ = 0;
  double resolution_ = 0.0, origin_x_ = 0.0, origin_y_ = 0.0;
  std::vector<unsigned char> costmap_;
};

void padFootprint(Footprint & footprint, double padding);
// (inscribed radius, circumscribed radius)
std::pair<double, double> calculateMinAndMaxDistances(const Footprint & footprint);

struct CostmapSettings
{
  double footprint_padding = 0.01;
  double inflation_radius = 1.0;
  double cost_scaling_factor = 5.0;
};

// The global costmap of the reference configuration after one full update: the map's occupied cells (known and
// observed) are lethal, unknown cells are free (track_unknown_space false), then the inflation layer. `footprint` is
// unpadded, as configured; the padded footprint gives the inscribed radius.
struct LayeredCostmap
{
  Costmap costmap;
  Footprint padded_footprint;
  double inscribed_radius = 0.0, circumscribed_radius = 0.0;
  double inflation_radius = 0.0, cost_scaling_factor = 0.0;
};
LayeredCostmap build_costmap(const GridSnapshot & grid, const Footprint & footprint, const CostmapSettings & settings);

// The inflation layer's distance-to-cost function (InflationLayer::computeCost), distance in cells.
unsigned char inflation_cost(double distance_cells, double resolution, double inscribed_radius,
                             double cost_scaling_factor);
// InflationLayer::updateCosts over the whole map (the static layer's values already in `master`).
void inflate(Costmap & master, double inflation_radius, double inscribed_radius, double cost_scaling_factor);
}  // namespace mobile_manipulator_routing::smac
