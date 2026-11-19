// Copyright (c) 2012, Willow Garage, Inc. (LineIterator, BSD-3-Clause); Copyright (c) 2019 Intel Corporation
// (FootprintCollisionChecker, Apache-2.0); Copyright (c) 2020, Samsung Research America (GridCollisionChecker,
// Apache-2.0). See third_party/NOTICE.md.
//
// Ported from navigation2 1.3.13: nav2_util/line_iterator.hpp, nav2_costmap_2d/footprint_collision_checker.cpp,
// nav2_smac_planner/collision_checker.cpp and utils.hpp (findCircumscribedCost). Spec 2 Amendment 1 / ADR 0019.
#pragma once
#include <vector>
#include "mobile_manipulator_routing/smac/costmap.hpp"

namespace mobile_manipulator_routing::smac
{
// Smac's float cost constants (nav2_smac_planner/constants.hpp).
constexpr float UNKNOWN = 255.0;
constexpr float OCCUPIED = 254.0;
constexpr float INSCRIBED = 253.0;
constexpr float MAX_NON_OBSTACLE = 252.0;

class LineIterator
{
public:
  LineIterator(int x0, int y0, int x1, int y1);
  bool isValid() const { return curpixel_ <= numpixels_; }
  void advance();
  int getX() const { return x_; }
  int getY() const { return y_; }

private:
  int x0_, y0_, x1_, y1_, x_, y_, deltax_, deltay_, curpixel_;
  int xinc1_, xinc2_, yinc1_, yinc2_, den_, num_, numadd_, numpixels_;
};

class FootprintCollisionChecker
{
public:
  explicit FootprintCollisionChecker(const Costmap * costmap = nullptr) : costmap_(costmap) {}
  double footprintCost(const Footprint & footprint) const;
  double footprintCostAtPose(double x, double y, double theta, const Footprint & footprint) const;
  double lineCost(int x0, int x1, int y0, int y1) const;
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;
  double pointCost(int x, int y) const;
  void setCostmap(const Costmap * costmap) { costmap_ = costmap; }
  const Costmap * getCostmap() const { return costmap_; }

protected:
  const Costmap * costmap_;
};

// The inflation cost at the circumscribed radius (findCircumscribedCost), or 0 when the inflation radius is below
// the circumscribed radius.
double findCircumscribedCost(const LayeredCostmap & layered);

class GridCollisionChecker : public FootprintCollisionChecker
{
public:
  GridCollisionChecker() = default;
  GridCollisionChecker(const Costmap * costmap, unsigned int num_quantizations);
  void setFootprint(const Footprint & footprint, bool radius, double possible_collision_cost);
  bool inCollision(const float & x, const float & y, const float & angle_bin, const bool & traverse_unknown);
  bool inCollision(const unsigned int & i, const bool & traverse_unknown);
  float getCost() const { return center_cost_; }
  const std::vector<float> & getPrecomputedAngles() const { return angles_; }

private:
  static bool outsideRange(const unsigned int & max, const float & value) { return value < 0.0f || value > max; }
  std::vector<Footprint> oriented_footprints_;
  Footprint unoriented_footprint_;
  float center_cost_ = 0.0f;
  bool footprint_is_radius_ = false;
  std::vector<float> angles_;
  float possible_collision_cost_ = -1.0f;
};
}  // namespace mobile_manipulator_routing::smac
