// Copyright (c) 2012, Willow Garage, Inc. (BSD-3-Clause); Copyright (c) 2019 Intel Corporation (Apache-2.0);
// Copyright (c) 2020, Samsung Research America (Apache-2.0). See third_party/NOTICE.md.
// Ported from navigation2 1.3.13; the source files are named in collision_checker.hpp.
#include "mobile_manipulator_routing/smac/collision_checker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace mobile_manipulator_routing::smac
{
LineIterator::LineIterator(int x0, int y0, int x1, int y1)
: x0_(x0), y0_(y0), x1_(x1), y1_(y1), x_(x0), y_(y0), deltax_(std::abs(x1 - x0)), deltay_(std::abs(y1 - y0)),
  curpixel_(0)
{
  if (x1_ >= x0_) {
    xinc1_ = 1;
    xinc2_ = 1;
  } else {
    xinc1_ = -1;
    xinc2_ = -1;
  }
  if (y1_ >= y0_) {
    yinc1_ = 1;
    yinc2_ = 1;
  } else {
    yinc1_ = -1;
    yinc2_ = -1;
  }
  if (deltax_ >= deltay_) {
    xinc1_ = 0;
    yinc2_ = 0;
    den_ = deltax_;
    num_ = deltax_ / 2;
    numadd_ = deltay_;
    numpixels_ = deltax_;
  } else {
    xinc2_ = 0;
    yinc1_ = 0;
    den_ = deltay_;
    num_ = deltay_ / 2;
    numadd_ = deltax_;
    numpixels_ = deltay_;
  }
}

void LineIterator::advance()
{
  num_ += numadd_;
  if (num_ >= den_) {
    num_ -= den_;
    x_ += xinc1_;
    y_ += yinc1_;
  }
  x_ += xinc2_;
  y_ += yinc2_;
  curpixel_++;
}

double FootprintCollisionChecker::footprintCost(const Footprint & footprint) const
{
  unsigned int x0, x1, y0, y1;
  double footprint_cost = 0.0;
  if (!worldToMap(footprint[0].x, footprint[0].y, x0, y0)) {
    return static_cast<double>(kLethalObstacle);
  }
  unsigned int xstart = x0;
  unsigned int ystart = y0;
  for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
    if (!worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1)) {
      return static_cast<double>(kLethalObstacle);
    }
    footprint_cost = std::max(lineCost(x0, x1, y0, y1), footprint_cost);
    x0 = x1;
    y0 = y1;
    if (footprint_cost == static_cast<double>(kLethalObstacle)) {
      return footprint_cost;
    }
  }
  return std::max(lineCost(xstart, x1, ystart, y1), footprint_cost);
}

double FootprintCollisionChecker::lineCost(int x0, int x1, int y0, int y1) const
{
  double line_cost = 0.0;
  double point_cost = -1.0;
  for (LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    point_cost = pointCost(line.getX(), line.getY());
    if (point_cost == static_cast<double>(kLethalObstacle)) {
      return point_cost;
    }
    if (line_cost < point_cost) {
      line_cost = point_cost;
    }
  }
  return line_cost;
}

bool FootprintCollisionChecker::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  return costmap_->worldToMap(wx, wy, mx, my);
}

double FootprintCollisionChecker::pointCost(int x, int y) const
{
  return static_cast<double>(costmap_->getCost(x, y));
}

double FootprintCollisionChecker::footprintCostAtPose(double x, double y, double theta,
                                                      const Footprint & footprint) const
{
  double cos_th = std::cos(theta);
  double sin_th = std::sin(theta);
  Footprint oriented_footprint;
  oriented_footprint.reserve(footprint.size());
  for (unsigned int i = 0; i < footprint.size(); ++i) {
    oriented_footprint.push_back({x + (footprint[i].x * cos_th - footprint[i].y * sin_th),
                                  y + (footprint[i].x * sin_th + footprint[i].y * cos_th)});
  }
  return footprintCost(oriented_footprint);
}

double findCircumscribedCost(const LayeredCostmap & layered)
{
  // The reference configuration has an inflation layer.
  const double circum_radius = layered.circumscribed_radius;
  const double resolution = layered.costmap.getResolution();
  if (layered.inflation_radius < circum_radius) {
    return 0.0;
  }
  return static_cast<double>(inflation_cost(circum_radius / resolution, resolution, layered.inscribed_radius,
                                            layered.cost_scaling_factor));
}

GridCollisionChecker::GridCollisionChecker(const Costmap * costmap, unsigned int num_quantizations)
: FootprintCollisionChecker(costmap)
{
  float bin_size = 2 * M_PI / static_cast<float>(num_quantizations);
  angles_.reserve(num_quantizations);
  for (unsigned int i = 0; i != num_quantizations; i++) {
    angles_.push_back(bin_size * i);
  }
}

void GridCollisionChecker::setFootprint(const Footprint & footprint, bool radius, double possible_collision_cost)
{
  possible_collision_cost_ = static_cast<float>(possible_collision_cost);
  footprint_is_radius_ = radius;
  if (radius) {
    return;
  }
  if (footprint == unoriented_footprint_) {
    return;
  }
  oriented_footprints_.clear();
  oriented_footprints_.reserve(angles_.size());
  double sin_th, cos_th;
  const unsigned int footprint_size = footprint.size();
  for (unsigned int i = 0; i != angles_.size(); i++) {
    sin_th = std::sin(angles_[i]);
    cos_th = std::cos(angles_[i]);
    Footprint oriented_footprint;
    oriented_footprint.reserve(footprint_size);
    for (unsigned int j = 0; j < footprint_size; j++) {
      oriented_footprint.push_back({footprint[j].x * cos_th - footprint[j].y * sin_th,
                                    footprint[j].x * sin_th + footprint[j].y * cos_th});
    }
    oriented_footprints_.push_back(oriented_footprint);
  }
  unoriented_footprint_ = footprint;
}

bool GridCollisionChecker::inCollision(const float & x, const float & y, const float & angle_bin,
                                       const bool & traverse_unknown)
{
  if (outsideRange(costmap_->getSizeInCellsX(), x) || outsideRange(costmap_->getSizeInCellsY(), y)) {
    return true;
  }
  center_cost_ = static_cast<float>(costmap_->getCost(static_cast<unsigned int>(x + 0.5f),
                                                      static_cast<unsigned int>(y + 0.5f)));
  if (!footprint_is_radius_) {
    if (center_cost_ < possible_collision_cost_ && possible_collision_cost_ > 0.0f) {
      return false;
    }
    if (center_cost_ == UNKNOWN && !traverse_unknown) {
      return true;
    }
    if (center_cost_ == INSCRIBED || center_cost_ == OCCUPIED) {
      return true;
    }
    double wx, wy;
    // As upstream: mapToWorld takes unsigned cells, so the footprint is centred on the centre of the truncated cell.
    costmap_->mapToWorld(static_cast<double>(x), static_cast<double>(y), wx, wy);
    const Footprint & oriented_footprint = oriented_footprints_[angle_bin];
    Footprint current_footprint;
    current_footprint.reserve(oriented_footprint.size());
    for (unsigned int i = 0; i < oriented_footprint.size(); ++i) {
      current_footprint.push_back({wx + oriented_footprint[i].x, wy + oriented_footprint[i].y});
    }
    float footprint_cost = static_cast<float>(footprintCost(current_footprint));
    if (footprint_cost == UNKNOWN && traverse_unknown) {
      return false;
    }
    return footprint_cost >= OCCUPIED;
  } else {
    if (center_cost_ == UNKNOWN && traverse_unknown) {
      return false;
    }
    return center_cost_ >= INSCRIBED;
  }
}

bool GridCollisionChecker::inCollision(const unsigned int & i, const bool & traverse_unknown)
{
  center_cost_ = costmap_->getCost(i);
  if (center_cost_ == UNKNOWN && traverse_unknown) {
    return false;
  }
  return center_cost_ >= INSCRIBED;
}
}  // namespace mobile_manipulator_routing::smac
