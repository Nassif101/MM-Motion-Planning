// Copyright (c) 2021, Samsung Research America. Licensed under the Apache License, Version 2.0; see
// third_party/NOTICE.md. Ported from navigation2 1.3.13 nav2_smac_planner smoother.cpp; the tf2 functions follow
// tf2 (Jazzy) LinearMath/Quaternion.hpp and impl/utils.hpp.
#include "mobile_manipulator_routing/smac/smoother.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

#include "mobile_manipulator_routing/smac/collision_checker.hpp"
#include "mobile_manipulator_routing/smac/node_lattice.hpp"

namespace mobile_manipulator_routing::smac
{
Quaternion quaternion_from_euler_roll(double theta)
{
  // tf2::Quaternion::setEuler(yaw = 0, pitch = 0, roll = theta)
  const double halfYaw = 0.0 * 0.5, halfPitch = 0.0 * 0.5, halfRoll = theta * 0.5;
  const double cosYaw = std::cos(halfYaw), sinYaw = std::sin(halfYaw);
  const double cosPitch = std::cos(halfPitch), sinPitch = std::sin(halfPitch);
  const double cosRoll = std::cos(halfRoll), sinRoll = std::sin(halfRoll);
  return {cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
          cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
          sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
          cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw};
}

Quaternion orientationAroundZAxis(double angle)
{
  // tf2::Quaternion::setRPY(roll = 0, pitch = 0, yaw = angle)
  const double halfYaw = angle * 0.5, halfPitch = 0.0 * 0.5, halfRoll = 0.0 * 0.5;
  const double cosYaw = std::cos(halfYaw), sinYaw = std::sin(halfYaw);
  const double cosPitch = std::cos(halfPitch), sinPitch = std::sin(halfPitch);
  const double cosRoll = std::cos(halfRoll), sinRoll = std::sin(halfRoll);
  return {sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
          cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
          cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
          cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw};
}

double getYaw(const Quaternion & q)
{
  double yaw;
  const double sqx = q.x * q.x, sqy = q.y * q.y, sqz = q.z * q.z, sqw = q.w * q.w;
  const double sarg = -2 * (q.x * q.z - q.w * q.y) / (sqx + sqy + sqz + sqw);
  if (sarg <= -0.99999) {
    yaw = -2 * std::atan2(q.y, q.x);
  } else if (sarg >= 0.99999) {
    yaw = 2 * std::atan2(q.y, q.x);
  } else {
    yaw = std::atan2(2 * (q.x * q.y + q.w * q.z), sqw + sqx - sqy - sqz);
  }
  return yaw;
}

namespace
{
// The cost upstream reads after worldToMap; outside the map upstream reads an uninitialized index, here lethal.
float cost_at(const Costmap * costmap, double x, double y)
{
  unsigned int mx, my;
  if (!costmap->worldToMap(x, y, mx, my)) return static_cast<float>(kLethalObstacle);
  return static_cast<float>(costmap->getCost(mx, my));
}
}  // namespace

Smoother::Smoother(const SmootherParams & params)
: tolerance_(params.tolerance_), data_w_(params.w_data_), smooth_w_(params.w_smooth_), max_its_(params.max_its_),
  refinement_num_(params.refinement_num_), is_holonomic_(params.holonomic_), do_refinement_(params.do_refinement_),
  state_space_(1.0)
{
}

void Smoother::initialize(const double & min_turning_radius)
{
  min_turning_rad_ = min_turning_radius;
  state_space_ = DubinsStateSpace(min_turning_rad_);
}

bool Smoother::smooth(Plan & path, const Costmap * costmap)
{
  if (max_its_ == 0) {
    return false;
  }
  bool success = true, reversing_segment;
  Plan curr_path_segment;
  std::vector<PathSegment> path_segments = findDirectionalPathSegments(path);
  for (unsigned int i = 0; i != path_segments.size(); i++) {
    if (path_segments[i].end - path_segments[i].start > 10) {
      curr_path_segment.clear();
      std::copy(path.begin() + path_segments[i].start, path.begin() + path_segments[i].end + 1,
                std::back_inserter(curr_path_segment));
      refinement_ctr_ = 0;
      const PlanPose start_pose = curr_path_segment.front();
      const PlanPose goal_pose = curr_path_segment.back();
      bool local_success = smoothImpl(curr_path_segment, reversing_segment, costmap);
      success = success && local_success;
      if (!is_holonomic_ && local_success) {
        enforceStartBoundaryConditions(start_pose, curr_path_segment, costmap, reversing_segment);
        enforceEndBoundaryConditions(goal_pose, curr_path_segment, costmap, reversing_segment);
      }
      std::copy(curr_path_segment.begin(), curr_path_segment.end(), path.begin() + path_segments[i].start);
    }
  }
  return success;
}

bool Smoother::smoothImpl(Plan & path, bool & reversing_segment, const Costmap * costmap)
{
  int its = 0;
  double change = tolerance_;
  const unsigned int & path_size = path.size();
  double x_i, y_i, y_m1, y_ip1, y_i_org;
  Plan new_path = path;
  Plan last_path = path;
  const auto get = [](const PlanPose & p, unsigned int dim) { return dim == 0 ? p.x : p.y; };
  const auto set = [](PlanPose & p, unsigned int dim, double v) { (dim == 0 ? p.x : p.y) = v; };

  while (change >= tolerance_) {
    its += 1;
    change = 0.0;
    if (its >= max_its_) {
      path = last_path;
      updateApproximatePathOrientations(path, reversing_segment);
      return false;
    }
    for (unsigned int i = 1; i != path_size - 1; i++) {
      for (unsigned int j = 0; j != 2; j++) {
        x_i = get(path[i], j);
        y_i = get(new_path[i], j);
        y_m1 = get(new_path[i - 1], j);
        y_ip1 = get(new_path[i + 1], j);
        y_i_org = y_i;
        y_i += data_w_ * (x_i - y_i) + smooth_w_ * (y_ip1 + y_m1 - (2.0 * y_i));
        set(new_path[i], j, y_i);
        change += std::abs(y_i - y_i_org);
      }
      float cost = 0.0;
      if (costmap) {
        cost = cost_at(costmap, new_path[i].x, new_path[i].y);
      }
      if (cost > MAX_NON_OBSTACLE && cost != UNKNOWN) {
        path = last_path;
        updateApproximatePathOrientations(path, reversing_segment);
        return false;
      }
    }
    last_path = new_path;
  }
  if (do_refinement_ && refinement_ctr_ < refinement_num_) {
    refinement_ctr_++;
    smoothImpl(new_path, reversing_segment, costmap);
  }
  updateApproximatePathOrientations(new_path, reversing_segment);
  path = new_path;
  return true;
}

std::vector<PathSegment> Smoother::findDirectionalPathSegments(const Plan & path)
{
  std::vector<PathSegment> segments;
  PathSegment curr_segment;
  curr_segment.start = 0;
  if (is_holonomic_) {
    curr_segment.end = path.size() - 1;
    segments.push_back(curr_segment);
    return segments;
  }
  for (unsigned int idx = 1; idx < path.size() - 1; ++idx) {
    double oa_x = path[idx].x - path[idx - 1].x;
    double oa_y = path[idx].y - path[idx - 1].y;
    double ab_x = path[idx + 1].x - path[idx].x;
    double ab_y = path[idx + 1].y - path[idx].y;
    double dot_product = (oa_x * ab_x) + (oa_y * ab_y);
    if (dot_product < 0.0) {
      curr_segment.end = idx;
      segments.push_back(curr_segment);
      curr_segment.start = idx;
    }
    double cur_theta = getYaw(path[idx].orientation);
    double next_theta = getYaw(path[idx + 1].orientation);
    double dtheta = shortest_angular_distance(cur_theta, next_theta);
    if (std::fabs(ab_x) < 1e-4 && std::fabs(ab_y) < 1e-4 && std::fabs(dtheta) > 1e-4) {
      curr_segment.end = idx;
      segments.push_back(curr_segment);
      curr_segment.start = idx;
    }
  }
  curr_segment.end = path.size() - 1;
  segments.push_back(curr_segment);
  return segments;
}

void Smoother::updateApproximatePathOrientations(Plan & path, bool & reversing_segment)
{
  double dx, dy, theta, pt_yaw;
  reversing_segment = false;
  dx = path[2].x - path[1].x;
  dy = path[2].y - path[1].y;
  theta = std::atan2(dy, dx);
  pt_yaw = getYaw(path[1].orientation);
  if (!is_holonomic_ && std::fabs(shortest_angular_distance(pt_yaw, theta)) > M_PI_2) {
    reversing_segment = true;
  }
  for (unsigned int i = 0; i != path.size() - 1; i++) {
    dx = path[i + 1].x - path[i].x;
    dy = path[i + 1].y - path[i].y;
    theta = std::atan2(dy, dx);
    if (std::fabs(dx) < 1e-4 && std::fabs(dy) < 1e-4) {
      continue;
    }
    if (reversing_segment) {
      theta += M_PI;
    }
    path[i].orientation = orientationAroundZAxis(theta);
  }
}

unsigned int Smoother::findShortestBoundaryExpansionIdx(const BoundaryExpansions & boundary_expansions)
{
  double min_length = 1e9;
  int shortest_boundary_expansion_idx = 1e9;
  for (unsigned int idx = 0; idx != boundary_expansions.size(); idx++) {
    if (boundary_expansions[idx].expansion_path_length < min_length && !boundary_expansions[idx].in_collision &&
        boundary_expansions[idx].path_end_idx > 0.0 && boundary_expansions[idx].expansion_path_length > 0.0)
    {
      min_length = boundary_expansions[idx].expansion_path_length;
      shortest_boundary_expansion_idx = idx;
    }
  }
  return shortest_boundary_expansion_idx;
}

void Smoother::findBoundaryExpansion(const PlanPose & start, const PlanPose & end, BoundaryExpansion & expansion,
                                     const Costmap * costmap)
{
  DubinsStateSpace::State from, to, s;
  from.x = start.x;
  from.y = start.y;
  from.yaw = getYaw(start.orientation);
  to.x = end.x;
  to.y = end.y;
  to.yaw = getYaw(end.orientation);

  double d = state_space_.distance(from, to);
  if (d > 2.0 * expansion.original_path_length) {
    return;
  }
  double theta(0.0), x(0.0), y(0.0);
  double x_m = start.x;
  double y_m = start.y;
  for (double i = 0; i <= expansion.path_end_idx; i++) {
    state_space_.interpolate(from, to, i / expansion.path_end_idx, s);
    theta = (s.yaw < 0.0) ? (s.yaw + 2.0 * M_PI) : s.yaw;
    theta = (theta > 2.0 * M_PI) ? (theta - 2.0 * M_PI) : theta;
    x = s.x;
    y = s.y;
    if (cost_at(costmap, x, y) >= INSCRIBED) {
      expansion.in_collision = true;
    }
    expansion.expansion_path_length += std::hypot(x - x_m, y - y_m);
    x_m = x;
    y_m = y;
    expansion.pts.emplace_back(x, y, theta);
  }
}

template<typename IteratorT>
BoundaryExpansions Smoother::generateBoundaryExpansionPoints(IteratorT start, IteratorT end)
{
  std::vector<double> distances = {min_turning_rad_, 2.0 * min_turning_rad_, M_PI * min_turning_rad_,
                                   2.0 * M_PI * min_turning_rad_};
  BoundaryExpansions boundary_expansions;
  boundary_expansions.resize(distances.size());
  double curr_dist = 0.0;
  double x_last = start->x;
  double y_last = start->y;
  unsigned int curr_dist_idx = 0;
  for (IteratorT iter = start; iter != end; iter++) {
    curr_dist += std::hypot(iter->x - x_last, iter->y - y_last);
    x_last = iter->x;
    y_last = iter->y;
    if (curr_dist >= distances[curr_dist_idx]) {
      boundary_expansions[curr_dist_idx].path_end_idx = iter - start;
      boundary_expansions[curr_dist_idx].original_path_length = curr_dist;
      curr_dist_idx++;
    }
    if (curr_dist_idx == boundary_expansions.size()) {
      break;
    }
  }
  return boundary_expansions;
}

void Smoother::enforceStartBoundaryConditions(const PlanPose & start_pose, Plan & path, const Costmap * costmap,
                                              const bool & reversing_segment)
{
  BoundaryExpansions boundary_expansions = generateBoundaryExpansionPoints(path.begin(), path.end());
  for (unsigned int i = 0; i != boundary_expansions.size(); i++) {
    BoundaryExpansion & expansion = boundary_expansions[i];
    if (expansion.path_end_idx == 0.0) {
      continue;
    }
    if (!reversing_segment) {
      findBoundaryExpansion(start_pose, path[expansion.path_end_idx], expansion, costmap);
    } else {
      findBoundaryExpansion(path[expansion.path_end_idx], start_pose, expansion, costmap);
    }
  }
  unsigned int best_expansion_idx = findShortestBoundaryExpansionIdx(boundary_expansions);
  if (best_expansion_idx > boundary_expansions.size()) {
    return;
  }
  BoundaryExpansion & best_expansion = boundary_expansions[best_expansion_idx];
  if (reversing_segment) {
    std::reverse(best_expansion.pts.begin(), best_expansion.pts.end());
  }
  for (unsigned int i = 0; i != best_expansion.pts.size(); i++) {
    path[i].x = best_expansion.pts[i].x;
    path[i].y = best_expansion.pts[i].y;
    path[i].orientation = orientationAroundZAxis(best_expansion.pts[i].theta);
  }
}

void Smoother::enforceEndBoundaryConditions(const PlanPose & end_pose, Plan & path, const Costmap * costmap,
                                            const bool & reversing_segment)
{
  BoundaryExpansions boundary_expansions = generateBoundaryExpansionPoints(path.rbegin(), path.rend());
  unsigned int expansion_starting_idx;
  for (unsigned int i = 0; i != boundary_expansions.size(); i++) {
    BoundaryExpansion & expansion = boundary_expansions[i];
    if (expansion.path_end_idx == 0.0) {
      continue;
    }
    expansion_starting_idx = path.size() - expansion.path_end_idx - 1;
    if (!reversing_segment) {
      findBoundaryExpansion(path[expansion_starting_idx], end_pose, expansion, costmap);
    } else {
      findBoundaryExpansion(end_pose, path[expansion_starting_idx], expansion, costmap);
    }
  }
  unsigned int best_expansion_idx = findShortestBoundaryExpansionIdx(boundary_expansions);
  if (best_expansion_idx > boundary_expansions.size()) {
    return;
  }
  BoundaryExpansion & best_expansion = boundary_expansions[best_expansion_idx];
  if (reversing_segment) {
    std::reverse(best_expansion.pts.begin(), best_expansion.pts.end());
  }
  expansion_starting_idx = path.size() - best_expansion.path_end_idx - 1;
  for (unsigned int i = 0; i != best_expansion.pts.size(); i++) {
    path[expansion_starting_idx + i].x = best_expansion.pts[i].x;
    path[expansion_starting_idx + i].y = best_expansion.pts[i].y;
    path[expansion_starting_idx + i].orientation = orientationAroundZAxis(best_expansion.pts[i].theta);
  }
}
}  // namespace mobile_manipulator_routing::smac
