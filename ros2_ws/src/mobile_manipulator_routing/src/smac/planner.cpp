// Copyright (c) 2021, Samsung Research America. Licensed under the Apache License, Version 2.0; see
// third_party/NOTICE.md. Ported from navigation2 1.3.13 nav2_smac_planner smac_planner_lattice.cpp and utils.hpp.
#include "mobile_manipulator_routing/smac/planner.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace mobile_manipulator_routing::smac
{
const char * to_string(SmacPlannerLattice::Outcome outcome)
{
  switch (outcome) {
    case SmacPlannerLattice::Outcome::Pending: return "pending";
    case SmacPlannerLattice::Outcome::Found: return "found";
    case SmacPlannerLattice::Outcome::StartOutsideMapBounds: return "start_outside_map_bounds";
    case SmacPlannerLattice::Outcome::GoalOutsideMapBounds: return "goal_outside_map_bounds";
    case SmacPlannerLattice::Outcome::StartOccupied: return "start_occupied";
    case SmacPlannerLattice::Outcome::NoValidPathCouldBeFound: return "no_valid_path";
    case SmacPlannerLattice::Outcome::PlannerTimedOut: return "exceeded_maximum_iterations";
  }
  return "unknown";
}

SmacPlannerLattice::SmacPlannerLattice(const LatticeSettings & settings, double costmap_resolution)
: settings_(settings), collision_checker_(nullptr, 72u)
{
  tolerance_ = settings.tolerance;
  int max_on_approach_iterations = settings.max_on_approach_iterations;
  max_iterations_ = settings.max_iterations;
  search_info_.lattice_filepath = settings.lattice_filepath;
  search_info_.cache_obstacle_heuristic = settings.cache_obstacle_heuristic;
  search_info_.reverse_penalty = settings.reverse_penalty;
  search_info_.change_penalty = settings.change_penalty;
  search_info_.non_straight_penalty = settings.non_straight_penalty;
  search_info_.cost_penalty = settings.cost_penalty;
  search_info_.retrospective_penalty = settings.retrospective_penalty;
  search_info_.rotation_penalty = settings.rotation_penalty;
  search_info_.analytic_expansion_ratio = settings.analytic_expansion_ratio;
  search_info_.analytic_expansion_max_cost = settings.analytic_expansion_max_cost;
  search_info_.analytic_expansion_max_cost_override = settings.analytic_expansion_max_cost_override;
  search_info_.analytic_expansion_max_length = settings.analytic_expansion_max_length_m / costmap_resolution;
  search_info_.allow_reverse_expansion = settings.allow_reverse_expansion;

  metadata_ = LatticeMotionTable::getLatticeMetadata(search_info_.lattice_filepath);
  search_info_.minimum_turning_radius = metadata_.min_turning_radius / costmap_resolution;
  if (metadata_.motion_model == "omni" || search_info_.allow_reverse_expansion) {
    throw std::invalid_argument("the port supports the forward-only, non-omni lattice of the reference configuration");
  }
  if (max_on_approach_iterations <= 0) {
    max_on_approach_iterations = std::numeric_limits<int>::max();
  }
  if (max_iterations_ <= 0) {
    max_iterations_ = std::numeric_limits<int>::max();
  }
  float lookup_table_dim = static_cast<float>(settings.lookup_table_size) / static_cast<float>(costmap_resolution);
  lookup_table_dim = static_cast<float>(static_cast<int>(lookup_table_dim));
  if (static_cast<int>(lookup_table_dim) % 2 == 0) {
    lookup_table_dim += 1.0;
  }
  a_star_ = std::make_unique<AStarAlgorithm>(search_info_, &context_);
  a_star_->initialize(settings.allow_unknown, max_iterations_, max_on_approach_iterations, lookup_table_dim,
                      metadata_.number_of_headings);
  if (settings.smooth_path) {
    smoother_ = std::make_unique<Smoother>(settings.smoother);
    smoother_->initialize(metadata_.min_turning_radius);
  }
}

SmacPlannerLattice::Outcome SmacPlannerLattice::begin(std::shared_ptr<const LayeredCostmap> costmap, double start_x,
                                                      double start_y, double start_yaw, double goal_x,
                                                      double goal_y, double goal_yaw)
{
  if (!costmap) throw std::invalid_argument("the planner needs a costmap");
  costmap_ = std::move(costmap);
  raw_plan_.clear();
  plan_.clear();
  smoothed_ = false;
  const Costmap * map = &costmap_->costmap;
  collision_checker_.setCostmap(map);
  collision_checker_.setFootprint(costmap_->padded_footprint, false, findCircumscribedCost(*costmap_));
  a_star_->setCollisionChecker(&collision_checker_);

  start_pose_ = {start_x, start_y, orientationAroundZAxis(start_yaw)};
  goal_pose_ = {goal_x, goal_y, orientationAroundZAxis(goal_yaw)};
  float mx_start, my_start, mx_goal, my_goal;
  if (!map->worldToMapContinuous(start_x, start_y, mx_start, my_start)) {
    return outcome_ = Outcome::StartOutsideMapBounds;
  }
  a_star_->setStart(mx_start, my_start,
                    context_.motion_table.getClosestAngularBin(getYaw(start_pose_.orientation)));
  if (!map->worldToMapContinuous(goal_x, goal_y, mx_goal, my_goal)) {
    return outcome_ = Outcome::GoalOutsideMapBounds;
  }
  a_star_->setGoal(mx_goal, my_goal, context_.motion_table.getClosestAngularBin(getYaw(goal_pose_.orientation)));

  if (std::floor(mx_start) == std::floor(mx_goal) && std::floor(my_start) == std::floor(my_goal)) {
    PlanPose pose{start_x, start_y, goal_pose_.orientation};
    plan_.push_back(pose);
    raw_plan_ = plan_;
    return outcome_ = Outcome::Found;
  }
  a_star_->beginPath(tolerance_ / static_cast<float>(map->getResolution()));
  return outcome_ = Outcome::Pending;
}

SmacPlannerLattice::Outcome SmacPlannerLattice::advance(int iterations)
{
  if (outcome_ != Outcome::Pending) return outcome_;
  const auto progress = a_star_->advance(iterations);
  if (progress == AStarAlgorithm::Progress::Running) return outcome_;
  if (progress == AStarAlgorithm::Progress::Failed) {
    const int num_iterations = a_star_->iterations();
    if (num_iterations == 1) return outcome_ = Outcome::StartOccupied;
    if (num_iterations < a_star_->getMaxIterations()) return outcome_ = Outcome::NoValidPathCouldBeFound;
    return outcome_ = Outcome::PlannerTimedOut;
  }
  finish_plan();
  return outcome_ = Outcome::Found;
}

void SmacPlannerLattice::finish_plan()
{
  const Costmap & map = costmap_->costmap;
  const auto & path = a_star_->path();
  plan_.reserve(path.size());
  PlanPose last_pose;  // upstream: a default pose (0, 0, yaw 0)
  for (int i = static_cast<int>(path.size()) - 1; i >= 0; --i) {
    PlanPose pose;
    pose.x = static_cast<float>(map.getOriginX()) + path[i].x * map.getResolution();  // getWorldCoords
    pose.y = static_cast<float>(map.getOriginY()) + path[i].y * map.getResolution();
    pose.orientation = quaternion_from_euler_roll(path[i].theta);  // getWorldOrientation
    if (std::fabs(pose.x - last_pose.x) < 1e-4 && std::fabs(pose.y - last_pose.y) < 1e-4 &&
        std::fabs(getYaw(pose.orientation) - getYaw(last_pose.orientation)) < 1e-4)
    {
      continue;
    }
    last_pose = pose;
    plan_.push_back(pose);
  }
  raw_plan_ = plan_;
  if (smoother_ && a_star_->iterations() > 1) {
    smoother_->smooth(plan_, &map);
    smoothed_ = true;
  }
}
}  // namespace mobile_manipulator_routing::smac
