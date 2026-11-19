// Copyright (c) 2021, Samsung Research America. Licensed under the Apache License, Version 2.0; see
// third_party/NOTICE.md.
//
// Ported from navigation2 1.3.13 (f4108e5b) nav2_smac_planner smac_planner_lattice.cpp (configure, createPlan) and
// utils.hpp (getWorldCoords, getWorldOrientation). Spec 2 Amendment 1 / ADR 0019. createPlan is split into begin()
// and advance() so that a simulation tick can run a number of iterations (A1.2 item 1); exceptions become outcomes
// (item 2); ROS messages, publishers and parameters are dropped (item 6).
#pragma once
#include <memory>
#include <string>

#include "mobile_manipulator_routing/smac/a_star.hpp"
#include "mobile_manipulator_routing/smac/smoother.hpp"

namespace mobile_manipulator_routing::smac
{
// The Lattice block of nav2_global_planning.yaml; field types are upstream's.
struct LatticeSettings
{
  std::string lattice_filepath;
  float tolerance = 0.20f;
  bool allow_unknown = false;
  int max_iterations = 1000000;
  int max_on_approach_iterations = 1000;
  double analytic_expansion_max_length_m = 3.0;
  float analytic_expansion_ratio = 3.5f;
  float analytic_expansion_max_cost = 200.0f;
  bool analytic_expansion_max_cost_override = false;
  float reverse_penalty = 2.0f;
  float change_penalty = 0.05f;
  float non_straight_penalty = 1.2f;
  float cost_penalty = 1.0f;
  float rotation_penalty = 5.0f;
  float retrospective_penalty = 0.015f;
  double lookup_table_size = 20.0;
  bool cache_obstacle_heuristic = true;
  bool allow_reverse_expansion = false;
  bool smooth_path = true;
  SmootherParams smoother{1e-10, 1000, 0.2, 0.3, false, true, 2};
};

class SmacPlannerLattice
{
public:
  // createPlan's outcomes; the names are upstream's exceptions.
  enum class Outcome
  {
    Pending, Found, StartOutsideMapBounds, GoalOutsideMapBounds, StartOccupied, NoValidPathCouldBeFound,
    PlannerTimedOut
  };
  // configure(): `costmap_resolution` is the costmap's (float-rounded) resolution, which upstream reads at configure.
  SmacPlannerLattice(const LatticeSettings & settings, double costmap_resolution);
  // createPlan up to the search loop. The costmap must outlive the plan; `start_yaw`/`goal_yaw` are converted through
  // a quaternion and back, as a PoseStamped orientation is.
  Outcome begin(std::shared_ptr<const LayeredCostmap> costmap, double start_x, double start_y, double start_yaw,
                double goal_x, double goal_y, double goal_yaw);
  // Runs up to `iterations` search iterations; Pending while the search continues.
  Outcome advance(int iterations);
  Outcome outcome() const { return outcome_; }
  int iterations() const { return a_star_->iterations(); }
  const Plan & raw_plan() const { return raw_plan_; }   // before smoothing (upstream's unsmoothed_plan)
  const Plan & plan() const { return plan_; }           // the returned plan
  bool smoothed() const { return smoothed_; }
  const LatticeMetadata & metadata() const { return metadata_; }
  std::size_t graph_size() const { return a_star_->graph_size(); }
  std::size_t queue_size() const { return a_star_->queue_size(); }

private:
  void finish_plan();
  LatticeSettings settings_;
  SearchInfo search_info_;
  LatticeMetadata metadata_;
  LatticeContext context_;
  GridCollisionChecker collision_checker_;
  std::unique_ptr<AStarAlgorithm> a_star_;
  std::unique_ptr<Smoother> smoother_;
  float tolerance_;
  int max_iterations_;
  std::shared_ptr<const LayeredCostmap> costmap_;
  Outcome outcome_ = Outcome::Pending;
  Plan raw_plan_, plan_;
  bool smoothed_ = false;
  PlanPose start_pose_, goal_pose_;
};

const char * to_string(SmacPlannerLattice::Outcome outcome);
}  // namespace mobile_manipulator_routing::smac
