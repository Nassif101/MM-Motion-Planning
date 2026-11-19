// Copyright (c) 2021, Samsung Research America. Licensed under the Apache License, Version 2.0; see
// third_party/NOTICE.md.
//
// Ported from navigation2 1.3.13 (f4108e5b) nav2_smac_planner smoother.hpp/.cpp, with the tf2 quaternion handling
// (Quaternion::setEuler/setRPY, tf2::getYaw) and nav2_util::geometry_utils::orientationAroundZAxis it relies on.
// Spec 2 Amendment 1 / ADR 0019. Differences: the wall-time limit is removed (A1.2 item 1); a point outside the
// costmap, where upstream reads an uninitialized cell index, counts as lethal.
#pragma once
#include <vector>

#include "mobile_manipulator_routing/smac/costmap.hpp"
#include "mobile_manipulator_routing/smac/dubins.hpp"

namespace mobile_manipulator_routing::smac
{
struct Quaternion { double x = 0.0, y = 0.0, z = 0.0, w = 1.0; };
Quaternion quaternion_from_euler_roll(double theta);  // tf2::Quaternion::setEuler(0, 0, theta), as getWorldOrientation
Quaternion orientationAroundZAxis(double angle);      // setRPY(0, 0, angle)
double getYaw(const Quaternion & q);                  // tf2::getYaw

struct PlanPose
{
  double x = 0.0, y = 0.0;
  Quaternion orientation;
};
using Plan = std::vector<PlanPose>;

struct SmootherParams
{
  double tolerance_ = 1e-10;
  int max_its_ = 1000;
  double w_data_ = 0.2;
  double w_smooth_ = 0.3;
  bool holonomic_ = false;
  bool do_refinement_ = true;
  int refinement_num_ = 2;
};

struct PathSegment { unsigned int start; unsigned int end; };
struct BoundaryPoints
{
  BoundaryPoints(double & x_in, double & y_in, double & theta_in) : x(x_in), y(y_in), theta(theta_in) {}
  double x, y, theta;
};
struct BoundaryExpansion
{
  double path_end_idx{0.0};
  double expansion_path_length{0.0};
  double original_path_length{0.0};
  std::vector<BoundaryPoints> pts;
  bool in_collision{false};
};
using BoundaryExpansions = std::vector<BoundaryExpansion>;

class Smoother
{
public:
  explicit Smoother(const SmootherParams & params);
  void initialize(const double & min_turning_radius);
  bool smooth(Plan & path, const Costmap * costmap);

private:
  bool smoothImpl(Plan & path, bool & reversing_segment, const Costmap * costmap);
  std::vector<PathSegment> findDirectionalPathSegments(const Plan & path);
  void updateApproximatePathOrientations(Plan & path, bool & reversing_segment);
  unsigned int findShortestBoundaryExpansionIdx(const BoundaryExpansions & boundary_expansions);
  void findBoundaryExpansion(const PlanPose & start, const PlanPose & end, BoundaryExpansion & expansion,
                             const Costmap * costmap);
  template<typename IteratorT>
  BoundaryExpansions generateBoundaryExpansionPoints(IteratorT start, IteratorT end);
  void enforceStartBoundaryConditions(const PlanPose & start_pose, Plan & path, const Costmap * costmap,
                                      const bool & reversing_segment);
  void enforceEndBoundaryConditions(const PlanPose & end_pose, Plan & path, const Costmap * costmap,
                                    const bool & reversing_segment);

  double min_turning_rad_ = 0.0, tolerance_, data_w_, smooth_w_;
  int max_its_, refinement_ctr_ = 0, refinement_num_;
  bool is_holonomic_, do_refinement_;
  DubinsStateSpace state_space_;
};
}  // namespace mobile_manipulator_routing::smac
