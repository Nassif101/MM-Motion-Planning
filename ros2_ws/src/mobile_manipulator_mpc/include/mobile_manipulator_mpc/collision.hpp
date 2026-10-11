#pragma once
// Linearized distance constraints between the robot's collision primitives (the URDF boxes
// and cylinders and the payload box) and the obstacle world, per predicted stage:
//   distance + gradient * dx >= margin,  gradient = n^T J(p),
// with n the unit normal from the obstacle to the nearest robot point p and J(p) the Jacobian
// of p with respect to the whole state (base pose and all six joints), so the escape
// directions stay consistent along the chain. Distances come from FCL (exact boxes and
// cylinders, GJK).
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "mobile_manipulator_geometry/robot_model.hpp"
#include "mobile_manipulator_mpc/model.hpp"
#include "mobile_manipulator_mpc/world.hpp"

namespace mobile_manipulator_mpc
{
struct CollisionConfig
{
  double margin_m = 0.05;
  double activation_m = 0.8;  // only pairs closer than this produce rows
  int pairs_per_stage = 8;    // closest pairs kept per stage; the rest are counted as dropped
  double floor_top_m = 0.15;  // the panel and wrist links stay above this plane
};

struct DistanceRow
{
  int stage;             // index into the nominal trajectory
  int link;              // link_index() of the robot primitive's link; link_index("panel") for the payload box
  std::string obstacle;  // obstacle name, "self/<base link>" (panel versus base), or "floor"
  double distance;       // signed: negative when penetrating
  Eigen::Matrix<double, 1, kNx> gradient;  // d distance / d x at the nominal
};

struct CollisionStats
{
  int active_pairs = 0;   // rows returned
  int dropped_pairs = 0;  // pairs within activation_m beyond the per-stage budget
  double compute_s = 0.0;
};

class CollisionModel
{
public:
  // `primitives` from mobile_manipulator_geometry::load_primitives with the payload; every
  // primitive's link must be known to the whole-body model (link_index).
  CollisionModel(const std::vector<mobile_manipulator_geometry::Primitive> & primitives, CollisionConfig config);

  // Rows for stage k = 0..N-1 of `nominal` (state at now + k * dt) against world.at(k * dt, now):
  // every robot primitive versus every obstacle box, the panel versus the base (one row: its
  // closest base primitive), and the panel and wrist links versus the floor plane at
  // floor_top_m. A primitive that only moves in the plane (base, shoulder pan) gets no row for
  // an obstacle it cannot reach in height. Per stage the pairs_per_stage closest pairs within
  // activation_m are returned, ordered by stage then distance.
  std::vector<DistanceRow> rows(const std::vector<State> & nominal, double dt, double now, const World & world,
                                CollisionStats * stats) const;

  // Minimum distance of the arm and panel primitives (everything the arm moves) to the
  // obstacles with the arm set to `joints` at each nominal base pose, capped at activation_m
  // (the tightness input).
  double clearance_with_joints(const std::vector<State> & nominal, const Eigen::Matrix<double, 6, 1> & joints,
                               double dt, double now, const World & world) const;

  const CollisionConfig & config() const;

private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
}  // namespace mobile_manipulator_mpc
