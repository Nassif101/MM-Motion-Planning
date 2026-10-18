#pragma once
// Linearized distance constraints between the robot's collision primitives (the URDF boxes
// and cylinders and the payload box) and the obstacle world, per predicted stage:
//   distance + gradient * dx >= margin,  gradient = n^T J(p),
// with n the unit normal from the obstacle to the nearest robot point p and J(p) the Jacobian
// of p with respect to the whole state (base pose and all six joints), so the escape
// directions stay consistent along the chain. Distances come from FCL (exact boxes and
// cylinders, GJK).
#include <memory>
#include <optional>
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
  // Rows of base primitives against obstacles (Phase 4B-A spec Section 2): kept outside the collision
  // monitor's wholebody StopZone (wholebody_mpc.yaml collision.base_margin_m); equal to margin_m by default.
  double base_margin_m = 0.05;
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
  int primitive = -1;    // the robot primitive (index in the model's primitives); -1: any of `link`'s
  double margin = 0.05;  // the row's margin: base_margin_m for a base primitive against an obstacle, else margin_m
};

struct CollisionStats
{
  int active_pairs = 0;   // rows returned
  int dropped_pairs = 0;  // pairs within activation_m beyond the per-stage budget
  double compute_s = 0.0;
};

// The closest robot primitive and obstacle at one state (CollisionModel::nearest).
struct NearestPair
{
  double distance;       // signed: negative when penetrating
  std::string link;      // link of the robot primitive ("panel" for the payload box)
  std::string obstacle;  // obstacle name
};

class CollisionModel
{
public:
  // `primitives` from mobile_manipulator_geometry::load_primitives with the payload; every
  // primitive's link must be known to the whole-body model (link_index). Throws
  // std::invalid_argument when a box carried by the last arm joint (the payload) is present but
  // none matches the model's "panel" frame (the payload the model was generated for).
  CollisionModel(const std::vector<mobile_manipulator_geometry::Primitive> & primitives, CollisionConfig config);

  // Rows for stage k = 0..N-1 of `nominal` (state at now + k * dt) against world.at(k * dt, now):
  // every robot primitive versus every obstacle box, the panel versus the base (one row: its
  // closest base primitive), and the panel and wrist links versus the floor plane at
  // floor_top_m. A primitive that only moves in the plane (base, shoulder pan) gets no row for
  // an obstacle it cannot reach in height. Per stage the pairs_per_stage closest pairs within
  // activation_m are returned, ordered by stage then distance.
  std::vector<DistanceRow> rows(const std::vector<State> & nominal, double dt, double now, const World & world,
                                CollisionStats * stats) const;

  // The robot primitive closest to any of `obstacles` at state `x`, with no margin and no
  // activation distance: every primitive (including the base's, whatever its height) against
  // every obstacle box, which is the true 3D distance a recording is scored with. Self distances
  // and the floor plane are not obstacles. Empty without obstacles.
  std::optional<NearestPair> nearest(const State & x, const std::vector<ObstacleBox> & obstacles) const;

  // The distance of `row`'s pair at state `x` with the obstacles where they are now
  // (world.at(0, now)): the row's robot primitive (every primitive of row.link when it has none)
  // against its obstacle, the panel against its closest base primitive for a "self/" row, the
  // lowest point of the primitive above the floor plane for a "floor" row. +infinity when the
  // obstacle is no longer in the world (e.g. a stale mover). The slack gate's current violation.
  double pair_distance(const State & x, const DistanceRow & row, const World & world, double now) const;

  // Minimum distance of the arm and panel primitives (everything the arm moves) to the
  // obstacles with the arm set to `joints` at each nominal base pose, capped at activation_m
  // (the tightness input).
  double clearance_with_joints(const std::vector<State> & nominal, const Eigen::Matrix<double, 6, 1> & joints,
                               double dt, double now, const World & world) const;

  const CollisionConfig & config() const;
  // True for a primitive that no arm joint moves (the base's), by index in the model's primitives.
  bool is_base(int primitive) const;

private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
}  // namespace mobile_manipulator_mpc
