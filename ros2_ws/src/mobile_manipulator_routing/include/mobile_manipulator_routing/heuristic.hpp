#pragma once
#include <memory>
#include <vector>
#include "mobile_manipulator_routing/primitives.hpp"

namespace mobile_manipulator_routing
{
struct SearchConfiguration;

// A heading-aware lower bound of the search's remaining cost (2026-11-19, user-approved after the reorientation
// measurement; it replaces Euclidean distance alone without weighting, so optimal costs are unchanged).
//
// For goal heading bin k (the primitives' heading nearest the goal yaw) a table holds, over lattice states within
// `radius_cells` of the goal's cell, the cheapest obstacle-free lattice cost to a goal-edge state (within two cells of
// the goal's cell, as the search's goal connectors) plus a lower bound of that connector's cost: the distance from
// the state's cell centre to the goal cell's square plus the rotation cost of |wrap(heading_k - state heading)|.
// Obstacles, the map's border and the soft cost only remove paths or add cost, so the table is a lower bound on any
// map of the primitives' resolution. Paths leaving the table's window are bounded by their out-and-back travel. A
// goal yaw off heading k lowers the bound by the rotation cost of the difference.
//
// Tables are built on first use (one backward Dijkstra per bin) and shared by a session's searches; calls are
// serialized by the owner.
class FreeSpaceHeuristic
{
public:
  FreeSpaceHeuristic(std::shared_ptr<const PrimitiveSet> primitives, double non_straight_penalty,
                     double rotation_cost_m_per_rad, int radius_cells = 80);
  // The bound from the lattice state at cell offset (dx, dy) from the goal's cell with heading index `heading`, for
  // goal heading bin `bin` (before the off-bin correction).
  double bound(int dx, int dy, std::size_t heading, std::size_t bin);
  bool serves(const PrimitiveSet & primitives, double non_straight_penalty, double rotation_cost_m_per_rad) const;
  std::size_t storage_bytes() const;
  int radius_cells() const { return radius_; }

private:
  const std::vector<float> & table(std::size_t bin);
  std::shared_ptr<const PrimitiveSet> primitives_;
  double penalty_, rotation_;
  int radius_;
  std::vector<double> edge_cost_;                  // each primitive's obstacle-free cost
  std::vector<std::vector<std::size_t>> ending_;  // primitives by end heading
  std::vector<std::vector<float>> tables_;        // by goal heading bin; empty until used
};

// Each primitive's obstacle-free search cost (the search's path cost with zero soft cost), from its relative poses
// with the last on the exact lattice state.
double free_space_cost(const MotionPrimitive & motion, const PrimitiveSet & primitives, double non_straight_penalty,
                       double rotation_cost_m_per_rad);
}  // namespace mobile_manipulator_routing
