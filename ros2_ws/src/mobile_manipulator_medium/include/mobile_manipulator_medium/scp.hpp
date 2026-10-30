#pragma once
// Sequential convex programming on the medium problem (Phase 5A spec Section 3.4): each outer iteration solves the
// problem's linearization as one dense QP inside a trust region and keeps the step only if the exact merit decreases.
#include <chrono>

#include "mobile_manipulator_medium/dense_qp.hpp"
#include "mobile_manipulator_medium/problem.hpp"

namespace mobile_manipulator_medium
{
struct ScpResult
{
  Trajectory traj;
  double merit = 0.0;
  Violations violations;
  int iterations = 0;      // outer iterations run (QPs solved or attempted)
  bool converged = false;  // the step or the merit change fell below the tolerances (or the trust region collapsed)
  bool feasible = false;   // collision < slack_feasible_m, torque, base speed and hull within tolerance
  double solve_s = 0.0, linearize_s = 0.0, qp_s = 0.0;
};

// Feasible by the spec's rule: collision slack below slack_feasible_m, the torque (1e-3 of the limit), the base speed
// (1e-3 m/s) and the hull (1e-6) within their tolerances.
bool feasible(const Violations & v, const MediumConfig & config);

// From `seed`, at most config.scp.max_iter outer iterations, stopping early at `deadline` (the result so far).
ScpResult solve_scp(const MediumProblem & problem, DenseQpSolver & solver, Trajectory seed, const RouteTask & task,
                    const mmm::World & world, double now, std::chrono::steady_clock::time_point deadline);
}  // namespace mobile_manipulator_medium
