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
  bool interrupted = false;  // stopped by the deadline before converging or reaching max_iter
  bool feasible = false;   // collision < slack_feasible_m, torque, base speed and hull within tolerance
  double solve_s = 0.0, linearize_s = 0.0, qp_s = 0.0, merit_s = 0.0;
  LinearizeTiming linearize_parts;  // summed over the iterations
};

// Feasible by the spec's rule: collision slack below slack_feasible_m, the torque (1e-3 of the limit), the base speed
// (1e-3 m/s) and the hull (1e-6) within their tolerances.
bool feasible(const Violations & v, const MediumConfig & config);

// The decrease of the merit the QP's model predicts for its solution `x` around a nominal with violations `at_nominal`:
// at dp = 0 each slack equals the nominal's violation of its kind, priced as the merit prices it.
double predicted_decrease(const MediumQp & lin, const Eigen::VectorXd & x, const Violations & at_nominal,
                          const MediumWeights & weights);

// From `seed`, at most config.scp.max_iter outer iterations, stopping early at `deadline` (the result so far) or, with
// `iterations_left`, when that count (decremented per iteration) reaches 0: interrupted as at the deadline.
ScpResult solve_scp(const MediumProblem & problem, DenseQpSolver & solver, Trajectory seed, const RouteTask & task,
                    const mmm::World & world, double now, std::chrono::steady_clock::time_point deadline,
                    int * iterations_left = nullptr);
}  // namespace mobile_manipulator_medium
