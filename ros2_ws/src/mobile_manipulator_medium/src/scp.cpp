#include "mobile_manipulator_medium/scp.hpp"

#include <algorithm>
#include <cmath>

namespace mobile_manipulator_medium
{
namespace
{
using Clock = std::chrono::steady_clock;
double since(Clock::time_point t0) { return std::chrono::duration<double>(Clock::now() - t0).count(); }

constexpr double kAcceptRatio = 0.1;  // actual / predicted decrease needed to keep a step
constexpr double kGrowRatio = 0.75;   // ... and to grow the trust region after a step on its boundary
}  // namespace

double predicted_decrease(const MediumQp & lin, const Eigen::VectorXd & x, const Violations & at_nominal,
                          const MediumWeights & weights)
{
  return at_nominal.penalty(weights.slack, weights.slack_quadratic) - (lin.qp.g.dot(x) + 0.5 * x.dot(lin.qp.H * x));
}

bool feasible(const Violations & v, const MediumConfig & config)
{
  return v.collision_m < config.slack_feasible_m && v.torque <= 1e-3 && v.speed_mps <= 1e-3 && v.hull <= 1e-6;
}

ScpResult solve_scp(const MediumProblem & problem, DenseQpSolver & solver, Trajectory seed, const RouteTask & task,
                    const mmm::World & world, double now, Clock::time_point deadline)
{
  const auto t0 = Clock::now();
  const auto & scp = problem.config().scp;
  ScpResult r;
  r.traj = std::move(seed);
  Violations current;
  r.merit = problem.merit(r.traj, task, world, now, &current);
  double trust = scp.trust_init;
  Eigen::VectorXd warm;
  while (r.iterations < scp.max_iter && Clock::now() < deadline) {
    ++r.iterations;
    const auto tl = Clock::now();
    const MediumQp lin = problem.linearize(r.traj, task, world, now, trust);
    r.linearize_s += since(tl);
    r.linearize_parts.collision_s += lin.timing.collision_s;
    r.linearize_parts.torque_s += lin.timing.torque_s;
    r.linearize_parts.cost_s += lin.timing.cost_s;
    r.linearize_parts.assemble_s += lin.timing.assemble_s;
    r.linearize_parts.total_s += lin.timing.total_s;
    const auto tq = Clock::now();
    const DenseQpResult qp = solver.solve(lin.qp, warm.size() == lin.qp.H.rows() ? &warm : nullptr);
    r.qp_s += since(tq);
    if (!qp.ok) {
      trust *= 0.5;
      if (trust < scp.trust_min) break;
      continue;
    }
    warm = qp.x;
    const Eigen::VectorXd dp = qp.x.head(lin.parameters);
    const double predicted = predicted_decrease(lin, qp.x, current, problem.config().weights);
    const Trajectory candidate = problem.apply(r.traj, dp);
    Violations at_candidate;
    const auto tm = Clock::now();
    const double merit = problem.merit(candidate, task, world, now, &at_candidate);
    r.merit_s += since(tm);
    const double actual = r.merit - merit;
    const double step = dp.lpNorm<Eigen::Infinity>();
    if (actual > 0.0 && (predicted <= 0.0 || actual >= kAcceptRatio * predicted)) {
      r.traj = candidate;
      const double previous = r.merit;
      r.merit = merit;
      current = at_candidate;
      if (predicted > 0.0 && actual >= kGrowRatio * predicted && step >= 0.99 * trust)
        trust = std::min(2.0 * trust, scp.trust_max);
      if (step < scp.step_tol || actual < scp.merit_tol * std::max(1.0, std::abs(previous))) {
        r.converged = true;
        break;
      }
    } else {
      // No decrease: a step this small changes nothing that matters; else retry inside a smaller region.
      if (step < scp.step_tol) {
        r.converged = true;
        break;
      }
      trust = std::min(trust, step) * 0.5;
      if (trust < scp.trust_min) {
        r.converged = true;
        break;
      }
    }
  }
  r.interrupted = !r.converged && r.iterations < scp.max_iter && Clock::now() >= deadline;
  r.violations = current;
  r.feasible = feasible(r.violations, problem.config());
  r.solve_s = since(t0);
  return r;
}
}  // namespace mobile_manipulator_medium
