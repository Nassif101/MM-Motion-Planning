#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>

#include "mobile_manipulator_mpc/osqp_solver.hpp"
#include "qp_problems.hpp"

using namespace mobile_manipulator_mpc;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;
using namespace qp_problems;

namespace
{

OsqpOptions tight()
{
  OsqpOptions o;
  o.eps_abs = o.eps_rel = 1e-9;
  o.max_iter = 20000;
  return o;
}

SolveResult solve_fresh(const QpProblem & p, const OsqpOptions & o = tight())
{
  OsqpSolver solver(o);
  solver.setup(p);
  return solver.solve(p, nullptr);
}
}  // namespace

TEST(Osqp, SolvesSmallLqrToRiccatiSolution)
{
  const int n = 10;
  const Vector2d x0(1.0, -0.3);
  const QpProblem p = double_integrator(n, x0);
  const SolveResult r = solve_fresh(p);
  ASSERT_TRUE(r.ok) << r.status;

  const MatrixXd a = dynamics_a(), b = dynamics_b(), q = p.stages[0].Q, rr = p.stages[0].R;
  std::vector<MatrixXd> gain(n);
  MatrixXd pk = p.terminal.Q;
  for (int k = n - 1; k >= 0; --k) {
    gain[k] = (rr + b.transpose() * pk * b).inverse() * b.transpose() * pk * a;
    pk = q + a.transpose() * pk * (a - b * gain[k]);
  }
  VectorXd x = x0;
  ASSERT_EQ(r.solution.dx.size(), static_cast<std::size_t>(n + 1));
  ASSERT_EQ(r.solution.du.size(), static_cast<std::size_t>(n));
  for (int k = 0; k < n; ++k) {
    const VectorXd u = -gain[k] * x;
    EXPECT_LT((r.solution.dx[k] - x).norm(), 1e-4) << "k=" << k;
    EXPECT_LT((r.solution.du[k] - u).norm(), 1e-4) << "k=" << k;
    x = a * x + b * u;
  }
  EXPECT_LT((r.solution.dx[n] - x).norm(), 1e-4);
  EXPECT_EQ(r.solution.slack.size(), 0);
  EXPECT_GT(r.solve_s, 0.0);
  EXPECT_GT(r.iterations, 0);
}

TEST(Osqp, RespectsBoxBounds)
{
  QpProblem p = double_integrator(15, Vector2d(2.0, 0.0));
  for (auto & s : p.stages) {
    s.u_lower = VectorXd::Constant(1, -0.5);
    s.u_upper = VectorXd::Constant(1, 0.5);
    s.x_lower = Vector2d(-kInf, -0.4);
    s.x_upper = Vector2d(kInf, 0.4);
  }
  const SolveResult r = solve_fresh(p);
  ASSERT_TRUE(r.ok) << r.status;
  double u_max = 0.0, v_min = 0.0;
  for (const auto & u : r.solution.du) {
    EXPECT_LE(std::abs(u(0)), 0.5 + 1e-6);
    u_max = std::max(u_max, std::abs(u(0)));
  }
  for (std::size_t k = 1; k < r.solution.dx.size(); ++k) {
    EXPECT_GE(r.solution.dx[k](1), -0.4 - 1e-6);
    EXPECT_LE(r.solution.dx[k](1), 0.4 + 1e-6);
    v_min = std::min(v_min, r.solution.dx[k](1));
  }
  EXPECT_NEAR(u_max, 0.5, 1e-6);  // the unconstrained answer would use more, so the bound is active
  EXPECT_LT((r.solution.dx[0] - Vector2d(2.0, 0.0)).norm(), 1e-9);
  EXPECT_LT(v_min, 0.0);
}

TEST(Osqp, SlackActivatesOnlyWhenInfeasible)
{
  OsqpOptions options;  // the L1 penalty makes the problem stiffer: more iterations
  options.eps_abs = options.eps_rel = 1e-6;
  options.max_iter = 20000;
  const SolveResult feasible = solve_fresh(with_slacked_position_row(0.02), options);
  ASSERT_TRUE(feasible.ok) << feasible.status;
  ASSERT_EQ(feasible.solution.slack.size(), 1);
  EXPECT_LT(feasible.solution.slack(0), 1e-6);
  EXPECT_GE(feasible.solution.dx[3](0), 0.02 - 1e-6);

  const SolveResult infeasible = solve_fresh(with_slacked_position_row(0.5), options);
  ASSERT_TRUE(infeasible.ok) << infeasible.status;
  // The position is pushed as far as the control limits allow and the slack takes the rest.
  EXPECT_GT(infeasible.solution.slack(0), 0.4);
  EXPECT_NEAR(infeasible.solution.dx[3](0) + infeasible.solution.slack(0), 0.5, 1e-5);
  EXPECT_NEAR(infeasible.solution.dx[3](0), 0.045, 1e-4);
}

TEST(Osqp, QuadraticSlackWeightKeepsTheExactPenaltySolution)
{
  // With the L1 weight above the multiplier, a quadratic slack term changes nothing at the optimum
  // (the position still goes to the control-limit reach) but makes the slack-active problem
  // strongly convex, so OSQP converges in fewer iterations.
  OsqpOptions options;
  options.eps_abs = options.eps_rel = 1e-6;
  options.max_iter = 20000;
  const SolveResult l1 = solve_fresh(with_slacked_position_row(0.5), options);
  QpProblem p = with_slacked_position_row(0.5);
  p.slack_quadratic = 10.0;
  const SolveResult both = solve_fresh(p, options);
  ASSERT_TRUE(l1.ok) << l1.status;
  ASSERT_TRUE(both.ok) << both.status;
  EXPECT_NEAR(both.solution.dx[3](0), 0.045, 1e-4);
  EXPECT_NEAR(both.solution.slack(0), l1.solution.slack(0), 1e-4);
  std::printf("iterations: L1 only %d, with the quadratic term %d\n", l1.iterations, both.iterations);
  EXPECT_LT(both.iterations, l1.iterations);
}

TEST(Osqp, SlackRelaxesTheFiniteSideOfAnUpperBoundedRow)
{
  QpProblem p = with_slacked_position_row(0.0);
  p.stages[3].lower(0) = -kInf;
  p.stages[3].upper(0) = -0.5;  // unreachable: the position can get to -0.045 at best
  OsqpOptions options;
  options.eps_abs = options.eps_rel = 1e-6;
  options.max_iter = 20000;
  const SolveResult r = solve_fresh(p, options);
  ASSERT_TRUE(r.ok) << r.status;
  EXPECT_NEAR(r.solution.dx[3](0) - r.solution.slack(0), -0.5, 1e-5);
  EXPECT_NEAR(r.solution.dx[3](0), -0.045, 1e-4);
}

TEST(Osqp, WarmStartReducesIterations)
{
  OsqpOptions options;
  options.max_iter = 4000;
  options.eps_abs = options.eps_rel = 1e-6;
  OsqpSolver solver(options);
  const QpProblem first = constrained_problem(Vector2d(3.0, 0.0));
  solver.setup(first);
  const SolveResult previous = solver.solve(first, nullptr);
  ASSERT_TRUE(previous.ok) << previous.status;

  const QpProblem next = constrained_problem(Vector2d(3.0, 0.02));  // a slightly moved start
  const SolveResult cold = solver.solve(next, nullptr);
  const SolveResult warm = solver.solve(next, &previous.solution);
  ASSERT_TRUE(cold.ok) << cold.status;
  ASSERT_TRUE(warm.ok) << warm.status;
  EXPECT_LT(warm.iterations, cold.iterations);
  EXPECT_LT((warm.solution.primal - cold.solution.primal).norm(), 1e-3);
}

TEST(Osqp, SamePatternUpdatesWithoutSetup)
{
  OsqpSolver solver(tight());
  const QpProblem first = constrained_problem(Vector2d(3.0, 0.0));
  solver.setup(first);
  EXPECT_EQ(solver.setup_count(), 1);
  ASSERT_TRUE(solver.solve(first, nullptr).ok);

  QpProblem second = constrained_problem(Vector2d(-1.0, 0.3));
  for (auto & s : second.stages) {
    s.Q(0, 0) = 5.0;
    s.u_upper(0) = 0.3;
    s.q = Vector2d(0.1, -0.1);
  }
  const SolveResult updated = solver.solve(second, nullptr);
  EXPECT_EQ(solver.setup_count(), 1);
  ASSERT_TRUE(updated.ok) << updated.status;
  const SolveResult reference = solve_fresh(second);
  ASSERT_TRUE(reference.ok);
  EXPECT_LT((updated.solution.primal - reference.solution.primal).norm(), 1e-5);

  // A different shape needs a new setup.
  EXPECT_THROW(solver.solve(double_integrator(7, Vector2d::Zero()), nullptr), std::invalid_argument);
  solver.setup(double_integrator(7, Vector2d::Zero()));
  EXPECT_EQ(solver.setup_count(), 2);
}

TEST(Osqp, NumericalFailureIsReportedNotThrown)
{
  QpProblem p = double_integrator(5, Vector2d::Zero());
  for (auto & s : p.stages) {
    s.u_lower = VectorXd::Constant(1, -0.1);
    s.u_upper = VectorXd::Constant(1, 0.1);
  }
  p.stages[4].x_lower = Vector2d(5.0, -kInf);  // unreachable, and no slack
  p.stages[4].x_upper = Vector2d(kInf, kInf);
  SolveResult r;
  EXPECT_NO_THROW(r = solve_fresh(p));
  EXPECT_FALSE(r.ok);
  EXPECT_FALSE(r.status.empty());
  EXPECT_EQ(r.solution.primal.size(), 0);

  p.stages[2].x_lower = Vector2d(1.0, -kInf);
  p.stages[2].x_upper = Vector2d(0.0, kInf);  // lower > upper
  EXPECT_NO_THROW(r = solve_fresh(p));
  EXPECT_FALSE(r.ok);
}

TEST(Osqp, MatchesDenseKktWithOffsetsCrossTermsAndTerminalRows)
{
  QpProblem p = varied_problem();
  const int n = static_cast<int>(p.stages.size()), nx = 3, nu = 2;

  // Unconstrained optimum, to place the bounds that should become active just past it.
  const VectorXd free_z = dense_kkt(p, {}, nullptr);
  auto row_value = [&](int k, int i) { return constraint_row(p, k, i).dot(free_z); };
  p.stages[2].lower(0) = row_value(2, 0) + 0.5;          // Cx, Cu row active at its lower bound
  p.stages[3].upper(1) = row_value(3, 1) + 5.0;          // two-sided row, inactive
  p.terminal.upper(0) = row_value(n, 0) - 0.1;           // terminal inequality row active
  const int x2 = n * nx + 2;                              // terminal x_N component 2
  p.terminal.x_lower(2) = free_z(x2) + 0.05;             // terminal x bound, active

  std::vector<ActiveRow> active;
  active.push_back({constraint_row(p, 2, 0), p.stages[2].lower(0)});
  active.push_back({constraint_row(p, n, 0), p.terminal.upper(0)});
  VectorXd unit = VectorXd::Zero(free_z.size());
  unit(x2) = 1.0;
  active.push_back({unit, p.terminal.x_lower(2)});
  VectorXd lambda;
  const VectorXd z = dense_kkt(p, active, &lambda);

  // The assumed active set is the true one: multipliers point the right way (lower-bounded rows
  // have l <= 0, upper-bounded >= 0) and every other bound holds.
  ASSERT_EQ(lambda.size(), 3);
  EXPECT_LT(lambda(0), -1e-6);
  EXPECT_GT(lambda(1), 1e-6);
  EXPECT_LT(lambda(2), -1e-6);
  for (int k = 0; k < n; ++k)
    for (int i = 0; i < 2; ++i) {
      const double v = constraint_row(p, k, i).dot(z);
      if (!(k == 2 && i == 0)) EXPECT_GT(v, p.stages[k].lower(i) + 1e-3) << k << "," << i;
      EXPECT_LT(v, p.stages[k].upper(i) - 1e-3) << k << "," << i;
    }
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < nu; ++j) EXPECT_LT(std::abs(z((n + 1) * nx + k * nu + j)), 10.0 - 1e-3);

  const SolveResult r = solve_fresh(p);
  ASSERT_TRUE(r.ok) << r.status;
  VectorXd stacked(z.size());
  for (int k = 0; k <= n; ++k) stacked.segment(k * nx, nx) = r.solution.dx[k];
  for (int k = 0; k < n; ++k) stacked.segment((n + 1) * nx + k * nu, nu) = r.solution.du[k];
  EXPECT_LT((stacked - z).cwiseAbs().maxCoeff(), 1e-4);
  // The free solution differs from the constrained one, so the active rows do matter.
  EXPECT_GT((free_z - z).cwiseAbs().maxCoeff(), 0.05);
}

TEST(Osqp, TimeLimitStopsTheSolve)
{
  // Phase 4B-B deadline cap. A limit set before setup applies to the first solve; on a later solve (OSQP counts the
  // update and the solve) a microsecond ends a cold tight solve as "time_limit"; 0 removes the limit again.
  const QpProblem p = double_integrator(30, Vector2d(1.0, -0.3));
  OsqpSolver early(tight());
  early.set_time_limit(1e-6);
  early.setup(p);
  EXPECT_EQ(early.solve(p, nullptr).status, "time_limit");

  OsqpSolver s(tight());
  s.setup(p);
  ASSERT_TRUE(s.solve(p, nullptr).ok);
  s.set_time_limit(1e-6);
  const SolveResult limited = s.solve(p, nullptr);
  EXPECT_FALSE(limited.ok);
  EXPECT_EQ(limited.status, "time_limit");
  s.set_time_limit(0.0);
  EXPECT_TRUE(s.solve(p, nullptr).ok);
}
