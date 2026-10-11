#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>

#include "mobile_manipulator_mpc/osqp_solver.hpp"

using namespace mobile_manipulator_mpc;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;

namespace
{
constexpr double kDt = 0.1;
const double kInf = std::numeric_limits<double>::infinity();

MatrixXd dynamics_a()
{
  MatrixXd a(2, 2);
  a << 1.0, kDt, 0.0, 1.0;
  return a;
}

MatrixXd dynamics_b()
{
  MatrixXd b(2, 1);
  b << 0.5 * kDt * kDt, kDt;
  return b;
}

// Double integrator: cost sum 0.5 (dx'Q dx + du'R du) plus 0.5 dx_N'Qf dx_N, from `x0`.
QpProblem double_integrator(int horizon, const Vector2d & x0)
{
  QpProblem p;
  StageQp s;
  s.Q = Eigen::DiagonalMatrix<double, 2>(2.0, 0.5);
  s.R = MatrixXd::Constant(1, 1, 0.1);
  s.A = dynamics_a();
  s.B = dynamics_b();
  s.c = VectorXd::Zero(2);
  p.stages.assign(horizon, s);
  p.terminal.Q = 4.0 * MatrixXd::Identity(2, 2);
  p.dx0 = x0;
  return p;
}

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

namespace
{
// Position at stage 3 must be at least `required`; the controls are limited to +-1, so the
// largest reachable position there is 0.5 * (3 dt)^2 = 0.045 (for a start at rest).
QpProblem with_slacked_position_row(double required)
{
  QpProblem p = double_integrator(8, Vector2d::Zero());
  for (auto & s : p.stages) {
    s.u_lower = VectorXd::Constant(1, -1.0);
    s.u_upper = VectorXd::Constant(1, 1.0);
  }
  // Rows are numbered across the stages: one row per stage, so stage 3 is row 3.
  for (std::size_t k = 0; k < p.stages.size(); ++k) {
    StageQp & s = p.stages[k];
    s.Cx = MatrixXd::Zero(1, 2);
    s.Cu = MatrixXd::Zero(1, 1);
    s.lower = VectorXd::Constant(1, -kInf);
    s.upper = VectorXd::Constant(1, kInf);
  }
  p.stages[3].Cx << 1.0, 0.0;
  p.stages[3].lower(0) = required;
  p.slack_count = 1;
  p.slack_rows = {3};
  p.slack_weight = 100.0;  // OSQP gets slow with a penalty far above the cost scale
  return p;
}
}  // namespace

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

namespace
{
QpProblem constrained_problem(const Vector2d & x0)
{
  QpProblem p = double_integrator(20, x0);
  for (auto & s : p.stages) {
    s.u_lower = VectorXd::Constant(1, -0.8);
    s.u_upper = VectorXd::Constant(1, 0.8);
    s.x_lower = Vector2d(-kInf, -1.0);
    s.x_upper = Vector2d(kInf, 1.0);
  }
  return p;
}
}  // namespace

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
