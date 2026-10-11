#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
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

namespace
{
// Independent dense reference over z = [dx_0..dx_N, du_0..du_{N-1}]: the equality-constrained
// QP with the dynamics, dx_0 = dx0 and the rows in `active` (each row . z = rhs), solved as a
// KKT system. `lambda` returns the multipliers of the active rows (stationarity H z + g + E'l = 0).
struct ActiveRow
{
  VectorXd row;
  double rhs;
};

VectorXd dense_kkt(const QpProblem & p, const std::vector<ActiveRow> & active, VectorXd * lambda)
{
  const int n = static_cast<int>(p.stages.size()), nx = 3, nu = 2;
  const int nz = (n + 1) * nx + n * nu;
  auto xi = [&](int k) { return k * nx; };
  auto ui = [&](int k) { return (n + 1) * nx + k * nu; };
  MatrixXd h = MatrixXd::Zero(nz, nz);
  VectorXd g = VectorXd::Zero(nz);
  for (int k = 0; k < n; ++k) {
    const StageQp & s = p.stages[k];
    h.block(xi(k), xi(k), nx, nx) = s.Q;
    h.block(ui(k), ui(k), nu, nu) = s.R;
    h.block(xi(k), ui(k), nx, nu) = s.S;
    h.block(ui(k), xi(k), nu, nx) = s.S.transpose();
    g.segment(xi(k), nx) = s.q;
    g.segment(ui(k), nu) = s.r;
  }
  h.block(xi(n), xi(n), nx, nx) = p.terminal.Q;
  g.segment(xi(n), nx) = p.terminal.q;

  std::vector<ActiveRow> eq;
  for (int i = 0; i < nx; ++i) {
    VectorXd row = VectorXd::Zero(nz);
    row(xi(0) + i) = 1.0;
    eq.push_back({row, p.dx0(i)});
  }
  for (int k = 0; k < n; ++k)
    for (int i = 0; i < nx; ++i) {
      VectorXd row = VectorXd::Zero(nz);
      row(xi(k + 1) + i) = 1.0;
      row.segment(xi(k), nx) -= p.stages[k].A.row(i).transpose();
      row.segment(ui(k), nu) -= p.stages[k].B.row(i).transpose();
      eq.push_back({row, p.stages[k].c(i)});
    }
  const int dynamics_rows = static_cast<int>(eq.size());
  eq.insert(eq.end(), active.begin(), active.end());

  const int m = static_cast<int>(eq.size());
  MatrixXd kkt = MatrixXd::Zero(nz + m, nz + m);
  VectorXd rhs = VectorXd::Zero(nz + m);
  kkt.topLeftCorner(nz, nz) = h;
  rhs.head(nz) = -g;
  for (int r = 0; r < m; ++r) {
    kkt.block(nz, 0, m, nz).row(r) = eq[r].row.transpose();
    kkt.block(0, nz, nz, m).col(r) = eq[r].row;
    rhs(nz + r) = eq[r].rhs;
  }
  const VectorXd sol = kkt.partialPivLu().solve(rhs);
  if (lambda != nullptr) *lambda = sol.tail(m).tail(m - dynamics_rows);
  return sol.head(nz);
}

// The row of Cx dx_k + Cu du_k (+ Cx dx_N for the terminal stage) as a vector over z.
VectorXd constraint_row(const QpProblem & p, int k, int i)
{
  const int n = static_cast<int>(p.stages.size()), nx = 3, nu = 2;
  VectorXd row = VectorXd::Zero((n + 1) * nx + n * nu);
  const StageQp & s = k < n ? p.stages[k] : p.terminal;
  row.segment(k * nx, nx) = s.Cx.row(i).transpose();
  if (k < n) row.segment((n + 1) * nx + k * nu, nu) = s.Cu.row(i).transpose();
  return row;
}

QpProblem varied_problem()
{
  const int n = 5, nx = 3, nu = 2;
  MatrixXd a0(nx, nx), b0(nx, nu), m(nx, nx), q(nx, nx), s0(nx, nu);
  a0 << 1.0, 0.1, 0.0, 0.0, 1.0, 0.1, 0.05, 0.0, 0.95;
  b0 << 0.0, 0.1, 0.1, 0.0, 0.05, 0.2;
  m << 0.0, 0.3, -0.2, 0.1, 0.0, 0.2, -0.3, 0.1, 0.0;
  q << 2.0, 0.3, 0.0, 0.3, 1.5, -0.2, 0.0, -0.2, 3.0;
  s0 << 0.2, -0.1, 0.05, 0.15, -0.1, 0.1;
  QpProblem p;
  for (int k = 0; k < n; ++k) {
    StageQp s;
    s.Q = q + 0.1 * k * MatrixXd::Identity(nx, nx);
    s.R = MatrixXd::Zero(nu, nu);
    s.R << 1.2, 0.1, 0.1, 0.8;
    s.S = (1.0 + 0.1 * k) * s0;
    s.q = Eigen::Vector3d(0.3, -0.2, 0.1 * (k + 1));
    s.r = Vector2d(0.05 * k - 0.1, 0.2);
    s.A = a0 + 0.02 * (k + 1) * m;
    s.B = (1.0 + 0.1 * k) * b0;
    s.c = Eigen::Vector3d(0.02 * (k + 1), -0.03, 0.01 * k);
    s.Cx.resize(2, nx);
    s.Cx << 1.0, 0.5, 0.0, 0.0, 1.0, -1.0;
    s.Cu.resize(2, nu);
    s.Cu << 0.5, 1.0, -1.0, 0.3;
    s.lower = Vector2d(-100.0, -50.0);  // loose; tightened below where a row should be active
    s.upper = Vector2d(kInf, 50.0);
    s.u_lower = Vector2d(-10.0, -10.0);
    s.u_upper = Vector2d(10.0, 10.0);
    p.stages.push_back(s);
  }
  p.terminal.Q = 2.0 * q;
  p.terminal.q = Eigen::Vector3d(-0.4, 0.2, 0.1);
  p.terminal.Cx.resize(1, nx);
  p.terminal.Cx << 1.0, -1.0, 0.5;
  p.terminal.lower = VectorXd::Constant(1, -kInf);
  p.terminal.upper = VectorXd::Constant(1, 100.0);
  p.terminal.x_lower = Eigen::Vector3d(-kInf, -kInf, -kInf);
  p.terminal.x_upper = Eigen::Vector3d(kInf, kInf, kInf);
  p.dx0 = Eigen::Vector3d(0.5, -0.3, 0.2);
  return p;
}
}  // namespace

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
