// The QP backends on the same stage-wise problems (Phase 4B-B spec Section 4.2): OSQP, HPIPM and HPIPM with partial
// condensing must give the same answers, slacked rows and per-row slack weights included.
#include <gtest/gtest.h>

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "mobile_manipulator_mpc/hpipm_solver.hpp"
#include "mobile_manipulator_mpc/osqp_solver.hpp"
#include "qp_problems.hpp"

using namespace mobile_manipulator_mpc;
using namespace qp_problems;

namespace
{
OsqpOptions osqp_tight()
{
  OsqpOptions o;
  o.eps_abs = o.eps_rel = 1e-9;
  o.max_iter = 50000;
  return o;
}

HpipmOptions hpipm_tight(int blocks)
{
  HpipmOptions o;
  o.tol = 1e-9;
  o.max_iter = 200;
  o.partial_condensing_blocks = blocks;
  return o;
}

struct Backend
{
  std::string name;
  std::function<std::unique_ptr<QpSolver>()> make;
};

std::ostream & operator<<(std::ostream & os, const Backend & b) { return os << b.name; }

SolveResult solve_with(const Backend & b, const QpProblem & p)
{
  auto solver = b.make();
  solver->setup(p);
  return solver->solve(p, nullptr);
}

SolveResult reference(const QpProblem & p)
{
  OsqpSolver s(osqp_tight());
  s.setup(p);
  return s.solve(p, nullptr);
}

void expect_same(const SolveResult & a, const SolveResult & b, double tol)
{
  ASSERT_TRUE(a.ok) << a.status;
  ASSERT_TRUE(b.ok) << b.status;
  ASSERT_EQ(a.solution.dx.size(), b.solution.dx.size());
  ASSERT_EQ(a.solution.du.size(), b.solution.du.size());
  for (size_t k = 0; k < a.solution.dx.size(); ++k)
    EXPECT_LT((a.solution.dx[k] - b.solution.dx[k]).cwiseAbs().maxCoeff(), tol) << "dx " << k;
  for (size_t k = 0; k < a.solution.du.size(); ++k)
    EXPECT_LT((a.solution.du[k] - b.solution.du[k]).cwiseAbs().maxCoeff(), tol) << "du " << k;
  ASSERT_EQ(a.solution.slack.size(), b.solution.slack.size());
  if (a.solution.slack.size()) EXPECT_LT((a.solution.slack - b.solution.slack).cwiseAbs().maxCoeff(), tol);
}

class Backends : public ::testing::TestWithParam<Backend>
{
};

INSTANTIATE_TEST_SUITE_P(
  All, Backends,
  ::testing::Values(Backend{"osqp", [] { return std::make_unique<OsqpSolver>(osqp_tight()); }},
                    Backend{"hpipm", [] { return std::make_unique<HpipmSolver>(hpipm_tight(0)); }},
                    Backend{"hpipm_pcond4", [] { return std::make_unique<HpipmSolver>(hpipm_tight(4)); }}),
  [](const ::testing::TestParamInfo<Backend> & info) { return info.param.name; });
}  // namespace

TEST_P(Backends, AgreeOnTheLqr)
{
  const QpProblem p = double_integrator(10, Vector2d(1.0, -0.3));
  expect_same(solve_with(GetParam(), p), reference(p), 1e-5);
}

TEST_P(Backends, RespectBoxBounds)
{
  const QpProblem p = constrained_problem(Vector2d(3.0, 0.0));
  const SolveResult r = solve_with(GetParam(), p);
  ASSERT_TRUE(r.ok) << r.status;
  for (const auto & u : r.solution.du) EXPECT_LE(std::abs(u(0)), 0.8 + 1e-6);
  for (size_t k = 1; k < r.solution.dx.size(); ++k) EXPECT_LE(std::abs(r.solution.dx[k](1)), 1.0 + 1e-6);
  EXPECT_LT((r.solution.dx[0] - Vector2d(3.0, 0.0)).norm(), 1e-9) << "dx0 is fixed";
  expect_same(r, reference(p), 1e-5);
}

TEST_P(Backends, SlackActivatesOnlyWhenInfeasible)
{
  const SolveResult feasible = solve_with(GetParam(), with_slacked_position_row(0.02));
  ASSERT_TRUE(feasible.ok) << feasible.status;
  ASSERT_EQ(feasible.solution.slack.size(), 1);
  EXPECT_LT(feasible.solution.slack(0), 1e-6);
  EXPECT_GE(feasible.solution.dx[3](0), 0.02 - 1e-6);
  const SolveResult infeasible = solve_with(GetParam(), with_slacked_position_row(0.5));
  ASSERT_TRUE(infeasible.ok) << infeasible.status;
  EXPECT_NEAR(infeasible.solution.dx[3](0) + infeasible.solution.slack(0), 0.5, 1e-5);
  EXPECT_NEAR(infeasible.solution.dx[3](0), 0.045, 1e-4);
}

TEST_P(Backends, QuadraticSlackKeepsTheExactPenaltySolution)
{
  QpProblem p = with_slacked_position_row(0.5);
  p.slack_quadratic = 10.0;
  const SolveResult r = solve_with(GetParam(), p);
  ASSERT_TRUE(r.ok) << r.status;
  EXPECT_NEAR(r.solution.dx[3](0), 0.045, 1e-4);
}

TEST_P(Backends, SlackRelaxesTheFiniteSideOfAnUpperBoundedRow)
{
  QpProblem p = with_slacked_position_row(0.0);
  p.stages[3].lower(0) = -kInf;
  p.stages[3].upper(0) = -0.5;
  const SolveResult r = solve_with(GetParam(), p);
  ASSERT_TRUE(r.ok) << r.status;
  EXPECT_NEAR(r.solution.dx[3](0) - r.solution.slack(0), -0.5, 1e-5);
  EXPECT_NEAR(r.solution.dx[3](0), -0.045, 1e-4);
}

TEST_P(Backends, PerRowSlackWeightsMatchOsqp)
{
  // Two unreachable rows pulling the same stage-3 position apart: a cheap lower bound at 0.5 and a dear upper bound at
  // -0.5 (on stage 3's second row). The weights decide which slack takes the shortfall.
  QpProblem p = with_slacked_position_row(0.5);
  for (auto & s : p.stages) {
    s.Cx.conservativeResize(2, 2);
    s.Cu.conservativeResize(2, 1);
    s.Cx.row(1).setZero();
    s.Cu.row(1).setZero();
    s.lower.conservativeResize(2);
    s.upper.conservativeResize(2);
    s.lower(1) = -kInf;
    s.upper(1) = kInf;
  }
  // Rows are numbered stage by stage: stage k's rows are 2k and 2k + 1.
  p.stages[3].Cx.row(1) << 1.0, 0.0;
  p.stages[3].upper(1) = -0.5;
  p.slack_count = 2;
  p.slack_rows = {6, 7};
  p.slack_weights = {50.0, 150.0};
  p.slack_quadratic = 10.0;
  const SolveResult r = solve_with(GetParam(), p);
  expect_same(r, reference(p), 1e-4);
  EXPECT_GT(r.solution.slack(0), r.solution.slack(1)) << "the cheaper slack gives more";
}

TEST_P(Backends, MatchTheDenseKktWithCrossTermsOffsetsAndTerminalRows)
{
  QpProblem p = varied_problem();
  const int n = static_cast<int>(p.stages.size()), nx = 3;
  const VectorXd free_z = dense_kkt(p, {}, nullptr);
  auto row_value = [&](int k, int i) { return constraint_row(p, k, i).dot(free_z); };
  p.stages[2].lower(0) = row_value(2, 0) + 0.5;
  p.stages[3].upper(1) = row_value(3, 1) + 5.0;
  p.terminal.upper(0) = row_value(n, 0) - 0.1;
  const int x2 = n * nx + 2;
  p.terminal.x_lower(2) = free_z(x2) + 0.05;
  std::vector<ActiveRow> active;
  active.push_back({constraint_row(p, 2, 0), p.stages[2].lower(0)});
  active.push_back({constraint_row(p, n, 0), p.terminal.upper(0)});
  VectorXd unit = VectorXd::Zero(free_z.size());
  unit(x2) = 1.0;
  active.push_back({unit, p.terminal.x_lower(2)});
  const VectorXd z = dense_kkt(p, active, nullptr);
  const SolveResult r = solve_with(GetParam(), p);
  ASSERT_TRUE(r.ok) << r.status;
  VectorXd stacked(z.size());
  for (int k = 0; k <= n; ++k) stacked.segment(k * nx, nx) = r.solution.dx[k];
  for (int k = 0; k < n; ++k) stacked.segment((n + 1) * nx + k * 2, 2) = r.solution.du[k];
  EXPECT_LT((stacked - z).cwiseAbs().maxCoeff(), 1e-4);
}

TEST_P(Backends, HandleStagesWithoutRows)
{
  // Rows on some stages only, none on the terminal stage, a row with both bounds infinite, and a stage with no
  // boxes at all.
  QpProblem p = varied_problem();
  p.stages[1].Cx.resize(0, 3);
  p.stages[1].Cu.resize(0, 2);
  p.stages[1].lower.resize(0);
  p.stages[1].upper.resize(0);
  p.stages[1].u_lower.resize(0);
  p.stages[1].u_upper.resize(0);
  p.stages[2].lower(1) = -kInf;
  p.stages[2].upper(1) = kInf;
  p.terminal.Cx.resize(0, 3);
  p.terminal.lower.resize(0);
  p.terminal.upper.resize(0);
  expect_same(solve_with(GetParam(), p), reference(p), 1e-4);
}

TEST_P(Backends, SamePatternUpdatesWithoutSetup)
{
  auto solver = GetParam().make();
  const QpProblem first = constrained_problem(Vector2d(3.0, 0.0));
  solver->setup(first);
  ASSERT_TRUE(solver->solve(first, nullptr).ok);
  QpProblem second = constrained_problem(Vector2d(-1.0, 0.3));
  for (auto & s : second.stages) {
    s.Q(0, 0) = 5.0;
    s.u_upper(0) = 0.3;
    s.q = Vector2d(0.1, -0.1);
  }
  expect_same(solver->solve(second, nullptr), reference(second), 1e-5);
  EXPECT_THROW(solver->solve(double_integrator(7, Vector2d::Zero()), nullptr), std::invalid_argument);
}

TEST_P(Backends, NumericalFailureIsReportedNotThrown)
{
  QpProblem p = double_integrator(5, Vector2d::Zero());
  for (auto & s : p.stages) {
    s.u_lower = VectorXd::Constant(1, -0.1);
    s.u_upper = VectorXd::Constant(1, 0.1);
  }
  p.stages[4].x_lower = Vector2d(5.0, -kInf);  // unreachable, and no slack
  p.stages[4].x_upper = Vector2d(kInf, kInf);
  SolveResult r;
  EXPECT_NO_THROW(r = solve_with(GetParam(), p));
  EXPECT_FALSE(r.ok);
  EXPECT_FALSE(r.status.empty());
}

TEST(Hpipm, PartialCondensingEqualsNone)
{
  const QpProblem p = constrained_problem(Vector2d(3.0, 0.0));
  for (int blocks : {2, 4, 5}) {
    HpipmSolver none(hpipm_tight(0)), condensed(hpipm_tight(blocks));
    none.setup(p);
    condensed.setup(p);
    expect_same(condensed.solve(p, nullptr), none.solve(p, nullptr), 1e-6);
  }
}

TEST(Hpipm, WarmStartFromItsOwnSolution)
{
  HpipmOptions o = hpipm_tight(0);
  o.warm_start = true;
  HpipmSolver s(o);
  const QpProblem first = constrained_problem(Vector2d(3.0, 0.0));
  s.setup(first);
  const SolveResult previous = s.solve(first, nullptr);
  ASSERT_TRUE(previous.ok);
  const QpProblem next = constrained_problem(Vector2d(3.0, 0.02));
  expect_same(s.solve(next, &previous.solution), reference(next), 1e-5);
}
