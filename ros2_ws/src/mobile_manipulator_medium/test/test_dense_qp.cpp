// The medium layer's dense-QP backends (Phase 5A Task 6, spec Section 6.1): HPIPM dense, OSQP and DAQP solve the same
// problems to the same answer.
#include <gtest/gtest.h>

#include <limits>
#include <random>
#include <string>

#include <Eigen/Cholesky>

#include "mobile_manipulator_medium/dense_qp.hpp"

namespace
{
namespace mmd = mobile_manipulator_medium;
constexpr double kInf = std::numeric_limits<double>::infinity();
const char * kBackends[] = {"hpipm", "osqp", "daqp"};

// A strictly convex QP with `n` variables and `m` general rows around a strictly feasible point: half the rows
// one-sided, a few two-sided, a third of the variables boxed. Rows with a small margin to the point are likely active.
mmd::DenseQp random_qp(int n, int m, std::mt19937 & rng)
{
  std::normal_distribution<double> g(0.0, 1.0);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  mmd::DenseQp qp;
  Eigen::MatrixXd L(n, n);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) L(i, j) = g(rng);
  qp.H = L * L.transpose() / n + 0.1 * Eigen::MatrixXd::Identity(n, n);
  qp.g.resize(n);
  for (int i = 0; i < n; ++i) qp.g(i) = 3.0 * g(rng);
  Eigen::VectorXd x0(n);
  for (int i = 0; i < n; ++i) x0(i) = g(rng);
  qp.A.resize(m, n);
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < n; ++j) qp.A(i, j) = g(rng);
  const Eigen::VectorXd Ax = qp.A * x0;
  qp.lA = Eigen::VectorXd::Constant(m, -kInf);
  qp.uA = Eigen::VectorXd::Constant(m, kInf);
  for (int i = 0; i < m; ++i) {
    const double margin = 0.05 + u(rng);
    if (i % 2 == 0) qp.uA(i) = Ax(i) + margin;
    else if (i % 7 == 0) {
      qp.lA(i) = Ax(i) - margin;
      qp.uA(i) = Ax(i) + margin;
    } else {
      qp.lA(i) = Ax(i) - margin;
    }
  }
  qp.lb = Eigen::VectorXd::Constant(n, -kInf);
  qp.ub = Eigen::VectorXd::Constant(n, kInf);
  for (int j = 0; j < n; j += 3) {
    qp.lb(j) = x0(j) - 0.3;
    qp.ub(j) = x0(j) + 0.3;
  }
  return qp;
}

double violation(const mmd::DenseQp & qp, const Eigen::VectorXd & x)
{
  const Eigen::VectorXd Ax = qp.A * x;
  double v = 0.0;
  for (int i = 0; i < Ax.size(); ++i) v = std::max({v, qp.lA(i) - Ax(i), Ax(i) - qp.uA(i)});
  for (int j = 0; j < x.size(); ++j) v = std::max({v, qp.lb(j) - x(j), x(j) - qp.ub(j)});
  return v;
}

double objective(const mmd::DenseQp & qp, const Eigen::VectorXd & x) { return 0.5 * x.dot(qp.H * x) + qp.g.dot(x); }

// The same answer within each method's accuracy (Task 6 ruling): the interior point (HPIPM) and ADMM (OSQP) stop within
// their tolerances where the active-set DAQP lands on the vertex, so the solutions are compared by feasibility
// (1e-5, metres on a collision row), objective (1e-6 relative to the best: OSQP's eps is 1e-7) and distance (1e-4).
TEST(DenseQp, BackendsAgreeOnRandomQps)
{
  std::mt19937 rng(11);
  for (int trial = 0; trial < 100; ++trial) {
    const mmd::DenseQp qp = random_qp(35, 300, rng);
    std::vector<Eigen::VectorXd> solutions;
    double best = std::numeric_limits<double>::infinity();
    for (const char * name : kBackends) {
      const auto solver = mmd::make_dense_solver(name);
      const mmd::DenseQpResult r = solver->solve(qp, nullptr);
      ASSERT_TRUE(r.ok) << name << " trial " << trial << ": " << r.status;
      EXPECT_LT(violation(qp, r.x), 1e-5) << name << " trial " << trial;
      EXPECT_GE(r.solve_s, 0.0);
      solutions.push_back(r.x);
      best = std::min(best, objective(qp, r.x));
    }
    for (size_t b = 0; b < solutions.size(); ++b) {
      EXPECT_LE(objective(qp, solutions[b]) - best, 1e-6 * std::max(1.0, std::abs(best))) << kBackends[b] << " trial " << trial;
      EXPECT_LT((solutions[b] - solutions[0]).lpNorm<Eigen::Infinity>(), 1e-4) << kBackends[b] << " vs hpipm, trial " << trial;
    }
  }
}

TEST(DenseQp, InfiniteBoundsAreFree)
{
  // No finite bound anywhere: the unconstrained minimizer -H^-1 g, with or without general rows.
  std::mt19937 rng(12);
  mmd::DenseQp qp = random_qp(10, 4, rng);
  qp.lA.setConstant(-kInf);
  qp.uA.setConstant(kInf);
  qp.lb.setConstant(-kInf);
  qp.ub.setConstant(kInf);
  const Eigen::VectorXd expected = -qp.H.ldlt().solve(qp.g);
  mmd::DenseQp no_rows = qp;
  no_rows.A.resize(0, 10);
  no_rows.lA.resize(0);
  no_rows.uA.resize(0);
  for (const char * name : kBackends)
    for (const auto * p : {&qp, &no_rows}) {
      const mmd::DenseQpResult r = mmd::make_dense_solver(name)->solve(*p, nullptr);
      ASSERT_TRUE(r.ok) << name << ": " << r.status;
      EXPECT_LT((r.x - expected).lpNorm<Eigen::Infinity>(), 1e-6) << name << " rows " << p->A.rows();
    }
}

TEST(DenseQp, InfeasibleReportsNotOk)
{
  std::mt19937 rng(13);
  mmd::DenseQp qp = random_qp(6, 2, rng);
  qp.A.row(1) = qp.A.row(0);
  qp.lA(0) = -kInf;
  qp.uA(0) = -1.0;
  qp.lA(1) = 1.0;
  qp.uA(1) = kInf;
  for (const char * name : kBackends) EXPECT_FALSE(mmd::make_dense_solver(name)->solve(qp, nullptr).ok) << name;
}

TEST(DenseQp, WarmStartAccepted)
{
  std::mt19937 rng(14);
  const mmd::DenseQp qp = random_qp(35, 300, rng);
  for (const char * name : kBackends) {
    const auto solver = mmd::make_dense_solver(name);
    const mmd::DenseQpResult cold = solver->solve(qp, nullptr);
    ASSERT_TRUE(cold.ok) << name;
    const mmd::DenseQpResult warm = solver->solve(qp, &cold.x);
    ASSERT_TRUE(warm.ok) << name;
    EXPECT_LT((warm.x - cold.x).lpNorm<Eigen::Infinity>(), 1e-6) << name;
    // A warm start of the wrong size is ignored, not an error.
    const Eigen::VectorXd wrong = Eigen::VectorXd::Zero(3);
    EXPECT_TRUE(solver->solve(qp, &wrong).ok) << name;
  }
}

TEST(DenseQp, ChangingDimensionsBetweenSolves)
{
  std::mt19937 rng(15);
  for (const char * name : kBackends) {
    const auto solver = mmd::make_dense_solver(name);
    for (const auto & [n, m] : {std::pair{35, 300}, std::pair{20, 50}, std::pair{35, 120}}) {
      const mmd::DenseQp qp = random_qp(n, m, rng);
      const mmd::DenseQpResult r = solver->solve(qp, nullptr);
      ASSERT_TRUE(r.ok) << name << " " << n << "x" << m;
      EXPECT_EQ(r.x.size(), n);
    }
  }
}

TEST(DenseQp, UnknownBackendThrows)
{
  EXPECT_THROW(mmd::make_dense_solver("qpoases"), std::invalid_argument);
}

TEST(DenseQp, MismatchedShapesThrow)
{
  std::mt19937 rng(16);
  mmd::DenseQp qp = random_qp(5, 3, rng);
  qp.uA.resize(2);
  for (const char * name : kBackends)
    EXPECT_THROW(mmd::make_dense_solver(name)->solve(qp, nullptr), std::invalid_argument) << name;
}
}  // namespace
