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
// The correctness tests ask every backend for 1e-9: they check the implementations, not the default accuracy (at the
// default 1e-6 the interior point's duality gap grows with the row count; Task 9 measures accuracy against time).
const mmd::DenseQpOptions kTight{1e-9};

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
// (1e-5, metres on a collision row), objective (1e-6 relative to the best) and distance (1e-4).
TEST(DenseQp, BackendsAgreeOnRandomQps)
{
  std::mt19937 rng(11);
  for (int trial = 0; trial < 100; ++trial) {
    const mmd::DenseQp qp = random_qp(35, 300, rng);
    std::vector<Eigen::VectorXd> solutions;
    double best = std::numeric_limits<double>::infinity();
    for (const char * name : kBackends) {
      const auto solver = mmd::make_dense_solver(name, kTight);
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
    const auto solver = mmd::make_dense_solver(name, kTight);
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

TEST(DenseQp, InvalidBoundsReportNotOk)
{
  // A bound pair the wrong way round (lb > ub, lA > uA): not ok from every backend, no exception, a finite result.
  std::mt19937 rng(17);
  for (int which = 0; which < 2; ++which) {
    mmd::DenseQp qp = random_qp(8, 5, rng);
    if (which == 0) {
      qp.lb(1) = 1.0;
      qp.ub(1) = 0.0;
    } else {
      qp.lA(2) = 1.0;
      qp.uA(2) = 0.0;
    }
    for (const char * name : kBackends) {
      mmd::DenseQpResult r;
      ASSERT_NO_THROW(r = mmd::make_dense_solver(name)->solve(qp, nullptr)) << name;
      EXPECT_FALSE(r.ok) << name;
      EXPECT_GE(r.iterations, 0) << name;
      EXPECT_EQ(r.x.size(), 8) << name;
      EXPECT_TRUE(r.x.allFinite()) << name;
    }
  }
}

TEST(DenseQp, ReusedSolverMatchesFreshSolver)
{
  // The SCP pattern: one solver, new data of the same size every call.
  std::mt19937 rng(18);
  std::vector<mmd::DenseQp> qps;
  for (int i = 0; i < 20; ++i) qps.push_back(random_qp(35, 300, rng));
  for (const char * name : kBackends) {
    const auto reused = mmd::make_dense_solver(name, kTight);
    for (size_t i = 0; i < qps.size(); ++i) {
      const mmd::DenseQpResult a = reused->solve(qps[i], nullptr);
      const mmd::DenseQpResult b = mmd::make_dense_solver(name, kTight)->solve(qps[i], nullptr);
      ASSERT_TRUE(a.ok && b.ok) << name << " qp " << i;
      EXPECT_LE(std::abs(objective(qps[i], a.x) - objective(qps[i], b.x)), 1e-6 * std::max(1.0, std::abs(objective(qps[i], b.x))))
        << name << " qp " << i;
      EXPECT_LT((a.x - b.x).lpNorm<Eigen::Infinity>(), 1e-4) << name << " qp " << i;
    }
  }
}

TEST(DenseQp, WarmStartCutsIterationsOnAPerturbedQp)
{
  // An SCP step: the next QP is the last one slightly changed. DAQP carries its working set, OSQP its iterates.
  std::mt19937 rng(19);
  std::normal_distribution<double> g(0.0, 1e-3);
  int cold_total[2] = {0, 0}, warm_total[2] = {0, 0};
  for (int trial = 0; trial < 10; ++trial) {
    const mmd::DenseQp a = random_qp(35, 300, rng);
    mmd::DenseQp b = a;
    for (int i = 0; i < b.g.size(); ++i) b.g(i) += g(rng);
    for (int i = 0; i < b.uA.size(); ++i)
      if (std::isfinite(b.uA(i))) b.uA(i) += g(rng);
    for (int k = 0; k < 2; ++k) {
      const char * name = k == 0 ? "daqp" : "osqp";
      const auto solver = mmd::make_dense_solver(name);
      const mmd::DenseQpResult first = solver->solve(a, nullptr);
      ASSERT_TRUE(first.ok) << name;
      const mmd::DenseQpResult warm = solver->solve(b, &first.x);
      const mmd::DenseQpResult cold = mmd::make_dense_solver(name)->solve(b, nullptr);
      ASSERT_TRUE(warm.ok && cold.ok) << name;
      EXPECT_LT((warm.x - cold.x).lpNorm<Eigen::Infinity>(), 1e-4) << name;
      warm_total[k] += warm.iterations;
      cold_total[k] += cold.iterations;
    }
  }
  EXPECT_LT(warm_total[0], cold_total[0] / 2) << "daqp warm " << warm_total[0] << " cold " << cold_total[0];
  EXPECT_LT(warm_total[1], cold_total[1]) << "osqp warm " << warm_total[1] << " cold " << cold_total[1];
}

TEST(DenseQp, ToleranceIsAnOption)
{
  std::mt19937 rng(20);
  const mmd::DenseQp qp = random_qp(35, 300, rng);
  for (const char * name : {"hpipm", "osqp"}) {
    const auto loose = mmd::make_dense_solver(name, mmd::DenseQpOptions{1e-4})->solve(qp, nullptr);
    const auto tight = mmd::make_dense_solver(name, mmd::DenseQpOptions{1e-9})->solve(qp, nullptr);
    ASSERT_TRUE(loose.ok && tight.ok) << name;
    EXPECT_LT(loose.iterations, tight.iterations) << name;
  }
  EXPECT_DOUBLE_EQ(mmd::DenseQpOptions{}.tolerance, 1e-6);
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
