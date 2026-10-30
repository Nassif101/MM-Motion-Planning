#pragma once
// The medium layer's QP (Phase 5A spec Sections 3.4 and 6.1): a small dense problem solved about ten times per seed and
// cycle, with three interchangeable backends benchmarked in plan Task 9.
//
//   min 0.5 x^T H x + g^T x   subject to   lA <= A x <= uA,   lb <= x <= ub
//
// H must be symmetric positive definite (DAQP factorizes it). A bound of magnitude >= kDenseQpInfinity (or an infinite
// one) is no bound; A may have no rows.
#include <memory>
#include <string>

#include <Eigen/Core>

namespace mobile_manipulator_medium
{
constexpr double kDenseQpInfinity = 1e20;

struct DenseQp
{
  Eigen::MatrixXd H;
  Eigen::VectorXd g;
  Eigen::MatrixXd A;
  Eigen::VectorXd lA, uA, lb, ub;
};

struct DenseQpResult
{
  bool ok = false;     // solved (the backend's optimal, or OSQP's solved inaccurate)
  std::string status;  // the backend's own status text
  Eigen::VectorXd x;
  int iterations = 0;
  double solve_s = 0.0;  // wall time of solve(), including copying the problem in
};

class DenseQpSolver
{
public:
  virtual ~DenseQpSolver() = default;
  // `warm` (the previous solution of a similar problem) is used when its size matches, else ignored. A solver keeps its
  // workspace between calls of the same dimensions. Throws std::invalid_argument for inconsistent shapes.
  virtual DenseQpResult solve(const DenseQp & qp, const Eigen::VectorXd * warm) = 0;
};

// "hpipm" (dense interior point), "osqp" (ADMM, polished) or "daqp" (dual active set); std::invalid_argument otherwise.
std::unique_ptr<DenseQpSolver> make_dense_solver(const std::string & backend);

// Throws std::invalid_argument unless the shapes of `qp` agree (H n x n, g n, A m x n, lA and uA m, lb and ub n).
void check_dense_qp(const DenseQp & qp);

std::unique_ptr<DenseQpSolver> make_hpipm_dense_solver();
std::unique_ptr<DenseQpSolver> make_osqp_dense_solver();
std::unique_ptr<DenseQpSolver> make_daqp_dense_solver();
}  // namespace mobile_manipulator_medium
