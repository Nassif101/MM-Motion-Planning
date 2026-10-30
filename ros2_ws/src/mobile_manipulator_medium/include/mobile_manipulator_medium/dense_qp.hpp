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

// The accuracy the backends are asked for, the same number for each so their timings compare (Task 6 review; recorded
// with every Task 9 table): HPIPM's stationarity, equality, inequality and complementarity tolerances, OSQP's absolute
// and relative eps (solutions polished), DAQP's primal tolerance (it stops on a vertex).
struct DenseQpOptions
{
  double tolerance = 1e-6;
};

class DenseQpSolver
{
public:
  virtual ~DenseQpSolver() = default;
  // `warm` is the previous solution of a similar problem (the last SCP step): HPIPM starts its primal from it, OSQP its
  // primal and keeps its own duals, DAQP starts from the working set it ended the last solve of the same dimensions with
  // (or, after a change of dimensions, the constraints active at `warm`); a `warm` of another size, or none, is a cold
  // start. A solver keeps its workspace between calls of the same dimensions. Bounds the wrong way round (lb > ub or
  // lA > uA) are not ok ("inconsistent bounds"), with x zero. Throws std::invalid_argument for inconsistent shapes.
  virtual DenseQpResult solve(const DenseQp & qp, const Eigen::VectorXd * warm) = 0;
};

// "hpipm" (dense interior point), "osqp" (ADMM, polished) or "daqp" (dual active set); std::invalid_argument otherwise.
std::unique_ptr<DenseQpSolver> make_dense_solver(const std::string & backend, DenseQpOptions options = {});

// Throws std::invalid_argument unless the shapes of `qp` agree (H n x n, g n, A m x n, lA and uA m, lb and ub n).
void check_dense_qp(const DenseQp & qp);
// False when a finite lower bound exceeds its upper bound (the backends report it as not ok instead of solving).
bool bounds_consistent(const DenseQp & qp);
// The result for inconsistent bounds: not ok, x zero.
DenseQpResult inconsistent_bounds_result(const DenseQp & qp);

std::unique_ptr<DenseQpSolver> make_hpipm_dense_solver(DenseQpOptions options);
std::unique_ptr<DenseQpSolver> make_osqp_dense_solver(DenseQpOptions options);
std::unique_ptr<DenseQpSolver> make_daqp_dense_solver(DenseQpOptions options);
}  // namespace mobile_manipulator_medium
