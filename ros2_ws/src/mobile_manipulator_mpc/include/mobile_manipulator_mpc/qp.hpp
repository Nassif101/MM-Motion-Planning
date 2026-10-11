#pragma once
// Solver-agnostic, stage-wise description of the MPC's quadratic program. The formulation fills
// a QpProblem each cycle; a backend (OSQP now, possibly HPIPM/acados for the B6 baseline) turns
// it into its own format. Nothing here depends on a solver library.
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace mobile_manipulator_mpc
{
// One stage k of the horizon with the variables dx_k (nx) and du_k (nu):
//   cost        0.5 dx'Q dx + 0.5 du'R du + dx'S du + q'dx + r'du       (S is nx x nu)
//   dynamics    dx_{k+1} = A dx_k + B du_k + c
//   inequality  lower <= Cx dx_k + Cu du_k <= upper
//   boxes       x_lower <= dx_k <= x_upper, u_lower <= du_k <= u_upper
// Empty S, q, r, c mean zero; empty bounds mean unbounded; no inequality rows is an empty
// lower/upper. +-infinity (or magnitudes of 1e20 and more) mark a missing bound.
// The terminal stage x_N uses only Q, q, Cx (with lower/upper) and x_lower/x_upper; its R, S,
// r, A, B, c, Cu and the control bounds are ignored.
struct StageQp
{
  Eigen::MatrixXd Q, R, S;
  Eigen::VectorXd q, r;
  Eigen::MatrixXd A, B;
  Eigen::VectorXd c;
  Eigen::MatrixXd Cx, Cu;
  Eigen::VectorXd lower, upper;
  Eigen::VectorXd x_lower, x_upper, u_lower, u_upper;
};

// stages[0..N-1] plus the terminal stage give the states dx_0..dx_N and the controls
// du_0..du_{N-1}. dx0 fixes the first state (x_lower/x_upper of stage 0 do not apply to it).
// Inequality rows are numbered across the horizon in order: the rows of stage 0, of stage 1,
// ..., of stage N-1, then those of the terminal stage. Each row listed in slack_rows (slack_count
// entries) gets its own slack s >= 0, penalised linearly with slack_weight; the slack relaxes
// the row's one finite bound (lower <= Cz + s, or Cz - s <= upper). A slacked row must have
// exactly one finite bound.
struct QpProblem
{
  std::vector<StageQp> stages;
  StageQp terminal;
  Eigen::VectorXd dx0;
  int slack_count = 0;
  double slack_weight = 1e4;
  std::vector<int> slack_rows;
};

// primal and dual are the solver's own vectors, kept for warm-starting the next solve; dx
// (N+1 states), du (N controls) and slack are views unpacked from them.
struct QpSolution
{
  std::vector<Eigen::VectorXd> dx, du;
  Eigen::VectorXd slack;
  Eigen::VectorXd primal, dual;
};

struct SolveResult
{
  bool ok = false;      // false on any numerical failure; `status` then says why
  std::string status;
  int iterations = 0;
  double solve_s = 0.0;  // update and solve, seconds
  QpSolution solution;   // filled only when ok
};

class QpSolver
{
public:
  virtual ~QpSolver() = default;
  // Fixes the dimensions and the sparsity pattern from `shape`; its numbers are irrelevant.
  virtual void setup(const QpProblem & shape) = 0;
  // Solves a problem of the set-up shape, updating values in place. `warm` (the previous
  // solution) warm-starts it; nullptr starts cold. Throws std::invalid_argument when the
  // problem does not match the shape; never throws for a numerical failure.
  virtual SolveResult solve(const QpProblem & problem, const QpSolution * warm) = 0;
};
}  // namespace mobile_manipulator_mpc
