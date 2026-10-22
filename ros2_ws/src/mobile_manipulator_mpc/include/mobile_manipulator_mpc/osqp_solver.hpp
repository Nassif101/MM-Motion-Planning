#pragma once
// OSQP backend of the stage-wise QP. The stages are stacked into one sparse QP over
// z = [dx_0..dx_N, du_0..du_{N-1}, slacks]; the rows are the dynamics, the inequalities and
// identity rows for the variable bounds. The sparsity pattern (dense blocks per stage) is fixed
// by setup(); solve() only rewrites values.
#include <memory>

#include "mobile_manipulator_mpc/qp.hpp"

namespace mobile_manipulator_mpc
{
struct OsqpOptions
{
  double eps_abs = 1e-4;
  double eps_rel = 1e-4;
  int max_iter = 200;
  bool polish = false;
  bool verbose = false;
  // Fixed rho-adaptation interval (iterations); OSQP's own default adapts on elapsed time,
  // which makes iteration counts depend on machine load.
  int adaptive_rho_interval = 100;
};

class OsqpSolver : public QpSolver
{
public:
  explicit OsqpSolver(const OsqpOptions & options = OsqpOptions());
  ~OsqpSolver() override;
  OsqpSolver(const OsqpSolver &) = delete;
  OsqpSolver & operator=(const OsqpSolver &) = delete;

  void setup(const QpProblem & shape) override;
  SolveResult solve(const QpProblem & problem, const QpSolution * warm) override;
  // OSQP's time_limit (its update and solve time), kept over setup().
  void set_time_limit(double seconds) override;

  // Number of osqp_setup calls so far (solve() must not add any).
  int setup_count() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace mobile_manipulator_mpc
