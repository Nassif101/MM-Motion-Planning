#pragma once
// HPIPM backend of the stage-wise QP (Phase 4B-B spec Section 4.2): QpProblem maps one to one onto HPIPM's OCP QP.
// Per stage Q, R, S (HPIPM's S is ours transposed), q, r, A, B, b = c; the general rows Cx, Cu with their bounds; the
// state and control boxes (stage 0's state box is dx0, an equality); each slacked row a soft constraint on its one
// finite bound with the L1 weight in zl/zu and the quadratic weight in Zl/Zu, slacks >= 0. An infinite bound is
// masked. Optional partial condensing into partial_condensing_blocks stages. setup() fixes the dimensions and the
// finite-bound pattern; solve() only writes values.
#include <memory>

#include "mobile_manipulator_mpc/qp.hpp"
#include "mobile_manipulator_mpc/solver_config.hpp"

namespace mobile_manipulator_mpc
{
class HpipmSolver : public QpSolver
{
public:
  explicit HpipmSolver(const HpipmOptions & options = HpipmOptions());
  ~HpipmSolver() override;
  HpipmSolver(const HpipmSolver &) = delete;
  HpipmSolver & operator=(const HpipmSolver &) = delete;

  void setup(const QpProblem & shape) override;
  // `warm` warm-starts the primal iterate when HpipmOptions::warm_start (HPIPM's warm_start 1); its primal is this
  // backend's own packing (per stage u then x).
  SolveResult solve(const QpProblem & problem, const QpSolution * warm) override;

  int setup_count() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace mobile_manipulator_mpc
