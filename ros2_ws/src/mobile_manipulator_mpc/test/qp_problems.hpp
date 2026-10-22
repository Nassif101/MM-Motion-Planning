#pragma once
// Stage-wise QPs with known answers for the QP backend tests (test_osqp_solver, test_solver_backends): a double
// integrator (with boxes and a slacked row) and a varied problem with cross terms, offsets and terminal rows checked
// against an independent dense KKT solve.
#include <limits>
#include <vector>

#include <Eigen/Dense>

#include "mobile_manipulator_mpc/qp.hpp"

namespace qp_problems
{
using mobile_manipulator_mpc::QpProblem;
using mobile_manipulator_mpc::StageQp;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;

inline constexpr double kDt = 0.1;
inline const double kInf = std::numeric_limits<double>::infinity();

inline MatrixXd dynamics_a()
{
  MatrixXd a(2, 2);
  a << 1.0, kDt, 0.0, 1.0;
  return a;
}

inline MatrixXd dynamics_b()
{
  MatrixXd b(2, 1);
  b << 0.5 * kDt * kDt, kDt;
  return b;
}

// Double integrator: cost sum 0.5 (dx'Q dx + du'R du) plus 0.5 dx_N'Qf dx_N, from `x0`.
inline QpProblem double_integrator(int horizon, const Vector2d & x0)
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

// Position at stage 3 must be at least `required`; the controls are limited to +-1, so the
// largest reachable position there is 0.5 * (3 dt)^2 = 0.045 (for a start at rest).
inline QpProblem with_slacked_position_row(double required)
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

inline QpProblem constrained_problem(const Vector2d & x0)
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

// Independent dense reference over z = [dx_0..dx_N, du_0..du_{N-1}]: the equality-constrained
// QP with the dynamics, dx_0 = dx0 and the rows in `active` (each row . z = rhs), solved as a
// KKT system. `lambda` returns the multipliers of the active rows (stationarity H z + g + E'l = 0).
struct ActiveRow
{
  VectorXd row;
  double rhs;
};

inline VectorXd dense_kkt(const QpProblem & p, const std::vector<ActiveRow> & active, VectorXd * lambda)
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
inline VectorXd constraint_row(const QpProblem & p, int k, int i)
{
  const int n = static_cast<int>(p.stages.size()), nx = 3, nu = 2;
  VectorXd row = VectorXd::Zero((n + 1) * nx + n * nu);
  const StageQp & s = k < n ? p.stages[k] : p.terminal;
  row.segment(k * nx, nx) = s.Cx.row(i).transpose();
  if (k < n) row.segment((n + 1) * nx + k * nu, nu) = s.Cu.row(i).transpose();
  return row;
}

inline QpProblem varied_problem()
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
}  // namespace qp_problems
