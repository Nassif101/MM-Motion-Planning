#include "mobile_manipulator_mpc/osqp_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>

#include "osqp.h"

namespace mobile_manipulator_mpc
{
namespace
{
constexpr double kInfinite = 1e20;  // |bound| at or above this is "no bound"

using Eigen::MatrixXd;
using Eigen::VectorXd;

double entry(const MatrixXd & m, int i, int j) { return m.size() == 0 ? 0.0 : m(i, j); }
double entry(const VectorXd & v, int i) { return v.size() == 0 ? 0.0 : v(i); }

double lower_of(const VectorXd & v, int i) { return v.size() == 0 ? -OSQP_INFTY : (v(i) <= -kInfinite ? -OSQP_INFTY : v(i)); }
double upper_of(const VectorXd & v, int i) { return v.size() == 0 ? OSQP_INFTY : (v(i) >= kInfinite ? OSQP_INFTY : v(i)); }

[[noreturn]] void bad(const std::string & what) { throw std::invalid_argument("QpProblem: " + what); }

void expect(const MatrixXd & m, int rows, int cols, bool optional, const char * name)
{
  if (optional && m.size() == 0) return;
  if (m.rows() != rows || m.cols() != cols) bad(std::string(name) + " has the wrong size");
}

void expect(const VectorXd & v, int size, bool optional, const char * name)
{
  if (optional && v.size() == 0) return;
  if (v.size() != size) bad(std::string(name) + " has the wrong size");
}

// Everything that fixes the sparsity pattern.
struct Dims
{
  int N = 0, nx = 0, nu = 0, ns = 0;
  std::vector<int> nc;  // inequality rows of stages 0..N-1 and the terminal stage
  std::vector<int> slack_rows;
  int total_ineq = 0;
  int n = 0, m = 0;
  int x_at(int k) const { return k * nx; }
  int u_at(int k) const { return (N + 1) * nx + k * nu; }
  int s_at(int j) const { return (N + 1) * nx + N * nu + j; }
  int dyn_row(int k) const { return k * nx; }
  int ineq_row(int g) const { return N * nx + g; }
  int box_row(int var) const { return N * nx + total_ineq + var; }

  bool operator==(const Dims & o) const
  {
    return N == o.N && nx == o.nx && nu == o.nu && ns == o.ns && nc == o.nc && slack_rows == o.slack_rows;
  }
};

Dims make_dims(const QpProblem & p)
{
  Dims d;
  d.N = static_cast<int>(p.stages.size());
  if (d.N < 1) bad("needs at least one stage");
  d.nx = static_cast<int>(p.stages[0].Q.rows());
  d.nu = static_cast<int>(p.stages[0].R.rows());
  if (d.nx < 1) bad("Q of stage 0 is empty");
  expect(p.dx0, d.nx, false, "dx0");

  auto check_ineq = [&](const StageQp & s, bool has_u, int k) {
    const std::string at = " (stage " + std::to_string(k) + ")";
    const int nc = static_cast<int>(s.lower.size());
    if (s.upper.size() != nc) bad("lower and upper differ in size" + at);
    if (nc > 0) {
      expect(s.Cx, nc, d.nx, false, "Cx");
      if (has_u) expect(s.Cu, nc, d.nu, d.nu == 0, "Cu");
    }
    d.nc.push_back(nc);
  };

  for (int k = 0; k < d.N; ++k) {
    const StageQp & s = p.stages[k];
    expect(s.Q, d.nx, d.nx, false, "Q");
    expect(s.R, d.nu, d.nu, false, "R");
    expect(s.S, d.nx, d.nu, true, "S");
    expect(s.q, d.nx, true, "q");
    expect(s.r, d.nu, true, "r");
    expect(s.A, d.nx, d.nx, false, "A");
    expect(s.B, d.nx, d.nu, false, "B");
    expect(s.c, d.nx, true, "c");
    expect(s.x_lower, d.nx, true, "x_lower");
    expect(s.x_upper, d.nx, true, "x_upper");
    expect(s.u_lower, d.nu, true, "u_lower");
    expect(s.u_upper, d.nu, true, "u_upper");
    check_ineq(s, true, k);
  }
  const StageQp & t = p.terminal;
  expect(t.Q, d.nx, d.nx, false, "terminal Q");
  expect(t.q, d.nx, true, "terminal q");
  expect(t.x_lower, d.nx, true, "terminal x_lower");
  expect(t.x_upper, d.nx, true, "terminal x_upper");
  check_ineq(t, false, d.N);

  d.total_ineq = std::accumulate(d.nc.begin(), d.nc.end(), 0);
  d.slack_rows = p.slack_rows;
  d.ns = static_cast<int>(d.slack_rows.size());
  if (p.slack_count != d.ns) bad("slack_count does not match slack_rows");
  std::vector<int> sorted = d.slack_rows;
  std::sort(sorted.begin(), sorted.end());
  if (std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end()) bad("slack_rows has duplicates");
  if (!sorted.empty() && (sorted.front() < 0 || sorted.back() >= d.total_ineq)) bad("slack_rows out of range");

  d.n = (d.N + 1) * d.nx + d.N * d.nu + d.ns;
  d.m = d.N * d.nx + d.total_ineq + d.n;
  return d;
}

// Bounds of the stacked inequality rows in the stage order.
void inequality_bounds(const QpProblem & p, const Dims & d, std::vector<double> & lo, std::vector<double> & hi)
{
  lo.clear();
  hi.clear();
  for (int k = 0; k <= d.N; ++k) {
    const StageQp & s = k < d.N ? p.stages[k] : p.terminal;
    for (int i = 0; i < d.nc[k]; ++i) {
      lo.push_back(lower_of(s.lower, i));
      hi.push_back(upper_of(s.upper, i));
    }
  }
}

// Calls emit(row, col, value) for each entry of the upper triangle of P, in a fixed order that
// depends only on the dimensions.
template <class F>
void emit_P(const QpProblem & p, const Dims & d, F && emit)
{
  for (int k = 0; k <= d.N; ++k) {
    const StageQp & s = k < d.N ? p.stages[k] : p.terminal;
    for (int j = 0; j < d.nx; ++j)
      for (int i = 0; i <= j; ++i) emit(d.x_at(k) + i, d.x_at(k) + j, 0.5 * (entry(s.Q, i, j) + entry(s.Q, j, i)));
    if (k == d.N) break;
    for (int j = 0; j < d.nu; ++j)
      for (int i = 0; i < d.nx; ++i) emit(d.x_at(k) + i, d.u_at(k) + j, entry(s.S, i, j));
    for (int j = 0; j < d.nu; ++j)
      for (int i = 0; i <= j; ++i) emit(d.u_at(k) + i, d.u_at(k) + j, 0.5 * (entry(s.R, i, j) + entry(s.R, j, i)));
  }
}

template <class F>
void emit_A(const QpProblem & p, const Dims & d, const std::vector<double> & slack_sign, F && emit)
{
  // dynamics: dx_{k+1} - A dx_k - B du_k = c
  for (int k = 0; k < d.N; ++k) {
    const StageQp & s = p.stages[k];
    for (int i = 0; i < d.nx; ++i) emit(d.dyn_row(k) + i, d.x_at(k + 1) + i, 1.0);
    for (int j = 0; j < d.nx; ++j)
      for (int i = 0; i < d.nx; ++i) emit(d.dyn_row(k) + i, d.x_at(k) + j, -entry(s.A, i, j));
    for (int j = 0; j < d.nu; ++j)
      for (int i = 0; i < d.nx; ++i) emit(d.dyn_row(k) + i, d.u_at(k) + j, -entry(s.B, i, j));
  }
  // inequalities: Cx dx_k + Cu du_k (+- slack)
  std::vector<int> slack_of(d.total_ineq, -1);
  for (int j = 0; j < d.ns; ++j) slack_of[d.slack_rows[j]] = j;
  int g = 0;
  for (int k = 0; k <= d.N; ++k) {
    const StageQp & s = k < d.N ? p.stages[k] : p.terminal;
    for (int i = 0; i < d.nc[k]; ++i, ++g) {
      for (int j = 0; j < d.nx; ++j) emit(d.ineq_row(g), d.x_at(k) + j, entry(s.Cx, i, j));
      if (k < d.N)
        for (int j = 0; j < d.nu; ++j) emit(d.ineq_row(g), d.u_at(k) + j, entry(s.Cu, i, j));
      if (slack_of[g] >= 0) emit(d.ineq_row(g), d.s_at(slack_of[g]), slack_sign[slack_of[g]]);
    }
  }
  // variable bounds
  for (int v = 0; v < d.n; ++v) emit(d.box_row(v), v, 1.0);
}

struct Pattern
{
  std::vector<c_int> col_ptr, row_idx;
  std::vector<std::size_t> position;  // generation order -> index in the CSC value array
  std::size_t nnz() const { return row_idx.size(); }
};

Pattern build_pattern(int cols, const std::vector<std::pair<int, int>> & rc)
{
  Pattern pat;
  std::vector<std::size_t> order(rc.size());
  std::iota(order.begin(), order.end(), std::size_t{0});
  std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    return rc[a].second != rc[b].second ? rc[a].second < rc[b].second : rc[a].first < rc[b].first;
  });
  pat.col_ptr.assign(cols + 1, 0);
  pat.row_idx.resize(rc.size());
  pat.position.resize(rc.size());
  for (std::size_t pos = 0; pos < order.size(); ++pos) {
    const auto & e = rc[order[pos]];
    if (pos > 0 && rc[order[pos - 1]] == e) throw std::logic_error("OsqpSolver: duplicate matrix entry");
    pat.row_idx[pos] = e.first;
    pat.position[order[pos]] = pos;
    ++pat.col_ptr[e.second + 1];
  }
  for (int c = 0; c < cols; ++c) pat.col_ptr[c + 1] += pat.col_ptr[c];
  return pat;
}
}  // namespace

struct OsqpSolver::Impl
{
  OsqpOptions options;
  OSQPWorkspace * work = nullptr;
  int setups = 0;
  Dims dims;
  std::vector<double> slack_sign;  // +1: the slack relaxes the lower bound, -1: the upper
  Pattern p_pattern, a_pattern;
  std::vector<c_float> p_values, a_values, q, l, u;  // CSC order for the matrices

  ~Impl() { release(); }

  void release()
  {
    if (work != nullptr) osqp_cleanup(work);
    work = nullptr;
  }

  // Fills every value array from the problem; the bounds have lower <= upper enforced when
  // `sanitize` (setup only, where the numbers are placeholders).
  bool fill(const QpProblem & p, bool sanitize, std::string * why)
  {
    const Dims & d = dims;
    {
      std::size_t i = 0;
      emit_P(p, d, [&](int, int, double v) { p_values[p_pattern.position[i++]] = v; });
    }
    {
      std::size_t i = 0;
      emit_A(p, d, slack_sign, [&](int, int, double v) { a_values[a_pattern.position[i++]] = v; });
    }

    q.assign(d.n, 0.0);
    for (int k = 0; k <= d.N; ++k) {
      const StageQp & s = k < d.N ? p.stages[k] : p.terminal;
      for (int i = 0; i < d.nx; ++i) q[d.x_at(k) + i] = entry(s.q, i);
      if (k < d.N)
        for (int i = 0; i < d.nu; ++i) q[d.u_at(k) + i] = entry(s.r, i);
    }
    for (int j = 0; j < d.ns; ++j) q[d.s_at(j)] = p.slack_weight;

    l.assign(d.m, 0.0);
    u.assign(d.m, 0.0);
    for (int k = 0; k < d.N; ++k)
      for (int i = 0; i < d.nx; ++i) l[d.dyn_row(k) + i] = u[d.dyn_row(k) + i] = entry(p.stages[k].c, i);
    std::vector<double> lo, hi;
    inequality_bounds(p, d, lo, hi);
    for (int g = 0; g < d.total_ineq; ++g) {
      l[d.ineq_row(g)] = lo[g];
      u[d.ineq_row(g)] = hi[g];
    }
    for (int j = 0; j < d.ns; ++j) {
      const int g = d.slack_rows[j];
      const bool lower_finite = lo[g] > -OSQP_INFTY, upper_finite = hi[g] < OSQP_INFTY;
      if (!sanitize && (slack_sign[j] > 0 ? upper_finite : lower_finite))
        bad("slacked inequality row " + std::to_string(g) + " has a bound on the side the slack cannot relax");
    }
    for (int k = 0; k <= d.N; ++k) {
      const StageQp & s = k < d.N ? p.stages[k] : p.terminal;
      for (int i = 0; i < d.nx; ++i) {
        const int row = d.box_row(d.x_at(k) + i);
        if (k == 0) {
          l[row] = u[row] = p.dx0(i);
        } else {
          l[row] = lower_of(s.x_lower, i);
          u[row] = upper_of(s.x_upper, i);
        }
      }
      if (k < d.N)
        for (int i = 0; i < d.nu; ++i) {
          const int row = d.box_row(d.u_at(k) + i);
          l[row] = lower_of(s.u_lower, i);
          u[row] = upper_of(s.u_upper, i);
        }
    }
    for (int j = 0; j < d.ns; ++j) {
      l[d.box_row(d.s_at(j))] = 0.0;
      u[d.box_row(d.s_at(j))] = OSQP_INFTY;
    }
    for (int r = 0; r < d.m; ++r) {
      if (l[r] <= u[r]) continue;
      if (sanitize) {
        l[r] = u[r] = 0.5 * (l[r] + u[r]);
      } else {
        if (why != nullptr) *why = "invalid bounds (lower > upper) in row " + std::to_string(r);
        return false;
      }
    }
    return true;
  }
};

OsqpSolver::OsqpSolver(const OsqpOptions & options) : impl_(std::make_unique<Impl>()) { impl_->options = options; }

OsqpSolver::~OsqpSolver() = default;

int OsqpSolver::setup_count() const { return impl_->setups; }

void OsqpSolver::setup(const QpProblem & shape)
{
  Impl & im = *impl_;
  const Dims d = make_dims(shape);

  // The relaxed side of each slacked row is the finite one in the shape.
  std::vector<double> lo, hi;
  inequality_bounds(shape, d, lo, hi);
  std::vector<double> sign(d.ns, 1.0);
  for (int j = 0; j < d.ns; ++j) {
    const int g = d.slack_rows[j];
    const bool lower_finite = lo[g] > -OSQP_INFTY, upper_finite = hi[g] < OSQP_INFTY;
    if (lower_finite && upper_finite) bad("slacked inequality row " + std::to_string(g) + " has two finite bounds");
    sign[j] = lower_finite || !upper_finite ? 1.0 : -1.0;
  }

  std::vector<std::pair<int, int>> p_rc, a_rc;
  emit_P(shape, d, [&](int r, int c, double) { p_rc.emplace_back(r, c); });
  emit_A(shape, d, sign, [&](int r, int c, double) { a_rc.emplace_back(r, c); });

  im.release();
  im.dims = d;
  im.slack_sign = sign;
  im.p_pattern = build_pattern(d.n, p_rc);
  im.a_pattern = build_pattern(d.n, a_rc);
  im.p_values.assign(im.p_pattern.nnz(), 0.0);
  im.a_values.assign(im.a_pattern.nnz(), 0.0);
  im.fill(shape, true, nullptr);

  csc P{static_cast<c_int>(im.p_pattern.nnz()), d.n, d.n, im.p_pattern.col_ptr.data(), im.p_pattern.row_idx.data(),
        im.p_values.data(), -1};
  csc A{static_cast<c_int>(im.a_pattern.nnz()), d.m, d.n, im.a_pattern.col_ptr.data(), im.a_pattern.row_idx.data(),
        im.a_values.data(), -1};
  OSQPData data;
  data.n = d.n;
  data.m = d.m;
  data.P = &P;
  data.A = &A;
  data.q = im.q.data();
  data.l = im.l.data();
  data.u = im.u.data();

  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.polish = im.options.polish ? 1 : 0;
  settings.verbose = im.options.verbose ? 1 : 0;
  settings.warm_start = 1;
  settings.eps_abs = im.options.eps_abs;
  settings.eps_rel = im.options.eps_rel;
  settings.max_iter = im.options.max_iter;
  settings.adaptive_rho_interval = im.options.adaptive_rho_interval;

  if (osqp_setup(&im.work, &data, &settings) != 0) {
    im.release();
    throw std::runtime_error("osqp_setup failed");
  }
  ++im.setups;
}

SolveResult OsqpSolver::solve(const QpProblem & problem, const QpSolution * warm)
{
  Impl & im = *impl_;
  if (im.work == nullptr) throw std::logic_error("OsqpSolver::solve before setup");
  const Dims d = make_dims(problem);
  if (!(d == im.dims)) throw std::invalid_argument("QpProblem does not match the shape given to setup");

  const auto t0 = std::chrono::steady_clock::now();
  SolveResult result;
  if (!im.fill(problem, false, &result.status)) return result;

  // One combined update, so the KKT system is refactorised once per solve, not twice.
  if (osqp_update_P_A(im.work, im.p_values.data(), OSQP_NULL, static_cast<c_int>(im.p_values.size()),
                      im.a_values.data(), OSQP_NULL, static_cast<c_int>(im.a_values.size())) != 0 ||
      osqp_update_lin_cost(im.work, im.q.data()) != 0 || osqp_update_bounds(im.work, im.l.data(), im.u.data()) != 0) {
    result.status = "osqp update failed";
    return result;
  }

  std::vector<c_float> x0(d.n, 0.0), y0(d.m, 0.0);
  if (warm != nullptr && warm->primal.size() == d.n && warm->dual.size() == d.m) {
    Eigen::Map<VectorXd>(x0.data(), d.n) = warm->primal;
    Eigen::Map<VectorXd>(y0.data(), d.m) = warm->dual;
  }
  osqp_warm_start(im.work, x0.data(), y0.data());

  osqp_solve(im.work);
  const OSQPInfo * info = im.work->info;
  result.status = info->status;
  result.iterations = static_cast<int>(info->iter);
  result.ok = info->status_val == OSQP_SOLVED || info->status_val == OSQP_SOLVED_INACCURATE;
  if (result.ok) {
    QpSolution & sol = result.solution;
    sol.primal = Eigen::Map<const VectorXd>(im.work->solution->x, d.n);
    sol.dual = Eigen::Map<const VectorXd>(im.work->solution->y, d.m);
    for (int k = 0; k <= d.N; ++k) sol.dx.push_back(sol.primal.segment(d.x_at(k), d.nx));
    for (int k = 0; k < d.N; ++k) sol.du.push_back(sol.primal.segment(d.u_at(k), d.nu));
    sol.slack = sol.primal.segment(d.s_at(0), d.ns);
  }
  result.solve_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  return result;
}
}  // namespace mobile_manipulator_mpc
