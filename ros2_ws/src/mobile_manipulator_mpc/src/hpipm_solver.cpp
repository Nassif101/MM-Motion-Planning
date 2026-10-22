#include "mobile_manipulator_mpc/hpipm_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "hpipm_d_ocp_qp.h"
#include "hpipm_d_ocp_qp_dim.h"
#include "hpipm_d_ocp_qp_ipm.h"
#include "hpipm_d_ocp_qp_sol.h"
#include "hpipm_d_part_cond.h"

namespace mobile_manipulator_mpc
{
namespace
{
using Eigen::MatrixXd;
using Eigen::VectorXd;
constexpr double kInfinite = 1e20;

bool finite(double v) { return std::isfinite(v) && std::abs(v) < kInfinite; }
double at(const VectorXd & v, int i, double fallback) { return v.size() == 0 ? fallback : v(i); }

hpipm_mode mode_of(const std::string & name)
{
  if (name == "speed") return SPEED;
  if (name == "robust") return ROBUST;
  if (name == "balance") return BALANCE;
  throw std::invalid_argument("HpipmSolver: mode must be speed, balance or robust, not " + name);
}

// Which bounds of a stage are finite, and the stage's soft constraints: the pattern setup() fixes.
struct StagePattern
{
  int nx = 0, nu = 0, ng = 0;
  std::vector<int> idxbu, idxbx;              // box indices with a finite side
  std::vector<double> lbu_on, ubu_on, lbx_on, ubx_on, lg_on, ug_on;  // HPIPM masks (1 finite, 0 masked)
  std::vector<int> soft_row;                  // local general-row index of each soft constraint (sorted)
  std::vector<int> soft_slack;                // its slack's index in QpProblem::slack_rows
  std::vector<bool> soft_lower;               // the slack relaxes the lower (else the upper) bound

  bool operator==(const StagePattern & o) const
  {
    return nx == o.nx && nu == o.nu && ng == o.ng && idxbu == o.idxbu && idxbx == o.idxbx && lbu_on == o.lbu_on &&
           ubu_on == o.ubu_on && lbx_on == o.lbx_on && ubx_on == o.ubx_on && lg_on == o.lg_on && ug_on == o.ug_on &&
           soft_row == o.soft_row && soft_slack == o.soft_slack && soft_lower == o.soft_lower;
  }
};

void box_pattern(const VectorXd & lower, const VectorXd & upper, int n, std::vector<int> & idx, std::vector<double> & lo_on,
                 std::vector<double> & up_on)
{
  for (int i = 0; i < n; ++i) {
    const bool lo = finite(at(lower, i, -kInfinite)), up = finite(at(upper, i, kInfinite));
    if (!lo && !up) continue;
    idx.push_back(i);
    lo_on.push_back(lo ? 1.0 : 0.0);
    up_on.push_back(up ? 1.0 : 0.0);
  }
}

std::vector<StagePattern> pattern_of(const QpProblem & p)
{
  const int n = static_cast<int>(p.stages.size());
  if (static_cast<int>(p.slack_rows.size()) != p.slack_count)
    throw std::invalid_argument("HpipmSolver: slack_rows does not match slack_count");
  std::vector<StagePattern> out(static_cast<size_t>(n + 1));
  int row_base = 0;
  std::vector<int> stage_of_row, local_of_row;
  for (int k = 0; k <= n; ++k) {
    const StageQp & s = k < n ? p.stages[static_cast<size_t>(k)] : p.terminal;
    StagePattern & sp = out[static_cast<size_t>(k)];
    sp.nx = static_cast<int>(s.Q.rows());
    sp.nu = k < n ? static_cast<int>(s.R.rows()) : 0;
    sp.ng = static_cast<int>(s.lower.size());
    if (s.upper.size() != s.lower.size()) throw std::invalid_argument("HpipmSolver: lower and upper sizes differ");
    if (k < n) box_pattern(s.u_lower, s.u_upper, sp.nu, sp.idxbu, sp.lbu_on, sp.ubu_on);
    if (k == 0) {
      for (int i = 0; i < sp.nx; ++i) {  // dx0 fixes the first state
        sp.idxbx.push_back(i);
        sp.lbx_on.push_back(1.0);
        sp.ubx_on.push_back(1.0);
      }
    } else {
      box_pattern(s.x_lower, s.x_upper, sp.nx, sp.idxbx, sp.lbx_on, sp.ubx_on);
    }
    for (int i = 0; i < sp.ng; ++i) {
      sp.lg_on.push_back(finite(s.lower(i)) ? 1.0 : 0.0);
      sp.ug_on.push_back(finite(s.upper(i)) ? 1.0 : 0.0);
      stage_of_row.push_back(k);
      local_of_row.push_back(i);
    }
    row_base += sp.ng;
  }
  for (int j = 0; j < p.slack_count; ++j) {
    const int row = p.slack_rows[static_cast<size_t>(j)];
    if (row < 0 || row >= row_base) throw std::invalid_argument("HpipmSolver: slack row out of range");
    StagePattern & sp = out[static_cast<size_t>(stage_of_row[static_cast<size_t>(row)])];
    const int local = local_of_row[static_cast<size_t>(row)];
    const bool lo = sp.lg_on[static_cast<size_t>(local)] > 0.0, up = sp.ug_on[static_cast<size_t>(local)] > 0.0;
    if (lo == up) throw std::invalid_argument("HpipmSolver: a slacked row needs exactly one finite bound");
    sp.soft_row.push_back(local);
    sp.soft_slack.push_back(j);
    sp.soft_lower.push_back(lo);
  }
  for (auto & sp : out) {  // HPIPM's soft constraints in row order
    std::vector<size_t> order(sp.soft_row.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return sp.soft_row[a] < sp.soft_row[b]; });
    StagePattern sorted = sp;
    for (size_t i = 0; i < order.size(); ++i) {
      sorted.soft_row[i] = sp.soft_row[order[i]];
      sorted.soft_slack[i] = sp.soft_slack[order[i]];
      sorted.soft_lower[i] = sp.soft_lower[order[i]];
    }
    sp = sorted;
  }
  return out;
}
}  // namespace

struct HpipmSolver::Impl
{
  HpipmOptions options;
  int setups = 0;
  int N = 0;
  std::vector<StagePattern> pattern;
  std::vector<int> nx, nu, nbx, nbu, ng, ns;
  // HPIPM's structures and their memory (HPIPM aligns inside the buffers it is given).
  std::vector<char> dim_mem, qp_mem, sol_mem, arg_mem, ws_mem;
  d_ocp_qp_dim dim{};
  d_ocp_qp qp{};
  d_ocp_qp_sol sol{};
  d_ocp_qp_ipm_arg arg{};
  d_ocp_qp_ipm_ws ws{};
  // Partial condensing (blocks > 0): the condensed problem, its solution and the condensing workspace.
  bool condensed = false;
  std::vector<int> block_size;
  std::vector<char> dim2_mem, qp2_mem, sol2_mem, part_arg_mem, part_ws_mem;
  d_ocp_qp_dim dim2{};
  d_ocp_qp qp2{};
  d_ocp_qp_sol sol2{};
  d_part_cond_qp_arg part_arg{};
  d_part_cond_qp_ws part_ws{};
  bool ready = false;

  void fill(const QpProblem & p);
};

HpipmSolver::HpipmSolver(const HpipmOptions & options) : impl_(std::make_unique<Impl>())
{
  mode_of(options.mode);
  if (options.max_iter < 1 || options.partial_condensing_blocks < 0 || !(options.tol > 0.0))
    throw std::invalid_argument("HpipmSolver: max_iter >= 1, partial_condensing_blocks >= 0 and tol > 0");
  impl_->options = options;
}

HpipmSolver::~HpipmSolver() = default;

int HpipmSolver::setup_count() const { return impl_->setups; }

void HpipmSolver::setup(const QpProblem & shape)
{
  Impl & im = *impl_;
  im.ready = false;
  im.pattern = pattern_of(shape);
  im.N = static_cast<int>(shape.stages.size());
  const size_t stages = static_cast<size_t>(im.N + 1);
  im.nx.assign(stages, 0);
  im.nu.assign(stages, 0);
  im.nbx.assign(stages, 0);
  im.nbu.assign(stages, 0);
  im.ng.assign(stages, 0);
  im.ns.assign(stages, 0);
  for (size_t k = 0; k < stages; ++k) {
    const StagePattern & sp = im.pattern[k];
    im.nx[k] = sp.nx;
    im.nu[k] = sp.nu;
    im.nbx[k] = static_cast<int>(sp.idxbx.size());
    im.nbu[k] = static_cast<int>(sp.idxbu.size());
    im.ng[k] = sp.ng;
    im.ns[k] = static_cast<int>(sp.soft_row.size());
  }
  im.dim_mem.assign(d_ocp_qp_dim_memsize(im.N), 0);
  d_ocp_qp_dim_create(im.N, &im.dim, im.dim_mem.data());
  d_ocp_qp_dim_set_all(im.nx.data(), im.nu.data(), im.nbx.data(), im.nbu.data(), im.ng.data(), im.ns.data(), &im.dim);
  im.qp_mem.assign(d_ocp_qp_memsize(&im.dim), 0);
  d_ocp_qp_create(&im.dim, &im.qp, im.qp_mem.data());
  im.sol_mem.assign(d_ocp_qp_sol_memsize(&im.dim), 0);
  d_ocp_qp_sol_create(&im.dim, &im.sol, im.sol_mem.data());

  const int blocks = im.options.partial_condensing_blocks;
  im.condensed = blocks > 0 && blocks < im.N;
  d_ocp_qp_dim * solve_dim = &im.dim;
  if (im.condensed) {
    im.block_size.assign(static_cast<size_t>(blocks + 1), 0);
    d_part_cond_qp_compute_block_size(im.N, blocks, im.block_size.data());
    im.dim2_mem.assign(d_ocp_qp_dim_memsize(blocks), 0);
    d_ocp_qp_dim_create(blocks, &im.dim2, im.dim2_mem.data());
    d_part_cond_qp_compute_dim(&im.dim, im.block_size.data(), &im.dim2);
    im.part_arg_mem.assign(d_part_cond_qp_arg_memsize(blocks), 0);
    d_part_cond_qp_arg_create(blocks, &im.part_arg, im.part_arg_mem.data());
    d_part_cond_qp_arg_set_default(&im.part_arg);
    im.part_ws_mem.assign(d_part_cond_qp_ws_memsize(&im.dim, im.block_size.data(), &im.dim2, &im.part_arg), 0);
    d_part_cond_qp_ws_create(&im.dim, im.block_size.data(), &im.dim2, &im.part_arg, &im.part_ws, im.part_ws_mem.data());
    im.qp2_mem.assign(d_ocp_qp_memsize(&im.dim2), 0);
    d_ocp_qp_create(&im.dim2, &im.qp2, im.qp2_mem.data());
    im.sol2_mem.assign(d_ocp_qp_sol_memsize(&im.dim2), 0);
    d_ocp_qp_sol_create(&im.dim2, &im.sol2, im.sol2_mem.data());
    solve_dim = &im.dim2;
  }
  im.arg_mem.assign(d_ocp_qp_ipm_arg_memsize(solve_dim), 0);
  d_ocp_qp_ipm_arg_create(solve_dim, &im.arg, im.arg_mem.data());
  d_ocp_qp_ipm_arg_set_default(mode_of(im.options.mode), &im.arg);
  int iter_max = im.options.max_iter;
  double tol = im.options.tol;
  d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &im.arg);
  d_ocp_qp_ipm_arg_set_tol_stat(&tol, &im.arg);
  d_ocp_qp_ipm_arg_set_tol_eq(&tol, &im.arg);
  d_ocp_qp_ipm_arg_set_tol_ineq(&tol, &im.arg);
  d_ocp_qp_ipm_arg_set_tol_comp(&tol, &im.arg);
  im.ws_mem.assign(d_ocp_qp_ipm_ws_memsize(solve_dim, &im.arg), 0);
  d_ocp_qp_ipm_ws_create(solve_dim, &im.arg, &im.ws, im.ws_mem.data());
  ++im.setups;
  im.ready = true;
}

void HpipmSolver::Impl::fill(const QpProblem & p)
{
  std::vector<double> buffer;
  auto put_matrix = [&](const MatrixXd & m, int rows, int cols, void (*set)(int, double *, d_ocp_qp *), int k) {
    if (rows == 0 || cols == 0) return;
    if (m.size() == 0) {
      buffer.assign(static_cast<size_t>(rows * cols), 0.0);
    } else {
      if (m.rows() != rows || m.cols() != cols) throw std::invalid_argument("HpipmSolver: matrix size");
      buffer.assign(m.data(), m.data() + m.size());  // column-major, as HPIPM reads it
    }
    set(k, buffer.data(), &qp);
  };
  auto put_vector = [&](const VectorXd & v, int n, void (*set)(int, double *, d_ocp_qp *), int k) {
    if (n == 0) return;
    if (v.size() == 0) {
      buffer.assign(static_cast<size_t>(n), 0.0);
    } else {
      if (v.size() != n) throw std::invalid_argument("HpipmSolver: vector size");
      buffer.assign(v.data(), v.data() + v.size());
    }
    set(k, buffer.data(), &qp);
  };
  auto put = [&](std::vector<double> values, void (*set)(int, double *, d_ocp_qp *), int k) {
    if (values.empty()) return;
    set(k, values.data(), &qp);
  };
  auto put_index = [&](std::vector<int> values, void (*set)(int, int *, d_ocp_qp *), int k) {
    if (values.empty()) return;
    set(k, values.data(), &qp);
  };

  for (int k = 0; k <= N; ++k) {
    const StageQp & s = k < N ? p.stages[static_cast<size_t>(k)] : p.terminal;
    const StagePattern & sp = pattern[static_cast<size_t>(k)];
    const int x = sp.nx, u = sp.nu;
    put_matrix(s.Q, x, x, d_ocp_qp_set_Q, k);
    put_vector(s.q, x, d_ocp_qp_set_q, k);
    if (k < N) {
      const int x_next = pattern[static_cast<size_t>(k + 1)].nx;
      put_matrix(s.A, x_next, x, d_ocp_qp_set_A, k);
      put_matrix(s.B, x_next, u, d_ocp_qp_set_B, k);
      put_vector(s.c, x_next, d_ocp_qp_set_b, k);
      put_matrix(s.R, u, u, d_ocp_qp_set_R, k);
      put_matrix(s.S.size() ? MatrixXd(s.S.transpose()) : MatrixXd(), u, x, d_ocp_qp_set_S, k);  // HPIPM: u' S x
      put_vector(s.r, u, d_ocp_qp_set_r, k);
      std::vector<double> lo, up;
      for (int i : sp.idxbu) {
        lo.push_back(finite(at(s.u_lower, i, -kInfinite)) ? s.u_lower(i) : 0.0);
        up.push_back(finite(at(s.u_upper, i, kInfinite)) ? s.u_upper(i) : 0.0);
      }
      put_index(sp.idxbu, d_ocp_qp_set_idxbu, k);
      put(lo, d_ocp_qp_set_lbu, k);
      put(up, d_ocp_qp_set_ubu, k);
      put(sp.lbu_on, d_ocp_qp_set_lbu_mask, k);
      put(sp.ubu_on, d_ocp_qp_set_ubu_mask, k);
    }
    std::vector<double> lo, up;
    for (int i : sp.idxbx) {
      if (k == 0) {
        if (p.dx0.size() != x) throw std::invalid_argument("HpipmSolver: dx0 size");
        lo.push_back(p.dx0(i));
        up.push_back(p.dx0(i));
      } else {
        lo.push_back(finite(at(s.x_lower, i, -kInfinite)) ? s.x_lower(i) : 0.0);
        up.push_back(finite(at(s.x_upper, i, kInfinite)) ? s.x_upper(i) : 0.0);
      }
    }
    put_index(sp.idxbx, d_ocp_qp_set_idxbx, k);
    put(lo, d_ocp_qp_set_lbx, k);
    put(up, d_ocp_qp_set_ubx, k);
    put(sp.lbx_on, d_ocp_qp_set_lbx_mask, k);
    put(sp.ubx_on, d_ocp_qp_set_ubx_mask, k);

    if (sp.ng > 0) {
      put_matrix(s.Cx, sp.ng, x, d_ocp_qp_set_C, k);
      if (k < N) put_matrix(s.Cu, sp.ng, u, d_ocp_qp_set_D, k);
      std::vector<double> lg, ug;
      for (int i = 0; i < sp.ng; ++i) {
        lg.push_back(sp.lg_on[static_cast<size_t>(i)] > 0.0 ? s.lower(i) : 0.0);
        ug.push_back(sp.ug_on[static_cast<size_t>(i)] > 0.0 ? s.upper(i) : 0.0);
      }
      put(lg, d_ocp_qp_set_lg, k);
      put(ug, d_ocp_qp_set_ug, k);
      put(sp.lg_on, d_ocp_qp_set_lg_mask, k);
      put(sp.ug_on, d_ocp_qp_set_ug_mask, k);
    }
    const size_t n_soft = sp.soft_row.size();
    if (n_soft > 0) {
      std::vector<int> idxs;
      std::vector<double> quadratic(n_soft, p.slack_quadratic), linear, zero(n_soft, 0.0);
      for (size_t i = 0; i < n_soft; ++i) {
        idxs.push_back(static_cast<int>(sp.idxbu.size() + sp.idxbx.size()) + sp.soft_row[i]);
        const int j = sp.soft_slack[i];
        linear.push_back(p.slack_weights.empty() ? p.slack_weight : p.slack_weights[static_cast<size_t>(j)]);
      }
      put_index(idxs, d_ocp_qp_set_idxs, k);
      put(quadratic, d_ocp_qp_set_Zl, k);
      put(quadratic, d_ocp_qp_set_Zu, k);
      put(linear, d_ocp_qp_set_zl, k);
      put(linear, d_ocp_qp_set_zu, k);
      put(zero, d_ocp_qp_set_lls, k);
      put(zero, d_ocp_qp_set_lus, k);
    }
  }
}

SolveResult HpipmSolver::solve(const QpProblem & problem, const QpSolution * warm)
{
  Impl & im = *impl_;
  if (!im.ready) throw std::logic_error("HpipmSolver::solve before setup");
  if (static_cast<int>(problem.stages.size()) != im.N || !(pattern_of(problem) == im.pattern))
    throw std::invalid_argument("QpProblem does not match the shape given to setup");
  if (!problem.slack_weights.empty() && static_cast<int>(problem.slack_weights.size()) != problem.slack_count)
    throw std::invalid_argument("HpipmSolver: slack_weights does not match slack_count");
  const auto t0 = std::chrono::steady_clock::now();
  SolveResult result;
  im.fill(problem);

  int warm_start = 0;
  size_t packed = 0;
  for (int k = 0; k <= im.N; ++k) packed += static_cast<size_t>(im.nu[static_cast<size_t>(k)] + im.nx[static_cast<size_t>(k)]);
  if (im.options.warm_start && warm != nullptr && static_cast<size_t>(warm->primal.size()) == packed) {
    std::vector<double> v(warm->primal.data(), warm->primal.data() + warm->primal.size());
    size_t at_k = 0;
    for (int k = 0; k <= im.N; ++k) {
      const size_t u = static_cast<size_t>(im.nu[static_cast<size_t>(k)]), x = static_cast<size_t>(im.nx[static_cast<size_t>(k)]);
      if (u) d_ocp_qp_sol_set_u(k, v.data() + at_k, &im.sol);
      if (x) d_ocp_qp_sol_set_x(k, v.data() + at_k + u, &im.sol);
      at_k += u + x;
    }
    warm_start = im.condensed ? 0 : 1;  // the condensed problem's iterate is not the stage-wise one
  }
  d_ocp_qp_ipm_arg_set_warm_start(&warm_start, &im.arg);

  if (im.condensed) {
    d_part_cond_qp_cond(&im.qp, &im.qp2, &im.part_arg, &im.part_ws);
    d_ocp_qp_ipm_solve(&im.qp2, &im.sol2, &im.arg, &im.ws);
    d_part_cond_qp_expand_sol(&im.qp, &im.sol2, &im.sol, &im.part_arg, &im.part_ws);
  } else {
    d_ocp_qp_ipm_solve(&im.qp, &im.sol, &im.arg, &im.ws);
  }
  int status = -1, iterations = 0;
  d_ocp_qp_ipm_get_status(&im.ws, &status);
  d_ocp_qp_ipm_get_iter(&im.ws, &iterations);
  result.iterations = iterations;
  static const char * kStatus[] = {"success", "max_iter", "min_step", "nan_sol", "inconsistent_eq"};
  result.status = status >= 0 && status <= 4 ? kStatus[status] : "status " + std::to_string(status);

  if (status == 0) {
    QpSolution & s = result.solution;
    s.primal.resize(static_cast<Eigen::Index>(packed));
    s.slack = VectorXd::Zero(problem.slack_count);
    size_t at_k = 0;
    bool numbers = true;
    for (int k = 0; k <= im.N; ++k) {
      const int u = im.nu[static_cast<size_t>(k)], x = im.nx[static_cast<size_t>(k)];
      VectorXd uk(u), xk(x);
      if (u) d_ocp_qp_sol_get_u(k, &im.sol, uk.data());
      if (x) d_ocp_qp_sol_get_x(k, &im.sol, xk.data());
      numbers = numbers && uk.allFinite() && xk.allFinite();
      s.primal.segment(static_cast<Eigen::Index>(at_k), u) = uk;
      s.primal.segment(static_cast<Eigen::Index>(at_k) + u, x) = xk;
      at_k += static_cast<size_t>(u + x);
      s.dx.push_back(xk);
      if (k < im.N) s.du.push_back(uk);
      const StagePattern & sp = im.pattern[static_cast<size_t>(k)];
      if (!sp.soft_row.empty()) {
        VectorXd sl(static_cast<Eigen::Index>(sp.soft_row.size())), su(sl.size());
        d_ocp_qp_sol_get_sl(k, &im.sol, sl.data());
        d_ocp_qp_sol_get_su(k, &im.sol, su.data());
        for (size_t i = 0; i < sp.soft_row.size(); ++i)
          s.slack(sp.soft_slack[i]) = sp.soft_lower[i] ? sl(static_cast<Eigen::Index>(i)) : su(static_cast<Eigen::Index>(i));
      }
    }
    result.ok = numbers && s.slack.allFinite();
    if (!result.ok) {
      result.status = "nan_sol";
      result.solution = QpSolution{};
    }
  }
  result.solve_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  return result;
}
}  // namespace mobile_manipulator_mpc
