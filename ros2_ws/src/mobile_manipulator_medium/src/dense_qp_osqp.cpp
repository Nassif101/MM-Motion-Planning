// OSQP (ADMM) on the dense problem: the constraint matrix is [I; A] (the bounds first), stored as full CSC so its
// pattern stays fixed and a solve of the same dimensions only updates values; solutions are polished.
#include <chrono>
#include <cmath>
#include <vector>

#include "mobile_manipulator_medium/dense_qp.hpp"
#include "osqp.h"

namespace mobile_manipulator_medium
{
namespace
{
constexpr double kEps = 1e-7;
constexpr int kMaxIter = 20000;

c_float lower(double v) { return std::isfinite(v) && v > -kDenseQpInfinity ? v : -OSQP_INFTY; }
c_float upper(double v) { return std::isfinite(v) && v < kDenseQpInfinity ? v : OSQP_INFTY; }

class OsqpDenseSolver : public DenseQpSolver
{
public:
  ~OsqpDenseSolver() override { release(); }

  DenseQpResult solve(const DenseQp & qp, const Eigen::VectorXd * warm) override
  {
    check_dense_qp(qp);
    const auto t0 = std::chrono::steady_clock::now();
    const int n = static_cast<int>(qp.H.rows()), m = static_cast<int>(qp.A.rows());
    fill(qp, n, m);
    DenseQpResult r;
    if (n != n_ || m != m_ || work_ == nullptr) {
      setup(n, m);
    } else if (osqp_update_P_A(work_, p_values_.data(), OSQP_NULL, static_cast<c_int>(p_values_.size()),
                               a_values_.data(), OSQP_NULL, static_cast<c_int>(a_values_.size())) != 0 ||
               osqp_update_lin_cost(work_, q_.data()) != 0 || osqp_update_bounds(work_, l_.data(), u_.data()) != 0) {
      r.status = "osqp update failed";
      return r;
    }
    std::vector<c_float> x0(n, 0.0), y0(n + m, 0.0);
    if (warm != nullptr && warm->size() == n) Eigen::Map<Eigen::VectorXd>(x0.data(), n) = *warm;
    osqp_warm_start(work_, x0.data(), y0.data());
    osqp_solve(work_);
    const OSQPInfo * info = work_->info;
    r.status = info->status;
    r.iterations = static_cast<int>(info->iter);
    r.ok = info->status_val == OSQP_SOLVED || info->status_val == OSQP_SOLVED_INACCURATE;
    r.x = Eigen::Map<const Eigen::VectorXd>(work_->solution->x, n);
    r.ok = r.ok && r.x.allFinite();
    r.solve_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return r;
  }

private:
  // CSC values of the upper triangle of H and of [I; A], the cost and the bounds.
  void fill(const DenseQp & qp, int n, int m)
  {
    p_values_.clear();
    for (int j = 0; j < n; ++j)
      for (int i = 0; i <= j; ++i) p_values_.push_back(qp.H(i, j));
    a_values_.clear();
    for (int j = 0; j < n; ++j) {
      a_values_.push_back(1.0);
      for (int i = 0; i < m; ++i) a_values_.push_back(qp.A(i, j));
    }
    q_.assign(qp.g.data(), qp.g.data() + n);
    l_.resize(n + m);
    u_.resize(n + m);
    for (int j = 0; j < n; ++j) {
      l_[j] = lower(qp.lb(j));
      u_[j] = upper(qp.ub(j));
    }
    for (int i = 0; i < m; ++i) {
      l_[n + i] = lower(qp.lA(i));
      u_[n + i] = upper(qp.uA(i));
    }
  }

  void setup(int n, int m)
  {
    release();
    n_ = n;
    m_ = m;
    p_col_.assign(1, 0);
    p_row_.clear();
    for (int j = 0; j < n; ++j) {
      for (int i = 0; i <= j; ++i) p_row_.push_back(i);
      p_col_.push_back(static_cast<c_int>(p_row_.size()));
    }
    a_col_.assign(1, 0);
    a_row_.clear();
    for (int j = 0; j < n; ++j) {
      a_row_.push_back(j);
      for (int i = 0; i < m; ++i) a_row_.push_back(n + i);
      a_col_.push_back(static_cast<c_int>(a_row_.size()));
    }
    csc P{static_cast<c_int>(p_values_.size()), n, n, p_col_.data(), p_row_.data(), p_values_.data(), -1};
    csc A{static_cast<c_int>(a_values_.size()), n + m, n, a_col_.data(), a_row_.data(), a_values_.data(), -1};
    OSQPData data;
    data.n = n;
    data.m = n + m;
    data.P = &P;
    data.A = &A;
    data.q = q_.data();
    data.l = l_.data();
    data.u = u_.data();
    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.polish = 1;
    settings.warm_start = 1;
    settings.eps_abs = kEps;
    settings.eps_rel = kEps;
    settings.max_iter = kMaxIter;
    if (osqp_setup(&work_, &data, &settings) != 0) {
      release();
      throw std::runtime_error("osqp_setup failed");
    }
  }

  void release()
  {
    if (work_ != nullptr) osqp_cleanup(work_);
    work_ = nullptr;
    n_ = m_ = -1;
  }

  OSQPWorkspace * work_ = nullptr;
  int n_ = -1, m_ = -1;
  std::vector<c_int> p_col_, p_row_, a_col_, a_row_;
  std::vector<c_float> p_values_, a_values_, q_, l_, u_;
};
}  // namespace

std::unique_ptr<DenseQpSolver> make_osqp_dense_solver() { return std::make_unique<OsqpDenseSolver>(); }
}  // namespace mobile_manipulator_medium
