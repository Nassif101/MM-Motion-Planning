// HPIPM's dense interior-point QP (balance mode): every variable carries a box (masked where infinite) and A is the
// general constraint block.
#include <chrono>
#include <cmath>
#include <vector>

#include "hpipm_d_dense_qp.h"
#include "hpipm_d_dense_qp_dim.h"
#include "hpipm_d_dense_qp_ipm.h"
#include "hpipm_d_dense_qp_sol.h"
#include "mobile_manipulator_medium/dense_qp.hpp"

namespace mobile_manipulator_medium
{
namespace
{
constexpr int kIterMax = 200;
constexpr double kTolerance = 1e-9;

const char * hpipm_status(int status)
{
  switch (status) {
    case 0: return "success";
    case 1: return "max iterations";
    case 2: return "min step length";
    case 3: return "NaN";
    default: return "unknown status";
  }
}

// value and mask (1 bound, 0 none) of one bound
void bound(double value, double & out, double & mask)
{
  const bool finite = std::isfinite(value) && std::abs(value) < kDenseQpInfinity;
  out = finite ? value : 0.0;
  mask = finite ? 1.0 : 0.0;
}

class HpipmDenseSolver : public DenseQpSolver
{
public:
  DenseQpResult solve(const DenseQp & qp, const Eigen::VectorXd * warm) override
  {
    check_dense_qp(qp);
    const auto t0 = std::chrono::steady_clock::now();
    const int n = static_cast<int>(qp.H.rows()), m = static_cast<int>(qp.A.rows());
    if (n != n_ || m != m_) setup(n, m);

    d_dense_qp_set_H(const_cast<double *>(qp.H.data()), &qp_);
    d_dense_qp_set_g(const_cast<double *>(qp.g.data()), &qp_);
    for (int j = 0; j < n; ++j) {
      bound(qp.lb(j), lb_[j], lb_mask_[j]);
      bound(qp.ub(j), ub_[j], ub_mask_[j]);
    }
    d_dense_qp_set_idxb(idxb_.data(), &qp_);
    d_dense_qp_set_lb(lb_.data(), &qp_);
    d_dense_qp_set_lb_mask(lb_mask_.data(), &qp_);
    d_dense_qp_set_ub(ub_.data(), &qp_);
    d_dense_qp_set_ub_mask(ub_mask_.data(), &qp_);
    if (m > 0) {
      for (int i = 0; i < m; ++i) {
        bound(qp.lA(i), lg_[i], lg_mask_[i]);
        bound(qp.uA(i), ug_[i], ug_mask_[i]);
      }
      d_dense_qp_set_C(const_cast<double *>(qp.A.data()), &qp_);  // column-major ng x nv, as Eigen stores it
      d_dense_qp_set_lg(lg_.data(), &qp_);
      d_dense_qp_set_lg_mask(lg_mask_.data(), &qp_);
      d_dense_qp_set_ug(ug_.data(), &qp_);
      d_dense_qp_set_ug_mask(ug_mask_.data(), &qp_);
    }
    int warm_start = 0;
    if (warm != nullptr && warm->size() == n) {
      d_dense_qp_sol_set_v(const_cast<double *>(warm->data()), &sol_);
      warm_start = 1;
    }
    d_dense_qp_ipm_arg_set_warm_start(&warm_start, &arg_);

    d_dense_qp_ipm_solve(&qp_, &sol_, &arg_, &ws_);
    int status = 0, iterations = 0;
    d_dense_qp_ipm_get_status(&ws_, &status);
    d_dense_qp_ipm_get_iter(&ws_, &iterations);
    DenseQpResult r;
    r.status = hpipm_status(status);
    r.iterations = iterations;
    r.x.resize(n);
    d_dense_qp_sol_get_v(&sol_, r.x.data());
    r.ok = status == 0 && r.x.allFinite();
    r.solve_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return r;
  }

private:
  void setup(int n, int m)
  {
    n_ = n;
    m_ = m;
    dim_mem_.assign(d_dense_qp_dim_memsize(), 0);
    d_dense_qp_dim_create(&dim_, dim_mem_.data());
    d_dense_qp_dim_set_all(n, 0, n, m, 0, &dim_);
    qp_mem_.assign(d_dense_qp_memsize(&dim_), 0);
    d_dense_qp_create(&dim_, &qp_, qp_mem_.data());
    sol_mem_.assign(d_dense_qp_sol_memsize(&dim_), 0);
    d_dense_qp_sol_create(&dim_, &sol_, sol_mem_.data());
    arg_mem_.assign(d_dense_qp_ipm_arg_memsize(&dim_), 0);
    d_dense_qp_ipm_arg_create(&dim_, &arg_, arg_mem_.data());
    d_dense_qp_ipm_arg_set_default(BALANCE, &arg_);
    int iter_max = kIterMax;
    double tol = kTolerance;
    d_dense_qp_ipm_arg_set_iter_max(&iter_max, &arg_);
    d_dense_qp_ipm_arg_set_tol_stat(&tol, &arg_);
    d_dense_qp_ipm_arg_set_tol_eq(&tol, &arg_);
    d_dense_qp_ipm_arg_set_tol_ineq(&tol, &arg_);
    d_dense_qp_ipm_arg_set_tol_comp(&tol, &arg_);
    ws_mem_.assign(d_dense_qp_ipm_ws_memsize(&dim_, &arg_), 0);
    d_dense_qp_ipm_ws_create(&dim_, &arg_, &ws_, ws_mem_.data());
    idxb_.resize(n);
    for (int j = 0; j < n; ++j) idxb_[j] = j;
    lb_.assign(n, 0.0);
    ub_.assign(n, 0.0);
    lb_mask_.assign(n, 0.0);
    ub_mask_.assign(n, 0.0);
    lg_.assign(m, 0.0);
    ug_.assign(m, 0.0);
    lg_mask_.assign(m, 0.0);
    ug_mask_.assign(m, 0.0);
  }

  int n_ = -1, m_ = -1;
  std::vector<char> dim_mem_, qp_mem_, sol_mem_, arg_mem_, ws_mem_;
  d_dense_qp_dim dim_;
  d_dense_qp qp_;
  d_dense_qp_sol sol_;
  d_dense_qp_ipm_arg arg_;
  d_dense_qp_ipm_ws ws_;
  std::vector<int> idxb_;
  std::vector<double> lb_, ub_, lb_mask_, ub_mask_, lg_, ug_, lg_mask_, ug_mask_;
};
}  // namespace

std::unique_ptr<DenseQpSolver> make_hpipm_dense_solver() { return std::make_unique<HpipmDenseSolver>(); }
}  // namespace mobile_manipulator_medium
