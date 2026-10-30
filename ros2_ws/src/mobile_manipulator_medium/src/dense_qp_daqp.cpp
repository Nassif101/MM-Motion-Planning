// DAQP (dual active set) on the dense problem: the variable bounds are DAQP's simple bounds and A its general rows
// (row-major). The working set a solve ends with seeds the next solve of the same dimensions (a hot start for SCP,
// whose consecutive QPs differ little); after a change of dimensions a warm point gives the constraints active at it.
#include <chrono>
#include <cmath>
#include <vector>

#include <Eigen/Core>

#include "mobile_manipulator_medium/dense_qp.hpp"

extern "C" {
#include "daqp/api.h"
}

namespace mobile_manipulator_medium
{
namespace
{
const char * daqp_status(int flag)
{
  switch (flag) {
    case DAQP_EXIT_OPTIMAL: return "optimal";
    case DAQP_EXIT_SOFT_OPTIMAL: return "soft optimal";
    case DAQP_EXIT_INFEASIBLE: return "infeasible";
    case DAQP_EXIT_CYCLE: return "cycling";
    case DAQP_EXIT_UNBOUNDED: return "unbounded";
    case DAQP_EXIT_ITERLIMIT: return "iteration limit";
    case DAQP_EXIT_NONCONVEX: return "nonconvex";
    case DAQP_EXIT_OVERDETERMINED_INITIAL: return "overdetermined initial active set";
    default: return "error";
  }
}

double daqp_bound(double v) { return std::isfinite(v) && std::abs(v) < kDenseQpInfinity ? v : (v < 0 ? -DAQP_INF : DAQP_INF); }

class DaqpDenseSolver : public DenseQpSolver
{
public:
  explicit DaqpDenseSolver(DenseQpOptions options) : options_(options) {}

  DenseQpResult solve(const DenseQp & qp, const Eigen::VectorXd * warm) override
  {
    check_dense_qp(qp);
    if (!bounds_consistent(qp)) return inconsistent_bounds_result(qp);
    const auto t0 = std::chrono::steady_clock::now();
    const int n = static_cast<int>(qp.H.rows()), m = static_cast<int>(qp.A.rows());
    H_ = qp.H;
    f_ = qp.g;
    A_ = qp.A;  // row-major copy
    upper_.resize(n + m);
    lower_.resize(n + m);
    for (int j = 0; j < n; ++j) {
      upper_[j] = daqp_bound(qp.ub(j));
      lower_[j] = daqp_bound(qp.lb(j));
    }
    for (int i = 0; i < m; ++i) {
      upper_[n + i] = daqp_bound(qp.uA(i));
      lower_[n + i] = daqp_bound(qp.lA(i));
    }
    const bool same_shape = n == last_n_ && m == last_m_;
    sense_.assign(n + m, 0);
    DAQPProblem problem = {n, n + m, n, H_.data(), f_.data(), A_.data(), upper_.data(), lower_.data(),
                           sense_.data(), nullptr, 0, 0};
    if (warm != nullptr && warm->size() == n) {
      if (same_shape) {
        // the last working set: active and lower/upper only (immutability is the new problem's to decide)
        for (int i = 0; i < n + m; ++i) sense_[i] = last_sense_[i] & (DAQP_ACTIVE + 2);
      } else {
        Eigen::VectorXd start = *warm;
        daqp_primal_init_active(&problem, start.data());
      }
    }

    DAQPSettings settings;
    daqp_default_settings(&settings);
    settings.primal_tol = options_.tolerance;
    DenseQpResult r;
    r.x = Eigen::VectorXd::Zero(n);
    lam_.assign(n + m, 0.0);
    DAQPResult result{};
    result.x = r.x.data();
    result.lam = lam_.data();
    // daqp_quadprog without equality elimination, so the working set keeps the problem's indices.
    DAQPWorkspace work;
    work.settings = &settings;
    c_float setup_time = 0.0;
    const int flag = setup_daqp_main(&problem, &work, &setup_time, DAQP_UPDATE_unconstrained);
    result.exitflag = flag;
    if (flag >= 0) {
      daqp_solve(&result, &work);
      last_sense_.assign(work.sense, work.sense + n + m);
      last_n_ = n;
      last_m_ = m;
      work.settings = nullptr;  // ours, not DAQP's to free
      free_daqp_workspace(&work);
      free_daqp_ldp(&work);
    }
    r.status = daqp_status(result.exitflag);
    r.iterations = result.iter;
    r.ok = (result.exitflag == DAQP_EXIT_OPTIMAL || result.exitflag == DAQP_EXIT_SOFT_OPTIMAL) && r.x.allFinite();
    if (!r.x.allFinite()) r.x.setZero();
    r.solve_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return r;
  }

private:
  DenseQpOptions options_;
  Eigen::MatrixXd H_;
  Eigen::VectorXd f_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_;
  std::vector<double> upper_, lower_, lam_;
  std::vector<int> sense_, last_sense_;
  int last_n_ = -1, last_m_ = -1;
};
}  // namespace

std::unique_ptr<DenseQpSolver> make_daqp_dense_solver(DenseQpOptions options)
{
  return std::make_unique<DaqpDenseSolver>(options);
}
}  // namespace mobile_manipulator_medium
