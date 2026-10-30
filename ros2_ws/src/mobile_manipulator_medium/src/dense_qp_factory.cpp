#include "mobile_manipulator_medium/dense_qp.hpp"

#include <stdexcept>

namespace mobile_manipulator_medium
{
void check_dense_qp(const DenseQp & qp)
{
  const auto n = qp.H.rows(), m = qp.A.rows();
  if (qp.H.cols() != n || qp.g.size() != n || qp.lb.size() != n || qp.ub.size() != n || qp.A.cols() != n ||
      qp.lA.size() != m || qp.uA.size() != m)
    throw std::invalid_argument("DenseQp: inconsistent shapes");
}

bool bounds_consistent(const DenseQp & qp)
{
  for (int j = 0; j < qp.lb.size(); ++j)
    if (qp.lb(j) > qp.ub(j)) return false;
  for (int i = 0; i < qp.lA.size(); ++i)
    if (qp.lA(i) > qp.uA(i)) return false;
  return true;
}

DenseQpResult inconsistent_bounds_result(const DenseQp & qp)
{
  DenseQpResult r;
  r.status = "inconsistent bounds";
  r.x = Eigen::VectorXd::Zero(qp.H.rows());
  return r;
}

std::unique_ptr<DenseQpSolver> make_dense_solver(const std::string & backend, DenseQpOptions options)
{
  if (backend == "hpipm") return make_hpipm_dense_solver(options);
  if (backend == "osqp") return make_osqp_dense_solver(options);
  if (backend == "daqp") return make_daqp_dense_solver(options);
  throw std::invalid_argument("make_dense_solver: backend must be hpipm, osqp or daqp, not " + backend);
}
}  // namespace mobile_manipulator_medium
