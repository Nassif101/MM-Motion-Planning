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

std::unique_ptr<DenseQpSolver> make_dense_solver(const std::string & backend)
{
  if (backend == "hpipm") return make_hpipm_dense_solver();
  if (backend == "osqp") return make_osqp_dense_solver();
  if (backend == "daqp") return make_daqp_dense_solver();
  throw std::invalid_argument("make_dense_solver: backend must be hpipm, osqp or daqp, not " + backend);
}
}  // namespace mobile_manipulator_medium
