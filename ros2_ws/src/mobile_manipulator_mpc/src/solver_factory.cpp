#include "mobile_manipulator_mpc/solver_factory.hpp"

#include <stdexcept>

#include "mobile_manipulator_mpc/osqp_solver.hpp"

namespace mobile_manipulator_mpc
{
std::unique_ptr<QpSolver> make_solver(const MpcConfig & config)
{
  if (config.solver.backend == "osqp") {
    OsqpOptions options;
    options.eps_abs = options.eps_rel = config.solver_eps;
    options.max_iter = config.solver_max_iter;
    return std::make_unique<OsqpSolver>(options);
  }
  throw std::invalid_argument("make_solver: no backend " + config.solver.backend);
}
}  // namespace mobile_manipulator_mpc
