#pragma once
// The QP backend of a wholebody_mpc.yaml configuration (solver.backend, Phase 4B-B spec Section 4.2).
#include <memory>

#include "mobile_manipulator_mpc/formulation.hpp"
#include "mobile_manipulator_mpc/qp.hpp"

namespace mobile_manipulator_mpc
{
// OSQP with solver.eps (absolute and relative) and solver.max_iter, or HPIPM with solver.hpipm and solver.eps as its
// tolerance. Throws std::invalid_argument for another backend.
std::unique_ptr<QpSolver> make_solver(const MpcConfig & config);
}  // namespace mobile_manipulator_mpc
