#pragma once
// The QP backend and per-cycle solve options of wholebody_mpc.yaml `solver:` (Phase 4B-B spec Section 4). Every
// default is the frozen B5 (tag b5-frozen-2026-10-22): OSQP, warm-started, no deadline cap. No solver library here.
#include <string>

namespace mobile_manipulator_mpc
{
// HPIPM's options (`solver.hpipm`; the tolerance is `solver.eps`).
struct HpipmOptions
{
  std::string mode = "balance";  // speed, balance or robust (HPIPM's ocp_qp_ipm_arg modes)
  int max_iter = 50;
  double tol = 1e-3;
  int partial_condensing_blocks = 0;  // 0: none; otherwise the N2 blocks of HPIPM's partial condensing
  bool warm_start = false;
};

struct SolverConfig
{
  std::string backend = "osqp";  // osqp or hpipm
  bool warm_start = true;        // start each solve from the previous solution (the frozen behaviour)
  // Each solve gets the time left in the cycle's period less publish_margin_s; running out is a solver failure
  // (the shifted-plan fallback) instead of a missed deadline.
  bool deadline_cap = false;
  double publish_margin_s = 0.002;
  int osqp_adaptive_rho_interval = 100;  // OSQP's rho adaptation every this many iterations (solver.adaptive_rho_interval)
  HpipmOptions hpipm;
};

// The preparation/feedback split's limit on the measured state's deviation from the prediction it was prepared
// for (`rti_max_deviation`); beyond it the cycle prepares again in-cycle.
struct RtiDeviation
{
  double base_m = 0.05, yaw_rad = 0.05, joint_rad = 0.05;
};
}  // namespace mobile_manipulator_mpc
