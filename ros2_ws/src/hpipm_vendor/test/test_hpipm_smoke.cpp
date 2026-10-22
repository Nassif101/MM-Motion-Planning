// HPIPM as vendored: x+ = x + u over two stages, cost 0.5 (x^2 + u^2) per stage and 0.5 x2^2, x0 = 1 (an equality
// box). The Riccati recursion gives u0 = -0.6, u1 = -0.2 (p2 = 1, p1 = 1.5, p0 = 1.6).
#include <gtest/gtest.h>

#include <vector>

#include "hpipm_d_ocp_qp.h"
#include "hpipm_d_ocp_qp_dim.h"
#include "hpipm_d_ocp_qp_ipm.h"
#include "hpipm_d_ocp_qp_sol.h"

TEST(HpipmVendor, SolvesATwoStageOcpQp)
{
  const int N = 2;
  int nx[] = {1, 1, 1}, nu[] = {1, 1, 0}, nbx[] = {1, 0, 0}, nbu[] = {0, 0, 0}, ng[] = {0, 0, 0}, ns[] = {0, 0, 0};
  std::vector<char> dim_mem(d_ocp_qp_dim_memsize(N));
  d_ocp_qp_dim dim;
  d_ocp_qp_dim_create(N, &dim, dim_mem.data());
  d_ocp_qp_dim_set_all(nx, nu, nbx, nbu, ng, ns, &dim);

  std::vector<char> qp_mem(d_ocp_qp_memsize(&dim));
  d_ocp_qp qp;
  d_ocp_qp_create(&dim, &qp, qp_mem.data());
  double one = 1.0, zero = 0.0;
  for (int k = 0; k < N; ++k) {
    d_ocp_qp_set_A(k, &one, &qp);
    d_ocp_qp_set_B(k, &one, &qp);
    d_ocp_qp_set_b(k, &zero, &qp);
    d_ocp_qp_set_Q(k, &one, &qp);
    d_ocp_qp_set_R(k, &one, &qp);
    d_ocp_qp_set_S(k, &zero, &qp);
    d_ocp_qp_set_q(k, &zero, &qp);
    d_ocp_qp_set_r(k, &zero, &qp);
  }
  d_ocp_qp_set_Q(N, &one, &qp);
  d_ocp_qp_set_q(N, &zero, &qp);
  int idx = 0;
  d_ocp_qp_set_idxbx(0, &idx, &qp);
  d_ocp_qp_set_lbx(0, &one, &qp);
  d_ocp_qp_set_ubx(0, &one, &qp);

  std::vector<char> sol_mem(d_ocp_qp_sol_memsize(&dim));
  d_ocp_qp_sol sol;
  d_ocp_qp_sol_create(&dim, &sol, sol_mem.data());
  std::vector<char> arg_mem(d_ocp_qp_ipm_arg_memsize(&dim));
  d_ocp_qp_ipm_arg arg;
  d_ocp_qp_ipm_arg_create(&dim, &arg, arg_mem.data());
  d_ocp_qp_ipm_arg_set_default(BALANCE, &arg);
  std::vector<char> ws_mem(d_ocp_qp_ipm_ws_memsize(&dim, &arg));
  d_ocp_qp_ipm_ws ws;
  d_ocp_qp_ipm_ws_create(&dim, &arg, &ws, ws_mem.data());

  d_ocp_qp_ipm_solve(&qp, &sol, &arg, &ws);
  int status = -1;
  d_ocp_qp_ipm_get_status(&ws, &status);
  EXPECT_EQ(status, 0);
  double u0 = 0.0, u1 = 0.0, x1 = 0.0;
  d_ocp_qp_sol_get_u(0, &sol, &u0);
  d_ocp_qp_sol_get_u(1, &sol, &u1);
  d_ocp_qp_sol_get_x(1, &sol, &x1);
  EXPECT_NEAR(u0, -0.6, 1e-5);  // the IPM's stationarity tolerance is 1e-6 in BALANCE mode
  EXPECT_NEAR(u1, -0.2, 1e-5);
  EXPECT_NEAR(x1, 0.4, 1e-5);
}
