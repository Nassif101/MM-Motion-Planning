// DAQP as vendored: min 0.5 (x1^2 + x2^2) - x1 - x2 subject to x1 + x2 <= 1 and x1 <= 0.3 (a simple bound). The bound
// is active, and the general row then gives x2 = 0.7.
#include <gtest/gtest.h>

extern "C" {
#include "api.h"
}

TEST(DaqpVendor, SolvesASmallQp)
{
  double H[] = {1.0, 0.0, 0.0, 1.0};
  double f[] = {-1.0, -1.0};
  double A[] = {1.0, 1.0};  // row-major, the general rows after the simple bounds
  double upper[] = {0.3, 1e30, 1.0};
  double lower[] = {-1e30, -1e30, -1e30};
  DAQPProblem qp = {2, 3, 2, H, f, A, upper, lower, nullptr, nullptr, 0, 0};
  double x[2], lam[3];
  DAQPResult result;
  result.x = x;
  result.lam = lam;
  daqp_quadprog(&result, &qp, nullptr);
  EXPECT_EQ(result.exitflag, 1);  // DAQP_EXIT_OPTIMAL
  EXPECT_NEAR(x[0], 0.3, 1e-9);
  EXPECT_NEAR(x[1], 0.7, 1e-9);
}
