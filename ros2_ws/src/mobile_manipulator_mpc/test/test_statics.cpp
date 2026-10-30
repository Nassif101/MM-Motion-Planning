// The arm's static gravity torque (Phase 5A Task 2): the medium layer's torque constraint and carry_pose_search.
#include <gtest/gtest.h>

#include <random>

#include "mobile_manipulator_mpc/statics.hpp"
#include "test_support.hpp"

namespace
{
using namespace mpc_test;
using Joints = Eigen::Matrix<double, 6, 1>;

const StaticTorqueModel & statics()
{
  static const StaticTorqueModel model(mpc_test::urdf(), mpc_test::payload());
  return model;
}

Joints pose(const std::string & name)
{
  Joints q;
  for (int i = 0; i < 6; ++i) q(i) = mpc_test::payload().at("poses_rad").at(name).at(i).get<double>();
  return q;
}

TEST(Statics, PanAxisCarriesNoGravityTorque)
{
  for (const char * name : {"home", "vertical_carry", "level_extension", "carry", "vertical_carry_right"})
    EXPECT_NEAR(statics().torque(pose(name))(0), 0.0, 1e-9) << name;
}

TEST(Statics, MatchesCarryPoseSearchAtTheNamedPoses)
{
  // carry_pose_search's printed table (0.1 N m) before it was moved onto StaticTorqueModel.
  const Joints carry = statics().torque(pose("carry"));
  EXPECT_NEAR(carry(1), 19.3, 0.05);
  EXPECT_NEAR(carry(2), -27.2, 0.05);
  EXPECT_NEAR(carry(4), 0.0, 0.05);
  const Joints extension = statics().torque(pose("level_extension"));
  EXPECT_NEAR(extension(1), 95.7, 0.05);
  EXPECT_NEAR(extension(2), 39.2, 0.05);
  EXPECT_NEAR(statics().torque(Joints::Zero()).norm(), 0.0, 0.05);  // home: everything above the shoulder
}

TEST(Statics, FoldShoulderAbout22Nm)
{
  Joints fold;
  fold << 0.0, 1.236, -2.156, 0.0, 0.920, 0.0;
  const double shoulder = std::abs(statics().torque(fold)(1));
  EXPECT_GE(shoulder, 20.0);
  EXPECT_LE(shoulder, 24.0);
}

TEST(Statics, JacobianMatchesFiniteDifferences)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(-1.5, 1.5);
  for (int trial = 0; trial < 10; ++trial) {
    Joints q;
    for (int i = 0; i < 6; ++i) q(i) = u(rng);
    const Eigen::Matrix<double, 6, 6> J = statics().jacobian(q);
    for (int j = 0; j < 6; ++j) {
      Joints dq = Joints::Zero();
      dq(j) = 1e-5;
      const Joints fd = (statics().torque(q + dq) - statics().torque(q - dq)) / 2e-5;
      for (int i = 0; i < 6; ++i) EXPECT_NEAR(J(i, j), fd(i), 1e-4) << "trial " << trial << " d" << i << "/d" << j;
    }
  }
}
}  // namespace
