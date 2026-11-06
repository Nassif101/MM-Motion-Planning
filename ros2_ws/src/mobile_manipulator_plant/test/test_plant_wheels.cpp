// The URDF-based realistic mode (MuJoCo plant amendment 2026-11-06, plan Task 7b): the base stands on its wheels
// (free body, driven wheel hinges, ground friction), the arm's gains by the bandwidth rule with the URDF's passive terms.
#include <gtest/gtest.h>

#include <cmath>

#include "mobile_manipulator_plant/plant.hpp"

namespace mmp = mobile_manipulator_plant;

namespace
{
std::string source_share(const std::string & package) { return std::string(PACKAGES_SRC) + "/" + package; }
// The default profile without delays: these tests check mechanisms
mmp::RobotProfile profile()
{
  auto p = mmp::load_profile(PROFILE_YAML, source_share);
  p.base_command_delay_s = p.arm_command_delay_s = p.base_state_delay_s = p.arm_state_delay_s = 0.0;
  return p;
}
const mmp::Pose2 kYard{-5.0, -5.5, 0.0};

mmp::PlantCommand twist(double t, double v, double w, const mmp::Vec6 & qd = mmp::Vec6::Zero())
{
  mmp::PlantCommand c;
  c.stamp = t;
  c.v = v;
  c.omega = w;
  c.arm.joint_velocity = qd;
  return c;
}

void run(mmp::Plant & plant, double seconds, double v, double w, const mmp::Vec6 & qd = mmp::Vec6::Zero())
{
  for (int k = 0; k < static_cast<int>(std::lround(seconds / 0.02)); ++k) plant.step(twist(plant.state().t, v, w, qd), 0.02);
}
}  // namespace

TEST(PlantWheels, StandsStill)
{
  const auto p = profile();
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  run(plant, 5.0, 0.0, 0.0);
  EXPECT_NEAR(plant.state().base.x, kYard.x, 0.001);
  EXPECT_NEAR(plant.state().base.y, kYard.y, 0.001);
  EXPECT_NEAR(plant.state().base.yaw, kYard.yaw, 0.001);
}

TEST(PlantWheels, DrivesStraight)
{
  const auto p = profile();
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  run(plant, 4.0, 0.3, 0.0);
  EXPECT_NEAR(plant.state().v, 0.3, 0.009);
  EXPECT_NEAR(plant.state().base.y, kYard.y, 0.01);
  EXPECT_GT(plant.state().base.x, kYard.x + 0.8);
}

TEST(PlantWheels, TurnsInPlace)
{
  const auto p = profile();
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  run(plant, 6.0, 0.0, 0.4);
  EXPECT_GT(plant.state().omega, 0.25);
  EXPECT_LT(std::abs(plant.state().v), 0.03);
}

TEST(PlantWheels, StopsWithinTheDecelerationLimit)
{
  const auto p = profile();
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  run(plant, 3.0, 0.3, 0.0);
  run(plant, 0.3 / p.base_limits.decel + 0.2, 0.0, 0.0);
  EXPECT_LT(std::abs(plant.state().v), 0.01);
}

TEST(PlantWheels, WheelGroundContactIsNotACollision)
{
  const auto p = profile();
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  run(plant, 0.5, 0.0, 0.0);
  ASSERT_GT(plant.data()->ncon, 0);  // the wheels stand on the ground
  EXPECT_TRUE(plant.ground_truth().contacts.empty());
}

TEST(PlantWheels, ReportsPoseAndTwistFromTheFreeBase)
{
  const auto p = profile();
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
  const mmp::Pose2 start{-5.0, -5.5, 0.5};
  plant.reset({}, start, p.poses.at("carry"), 1);
  EXPECT_NEAR(plant.state().base.yaw, 0.5, 1e-9);
  run(plant, 3.0, 0.3, 0.0);
  const double dx = plant.state().base.x - start.x, dy = plant.state().base.y - start.y;
  EXPECT_NEAR(std::atan2(dy, dx), 0.5, 0.02);  // it drove along its heading
  EXPECT_NEAR(plant.state().v, 0.3, 0.009);    // in the body frame
}

TEST(PlantWheels, IdealModeStaysPlanar)
{
  const auto p = profile();
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Ideal);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  EXPECT_GE(mj_name2id(plant.model(), mjOBJ_JOINT, "base_x"), 0);
  EXPECT_LT(mj_name2id(plant.model(), mjOBJ_JOINT, "base_free"), 0);
}

TEST(ArmGains, FromTheBandwidthRule)
{
  const auto p = profile();
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  // the joint-space inertia's diagonal at the rule's pose, armature included
  const mjModel * m = plant.model();
  mjData * d = mj_makeData(m);
  for (int i = 0; i < 6; ++i) d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, p.arm_joints[i].c_str())]] = p.poses.at(p.gain_rule.pose)(i);
  mj_fwdPosition(m, d);
  std::vector<mjtNum> M(static_cast<size_t>(m->nv) * m->nv);
  mj_fullM(m, d, M.data());
  const double w = 2.0 * M_PI * p.gain_rule.bandwidth_hz;
  for (int i = 0; i < 6; ++i) {
    const int dof = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, p.arm_joints[i].c_str())];
    const double inertia = M[static_cast<size_t>(dof) * m->nv + dof];
    EXPECT_NEAR(plant.profile().kp(i), inertia * w * w, 1e-6 * inertia * w * w) << p.arm_joints[i];
    EXPECT_NEAR(plant.profile().kd(i), 2.0 * p.gain_rule.zeta * inertia * w, 1e-6 * inertia * w) << p.arm_joints[i];
    const int a = mj_name2id(m, mjOBJ_ACTUATOR, ("servo/" + p.arm_joints[i]).c_str());
    EXPECT_DOUBLE_EQ(m->actuator_gainprm[a * mjNGAIN], plant.profile().kp(i));
  }
  mj_deleteData(d);
}

TEST(PlantWheels, ArmHoldsAndTracksWithTheUrdfFriction)
{
  const auto p = profile();
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  run(plant, 2.0, 0.0, 0.0);
  EXPECT_LT((plant.state().q - plant.state().q_ref).cwiseAbs().maxCoeff(), 0.002);  // holds carry
  mmp::Vec6 qd = mmp::Vec6::Zero();
  qd(0) = 0.3;
  double worst = 0.0;
  for (int k = 0; k < 50; ++k) {
    plant.step(twist(plant.state().t, 0.0, 0.0, qd), 0.02);
    worst = std::max(worst, std::abs(plant.state().q(0) - plant.state().q_ref(0)));
  }
  EXPECT_LT(worst, 0.01);  // tracks a 0.3 rad/s pan ramp
}
