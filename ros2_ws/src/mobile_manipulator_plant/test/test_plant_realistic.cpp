// The plant in realistic mode (MuJoCo plant plan Task 6, spec Sections 4.1, 4.3, 4.4): the base through Unity's limiter
// on a force-limited planar velocity servo, the arm through the JTC sampler and the servos with MuJoCo's feedforward,
// command and state delays, physical contacts.
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "mobile_manipulator_plant/plant.hpp"

namespace mmp = mobile_manipulator_plant;

namespace
{
std::string source_share(const std::string & package) { return std::string(PACKAGES_SRC) + "/" + package; }
mmp::RobotProfile profile() { return mmp::load_profile(PROFILE_YAML, source_share); }
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

double true_x(const mmp::Plant & plant) { return plant.data()->qpos[plant.model()->jnt_qposadr[mj_name2id(plant.model(), mjOBJ_JOINT, "base_x")]]; }
}  // namespace

TEST(PlantRealistic, DriveReachesTheCommandedSpeed)
{
  const auto p = profile();
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  for (int k = 0; k < 100; ++k) plant.step(twist(plant.state().t, 0.3, 0.0), 0.02);  // 2 s
  EXPECT_NEAR(plant.state().v, 0.3, 0.01);
  EXPECT_NEAR(plant.state().omega, 0.0, 0.01);
  EXPECT_NEAR(plant.state().base.y, kYard.y, 0.01);  // no sideslip
}

TEST(PlantRealistic, CommandDelayShiftsTheResponse)
{
  double start[2];
  for (const double delay : {0.0, 0.04}) {
    auto p = profile();
    p.base_command_delay_s = delay;
    mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
    plant.reset({}, kYard, p.poses.at("carry"), 1);
    start[delay > 0.0] = -1.0;
    for (int k = 0; k < 100 && start[delay > 0.0] < 0.0; ++k) {
      plant.step(twist(plant.state().t, 0.3, 0.0), 0.002);
      if (true_x(plant) > kYard.x + 1e-6) start[delay > 0.0] = plant.state().t;
    }
  }
  EXPECT_NEAR(start[1] - start[0], 0.04, 0.002 + 1e-9);
}

TEST(PlantRealistic, StateDelayShiftsTheReportedState)
{
  auto p = profile();
  p.state_delay_s = 0.04;
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  std::vector<double> truth;
  for (int k = 0; k < 400; ++k) {
    plant.step(twist(plant.state().t, 0.3, 0.0), 0.002);
    truth.push_back(true_x(plant));
  }
  // the report at step k is the truth at step k - 20 (0.04 s)
  EXPECT_DOUBLE_EQ(plant.state().base.x, truth[truth.size() - 21]);
}

TEST(PlantRealistic, ArmHoldsItsPoseAtRest)
{
  const auto p = profile();
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  for (int k = 0; k < 100; ++k) plant.step(twist(plant.state().t, 0.0, 0.0), 0.02);
  EXPECT_LT((plant.state().q - p.poses.at("carry")).cwiseAbs().maxCoeff(), 2e-3);
}

TEST(PlantRealistic, FeedforwardSwitchesOff)
{
  double sag[2];
  for (const bool ff : {true, false}) {
    auto p = profile();
    p.gravity_ff = ff;
    p.motion_ff = ff;
    mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
    plant.reset({}, kYard, p.poses.at("carry"), 1);
    for (int k = 0; k < 100; ++k) plant.step(twist(plant.state().t, 0.0, 0.0), 0.02);
    sag[ff] = (plant.state().q - p.poses.at("carry")).cwiseAbs().maxCoeff();
  }
  EXPECT_GT(sag[0], 2.0 * sag[1]);
}

TEST(PlantRealistic, ArmFollowsAStream)
{
  const auto p = profile();
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  mmp::PlantCommand c = twist(0.0, 0.0, 0.0);
  c.arm.joint_velocity.reset();
  c.arm.start_time = 0.0;
  const mmp::Vec6 q0 = p.poses.at("carry");
  mmp::Vec6 q1 = q0;
  q1(0) += 0.5;  // the pan 0.5 rad in 1 s, from rest to rest
  c.arm.points = {{0.0, q0, mmp::Vec6::Zero()}, {1.0, q1, mmp::Vec6::Zero()}};
  for (int k = 0; k < 100; ++k) plant.step(c, 0.02);  // 2 s: the move and a second at rest
  EXPECT_NEAR(plant.state().q(0), q1(0), 2e-3);
}

TEST(PlantRealistic, SideslipUnderArmSwingIsSmall)
{
  const auto p = profile();
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  for (int k = 0; k < 150; ++k) {  // the pan swings at full speed while the base is told to stand
    mmp::Vec6 qd = mmp::Vec6::Zero();
    qd(0) = (k / 25) % 2 ? -1.5 : 1.5;
    plant.step(twist(plant.state().t, 0.0, 0.0, qd), 0.02);
  }
  EXPECT_LT(std::hypot(plant.state().base.x - kYard.x, plant.state().base.y - kYard.y), 0.02);
}

TEST(PlantRealistic, ContactsPushBack)
{
  const auto p = profile();
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
  mmp::ScenarioSetup s;
  s.name = "wall";
  s.boxes.push_back({"wall", Eigen::Vector3d(-4.2, -5.5, 0.2), Eigen::Vector3d(0.2, 4.0, 0.4), 0.0});  // face at x -4.3
  plant.reset(s, kYard, p.poses.at("home"), 1);
  for (int k = 0; k < 200; ++k) plant.step(twist(plant.state().t, 0.3, 0.0), 0.02);  // 4 s at 0.3 m/s into the wall
  EXPECT_LT(plant.state().base.x, -4.3 - 0.425 + 0.03);  // the chassis front (x + 0.425) stops at the wall
  EXPECT_FALSE(plant.ground_truth().contacts.empty());
}

TEST(PlantRealistic, SameSeedSameRun)
{
  const auto p = profile();
  mmp::Plant a(p, SITE_MODEL, mmp::PlantMode::Realistic), b(p, SITE_MODEL, mmp::PlantMode::Realistic);
  a.reset({}, kYard, p.poses.at("carry"), 9);
  b.reset({}, kYard, p.poses.at("carry"), 9);
  mmp::Vec6 qd;
  qd << 0.2, -0.1, 0.2, 0.0, 0.1, -0.2;
  for (int k = 0; k < 250; ++k) {
    const auto c = twist(a.state().t, 0.2 + 0.1 * std::sin(0.02 * k), 0.3 * std::cos(0.03 * k), qd * std::sin(0.05 * k));
    a.step(c, 0.02);
    b.step(c, 0.02);
  }
  EXPECT_EQ(std::memcmp(a.data()->qpos, b.data()->qpos, sizeof(mjtNum) * a.model()->nq), 0);
  EXPECT_EQ(std::memcmp(a.data()->qvel, b.data()->qvel, sizeof(mjtNum) * a.model()->nv), 0);
}

TEST(PlantRealistic, StaleCommandRampsToZero)
{
  // a command sent once (stamp 0) and not renewed: after Unity's 0.5 s watchdog the base brakes to a stop
  auto p = profile();
  p.base_command_delay_s = 0.0;
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  const auto once = twist(0.0, 0.3, 0.0);
  for (int k = 0; k < 25; ++k) plant.step(once, 0.02);  // 0.5 s: driving
  EXPECT_GT(plant.state().v, 0.2);
  for (int k = 0; k < 50; ++k) plant.step(once, 0.02);  // 1.5 s: stale since 0.5 s, braked at 0.8 m/s^2
  EXPECT_NEAR(plant.state().v, 0.0, 0.01);
}

TEST(PlantRealistic, StateDelayIsQuantizedToThePhysicsStep)
{
  // a 30 ms state delay read at the closed loop's 20 ms step: the truth 30 ms ago, not 40
  auto p = profile();
  p.state_delay_s = 0.03;
  mmp::Plant coarse(p, SITE_MODEL, mmp::PlantMode::Realistic), fine(p, SITE_MODEL, mmp::PlantMode::Realistic);
  coarse.reset({}, kYard, p.poses.at("carry"), 1);
  fine.reset({}, kYard, p.poses.at("carry"), 1);
  std::vector<double> truth{true_x(fine)};
  for (int k = 0; k < 500; ++k) {
    fine.step(twist(0.0, 0.3, 0.0), 0.002);
    truth.push_back(true_x(fine));
  }
  for (int k = 0; k < 50; ++k) coarse.step(twist(0.0, 0.3, 0.0), 0.02);
  EXPECT_DOUBLE_EQ(coarse.state().base.x, truth[500 - 15]);  // 1.0 s - 0.03 s
}

TEST(PlantRealistic, ReportsTheStartUntilTheStateDelayElapses)
{
  auto p = profile();
  p.state_delay_s = 0.04;
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  plant.step(twist(0.0, 0.3, 0.0), 0.02);
  EXPECT_EQ(plant.state().t, 0.0);
  EXPECT_EQ(plant.state().base.x, kYard.x);
}

TEST(PlantRealistic, ArmAndBaseDelaysAreSeparate)
{
  auto p = profile();
  p.base_command_delay_s = 0.02;
  p.arm_command_delay_s = 0.06;
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  mmp::Vec6 qd = mmp::Vec6::Zero();
  qd(0) = 1.0;
  double base_start = -1, arm_start = -1;
  const double q0 = plant.state().q(0);
  for (int k = 0; k < 60; ++k) {
    plant.step(twist(plant.state().t, 0.3, 0.0, qd), 0.002);
    if (base_start < 0 && true_x(plant) > kYard.x + 1e-6) base_start = plant.state().t;
    if (arm_start < 0 && plant.state().q(0) > q0 + 1e-6) arm_start = plant.state().t;
  }
  EXPECT_NEAR(base_start, 0.02 + 0.002, 0.0021);
  EXPECT_NEAR(arm_start, 0.06 + 0.002, 0.0041);  // the servo's first response
}

TEST(PlantRealistic, NoPushAgainstAJointLimit)
{
  const auto p = profile();
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  mmp::Vec6 qd = mmp::Vec6::Zero();
  qd(1) = 1.0;  // the shoulder lift into its 1.745 rad limit and held there by the command
  for (int k = 0; k < 150; ++k) plant.step(twist(plant.state().t, 0.0, 0.0, qd), 0.02);
  const int a = mj_name2id(plant.model(), mjOBJ_ACTUATOR, "servo/shoulder_lift_joint");
  // a velocity target past the limit would add kd * 1 rad/s = 360 N m (capped at 160): only the hold torque remains
  EXPECT_LT(std::abs(plant.data()->actuator_force[a]), 120.0);
}

TEST(PlantRealistic, TheReferenceIsNotDelayedLikeTheJoints)
{
  // the controller's own reference is current; only the measured joints arrive after the state delay (Unity's JTC
  // compares its reference with joint states that came through the transport)
  auto p = profile();
  p.state_delay_s = 0.04;
  mmp::Plant plant(p, SITE_MODEL, mmp::PlantMode::Realistic);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  mmp::Vec6 qd = mmp::Vec6::Zero();
  qd(0) = 0.5;
  for (int k = 0; k < 50; ++k) plant.step(twist(plant.state().t, 0.0, 0.0, qd), 0.02);  // 1 s
  const double start = p.poses.at("carry")(0);
  EXPECT_NEAR(plant.state().q_ref(0), start + 0.5 * (1.0 - p.arm_command_delay_s), 0.002);  // now
  EXPECT_LT(plant.state().q(0), plant.state().q_ref(0) - 0.5 * 0.03);                        // 40 ms behind and more
}
