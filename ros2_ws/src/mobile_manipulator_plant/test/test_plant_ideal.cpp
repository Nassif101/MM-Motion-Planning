// The plant in ideal mode (MuJoCo plant plan Task 4, spec Section 4.2).
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <stdexcept>

#include "mobile_manipulator_plant/model_builder.hpp"
#include "mobile_manipulator_plant/plant.hpp"

namespace mmp = mobile_manipulator_plant;

namespace
{
std::string source_share(const std::string & package) { return std::string(PACKAGES_SRC) + "/" + package; }
mmp::RobotProfile profile() { return mmp::load_profile(PROFILE_YAML, source_share); }

// The open delivery yard: nothing within 3 m of (-5, -5.5).
const mmp::Pose2 kYard{-5.0, -5.5, 0.0};

mmp::PlantCommand twist(double v, double omega, const mmp::Vec6 & qd = mmp::Vec6::Zero())
{
  mmp::PlantCommand c;
  c.v = v;
  c.omega = omega;
  c.arm.joint_velocity = qd;
  return c;
}

mmp::SceneBox box(const std::string & name, Eigen::Vector3d center, Eigen::Vector3d size, double yaw = 0.0)
{
  return {name, center, size, yaw};
}
}  // namespace

class PlantIdeal : public ::testing::Test
{
protected:
  mmp::RobotProfile p = profile();
  mmp::Plant plant{p, SITE_MODEL, mmp::PlantMode::Ideal};
};

TEST_F(PlantIdeal, StepRejectsNonMultipleDt)
{
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  EXPECT_THROW(plant.step(twist(0.1, 0.0), 0.003), std::invalid_argument);
  EXPECT_NO_THROW(plant.step(twist(0.1, 0.0), 0.02));
  EXPECT_NEAR(plant.state().t, 0.02, 1e-12);
}

TEST_F(PlantIdeal, ArcIntegrationIsExact)
{
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  for (int k = 0; k < 100; ++k) plant.step(twist(0.3, 0.4), 0.02);  // 2 s
  const double r = 0.3 / 0.4, th = 0.4 * 2.0;
  EXPECT_NEAR(plant.state().base.x, kYard.x + r * std::sin(th), 1e-9);
  EXPECT_NEAR(plant.state().base.y, kYard.y + r * (1.0 - std::cos(th)), 1e-9);
  EXPECT_NEAR(plant.state().base.yaw, th, 1e-9);
  EXPECT_DOUBLE_EQ(plant.state().v, 0.3);
  EXPECT_DOUBLE_EQ(plant.state().omega, 0.4);
}

TEST_F(PlantIdeal, ArmFollowsJointVelocitiesWithinLimits)
{
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  mmp::Vec6 qd = mmp::Vec6::Zero();
  qd(0) = 0.5;
  qd(1) = 2.0;  // drives the shoulder into its 1.745 rad limit
  for (int k = 0; k < 100; ++k) plant.step(twist(0.0, 0.0, qd), 0.02);
  EXPECT_NEAR(plant.state().q(0), p.poses.at("carry")(0) + 1.0, 1e-9);
  EXPECT_NEAR(plant.state().q(1), 1.74532925199, 1e-9);
}

TEST_F(PlantIdeal, IdealBreakawayHoldsBaseFromRest)
{
  plant.set_ideal_breakaway(true);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  for (int k = 0; k < 50; ++k) plant.step(twist(0.03, 0.05), 0.02);  // below 0.0375 m/s and 0.07 rad/s
  EXPECT_DOUBLE_EQ(plant.state().base.x, kYard.x);
  EXPECT_TRUE(plant.base_at_rest());
  plant.step(twist(0.05, 0.0), 0.02);  // above: starts
  EXPECT_GT(plant.state().base.x, kYard.x);
  for (int k = 0; k < 10; ++k) plant.step(twist(0.03, 0.0), 0.02);  // moving: a slow command now executes
  EXPECT_NEAR(plant.state().v, 0.03, 1e-12);
}

TEST_F(PlantIdeal, BreakawayIsJudgedOncePerCommand)
{
  // the kinematic plant judges "from rest" once per command: a slow command after motion executes for the whole step
  plant.set_ideal_breakaway(true);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  for (int k = 0; k < 10; ++k) plant.step(twist(0.3, 0.0), 0.02);
  const double x0 = plant.state().base.x;
  plant.step(twist(0.007, 0.0), 0.02);  // below the rest threshold 0.01 m/s, the base was moving
  EXPECT_NEAR(plant.state().base.x - x0, 0.007 * 0.02, 1e-12);
  EXPECT_DOUBLE_EQ(plant.state().v, 0.007);
  plant.step(twist(0.007, 0.0), 0.02);  // now at rest: below the breakaway, it stays
  EXPECT_NEAR(plant.state().base.x - x0, 0.007 * 0.02, 1e-12);
}

TEST_F(PlantIdeal, ReportedJointVelocityIsTheCommandUnlessClamped)
{
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  mmp::Vec6 qd;
  qd << 0.123456789, -0.2, 0.3, 0.0, 0.1, -0.1;
  plant.step(twist(0.0, 0.0, qd), 0.02);
  for (int i = 0; i < 6; ++i) EXPECT_EQ(plant.state().qd(i), qd(i)) << i;  // bit for bit
}

TEST_F(PlantIdeal, TinyYawRateStillTurns)
{
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  plant.step(twist(0.0, 1e-13), 0.02);
  EXPECT_DOUBLE_EQ(plant.state().base.yaw, 1e-13 * 0.02);
}

TEST_F(PlantIdeal, MoveBodyRejectsMovers)
{
  mmp::ScenarioSetup s;
  s.name = "crossing";
  mmp::MoverSpec mover;
  mover.name = "worker";
  mover.start = {-2.0, -3.0};
  mover.end = {-2.0, -8.0};
  mover.size = {0.5, 0.5, 1.8};
  mover.speed_mps = 0.8;
  mover.trigger_distance_m = 2.5;
  s.movers.push_back(mover);
  plant.reset(s, kYard, p.poses.at("carry"), 1);
  EXPECT_THROW(plant.move_body("mover/worker", {0.0, 0.0, 0.9}, 0.0), std::invalid_argument);  // its motion owns it
}

TEST_F(PlantIdeal, ReservedBodiesAppearWhereMoved)
{
  // an obstacle appearing mid-run (spec Section 4): a reserved mocap body, parked far away until moved
  const mmp::Pose2 far{-20.0, -15.0, 0.0};
  plant.reset({}, far, p.poses.at("carry"), 1);
  EXPECT_FALSE(std::isfinite(plant.ground_truth().min_clearance_m));
  plant.move_body("reserved/0", {far.x + 1.5, far.y, 0.9}, 0.3);
  const mmp::GroundTruth truth = plant.ground_truth();
  ASSERT_TRUE(std::isfinite(truth.min_clearance_m));
  EXPECT_LT(truth.min_clearance_m, 1.2);
  EXPECT_EQ(truth.other_geom, "reserved/0");
  EXPECT_THROW(plant.move_body("reserved/9", {0.0, 0.0, 0.9}, 0.0), std::invalid_argument);
  const auto bodies = plant.reserved_positions();
  ASSERT_EQ(bodies.size(), 1u);  // only the moved one
  EXPECT_EQ(bodies[0].name, "reserved/0");
  EXPECT_NEAR(bodies[0].yaw, 0.3, 1e-9);
}

TEST_F(PlantIdeal, ClearanceIsInfiniteFarFromEverything)
{
  plant.reset({}, {-20.0, -15.0, 0.0}, p.poses.at("carry"), 1);
  const mmp::GroundTruth truth = plant.ground_truth();
  if (std::isfinite(truth.min_clearance_m)) EXPECT_LT(truth.min_clearance_m, 2.0) << truth.other_geom;  // never the cap
}

TEST_F(PlantIdeal, ContactsLoggedNotPushed)
{
  mmp::ScenarioSetup s;
  s.name = "wall";
  // a low wall (only the chassis can reach it) whose face is at x -4.3; the chassis front reaches -4.275
  s.boxes.push_back(box("wall", {-4.2, -5.5, 0.2}, {0.2, 4.0, 0.4}));
  plant.reset(s, kYard, p.poses.at("home"), 1);
  for (int k = 0; k < 50; ++k) plant.step(twist(0.3, 0.0), 0.02);  // 1 s at 0.3 m/s
  EXPECT_NEAR(plant.state().base.x, kYard.x + 0.3, 1e-9);
  const mmp::GroundTruth truth = plant.ground_truth();
  ASSERT_FALSE(truth.contacts.empty());
  EXPECT_EQ(truth.contacts.front().other_geom, "scenario/wall");
  EXPECT_LT(truth.min_clearance_m, 0.0);
}

TEST_F(PlantIdeal, ClearanceToNearestBox)
{
  mmp::ScenarioSetup s;
  s.name = "wall";
  // the front wheels reach x + 0.44 (axle x + 0.3, radius 0.14), ahead of the chassis face (x + 0.425): 0.485 m to a
  // low wall face at x + 0.925 (the wheels count, as in B5's clearance, though they have no contacts)
  s.boxes.push_back(box("wall", {kYard.x + 1.025, -5.5, 0.2}, {0.2, 4.0, 0.4}));
  plant.reset(s, kYard, p.poses.at("home"), 1);
  const mmp::GroundTruth truth = plant.ground_truth();
  EXPECT_NEAR(truth.min_clearance_m, 0.485, 1e-6);
  EXPECT_EQ(truth.other_geom, "scenario/wall");
  EXPECT_NE(truth.robot_geom.find("front_"), std::string::npos) << truth.robot_geom;
  EXPECT_NEAR(truth.witness_other.x(), kYard.x + 0.925, 1e-6);
}

TEST_F(PlantIdeal, ResetClearsState)
{
  mmp::ScenarioSetup first;
  first.name = "first";
  first.boxes.push_back(box("post", {0.0, -5.5, 1.0}, {0.2, 0.2, 2.0}));
  mmp::MoverSpec mover;
  mover.name = "worker";
  mover.start = {0.0, -3.0};
  mover.end = {0.0, -8.0};
  mover.size = {0.5, 0.5, 1.8};
  mover.speed_mps = 0.8;
  mover.trigger_distance_m = 5.0;
  first.movers.push_back(mover);
  plant.reset(first, kYard, p.poses.at("carry"), 1);
  for (int k = 0; k < 50; ++k) plant.step(twist(0.3, 0.0), 0.02);
  EXPECT_GE(mj_name2id(plant.model(), mjOBJ_GEOM, "scenario/post"), 0);
  EXPECT_EQ(plant.mover_positions().size(), 1u);
  plant.reset({}, kYard, p.poses.at("carry"), 1);
  EXPECT_LT(mj_name2id(plant.model(), mjOBJ_GEOM, "scenario/post"), 0);
  EXPECT_TRUE(plant.mover_positions().empty());
  EXPECT_EQ(plant.state().t, 0.0);
  EXPECT_DOUBLE_EQ(plant.state().base.x, kYard.x);
}

TEST_F(PlantIdeal, MoverWalksAfterTrigger)
{
  mmp::ScenarioSetup s;
  s.name = "crossing";
  mmp::MoverSpec mover;
  mover.name = "worker";
  mover.start = {-2.0, -3.0};
  mover.end = {-2.0, -8.0};
  mover.size = {0.5, 0.5, 1.8};
  mover.speed_mps = 0.8;
  mover.trigger_distance_m = 2.5;  // the robot at (-5, -5.5) is 3 m from the midpoint (-2, -5.5)
  s.movers.push_back(mover);
  plant.reset(s, kYard, p.poses.at("carry"), 1);
  plant.step(twist(0.0, 0.0), 0.5);
  EXPECT_NEAR(plant.mover_positions().front().second.y(), -3.0, 1e-12);  // waiting
  for (int k = 0; k < 50; ++k) plant.step(twist(0.6, 0.0), 0.02);      // 0.6 m closer: triggered
  const double y = plant.mover_positions().front().second.y();
  EXPECT_LT(y, -3.0);
  const int mocap = plant.model()->body_mocapid[mj_name2id(plant.model(), mjOBJ_BODY, "mover/worker")];
  EXPECT_NEAR(plant.data()->mocap_pos[3 * mocap + 1], y, 1e-12);
  EXPECT_NEAR(plant.data()->mocap_pos[3 * mocap + 2], 0.9, 1e-12);  // stands on the ground
}

TEST_F(PlantIdeal, MoverHoldsWhileRobotWithinStandOff)
{
  mmp::ScenarioSetup s;
  s.name = "blocked";
  mmp::MoverSpec mover;
  mover.name = "worker";
  mover.start = {-4.0, -3.0};  // walks -y towards the robot standing in its lane at (-4, -5.5)
  mover.end = {-4.0, -9.0};
  mover.size = {0.5, 0.5, 1.8};
  mover.speed_mps = 0.8;
  mover.trigger_distance_m = 5.0;
  s.movers.push_back(mover);
  for (const std::string pose : {"home", "carry"}) {  // the nearest robot geom differs (the panel, the base)
    plant.reset(s, {-4.0, -5.5, 0.0}, p.poses.at(pose), 1);
    for (int k = 0; k < 250; ++k) plant.step(twist(0.0, 0.0), 0.02);  // 5 s
    const auto status = plant.mover_status().front();
    // stopped within one physics step's walk of the stand-off from the nearest robot geom
    const double gap = plant.mover_robot_distance("worker");
    EXPECT_GE(gap, mmp::kMoverStandOffM - 1e-9) << pose;
    EXPECT_LE(gap, mmp::kMoverStandOffM + 0.8 * 0.002 + 1e-9) << pose;
    EXPECT_GT(status.blocked_s, 5.0 - status.walked / 0.8 - 0.01) << pose;  // waiting ever since it stopped
    EXPECT_LT(status.walked, 2.5) << pose;                                  // short of the robot
  }
}

TEST_F(PlantIdeal, SameSeedSameRun)
{
  mmp::Plant other{p, SITE_MODEL, mmp::PlantMode::Ideal};
  plant.reset({}, kYard, p.poses.at("carry"), 7);
  other.reset({}, kYard, p.poses.at("carry"), 7);
  mmp::Vec6 qd;
  qd << 0.1, -0.05, 0.2, 0.0, 0.1, -0.1;
  for (int k = 0; k < 500; ++k) {  // 10 s
    const auto c = twist(0.2 + 0.1 * std::sin(0.01 * k), 0.3 * std::cos(0.02 * k), qd * std::sin(0.03 * k));
    plant.step(c, 0.02);
    other.step(c, 0.02);
  }
  EXPECT_EQ(std::memcmp(plant.data()->qpos, other.data()->qpos, sizeof(mjtNum) * plant.model()->nq), 0);
}

TEST(ModelBuilder, ScenarioBoxNamedLikeASiteBoxMustMatchIt)
{
  const auto p = profile();
  mmp::ScenarioSetup s;
  s.name = "dup";
  // a site box with its exact export values is accepted and not duplicated
  s.boxes.push_back(box("Environment/NavigationObstacles/Building/ColumnE/proxy_0", {18.8, 11.25, 1.5}, {0.4, 0.4, 3.0}));
  mjModel * m = mmp::build_model(p, SITE_MODEL, s);
  EXPECT_LT(mj_name2id(m, mjOBJ_GEOM, "scenario/Environment/NavigationObstacles/Building/ColumnE/proxy_0"), 0);
  EXPECT_EQ(m->opt.timestep, 0.002);
  EXPECT_EQ(m->opt.integrator, mjINT_IMPLICITFAST);
  mj_deleteModel(m);
  s.boxes.front().size.x() = 0.5;
  EXPECT_THROW(mmp::build_model(p, SITE_MODEL, s), std::runtime_error);
}

TEST(ModelBuilder, GainsComeFromTheProfile)
{
  auto p = profile();
  p.kp(1) = 7000.0;  // as a calibration would change them, without regenerating robot.xml
  p.kd(5) = 30.0;
  p.armature(2) = 0.05;
  p.damping(3) = 0.4;
  p.frictionloss(4) = 0.2;
  p.base_servo_kv = Eigen::Vector3d(1000.0, 2000.0, 3000.0);
  mjModel * m = mmp::build_model(p, SITE_MODEL, {});
  for (int i = 0; i < 6; ++i) {
    const int a = mj_name2id(m, mjOBJ_ACTUATOR, ("servo/" + p.arm_joints[i]).c_str());
    ASSERT_GE(a, 0);
    EXPECT_EQ(m->actuator_gainprm[a * mjNGAIN], p.kp(i));
    EXPECT_EQ(m->actuator_biasprm[a * mjNBIAS + 1], -p.kp(i));
    EXPECT_EQ(m->actuator_biasprm[a * mjNBIAS + 2], -p.kd(i));
    const int dof = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, p.arm_joints[i].c_str())];
    EXPECT_EQ(m->dof_armature[dof], p.armature(i));
    EXPECT_EQ(m->dof_damping[dof], p.damping(i));
    EXPECT_EQ(m->dof_frictionloss[dof], p.frictionloss(i));
  }
  const char * drives[3] = {"drive/base_x", "drive/base_y", "drive/base_yaw"};
  for (int i = 0; i < 3; ++i) {
    const int a = mj_name2id(m, mjOBJ_ACTUATOR, drives[i]);
    ASSERT_GE(a, 0);
    EXPECT_EQ(m->actuator_gainprm[a * mjNGAIN], p.base_servo_kv(i));
    EXPECT_EQ(m->actuator_biasprm[a * mjNBIAS + 2], -p.base_servo_kv(i));
  }
  mj_deleteModel(m);
}
