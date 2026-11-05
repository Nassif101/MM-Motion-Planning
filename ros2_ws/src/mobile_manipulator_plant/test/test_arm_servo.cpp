// The realistic arm (MuJoCo plant plan Task 6): the streamed reference sampled as the JTC samples it (cubic Hermite,
// at the controller rate, held between samples, realized acceleration capped), and the servo's control with MuJoCo's
// feedforward.
#include <gtest/gtest.h>

#include "mobile_manipulator_plant/arm_servo.hpp"
#include "mobile_manipulator_plant/model_builder.hpp"

namespace mmp = mobile_manipulator_plant;

namespace
{
std::string source_share(const std::string & package) { return std::string(PACKAGES_SRC) + "/" + package; }

mmp::ArmReference stream()
{
  mmp::ArmReference r;
  r.start_time = 1.0;
  mmp::Vec6 q0 = mmp::Vec6::Zero(), q1 = mmp::Vec6::Constant(0.2), q2 = mmp::Vec6::Constant(0.3);
  r.points = {{0.0, q0, mmp::Vec6::Constant(0.5)}, {0.4, q1, mmp::Vec6::Constant(0.25)}, {1.0, q2, mmp::Vec6::Zero()}};
  return r;
}
}  // namespace

TEST(StreamSampler, HermiteBetweenPointsAndHeldAfter)
{
  const auto r = stream();
  mmp::Vec6 q, v;
  mmp::sample_stream(r, mmp::Vec6::Zero(), 1.0, q, v);
  EXPECT_NEAR(q(0), 0.0, 1e-12);
  EXPECT_NEAR(v(0), 0.5, 1e-12);
  mmp::sample_stream(r, mmp::Vec6::Zero(), 1.4, q, v);
  EXPECT_NEAR(q(0), 0.2, 1e-12);
  EXPECT_NEAR(v(0), 0.25, 1e-12);
  mmp::sample_stream(r, mmp::Vec6::Zero(), 5.0, q, v);  // after the last point: at rest there
  EXPECT_NEAR(q(0), 0.3, 1e-12);
  EXPECT_EQ(v(0), 0.0);
  mmp::sample_stream(r, mmp::Vec6::Constant(0.7), 0.5, q, v);  // before the start: the first point
  EXPECT_NEAR(q(0), 0.0, 1e-12);
}

TEST(StreamSampler, EmptyStreamHolds)
{
  mmp::ArmReference r;
  mmp::Vec6 q, v;
  mmp::sample_stream(r, mmp::Vec6::Constant(0.4), 2.0, q, v);
  EXPECT_EQ(q, mmp::Vec6::Constant(0.4));
  EXPECT_EQ(v, mmp::Vec6::Zero());
}

TEST(JtcSampler, HoldsBetweenTicksAndCapsTheAcceleration)
{
  mmp::JtcSampler jtc(50.0, 4.0);
  jtc.reset(mmp::Vec6::Zero());
  mmp::ArmReference r;
  r.points = {{0.0, mmp::Vec6::Zero(), mmp::Vec6::Zero()}, {0.02, mmp::Vec6::Constant(0.01), mmp::Vec6::Constant(1.0)}};
  const auto a = jtc.sample(r, 0.0);
  EXPECT_EQ(a.q, mmp::Vec6::Zero());
  const auto b = jtc.sample(r, 0.01);  // between ticks: the tick at 0 holds
  EXPECT_EQ(b.q, a.q);
  const auto c = jtc.sample(r, 0.02);  // the tick at 0.02: velocity 1.0, realized acceleration 50 rad/s^2 capped at 4
  EXPECT_NEAR(c.v(0), 1.0, 1e-12);
  EXPECT_NEAR(c.a(0), 4.0, 1e-12);
}

class ArmServoTest : public ::testing::Test
{
protected:
  mmp::RobotProfile p = mmp::load_profile(PROFILE_YAML, source_share);
  mjModel * m = mmp::build_model(p, SITE_MODEL, {});
  mjData * d = mj_makeData(m);
  void TearDown() override
  {
    mj_deleteData(d);
    mj_deleteModel(m);
  }
};

TEST_F(ArmServoTest, GravityFeedforwardHoldsCarry)
{
  // with Kp at 10 % of the profile the static sag is large without feedforward, small with it
  auto weak = p;
  weak.kp *= 0.1;
  weak.kd *= 0.1;
  mjModel * mw = mmp::build_model(weak, SITE_MODEL, {});
  for (const bool ff : {false, true}) {
    mjData * dw = mj_makeData(mw);
    mmp::ArmServo servo(mw, weak, ff, false);
    const mmp::Vec6 q_r = p.poses.at("carry");
    for (int i = 0; i < 6; ++i) dw->qpos[servo.qadr(i)] = q_r(i);
    for (int k = 0; k < 1000; ++k) {  // 2 s
      servo.apply(dw, {q_r, mmp::Vec6::Zero(), mmp::Vec6::Zero()});
      mj_step(mw, dw);
    }
    double sag = 0.0;
    for (int i = 0; i < 6; ++i) sag = std::max(sag, std::abs(dw->qpos[servo.qadr(i)] - q_r(i)));
    if (ff) EXPECT_LT(sag, 1e-3);
    else EXPECT_GT(sag, 5e-3);
    mj_deleteData(dw);
  }
  mj_deleteModel(mw);
}

TEST_F(ArmServoTest, ClampsToEffortAndLimits)
{
  mmp::ArmServo servo(m, p, true, true);
  mmp::Vec6 q_r = p.poses.at("carry");
  q_r(1) = 2.75;  // a radian beyond the shoulder lift's 1.745 rad limit
  for (int k = 0; k < 1000; ++k) {
    servo.apply(d, {q_r, mmp::Vec6::Zero(), mmp::Vec6::Zero()});
    mj_step(m, d);
  }
  const int a = mj_name2id(m, mjOBJ_ACTUATOR, "servo/shoulder_lift_joint");
  EXPECT_LE(std::abs(d->actuator_force[a]), 160.0 + 1e-9);
  for (int i = 0; i < m->nq; ++i) ASSERT_TRUE(std::isfinite(d->qpos[i]));
  EXPECT_LE(d->qpos[servo.qadr(1)], 1.74532925199 + 0.05);
}

TEST_F(ArmServoTest, TracksAMovingReferenceBetterWithMotionFeedforward)
{
  double err[2];
  for (const bool motion : {false, true}) {
    mjData * dd = mj_makeData(m);
    mmp::ArmServo servo(m, p, true, motion);
    const mmp::Vec6 c = p.poses.at("carry");
    for (int i = 0; i < 6; ++i) dd->qpos[servo.qadr(i)] = c(i);
    double worst = 0.0;
    for (int k = 0; k < 1000; ++k) {  // the pan swings at 1 rad/s amplitude, 0.5 Hz
      const double t = 0.002 * k, w = M_PI;
      mmp::Vec6 q = c, v = mmp::Vec6::Zero(), acc = mmp::Vec6::Zero();
      q(0) += std::sin(w * t) / w;
      v(0) = std::cos(w * t);
      acc(0) = -w * std::sin(w * t);
      servo.apply(dd, {q, v, acc});
      mj_step(m, dd);
      if (k > 250) worst = std::max(worst, std::abs(dd->qpos[servo.qadr(0)] - (c(0) + std::sin(w * (t + 0.002)) / w)));
    }
    err[motion] = worst;
    mj_deleteData(dd);
  }
  EXPECT_LT(err[1], err[0]);
}
