// The node's arm command path in the plant's glue (MuJoCo plant plan Task 6): B5's arm state is the reference it
// streamed (re-anchored on the measured joints beyond 0.15 rad), Ok/Fallback cycles stream the plan with point 0 and a
// braking tail, braking/holding/stale/blocked cycles hold once, as wholebody_mpc_node does.
#include <gtest/gtest.h>

#include <random>
#include <rclcpp/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "mobile_manipulator_campaign/arm_command_path.hpp"
#include "mobile_manipulator_medium/bench_setup.hpp"
#include "mobile_manipulator_mpc/arm_stream.hpp"
#include "mobile_manipulator_plant/arm_servo.hpp"

namespace mmc = mobile_manipulator_campaign;
namespace mmm = mobile_manipulator_mpc;
namespace mmp = mobile_manipulator_plant;

namespace
{
struct Fixture
{
  mobile_manipulator_medium::BenchSetup bench{
    mobile_manipulator_medium::load_medium_config(mobile_manipulator_medium::package_share("mobile_manipulator_medium") + "/config/medium.yaml"),
    "v2_door_105_nav"};
  mmm::MpcConfig config = mmm::load_mpc_config(mobile_manipulator_medium::package_share("mobile_manipulator_mpc") + "/config/wholebody_mpc.yaml");
  mmc::ArmCommandPath path{bench.limits(), config, bench.primitives()};

  mmm::State carry() const
  {
    mmm::State x;
    x << 12.0, 0.0, 0.0, bench.poses().at("carry");
    return x;
  }
  // a plan whose arm moves the pan at 0.2 rad/s
  mmm::CycleOutput moving(const mmm::State & x0, mmm::CycleStatus status = mmm::CycleStatus::Ok) const
  {
    mmm::CycleOutput out;
    out.status = status;
    mmm::State x = x0;
    for (int k = 0; k <= config.horizon; ++k) {
      out.plan.x.push_back(x);
      if (k < config.horizon) {
        mmm::Control u = mmm::Control::Zero();
        u(2) = 0.2;
        out.plan.u.push_back(u);
        x(3) += 0.2 * config.dt;
      }
    }
    return out;
  }
};
}  // namespace

TEST(ArmCommandPath, StreamsThePlanLikeTheNode)
{
  Fixture f;
  const mmm::State x = f.carry();
  const auto sent = f.path.command(f.moving(x), x, 10.0, nullptr, Eigen::Vector2d::Zero());
  ASSERT_TRUE(sent.has_value());
  const auto & pts = sent->points;
  ASSERT_GE(pts.size(), 3u);
  EXPECT_EQ(pts.front().t, 0.0);
  EXPECT_LT((pts.front().q - x.tail<6>()).norm(), 1e-12);  // offset 0, no reference velocity before the first stream
  // the middle points are the plan's (mmm::plan_stream_points), the last the braking tail at rest
  EXPECT_EQ(pts.back().v, mmp::Vec6::Zero());
  const double tail = std::max(0.5, 0.2 / f.bench.limits().qdd_max(0));
  EXPECT_NEAR(pts.back().t, f.config.horizon * f.config.dt + tail, 1e-12);
  EXPECT_NEAR(pts.back().q(0), x(3) + 0.2 * f.config.horizon * f.config.dt + 0.5 * tail * 0.2, 1e-9);
}

TEST(ArmCommandPath, B5SeesTheReferenceUntilItIsFarFromTheJoints)
{
  Fixture f;
  const mmm::State x = f.carry();
  f.path.command(f.moving(x), x, 10.0, nullptr, Eigen::Vector2d::Zero());
  mmm::State measured = x;  // the arm lagging behind its reference
  const mmm::State b5 = f.path.b5_state(measured, 10.5);
  EXPECT_NEAR(b5(3), x(3) + 0.2 * 0.5, 1e-6);  // the reference 0.5 s on
  EXPECT_FALSE(f.path.reanchored());
  EXPECT_NEAR(f.path.reference_v()(0), 0.2, 1e-9);
  measured(3) -= 0.2;  // more than 0.15 rad from the reference: re-anchored on the measured joints
  EXPECT_EQ(f.path.b5_state(measured, 10.5)(3), measured(3));
  EXPECT_TRUE(f.path.reanchored());
}

TEST(ArmCommandPath, HoldsOnceOnBraking)
{
  Fixture f;
  const mmm::State x = f.carry();
  f.path.command(f.moving(x), x, 10.0, nullptr, Eigen::Vector2d::Zero());
  const auto hold = f.path.command(f.moving(x, mmm::CycleStatus::Braking), x, 10.2, nullptr, Eigen::Vector2d::Zero());
  ASSERT_TRUE(hold.has_value());
  ASSERT_EQ(hold->points.size(), 2u);
  const double T = std::max(mmm::kArmHoldMinS, 0.2 / f.bench.limits().qdd_max(0));
  EXPECT_NEAR(hold->points[1].t, T, 1e-9);
  EXPECT_NEAR(hold->points[1].q(0), hold->points[0].q(0) + 0.5 * T * hold->points[0].v(0), 1e-9);
  EXPECT_FALSE(f.path.command(f.moving(x, mmm::CycleStatus::Holding), x, 10.22, nullptr, Eigen::Vector2d::Zero()));  // held
  EXPECT_TRUE(f.path.command(f.moving(x), x, 10.24, nullptr, Eigen::Vector2d::Zero()));  // tracking resumes
  EXPECT_FALSE(f.path.command(f.moving(x, mmm::CycleStatus::GoalReached), x, 10.26, nullptr, Eigen::Vector2d::Zero()));
}

TEST(ArmCommandPath, AnEmptyPlanSendsNothing)
{
  Fixture f;
  mmm::CycleOutput out;
  out.status = mmm::CycleStatus::Ok;  // no stages
  EXPECT_FALSE(f.path.command(out, f.carry(), 1.0, nullptr, Eigen::Vector2d::Zero()).has_value());
}

TEST(ArmCommandPath, NothingSentMeansMeasuredAtRest)
{
  Fixture f;
  const mmm::State x = f.carry();
  EXPECT_EQ(f.path.b5_state(x, 3.0), x);
  EXPECT_EQ(f.path.reference_v(), mmp::Vec6::Zero());
}

// The plant samples streams exactly as the JTC does (mmm::sample_trajectory).
TEST(StreamSampler, MatchesJtcSampling)
{
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int trial = 0; trial < 100; ++trial) {
    mmp::ArmReference r;
    r.start_time = 2.0;
    trajectory_msgs::msg::JointTrajectory msg;
    double t = 0.0;
    for (int k = 0; k < 6; ++k) {
      // both samplers see the same time: ROS durations hold whole nanoseconds
      const double t_ns = rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(std::llround(t * 1e9))).seconds();
      mmp::StreamPoint p{t_ns, mmp::Vec6::NullaryExpr([&] { return u(rng); }), mmp::Vec6::NullaryExpr([&] { return u(rng); })};
      r.points.push_back(p);
      trajectory_msgs::msg::JointTrajectoryPoint m;
      m.positions.assign(p.q.data(), p.q.data() + 6);
      m.velocities.assign(p.v.data(), p.v.data() + 6);
      m.time_from_start = rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(std::llround(t * 1e9)));
      msg.points.push_back(m);
      t += 0.05 + 0.1 * std::abs(u(rng));
    }
    for (double s = -0.1; s < t + 0.2; s += 0.013) {
      mmp::Vec6 qa, va;
      Eigen::Matrix<double, 6, 1> qb, vb;
      mmp::sample_stream(r, mmp::Vec6::Zero(), 2.0 + s, qa, va);
      mmm::sample_trajectory(msg, s, qb, vb);
      ASSERT_LT((qa - qb).cwiseAbs().maxCoeff(), 1e-12) << trial << " " << s;
      ASSERT_LT((va - vb).cwiseAbs().maxCoeff(), 1e-12) << trial << " " << s;
    }
  }
}
