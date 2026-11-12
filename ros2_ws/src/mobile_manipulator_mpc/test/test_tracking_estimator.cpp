// E2 Task 6b (spec Amendment 2): the online correction of the tracking model's turn gains and sideslip.
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <functional>
#include <random>
#include <vector>

#include "mobile_manipulator_mpc/tracking_estimator.hpp"

using namespace mobile_manipulator_mpc;

namespace
{
// the identified mu 0.6 model (config/tracking_model_mujoco.yaml), the estimator's prior
TrackingModel prior()
{
  TrackingModel m;
  m.turn_delta_inf = -0.018;
  m.turn_delta_zero = -0.975;
  m.turn_scale = 0.0225;
  m.speed_turn = 0.21;
  m.sideslip = 0.196;
  return m;
}

TrackingModel plant_with(double g_inf, double g_0, double k, double c = 0.0225)
{
  TrackingModel m = prior();
  m.turn_delta_inf = g_inf - 1.0;
  m.turn_delta_zero = g_0 - 1.0;
  m.turn_scale = c;
  m.sideslip = k;
  return m;
}

// The base as the plant moves it: the tracking map of `truth` applied to the command, integrated at the mid heading,
// with a small deterministic yaw chatter (about 1 mrad per cycle, like the MuJoCo base's turning chatter).
struct SyntheticBase
{
  TrackingModel truth;
  Eigen::Vector3d pose = Eigen::Vector3d::Zero();
  std::mt19937 rng{7};
  Eigen::Vector3d step(const Control & u, double dt)
  {
    const double g_inf = 1 + truth.turn_delta_inf, g_0 = 1 + truth.turn_delta_zero, c = truth.turn_scale;
    const double w = g_inf * u(1) - (g_inf - g_0) * c * std::tanh(u(1) / c);
    const double vf = u(0) * (1 - truth.speed_turn * (std::sqrt(w * w + 1e-4) - 0.01));
    const double vl = truth.sideslip * vf * w;
    std::normal_distribution<double> chatter(0.0, 0.001);
    const double mid = pose(2) + 0.5 * w * dt;
    pose += Eigen::Vector3d(vf * std::cos(mid) - vl * std::sin(mid), vf * std::sin(mid) + vl * std::cos(mid), w) * dt;
    pose(2) += chatter(rng);
    return pose;
  }
};

Control twist(double v, double w)
{
  Control u = Control::Zero();
  u(0) = v;
  u(1) = w;
  return u;
}

// Mixed driving for `seconds`: turns at speed, slow in-place turns, straight stretches, each held 2 s, ramped at the
// envelope's rates (0.45 m/s^2, 0.8 rad/s^2).
std::vector<Control> mixed(double seconds, double dt = 0.02)
{
  const double levels[][2] = {{0.3, 0.2}, {0.0, 0.05}, {0.3, 0.0}, {0.2, -0.35}, {0.0, -0.04}, {0.3, 0.1}, {0.0, 0.07},
                              {0.25, -0.15}, {0.3, 0.0}, {0.0, -0.06}};
  std::vector<Control> out;
  Eigen::Vector2d now(0, 0);
  for (int k = 0; k * dt < seconds; ++k) {
    const auto & level = levels[static_cast<size_t>(k * dt / 2.0) % 10];
    now(0) += std::clamp(level[0] - now(0), -0.45 * dt, 0.45 * dt);
    now(1) += std::clamp(level[1] - now(1), -0.8 * dt, 0.8 * dt);
    out.push_back(twist(now(0), now(1)));
  }
  return out;
}

void drive(TrackingEstimator & e, SyntheticBase & base, const std::vector<Control> & commands, double dt = 0.02)
{
  for (const Control & u : commands) {
    const Eigen::Vector3d before = base.pose;
    const Eigen::Vector3d after = base.step(u, dt);
    e.observe(before, after, u, dt);
  }
}
}  // namespace

TEST(Estimator, PriorIsTheIdentifiedModel)
{
  const TrackingEstimator e(prior());
  const TrackingModel m = e.model();
  EXPECT_DOUBLE_EQ(m.turn_delta_inf, prior().turn_delta_inf);
  EXPECT_DOUBLE_EQ(m.turn_delta_zero, prior().turn_delta_zero);
  EXPECT_DOUBLE_EQ(m.sideslip, prior().sideslip);
  EXPECT_DOUBLE_EQ(m.turn_scale, prior().turn_scale);
  EXPECT_DOUBLE_EQ(m.speed_turn, prior().speed_turn);
}

static double yaw_rate(const TrackingModel & m, double w)
{
  const double g_inf = 1 + m.turn_delta_inf, g_0 = 1 + m.turn_delta_zero, c = m.turn_scale;
  return g_inf * w - (g_inf - g_0) * c * std::tanh(w / c);
}

// the rms of the yaw rate a model predicts against the plant's, over the commands of the last two thirds
static double yaw_rate_error(const TrackingModel & model, const TrackingModel & truth, const std::vector<Control> & commands)
{
  double sum = 0;
  size_t n = 0;
  for (size_t i = commands.size() / 3; i < commands.size(); ++i, ++n)
    sum += std::pow(yaw_rate(model, commands[i](1)) - yaw_rate(truth, commands[i](1)), 2);
  return std::sqrt(sum / n);
}

TEST(Estimator, ConvergesToEachGrip)
{
  // the closed-loop fits at mu 0.4, 0.6 and 0.8 (e2.md), with the prior's turn scale (the estimator keeps c fixed).
  // g_0 is weakly observable (it matters below about c): it is judged through the yaw rate predicted for the commands
  const double plants[][3] = {{1.270, 0.631, 0.083}, {0.982, 0.025, 0.196}, {0.801, 0.010, 0.163}};
  for (const auto & p : plants) {
    TrackingEstimator e(prior());
    SyntheticBase base{plant_with(p[0], p[1], p[2])};
    const std::vector<Control> commands = mixed(20.0);
    drive(e, base, commands);
    const Eigen::Vector3d est = e.estimate();
    EXPECT_NEAR(est(0), p[0], 0.1) << "g_inf of plant " << p[0];
    EXPECT_NEAR(est(2), p[2], 0.03) << "sideslip of plant " << p[0];
    EXPECT_LT(yaw_rate_error(e.model(), base.truth, commands), 0.01) << "yaw rate of plant " << p[0];
  }
}

TEST(Estimator, PredictsAPlantWithAnotherTurnScale)
{
  // the mu 0.4 plant with its own c (0.0425): the gains are only effective values then, but the yaw rate it predicts
  // for the commands of the last 10 s is much closer than the prior's
  TrackingEstimator e(prior());
  SyntheticBase base{plant_with(1.270, 0.631, 0.083, 0.0425)};
  const std::vector<Control> commands = mixed(30.0);
  drive(e, base, commands);
  const double adapted = yaw_rate_error(e.model(), base.truth, commands), start = yaw_rate_error(prior(), base.truth, commands);
  std::printf("yaw-rate rms: prior %.4f, adapted %.4f rad/s\n", start, adapted);
  EXPECT_LT(adapted, 0.3 * start);
}

TEST(Estimator, HoldsWithoutExcitation)
{
  TrackingEstimator e(prior());
  SyntheticBase base{plant_with(1.270, 0.631, 0.083)};
  drive(e, base, mixed(20.0));
  const Eigen::Vector3d settled = e.estimate();
  std::vector<Control> straight(1000, twist(0.3, 0.0));  // 20 s straight
  drive(e, base, straight);
  EXPECT_LT((e.estimate() - settled).cwiseAbs().maxCoeff(), 0.01);
  std::vector<Control> still(1000, twist(0.0, 0.0));  // 20 s at rest
  drive(e, base, still);
  EXPECT_LT((e.estimate() - settled).cwiseAbs().maxCoeff(), 0.01);
}

TEST(Estimator, FollowsAGripChange)
{
  TrackingEstimator e(prior());
  SyntheticBase base{plant_with(0.982, 0.025, 0.196)};
  drive(e, base, mixed(20.0));
  base.truth = plant_with(1.270, 0.631, 0.083);  // the ground turns slippery
  drive(e, base, mixed(20.0));  // twenty seconds of mixed driving on the new ground (a slow drift: steady estimates)
  EXPECT_NEAR(e.estimate()(0), 1.270, 0.1);
}

TEST(Estimator, StaysInRangeAndBounded)
{
  // a plant turning three times the command: g_inf runs into the loader's ceiling (1.5) and stays there; the
  // covariance never exceeds the prior's
  TrackingEstimator e(prior());
  SyntheticBase base{plant_with(3.0, 3.0, 0.9)};
  drive(e, base, mixed(30.0));
  const TrackingModel m = e.model();
  EXPECT_LE(m.turn_delta_inf, 0.5);
  EXPECT_LE(m.turn_delta_zero, 0.5);
  EXPECT_LE(m.sideslip, 0.5);
  const EstimatorConfig c;
  EXPECT_LE(e.covariance().trace(), c.sigma_g_inf * c.sigma_g_inf + c.sigma_g_0 * c.sigma_g_0 + c.sigma_k * c.sigma_k + 1e-12);
  EXPECT_TRUE(e.estimate().allFinite() && e.covariance().allFinite());
}

TEST(Estimator, IgnoresGapsAndBadSteps)
{
  TrackingEstimator e(prior());
  const Eigen::Vector3d settled = e.estimate();
  e.observe(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.1, 0.0, 0.5), twist(0.3, 0.2), 0.0);   // no time
  e.observe(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.1, 0.0, 0.5), twist(0.3, 0.2), 0.5);   // a gap
  e.observe(Eigen::Vector3d::Zero(), Eigen::Vector3d(NAN, 0.0, 0.5), twist(0.3, 0.2), 0.02);  // non-finite
  EXPECT_EQ(e.estimate(), settled);
}

TEST(Estimator, IsDeterministic)
{
  auto run = [] {
    TrackingEstimator e(prior());
    SyntheticBase base{plant_with(1.270, 0.631, 0.083)};
    drive(e, base, mixed(10.0));
    return e.estimate();
  };
  const Eigen::Vector3d a = run(), b = run();
  EXPECT_EQ(std::memcmp(a.data(), b.data(), sizeof(double) * 3), 0);
}

TEST(Estimator, ResetReturnsToThePrior)
{
  TrackingEstimator e(prior());
  SyntheticBase base{plant_with(1.270, 0.631, 0.083)};
  drive(e, base, mixed(10.0));
  e.reset();
  EXPECT_DOUBLE_EQ(e.model().turn_delta_inf, prior().turn_delta_inf);
}
