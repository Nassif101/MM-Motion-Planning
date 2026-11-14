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
  double lag_s = 0.0;       // a first-order lag of the executed yaw rate (the start of motion, review C1)
  double stiction = 1.0;    // the share of a command below 0.08 rad/s the base executes (in-place stiction, review C1)
  bool blocked = false;     // the wheels do not move (review M3)
  double com = 0.0;         // the centre of mass's x in base_footprint (spec Amendment 3)
  double w_now = 0.0;
  Eigen::Vector3d step(const Control & u, double dt)
  {
    const double g_inf = 1 + truth.turn_delta_inf, g_0 = 1 + truth.turn_delta_zero, c = truth.turn_scale;
    double target = g_inf * u(1) - (g_inf - g_0) * c * std::tanh(u(1) / c);
    if (std::abs(u(1)) < 0.08) target *= stiction;
    w_now = lag_s > 0 ? w_now + (target - w_now) * (1 - std::exp(-dt / lag_s)) : target;
    if (blocked) return pose;
    const double w = w_now;
    const double vf = u(0) * (1 - truth.speed_turn * (std::sqrt(w * w + 1e-4) - 0.01));
    const double vl = truth.sideslip * vf * w + (truth.turn_offset + truth.turn_offset_com * com) * w;
    std::normal_distribution<double> chatter(0.0, 0.001);
    const double mid = pose(2) + 0.5 * w * dt;
    pose += Eigen::Vector3d(vf * std::cos(mid) - vl * std::sin(mid), vf * std::sin(mid) + vl * std::cos(mid), w) * dt;
    pose(2) = std::remainder(pose(2) + chatter(rng), 2.0 * M_PI);  // a wrapped heading, as the plant reports it
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
static double yaw_rate_error(const TrackingModel & model, const TrackingModel & truth, const std::vector<Control> & commands,
                             double band_low = 0.0, double band_high = 10.0)
{
  double sum = 0;
  size_t n = 0;
  for (size_t i = commands.size() / 3; i < commands.size(); ++i) {
    const double w = std::abs(commands[i](1));
    if (w < band_low || w >= band_high) continue;
    sum += std::pow(yaw_rate(model, commands[i](1)) - yaw_rate(truth, commands[i](1)), 2);
    ++n;
  }
  return n ? std::sqrt(sum / n) : 0.0;
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
    EXPECT_EQ(est(1), 1.0 + prior().turn_delta_zero) << "g_0 is not estimated (review I3)";
    EXPECT_NEAR(est(2), p[2], 0.03) << "sideslip of plant " << p[0];
    // the band the estimator learns from: turns at or above the stiction band
    // with g_0 fixed at the prior, the map matches another grip's at-speed band up to an offset of (g_inf - g_0) c,
    // about 0.014 rad/s at mu 0.4
    EXPECT_LT(yaw_rate_error(e.model(), base.truth, commands, 0.08), 0.015) << "yaw rate of plant " << p[0];
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
  const double adapted = yaw_rate_error(e.model(), base.truth, commands, 0.08),
               start = yaw_rate_error(prior(), base.truth, commands, 0.08);
  std::printf("yaw-rate rms at 0.08 rad/s and above: prior %.4f, adapted %.4f rad/s\n", start, adapted);
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
  // review I2: tracking under steady turning is fast: within 0.1 after 3 s of turning at 0.2 rad/s and 0.3 m/s
  std::vector<Control> turn = mixed(0.0);
  for (int k = 0; k < 150; ++k) turn.push_back(twist(0.3, 0.2));
  drive(e, base, turn);
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
  EXPECT_EQ(m.turn_delta_zero, prior().turn_delta_zero);
  EXPECT_LE(m.sideslip, 0.5);
  const EstimatorConfig c;
  EXPECT_LE(e.covariance().trace(), c.sigma_g_inf * c.sigma_g_inf + c.sigma_k * c.sigma_k + 1e-12);
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


// ---- Task 6b review ---------------------------------------------------------------------------------------------------

TEST(Estimator, StartsOfMotionDoNotDragTheGain)
{
  // review C1: the command ramps from rest while the base lags it (0.1 s); the plant's map is the prior's, so the
  // estimate must stay at the prior: ramp windows are not used
  TrackingEstimator e(prior());
  SyntheticBase base{plant_with(0.982, 0.025, 0.196)};
  base.lag_s = 0.1;
  double worst = 0.0;
  for (int start = 0; start < 6; ++start) {
    std::vector<Control> commands;
    for (int k = 0; k < 25; ++k) commands.push_back(twist(std::min(0.3, 0.009 * k), std::min(0.3, 0.016 * k)));  // ramps
    for (int k = 0; k < 25; ++k) commands.push_back(twist(0.0, 0.0));
    for (const Control & u : commands) {
      const Eigen::Vector3d before = base.pose, after = base.step(u, 0.02);
      e.observe(before, after, u, 0.02);
      worst = std::max(worst, std::abs(e.estimate()(0) - 0.982));
    }
  }
  EXPECT_LT(worst, 0.05);
}

TEST(Estimator, InPlaceStictionDoesNotDragTheGain)
{
  // review C1: slow in-place turns where the base executes a third of the command (stiction): the static map cannot
  // represent it, and the estimate must not take it out on g_inf
  TrackingEstimator e(prior());
  SyntheticBase base{plant_with(0.982, 0.025, 0.196)};
  base.stiction = 1.0 / 3.0;
  std::vector<Control> slow;
  for (int k = 0; k < 500; ++k) slow.push_back(twist(0.0, k % 200 < 100 ? 0.06 : -0.06));
  drive(e, base, slow);
  EXPECT_NEAR(e.estimate()(0), 0.982, 0.05);
}

TEST(Estimator, BlockedWheelsAreNotLearned)
{
  // review M3: a window commanding a turn while the base does not move (wheels blocked, a contact) is not a grip
  TrackingEstimator e(prior());
  SyntheticBase base{plant_with(0.982, 0.025, 0.196)};
  drive(e, base, mixed(20.0));
  const double before = e.estimate()(0);
  base.blocked = true;
  std::vector<Control> held(10, twist(0.3, 0.3));
  drive(e, base, held);
  EXPECT_NEAR(e.estimate()(0), before, 0.02);
}

TEST(Estimator, LearnsAcrossTheHeadingWrap)
{
  // review M1: the plant reports a wrapped heading. The same drive started at 3.1 rad (crossing +-pi many times) and at
  // 0 rad must learn the same: a window across the wrap is used, not dropped as an outlier
  auto run = [](double yaw0) {
    TrackingEstimator e(prior());
    SyntheticBase base{plant_with(1.270, 0.631, 0.083)};
    base.pose(2) = yaw0;
    drive(e, base, mixed(20.0));
    return e.estimate();
  };
  const Eigen::Vector3d crossing = run(3.1), plain = run(0.0);
  EXPECT_NEAR(crossing(0), 1.270, 0.1);
  EXPECT_LT((crossing - plain).cwiseAbs().maxCoeff(), 1e-6);
}

TEST(Estimator, ABadStepDiscardsTheWindow)
{
  // review M1: during a steady turn that is being learned, four good steps, a bad one, then one more good step: the
  // window restarted at the bad step (and the steady clock with it), so the estimate does not move
  TrackingEstimator e(prior());
  SyntheticBase base{plant_with(1.5, 0.025, 0.4)};
  drive(e, base, std::vector<Control>(50, twist(0.3, 0.3)));  // 1 s steady: updates are happening
  const Eigen::Vector3d settled = e.estimate();
  ASSERT_GT(std::abs(settled(0) - 0.982), 0.01) << "the steady turn should have moved the estimate";
  for (int k = 0; k < 4; ++k) {
    const Eigen::Vector3d before = base.pose, after = base.step(twist(0.3, 0.3), 0.02);
    e.observe(before, after, twist(0.3, 0.3), 0.02);
  }
  e.observe(base.pose, base.pose, twist(0.3, 0.3), 0.5);  // a gap
  const Eigen::Vector3d before = base.pose, after = base.step(twist(0.3, 0.3), 0.02);
  e.observe(before, after, twist(0.3, 0.3), 0.02);
  EXPECT_EQ(e.estimate(), settled);
}

TEST(Estimator, TheCovarianceReachesItsCapWithoutExcitation)
{
  // review M1: without excitation the drift fills the covariance up to the cap, and no further
  TrackingEstimator e(prior());
  SyntheticBase base{plant_with(0.982, 0.025, 0.196)};
  drive(e, base, mixed(20.0));
  std::vector<Control> still(6000, twist(0.0, 0.0));  // 120 s at rest
  drive(e, base, still);
  const EstimatorConfig c;
  EXPECT_NEAR(e.covariance()(0, 0), c.sigma_g_inf * c.sigma_g_inf, 1e-12);
  EXPECT_NEAR(e.covariance()(2, 2), c.sigma_k * c.sigma_k, 1e-12);
}

TEST(Estimator, TheLateralSpeedIsTakenAtTheMidHeading)
{
  // review M1: fast turns (0.6 rad/s: 12 mrad per cycle) with sideslip: the start heading would bias k
  TrackingEstimator e(prior());
  SyntheticBase base{plant_with(0.982, 0.025, 0.35)};
  std::vector<Control> fast;
  for (int k = 0; k < 40; ++k) fast.push_back(twist(std::min(0.15, 0.009 * k), std::min(0.6, 0.016 * k)));
  for (int k = 0; k < 1000; ++k) fast.push_back(twist(0.15, 0.6));
  drive(e, base, fast);
  EXPECT_NEAR(e.estimate()(2), 0.35, 0.01);
}

// ---- spec Amendment 3: the turning centre ----

namespace
{
void drive_with_com(TrackingEstimator & e, SyntheticBase & base, const std::vector<Control> & commands, double dt = 0.02)
{
  for (const Control & u : commands) {
    const Eigen::Vector3d before = base.pose;
    const Eigen::Vector3d after = base.step(u, dt);
    e.observe(before, after, u, dt, base.com);
  }
}

TrackingModel offset_model(double k, double offset, double offset_com)
{
  TrackingModel m = prior();
  m.sideslip = k;
  m.turn_offset = offset;
  m.turn_offset_com = offset_com;
  return m;
}
}  // namespace

TEST(Estimator, AnOffsetPlantLeavesTheSideslipAtZero)
{
  // a base turning about a point behind its frame (0.003 - 2.23 x_com, at x_com -0.03: 0.07 m) with no sideslip. An
  // estimator that knows the offset keeps k near 0; one that does not makes k stand in for it (Task 10's finding)
  const TrackingModel truth = offset_model(0.0, 0.003, -2.23);
  const auto commands = mixed(30.0);
  SyntheticBase base{truth};
  base.com = -0.03;
  TrackingEstimator knows(truth);
  drive_with_com(knows, base, commands);
  EXPECT_LT(std::abs(knows.estimate()(2)), 0.02);
  SyntheticBase base2{truth};
  base2.com = -0.03;
  TrackingEstimator blind(offset_model(0.0, 0.0, 0.0));
  drive_with_com(blind, base2, commands);
  EXPECT_GT(blind.estimate()(2), 0.05);
  EXPECT_DOUBLE_EQ(knows.model().turn_offset, 0.003);  // the offset is not estimated
  EXPECT_DOUBLE_EQ(knows.model().turn_offset_com, -2.23);
}

TEST(Estimator, WithoutTheCentreOfMassAnOffsetModelIsRefused)
{
  // the four-argument observe has no centre of mass: with an offset in the model it would silently use the wrong one
  TrackingEstimator e(offset_model(0.0, 0.003, -2.23));
  EXPECT_THROW(e.observe(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), twist(0.3, 0.2), 0.02), std::logic_error);
  TrackingEstimator plain(prior());
  EXPECT_NO_THROW(plain.observe(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), twist(0.3, 0.2), 0.02));
}

TEST(Estimator, RejectsAnInvalidConfig)
{
  // review M4: a zero noise or drift time would make the filter divide by zero for good
  for (auto bad : std::vector<std::function<void(EstimatorConfig &)>>{
         [](EstimatorConfig & c) { c.meas_yaw = 0.0; }, [](EstimatorConfig & c) { c.settle_s = 0.0; },
         [](EstimatorConfig & c) { c.window_s = -1.0; }, [](EstimatorConfig & c) { c.sigma_g_inf = NAN; }}) {
    EstimatorConfig c;
    bad(c);
    EXPECT_THROW(TrackingEstimator(prior(), c), std::invalid_argument);
  }
}
