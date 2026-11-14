// E2 Task 5 (plan amended at CP1): the tracking model in B5's prediction, its generated map and its loader.
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "mobile_manipulator_mpc/model.hpp"
#include "mobile_manipulator_mpc/tracking_model.hpp"

using namespace mobile_manipulator_mpc;

namespace
{
TrackingModel full()
{
  TrackingModel m;
  m.turn_delta_inf = 0.0;
  m.turn_delta_zero = -0.95;
  m.turn_scale = 0.025;
  m.speed_turn = 0.17;
  m.sideslip = 0.2;
  m.turn_offset = 0.02;  // Amendment 3: the Jacobians then include the centre of mass's dependence on the arm
  m.turn_offset_com = -2.0;
  return m;
}

State random_state(std::mt19937 & rng)
{
  std::uniform_real_distribution<double> d(-1.0, 1.0);
  State x;
  for (int i = 0; i < kNx; ++i) x(i) = (i < 3 ? 3.0 : 1.0) * d(rng);
  return x;
}

Control random_control(std::mt19937 & rng, double omega_max = 0.6)
{
  std::uniform_real_distribution<double> d(-1.0, 1.0);
  Control u;
  u(0) = 0.5 * d(rng);
  u(1) = omega_max * d(rng);
  for (int i = 2; i < kNu; ++i) u(i) = 0.4 * d(rng);
  return u;
}

Control twist(double v, double omega)
{
  Control u = Control::Zero();
  u(0) = v;
  u(1) = omega;
  return u;
}

double sabs(double x) { return std::sqrt(x * x + 1e-4) - 0.01; }

std::string write_file(const std::string & name, const std::string & text)
{
  const auto dir = std::filesystem::temp_directory_path() / "e2_tracking_model_test";
  std::filesystem::create_directories(dir);
  const auto path = (dir / name).string();
  std::ofstream(path) << text;
  return path;
}
}  // namespace

TEST(Identity, DispatchesToTheUntouchedStep)
{
  std::mt19937 rng(1);
  for (int n = 0; n < 50; ++n)
    for (double dt : {0.02, 0.05, 0.1}) {
      const State x = random_state(rng);
      const Control u = random_control(rng);
      const Step a = discrete_step(x, u, dt, TrackingModel{}), b = discrete_step(x, u, dt);
      EXPECT_EQ(std::memcmp(a.x_next.data(), b.x_next.data(), sizeof(double) * kNx), 0);
      EXPECT_EQ(std::memcmp(a.A.data(), b.A.data(), sizeof(double) * kNx * kNx), 0);
      EXPECT_EQ(std::memcmp(a.B.data(), b.B.data(), sizeof(double) * kNx * kNu), 0);
    }
  EXPECT_TRUE(TrackingModel{}.identity());
  EXPECT_TRUE(TrackingModel{}.map_identity());
  EXPECT_FALSE(full().map_identity());
}

TEST(Map, JacobiansMatchFiniteDifferences)
{
  std::mt19937 rng(2);
  const TrackingModel m = full();
  const double h = 1e-6, dt = 0.1;
  for (int n = 0; n < 50; ++n) {
    const State x = random_state(rng);
    const Control u = random_control(rng);
    const Step s = discrete_step(x, u, dt, m);
    for (int j = 0; j < kNx; ++j) {
      State p = x, q = x;
      p(j) += h;
      q(j) -= h;
      const State fd = (discrete_step(p, u, dt, m).x_next - discrete_step(q, u, dt, m).x_next) / (2 * h);
      EXPECT_LT((s.A.col(j) - fd).cwiseAbs().maxCoeff(), 1e-5) << "A column " << j;
    }
    for (int j = 0; j < kNu; ++j) {
      Control p = u, q = u;
      p(j) += h;
      q(j) -= h;
      const State fd = (discrete_step(x, p, dt, m).x_next - discrete_step(x, q, dt, m).x_next) / (2 * h);
      EXPECT_LT((s.B.col(j) - fd).cwiseAbs().maxCoeff(), 1e-5) << "B column " << j;
    }
  }
}

TEST(Map, EachTermMovesThePredictionTheExpectedWay)
{
  const State x0 = State::Zero();
  const double dt = 0.1;
  {  // the turn map
    TrackingModel m;
    m.turn_delta_zero = -0.95;
    m.turn_scale = 0.025;
    const double w = 0.045, expected = dt * (w - 0.95 * 0.025 * std::tanh(w / 0.025));
    EXPECT_NEAR(discrete_step(x0, twist(0.0, w), dt, m).x_next(2), expected, 1e-12);
  }
  {  // the speed falling in turns: the arc length of a constant twist
    TrackingModel m;
    m.speed_turn = 0.17;
    const double v = 0.3, w = 0.4, vf = v * (1.0 - 0.17 * sabs(w));
    const State x = discrete_step(x0, twist(v, w), dt, m).x_next;
    const double chord = std::hypot(x(0), x(1)), arc = chord * (w * dt / 2.0) / std::sin(w * dt / 2.0);
    EXPECT_NEAR(arc, vf * dt, 1e-9);
  }
  {  // sideslip: to the left in a left turn, about sideslip v omega dt
    TrackingModel m;
    m.sideslip = 0.2;
    const double v = 0.3, w = 0.4;
    const State with = discrete_step(x0, twist(v, w), dt, m).x_next, without = discrete_step(x0, twist(v, w), dt).x_next;
    const double lateral = -std::sin(0.0) * (with(0) - without(0)) + std::cos(0.0) * (with(1) - without(1));
    EXPECT_GT(lateral, 0.0);
    EXPECT_NEAR(lateral, 0.2 * v * w * dt, 0.02 * 0.2 * v * w * dt);
  }
}

TEST(Map, ZeroAndTinyCommandsAreFinite)
{
  const TrackingModel m = full();
  for (double c : {0.0, 1e-6}) {
    const Step s = discrete_step(State::Zero(), twist(c, c), 0.1, m);
    EXPECT_TRUE(s.x_next.allFinite() && s.A.allFinite() && s.B.allFinite()) << c;
  }
  const Step s = discrete_step(State::Zero(), Control::Zero(), 0.1, m);
  EXPECT_EQ(s.x_next.head<3>().norm(), 0.0);
  // the slope the QP sees at a standstill: g_0 dt
  EXPECT_NEAR(s.B(2, 1), (1.0 + m.turn_delta_zero) * 0.1, 1e-9);
}

TEST(Map, MirroredCommandsGiveMirroredPredictions)
{
  const TrackingModel m = full();
  const State x0 = State::Zero();
  const State a = discrete_step(x0, twist(0.3, 0.4), 0.1, m).x_next;
  const State b = discrete_step(x0, twist(0.3, -0.4), 0.1, m).x_next;
  // driving backwards mirrors the terms odd in v (the turn map, the speed term, sideslip), not a turning centre off the
  // frame (spec Amendment 3: it stays behind the frame whichever way the base drives), so this part has no offset
  TrackingModel odd = m;
  odd.turn_offset = odd.turn_offset_com = 0.0;
  const State a_odd = discrete_step(x0, twist(0.3, 0.4), 0.1, odd).x_next;
  const State c = discrete_step(x0, twist(-0.3, 0.4), 0.1, odd).x_next;
  EXPECT_NEAR(b(0), a(0), 1e-12);
  EXPECT_NEAR(b(1), -a(1), 1e-12);
  EXPECT_NEAR(b(2), -a(2), 1e-12);
  EXPECT_NEAR(c(0), -a_odd(0), 1e-12);
  EXPECT_NEAR(c(1), -a_odd(1), 1e-12);
  EXPECT_NEAR(c(2), a_odd(2), 1e-12);
}

// ---- spec Amendment 3: the turning centre follows the centre of mass ----

namespace
{
State arm_at(std::initializer_list<double> q)
{
  State x = State::Zero();
  int i = 3;
  for (double v : q) x(i++) = v;
  return x;
}

// the body-frame lateral displacement of one step from x, relative to the unicycle's
double lateral_shift(const State & x, const Control & u, double dt, const TrackingModel & m)
{
  const State with = discrete_step(x, u, dt, m).x_next, without = discrete_step(x, u, dt).x_next;
  return -std::sin(x(2)) * (with(0) - without(0)) + std::cos(x(2)) * (with(1) - without(1));
}
}  // namespace

TEST(Offset, InPlaceTurnMovesTheFrameSidewaysByTheOffset)
{
  // a base turning in place about a point 0.05 m behind its frame: the frame moves left at 0.05 x yaw rate
  TrackingModel m;
  m.turn_offset = 0.05;
  EXPECT_FALSE(m.map_identity());
  const double w = 0.3, dt = 0.1;
  // the chord of the arc the frame swings through: 2 d sin(w dt / 2), at the mid heading
  EXPECT_NEAR(lateral_shift(State::Zero(), twist(0.0, w), dt, m), 2 * 0.05 * std::sin(w * dt / 2) * std::cos(w * dt / 2),
              1e-9);
  EXPECT_NEAR(lateral_shift(State::Zero(), twist(0.0, w), dt, m), 0.05 * w * dt, 1e-4);
  EXPECT_LT(lateral_shift(State::Zero(), twist(0.0, -w), dt, m), 0.0);  // a right turn moves it right
}

TEST(Offset, DrivingBackwardsKeepsTheOffsetsSide)
{
  // Task 10b review: the offset's lateral speed is d x yaw rate, even in the forward command: driving backwards in a
  // left turn moves the frame left as driving forwards does (the mirror test leaves this case out)
  TrackingModel m;
  m.turn_offset = 0.05;
  const double fwd = lateral_shift(State::Zero(), twist(0.3, 0.3), 0.1, m);
  const double back = lateral_shift(State::Zero(), twist(-0.3, 0.3), 0.1, m);
  EXPECT_GT(fwd, 0.0);
  EXPECT_GT(back, 0.0);
  EXPECT_NEAR(back, fwd, 0.1 * fwd);
}

TEST(Offset, ComXOfThePosesMatchesTheUrdf)
{
  // the whole robot's centre of mass in base_footprint (URDF links and the qualified payload), Task 10's values
  EXPECT_NEAR(com_x(arm_at({0, 0, 0, 0, 0, 0})), -0.0284, 1e-3);                   // home
  EXPECT_NEAR(com_x(arm_at({0, M_PI / 2, 0, 0, -M_PI / 2, 0})), 0.0603, 1e-3);     // level_extension
  EXPECT_NEAR(com_x(arm_at({0, 0.968, -1.735, 0, 0.767, 0})), -0.0104, 1e-3);      // carry
}

TEST(Offset, FollowsTheCentreOfMass)
{
  // turn_offset_com alone: the lateral shift is proportional to the centre of mass's x, so it changes sign between
  // home (behind) and level_extension (ahead)
  TrackingModel m;
  m.turn_offset_com = -2.0;
  const State home = arm_at({0, 0, 0, 0, 0, 0}), level = arm_at({0, M_PI / 2, 0, 0, -M_PI / 2, 0});
  const double a = lateral_shift(home, twist(0.0, 0.3), 0.1, m), b = lateral_shift(level, twist(0.0, 0.3), 0.1, m);
  EXPECT_GT(a, 0.0);
  EXPECT_LT(b, 0.0);
  EXPECT_NEAR(a / b, com_x(home) / com_x(level), 1e-6);
}

TEST(Offset, LoaderReadsAndBoundsTheOffsets)
{
  const TrackingModel m = load_tracking_model(write_file("offset.yaml", "turn_offset: 0.003\nturn_offset_com: -2.23\n"));
  EXPECT_DOUBLE_EQ(m.turn_offset, 0.003);
  EXPECT_DOUBLE_EQ(m.turn_offset_com, -2.23);
  for (const char * text : {"turn_offset: 0.4\n", "turn_offset_com: 6\n"})
    EXPECT_THROW(load_tracking_model(write_file("offset_range.yaml", text)), std::runtime_error) << text;
  const auto p = m.map_parameters();
  ASSERT_EQ(p.size(), 7u);
  EXPECT_DOUBLE_EQ(p[5], 0.003);
  EXPECT_DOUBLE_EQ(p[6], -2.23);
}

TEST(Loader, RejectsUnknownKeysNonFiniteAndOutOfRange)
{
  const std::pair<std::string, std::string> cases[] = {
    {"unknown.yaml", "turn_gain: 0.5\n"}, {"nonfinite.yaml", "sideslip: .nan\n"}, {"range.yaml", "speed_turn: 2.0\n"}};
  const char * keys[] = {"turn_gain", "sideslip", "speed_turn"};
  for (int i = 0; i < 3; ++i) {
    const std::string path = write_file(cases[i].first, cases[i].second);
    try {
      load_tracking_model(path);
      ADD_FAILURE() << cases[i].first << " loaded";
    } catch (const std::runtime_error & e) {
      const std::string what = e.what();
      EXPECT_NE(what.find(cases[i].first), std::string::npos) << what;
      EXPECT_NE(what.find(keys[i]), std::string::npos) << what;
    }
  }
}

TEST(Loader, MissingFileIsNamed)
{
  try {
    load_tracking_model("/nonexistent/tracking_model_e2.yaml");
    ADD_FAILURE() << "a missing file loaded";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("tracking_model_e2.yaml"), std::string::npos) << e.what();
  }
}

TEST(Loader, EmptyFileIsIdentity)
{
  EXPECT_TRUE(load_tracking_model(write_file("empty.yaml", "")).identity());
  const TrackingModel m = load_tracking_model(write_file("some.yaml", "turn_delta_zero: -0.9\nturn_scale: 0.03\nsideslip: 0.2\n"));
  EXPECT_DOUBLE_EQ(m.turn_delta_zero, -0.9);
  EXPECT_DOUBLE_EQ(m.turn_scale, 0.03);
  EXPECT_DOUBLE_EQ(m.sideslip, 0.2);
  EXPECT_FALSE(m.map_identity());
}

// ---- E2 Task 8: the state at actuation ----

namespace
{
Control command(double v, double w, double qd1)
{
  Control u = Control::Zero();
  u(0) = v;
  u(1) = w;
  u(2) = qd1;
  return u;
}
}  // namespace

TEST(Advance, HandComputedCase)
{
  TrackingModel m;
  m.base_delay_s = 0.04;
  m.arm_delay_s = 0.03;
  const double now = 10.0;
  const std::vector<SentCommand> sent{{now - 0.04, command(0.3, 0.0, 0.5)}, {now - 0.02, command(0.1, 0.0, 0.5)}};
  const State x = advance_to_actuation(State::Zero(), sent, now, m);
  EXPECT_NEAR(x(0), 0.3 * 0.02 + 0.1 * 0.02, 1e-12);
  EXPECT_NEAR(x(1), 0.0, 1e-12);
  EXPECT_NEAR(x(3), 0.5 * 0.03, 1e-12);  // both carry qdot1 0.5: the first over [now - 0.03, now - 0.02)
  for (int j = 4; j < kNx; ++j) EXPECT_EQ(x(j), 0.0);
}

TEST(Advance, ZeroDelaysReturnTheState)
{
  TrackingModel m;
  m.turn_delta_inf = m.turn_delta_zero = -0.2;
  State x;
  x << 1.0, 2.0, 0.3, 0.1, -0.2, 0.3, -0.4, 0.5, -0.6;
  const std::vector<SentCommand> sent{{9.98, command(0.3, 0.4, 0.5)}};
  const State a = advance_to_actuation(x, sent, 10.0, m);
  EXPECT_EQ(std::memcmp(a.data(), x.data(), sizeof(double) * kNx), 0);
}

TEST(Advance, ShortHistoryAndNoHistory)
{
  // Review Focus 3: a delay longer than the history covers only the commands sent (before the first: zero)
  TrackingModel m;
  m.base_delay_s = 0.2;
  m.arm_delay_s = 0.2;
  const double now = 5.0;
  const State one = advance_to_actuation(State::Zero(), {{now - 0.02, command(0.3, 0.0, 0.5)}}, now, m);
  ASSERT_TRUE(one.allFinite());
  EXPECT_NEAR(one(0), 0.3 * 0.02, 1e-12);
  EXPECT_NEAR(one(3), 0.5 * 0.02, 1e-12);
  State x;
  x << 1.0, 2.0, 0.3, 0.1, -0.2, 0.3, -0.4, 0.5, -0.6;
  const State none = advance_to_actuation(x, {}, now, m);
  EXPECT_EQ(none, x);
}

TEST(Advance, UsesTheCommandMap)
{
  TrackingModel m;
  m.turn_delta_inf = m.turn_delta_zero = -0.2;  // yaw rate 0.8 omega
  m.base_delay_s = 0.04;
  const State x = advance_to_actuation(State::Zero(), {{0.96, command(0.0, 0.5, 0.0)}}, 1.0, m);
  EXPECT_NEAR(x(2), 0.016, 1e-12);
}

TEST(Advance, CommandsSentLaterThanNowAreIgnored)
{
  // a command stamped at or after now has not acted yet
  TrackingModel m;
  m.base_delay_s = 0.04;
  const State x = advance_to_actuation(State::Zero(), {{0.98, command(0.3, 0.0, 0.0)}, {1.0, command(9.0, 0.0, 0.0)}},
                                       1.0, m);
  EXPECT_NEAR(x(0), 0.3 * 0.02, 1e-12);
}
