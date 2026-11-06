// The plant session (MuJoCo plant plan Task 8): resets, goals, events, frames, the run log and the summary.
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>

#include "mobile_manipulator_campaign/session.hpp"
#include "mobile_manipulator_plant/frames.hpp"
#include "mobile_manipulator_plant/run_log.hpp"

namespace mmc = mobile_manipulator_campaign;
namespace mmp = mobile_manipulator_plant;
using json = nlohmann::json;

namespace
{
bool has_geom_prefix(const mjModel * m, const std::string & prefix)
{
  for (int g = 0; g < m->ngeom; ++g) {
    const char * name = mj_id2name(m, mjOBJ_GEOM, g);
    if (name && std::string(name).rfind(prefix, 0) == 0) return true;
  }
  return false;
}
mmc::SessionConfig ideal()
{
  mmc::SessionConfig c;
  c.queue_frames = true;
  return c;
}
// A goal 2 m ahead of the open yard's start, in the open
mobile_manipulator_navigation::Pose2 ahead(const mmc::Session & s)
{
  const auto & x = s.start_state();
  return {x(0) + 2.0 * std::cos(x(2)), x(1) + 2.0 * std::sin(x(2)), x(2)};
}
}  // namespace

TEST(Session, ListsTheV2Scenarios)
{
  const auto names = mmc::Session::list_scenarios();
  EXPECT_NE(std::find(names.begin(), names.end(), "v2_open_yard_nav"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "v2_worker_crossing_nav"), names.end());
}

TEST(Session, ResetToOtherScenarioLeavesNothingBehind)
{
  mmc::Session s(ideal());
  s.reset("v2_worker_crossing_nav", "", std::nullopt, 1);
  ASSERT_TRUE(has_geom_prefix(s.plant().plant().model(), "mover/"));
  for (int k = 0; k < 25; ++k) s.step_cycle();
  EXPECT_GT(s.time(), 0.4);
  s.reset("v2_open_yard_nav", "", std::nullopt, 2);
  EXPECT_FALSE(has_geom_prefix(s.plant().plant().model(), "mover/"));
  EXPECT_FALSE(has_geom_prefix(s.plant().plant().model(), "scenario/"));
  EXPECT_DOUBLE_EQ(s.time(), 0.0);
  EXPECT_TRUE(s.take_events().empty());
  const auto frames = s.take_frames();
  ASSERT_FALSE(frames.empty());
  mmp::FrameType type{};
  json h;
  std::vector<std::vector<float>> b;
  size_t consumed = 0;
  ASSERT_TRUE(mmp::decode(frames.front(), type, h, b, consumed));
  EXPECT_EQ(type, mmp::FrameType::Hello);
  EXPECT_EQ(h.at("scenario"), "v2_open_yard_nav");
}

TEST(Session, EmitsGoalReached)
{
  mmc::Session s(ideal());
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  s.set_goal(ahead(s));
  int cycles = 0;
  while (s.step_cycle() && cycles < 2000) ++cycles;
  const auto events = s.take_events();
  ASSERT_FALSE(events.empty());
  EXPECT_EQ(events.back().at("event"), "goal_reached");
  EXPECT_FALSE(s.active());
  const auto frames = s.take_frames();
  const size_t states = std::count_if(frames.begin(), frames.end(), [](const std::string & f) {
    mmp::FrameType type{};
    json h;
    std::vector<std::vector<float>> b;
    size_t consumed = 0;
    return mmp::decode(f, type, h, b, consumed) && type == mmp::FrameType::State;
  });
  EXPECT_EQ(static_cast<int>(states), cycles);  // one per stepped cycle
}

TEST(Session, SummaryHasProfileFields)
{
  mmc::Session s(ideal());
  s.reset("v2_open_yard_nav", "", std::nullopt, 3);
  s.set_goal(ahead(s));
  while (s.step_cycle()) {
  }
  const json r = s.summary();
  EXPECT_EQ(r.at("plant_profile"), "mujoco-ideal");
  EXPECT_EQ(r.at("seed"), 3);
  EXPECT_EQ(r.at("cell"), "v2_open_yard_nav");
  EXPECT_FALSE(r.at("site_checksum").get<std::string>().empty());
  EXPECT_TRUE(r.at("reached").get<bool>());
  EXPECT_GT(r.at("real_time_factor").get<double>(), 0.0);
  for (const char * key : {"end_status", "time_s", "b5_cycles", "following_cycles", "fallbacks", "min_clearance_m",
                           "plant_min_clearance_m", "plant_contacts", "host", "start_perturbation", "goals"})
    EXPECT_TRUE(r.contains(key)) << key;
}

TEST(Session, SeedsPerturbTheStart)
{
  mmc::Session s(ideal());
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const auto a = s.start_state();
  s.reset("v2_open_yard_nav", "", std::nullopt, 2);
  const auto b = s.start_state();
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const auto c = s.start_state();
  EXPECT_EQ(a, c);  // the same seed, the same start
  EXPECT_NE(a.head<3>(), b.head<3>());
  EXPECT_LE((a.head<2>() - b.head<2>()).cwiseAbs().maxCoeff(), 0.002);
  EXPECT_LE(std::abs(a(2) - b(2)), 0.002);
}

TEST(Session, WritesARunLog)
{
  auto config = ideal();
  config.log_path = (std::filesystem::temp_directory_path() / "test_session.mmlog").string();
  {
    mmc::Session s(config);
    s.reset("v2_open_yard_nav", "", std::nullopt, 1);
    for (int k = 0; k < 50; ++k) s.step_cycle();
  }
  mmp::RunLogReader r(config.log_path);
  EXPECT_EQ(r.header().at("scenario"), "v2_open_yard_nav");
  std::string frame;
  int n = 0;
  while (r.next(frame)) ++n;
  EXPECT_GE(n, 50);
  std::filesystem::remove(config.log_path);
}

TEST(Session, ATimeoutIsATimeoutEvent)
{
  // the record's end_status is B5's last status (as medium_bench closed); the event says the run timed out
  auto config = ideal();
  config.timeout_s = 1.0;
  mmc::Session s(config);
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  while (s.step_cycle()) {
  }
  const auto events = s.take_events();
  ASSERT_FALSE(events.empty());
  EXPECT_EQ(events.back().at("event"), "timeout");
  EXPECT_TRUE(s.summary().at("timed_out").get<bool>());
}

TEST(Session, CommandsBeforeResetThrow)
{
  // a demo client may click a goal before choosing a scenario: an error, not a crash
  mmc::Session s(ideal());
  EXPECT_THROW(s.set_goal({1.0, 2.0, 0.0}), std::logic_error);
  EXPECT_THROW(s.move_body("reserved/0", {0.0, 0.0, 0.9}, 0.0), std::logic_error);
  EXPECT_FALSE(s.step_cycle());
}

TEST(Session, LogStaysOpenForGoalsAfterTheLast)
{
  // a goal clicked after the run's goals ended keeps logging (the log closes at the next reset or the end)
  const auto path = std::filesystem::temp_directory_path() / "session_after_last.mmlog";
  auto c = ideal();
  c.log_path = path.string();
  {
    mmc::Session s(c);
    s.reset("v2_open_yard_nav", "", std::nullopt, 1);
    s.set_goal(ahead(s));
    while (s.step_cycle()) {
    }
    const auto x = s.plant().state();
    s.set_goal({x(0) + 1.0 * std::cos(x(2)), x(1) + 1.0 * std::sin(x(2)), x(2)});
    int cycles = 0;
    EXPECT_NO_THROW(while (s.step_cycle() && cycles < 2000) ++cycles);
    EXPECT_GT(cycles, 0);
  }
  mmp::RunLogReader log(path.string());
  std::string frame;
  int reached = 0;
  while (log.next(frame)) {
    mmp::FrameType type{};
    json h;
    std::vector<std::vector<float>> b;
    size_t used = 0;
    ASSERT_TRUE(mmp::decode(frame, type, h, b, used));
    reached += type == mmp::FrameType::Event && h.at("event") == "goal_reached";
  }
  EXPECT_EQ(reached, 2);  // both goals in the one log
  std::filesystem::remove(path);
}
