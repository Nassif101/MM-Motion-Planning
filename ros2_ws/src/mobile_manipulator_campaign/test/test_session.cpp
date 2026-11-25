// The plant session (MuJoCo plant plan Task 8): resets, goals, events, frames, the run log and the summary.
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <tuple>

#include "mobile_manipulator_campaign/closed_loop_case.hpp"
#include "mobile_manipulator_campaign/session.hpp"
#include "mobile_manipulator_plant/frames.hpp"
#include "mobile_manipulator_plant/run_log.hpp"
#include "mobile_manipulator_mpc/model.hpp"
#include "mobile_manipulator_mpc/replay.hpp"
#include "mobile_manipulator_mpc/tracking_model.hpp"

namespace mmc = mobile_manipulator_campaign;
namespace mmp = mobile_manipulator_plant;
namespace mmm = mobile_manipulator_mpc;
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
                           "plant_min_clearance_m", "plant_contacts", "host", "start_perturbation", "goals", "stall_s"})
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

namespace
{
std::string bytes_of(const std::filesystem::path & p)
{
  std::ifstream in(p, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(in), {});
}
std::string run_logged(const std::string & name, uint64_t seed)
{
  const auto path = std::filesystem::temp_directory_path() / name;
  auto c = ideal();
  c.log_path = path.string();
  {
    mmc::Session s(c);  // the medium layer on (E2 covers F2 logs)
    s.reset("v2_open_yard_nav", "", std::nullopt, seed);
    s.set_goal(ahead(s));
    while (s.step_cycle()) {
    }
  }
  const std::string b = bytes_of(path);
  std::filesystem::remove(path);
  return b;
}
}  // namespace

TEST(Session, SameSeedSameLogBytes)
{
  // E2 for run logs: nothing of the wall clock (solve times) in the frames
  const std::string a = run_logged("e2_a.mmlog", 4), b = run_logged("e2_b.mmlog", 4);
  ASSERT_GT(a.size(), 1000u);
  EXPECT_TRUE(a == b);
}

TEST(Session, HelloCarriesProvenance)
{
  auto c = ideal();
  c.provenance = {{"git_commit", "abc123"}};
  mmc::Session s(c);
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  json hello;
  for (const auto & f : s.take_frames()) {
    mmp::FrameType type{};
    std::vector<std::vector<float>> b;
    size_t used = 0;
    json h;
    if (mmp::decode(f, type, h, b, used) && type == mmp::FrameType::Hello) hello = h;
  }
  ASSERT_FALSE(hello.is_null());
  EXPECT_EQ(hello.at("provenance").at("git_commit"), "abc123");
  EXPECT_EQ(hello.at("profile_sha256").get<std::string>().size(), 64u);
  EXPECT_EQ(hello.at("robot_model_sha256").get<std::string>().size(), 64u);
  EXPECT_EQ(hello.at("site_model_sha256").get<std::string>().size(), 64u);
  const json & config = hello.at("config");
  for (const char * key : {"medium", "medium_config", "lidar", "scan_decimation", "timeout_s", "iteration_budget", "perturb_start",
                           "stall_s"})
    EXPECT_TRUE(config.contains(key)) << key;
}

TEST(Session, MissionGoalsMergeReplansOfOneGoal)
{
  // Nav2 publishes a /plan per replan: consecutive plans to the same goal are one mission goal (its first route)
  auto goal = [](double x, std::vector<Eigen::Vector2d> pts) {
    mmm::RecordedGoal g;
    g.points = std::move(pts);
    g.goal = {x, 0.0, 0.0};
    return g;
  };
  const std::vector<Eigen::Vector2d> line{{0.0, 0.0}, {1.0, 0.0}};
  const auto goals = mmc::mission_goals({goal(1.0, line), goal(1.0, {{0.5, 0.0}, {1.0, 0.0}}), goal(2.0, line),
                                         goal(2.0, {{0.0, 0.0}}), goal(1.0, line)});
  ASSERT_EQ(goals.size(), 3u);
  EXPECT_EQ(goals[0].goal[0], 1.0);
  EXPECT_EQ(goals[0].points.front().x(), 0.0);  // the first route of the run
  EXPECT_EQ(goals[1].goal[0], 2.0);
  EXPECT_EQ(goals[2].goal[0], 1.0);
}

TEST(Session, MissionRunsItsGoalsInOrder)
{
  // two goals: the second starts where and when the first ended
  mmc::Session s(ideal());
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const auto & x = s.start_state();
  const Eigen::Vector2d a(x(0), x(1)), b = a + 2.0 * Eigen::Vector2d(std::cos(x(2)), std::sin(x(2)));
  const Eigen::Vector2d c = b + 1.5 * Eigen::Vector2d(std::cos(x(2)), std::sin(x(2)));
  s.set_mission({{mmm::Route({a, b}), {b.x(), b.y(), x(2)}}, {mmm::Route({b, c}), {c.x(), c.y(), x(2)}}});
  int cycles = 0;
  while (s.step_cycle() && cycles < 4000) ++cycles;
  std::vector<json> reached;
  for (const auto & e : s.take_events())
    if (e.at("event") == "goal_reached") reached.push_back(e);
  ASSERT_EQ(reached.size(), 2u);
  EXPECT_EQ(reached[0].at("goal"), 0);
  EXPECT_EQ(reached[1].at("goal"), 1);
  EXPECT_GT(reached[1].at("t").get<double>(), reached[0].at("t").get<double>());
  const json r = s.summary();
  EXPECT_TRUE(r.at("reached").get<bool>());
  EXPECT_EQ(r.at("goals").size(), 2u);
  EXPECT_NEAR(s.plant().state()(0), c.x(), 0.15);
  EXPECT_NEAR(s.plant().state()(1), c.y(), 0.15);
}

TEST(Session, TimesOutAtTheScenariosTimeout)
{
  // Unity's runner gives each scenario its own timeout_s (store_room 300 s, low_beam_123 120 s): the replicas must too
  auto config_timeout = [](const std::string & scenario) {
    mmc::Session s(ideal());
    s.reset(scenario, "", std::nullopt, 1);
    for (const auto & f : s.take_frames()) {
      mmp::FrameType type{};
      json h;
      std::vector<std::vector<float>> b;
      size_t used = 0;
      if (mmp::decode(f, type, h, b, used) && type == mmp::FrameType::Hello) return h.at("config").at("timeout_s").get<double>();
    }
    return -1.0;
  };
  EXPECT_EQ(config_timeout("v2_store_room_nav"), 300.0);
  EXPECT_EQ(config_timeout("v2_low_beam_123_nav"), 120.0);
  auto c = ideal();
  c.timeout_s = 5.0;  // an explicit timeout wins
  mmc::Session s(c);
  s.reset("v2_store_room_nav", "", std::nullopt, 1);
  int cycles = 0;
  while (s.step_cycle()) ++cycles;
  EXPECT_LE(cycles, 5 * 50 + 2);
}

TEST(Session, ProfilesTiming)
{
  // gate P: the plant step's wall time per cycle against the 20 ms cycle, and the scans' cast times
  auto c = ideal();
  c.profile_timing = true;
  c.lidar = true;
  c.medium = false;
  mmc::Session s(c);
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  s.set_goal(ahead(s));
  for (int k = 0; k < 60 && s.step_cycle(); ++k) {
  }
  const json timing = s.summary().at("timing");
  EXPECT_GT(timing.at("plant_step_ms").at("p50").get<double>(), 0.0);
  EXPECT_GE(timing.at("plant_step_ms").at("p99").get<double>(), timing.at("plant_step_ms").at("p50").get<double>());
  EXPECT_GT(timing.at("plant_step_fraction_of_real_time").get<double>(), 0.0);
  EXPECT_GE(timing.at("scan_cast_ms").at("n").get<int>(), 5);
  EXPECT_GT(timing.at("scan_cast_ms").at("p99").get<double>(), 0.0);
}

TEST(Session, ProfilesB5Timing)
{
  // E2 Task 9 (T7): with profile_timing the summary also has B5's QP solve time and its cycle time without the
  // diagnostics, one sample per B5 cycle
  auto c = ideal();
  c.profile_timing = true;
  c.medium = false;
  c.diagnostics = true;
  mmc::Session s(c);
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  s.set_goal(ahead(s));
  int cycles = 0;
  for (int k = 0; k < 60 && s.step_cycle(); ++k) ++cycles;
  const json timing = s.summary().at("timing");
  const json solve = timing.at("b5_solve_ms"), cycle = timing.at("b5_cycle_ms");
  EXPECT_EQ(solve.at("n").get<int>(), cycles);
  EXPECT_EQ(cycle.at("n").get<int>(), cycles);
  EXPECT_GT(solve.at("p50").get<double>(), 0.0);
  EXPECT_GE(solve.at("p99").get<double>(), solve.at("p50").get<double>());
  EXPECT_GE(cycle.at("p50").get<double>(), solve.at("p50").get<double>()) << "a cycle includes its solve";
}

TEST(Session, B5TimingSampleTakesTheCycleTimeAsTheControllerReportsIt)
{
  // Task 9 review I3: CycleTiming::total_s already leaves the diagnostics out; the sample must not subtract them again
  mmm::CycleTiming tm;
  tm.solve_s = 0.001;
  tm.total_s = 0.003;
  tm.diagnostics_s = 0.002;
  const auto [solve, cycle] = mmc::b5_timing_sample(tm);
  EXPECT_EQ(solve, 0.001);
  EXPECT_EQ(cycle, 0.003);
}

TEST(Session, StateFramesCarryTheSlackGate)
{
  // diagnosing holds (gate R's store room): B5's slack gate each cycle, [gate_slack, gate_violation, unsafe]
  auto c = ideal();
  c.medium = false;
  mmc::Session s(c);
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  s.set_goal(ahead(s));
  for (int k = 0; k < 5; ++k) s.step_cycle();
  int states = 0;
  for (const auto & f : s.take_frames()) {
    mmp::FrameType type{};
    json h;
    std::vector<std::vector<float>> b;
    size_t used = 0;
    if (!mmp::decode(f, type, h, b, used) || type != mmp::FrameType::State) continue;
    ++states;
    ASSERT_TRUE(h.contains("gate")) << h.dump();
    ASSERT_EQ(h.at("gate").size(), 3u);
    EXPECT_GE(h.at("gate")[0].get<double>(), 0.0);
  }
  EXPECT_EQ(states, 5);
}

TEST(Session, FailedResetKeepsThePreviousRun)
{
  // final review: a reset that throws (a V1 scenario: the plant has only the V2 site) must leave the previous run
  // whole, not a plant that was never reset (move_body on it dereferenced a null model)
  mmc::Session s(ideal());
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  EXPECT_ANY_THROW(s.reset("overhead_course_nav", "", std::nullopt, 1));
  EXPECT_NO_THROW(s.move_body("reserved/0", {-3.0, -4.0, 0.9}, 0.0));
  EXPECT_EQ(s.summary().at("cell").get<std::string>(), "v2_open_yard_nav");
  EXPECT_TRUE(s.step_cycle());
}

TEST(Session, SameSeedSameLogBytesRealisticWithLidar)
{
  // E2 in the campaigns' configuration: realistic (the wheels base), closed loop with F2, the lidar on
  auto run = [](const std::string & name) {
    const auto path = std::filesystem::temp_directory_path() / name;
    auto c = ideal();
    c.mode = mmp::PlantMode::Realistic;
    c.lidar = true;
    c.log_path = path.string();
    {
      mmc::Session s(c);
      s.reset("v2_open_yard_nav", "", std::nullopt, 6);
      s.set_goal(ahead(s));
      for (int k = 0; k < 400 && s.step_cycle(); ++k) {
      }
    }
    const std::string b = bytes_of(path);
    std::filesystem::remove(path);
    return b;
  };
  const std::string a = run("e2_real_a.mmlog"), b = run("e2_real_b.mmlog");
  ASSERT_GT(a.size(), 100000u);
  EXPECT_TRUE(a == b);
}

TEST(Session, HelloHashesTheModelTheModeBuilds)
{
  // ideal mode builds the planar robot.xml, realistic the wheels model: the Hello's hash names the one simulated
  auto hash = [](mmp::PlantMode mode) {
    auto c = ideal();
    c.mode = mode;
    mmc::Session s(c);
    s.reset("v2_open_yard_nav", "", std::nullopt, 1);
    for (const auto & f : s.take_frames()) {
      mmp::FrameType type{};
      json h;
      std::vector<std::vector<float>> b;
      size_t used = 0;
      if (mmp::decode(f, type, h, b, used) && type == mmp::FrameType::Hello) return h.at("robot_model_sha256").get<std::string>();
    }
    return std::string();
  };
  const std::string ideal_hash = hash(mmp::PlantMode::Ideal), realistic_hash = hash(mmp::PlantMode::Realistic);
  ASSERT_EQ(ideal_hash.size(), 64u);
  EXPECT_NE(ideal_hash, realistic_hash);
}

TEST(Session, StallEndsARunThatStopsMakingProgress)
{
  // door_105's straight start-goal line runs into the building's wall: B5 stops there; without a stall detector the run
  // waits out its 150 s timeout
  auto c = ideal();
  c.stall_s = 3.0;
  c.medium = false;
  mmc::Session s(c);
  s.reset("v2_door_105_nav", "", std::nullopt, 1);
  while (s.step_cycle()) {
  }
  bool stalled = false;
  for (const auto & e : s.take_events()) stalled = stalled || e.at("event") == "stalled";
  EXPECT_TRUE(stalled);
  const json r = s.summary();
  EXPECT_FALSE(r.at("reached").get<bool>());
  EXPECT_TRUE(r.at("stalled").get<bool>());
  EXPECT_FALSE(r.at("timed_out").get<bool>());
  EXPECT_LT(r.at("time_s").get<double>(), 60.0);
}

TEST(Session, StallDetectorLeavesAMovingRunAlone)
{
  auto c = ideal();
  c.stall_s = 3.0;
  mmc::Session s(c);
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  s.set_goal(ahead(s));
  while (s.step_cycle()) {
  }
  const json r = s.summary();
  EXPECT_TRUE(r.at("reached").get<bool>());
  EXPECT_FALSE(r.at("stalled").get<bool>());
}

TEST(Session, ReachedAfterClickedGoals)
{
  // a goal clicked after a reached one: the summary covers the goals set last
  mmc::Session s(ideal());
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  s.set_goal(ahead(s));
  while (s.step_cycle()) {
  }
  const auto x = s.plant().state();
  s.set_goal({x(0) + 1.0 * std::cos(x(2)), x(1) + 1.0 * std::sin(x(2)), x(2)});
  while (s.step_cycle()) {
  }
  EXPECT_TRUE(s.summary().at("reached").get<bool>());
}

TEST(Session, SummaryTimeIsTheGoalsTime)
{
  auto c = ideal();
  c.timeout_s = 1.0;
  mmc::Session s(c);
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  while (s.step_cycle()) {
  }
  const json r = s.summary();
  EXPECT_DOUBLE_EQ(r.at("time_s").get<double>(), r.at("goals").back().at("time_s").get<double>());
}

TEST(Session, CellNamesTheCorpusCell)
{
  auto c = ideal();
  c.cell = "v2_door_105_nav@20261101T163932Z";
  mmc::Session s(c);
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  EXPECT_EQ(s.summary().at("cell").get<std::string>(), "v2_door_105_nav@20261101T163932Z");
}

// ---- E1 (spec 2026-11-08 Section 3.2-3.3): the decision trace, the B5 config variant and the start nudge ----
namespace
{
std::vector<std::pair<mmp::FrameType, json>> headers(mmc::Session & s)
{
  std::vector<std::pair<mmp::FrameType, json>> out;
  for (const auto & f : s.take_frames()) {
    mmp::FrameType type{};
    json h;
    std::vector<std::vector<float>> b;
    size_t used = 0;
    if (mmp::decode(f, type, h, b, used)) out.emplace_back(type, h);
  }
  return out;
}

json hello_of(mmc::Session & s)
{
  for (const auto & [type, h] : headers(s))
    if (type == mmp::FrameType::Hello) return h;
  return json();
}

std::string shipped_b5_yaml()
{
  return ament_index_cpp::get_package_share_directory("mobile_manipulator_mpc") + "/config/wholebody_mpc.yaml";
}

// a copy of the shipped B5 config with its slack_gate_inside line replaced by `rule`
std::string b5_variant(const std::string & name, const std::string & rule)
{
  std::ifstream in(shipped_b5_yaml());
  const auto path = std::filesystem::temp_directory_path() / name;
  std::ofstream out(path);
  for (std::string line; std::getline(in, line);)
    out << (line.rfind("slack_gate_inside:", 0) == 0 ? "slack_gate_inside: " + rule : line) << "\n";
  return path.string();
}
}  // namespace

TEST(Session, StateFramesCarryTheDwell)
{
  auto c = ideal();
  c.medium = false;
  mmc::Session s(c);
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  s.set_goal(ahead(s));
  for (int k = 0; k < 5; ++k) s.step_cycle();
  int states = 0;
  for (const auto & [type, h] : headers(s)) {
    if (type != mmp::FrameType::State) continue;
    ++states;
    ASSERT_TRUE(h.contains("dwell")) << h.dump();
    const int dwell = h.at("dwell").get<int>();
    EXPECT_TRUE(dwell == 0 || dwell == 1);
    if (dwell == 1) EXPECT_EQ(h.at("status"), "holding");
  }
  EXPECT_EQ(states, 5);
}

TEST(Session, PlansFramesCarryTheDecision)
{
  mmc::Session s(ideal());  // the medium layer on
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  s.set_goal(ahead(s));
  for (int k = 0; k < 60; ++k) s.step_cycle();
  int plans = 0;
  for (const auto & [type, h] : headers(s)) {
    if (type != mmp::FrameType::Plans) continue;
    ++plans;
    for (const char * key : {"seed", "full_multistart", "waiting", "homotopy_switched", "seeds"})
      ASSERT_TRUE(h.contains(key)) << key << " in " << h.dump();
    ASSERT_FALSE(h.at("seeds").empty());
    bool winner_listed = false;
    for (const auto & row : h.at("seeds")) {
      ASSERT_EQ(row.size(), 5u) << row.dump();  // name, merit, feasible 0/1, progress_m, merged 0/1 (CP1)
      EXPECT_TRUE(row[2] == 0 || row[2] == 1);
      EXPECT_TRUE(row[4] == 0 || row[4] == 1);
      winner_listed = winner_listed || row[0] == h.at("seed");
    }
    if (h.at("status") != "stale_reused") EXPECT_TRUE(winner_listed) << h.dump();  // stale: the reused plan's winner
  }
  EXPECT_GT(plans, 0);
}

TEST(Session, StartNudgeMovesOnlyTheYaw)
{
  mmc::Session plain(ideal());
  plain.reset("v2_open_yard_nav", "", std::nullopt, 5);
  auto c = ideal();
  c.start_nudge_rad = 1e-12;
  mmc::Session nudged(c);
  nudged.reset("v2_open_yard_nav", "", std::nullopt, 5);
  const auto a = plain.start_state(), b = nudged.start_state();
  for (int i = 0; i < a.size(); ++i)
    if (i != 2) EXPECT_EQ(a(i), b(i)) << i;
  EXPECT_NEAR(b(2) - a(2), 1e-12, 1e-15);
  EXPECT_EQ(plain.summary().at("start_nudge_rad").get<double>(), 0.0);
  EXPECT_EQ(nudged.summary().at("start_nudge_rad").get<double>(), 1e-12);
  EXPECT_EQ(hello_of(nudged).at("config").at("start_nudge_rad").get<double>(), 1e-12);
}

TEST(Session, DiagnosticsAddTheGateDepthTraceWithoutChangingTheRun)
{
  // E1 Task 9: with diagnostics the State frames carry diag = [deepest pair's depth m, its stage-1 prediction m,
  // the plan's linearization error m or null]; the run itself (state blocks, outcome) is unchanged
  auto run = [](bool diagnostics) {
    auto c = ideal();
    c.medium = false;
    c.diagnostics = diagnostics;
    mmc::Session s(c);
    s.reset("v2_open_yard_nav", "", std::nullopt, 2);
    s.set_goal(ahead(s));
    for (int k = 0; k < 40; ++k) s.step_cycle();
    std::vector<std::vector<float>> poses;
    int with_diag = 0;
    for (const auto & f : s.take_frames()) {
      mmp::FrameType type{};
      json h;
      std::vector<std::vector<float>> b;
      size_t used = 0;
      if (!mmp::decode(f, type, h, b, used) || type != mmp::FrameType::State) continue;
      poses.push_back(b.at(0));
      if (h.contains("diag")) {
        ++with_diag;
        EXPECT_EQ(h.at("diag").size(), 3u) << h.dump();
      }
    }
    return std::make_pair(poses, with_diag);
  };
  const auto plain = run(false), diag = run(true);
  EXPECT_EQ(plain.second, 0);
  EXPECT_EQ(diag.second, 40);
  EXPECT_EQ(plain.first, diag.first) << "diagnostics changed the run";
}

TEST(Session, DiagnosticsCarryPairGradientAndCommand)
{
  // E2 Task 1: under diagnostics every State frame also carries diag_pair (the deepest gated pair, empty without one),
  // diag_grad (its distance gradient, 9) and diag_cmd (the command B5 sent this cycle, 8); without diagnostics none
  auto run = [](bool diagnostics) {
    auto c = ideal();
    c.medium = false;
    c.diagnostics = diagnostics;
    mmc::Session s(c);
    s.reset("v2_open_yard_nav", "", std::nullopt, 2);
    s.set_goal(ahead(s));
    for (int k = 0; k < 40; ++k) s.step_cycle();
    std::vector<std::pair<json, std::vector<std::vector<float>>>> states;
    for (const auto & f : s.take_frames()) {
      mmp::FrameType type{};
      json h;
      std::vector<std::vector<float>> b;
      size_t used = 0;
      if (mmp::decode(f, type, h, b, used) && type == mmp::FrameType::State) states.emplace_back(h, b);
    }
    return states;
  };
  const auto plain = run(false), diag = run(true);
  ASSERT_EQ(plain.size(), 40u);
  ASSERT_EQ(diag.size(), 40u);
  for (const auto & [h, b] : plain)
    for (const char * key : {"diag_pair", "diag_grad", "diag_kink", "diag_cmd", "diag_plan_u0", "diag_nearest"})
      EXPECT_FALSE(h.contains(key)) << key;
  int moving = 0, arm_moving = 0;
  for (const auto & [h, b] : diag) {
    ASSERT_TRUE(h.contains("diag_cmd") && h.contains("diag_grad") && h.contains("diag_pair")) << h.dump();
    ASSERT_EQ(h.at("diag_cmd").size(), 8u);
    ASSERT_EQ(h.at("diag_grad").size(), 9u);
    ASSERT_EQ(h.at("diag_kink").size(), 9u);
    // E2 Task 4b: [link, obstacle, distance, in stage 1 0/1, in any row 0/1, rows dropped]
    ASSERT_TRUE(h.contains("diag_nearest")) << h.dump();
    ASSERT_EQ(h.at("diag_nearest").size(), 6u);
    const bool deep = h.at("diag").at(0).get<double>() > 0.0;
    EXPECT_EQ(deep, !h.at("diag_pair").get<std::string>().empty()) << h.dump();
    // the ideal plant executes the base command as sent (zero while static friction holds it)
    const double v = h.at("diag_cmd").at(0).get<double>(), w = h.at("diag_cmd").at(1).get<double>();
    const double ev = b.at(2).at(0), ew = b.at(2).at(1);
    if (ev != 0.0 || ew != 0.0) {
      ++moving;
      EXPECT_NEAR(ev, v, 1e-6);
      EXPECT_NEAR(ew, w, 1e-6);
    }
    // and the arm's joint velocities as sent (Task 1 response check: all six entries, not only the base)
    for (size_t i = 2; i < 8; ++i) {
      const double requested = h.at("diag_cmd").at(i).get<double>(), executed = b.at(2).at(i);
      EXPECT_NEAR(executed, requested, 1e-7 + 1e-6 * std::abs(requested)) << "joint " << i - 2;
      arm_moving += std::abs(executed) > 1e-6;
    }
  }
  EXPECT_GT(moving, 10) << "the base never moved: the command check saw nothing";
  EXPECT_GT(arm_moving, 10) << "the arm never moved: the arm entries were not checked";
}

namespace
{
// The State frames of a diagnostics run: (header, blocks), in order.
std::vector<std::pair<json, std::vector<std::vector<float>>>> diagnostic_states(mmp::PlantMode mode, int cycles)
{
  auto c = ideal();
  c.medium = false;
  c.mode = mode;
  c.diagnostics = true;
  mmc::Session s(c);
  s.reset("v2_open_yard_nav", "", std::nullopt, 2);
  s.set_goal(ahead(s));
  for (int k = 0; k < cycles && s.step_cycle(); ++k) {
  }
  std::vector<std::pair<json, std::vector<std::vector<float>>>> states;
  for (const auto & f : s.take_frames()) {
    mmp::FrameType type{};
    json h;
    std::vector<std::vector<float>> b;
    size_t used = 0;
    if (mmp::decode(f, type, h, b, used) && type == mmp::FrameType::State) states.emplace_back(h, b);
  }
  return states;
}
}  // namespace

TEST(Session, DiagnosticCommandDrivesTheTransitionIntoItsFrame)
{
  // E2 Task 1 review F2: a State frame at t carries the command sent at t - 0.02 (and the gate's pair and gradient
  // evaluated then): the transition from record j-1's pose to record j's uses record j's diag_cmd. On the ideal plant
  // (B5's own model, executed as sent) that replay is exact up to the float32 pose blocks; the start frame's command
  // is a cycle off and is not.
  const auto states = diagnostic_states(mmp::PlantMode::Ideal, 60);
  ASSERT_EQ(states.size(), 60u);
  auto pose = [&](size_t j) {
    mmm::State x;
    for (int i = 0; i < 9; ++i) x(i) = states[j].second.at(0).at(static_cast<size_t>(i));
    return x;
  };
  auto cmd = [&](size_t j) {
    mmm::Control u;
    for (int i = 0; i < 8; ++i) u(i) = states[j].first.at("diag_cmd").at(static_cast<size_t>(i)).get<double>();
    return u;
  };
  double ending = 0.0, starting = 0.0;
  for (size_t j = 1; j < states.size(); ++j) {
    const double dt = states[j].first.at("t").get<double>() - states[j - 1].first.at("t").get<double>();
    ASSERT_NEAR(dt, 0.02, 1e-9);
    ending = std::max(ending, (mmm::discrete_step(pose(j - 1), cmd(j), dt).x_next - pose(j)).head<3>().cwiseAbs().maxCoeff());
    starting =
      std::max(starting, (mmm::discrete_step(pose(j - 1), cmd(j - 1), dt).x_next - pose(j)).head<3>().cwiseAbs().maxCoeff());
  }
  EXPECT_LT(ending, 2e-6);
  EXPECT_GT(starting, 2e-5) << "the speed changes along the run, so a cycle's offset shows";
}

TEST(Session, DiagnosticCommandIsTheRequestedControlNotTheExecutedOne)
{
  // E2 Task 1 review F1: diag_cmd is the control B5 requested (what B5's model assumes; the identified delay and lag
  // are the whole command pipeline's). In realistic mode the base lags its command and the arm follows the trajectory
  // built from the plan (the executed block's last six), so the executed block is not the command.
  const auto states = diagnostic_states(mmp::PlantMode::Realistic, 150);
  ASSERT_GT(states.size(), 100u);
  double base = 0.0, arm = 0.0;
  for (const auto & [h, b] : states) {
    for (size_t i = 0; i < 2; ++i) base = std::max(base, std::abs(h.at("diag_cmd").at(i).get<double>() - b.at(2).at(i)));
    for (size_t i = 2; i < 8; ++i) arm = std::max(arm, std::abs(h.at("diag_cmd").at(i).get<double>() - b.at(2).at(i)));
  }
  EXPECT_GT(base, 1e-3) << "the realistic base never lagged its logged command: is diag_cmd the executed twist?";
  EXPECT_GT(arm, 0.0) << "the requested arm velocity equals the trajectory's in every frame: is diag_cmd the reference?";
}

TEST(Session, DiagnosticsCarryThePlansFirstControl)
{
  // E2 Task 4: diag_plan_u0 is the control B5's plan asked for before the output stage (the rate band around the last
  // command, the boxes and the breakaway boost from rest), so the measurement can tell what the plan predicted from
  // what was sent. Starting from rest the boost raises a planned creep to the breakaway: the two differ there.
  const auto states = diagnostic_states(mmp::PlantMode::Ideal, 60);
  ASSERT_EQ(states.size(), 60u);
  int differ = 0, equal = 0;
  for (const auto & [h, b] : states) {
    ASSERT_TRUE(h.contains("diag_plan_u0")) << h.dump();
    ASSERT_EQ(h.at("diag_plan_u0").size(), 8u);
    double gap = 0.0;
    for (size_t i = 0; i < 8; ++i)
      gap = std::max(gap, std::abs(h.at("diag_plan_u0").at(i).get<double>() - h.at("diag_cmd").at(i).get<double>()));
    (gap > 1e-6 ? differ : equal) += 1;
  }
  EXPECT_GT(differ, 0) << "the output stage never changed the plan's control: is diag_plan_u0 the command?";
  EXPECT_GT(equal, 0) << "the plan's control never went out unchanged: is diag_plan_u0 the plan's?";
}

TEST(Session, TheDepthGuardIsTracedWhenOn)
{
  // E1 CP3: State frames carry guard = [held 0/1, deepest measured depth] when the B5 config turns the guard on
  auto guard_frames = [](const std::string & config) {
    auto c = ideal();
    c.medium = false;
    c.b5_config = config;
    mmc::Session s(c);
    s.reset("v2_open_yard_nav", "", std::nullopt, 1);
    s.set_goal(ahead(s));
    for (int k = 0; k < 5; ++k) s.step_cycle();
    int n = 0;
    for (const auto & [type, h] : headers(s))
      if (type == mmp::FrameType::State && h.contains("guard")) {
        ++n;
        EXPECT_EQ(h.at("guard").size(), 2u);
      }
    return n;
  };
  const auto on = std::filesystem::temp_directory_path() / "e1_b5_guard.yaml";
  {
    std::ifstream in(shipped_b5_yaml());
    std::ofstream out(on);
    for (std::string line; std::getline(in, line);)
      out << (line.rfind("slack_gate_depth_guard_m:", 0) == 0 ? "slack_gate_depth_guard_m: 0.002" : line) << "\n";
  }
  EXPECT_EQ(guard_frames(""), 0);
  EXPECT_EQ(guard_frames(on.string()), 5);
}

TEST(Session, B5ConfigVariantIsLoadedAndHashed)
{
  mmc::Session shipped(ideal());
  shipped.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const json base = hello_of(shipped).at("config");
  EXPECT_EQ(base.at("gate_rule"), "absolute");  // the shipped default since E1 (2026-11-09)
  EXPECT_EQ(base.at("b5_config_sha256").get<std::string>().size(), 64u);
  auto c = ideal();
  c.b5_config = b5_variant("e1_b5_increment.yaml", "increment");  // the frozen rule as a variant
  mmc::Session variant(c);
  variant.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const json v = hello_of(variant).at("config");
  EXPECT_EQ(v.at("gate_rule"), "increment");
  EXPECT_EQ(v.at("b5_config"), c.b5_config);
  EXPECT_NE(v.at("b5_config_sha256"), base.at("b5_config_sha256"));
  EXPECT_EQ(variant.summary().at("gate_rule"), "increment");
  EXPECT_EQ(variant.summary().at("b5_config_sha256"), v.at("b5_config_sha256"));
  // the same bytes, the same hash
  c.b5_config = b5_variant("e1_b5_absolute.yaml", "absolute");
  mmc::Session copy(c);
  copy.reset("v2_open_yard_nav", "", std::nullopt, 1);
  EXPECT_EQ(hello_of(copy).at("config").at("b5_config_sha256"), base.at("b5_config_sha256"));
}

namespace
{
std::string mujoco_tracking_yaml()
{
  return ament_index_cpp::get_package_share_directory("mobile_manipulator_mpc") + "/config/tracking_model_mujoco.yaml";
}
}  // namespace

TEST(Session, DefaultTrackingModelIsTheMujocoOne)
{
  // E2 Task 11 (CP4, user 2026-11-14): a plant run whose B5 config names no tracking model predicts with the installed
  // MuJoCo model and corrects its turn gain alone; "none" is the unicycle; a config that names a model keeps it and
  // its own correction; --tracking-adaptation still overrides
  mmc::Session plain(ideal());
  plain.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const json d = hello_of(plain).at("config");
  EXPECT_EQ(d.at("tracking_model"), mujoco_tracking_yaml());
  EXPECT_EQ(d.at("tracking_model_sha256").get<std::string>().size(), 64u);
  EXPECT_EQ(d.at("tracking_adaptation"), true);
  EXPECT_EQ(d.at("tracking_adapt_sideslip"), false);
  auto c = ideal();
  c.tracking_model = "none";
  mmc::Session unicycle(c);
  unicycle.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const json u = hello_of(unicycle).at("config");
  EXPECT_EQ(u.at("tracking_model"), "");
  EXPECT_EQ(u.at("tracking_adaptation"), false);
  c = ideal();
  c.tracking_adaptation = 0;
  mmc::Session fixed(c);
  fixed.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const json f = hello_of(fixed).at("config");
  EXPECT_EQ(f.at("tracking_model"), mujoco_tracking_yaml());
  EXPECT_EQ(f.at("tracking_adaptation"), false);
  // a B5 config naming its own model (and no correction) keeps both
  std::ifstream in(shipped_b5_yaml());
  const auto own = std::filesystem::temp_directory_path() / "e2_own_tracking_model.yaml";
  {
    std::ofstream out(own);
    for (std::string line; std::getline(in, line);)
      out << (line.rfind("tracking_model:", 0) == 0 ? "tracking_model: " + mujoco_tracking_yaml() : line) << "\n";
  }
  c = ideal();
  c.b5_config = own.string();
  mmc::Session configured(c);
  configured.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const json o = hello_of(configured).at("config");
  EXPECT_EQ(o.at("tracking_model"), mujoco_tracking_yaml());
  EXPECT_EQ(o.at("tracking_adaptation"), false);  // the config's, not the default's (on)
}

TEST(Session, ConfigTrackingModelReadsEveryYamlSpelling)
{
  // final review C3: whether a B5 config names a tracking model comes from B5's own loader (the scripts ask
  // plant_campaign --print-config-tracking-model), so every YAML spelling counts, and an empty value is none
  const std::string m = mujoco_tracking_yaml();
  const std::pair<std::string, std::string> cases[] = {
    {"tracking_model: \"" + m + "\"", m}, {"tracking_model: '" + m + "'", m}, {"\"tracking_model\": " + m, m},
    {"tracking_model: " + m + "  # a comment", m}, {"tracking_model: ''", ""}, {"tracking_model: \"\"", ""}};
  int n = 0;
  for (const auto & [line, expected] : cases) {
    std::ifstream in(shipped_b5_yaml());
    const auto path = std::filesystem::temp_directory_path() / ("e2_spelling_" + std::to_string(n++) + ".yaml");
    {
      std::ofstream out(path);
      for (std::string l; std::getline(in, l);) out << (l.rfind("tracking_model:", 0) == 0 ? line : l) << "\n";
    }
    EXPECT_EQ(mmc::config_tracking_model(path.string()), expected) << line;
  }
  EXPECT_EQ(mmc::config_tracking_model(""), "");  // the installed config names none
}

TEST(Session, NoneIsTheUnicycleEvenUnderAnAdaptiveConfig)
{
  // final review C2: "none" is the unicycle, so it turns off a correction the B5 config enables (else the estimator
  // would learn a turn map onto the identity prior); asking for a correction with "none" is refused
  std::ifstream in(shipped_b5_yaml());
  const auto adaptive = std::filesystem::temp_directory_path() / "e2_adaptive_tracking.yaml";
  {
    std::ofstream out(adaptive);
    for (std::string line; std::getline(in, line);) {
      if (line.rfind("tracking_model:", 0) == 0) line = "tracking_model: " + mujoco_tracking_yaml();
      if (line.rfind("tracking_adaptation:", 0) == 0) line = "tracking_adaptation: true";
      out << line << "\n";
    }
  }
  auto c = ideal();
  c.b5_config = adaptive.string();
  c.tracking_model = "none";
  mmc::Session s(c);
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const json h = hello_of(s).at("config");
  EXPECT_EQ(h.at("tracking_model"), "");
  EXPECT_EQ(h.at("tracking_adaptation"), false);
  for (int adaptation : {1, 2}) {
    c.tracking_adaptation = adaptation;
    mmc::Session contradictory(c);
    EXPECT_THROW(contradictory.reset("v2_open_yard_nav", "", std::nullopt, 1), std::invalid_argument) << adaptation;
  }
  c.tracking_adaptation = 0;  // off with none: consistent
  mmc::Session off(c);
  EXPECT_NO_THROW(off.reset("v2_open_yard_nav", "", std::nullopt, 1));
}

TEST(Session, HelloRecordsTheTrackingModel)
{
  // E2 Task 7: the hello and the summary say which tracking model B5 predicted with (its path and hash; "" the
  // identity) and whether the online correction ran. --tracking-model overrides the B5 config's; "none" the identity
  // (pinned here since the MuJoCo model became the plant runs' default, Task 11).
  auto none = ideal();
  none.tracking_model = "none";
  mmc::Session shipped(none);
  shipped.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const json base = hello_of(shipped).at("config");
  EXPECT_EQ(base.at("tracking_model"), "");
  EXPECT_EQ(base.at("tracking_model_sha256"), "");
  EXPECT_EQ(base.at("tracking_adaptation"), false);
  EXPECT_EQ(base.at("tracking_adapt_sideslip"), false);  // recorded; the shipped config's (false since CP4)
  auto c = ideal();
  c.tracking_model = mujoco_tracking_yaml();
  c.tracking_adaptation = 1;
  mmc::Session modelled(c);
  modelled.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const json m = hello_of(modelled).at("config");
  EXPECT_EQ(m.at("tracking_model"), c.tracking_model);
  EXPECT_EQ(m.at("tracking_model_sha256").get<std::string>().size(), 64u);
  EXPECT_EQ(m.at("tracking_adaptation"), true);
  EXPECT_EQ(modelled.summary().at("tracking_model_sha256"), m.at("tracking_model_sha256"));
  EXPECT_EQ(modelled.summary().at("tracking_adaptation"), true);
  EXPECT_EQ(modelled.summary().at("tracking_adapt_sideslip"), false);  // the shipped config's
  c.tracking_adaptation = 2;  // Task 10c: --tracking-adaptation gain, the turn gain alone
  mmc::Session gain(c);
  gain.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const json g = hello_of(gain).at("config");  // the hello is read once: the session's frames are drained
  EXPECT_EQ(g.at("tracking_adaptation"), true);
  EXPECT_EQ(g.at("tracking_adapt_sideslip"), false);
  c.tracking_model = "none";
  c.tracking_adaptation = -1;
  mmc::Session identity(c);
  identity.reset("v2_open_yard_nav", "", std::nullopt, 1);
  EXPECT_EQ(hello_of(identity).at("config").at("tracking_model"), "");
  c.tracking_model = (std::filesystem::temp_directory_path() / "e2_no_such_tracking_model.yaml").string();
  mmc::Session missing(c);
  EXPECT_ANY_THROW(missing.reset("v2_open_yard_nav", "", std::nullopt, 1));
}

TEST(Session, DiagnosticsCarryTheTrackingEstimate)
{
  // E2 Task 7: under diagnostics every State frame carries diag_track = [g_inf, g_0, sideslip k], the tracking model
  // the cycle planned with: the identity's [1, 1, 0] with "none" (the default before Task 11), the file's numbers with
  // the model and no correction
  auto tracks = [](const std::string & model) {
    auto c = ideal();
    c.medium = false;
    c.diagnostics = true;
    c.tracking_model = model;
    mmc::Session s(c);
    s.reset("v2_open_yard_nav", "", std::nullopt, 2);
    s.set_goal(ahead(s));
    for (int k = 0; k < 20; ++k) s.step_cycle();
    std::vector<std::vector<double>> out;
    for (const auto & [type, h] : headers(s))
      if (type == mmp::FrameType::State) {
        EXPECT_TRUE(h.contains("diag_track")) << h.dump();
        if (h.contains("diag_track")) out.push_back(h.at("diag_track").get<std::vector<double>>());
      }
    return out;
  };
  const auto identity = tracks("none");
  ASSERT_EQ(identity.size(), 20u);
  for (const auto & t : identity) EXPECT_EQ(t, (std::vector<double>{1.0, 1.0, 0.0}));
  const auto modelled = tracks(mujoco_tracking_yaml());
  ASSERT_EQ(modelled.size(), 20u);
  const auto file = mmm::load_tracking_model(mujoco_tracking_yaml());
  for (const auto & t : modelled) {
    ASSERT_EQ(t.size(), 3u);
    EXPECT_DOUBLE_EQ(t[0], 1.0 + file.turn_delta_inf);
    EXPECT_DOUBLE_EQ(t[1], 1.0 + file.turn_delta_zero);
    EXPECT_DOUBLE_EQ(t[2], file.sideslip);
  }
}

TEST(Session, RowTraceLeavesOtherFramesIdentical)
{
  // E3 Task 1: with row_trace one State frame per 0.1 s (every fifth at 50 Hz) carries rows (one
  // [pair, type, stage, margin, d_lin, d_plan, g0..g8] per traced row), rows_x (the plan at stages 1, 2, 5, 10) and
  // rows_u (its controls at stages 0..9); Hello's config names the period and B5's stage. The trace turns the
  // diagnostics on (review I1: its analysis reads diag_cmd): against a diagnostics run every other part of every
  // frame is unchanged.
  auto run = [](bool trace) {
    auto c = ideal();
    c.medium = false;
    c.row_trace = trace;
    c.diagnostics = !trace;
    mmc::Session s(c);
    s.reset("v2_door_105_nav", "", std::nullopt, 1);
    s.set_goal(ahead(s));
    for (int k = 0; k < 40; ++k) s.step_cycle();
    std::vector<std::tuple<mmp::FrameType, json, std::vector<std::vector<float>>>> frames;
    for (const auto & f : s.take_frames()) {
      mmp::FrameType type{};
      json h;
      std::vector<std::vector<float>> b;
      size_t used = 0;
      if (mmp::decode(f, type, h, b, used)) frames.emplace_back(type, h, b);
    }
    return frames;
  };
  auto plain = run(false), traced = run(true);
  ASSERT_EQ(plain.size(), traced.size());
  int with_rows = 0, rows_total = 0, state_index = 0;
  for (size_t i = 0; i < traced.size(); ++i) {
    json & h = std::get<1>(traced[i]);
    json & p = std::get<1>(plain[i]);
    ASSERT_EQ(std::get<0>(traced[i]), std::get<0>(plain[i])) << "frame " << i;
    if (h.contains("config")) {  // Hello
      EXPECT_DOUBLE_EQ(h.at("config").at("row_trace").get<double>(), 0.1);
      EXPECT_DOUBLE_EQ(h.at("config").at("row_trace_stage_s").get<double>(), 0.1);
      EXPECT_TRUE(h.at("config").at("diagnostics").get<bool>());
      EXPECT_FALSE(p.at("config").contains("row_trace")) << "review M1: no trace key without the trace";
      EXPECT_FALSE(p.at("config").contains("row_trace_stage_s"));
      h.at("config").erase("row_trace");
      h.at("config").erase("row_trace_stage_s");
    }
    if (h.contains("rows_x")) {
      ++with_rows;
      EXPECT_EQ(state_index % 5, 0) << "a traced frame off the period";
      EXPECT_EQ(h.at("rows_x").size(), 4u * 9u);
      EXPECT_EQ(h.at("rows_u").size(), 10u * 8u);
      for (const json & r : h.at("rows")) {
        ASSERT_EQ(r.size(), 15u) << r.dump();
        EXPECT_EQ(r.at(0).get<std::string>().size() > 0, true);
        const std::string type = r.at(1).get<std::string>();
        EXPECT_TRUE(type == "arm" || type == "panel" || type == "base" || type == "self" || type == "floor") << type;
        ++rows_total;
      }
      h.erase("rows");
      h.erase("rows_x");
      h.erase("rows_u");
    }
    if (std::get<0>(traced[i]) == mmp::FrameType::State) {
      EXPECT_TRUE(h.contains("diag_cmd") && h.contains("diag_track")) << "review I1: the analysis' inputs";
      ++state_index;
    }
    EXPECT_EQ(h, p) << "frame " << i;
    EXPECT_EQ(std::get<2>(traced[i]), std::get<2>(plain[i])) << "frame " << i;
  }
  EXPECT_EQ(with_rows, 8) << "40 cycles at a period of 5";
  EXPECT_GT(rows_total, 0) << "no row traced at door_105's start: the case does not test the encoding";
}

// E3 Task 3: the tightness cells' routes (their obstacles: test_scenario_setup.cpp E3Cells)
TEST(E3Cells, TurnCellsFollowTheirOwnRoute)
{
  // a straight cell: the line from its start to its goal; a turn cell: the scenario's route, 2.5 m east, a quarter
  // turn of radius 1 m, then north, the obstacle's centre 0.5 m after the turn's end
  const mmc::ClosedLoopCase straight = mmc::make_case("e3_beam_straight_1", "");
  EXPECT_EQ(straight.route.points().size(), 2u);
  EXPECT_NEAR(straight.route.length(), 9.0, 1e-9);
  for (const std::string name : {"e3_beam_turn_1", "e3_gap_turn_4"}) {
    const mmc::ClosedLoopCase turn = mmc::make_case(name, "");
    const auto & p = turn.route.points();
    ASSERT_EQ(p.size(), 9u) << name;
    EXPECT_NEAR((p.front() - Eigen::Vector2d(-5.0, -8.0)).norm(), 0.0, 1e-6);
    EXPECT_NEAR((p.back() - Eigen::Vector2d(-1.5, -3.5)).norm(), 0.0, 1e-6);
    EXPECT_NEAR((p[7] - Eigen::Vector2d(-1.5, -7.0)).norm(), 0.0, 1e-3) << "the turn ends heading north at y -7.0";
    EXPECT_NEAR(-6.5 - p[7].y(), 0.5, 1e-3) << "the obstacle (y -6.5) is 0.5 m after the turn";
    EXPECT_NEAR(turn.route.length(), 2.5 + 1.5708 + 3.5, 0.01) << name;  // the polyline's chords are 0.005 m short
    EXPECT_NEAR(turn.goal[2], 1.570796, 1e-6);
    EXPECT_NEAR(turn.start(2), 0.0, 1e-9);
  }
}

TEST(Session, AnExplicitMediumConfigIsFingerprinted)
{
  // E3 final review I2: an explicit medium config's bytes are recorded (Hello config and summary), so a resume can tell
  // an edited file from the one a run used; the default (installed medium.yaml) keeps the Hello it had.
  const std::string installed =
    ament_index_cpp::get_package_share_directory("mobile_manipulator_medium") + "/config/medium.yaml";
  std::ifstream in(installed);
  std::stringstream text;
  text << in.rdbuf();
  const std::string a = ::testing::TempDir() + "e3_medium_a.yaml", b = ::testing::TempDir() + "e3_medium_b.yaml";
  std::ofstream(a) << text.str();
  std::ofstream(b) << text.str() << "\n# edited\n";
  auto hello_and_summary = [](const std::string & medium_config) {
    auto c = ideal();
    c.medium_config = medium_config;
    c.queue_frames = true;
    mmc::Session s(c);
    s.reset("v2_door_105_nav", "", std::nullopt, 1);
    json hello;
    for (const auto & f : s.take_frames()) {
      mmp::FrameType type{};
      json h;
      std::vector<std::vector<float>> blocks;
      size_t used = 0;
      if (mmp::decode(f, type, h, blocks, used) && type == mmp::FrameType::Hello) hello = h;
    }
    return std::make_pair(hello, s.summary());
  };
  const auto [hello_a, summary_a] = hello_and_summary(a);
  const auto [hello_b, summary_b] = hello_and_summary(b);
  const auto [hello_default, summary_default] = hello_and_summary("");
  const std::string sha_a = hello_a.at("config").at("medium_config_sha256");
  EXPECT_EQ(sha_a.size(), 64u);
  EXPECT_EQ(summary_a.at("medium_config_sha256"), sha_a);
  EXPECT_NE(hello_b.at("config").at("medium_config_sha256"), sha_a);
  EXPECT_FALSE(hello_default.at("config").contains("medium_config_sha256"));
  EXPECT_FALSE(summary_default.contains("medium_config_sha256"));
}

// Spec 4B-2 Task 1: the viewer's new data (stream-protocol.md, Prediction; Plans block 1; Scan block 2)
namespace
{
struct Decoded
{
  mmp::FrameType type{};
  json header;
  std::vector<std::vector<float>> blocks;
};
std::string capabilities(const std::vector<Decoded> & frames)
{
  for (const auto & d : frames)
    if (d.type == mmp::FrameType::Hello && d.header.contains("capabilities")) return d.header.at("capabilities").dump();
  return "";
}
std::vector<Decoded> decoded(mmc::Session & s)
{
  std::vector<Decoded> out;
  for (const auto & f : s.take_frames()) {
    Decoded d;
    size_t used = 0;
    if (mmp::decode(f, d.type, d.header, d.blocks, used)) out.push_back(std::move(d));
  }
  return out;
}
}  // namespace

TEST(Session, PredictionFrameLayoutAndRate)
{
  auto c = ideal();
  c.prediction = true;
  mmc::Session s(c);
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  s.set_goal(ahead(s));
  for (int k = 0; k < 50; ++k) s.step_cycle();
  int predictions = 0, states = 0;
  const auto frames = decoded(s);
  for (const auto & d : frames) {
    if (d.type == mmp::FrameType::State) ++states;
    if (d.type != mmp::FrameType::Prediction) continue;
    ++predictions;
    EXPECT_DOUBLE_EQ(d.header.at("stage_dt").get<double>(), 0.1);
    const int stages = d.header.at("stages").get<int>();
    EXPECT_EQ(stages, 20);
    ASSERT_EQ(d.blocks.size(), 1u);
    EXPECT_EQ(d.blocks[0].size(), static_cast<size_t>((stages + 1) * 9));
    for (const char * key : {"t", "status", "plan_id"}) EXPECT_TRUE(d.header.contains(key)) << key;
  }
  EXPECT_EQ(states, 50);
  EXPECT_EQ(predictions, 10) << "every fifth B5 cycle";
  EXPECT_NE(capabilities(frames).find("b5_prediction"), std::string::npos);
}

TEST(Session, PredictionDoesNotChangeTheRun)
{
  auto run = [](bool prediction) {
    auto c = ideal();
    c.prediction = prediction;
    mmc::Session s(c);
    s.reset("v2_open_yard_nav", "", std::nullopt, 2);
    s.set_goal(ahead(s));
    for (int k = 0; k < 60; ++k) s.step_cycle();
    std::vector<std::vector<float>> states;
    for (const auto & d : decoded(s))
      if (d.type == mmp::FrameType::State) states.push_back(d.blocks.at(0));
    return states;
  };
  EXPECT_EQ(run(false), run(true)) << "the prediction changed the run";
}

TEST(Session, DisplaySamplesMatchThePlanSamples)
{
  mmc::Session s(ideal());  // the medium layer on
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  s.set_goal(ahead(s));
  for (int k = 0; k < 60; ++k) s.step_cycle();
  int checked = 0;
  const auto frames = decoded(s);
  for (const auto & d : frames) {
    if (d.type != mmp::FrameType::Plans || d.header.at("status") == "stale_reused") continue;
    ASSERT_GE(d.blocks.size(), 2u) << "block 1: the display samples";
    const auto & display = d.blocks[1];
    ASSERT_EQ(display.size() % 10, 0u);
    const size_t n = display.size() / 10;
    ASSERT_GE(n, 2u);
    for (size_t i = 0; i < n; ++i) EXPECT_NEAR(display[i * 10], 0.1 * static_cast<double>(i), 1e-5);
    for (int k = 0; k < 9; ++k) EXPECT_NEAR(display[1 + k], d.blocks[0][k], 1e-5) << "the first sample, value " << k;
    ++checked;
  }
  EXPECT_GT(checked, 0);
  EXPECT_NE(capabilities(frames).find("plan_display_samples"), std::string::npos);
}

TEST(Session, ScanCarriesPointTimes)
{
  auto c = ideal();
  c.lidar = true;
  mmc::Session s(c);
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  for (int k = 0; k < 30; ++k) s.step_cycle();
  int scans = 0;
  const auto frames = decoded(s);
  for (const auto & d : frames) {
    if (d.type != mmp::FrameType::Scan) continue;
    ++scans;
    ASSERT_EQ(d.blocks.size(), 3u);
    const auto & times = d.blocks[2];
    ASSERT_EQ(times.size(), d.blocks[1].size()) << "one time per point";
    const double span = d.header.at("t_end").get<double>() - d.header.at("t_start").get<double>();
    for (size_t i = 0; i < times.size(); ++i) {
      EXPECT_GE(times[i], 0.0f);
      EXPECT_LE(times[i], span + 1e-6);
      if (i > 0) EXPECT_GE(times[i], times[i - 1]) << "ray-time order";
    }
  }
  EXPECT_GT(scans, 0);
  EXPECT_NE(capabilities(frames).find("scan_point_times"), std::string::npos);
}

// Spec 4B-2 Task 1 review M3: the one-in-five phase restarts with each run (the demo server reuses its Session)
TEST(Session, PredictionPhaseRestartsWithTheRun)
{
  auto c = ideal();
  c.prediction = true;
  mmc::Session s(c);
  for (int run = 0; run < 2; ++run) {
    s.reset("v2_open_yard_nav", "", std::nullopt, 1);
    s.set_goal(ahead(s));
    s.take_frames();
    for (int k = 0; k < 2; ++k) s.step_cycle();  // without the restart, the second run's two are cycles 2 and 3
    int predictions = 0;
    for (const auto & d : decoded(s)) predictions += d.type == mmp::FrameType::Prediction;
    EXPECT_EQ(predictions, 1) << "run " << run << ": the first B5 cycle's";
  }
}

namespace
{
using SeedFrames = std::vector<std::tuple<mmp::FrameType, json, std::vector<std::vector<float>>>>;
// Medium diagnosis Task 1: a short F2 run at the 1.05 m door, with or without the seed detail
SeedFrames run_seed_detail(bool detail)
{
  auto c = ideal();
  c.seed_detail = detail;
  mmc::Session s(c);
  s.reset("v2_door_105_nav", "", std::nullopt, 1);
  s.set_goal(ahead(s));
  for (int k = 0; k < 150; ++k) s.step_cycle();
  SeedFrames frames;
  for (const auto & f : s.take_frames()) {
    mmp::FrameType type{};
    json h;
    std::vector<std::vector<float>> b;
    size_t used = 0;
    if (mmp::decode(f, type, h, b, used)) frames.emplace_back(type, h, b);
  }
  return frames;
}
}  // namespace

TEST(Session, SeedDetailAppendsToTheSeedRows)
{
  // Each seed row: the five fields of E1's trace, then slack, collision_m, torque, speed_mps, hull, iterations,
  // converged, resumed, homotopy, deepest_link, deepest_obstacle, deepest_distance, deepest_t, interrupted (review I1),
  // cost_terms (Task 4b).
  // Review M4: the values agree with each other (slack the soft maximum, the deepest row present exactly when the
  // collision violates, then distance + margin = collision_m's margin and its time on the 0.16 s grid)
  const SeedFrames plain = run_seed_detail(false), detailed = run_seed_detail(true);
  ASSERT_EQ(plain.size(), detailed.size());
  int rows = 0, violating = 0;
  for (size_t i = 0; i < plain.size(); ++i) {
    if (std::get<0>(plain[i]) != mmp::FrameType::Plans) continue;
    const json & a = std::get<1>(plain[i]).at("seeds");
    const json & b = std::get<1>(detailed[i]).at("seeds");
    ASSERT_EQ(a.size(), b.size()) << "frame " << i;
    for (size_t r = 0; r < a.size(); ++r, ++rows) {
      ASSERT_EQ(a[r].size(), 5u);
      ASSERT_EQ(b[r].size(), 20u);
      for (size_t k = 0; k < 5; ++k) EXPECT_EQ(a[r][k], b[r][k]) << "frame " << i << " row " << r << " field " << k;
      const json & row = b[r];
      const double slack = row[5], collision = row[6], torque = row[7], speed = row[8];
      EXPECT_DOUBLE_EQ(slack, std::max({collision, torque, speed})) << "frame " << i << " row " << r;
      EXPECT_TRUE(row[10].is_number_integer());  // iterations
      EXPECT_TRUE(row[13].is_string());          // homotopy
      EXPECT_TRUE(row[18] == 0 || row[18] == 1);  // interrupted
      // Task 4b: the cost terms, an object; with the slack penalty they add up to the merit (medium.yaml's weights:
      // slack 1000, slack_quadratic 10000 for each of the four kinds)
      ASSERT_TRUE(row[19].is_object());
      EXPECT_TRUE(row[19].contains("progress"));
      if (row[1].is_number()) {
        double total = 0.0;
        for (const auto & [name, value] : row[19].items()) total += value.get<double>();
        for (const int k : {6, 7, 8, 9}) {  // collision, torque, speed, hull (Violations::penalty)
          const double v = row[k].get<double>();
          total += 1000.0 * v + 0.5 * 10000.0 * v * v;
        }
        EXPECT_NEAR(total, row[1].get<double>(), 1e-6 * std::max(1.0, std::abs(row[1].get<double>())))
          << "frame " << i << " row " << r;
      }
      if (collision > 0.0) {
        ++violating;
        EXPECT_FALSE(row[15].get<std::string>().empty()) << "frame " << i << " row " << r;
        EXPECT_NEAR(collision + row[16].get<double>(), 0.05, 1e-9);  // the arm and panel rows' margin (medium.yaml)
        const double t = row[17];
        EXPECT_NEAR(t / 0.16, std::round(t / 0.16), 1e-9);
        EXPECT_GT(t, 0.0);  // review M5: stage 0 is not a row
      } else {
        EXPECT_EQ(row[15], "");
        EXPECT_TRUE(row[16].is_null());
      }
    }
  }
  EXPECT_GT(rows, 0);
  EXPECT_GT(violating, 0) << "the case must have violating seeds";
}

TEST(Session, SeedDetailLeavesOtherFramesIdentical)
{
  // Off, the Hello has no seed_detail key (as the row trace, E3 review M1); every frame but the Plans' seed rows and
  // the Hello's key is equal
  SeedFrames plain = run_seed_detail(false), detailed = run_seed_detail(true);
  ASSERT_EQ(plain.size(), detailed.size());
  for (size_t i = 0; i < plain.size(); ++i) {
    ASSERT_EQ(std::get<0>(plain[i]), std::get<0>(detailed[i])) << "frame " << i;
    json & a = std::get<1>(plain[i]);
    json & b = std::get<1>(detailed[i]);
    if (std::get<0>(plain[i]) == mmp::FrameType::Hello) {
      EXPECT_FALSE(a.at("config").contains("seed_detail"));
      EXPECT_TRUE(b.at("config").at("seed_detail").get<bool>());
      b.at("config").erase("seed_detail");
    }
    if (std::get<0>(plain[i]) == mmp::FrameType::Plans) {
      for (auto & row : b.at("seeds")) row.erase(row.begin() + 5, row.end());
    }
    EXPECT_EQ(a, b) << "frame " << i;
    EXPECT_EQ(std::get<2>(plain[i]), std::get<2>(detailed[i])) << "frame " << i;
  }
}

// --- Medium diagnosis Task 4: the fork (a setting changed from a time on, the run otherwise the same) ---
namespace
{
struct ForkRun
{
  SeedFrames frames;
  json summary;
};
ForkRun run_fork(const mmc::ForkConfig & fork, int cycles = 150, const std::string & scenario = "v2_door_105_nav")
{
  auto c = ideal();
  c.fork = fork;
  mmc::Session s(c);
  s.reset(scenario, "", std::nullopt, 1);
  s.set_goal(ahead(s));
  for (int k = 0; k < cycles && s.step_cycle(); ++k) {
  }
  ForkRun r;
  for (const auto & f : s.take_frames()) {
    mmp::FrameType type{};
    json h;
    std::vector<std::vector<float>> b;
    size_t used = 0;
    if (mmp::decode(f, type, h, b, used)) r.frames.emplace_back(type, h, b);
  }
  r.summary = s.summary();
  return r;
}
mmc::ForkConfig fork_at(double at, std::map<std::string, std::string> set = {}, int cycles = 0)
{
  mmc::ForkConfig f;
  f.at = at;
  f.set = std::move(set);
  f.cycles = cycles;
  return f;
}
double frame_t(const json & h) { return h.contains("t") ? h.at("t").get<double>() : -1.0; }
}  // namespace

TEST(Session, ForkWithNoSettingReproducesTheRunWhole)
{
  const ForkRun plain = run_fork({}), forked = run_fork(fork_at(1.0));
  ASSERT_EQ(plain.frames.size(), forked.frames.size());
  for (size_t i = 1; i < plain.frames.size(); ++i) {  // the Hello names the fork
    EXPECT_EQ(std::get<1>(plain.frames[i]), std::get<1>(forked.frames[i])) << "frame " << i;
    EXPECT_EQ(std::get<2>(plain.frames[i]), std::get<2>(forked.frames[i])) << "frame " << i;
  }
}

TEST(Session, ForkedRunMatchesUpToTheForkTime)
{
  const ForkRun plain = run_fork({}), forked = run_fork(fork_at(1.0, {{"iteration_budget", "5"}}));
  bool later_differs = false;
  for (size_t i = 1; i < std::min(plain.frames.size(), forked.frames.size()); ++i) {
    const json & a = std::get<1>(plain.frames[i]);
    const json & b = std::get<1>(forked.frames[i]);
    if (frame_t(a) < 1.0 - 1e-9) {
      EXPECT_EQ(a, b) << "frame " << i;
      EXPECT_EQ(std::get<2>(plain.frames[i]), std::get<2>(forked.frames[i])) << "frame " << i;
    } else if (a != b || std::get<2>(plain.frames[i]) != std::get<2>(forked.frames[i])) {
      later_differs = true;
    }
  }
  EXPECT_TRUE(later_differs) << "the setting must change the run after the fork";
}

TEST(Session, OneCycleForkStopsAfterOnePlan)
{
  const ForkRun r = run_fork(fork_at(1.0, {{"iteration_budget", "60"}}, 1), 400);
  int after = 0;
  for (const auto & f : r.frames)
    if (std::get<0>(f) == mmp::FrameType::Plans && frame_t(std::get<1>(f)) >= 1.0 - 1e-9) ++after;
  EXPECT_EQ(after, 1);
  EXPECT_EQ(r.summary.at("end_status"), "forked");
  EXPECT_LT(r.summary.at("time_s").get<double>(), 2.0);
}

TEST(Session, ForkIsRecordedInHelloAndSummary)
{
  const ForkRun r = run_fork(fork_at(1.0, {{"medium_arm_margin", "0.04"}}, 2), 400);
  const json & hello = std::get<1>(r.frames.front());
  EXPECT_DOUBLE_EQ(hello.at("config").at("fork").at("at").get<double>(), 1.0);
  EXPECT_EQ(hello.at("config").at("fork").at("set").at("medium_arm_margin"), "0.04");
  EXPECT_EQ(hello.at("config").at("fork").at("cycles"), 2);
  const json & f = r.summary.at("fork");
  EXPECT_GE(f.at("applied_t").get<double>(), 1.0);
  EXPECT_LT(f.at("applied_t").get<double>(), 1.0 + 0.4 + 1e-9);  // the next medium cycle
  EXPECT_EQ(f.at("cycle_wall_s").size(), 2u);
  EXPECT_EQ(f.at("deadline_cut_cycles"), 0);
  const ForkRun plain = run_fork({});
  EXPECT_FALSE(std::get<1>(plain.frames.front()).at("config").contains("fork")) << "no key without a fork";
  EXPECT_FALSE(plain.summary.contains("fork"));
}

TEST(Session, UnknownForkKeyIsRejected)
{
  for (const auto & set : std::vector<std::map<std::string, std::string>>{
         {{"no_such_key", "1"}}, {{"medium_arm_margin", "abc"}}, {{"iteration_budget", "0"}}, {{"max_iter", "-3"}},
         {{"b5_max_infeasible_s", "-0.5"}}, {{"seeds", "no_such_seed"}}, {{"full_every_cycle", "2"}},
         // Task 4 review I1: B5's margin would not reach the realistic plant's arm hold; M1: above the base margin
         {{"b5_arm_margin", "0.03"}}, {{"medium_arm_margin", "0.5"}}}) {
    auto c = ideal();
    c.fork = fork_at(1.0, set);
    EXPECT_THROW(
      {
        mmc::Session s(c);
        s.reset("v2_door_105_nav", "", std::nullopt, 1);
      },
      std::invalid_argument)
      << set.begin()->first << "=" << set.begin()->second;
  }
}

TEST(Session, ForkAfterTheEndIsReportedNotApplied)
{
  const ForkRun r = run_fork(fork_at(1e6, {{"iteration_budget", "60"}}));
  EXPECT_TRUE(r.summary.at("fork").at("applied_t").is_null());
}

TEST(Session, ForkAppliesInAGeneratedRun)
{
  // a lattice session: the fork applies at the first medium cycle at or after T, whichever stepper runs it
  auto c = ideal();
  c.generated_routes = true;
  c.auto_goal = true;
  c.fork = fork_at(0.5, {{"iteration_budget", "60"}});
  mmc::Session s(c);
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  for (int k = 0; k < 200 && s.advance(); ++k) {
  }
  const json f = s.summary().at("fork");
  ASSERT_FALSE(f.at("applied_t").is_null());
  EXPECT_GE(f.at("applied_t").get<double>(), 0.5);
}

TEST(Session, ForkB5LimitReachesTheStepper)
{
  // e3_gap_turn_4 seed 14 (the stalled reference): F2 infeasible from 14.4 s, B5 falls back ("infeasible") 2 s later;
  // with b5_max_infeasible_s -1 from 14.0 s B5 keeps following F2's plans
  auto run = [](const mmc::ForkConfig & fork) {
    auto c = ideal();
    c.mode = mmp::PlantMode::Realistic;
    c.fork = fork;
    mmc::Session s(c);
    s.reset("e3_gap_turn_4", "", std::nullopt, 14);
    std::vector<double> fallbacks;
    while (s.step_cycle() && s.time() < 20.0) {
    }
    for (const auto & f : s.take_frames()) {
      mmp::FrameType type{};
      json h;
      std::vector<std::vector<float>> b;
      size_t used = 0;
      if (mmp::decode(f, type, h, b, used) && type == mmp::FrameType::Event && h.value("event", "") == "fallback" &&
          h.value("reason", "") == "infeasible")
        fallbacks.push_back(h.at("t").get<double>());
    }
    return fallbacks;
  };
  const auto plain = run({}), forked = run(fork_at(14.0, {{"b5_max_infeasible_s", "-1"}}));
  ASSERT_FALSE(plain.empty()) << "the reference falls back";
  EXPECT_GT(plain.front(), 14.0);
  for (const double t : forked) EXPECT_LT(t, 14.0) << "no infeasible fallback after the fork";
}
