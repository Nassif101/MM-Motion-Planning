// The plant session (MuJoCo plant plan Task 8): resets, goals, events, frames, the run log and the summary.
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>

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

TEST(Session, HelloRecordsTheTrackingModel)
{
  // E2 Task 7: the hello and the summary say which tracking model B5 predicted with (its path and hash; "" the
  // identity) and whether the online correction ran. --tracking-model overrides the B5 config's; "none" the identity.
  mmc::Session shipped(ideal());
  shipped.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const json base = hello_of(shipped).at("config");
  EXPECT_EQ(base.at("tracking_model"), "");
  EXPECT_EQ(base.at("tracking_model_sha256"), "");
  EXPECT_EQ(base.at("tracking_adaptation"), false);
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
  // the cycle planned with: the identity's [1, 1, 0] by default, the file's numbers with the model and no correction
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
  const auto identity = tracks("");
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
