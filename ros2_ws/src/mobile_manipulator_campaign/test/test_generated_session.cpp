// Spec 2 Task 7: the session's generated-route mode. The lattice planner supplies each accepted destination's route,
// one persistent executor keeps the plant clock running while idle, planning or after a goal ended, and the goal
// manager owns every goal's outcome.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "mobile_manipulator_campaign/session.hpp"
#include "mobile_manipulator_plant/frames.hpp"

namespace mmc = mobile_manipulator_campaign;
namespace mmp = mobile_manipulator_plant;
using json = nlohmann::json;

namespace
{
constexpr double kTickS = 0.02;

mmc::SessionConfig generated(bool auto_goal = false)
{
  mmc::SessionConfig c;
  c.queue_frames = true;
  c.generated_routes = true;
  c.auto_goal = auto_goal;
  return c;
}

mobile_manipulator_routing::Pose2 ahead(const mmc::Session & s, double distance, double lateral = 0.0)
{
  const auto & x = s.start_state();
  const double c = std::cos(x(2)), sn = std::sin(x(2));
  return {x(0) + distance * c - lateral * sn, x(1) + distance * sn + lateral * c, x(2)};
}

mmc::GoalSubmission request(const mmc::Session & s, uint64_t id, const mobile_manipulator_routing::Pose2 & target,
                            double timeout_s = 60.0)
{
  return {s.goal_manager().epoch(), "test", id, target, timeout_s};
}

// advance until the predicate holds or the simulation time limit passes; returns whether it held
template <class Predicate>
bool advance_until(mmc::Session & s, double limit_s, Predicate done)
{
  while (s.time() < limit_s) {
    if (done()) return true;
    s.advance();
  }
  return done();
}

std::vector<std::string> event_names(const std::vector<json> & events)
{
  std::vector<std::string> names;
  for (const auto & e : events) names.push_back(e.at("event").get<std::string>());
  return names;
}

bool has(const std::vector<std::string> & names, const std::string & name)
{
  return std::find(names.begin(), names.end(), name) != names.end();
}

// the session's events so far, for a failure message
std::string journal(mmc::Session & s)
{
  json all = json::array();
  for (const auto & e : s.take_events()) all.push_back(e);
  return all.dump();
}

mmc::GoalState state_of(const mmc::Session & s)
{
  return s.goal_manager().current() ? s.goal_manager().current()->state : mmc::GoalState::Canceled;
}
}  // namespace

TEST(GeneratedSession, RefusesRecordedInputsAndTheReducedHierarchy)
{
  mmc::SessionConfig no_medium = generated();
  no_medium.medium = false;
  EXPECT_THROW(mmc::Session{no_medium}, std::invalid_argument);
  mmc::Session s(generated());
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  EXPECT_THROW(s.set_route(mobile_manipulator_mpc::Route({Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 0)}), {1, 0, 0}),
               std::logic_error);
  EXPECT_FALSE(s.active());  // no automatic goal
  EXPECT_EQ(s.summary().at("end_status"), "idle");
}

TEST(GeneratedSession, IdleAdvancesThePlantClockAndAScheduledGoalArrives)
{
  mmc::Session s(generated());
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const auto ack = s.schedule_goal(request(s, 1, ahead(s, 2.0)), 1.0);
  EXPECT_EQ(ack.action, mmc::GoalAction::Scheduled);
  for (int k = 0; k < 40; ++k) EXPECT_TRUE(s.advance());
  EXPECT_NEAR(s.time(), 40 * kTickS, 1e-6);  // physics ran while idle
  EXPECT_FALSE(s.goal_manager().current().has_value());
  ASSERT_TRUE(advance_until(s, 60.0, [&] { return state_of(s) == mmc::GoalState::Reached; }))
    << s.summary().dump();
  const auto & goal = *s.goal_manager().current();
  EXPECT_NEAR(goal.accepted_at, 1.0, kTickS + 1e-9);  // applied at the first boundary at or after 1 s
  EXPECT_GT(goal.route_id, 0u);
  const auto names = event_names(s.take_events());
  for (const char * name : {"goal_scheduled", "goal_accepted", "route_accepted", "goal_reached"})
    EXPECT_TRUE(has(names, name)) << name;
  const json summary = s.summary();
  EXPECT_TRUE(summary.at("reached").get<bool>()) << summary.dump();
  EXPECT_TRUE(summary.at("generated_routes").get<bool>());
  ASSERT_EQ(summary.at("goals").size(), 1u);
  EXPECT_EQ(summary.at("goals")[0].at("end_status"), "goal_reached");
  // the clock keeps running after the goal; frames keep coming, marked as holding
  const double reached_at = s.time();
  for (int k = 0; k < 25; ++k) s.advance();
  EXPECT_NEAR(s.time(), reached_at + 25 * kTickS, 1e-6);
  int holding = 0;
  for (const auto & f : s.take_frames()) {
    mmp::FrameType type{};
    json h;
    std::vector<std::vector<float>> b;
    size_t used = 0;
    if (mmp::decode(f, type, h, b, used) && type == mmp::FrameType::State && h.value("execution", "") == "holding")
      ++holding;
  }
  EXPECT_GE(holding, 25);
}

TEST(GeneratedSession, AReplacementWhileMovingSupersedesTheOldGoal)
{
  mmc::Session s(generated());
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const auto a = ahead(s, 3.0);
  const auto b = ahead(s, 1.5, 0.5);
  ASSERT_EQ(s.submit_goal(request(s, 1, a)).action, mmc::GoalAction::Accepted);
  ASSERT_TRUE(advance_until(s, 10.0, [&] { return state_of(s) == mmc::GoalState::Tracking; })) << journal(s);
  const uint64_t first = s.goal_manager().current()->search_token.goal;
  for (int k = 0; k < 75; ++k) s.advance();  // moving toward A
  ASSERT_EQ(state_of(s), mmc::GoalState::Tracking);
  ASSERT_EQ(s.submit_goal(request(s, 2, b)).action, mmc::GoalAction::Accepted);
  EXPECT_NE(s.goal_manager().current()->search_token.goal, first);
  ASSERT_TRUE(advance_until(s, 80.0, [&] { return state_of(s) == mmc::GoalState::Reached; })) << s.summary().dump();
  const auto & x = s.plant().state();
  EXPECT_LT(std::hypot(x(0) - b.x, x(1) - b.y), 0.15 + 1e-6);
  const auto names = event_names(s.take_events());
  EXPECT_TRUE(has(names, "goal_superseded"));
  const json summary = s.summary();
  ASSERT_EQ(summary.at("goals").size(), 2u);
  EXPECT_EQ(summary.at("goals")[0].at("end_status"), "superseded");
  EXPECT_EQ(summary.at("goals")[1].at("end_status"), "goal_reached");
  EXPECT_TRUE(summary.at("reached").get<bool>());  // the current destination decides navigation completion
}

TEST(GeneratedSession, AGoalAfterCompletionAndIdleGetsItsOwnInterval)
{
  mmc::Session s(generated());
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  ASSERT_EQ(s.submit_goal(request(s, 1, ahead(s, 1.5), 20.0)).action, mmc::GoalAction::Accepted);
  ASSERT_TRUE(advance_until(s, 20.0, [&] { return state_of(s) == mmc::GoalState::Reached; }));
  // idle past the first goal's deadline: the executor holds, nothing times out
  const double first_deadline = s.goal_manager().current()->deadline;
  ASSERT_TRUE(advance_until(s, first_deadline + 5.0, [] { return false; }) || s.time() >= first_deadline + 5.0);
  EXPECT_EQ(state_of(s), mmc::GoalState::Reached);
  // a second destination ahead (a goal behind the robot needs a turn the Euclidean heuristic searches slowly: the
  // implementation record's open item)
  ASSERT_EQ(s.submit_goal(request(s, 2, ahead(s, 3.0, 0.4), 30.0)).action, mmc::GoalAction::Accepted);
  const double accepted = s.goal_manager().current()->accepted_at;
  EXPECT_GE(accepted, first_deadline + 5.0 - 1e-9);
  ASSERT_TRUE(advance_until(s, accepted + 30.0, [&] {
    const auto st = state_of(s);
    return st != mmc::GoalState::Planning && st != mmc::GoalState::Tracking;
  }));
  EXPECT_EQ(state_of(s), mmc::GoalState::Reached) << journal(s);
  const json summary = s.summary();
  ASSERT_EQ(summary.at("goals").size(), 2u);
  EXPECT_FALSE(summary.at("goals")[1].at("timed_out").get<bool>());
  EXPECT_GT(summary.at("goals")[1].at("b5_cycles").get<int>(), 0);  // its own cycles, not the first goal's
}

TEST(GeneratedSession, AnInvalidReplacementLeavesTheCurrentGoalIntact)
{
  mmc::Session s(generated());
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  ASSERT_EQ(s.submit_goal(request(s, 1, ahead(s, 2.0))).action, mmc::GoalAction::Accepted);
  ASSERT_TRUE(advance_until(s, 10.0, [&] { return state_of(s) == mmc::GoalState::Tracking; })) << journal(s);
  const uint64_t goal = s.goal_manager().current()->search_token.goal;
  const auto ack = s.submit_goal(request(s, 2, {1.0e4, 1.0e4, 0.0}));
  EXPECT_EQ(ack.action, mmc::GoalAction::Rejected);
  EXPECT_EQ(s.goal_manager().current()->search_token.goal, goal);
  EXPECT_EQ(state_of(s), mmc::GoalState::Tracking);
  ASSERT_TRUE(advance_until(s, 60.0, [&] { return state_of(s) == mmc::GoalState::Reached; })) << s.summary().dump();
  EXPECT_TRUE(has(event_names(s.take_events()), "goal_rejected"));
}

TEST(GeneratedSession, TheScenarioDestinationIsSubmittedAtReset)
{
  mmc::Session s(generated(true));
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  ASSERT_TRUE(s.goal_manager().current().has_value());
  EXPECT_EQ(s.goal_manager().current()->submission.client, "scenario");
  EXPECT_TRUE(s.active());
  // step_cycle keeps its recorded-mode meaning: false once no goal is active
  int ticks = 0;
  while (s.step_cycle() && ticks < 20) ++ticks;
  EXPECT_EQ(ticks, 20);
}

TEST(GeneratedSession, ProfileTimingMeasuresTheRoutePlannerSetupAndEverySearchTick)
{
  // Spec 2 Task 14: the search's wall time per tick and the planner's setup (inputs, costmap) in the timing block;
  // the search time also counts in wall_s, so the real-time factor includes it.
  auto c = generated(true);
  c.profile_timing = true;
  c.routing_budget.expansions = 64;  // several search ticks
  mmc::Session s(c);
  s.reset("v2_room_r1_nav", "", std::nullopt, 1);
  for (int k = 0; k < 100 && s.step_cycle(); ++k) {
  }
  const auto timing = s.summary().at("timing");
  EXPECT_GT(timing.at("routing_setup_ms").get<double>(), 0.0);
  const auto & search = timing.at("route_search_ms");
  EXPECT_GE(search.at("n").get<int>(), 2);
  EXPECT_GT(search.at("p50").get<double>(), 0.0);
  EXPECT_GE(search.at("max").get<double>(), search.at("p99").get<double>());
  // Task 14 review M1: the run's wall_s is execution plus search work; a goal's wall_s is execution work only, from
  // its adoption, so it never includes a search tick (the adoption tick's included) and stays within the execution
  const auto summary = s.summary();
  const double execution = summary.at("execution_wall_s").get<double>(), searching = summary.at("search_wall_s").get<double>();
  EXPECT_GT(searching, 0.0);
  EXPECT_NEAR(summary.at("wall_s").get<double>(), execution + searching, 1e-9);
  ASSERT_FALSE(summary.at("goals").empty());
  for (const auto & g : summary.at("goals")) EXPECT_LE(g.at("wall_s").get<double>(), execution + 1e-12);
}

TEST(GeneratedSession, TheRoutingFingerprintRecordsTheSearchBudgetAndValidationSettings)
{
  // Spec 2 Task 14: the per-tick budget sets the planning hold, so a run's routing fingerprint (summary and Hello)
  // records it with the validation's sampling step and the adoption connector; resume compares the whole fingerprint.
  auto c = generated(true);
  c.routing_budget.expansions = 1234;
  mmc::Session s(c);
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const auto search = s.summary().at("routing").at("search");
  EXPECT_EQ(search.at("expansions_per_tick").get<std::size_t>(), 1234u);
  EXPECT_EQ(search.at("cell_tests_per_tick").get<std::size_t>(), c.routing_budget.cell_tests);
  EXPECT_DOUBLE_EQ(search.at("max_point_step_m").get<double>(), c.routing_search.max_point_step_m);
  EXPECT_EQ(search.at("connector_radius_cells").get<int>(), c.routing_search.connector_radius_cells);
  EXPECT_EQ(search.at("max_path_poses").get<std::size_t>(), c.routing_search.max_path_poses);
  const auto frames = s.resync_frames();
  ASSERT_FALSE(frames.empty());
  mmp::FrameType type{};
  nlohmann::json hello;
  std::vector<std::vector<float>> blocks;
  size_t used = 0;
  ASSERT_TRUE(mmp::decode(frames.front(), type, hello, blocks, used));
  ASSERT_EQ(type, mmp::FrameType::Hello);
  EXPECT_EQ(hello.at("routing"), s.summary().at("routing"));
}

TEST(GeneratedSession, RealisticPlantHoldsBaseAndArmWhileAReplacementIsPlanned)
{
  // the realistic arm stream: a replacement while moving sends the executor's hold (status holding), so the base and
  // the arm come to rest during planning instead of finishing the old plan's stream; the replacement then executes
  mmc::SessionConfig c = generated();
  c.mode = mmp::PlantMode::Realistic;
  c.routing_budget.expansions = 1;  // a slow search on purpose: the hold lasts long enough to observe rest
  mmc::Session s(c);
  s.reset("v2_open_yard_nav", "", std::nullopt, 2);
  ASSERT_EQ(s.submit_goal(request(s, 1, ahead(s, 3.0))).action, mmc::GoalAction::Accepted);
  ASSERT_TRUE(advance_until(s, 10.0, [&] { return state_of(s) == mmc::GoalState::Tracking; })) << journal(s);
  for (int k = 0; k < 100; ++k) s.advance();
  ASSERT_GT(std::abs(s.plant().plant().true_state().v), 0.05);
  // a replacement behind the robot: it needs a turn-around, so with one Smac iteration per tick its search spans
  // many ticks of hold (a goal ahead or to the side is found within a few iterations by the analytic expansion)
  ASSERT_EQ(s.submit_goal(request(s, 2, ahead(s, -4.0))).action, mmc::GoalAction::Accepted);
  // after the braking time (about 0.7 s from 0.3 m/s here), every tick still planning is at rest
  double speed = 0.0, arm = 0.0;
  int held = 0;
  while (s.time() < 40.0) {
    s.advance();
    if (state_of(s) != mmc::GoalState::Planning) break;  // this tick adopted the route and tracked
    if (++held < 50) continue;
    speed = std::max(speed, std::abs(s.plant().plant().true_state().v));
    arm = std::max(arm, s.plant().plant().true_state().qd.cwiseAbs().maxCoeff());
  }
  ASSERT_GT(held, 80) << "the replacement's search was too short to observe the hold";
  EXPECT_LT(speed, 0.01);
  EXPECT_LT(arm, 0.01);
  ASSERT_TRUE(advance_until(s, 90.0, [&] { return state_of(s) == mmc::GoalState::Reached; })) << journal(s);
}

TEST(GeneratedSession, ResetDuringASearchStartsAnIdleRunInANewEpoch)
{
  mmc::SessionConfig c = generated();
  c.routing_budget.expansions = 1;  // the search spans ticks, so the reset lands inside it
  mmc::Session s(c);
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const auto epoch = s.goal_manager().epoch();
  ASSERT_EQ(s.submit_goal(request(s, 1, ahead(s, -4.0))).action, mmc::GoalAction::Accepted);
  s.advance();
  ASSERT_EQ(state_of(s), mmc::GoalState::Planning);
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  EXPECT_GT(s.goal_manager().epoch(), epoch);
  EXPECT_FALSE(s.goal_manager().current().has_value());
  EXPECT_FALSE(s.active());
  EXPECT_NEAR(s.time(), 0.0, 1e-12);
  // a request for the old epoch is stale; the new epoch's goal runs
  auto old = request(s, 2, ahead(s, 2.0));
  old.epoch = epoch;
  EXPECT_NE(s.submit_goal(old).action, mmc::GoalAction::Accepted);
  ASSERT_EQ(s.submit_goal(request(s, 3, ahead(s, 2.0))).action, mmc::GoalAction::Accepted);
  ASSERT_TRUE(advance_until(s, 60.0, [&] { return state_of(s) == mmc::GoalState::Reached; })) << journal(s);
}

TEST(GeneratedSession, ContactsDuringAPlanningHoldReachTheRunSummary)
{
  // Task 7 review I3: a reserved body placed over the robot while its goal is still planning (one validation cell test
  // per tick keeps it planning); the goal expires before adoption. The run summary's safety fields cover that hold.
  mmc::SessionConfig c = generated();
  c.routing_budget = {1, 1};
  mmc::Session s(c);
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  ASSERT_EQ(s.submit_goal(request(s, 1, ahead(s, 2.0), 0.2)).action, mmc::GoalAction::Accepted);
  const auto x = s.start_state();
  s.move_body("reserved/0", Eigen::Vector3d(x(0), x(1), 0.9), x(2));
  int observed = 0;
  while (s.time() < 0.22) {
    s.advance();
    observed = std::max(observed, s.plant().contacts());
  }
  ASSERT_GT(observed, 0) << "the fixture needs a contact during the hold";
  const auto summary = s.summary();
  EXPECT_GT(summary.at("plant_contacts").get<int>(), 0);
  EXPECT_FALSE(summary.at("plant_min_clearance_m").is_null());
  EXPECT_EQ(summary.at("safety_scope"), "every_tick");
}

TEST(GeneratedSession, AGoalBehindTheRobotTurnsInPlaceFirst)
{
  // F2's plan cannot start with a turn on the spot (B5 alone can): a route starting more than 0.785 rad from the
  // base's heading runs as a stationary turn leg, then the drive leg, for the same goal
  mmc::Session s(generated());
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  auto target = ahead(s, -3.0);
  target.yaw = std::remainder(target.yaw + M_PI, 2.0 * M_PI);  // facing back along the way it drives
  ASSERT_EQ(s.submit_goal(request(s, 1, target)).action, mmc::GoalAction::Accepted);
  std::vector<std::string> names;
  const bool reached = advance_until(s, 60.0, [&] {
    for (const auto & n : event_names(s.take_events())) names.push_back(n);
    return state_of(s) == mmc::GoalState::Reached;
  });
  EXPECT_TRUE(has(names, "drive_leg_started"));
  ASSERT_TRUE(reached) << json(names).dump();
}

TEST(GeneratedSession, TheTurnToDriveHandoffKeepsTheCommandAndTheGoalsStatistics)
{
  // Task 9 review I1, I2: at drive_leg_started the yaw command changes by at most the envelope's deceleration per tick
  // (it dropped from 0.357 rad/s to 0), and the goal's B5 and medium cycle counts keep growing (they were reset)
  mmc::Session s(generated());
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  auto target = ahead(s, -3.0);
  target.yaw = std::remainder(target.yaw + M_PI, 2.0 * M_PI);
  ASSERT_EQ(s.submit_goal(request(s, 1, target)).action, mmc::GoalAction::Accepted);
  std::vector<double> omega;  // the command's yaw rate each physics tick (ideal mode: State block 2 is the command)
  int handoff_at = -1, cycles_before = -1, cycles_after = -1;
  for (int k = 0; k < 3000 && cycles_after < 0; ++k) {
    s.advance();
    for (const auto & f : s.take_frames()) {
      mmp::FrameType type{};
      json h;
      std::vector<std::vector<float>> b;
      size_t used = 0;
      ASSERT_TRUE(mmp::decode(f, type, h, b, used));
      if (type == mmp::FrameType::State) omega.push_back(b.at(2).at(1));
    }
    const auto names = event_names(s.take_events());
    const auto row = [&] { return s.summary().at("goals").back().at("b5_cycles").get<int>(); };
    if (handoff_at >= 0 && static_cast<int>(omega.size()) >= handoff_at + 3) cycles_after = row();
    if (handoff_at < 0 && has(names, "drive_leg_started")) {
      handoff_at = static_cast<int>(omega.size());
      cycles_before = row();
    }
  }
  ASSERT_GE(handoff_at, 1) << "no turn-first handoff";
  const double max_step = 1.0 * kTickS + 1e-6;  // yaw_decel_radps2 1.0
  for (int i = std::max(1, handoff_at - 1); i < std::min<int>(omega.size(), handoff_at + 3); ++i)
    EXPECT_LE(std::abs(omega[i] - omega[i - 1]), max_step) << "tick " << i << " of the handoff at " << handoff_at;
  EXPECT_GT(cycles_before, 0);
  EXPECT_GE(cycles_after, cycles_before);
}

// Spec 4A Task 1: a client's cancel ends the tracked goal; the robot holds and the clock keeps running
TEST(GeneratedSession, CancelHoldsTheRobot)
{
  mmc::Session s(generated());
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  ASSERT_EQ(s.submit_goal(request(s, 1, ahead(s, 3.0))).action, mmc::GoalAction::Accepted);
  ASSERT_TRUE(advance_until(s, 10.0, [&] { return state_of(s) == mmc::GoalState::Tracking; })) << journal(s);
  for (int k = 0; k < 25; ++k) s.advance();  // moving
  s.take_frames();
  const auto ack = s.cancel_goal();
  EXPECT_EQ(ack.action, mmc::GoalAction::Applied);
  ASSERT_TRUE(s.goal_manager().current());
  EXPECT_EQ(s.goal_manager().current()->state, mmc::GoalState::Canceled);
  EXPECT_EQ(s.cancel_goal().action, mmc::GoalAction::Rejected);  // nothing left to cancel
  const double canceled_at = s.time();
  for (int k = 0; k < 10; ++k) s.advance();
  EXPECT_NEAR(s.time(), canceled_at + 10 * kTickS, 1e-6);
  std::string last_goal_event;
  int states = 0, holding = 0;
  for (const auto & f : s.take_frames()) {
    mmp::FrameType type{};
    json h;
    std::vector<std::vector<float>> b;
    size_t used = 0;
    if (!mmp::decode(f, type, h, b, used)) continue;
    if (type == mmp::FrameType::Goal) last_goal_event = h.at("event").get<std::string>();
    if (type == mmp::FrameType::State) {
      ++states;
      holding += h.value("execution", "") == "holding";
    }
  }
  EXPECT_EQ(last_goal_event, "goal_canceled");
  EXPECT_GE(states, 10);
  EXPECT_EQ(holding, states);
  const json summary = s.summary();
  ASSERT_EQ(summary.at("goals").size(), 1u);
  EXPECT_EQ(summary.at("goals")[0].at("end_status"), "canceled");
}

// Spec 4B-2 Task 1 review I1: a Prediction frame only for a cycle B5 solved. Idle cycles (before the first goal,
// after one) hold without solving: no frame, so the viewer's last prediction goes stale instead of showing B5's
// initial plan at the map origin, or a frozen plan as fresh.
TEST(GeneratedSession, NoPredictionWithoutABSolve)
{
  auto c = generated();
  c.prediction = true;
  mmc::Session s(c);
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  struct Seen { int predictions = 0, near = 0; };
  auto scan = [&] {
    Seen seen;
    double x = 0.0, y = 0.0;
    for (const auto & f : s.take_frames()) {
      mmp::FrameType type{};
      json h;
      std::vector<std::vector<float>> b;
      size_t used = 0;
      if (!mmp::decode(f, type, h, b, used)) continue;
      if (type == mmp::FrameType::State) {
        x = b[0][0];
        y = b[0][1];
      }
      if (type != mmp::FrameType::Prediction) continue;
      ++seen.predictions;
      if (std::hypot(b[0][0] - x, b[0][1] - y) < 0.5) ++seen.near;  // stage 0 at the robot, not at the origin
    }
    return seen;
  };
  for (int k = 0; k < 40; ++k) s.advance();
  EXPECT_EQ(scan().predictions, 0) << "idle before the first goal";
  const auto ack = s.submit_goal(request(s, 1, ahead(s, 2.0)));
  ASSERT_TRUE(ack.action == mmc::GoalAction::Accepted || ack.action == mmc::GoalAction::Applied) << to_string(ack.action);
  ASSERT_TRUE(advance_until(s, 60.0, [&] { return state_of(s) == mmc::GoalState::Reached; }));
  const auto tracking = scan();
  EXPECT_GT(tracking.predictions, 0);
  EXPECT_EQ(tracking.near, tracking.predictions);
  for (int k = 0; k < 40; ++k) s.advance();
  EXPECT_EQ(scan().predictions, 0) << "idle after the goal";
}
