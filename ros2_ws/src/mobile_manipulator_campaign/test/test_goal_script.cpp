// Spec 2 Task 8: the headless goal script and its driver on generated-route sessions.
#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "mobile_manipulator_campaign/goal_script.hpp"
#include "mobile_manipulator_campaign/session.hpp"

namespace mmc = mobile_manipulator_campaign;
using json = nlohmann::json;

namespace
{
mmc::SessionConfig generated()
{
  mmc::SessionConfig c;
  c.queue_frames = true;  // events are queued for take_events
  c.generated_routes = true;
  c.auto_goal = false;
  return c;
}

json ahead(const mmc::Session & s, double distance, double lateral = 0.0)
{
  const auto & x = s.start_state();
  const double c = std::cos(x(2)), sn = std::sin(x(2));
  return {x(0) + distance * c - lateral * sn, x(1) + distance * sn + lateral * c, x(2)};
}

std::vector<std::string> events_named(mmc::Session & s, const std::string & name)
{
  std::vector<std::string> out;
  for (const auto & e : s.take_events())
    if (e.at("event") == name) out.push_back(e.dump());
  return out;
}
}  // namespace

TEST(GoalScript, ParsesAValidScriptAndRejectsMalformedOnes)
{
  const auto ok = mmc::parse_goal_script(
    R"({"goals": [{"id": "a", "target": [1, 2, 0.5], "at_s": 5}, {"id": "b", "target": [3, 4, 0], "after": "previous",
        "timeout_s": 30}], "time_limit_s": 100, "end": "idle"})");
  ASSERT_EQ(ok.goals.size(), 2u);
  EXPECT_EQ(*ok.goals[0].at_s, 5.0);
  EXPECT_FALSE(ok.goals[1].at_s.has_value());
  EXPECT_EQ(ok.goals[1].timeout_s, 30.0);
  EXPECT_EQ(*ok.time_limit_s, 100.0);
  EXPECT_EQ(ok.sha256.size(), 64u);
  for (const char * bad : {
         "not json",
         R"({"goals": [], "extra": 1})",
         R"({"goals": [{"id": "a", "target": [1, 2, 0]}]})",                                         // no trigger
         R"({"goals": [{"id": "a", "target": [1, 2, 0], "at_s": 1, "after": "previous"}]})",       // both
         R"({"goals": [{"id": "a", "target": [1, 2], "at_s": 1}]})",                               // short target
         R"({"goals": [{"id": "a", "target": [1, 2, 0], "at_s": -1}]})",                           // negative time
         R"({"goals": [{"id": "a", "target": [1, 2, 0], "at_s": 1}, {"id": "a", "target": [1, 2, 0], "at_s": 2}]})",
         R"({"goals": [{"id": "a", "target": [1, 2, 0], "after": "first"}]})",
         R"({"goals": [], "end": "time_limit"})",                                                    // no limit
         R"({"goals": [{"id": "a", "target": [1, 2, 0], "at_s": 1, "colour": 3}]})"}) {
    EXPECT_THROW(mmc::parse_goal_script(bad), std::invalid_argument) << bad;
  }
  std::string many = R"({"goals": [)";
  for (int i = 0; i <= 64; ++i) many += (i ? "," : "") + std::string(R"({"id": "g)") + std::to_string(i) + R"(", "target": [0, 0, 0], "at_s": 1})";
  EXPECT_THROW(mmc::parse_goal_script(many + "]}"), std::invalid_argument);  // over a client's receipt bound
}

TEST(GoalScript, AScheduledFirstGoalArrivesWhileIdleAndIsReached)
{
  mmc::Session s(generated());
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const auto script = mmc::parse_goal_script(json({{"goals", {{{"id", "a"}, {"target", ahead(s, 2.0)}, {"at_s", 5.0}}}}}).dump());
  const auto run = mmc::run_goal_script(s, script);
  ASSERT_EQ(run.submissions.size(), 1u);
  EXPECT_EQ(run.submissions[0].action, mmc::GoalAction::Scheduled);
  EXPECT_FALSE(run.time_limit_reached);
  const auto summary = s.summary();
  EXPECT_TRUE(summary.at("reached").get<bool>()) << summary.dump(1);
  EXPECT_GT(run.end_time_s, 5.0);
  EXPECT_FALSE(s.active());
  EXPECT_TRUE(run.success);
  EXPECT_EQ(run.outcomes.at(0).outcome, "reached");
}

TEST(GoalScript, SuccessiveGoalsFollowTheirPredecessors)
{
  mmc::Session s(generated());
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const auto script = mmc::parse_goal_script(json({{"goals", {
    {{"id", "a"}, {"target", ahead(s, 2.0)}, {"after", "previous"}},
    {{"id", "b"}, {"target", ahead(s, 3.5, 0.5)}, {"after", "previous"}}}}}).dump());
  const auto run = mmc::run_goal_script(s, script);
  ASSERT_EQ(run.submissions.size(), 2u);
  EXPECT_EQ(run.submissions[0].action, mmc::GoalAction::Accepted);
  EXPECT_EQ(run.submissions[1].action, mmc::GoalAction::Accepted);
  EXPECT_GT(run.submissions[1].at_s, run.submissions[0].at_s + 1.0);  // after the first ended
  EXPECT_EQ(events_named(s, "goal_reached").size(), 2u);
  EXPECT_TRUE(s.summary().at("reached").get<bool>());
  EXPECT_TRUE(run.success);
}

TEST(GoalScript, AReplacementWhileMovingAndAnUnreachableGoalEndTheRunByItsPolicy)
{
  mmc::Session s(generated());
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const auto script = mmc::parse_goal_script(json({{"goals", {
    {{"id", "a"}, {"target", ahead(s, 4.0)}, {"at_s", 0.0}},
    {{"id", "b"}, {"target", ahead(s, 2.5, 0.5)}, {"at_s", 3.0}},          // replaces a while it moves
    {{"id", "c"}, {"target", {500.0, 500.0, 0.0}}, {"after", "previous"}}}},  // outside the map
    {"time_limit_s", 120.0}}).dump());
  const auto run = mmc::run_goal_script(s, script);
  ASSERT_EQ(run.submissions.size(), 3u);
  EXPECT_FALSE(run.time_limit_reached);
  EXPECT_EQ(events_named(s, "goal_superseded").size(), 1u);
  EXPECT_FALSE(s.active());
  const auto summary = s.summary();
  // c is refused at submission (outside the map), so it never becomes the current destination: the session's
  // completion is the current destination's (b, reached); the refusal is in the script's submissions
  EXPECT_EQ(run.submissions[2].action, mmc::GoalAction::Rejected);
  EXPECT_TRUE(summary.at("reached").get<bool>());
  // Task 8 review I4: the script's own outcome is not inherited from the current destination
  EXPECT_EQ(run.outcomes.at(0).outcome, "superseded");
  EXPECT_EQ(run.outcomes.at(1).outcome, "reached");
  EXPECT_EQ(run.outcomes.at(2).outcome, "refused");
  EXPECT_FALSE(run.success);
}

TEST(GoalScript, ATimeLimitEndsAnIdleRun)
{
  mmc::Session s(generated());
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const auto run = mmc::run_goal_script(s, mmc::parse_goal_script(R"({"goals": [], "time_limit_s": 3, "end": "time_limit"})"));
  EXPECT_TRUE(run.time_limit_reached);
  EXPECT_NEAR(run.end_time_s, 3.0, 0.021);
  mmc::Session recorded{mmc::SessionConfig{}};
  recorded.reset("v2_open_yard_nav", "", std::nullopt, 1);
  EXPECT_THROW(mmc::run_goal_script(recorded, mmc::GoalScript{}), std::invalid_argument);
}

TEST(GoalScript, TheMaximumNumberOfTimedGoalsIsAllScheduled)
{
  // Task 8 review I2: the advertised maximum is accepted whole (each Scheduled), not cut off by backpressure
  mmc::Session s(generated());
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  json goals = json::array();
  for (std::size_t i = 0; i < mmc::GoalScript::kMaxScheduled; ++i)
    goals.push_back({{"id", "g" + std::to_string(i)}, {"target", ahead(s, 0.0)}, {"at_s", 5.0}});
  const auto run = mmc::run_goal_script(s, mmc::parse_goal_script(json({{"goals", goals}, {"time_limit_s", 0.02},
                                                                          {"end", "time_limit"}}).dump()));
  ASSERT_EQ(run.submissions.size(), mmc::GoalScript::kMaxScheduled);
  for (const auto & sub : run.submissions) EXPECT_EQ(sub.action, mmc::GoalAction::Scheduled) << sub.id << ": " << sub.reason;
}

TEST(GoalScript, AGoalStillScheduledAtTheTimeLimitIsPendingAndTheScriptFails)
{
  // Task 8 review I4: the first goal is reached, the second is scheduled after the limit
  mmc::Session s(generated());
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  const auto run = mmc::run_goal_script(s, mmc::parse_goal_script(json({{"goals", {
    {{"id", "a"}, {"target", ahead(s, 1.5)}, {"at_s", 0.0}},
    {{"id", "b"}, {"target", ahead(s, 3.0)}, {"at_s", 200.0}}}}, {"time_limit_s", 30.0}}).dump()));
  EXPECT_TRUE(run.time_limit_reached);
  ASSERT_EQ(run.outcomes.size(), 2u);
  EXPECT_EQ(run.outcomes[0].outcome, "reached");
  EXPECT_EQ(run.outcomes[1].outcome, "pending");
  EXPECT_FALSE(run.success);
  EXPECT_TRUE(s.summary().at("reached").get<bool>());  // the session's meaning is unchanged
}
