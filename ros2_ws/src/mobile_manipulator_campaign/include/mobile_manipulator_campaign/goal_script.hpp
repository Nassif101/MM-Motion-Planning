// Spec 2 Task 8: the headless goal source of generated-route runs. A bounded JSON script of destinations, submitted
// through the session's goal manager at control boundaries, and the driver that runs a session by it.
//
//   {"goals": [{"id": "a", "target": [x, y, yaw], "timeout_s": 120, "at_s": 5.0},
//              {"id": "b", "target": [x, y, yaw], "after": "previous"}],
//    "time_limit_s": 600, "end": "idle"}
//
// Each goal has exactly one trigger: `at_s` (simulation time; the goal manager's schedule, so a goal due while another
// runs replaces it) or `after: "previous"` (submitted at the first control boundary once the previous script goal
// has been submitted and no goal is active or scheduled). `end`: "idle" (default) stops once every goal has been
// submitted and none is active or scheduled; "time_limit" runs to `time_limit_s`. A run always stops at
// `time_limit_s` when it is given.
#pragma once
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "mobile_manipulator_campaign/goal_manager.hpp"
#include "mobile_manipulator_campaign/session.hpp"

namespace mobile_manipulator_campaign
{
struct ScriptGoal
{
  std::string id;
  mobile_manipulator_routing::Pose2 target;
  double timeout_s = 120.0;
  std::optional<double> at_s;  // otherwise after the previous goal
};

struct GoalScript
{
  static constexpr std::size_t kMaxGoals = 1000;
  static constexpr std::size_t kMaxScheduled = 128;  // the goal manager's schedule bound
  std::vector<ScriptGoal> goals;
  std::optional<double> time_limit_s;
  bool end_when_idle = true;
  std::string sha256;  // of the script's bytes ("" when built in code)
};

GoalScript parse_goal_script(const std::string & text);  // std::invalid_argument on a malformed script
GoalScript load_goal_script(const std::string & path);   // at most 1 MiB

struct ScriptRun
{
  struct Submission
  {
    std::string id;
    double at_s;  // the simulation time it was handed to the goal manager
    GoalAction action;
    std::string reason;
  };
  std::vector<Submission> submissions;
  bool time_limit_reached = false;
  double end_time_s = 0.0;
};

// Runs a generated-route session (after reset) by the script, one Session::advance() per tick. The outcome of each
// goal is the session's; the run ends by the script's policy only.
ScriptRun run_goal_script(Session & session, const GoalScript & script);
nlohmann::json to_json(const ScriptRun & run);
}  // namespace mobile_manipulator_campaign
