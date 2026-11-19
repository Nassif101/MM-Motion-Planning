// Spec 2 Task 9: the schema-3 Goal and Route records of generated-route sessions (stream-protocol.md, "Goal" and
// "Route"). Pure functions of their inputs, so the C++ and Python codecs can be checked against committed fixtures.
#pragma once
#include <nlohmann/json.hpp>

#include <string>
#include <utility>
#include <vector>

#include "mobile_manipulator_campaign/goal_manager.hpp"
#include "mobile_manipulator_routing/types.hpp"

namespace mobile_manipulator_campaign
{
// The session's name of a goal manager event (also the name of its Event frame).
const char * goal_event_name(GoalEventKind kind);
const char * goal_state_name(GoalState state);

// A Goal frame's header: the event, its time `t`, the epoch and IDs, the requested target, timing and revisions.
nlohmann::json goal_record(const GoalEvent & event, double t);

struct RouteRecord
{
  double t = 0.0;
  std::uint64_t epoch = 0, goal_id = 0, request_id = 0, route_id = 0;
  mobile_manipulator_routing::Pose2 requested_goal, resolved_goal;
  std::string planner, path_kind;  // path_kind: "translating" or "stationary"
  bool smoothed = false, turn_first = false;
  double length_m = 0.0;
  std::size_t expansions = 0, cell_tests = 0;
  std::vector<mobile_manipulator_routing::Pose2> path;  // the adopted path: connector from the plant, then the plan
  std::vector<std::pair<double, double>> route;         // the hierarchy route's points
};

// A Route frame: its header, and blocks [path x, y, yaw ...] and [route x, y ...] (float32).
std::pair<nlohmann::json, std::vector<std::vector<float>>> route_record(const RouteRecord & route);
}  // namespace mobile_manipulator_campaign
