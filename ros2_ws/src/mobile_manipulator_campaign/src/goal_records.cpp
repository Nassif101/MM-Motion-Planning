#include "mobile_manipulator_campaign/goal_records.hpp"

namespace mobile_manipulator_campaign
{
using json = nlohmann::json;

const char * goal_event_name(GoalEventKind kind)
{
  switch (kind) {
    case GoalEventKind::Queued: return "goal_scheduled";
    case GoalEventKind::Accepted: return "goal_accepted";
    case GoalEventKind::Rejected: return "goal_rejected";
    case GoalEventKind::Superseded: return "goal_superseded";
    case GoalEventKind::ReplanRequested: return "replan_requested";
    case GoalEventKind::RouteAccepted: return "route_accepted";
    case GoalEventKind::Reached: return "goal_reached";
    case GoalEventKind::Failed: return "failed";
    case GoalEventKind::TimedOut: return "timeout";
    case GoalEventKind::Canceled: return "goal_canceled";
    case GoalEventKind::Reset: return "session_reset";
    case GoalEventKind::EnvironmentChanged: return "routing_environment_changed";
  }
  return "goal_event";
}

const char * goal_state_name(GoalState state)
{
  switch (state) {
    case GoalState::Planning: return "planning";
    case GoalState::Tracking: return "tracking";
    case GoalState::Reached: return "reached";
    case GoalState::Failed: return "failed";
    case GoalState::TimedOut: return "timed_out";
    case GoalState::Superseded: return "superseded";
    case GoalState::Canceled: return "canceled";
  }
  return "unknown";
}

json goal_record(const GoalEvent & e, double t)
{
  json r = {{"t", t}, {"event", goal_event_name(e.kind)}, {"sequence", e.sequence}, {"input_sequence", e.input_sequence},
            {"epoch", e.epoch}, {"requested_epoch", e.requested_epoch}, {"applied_at", e.applied_at},
            {"client", e.client}, {"client_request", e.client_request}, {"reason", e.reason},
            {"scheduled_for", e.scheduled_for ? json(*e.scheduled_for) : json(nullptr)}};
  if (e.goal) {
    const auto & g = *e.goal;
    r["goal"] = {{"goal_id", g.search_token.goal}, {"request_id", g.search_token.request}, {"route_id", g.route_id},
                 {"state", goal_state_name(g.state)},
                 {"target", {g.submission.target.x, g.submission.target.y, g.submission.target.yaw}},
                 {"timeout_s", g.submission.timeout_s}, {"accepted_at", g.accepted_at}, {"deadline", g.deadline},
                 {"revisions", {{"map", g.search_token.map}, {"footprint", g.search_token.footprint},
                                {"configuration", g.search_token.configuration}}}};
  } else {
    r["goal"] = nullptr;
  }
  return r;
}

std::pair<json, std::vector<std::vector<float>>> route_record(const RouteRecord & route)
{
  const auto pose = [](const mobile_manipulator_routing::Pose2 & p) { return json::array({p.x, p.y, p.yaw}); };
  json header = {{"t", route.t}, {"epoch", route.epoch}, {"goal_id", route.goal_id}, {"request_id", route.request_id},
                 {"route_id", route.route_id}, {"planner", route.planner}, {"path_kind", route.path_kind},
                 {"smoothed", route.smoothed}, {"turn_first", route.turn_first}, {"length_m", route.length_m},
                 {"requested_goal", pose(route.requested_goal)}, {"resolved_goal", pose(route.resolved_goal)},
                 {"expansions", route.expansions}, {"cell_tests", route.cell_tests},
                 {"path_poses", route.path.size()}, {"route_points", route.route.size()},
                 {"layout", "block 0: path x, y, yaw per pose; block 1: route x, y per point"}};
  std::vector<std::vector<float>> blocks(2);
  for (const auto & p : route.path) {
    blocks[0].push_back(static_cast<float>(p.x));
    blocks[0].push_back(static_cast<float>(p.y));
    blocks[0].push_back(static_cast<float>(p.yaw));
  }
  for (const auto & [x, y] : route.route) {
    blocks[1].push_back(static_cast<float>(x));
    blocks[1].push_back(static_cast<float>(y));
  }
  return {header, blocks};
}
}  // namespace mobile_manipulator_campaign
