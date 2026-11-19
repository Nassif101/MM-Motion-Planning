#include "mobile_manipulator_campaign/goal_script.hpp"

#include <openssl/evp.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <set>
#include <sstream>
#include <stdexcept>

namespace mobile_manipulator_campaign
{
using json = nlohmann::json;

namespace
{
constexpr double kMaxTimeS = 86400.0;  // one simulated day bounds a script with no time limit

std::string digest(const std::string & bytes)
{
  unsigned char out[EVP_MAX_MD_SIZE];
  unsigned int n = 0;
  EVP_Digest(bytes.data(), bytes.size(), out, &n, EVP_sha256(), nullptr);
  std::ostringstream hex;
  for (unsigned int i = 0; i < n; ++i) hex << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(out[i]);
  return hex.str();
}

double finite_number(const json & value, const std::string & what, double lo, double hi)
{
  if (!value.is_number() || value.is_boolean()) throw std::invalid_argument(what + " must be a number");
  const double v = value.get<double>();
  if (!std::isfinite(v) || v < lo || v > hi) throw std::invalid_argument(what + " is outside its range");
  return v;
}

void only_keys(const json & object, const std::set<std::string> & allowed, const std::string & what)
{
  if (!object.is_object()) throw std::invalid_argument(what + " must be an object");
  for (auto it = object.begin(); it != object.end(); ++it)
    if (!allowed.count(it.key())) throw std::invalid_argument("unknown " + what + " field: " + it.key());
}
}  // namespace

GoalScript parse_goal_script(const std::string & text)
{
  json root;
  try {
    root = json::parse(text);
  } catch (const json::exception & e) {
    throw std::invalid_argument(std::string("goal script is not JSON: ") + e.what());
  }
  only_keys(root, {"goals", "time_limit_s", "end"}, "goal script");
  GoalScript script;
  script.sha256 = digest(text);
  if (root.contains("time_limit_s")) script.time_limit_s = finite_number(root.at("time_limit_s"), "time_limit_s", 0.0, kMaxTimeS);
  if (root.contains("end")) {
    const auto & end = root.at("end");
    if (!end.is_string() || (end != "idle" && end != "time_limit")) throw std::invalid_argument("end must be idle or time_limit");
    script.end_when_idle = end == "idle";
    if (!script.end_when_idle && !script.time_limit_s) throw std::invalid_argument("end time_limit needs time_limit_s");
  }
  if (!root.contains("goals") || !root.at("goals").is_array()) throw std::invalid_argument("goal script needs a goals list");
  const auto & goals = root.at("goals");
  if (goals.size() > GoalScript::kMaxGoals) throw std::invalid_argument("goal script has too many goals");
  std::set<std::string> ids;
  std::size_t scheduled = 0;
  for (std::size_t i = 0; i < goals.size(); ++i) {
    const auto & g = goals[i];
    const std::string where = "goal " + std::to_string(i);
    only_keys(g, {"id", "target", "timeout_s", "at_s", "after"}, where);
    ScriptGoal goal;
    if (!g.contains("id") || !g.at("id").is_string()) throw std::invalid_argument(where + " needs a string id");
    goal.id = g.at("id").get<std::string>();
    if (goal.id.empty() || goal.id.size() > 64 || !ids.insert(goal.id).second)
      throw std::invalid_argument(where + ": ids are unique, 1 to 64 characters");
    if (!g.contains("target") || !g.at("target").is_array() || g.at("target").size() != 3)
      throw std::invalid_argument(where + " needs target [x, y, yaw]");
    goal.target = {finite_number(g.at("target")[0], where + " x", -1e4, 1e4),
                   finite_number(g.at("target")[1], where + " y", -1e4, 1e4),
                   finite_number(g.at("target")[2], where + " yaw", -1e4, 1e4)};
    if (g.contains("timeout_s")) goal.timeout_s = finite_number(g.at("timeout_s"), where + " timeout_s", 1e-3, kMaxTimeS);
    const bool at = g.contains("at_s"), after = g.contains("after");
    if (at == after) throw std::invalid_argument(where + " needs exactly one of at_s and after");
    if (at) {
      goal.at_s = finite_number(g.at("at_s"), where + " at_s", 0.0, kMaxTimeS);
      if (++scheduled > GoalScript::kMaxScheduled) throw std::invalid_argument("goal script schedules too many goals");
    } else if (!g.at("after").is_string() || g.at("after") != "previous") {
      throw std::invalid_argument(where + ": after must be \"previous\"");
    }
    script.goals.push_back(std::move(goal));
  }
  return script;
}

GoalScript load_goal_script(const std::string & path)
{
  if (!std::filesystem::is_regular_file(path) || std::filesystem::file_size(path) > (1u << 20))
    throw std::invalid_argument("goal script must be a regular file no larger than 1 MiB: " + path);
  std::ifstream in(path, std::ios::binary);
  const std::string text(std::istreambuf_iterator<char>(in), {});
  return parse_goal_script(text);
}

ScriptRun run_goal_script(Session & session, const GoalScript & script)
{
  if (!session.config().generated_routes) throw std::invalid_argument("a goal script needs a generated-route session");
  ScriptRun run;
  const double limit = script.time_limit_s.value_or(kMaxTimeS);
  std::uint64_t request = 0;
  auto submission = [&](const ScriptGoal & g) {
    return GoalSubmission{session.goal_manager().epoch(), "script", ++request, g.target, g.timeout_s};
  };
  // the timed goals go to the goal manager's schedule now; it applies each at its control boundary
  for (const auto & g : script.goals) {
    if (!g.at_s) continue;
    const auto ack = session.schedule_goal(submission(g), *g.at_s);
    run.submissions.push_back({g.id, *g.at_s, ack.action, ack.reason});
  }
  std::size_t next_after = 0;  // the next script goal to consider for an `after` submission
  auto idle = [&] { return !session.active() && session.goal_manager().scheduled_count() == 0; };
  for (;;) {
    // `after` goals: the previous script goal has been handed over and nothing runs or waits
    while (next_after < script.goals.size()) {
      const auto & g = script.goals[next_after];
      if (g.at_s) { ++next_after; continue; }
      if (!idle()) break;
      const auto ack = session.submit_goal(submission(g));
      run.submissions.push_back({g.id, session.time(), ack.action, ack.reason});
      ++next_after;
      break;  // one per boundary: the next waits for this one to end
    }
    if (session.time() >= limit - 1e-12) { run.time_limit_reached = true; break; }
    if (script.end_when_idle && next_after >= script.goals.size() && idle()) break;
    session.advance();
  }
  run.end_time_s = session.time();
  return run;
}

json to_json(const ScriptRun & run)
{
  json submissions = json::array();
  for (const auto & s : run.submissions)
    submissions.push_back({{"id", s.id}, {"at_s", s.at_s}, {"action", to_string(s.action)}, {"reason", s.reason}});
  return {{"submissions", submissions}, {"time_limit_reached", run.time_limit_reached}, {"end_time_s", run.end_time_s}};
}
}  // namespace mobile_manipulator_campaign
