#include "mobile_manipulator_navigation/mission.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace mobile_manipulator_navigation
{
namespace
{
bool is_vector(const Json & value, size_t size)
{
  if (!value.is_array() || value.size() != size) return false;
  return std::all_of(value.begin(), value.end(), [](const Json & v) { return v.is_number(); });
}

std::string pose_text(const Pose2 & pose)
{
  char text[64];
  std::snprintf(text, sizeof(text), "(%.2f, %.2f, %.2f)", pose[0], pose[1], pose[2]);
  return text;
}
}  // namespace

std::vector<MissionStep> parse_mission(const Json & scenario)
{
  if (!scenario.contains("steps") || !scenario.at("steps").is_array() || scenario.at("steps").empty()) {
    throw InvalidScenario("a mission needs a non-empty steps list");
  }
  std::vector<MissionStep> steps;
  for (const auto & item : scenario.at("steps")) {
    if (!item.is_object() || item.size() != 1) throw InvalidScenario("mission step must be one mapping: " + item.dump());
    if (item.contains("navigate")) {
      if (!is_vector(item.at("navigate"), 3)) throw InvalidScenario("navigate needs [x, y, yaw]: " + item.dump());
      steps.push_back({MissionStep::Kind::Navigate, pose_of(item.at("navigate")), Json()});
    } else if (item.contains("reconfigure") && item.at("reconfigure").is_object()) {
      steps.push_back({MissionStep::Kind::Reconfigure, {}, item.at("reconfigure")});
    } else {
      throw InvalidScenario("mission step must be navigate or reconfigure: " + item.dump());
    }
  }
  return steps;
}

std::vector<std::string> mission_problems(const StaticMap & map, const Json & scenario, const Json & profiles)
{
  std::vector<MissionStep> steps;
  try {
    steps = parse_mission(scenario);
  } catch (const InvalidScenario & error) {
    return {error.what()};
  }
  std::vector<std::string> problems;
  std::string profile = scenario.at("footprint_profile").get<std::string>();
  Pose2 at = pose_of(scenario.at("start"));
  std::optional<Pose2> last_drive;
  const auto free_for = [&](const std::string & name, const Pose2 & pose) {
    return profiles.contains(name) && start_is_free(map, polygon_of(profiles.at(name).at("polygon")), pose).free;
  };
  for (size_t i = 0; i < steps.size(); ++i) {
    const std::string label = "step " + std::to_string(i + 1);
    const auto & step = steps[i];
    if (step.kind == MissionStep::Kind::Navigate) {
      if (!free_for(profile, step.pose)) {
        problems.push_back(label + ": drive end " + pose_text(step.pose) + " is not free for profile " + profile);
      }
      at = step.pose;
      last_drive = step.pose;
      continue;
    }
    const auto & r = step.reconfigure;
    const std::string next = r.value("footprint_profile", std::string());
    if (!profiles.contains(next)) {
      problems.push_back(label + ": unknown footprint profile '" + next + "'");
      continue;
    }
    if (r.contains("named_state") == r.contains("panel_pose")) {
      problems.push_back(label + ": needs exactly one of named_state or panel_pose");
    } else if (r.contains("named_state") && !r.at("named_state").is_string()) {
      problems.push_back(label + ": named_state must be a string");
    } else if (r.contains("panel_pose")) {
      const auto & pose = r.at("panel_pose");
      if (!pose.is_object() || !pose.contains("xyz") || !pose.contains("rpy") || !is_vector(pose.at("xyz"), 3) ||
          !is_vector(pose.at("rpy"), 3)) {
        problems.push_back(label + ": panel_pose needs xyz and rpy");
      }
      for (const char * key : {"position_tolerance", "orientation_tolerance"}) {
        if (!r.contains(key) || !is_vector(r.at(key), 3)) problems.push_back(label + ": panel_pose needs " + key);
      }
    }
    for (const auto & name : {profile, next}) {
      if (!free_for(name, at)) {
        problems.push_back(label + ": reconfiguration pose " + pose_text(at) + " is not free for profile " + name);
      }
    }
    profile = next;
  }
  if (!last_drive) {
    problems.push_back("a mission needs at least one navigate step");
  } else {
    const Pose2 goal = pose_of(scenario.at("goal"));
    if (std::hypot((*last_drive)[0] - goal[0], (*last_drive)[1] - goal[1]) > 1e-6 ||
        std::abs(std::remainder((*last_drive)[2] - goal[2], 2 * M_PI)) > 1e-6) {
      problems.push_back("the last navigate step must end at the scenario goal");
    }
  }
  return problems;
}

Polygon padded_rectangle(const Polygon & polygon, double margin)
{
  double x0 = polygon.at(0)[0], x1 = x0, y0 = polygon.at(0)[1], y1 = y0;
  for (const auto & [x, y] : polygon) {
    x0 = std::min(x0, x), x1 = std::max(x1, x), y0 = std::min(y0, y), y1 = std::max(y1, y);
  }
  return {{x1 + margin, y1 + margin}, {x1 + margin, y0 - margin}, {x0 - margin, y0 - margin}, {x0 - margin, y1 + margin}};
}

std::vector<MonitorZone> monitor_zones(const Json & nav2_navigation)
{
  const auto & monitor = nav2_navigation.at("collision_monitor").at("ros__parameters");
  std::vector<MonitorZone> zones;
  for (const auto & name : monitor.at("polygons")) {
    const auto & zone = monitor.at(name.get<std::string>());
    if (!zone.contains("margin_m")) continue;
    const auto visual = zone.at("polygon_pub_topic").get<std::string>();
    zones.push_back({name.get<std::string>(), zone.at("margin_m").get<double>(),
                     zone.at("dynamic_polygon_topic").get<std::string>(),
                     visual.rfind("/", 0) == 0 ? visual : "/" + visual});
  }
  return zones;
}

std::vector<std::string> monitor_zone_problems(const std::vector<MonitorZone> & zones,
                                               const std::map<std::string, std::string> & subscribed)
{
  const auto absolute = [](const std::string & topic) { return topic.rfind("/", 0) == 0 ? topic : "/" + topic; };
  std::vector<std::string> problems;
  for (const auto & zone : zones) {
    const auto found = subscribed.find(zone.name);
    if (found == subscribed.end() || found->second.empty()) {
      problems.push_back("collision monitor zone " + zone.name + " has static points, not " + zone.polygon_topic +
                         " (launch navigation with footprint_mode:=profiles or dynamic)");
    } else if (absolute(found->second) != zone.polygon_topic) {
      problems.push_back("collision monitor zone " + zone.name + " follows " + found->second + ", not " +
                         zone.polygon_topic);
    }
  }
  return problems;
}

void Stillness::add(double t, double linear, double angular)
{
  if (std::abs(linear) >= v_max_ || std::abs(angular) >= w_max_) still_since_.reset();
  else if (!still_since_) still_since_ = t;
}

bool FootprintDriftGuard::republish(bool matches, double now)
{
  if (matches) {
    since_.reset();
    return false;
  }
  if (!since_) since_ = now;
  if (now - *since_ < patience_s_) return false;
  since_ = now;
  return true;
}

std::vector<std::string> ownership_problems(const std::string & mode,
                                            const std::map<std::string, std::vector<std::string>> & publishers_by_topic)
{
  const std::map<std::string, std::string> owners = {
    {"static", ""}, {"profiles", "reconfigure_panel_server"}, {"dynamic", "dynamic_footprint_node"}};
  const auto owner = owners.find(mode);
  if (owner == owners.end()) return {"unknown footprint_mode '" + mode + "' (static, profiles or dynamic)"};
  std::vector<std::string> problems;
  for (const auto & topic : kFootprintTopics) {
    const auto found = publishers_by_topic.find(topic);
    const std::vector<std::string> nodes = found == publishers_by_topic.end() ? std::vector<std::string>{} : found->second;
    const std::vector<std::string> expected = owner->second.empty() ? std::vector<std::string>{}
                                                                    : std::vector<std::string>{owner->second};
    if (nodes == expected) continue;
    std::string listed;
    for (const auto & node : nodes) listed += (listed.empty() ? "" : ", ") + node;
    problems.push_back(topic + ": published by [" + listed + "], footprint_mode " + mode + " needs " +
                       (expected.empty() ? std::string("no publisher") : "only " + expected[0]));
  }
  return problems;
}

double Stillness::still_for(double now) const { return still_since_ ? std::max(0.0, now - *still_since_) : 0.0; }

Polygon to_base_frame(const Polygon & world, const Pose2 & robot)
{
  const double c = std::cos(robot[2]), s = std::sin(robot[2]);
  Polygon base;
  for (const auto & [x, y] : world) {
    const double dx = x - robot[0], dy = y - robot[1];
    base.push_back({c * dx + s * dy, -s * dx + c * dy});
  }
  return base;
}

bool footprint_matches(const Polygon & published, const Polygon & expected, double padding, double tolerance)
{
  if (published.size() != expected.size()) return false;
  std::vector<bool> used(published.size(), false);
  for (const auto & [x, y] : expected) {
    const double px = x + std::copysign(padding, x), py = y + std::copysign(padding, y);
    bool found = false;
    for (size_t i = 0; i < published.size() && !found; ++i) {
      if (!used[i] && std::abs(published[i][0] - px) <= tolerance && std::abs(published[i][1] - py) <= tolerance) {
        used[i] = found = true;
      }
    }
    if (!found) return false;
  }
  return true;
}

bool footprints_applied(const std::map<std::string, PublishedPolygon> & latest,
                        const std::vector<std::string> & costmap_topics, const std::vector<MonitorZone> & zones,
                        const Polygon & profile, const Pose2 & robot, double since, double padding)
{
  const auto in_base = [&robot](const PublishedPolygon & p) {
    return p.frame == "base_footprint" ? p.polygon : to_base_frame(p.polygon, robot);
  };
  for (const auto & topic : costmap_topics) {
    const auto found = latest.find(topic);
    if (found == latest.end() || found->second.received < since ||
        !footprint_matches(in_base(found->second), profile, padding))
    {
      return false;
    }
  }
  for (const auto & zone : zones) {
    const auto found = latest.find(zone.polygon_topic);
    if (found == latest.end() ||
        !footprint_matches(in_base(found->second), padded_rectangle(profile, zone.margin_m), 0.0))
    {
      return false;
    }
  }
  return true;
}

bool FootprintRefresh::observe(bool applied, double now)
{
  if (applied && !shown_) {
    shown_ = true;
    since_ = now;
  } else if (applied && shown_) {
    refreshed_ = true;
  }
  return refreshed_;
}
}  // namespace mobile_manipulator_navigation
