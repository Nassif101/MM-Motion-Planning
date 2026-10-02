#include "mobile_manipulator_navigation/scenario_spec.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <set>
#include <sstream>
#include <tuple>

namespace mobile_manipulator_navigation
{
namespace
{
const std::set<std::string> kObstacleKeys = {"name", "x", "y", "size_x", "size_y", "height"};
const std::set<std::string> kMoverKeys = {"name", "start", "end", "size_x", "size_y", "height",
                                          "speed_mps", "trigger_distance_m", "crossings"};

std::set<std::string> keys_of(const Json & object)
{
  std::set<std::string> keys;
  for (const auto & item : object.items()) keys.insert(item.key());
  return keys;
}

double round_to(double value, int digits)
{
  const double scale = std::pow(10.0, digits);
  return std::round(value * scale) / scale;
}

// Shortest decimal form, as Python prints floats.
std::string number_text(double value)
{
  return Json(value).dump();
}

struct Bounds
{
  double x_min, x_max, y_min, y_max;
};

Bounds bounds_of(const Polygon & polygon, double margin)
{
  Bounds b{polygon[0][0], polygon[0][0], polygon[0][1], polygon[0][1]};
  for (const auto & p : polygon) {
    b.x_min = std::min(b.x_min, p[0]);
    b.x_max = std::max(b.x_max, p[0]);
    b.y_min = std::min(b.y_min, p[1]);
    b.y_max = std::max(b.y_max, p[1]);
  }
  return {b.x_min - margin, b.x_max + margin, b.y_min - margin, b.y_max + margin};
}
// True when any point is within `clearance` of the footprint posed at `pose`.
bool near_footprint(const std::vector<std::array<double, 2>> & points, const Pose2 & pose, const Bounds & b)
{
  const double c = std::cos(pose[2]), s = std::sin(pose[2]);
  for (const auto & [x, y] : points) {
    const double dx = x - pose[0], dy = y - pose[1];
    const double bx = c * dx + s * dy, by = -s * dx + c * dy;
    if (b.x_min <= bx && bx <= b.x_max && b.y_min <= by && by <= b.y_max) return true;
  }
  return false;
}

// Why the box points are not all in free map space (empty when they are).
std::string occupancy_problem(const StaticMap & map, const std::vector<std::array<double, 2>> & points)
{
  for (const auto & [x, y] : points) {
    const int col = static_cast<int>((x - map.origin_x) / map.resolution);
    const int row = static_cast<int>((y - map.origin_y) / map.resolution);
    if (!map.contains(col, row)) return "outside the map";
    if (map.value(col, row) != 254) {
      char where[96];
      std::snprintf(where, sizeof(where), "overlaps mapped obstacle or unknown space at (%.2f, %.2f)", x, y);
      return where;
    }
  }
  return "";
}

bool is_point(const Json & value)
{
  return value.is_array() && value.size() == 2 && value[0].is_number() && value[1].is_number();
}
}  // namespace

StaticMap read_map(const std::string & root)
{
  const Json meta = load_yaml_file(root + "/maps/construction_site.yaml");
  std::ifstream stream(root + "/maps/" + meta.at("image").get<std::string>(), std::ios::binary);
  if (!stream) throw std::runtime_error("Cannot read the static map image");
  std::string line;
  std::getline(stream, line);
  if (line.rfind("P5", 0) != 0) throw std::runtime_error("The static map is not a binary PGM");
  do {
    std::getline(stream, line);
  } while (line.rfind("#", 0) == 0);
  StaticMap map{meta.at("resolution").get<double>(), meta.at("origin")[0].get<double>(),
                meta.at("origin")[1].get<double>(), 0, 0, {}};
  std::istringstream(line) >> map.width >> map.height;
  std::getline(stream, line);  // maximum value
  map.pixels.assign(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
  if (map.pixels.size() < static_cast<size_t>(map.width) * map.height) {
    throw std::runtime_error("The static map image is truncated");
  }
  return map;
}

Polygon polygon_of(const Json & points)
{
  Polygon polygon;
  for (const auto & p : points) polygon.push_back({p[0].get<double>(), p[1].get<double>()});
  return polygon;
}

Pose2 pose_of(const Json & pose)
{
  return {pose[0].get<double>(), pose[1].get<double>(), pose[2].get<double>()};
}

FreeCheck start_is_free(const StaticMap & map, const Polygon & polygon, const Pose2 & pose, double margin)
{
  const Bounds b = bounds_of(polygon, margin);
  const double cos_yaw = std::cos(pose[2]), sin_yaw = std::sin(pose[2]);
  const double reach = std::hypot(std::max(std::abs(b.x_min), std::abs(b.x_max)),
                                  std::max(std::abs(b.y_min), std::abs(b.y_max)));
  Json blocked = Json::array();
  const int col0 = static_cast<int>((pose[0] - reach - map.origin_x) / map.resolution);
  const int row0 = static_cast<int>((pose[1] - reach - map.origin_y) / map.resolution);
  const int span = static_cast<int>(2 * reach / map.resolution) + 2;
  for (int col = col0; col < col0 + span; ++col) {
    for (int row = row0; row < row0 + span; ++row) {
      const double cx = map.origin_x + (col + 0.5) * map.resolution - pose[0];
      const double cy = map.origin_y + (row + 0.5) * map.resolution - pose[1];
      // Cell centre in the robot frame.
      const double bx = cos_yaw * cx + sin_yaw * cy, by = -sin_yaw * cx + cos_yaw * cy;
      if (!(b.x_min <= bx && bx <= b.x_max && b.y_min <= by && by <= b.y_max)) continue;
      if (!map.contains(col, row)) {
        blocked.push_back({col, row, "outside map"});
        continue;
      }
      const int value = map.value(col, row);
      if (value != 254) {  // trinary map: 254 free, 0 occupied, 205 unknown
        blocked.push_back({round_to(cx + pose[0], 3), round_to(cy + pose[1], 3), value});
      }
    }
  }
  const bool free = blocked.empty();
  Json first = Json::array();
  for (size_t i = 0; i < std::min<size_t>(10, blocked.size()); ++i) first.push_back(blocked[i]);
  return {free, first};
}

std::vector<std::array<double, 2>> box_points(const Json & obstacle, double spacing)
{
  const double size_x = obstacle.at("size_x").get<double>(), size_y = obstacle.at("size_y").get<double>();
  const int nx = std::max(2, static_cast<int>(std::nearbyint(size_x / spacing)) + 1);
  const int ny = std::max(2, static_cast<int>(std::nearbyint(size_y / spacing)) + 1);
  const double x0 = obstacle.at("x").get<double>() - size_x / 2;
  const double y0 = obstacle.at("y").get<double>() - size_y / 2;
  std::vector<std::array<double, 2>> points;
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) points.push_back({x0 + i * size_x / (nx - 1), y0 + j * size_y / (ny - 1)});
  }
  return points;
}

std::vector<std::string> obstacle_problems(const StaticMap & map, const Json & scenario,
                                           const Polygon & polygon, double clearance)
{
  std::vector<std::string> problems;
  if (!scenario.contains("obstacles")) return problems;
  for (const auto & obstacle : scenario.at("obstacles")) {
    std::set<std::string> keys;
    for (const auto & item : obstacle.items()) {
      if (item.key() != "clearance_m") keys.insert(item.key());
    }
    if (keys != kObstacleKeys) {
      problems.push_back("obstacle needs exactly ['height', 'name', 'size_x', 'size_y', 'x', 'y'] "
                         "(optionally 'clearance_m'): " + obstacle.dump());
      continue;
    }
    const std::string label = obstacle.at("name").get<std::string>();
    const double size_x = obstacle.at("size_x").get<double>(), size_y = obstacle.at("size_y").get<double>();
    const double height = obstacle.at("height").get<double>();
    if (!(0 < size_x && size_x <= 5 && 0 < size_y && size_y <= 5 && 0 < height && height <= 3)) {
      problems.push_back(label + ": sizes must be in (0, 5] m and height in (0, 3] m");
    }
    const auto points = box_points(obstacle);
    for (const auto & [x, y] : points) {
      const int col = static_cast<int>((x - map.origin_x) / map.resolution);
      const int row = static_cast<int>((y - map.origin_y) / map.resolution);
      if (!map.contains(col, row)) {
        problems.push_back(label + ": outside the map");
        break;
      }
      if (map.value(col, row) != 254) {
        char where[64];
        std::snprintf(where, sizeof(where), "(%.2f, %.2f)", x, y);
        problems.push_back(label + ": overlaps mapped obstacle or unknown space at " + where);
        break;
      }
    }
    // A scenario may stand one obstacle closer to the robot on purpose (e.g. beside a
    // reconfiguration pose); it never overlaps the footprint.
    const double own_clearance = obstacle.value("clearance_m", clearance);
    if (own_clearance < 0.0) problems.push_back(label + ": clearance_m must not be negative");
    const Bounds b = bounds_of(polygon, std::max(own_clearance, 0.0));
    for (const char * which : {"start", "goal"}) {
      const Pose2 pose = pose_of(scenario.at(which));
      const double c = std::cos(pose[2]), s = std::sin(pose[2]);
      for (const auto & [x, y] : points) {
        const double dx = x - pose[0], dy = y - pose[1];
        const double bx = c * dx + s * dy, by = -s * dx + c * dy;
        if (b.x_min <= bx && bx <= b.x_max && b.y_min <= by && by <= b.y_max) {
          problems.push_back(label + ": within " + number_text(own_clearance) + " m of the " + which + " footprint");
          break;
        }
      }
    }
  }
  return problems;
}

std::vector<std::string> mover_problems(const StaticMap & map, const Json & scenario,
                                        const Polygon & polygon, double clearance)
{
  std::vector<std::string> problems;
  if (!scenario.contains("movers")) return problems;
  const Bounds bounds = bounds_of(polygon, clearance);
  for (const auto & mover : scenario.at("movers")) {
    if (keys_of(mover) != kMoverKeys) {
      problems.push_back("mover needs exactly ['crossings', 'end', 'height', 'name', 'size_x', 'size_y', "
                         "'speed_mps', 'start', 'trigger_distance_m']: " + mover.dump());
      continue;
    }
    const std::string label = mover.at("name").get<std::string>();
    if (!is_point(mover.at("start")) || !is_point(mover.at("end"))) {
      problems.push_back(label + ": start and end must be [x, y]");
      continue;
    }
    const double size_x = mover.at("size_x").get<double>(), size_y = mover.at("size_y").get<double>();
    const double height = mover.at("height").get<double>(), speed = mover.at("speed_mps").get<double>();
    const double trigger = mover.at("trigger_distance_m").get<double>();
    const Json & crossings = mover.at("crossings");
    if (!(0 < size_x && size_x <= 5 && 0 < size_y && size_y <= 5 && 0 < height && height <= 3)) {
      problems.push_back(label + ": sizes must be in (0, 5] m and height in (0, 3] m");
    }
    if (!(0 < speed && speed <= 2) || !(0 < trigger && trigger <= 20)) {
      problems.push_back(label + ": speed_mps must be in (0, 2] and trigger_distance_m in (0, 20]");
    }
    if (!crossings.is_number_integer() || crossings.get<int>() < 1 || crossings.get<int>() > 10) {
      problems.push_back(label + ": crossings must be an integer from 1 to 10");
      continue;
    }
    const double sx = mover.at("start")[0].get<double>(), sy = mover.at("start")[1].get<double>();
    const double ex = mover.at("end")[0].get<double>(), ey = mover.at("end")[1].get<double>();
    const double length = std::hypot(ex - sx, ey - sy);
    if (!(length >= 0.1)) {
      problems.push_back(label + ": the segment must be at least 0.1 m long");
      continue;
    }
    const auto box_at = [&](double x, double y) {
      return box_points({{"x", x}, {"y", y}, {"size_x", size_x}, {"size_y", size_y}});
    };
    const int samples = static_cast<int>(std::ceil(length / 0.05));
    for (int i = 0; i <= samples; ++i) {
      const double u = static_cast<double>(i) / samples;
      const auto problem = occupancy_problem(map, box_at(sx + u * (ex - sx), sy + u * (ey - sy)));
      if (!problem.empty()) {
        problems.push_back(label + ": walked area " + problem);
        break;
      }
    }
    const bool returns = crossings.get<int>() % 2 == 0;
    const std::array<std::tuple<const char *, double, double>, 2> rests = {
      std::tuple<const char *, double, double>{"waiting", sx, sy}, {"final", returns ? sx : ex, returns ? sy : ey}};
    for (const auto & [where, x, y] : rests) {
      for (const char * which : {"start", "goal"}) {
        if (near_footprint(box_at(x, y), pose_of(scenario.at(which)), bounds)) {
          problems.push_back(label + ": " + where + " position within " + number_text(clearance) + " m of the " +
                             which + " footprint");
        }
      }
    }
  }
  return problems;
}

ScenarioConfig::ScenarioConfig(std::string root) : root_(std::move(root)), map_(read_map(root_)) {}

Json ScenarioConfig::load(const std::string & config_name) const
{
  return load_yaml_file(root_ + "/config/" + config_name);
}

Json ScenarioConfig::resolve(const std::string & name) const
{
  const Json scenarios = load("scenarios.yaml").at("scenarios");
  if (!scenarios.contains(name)) {
    std::vector<std::string> names;
    for (const auto & item : scenarios.items()) names.push_back(item.key());
    std::sort(names.begin(), names.end());
    std::string known;
    for (const auto & known_name : names) known += (known.empty() ? "" : ", ") + known_name;
    throw UnknownScenario("Unknown scenario '" + name + "'; expected one of " + known);
  }
  Json scenario = scenarios.at(name);
  scenario["name"] = name;
  const Json profiles = load("footprint_profiles.yaml").at("profiles");
  const std::string profile_name = scenario.at("footprint_profile").get<std::string>();
  if (!profiles.contains(profile_name)) throw UnknownScenario("Unknown footprint profile '" + profile_name + "'");
  const Json & profile = profiles.at(profile_name);
  if (profile.at("arm_pose") != scenario.at("arm_pose")) {
    throw InvalidScenario(name + ": footprint profile " + profile_name + " describes arm pose " +
                          profile.at("arm_pose").get<std::string>() + ", not " +
                          scenario.at("arm_pose").get<std::string>());
  }
  const Polygon polygon = polygon_of(profile.at("polygon"));
  const FreeCheck start = start_is_free(map_, polygon, pose_of(scenario.at("start")));
  const FreeCheck goal = start_is_free(map_, polygon, pose_of(scenario.at("goal")));
  if (scenario.at("task") == "navigate_to_pose" &&
      (!scenario.contains("timeout_s") || scenario.at("timeout_s").is_null() || scenario.at("timeout_s") == 0)) {
    throw InvalidScenario(name + ": navigate_to_pose needs timeout_s");
  }
  auto problems = obstacle_problems(map_, scenario, polygon);
  const auto mover_issues = mover_problems(map_, scenario, polygon);
  problems.insert(problems.end(), mover_issues.begin(), mover_issues.end());
  std::set<std::string> names;
  for (const char * kind : {"obstacles", "movers"}) {
    if (!scenario.contains(kind)) continue;
    for (const auto & item : scenario.at(kind)) {
      if (item.contains("name") && !names.insert(item.at("name").dump()).second) {
        problems.push_back("duplicate obstacle or mover name " + item.at("name").dump());
      }
    }
  }
  if (!problems.empty()) {
    std::string text = name + ": ";
    for (size_t i = 0; i < problems.size(); ++i) text += (i ? "; " : "") + problems[i];
    throw InvalidScenario(text);
  }
  Json blocked = start.blocked;
  for (const auto & cell : goal.blocked) blocked.push_back(cell);
  return {{"scenario", scenario}, {"polygon", profile.at("polygon")},
          {"start_free", start.free && goal.free}, {"blocked_cells", blocked}};
}
}  // namespace mobile_manipulator_navigation
