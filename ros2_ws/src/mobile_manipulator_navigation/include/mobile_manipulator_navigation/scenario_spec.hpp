#pragma once
// Navigation scenarios (config/scenarios.yaml) and their static-map checks; no ROS graph.
//
// A scenario is accepted when its footprint profile matches its arm pose, a
// navigate_to_pose task has a timeout, and its unmapped obstacle boxes lie in free map
// space clear of the start and goal footprints. Whether every map cell under the posed,
// enlarged footprint at the start and goal is free is reported separately.
#include <array>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "mobile_manipulator_geometry/polygon.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace mobile_manipulator_navigation
{
// Unknown scenario or footprint profile.
struct UnknownScenario : std::runtime_error
{
  using std::runtime_error::runtime_error;
};

// Inconsistent scenario definition.
struct InvalidScenario : std::runtime_error
{
  using std::runtime_error::runtime_error;
};

using Pose2 = std::array<double, 3>;               // x, y, yaw
using Polygon = mobile_manipulator_geometry::Polygon;  // base_footprint frame

struct StaticMap
{
  double resolution, origin_x, origin_y;
  int width, height;
  std::vector<unsigned char> pixels;  // PGM rows, top row first

  // Trinary map value of a cell (row 0 at the map origin).
  unsigned char value(int col, int row) const { return pixels[(height - 1 - row) * width + col]; }
  bool contains(int col, int row) const { return col >= 0 && col < width && row >= 0 && row < height; }
};

// The site of a scenario file without a top-level `site:` key, and of V1.
inline constexpr const char * kDefaultSite = "construction_site";

// Package configuration and maps, from a package root (source or share directory). Scenarios
// come from every config/scenarios*.yaml; a file's top-level `site:` (default kDefaultSite)
// names the site its scenarios run in, and maps/<site>.yaml is that site's static map.
class ScenarioConfig
{
public:
  explicit ScenarioConfig(std::string root);

  Json load(const std::string & config_name) const;  // config/<name>
  // The site's static map, read on first use.
  const StaticMap & map(const std::string & site = kDefaultSite) const;

  // Every scenario name across all scenario files, sorted. A name in two files is InvalidScenario.
  std::vector<std::string> names() const;
  // The site of the scenario's file (UnknownScenario for an unknown name).
  std::string site_of(const std::string & name) const;
  // The Unity scene of a site: `sourceScene` of maps/<site>.metadata.json.
  std::string scene_of(const std::string & site) const;

  // The scenario, its footprint polygon, and the start/goal free-space check. With arm_pose
  // (run_nav_scenario.py --arm-pose), the scenario starts in that qualified pose instead: its
  // arm_pose and footprint_profile become that pose and the profile of the same name, which must
  // describe it (InvalidScenario otherwise), and every check uses that profile. The result also
  // has "site", "scene" (its Unity scene) and "disturbances" (the scenario's flag, default
  // false); every map check uses the scenario's site map.
  Json resolve(const std::string & name, const std::optional<std::string> & arm_pose = std::nullopt) const;

private:
  struct Entry
  {
    std::string file, site;
    Json scenario;
  };
  std::map<std::string, Entry> index() const;  // scenario name -> definition

  std::string root_;
  mutable std::map<std::string, StaticMap> maps_;  // by site
};

StaticMap read_map(const std::string & root, const std::string & site = kDefaultSite);
Polygon polygon_of(const Json & points);
Pose2 pose_of(const Json & pose);

struct FreeCheck
{
  bool free;
  Json blocked;  // up to 10 entries: [col, row, "outside map"] or [x, y, value]
};

// True when every map cell whose centre lies under the posed, enlarged footprint is free.
FreeCheck start_is_free(const StaticMap & map, const Polygon & polygon, const Pose2 & pose,
                        double margin = 0.05);

// Points covering an axis-aligned obstacle box (ROS map frame), boundary and interior.
std::vector<std::array<double, 2>> box_points(const Json & obstacle, double spacing = 0.05);

// Why the scenario's obstacles are invalid (empty when they are fine).
std::vector<std::string> obstacle_problems(const StaticMap & map, const Json & scenario,
                                           const Polygon & polygon, double clearance = 0.3);

// Why the scenario's movers are invalid (empty when they are fine). A mover is a box that
// walks from `start` to `end` (and back, for even crossings) once the robot comes within
// trigger_distance_m of the segment midpoint. Its whole walked area must be free map space;
// only where it waits and where it stops must be clear of the start and goal footprints,
// since crossing the route is the point.
std::vector<std::string> mover_problems(const StaticMap & map, const Json & scenario,
                                        const Polygon & polygon, double clearance = 0.3);
}  // namespace mobile_manipulator_navigation
