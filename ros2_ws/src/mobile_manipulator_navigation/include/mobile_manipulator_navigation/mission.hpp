#pragma once
// B3 mission scenarios (task: mission): an ordered list of NavigateToPose drives and
// ReconfigurePanel steps; no ROS graph.
//
//   steps:
//     - navigate: [x, y, yaw]                       # map frame
//     - reconfigure:
//         panel_pose: {xyz: [x, y, z], rpy: [r, p, y]}   # panel centre in base_footprint
//         position_tolerance: [dx, dy, dz]
//         orientation_tolerance: [rx, ry, rz]           # rotation-vector components
//         footprint_profile: vertical_carry
//     - reconfigure: {named_state: home, footprint_profile: home}
//
// The scenario's footprint_profile is the profile at the start; each reconfigure switches it.
#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "mobile_manipulator_navigation/scenario_spec.hpp"

namespace mobile_manipulator_navigation
{
struct MissionStep
{
  enum class Kind { Navigate, Reconfigure } kind;
  Pose2 pose{};       // Navigate
  Json reconfigure;   // Reconfigure: the step's mapping as written
};

// The steps in order; throws InvalidScenario when a step is not exactly one known kind.
std::vector<MissionStep> parse_mission(const Json & scenario);

// Why the mission is invalid (empty when it is fine): malformed steps, unknown profiles,
// a reconfigure without exactly one of named_state / panel_pose (a panel pose also needs
// both tolerances), a drive whose end pose is not free for the active profile, a
// reconfiguration pose not free for the profile before and after it, or a last drive that
// does not end at the scenario goal.
std::vector<std::string> mission_problems(const StaticMap & map, const Json & scenario, const Json & profiles);

// Axis-aligned rectangle around `polygon` grown by `margin` on every side
// (navigation.launch.py padded(); the profiles are rectangles).
Polygon padded_rectangle(const Polygon & polygon, double margin);

// A collision-monitor zone sized from the footprint profile (config/nav2_navigation.yaml).
struct MonitorZone
{
  std::string name;
  double margin_m;
  std::string polygon_topic;  // dynamic_polygon_topic: zone input with footprint_mode profiles or dynamic
  std::string visual_topic;   // polygon_pub_topic: the zone the monitor is using
};

// The margin-sized zones, in the order the monitor lists them.
std::vector<MonitorZone> monitor_zones(const Json & nav2_navigation);

// Why the collision monitor does not take its zones from ReconfigurePanel (empty when it
// does). `subscribed` maps a zone name to the monitor's <zone>.polygon_sub_topic parameter;
// a zone is missing when the monitor does not declare it (static zones, launched without
// footprint_mode profiles or dynamic).
std::vector<std::string> monitor_zone_problems(const std::vector<MonitorZone> & zones,
                                               const std::map<std::string, std::string> & subscribed);

// How long the base has been at rest: speeds below the limits since the first such /odom
// sample after the last moving one (0 while moving or before any sample).
class Stillness
{
public:
  Stillness(double v_max, double w_max) : v_max_(v_max), w_max_(w_max) {}
  void add(double t, double linear, double angular);
  double still_for(double now) const;

private:
  double v_max_, w_max_;
  std::optional<double> still_since_;
};

// A polygon given in the global frame (map; the identity map -> odom makes odom the same)
// expressed in base_footprint for the robot pose (x, y, yaw) in that frame. Costmaps
// publish their footprint posed in the global frame.
Polygon to_base_frame(const Polygon & world, const Pose2 & robot);

// A footprint padded the way Nav2's padFootprint pads it: each coordinate moved outward by
// `padding` (sign0: a coordinate of exactly 0 stays).
Polygon nav2_padded(const Polygon & polygon, double padding);

// Whether a costmap's published (padded) footprint is `expected` grown by `padding` the way
// Nav2 pads a footprint (each vertex moved outward by `padding` in x and y), in any vertex
// order, within `tolerance`.
bool footprint_matches(const Polygon & published, const Polygon & expected, double padding,
                       double tolerance = 0.005);

// When to republish the active footprint to a costmap that shows another one. The costmaps
// subscribe to their footprint topic as volatile, so a relaunched Nav2 keeps its launch
// profile and never receives the latched message. Feed one check per costmap per tick;
// `matches` is true when it shows the active footprint or cannot be judged (not publishing,
// base moving, reconfiguration running). Republish after `patience_s` of disagreement (a
// switch shows within one costmap cycle), then again every `patience_s` while it persists.
class FootprintDriftGuard
{
public:
  explicit FootprintDriftGuard(double patience_s = 2.0) : patience_s_(patience_s) {}
  bool republish(bool matches, double now);

private:
  double patience_s_;
  std::optional<double> since_;
};

// The topics only the footprint owner may publish: both costmap footprints and the
// collision monitor's stop and slowdown zone inputs.
inline const std::array<std::string, 4> kFootprintTopics = {
  "/global_costmap/footprint", "/local_costmap/footprint", "/collision_monitor/stop_zone_in",
  "/collision_monitor/slowdown_zone_in"};

// Why the publishers on kFootprintTopics do not match footprint_mode (empty when they do):
// static needs none, profiles exactly reconfigure_panel_server, dynamic exactly
// dynamic_footprint_node. `publishers_by_topic` maps a topic to its publishing node names.
std::vector<std::string> ownership_problems(const std::string & mode,
                                            const std::map<std::string, std::vector<std::string>> & publishers_by_topic);

// A costmap as nav_msgs/OccupancyGrid carries it: row-major from the origin cell, costs
// 0-100 (100 lethal, 99 inscribed), -1 unknown.
struct CostGrid
{
  double resolution, origin_x, origin_y;
  int width, height;
  std::vector<int8_t> data;
};

struct HullClearance
{
  bool collision;
  double clearance_m;
};

// Whether the footprint (base_footprint) posed at `base_in_map` covers a lethal cell: the
// polygon overlaps (or touches) a lethal cell's square, as Nav2's outline rasterization would
// hit it (pass the footprint padded as Nav2 pads it, nav2_padded). clearance_m is the
// distance from the polygon to the nearest lethal cell centre minus half a cell (0 on
// collision), or search_radius_m when no lethal cell lies within it. Inscribed and unknown
// costs are not collisions: Nav2 rejects a start pose only on lethal cost.
HullClearance hull_clearance(const CostGrid & costmap, const Polygon & footprint_base, const Pose2 & base_in_map,
                             int lethal = 100, double search_radius_m = 1.0);

// The latest polygon received on a footprint or zone topic.
struct PublishedPolygon
{
  Polygon polygon;
  std::string frame;  // base_footprint, or the costmap's global frame (map / odom)
  double received;    // receive time (wall seconds)
};

// Whether the costmaps and the collision-monitor zone inputs all show `profile` at `robot`:
// each costmap topic has a message received at or after `since` matching the profile grown by
// `padding`, and each zone input (latched) matches the profile grown by the zone's margin.
bool footprints_applied(const std::map<std::string, PublishedPolygon> & latest,
                        const std::vector<std::string> & costmap_topics, const std::vector<MonitorZone> & zones,
                        const Polygon & profile, const Pose2 & robot, double since, double padding);

// The same with explicit zone polygons (one per zone, in base_footprint): a dynamic footprint's
// zones are offset_outward(footprint, margin). The overload above builds padded rectangles.
bool footprints_applied(const std::map<std::string, PublishedPolygon> & latest,
                        const std::vector<std::string> & costmap_topics, const std::vector<MonitorZone> & zones,
                        const Polygon & footprint, const std::vector<Polygon> & zone_polygons, const Pose2 & robot,
                        double since, double padding);

// When the costmaps are ready to plan with a new footprint. A costmap publishes its footprint
// at the end of each update cycle but takes a new one between cycles, so the first message
// showing it can close a cycle whose inflation still used the old footprint; the global
// costmap (2 Hz) then plans on the old inflation for up to 0.5 s, which filled the 1.05 m
// gate and bent the plans. A second matching message, received after the first had arrived
// on every topic, closes a cycle that ran wholly on the new footprint.
class FootprintRefresh
{
public:
  explicit FootprintRefresh(double switched) : since_(switched) {}
  // Messages must be received at or after this time (wall seconds) to count.
  double since() const { return since_; }
  // Feed footprints_applied(..., since(), ...) at `now`; true once the cycle has completed.
  bool observe(bool applied, double now);
  bool shown() const { return shown_; }
  bool refreshed() const { return refreshed_; }

private:
  double since_;
  bool shown_ = false, refreshed_ = false;
};
}  // namespace mobile_manipulator_navigation
