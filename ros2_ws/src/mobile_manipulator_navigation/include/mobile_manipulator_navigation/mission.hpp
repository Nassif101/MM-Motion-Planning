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
  std::string polygon_topic;  // dynamic_polygon_topic: zone input when dynamic_monitor_zones:=true
  std::string visual_topic;   // polygon_pub_topic: the zone the monitor is using
};

// The margin-sized zones, in the order the monitor lists them.
std::vector<MonitorZone> monitor_zones(const Json & nav2_navigation);

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

// Whether a costmap's published (padded) footprint is `expected` grown by `padding` the way
// Nav2 pads a footprint (each vertex moved outward by `padding` in x and y), in any vertex
// order, within `tolerance`.
bool footprint_matches(const Polygon & published, const Polygon & expected, double padding,
                       double tolerance = 0.005);

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
}  // namespace mobile_manipulator_navigation
