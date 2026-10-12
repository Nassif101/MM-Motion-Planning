#pragma once
// Panel references anchored on the route, and posture targets. The route-anchored references
// serve only the labelled "scripted" control row on the three B3 missions; the primary B5
// result is unscripted and uses only the posture targets.
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "mobile_manipulator_geometry/footprint_projection.hpp"
#include "mobile_manipulator_mpc/route.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace mobile_manipulator_mpc
{
using Json = mobile_manipulator_navigation::Json;

constexpr double kDefaultRampM = 1.5;

// A panel pose (base_footprint -> panel centre) wanted from start_s to end_s along the route.
struct PanelReference
{
  Eigen::Isometry3d panel_in_base;
  double start_s, end_s, ramp_m;
};

// 0 before start_s - ramp_m, linear to 1 at start_s, 1 until end_s, linear to 0 at end_s + ramp_m.
double reference_weight(const PanelReference & r, double s);

// A reference given by map-frame anchor points instead of arc length.
struct AnchoredReference
{
  Eigen::Isometry3d panel_in_base;
  Eigen::Vector2d start, end;
};

// One reference per reconfigure step of a mission scenario. It starts at the waypoint of the
// navigate step before the reconfigure (the scenario start when none) and stays active until
// the waypoint before the next reconfigure; the last one ends at the scenario goal. The pose is
// the step's panel_pose (xyz, URDF rpy) or the panel pose of its named_state from
// qualified_payload.json. Throws mobile_manipulator_navigation::InvalidScenario for a malformed
// mission and std::invalid_argument for an unknown named_state.
std::vector<AnchoredReference> mission_references(
  const Json & scenario, const mobile_manipulator_geometry::FootprintProjector & projector,
  const Json & qualified_payload);

// Anchors projected onto the route (the end at or beyond its start). Throws
// std::invalid_argument when an anchor is farther than 0.5 m from the route.
std::vector<PanelReference> anchor_on_route(const std::vector<AnchoredReference> & refs, const Route & route,
                                            double ramp_m = kDefaultRampM);

// anchor_on_route for a route that may start part-way through the mission (a path replanned
// from the robot's position): an anchor that projects onto the route start from farther than
// 0.5 m lies behind it (passed). A reference whose end anchor is passed is over and dropped;
// one whose start anchor is passed is active from s = 0 (full weight there). Other anchors as
// anchor_on_route (std::invalid_argument when farther than 0.5 m from the route).
std::vector<PanelReference> anchor_on_remaining_route(const std::vector<AnchoredReference> & refs,
                                                      const Route & route, double ramp_m = kDefaultRampM);

struct PostureTargets
{
  Eigen::Matrix<double, 6, 1> home, tight;  // tight: vertical_carry
};

// `home` and `vertical_carry` of poses_rad, in joint_order.
PostureTargets posture_targets(const Json & qualified_payload);
}  // namespace mobile_manipulator_mpc
