#pragma once
// Panel references anchored on the route, and posture targets. The route-anchored references
// serve only the labelled "scripted" control row on the three B3 missions; the primary B5
// result is unscripted and uses only the posture targets.
#include <string>
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

// How the posture preference uses its two attractors (spec Section 3, amended 2026-10-13):
//  - Blend (the recorded B5): (1 - alpha)|q - comfort|^2 + alpha|q - tight|^2, alpha the tightness;
//  - ComfortOnly: |q - comfort|^2 alone (no pull towards the tight pose; the tightness is still
//    computed and logged);
//  - None: no posture term (the joint effort and smoothness costs keep the QP well posed).
enum class PostureMode { Blend, ComfortOnly, None };
// "blend", "comfort_only", "none"; throws std::invalid_argument for any other name.
PostureMode parse_posture_mode(const std::string & name);
const char * to_string(PostureMode mode);

// The comfort poses the posture preference may use (comfort_pose in wholebody_mpc.yaml): the
// straight `home` (the recorded B5) or the bent `carry` (to be qualified; poses_rad.carry).
inline constexpr const char * kComfortPoses[] = {"home", "carry"};

struct PostureTargets
{
  // comfort: the comfort pose (home by default), also the pose the tightness probes; tight:
  // vertical_carry.
  Eigen::Matrix<double, 6, 1> comfort, tight;
  PostureMode mode = PostureMode::Blend;
};

// Throws std::runtime_error unless the payload's joint_order is the model's (kArmJoints): the
// state, the posture targets, the limits and the streamed trajectory all use that order.
void require_model_joint_order(const Json & qualified_payload);

// `comfort_pose` (home or carry) and `vertical_carry` of poses_rad, in joint_order (which must be
// the model's: throws std::runtime_error otherwise, see require_model_joint_order). Throws
// std::invalid_argument for a comfort pose other than home or carry, or one that poses_rad does
// not have yet (the message names the missing pose).
PostureTargets posture_targets(const Json & qualified_payload, const std::string & comfort_pose = "home",
                               PostureMode mode = PostureMode::Blend);
}  // namespace mobile_manipulator_mpc
