#include "mobile_manipulator_mpc/references.hpp"

#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include "mobile_manipulator_mpc/model.hpp"
#include "mobile_manipulator_navigation/mission.hpp"

namespace mobile_manipulator_mpc
{
namespace mmg = mobile_manipulator_geometry;
namespace mmn = mobile_manipulator_navigation;

namespace
{
constexpr double kMaxAnchorDistanceM = 0.5;

Eigen::Matrix<double, 6, 1> pose_vector(const Json & payload, const std::string & name)
{
  if (!payload.at("poses_rad").contains(name)) {
    throw std::invalid_argument("qualified_payload.json has no pose '" + name + "'");
  }
  const auto & values = payload.at("poses_rad").at(name);
  Eigen::Matrix<double, 6, 1> q;
  for (int i = 0; i < 6; ++i) q(i) = values.at(i).get<double>();
  return q;
}

Eigen::Isometry3d panel_of(const Json & reconfigure, const mmg::FootprintProjector & projector, const Json & payload)
{
  if (reconfigure.contains("named_state")) {
    const std::string name = reconfigure.at("named_state").get<std::string>();
    const auto q = pose_vector(payload, name);
    mmg::JointMap joints;
    for (int i = 0; i < 6; ++i) joints[payload.at("joint_order").at(i).get<std::string>()] = q(i);
    return projector.panel_pose(joints);
  }
  if (!reconfigure.contains("panel_pose")) {
    throw mmn::InvalidScenario("reconfigure needs named_state or panel_pose: " + reconfigure.dump());
  }
  const auto & pose = reconfigure.at("panel_pose");
  const auto & xyz = pose.at("xyz");
  const auto & rpy = pose.at("rpy");
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = Eigen::Vector3d(xyz.at(0).get<double>(), xyz.at(1).get<double>(), xyz.at(2).get<double>());
  // URDF roll-pitch-yaw: R = Rz(yaw) Ry(pitch) Rx(roll).
  out.linear() = (Eigen::AngleAxisd(rpy.at(2).get<double>(), Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(rpy.at(1).get<double>(), Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(rpy.at(0).get<double>(), Eigen::Vector3d::UnitX()))
                   .toRotationMatrix();
  return out;
}
}  // namespace

double reference_weight(const PanelReference & r, double s)
{
  if (s >= r.start_s && s <= r.end_s) return 1.0;
  if (r.ramp_m <= 0.0) return 0.0;
  if (s < r.start_s) return std::clamp((s - (r.start_s - r.ramp_m)) / r.ramp_m, 0.0, 1.0);
  return std::clamp(1.0 - (s - r.end_s) / r.ramp_m, 0.0, 1.0);
}

std::vector<AnchoredReference> mission_references(const Json & scenario, const mmg::FootprintProjector & projector,
                                                  const Json & qualified_payload)
{
  const auto steps = mmn::parse_mission(scenario);
  const auto start = mmn::pose_of(scenario.at("start"));
  const auto goal = mmn::pose_of(scenario.at("goal"));
  Eigen::Vector2d waypoint(start[0], start[1]);
  std::vector<AnchoredReference> refs;
  for (const auto & step : steps) {
    if (step.kind == mmn::MissionStep::Kind::Navigate) {
      waypoint = Eigen::Vector2d(step.pose[0], step.pose[1]);
      continue;
    }
    if (!refs.empty()) refs.back().end = waypoint;
    refs.push_back({panel_of(step.reconfigure, projector, qualified_payload), waypoint, Eigen::Vector2d(goal[0], goal[1])});
  }
  return refs;
}

std::vector<PanelReference> anchor_on_route(const std::vector<AnchoredReference> & refs, const Route & route,
                                            double ramp_m)
{
  const auto check = [&](const Eigen::Vector2d & anchor, const Route::Projection & p) {
    if ((anchor - p.point).norm() > kMaxAnchorDistanceM) {
      throw std::invalid_argument("anchor (" + std::to_string(anchor.x()) + ", " + std::to_string(anchor.y()) +
                                  ") is " + std::to_string((anchor - p.point).norm()) + " m from the route");
    }
  };
  std::vector<PanelReference> out;
  for (const auto & ref : refs) {
    const auto start = route.project_range(ref.start, 0.0, route.length());
    check(ref.start, start);
    const auto end = route.project_range(ref.end, start.s, route.length());
    check(ref.end, end);
    out.push_back({ref.panel_in_base, start.s, end.s, ramp_m});
  }
  return out;
}

std::vector<PanelReference> anchor_on_remaining_route(const std::vector<AnchoredReference> & refs,
                                                      const Route & route, double ramp_m)
{
  const auto passed = [&](const Eigen::Vector2d & anchor) {
    const auto p = route.project_range(anchor, 0.0, route.length());
    return p.s <= 0.0 && (anchor - p.point).norm() > kMaxAnchorDistanceM;
  };
  std::vector<PanelReference> out;
  for (const auto & ref : refs) {
    if (passed(ref.end)) continue;
    if (!passed(ref.start)) {
      out.push_back(anchor_on_route({ref}, route, ramp_m).front());
      continue;
    }
    // Active from the route start: anchor the end alone.
    const auto end = anchor_on_route({{ref.panel_in_base, ref.end, ref.end}}, route, ramp_m).front();
    out.push_back({ref.panel_in_base, 0.0, end.end_s, ramp_m});
  }
  return out;
}

void require_model_joint_order(const Json & qualified_payload)
{
  const auto order = qualified_payload.at("joint_order").get<std::vector<std::string>>();
  bool same = order.size() == kArmJoints.size();
  for (size_t i = 0; same && i < order.size(); ++i) same = order[i] == kArmJoints[i];
  if (same) return;
  std::string model, payload;
  for (const char * name : kArmJoints) model += std::string(model.empty() ? "" : ", ") + name;
  for (const auto & name : order) payload += (payload.empty() ? "" : ", ") + name;
  throw std::runtime_error("qualified_payload.json joint_order [" + payload +
                           "] is not the whole-body model's joint order [" + model + "] (regenerate the model)");
}

PostureMode parse_posture_mode(const std::string & name)
{
  if (name == "blend") return PostureMode::Blend;
  if (name == "comfort_only") return PostureMode::ComfortOnly;
  if (name == "none") return PostureMode::None;
  throw std::invalid_argument("posture_mode must be blend, comfort_only or none, not '" + name + "'");
}

const char * to_string(PostureMode mode)
{
  switch (mode) {
    case PostureMode::Blend:
      return "blend";
    case PostureMode::ComfortOnly:
      return "comfort_only";
    case PostureMode::None:
      return "none";
  }
  return "unknown";
}

PostureTargets posture_targets(const Json & qualified_payload, const std::string & comfort_pose, PostureMode mode)
{
  require_model_joint_order(qualified_payload);
  if (std::find_if(std::begin(kComfortPoses), std::end(kComfortPoses),
                   [&](const char * name) { return comfort_pose == name; }) == std::end(kComfortPoses)) {
    throw std::invalid_argument("comfort_pose must be home or carry, not '" + comfort_pose + "'");
  }
  const auto & poses = qualified_payload.at("poses_rad");
  if (!poses.contains(comfort_pose)) {
    std::string known;
    for (const auto & item : poses.items()) known += (known.empty() ? "" : ", ") + item.key();
    throw std::invalid_argument("comfort_pose '" + comfort_pose + "' is not in qualified_payload.json poses_rad (" +
                                known + "): qualify the '" + comfort_pose + "' pose and add it there first");
  }
  return {pose_vector(qualified_payload, comfort_pose), pose_vector(qualified_payload, "vertical_carry"), mode};
}
}  // namespace mobile_manipulator_mpc
