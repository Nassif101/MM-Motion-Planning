#include "mobile_manipulator_geometry/footprint_projection.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mobile_manipulator_geometry
{
namespace
{
Eigen::Isometry3d to_isometry(const urdf::Pose & pose)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  result.linear() = Eigen::Quaterniond(pose.rotation.w, pose.rotation.x, pose.rotation.y,
                                       pose.rotation.z).toRotationMatrix();
  return result;
}

Eigen::Vector3d vector3(const nlohmann::ordered_json & values)
{
  return {values.at(0).get<double>(), values.at(1).get<double>(), values.at(2).get<double>()};
}

}  // namespace

Payload payload_from_json(const nlohmann::ordered_json & qualified_payload)
{
  const auto & payload = qualified_payload.at("payload");
  return {"tool0", vector3(payload.at("dimensions_tool_ros_m")), vector3(payload.at("com_tool_ros_m"))};
}

FootprintProjector::FootprintProjector(const std::string & urdf_text, const Payload & payload)
: payload_(payload), primitives_(load_primitives(urdf_text, payload))
{
  if (!model_.initString(urdf_text)) throw std::invalid_argument("Could not parse the URDF");
}

Eigen::Isometry3d FootprintProjector::link_pose(const std::string & link, const JointMap & joints) const
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  auto current = model_.getLink(link);
  if (!current) throw std::invalid_argument("Unknown link " + link);
  while (current->name != "base_footprint") {
    const auto & joint = current->parent_joint;
    if (!joint) throw std::invalid_argument("Link " + link + " is not below base_footprint");
    Eigen::Isometry3d step = to_isometry(joint->parent_to_joint_origin_transform);
    if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS) {
      const auto found = joints.find(joint->name);
      const double angle = found == joints.end() ? 0.0 : found->second;
      const Eigen::Vector3d axis(joint->axis.x, joint->axis.y, joint->axis.z);
      step.rotate(Eigen::AngleAxisd(angle, axis.normalized()));
    }
    pose = step * pose;
    current = model_.getLink(joint->parent_link_name);
  }
  return pose;
}

Eigen::Isometry3d FootprintProjector::panel_pose(const JointMap & joints) const
{
  return link_pose(payload_.link, joints) * Eigen::Translation3d(payload_.center);
}

std::vector<Point2> FootprintProjector::projected_points(const JointMap & joints) const
{
  return projected_points(joints, ProjectionOptions{});
}

std::vector<Point2> FootprintProjector::projected_points(const JointMap & joints,
                                                         const ProjectionOptions & options) const
{
  if (options.cylinder_sides < 3) throw std::invalid_argument("cylinder_sides must be at least 3");
  std::map<std::string, Eigen::Isometry3d> links;
  std::vector<Point2> points;
  for (size_t i = 0; i < primitives_.size(); ++i) {
    const auto & primitive = primitives_[i];
    // load_primitives appends the payload box last.
    const bool is_payload = i + 1 == primitives_.size();
    if (is_payload ? !options.payload : !options.links.empty() && !options.links.count(primitive.link)) continue;
    auto found = links.find(primitive.link);
    if (found == links.end()) found = links.emplace(primitive.link, link_pose(primitive.link, joints)).first;
    const Eigen::Isometry3d frame = found->second * primitive.pose;
    for (const auto & local : primitive_points(primitive, options.cylinder_sides)) {
      const Eigen::Vector3d point = frame * local;
      points.push_back({point.x(), point.y()});
    }
  }
  return points;
}

std::array<double, 3> rpy_of(const Eigen::Matrix3d & r)
{
  constexpr double kLock = 1.0 - 1e-9;
  if (r(2, 0) <= -kLock) return {std::atan2(r(0, 1), r(0, 2)), M_PI / 2.0, 0.0};
  if (r(2, 0) >= kLock) return {std::atan2(-r(0, 1), -r(0, 2)), -M_PI / 2.0, 0.0};
  return {std::atan2(r(2, 1), r(2, 2)), std::asin(-r(2, 0)), std::atan2(r(1, 0), r(0, 0))};
}

Containment contains(const Polygon & convex_polygon, const std::vector<Point2> & points)
{
  if (convex_polygon.size() < 3) throw std::invalid_argument("Footprint polygon needs at least 3 vertices");
  double twice_area = 0.0;
  for (size_t i = 0; i < convex_polygon.size(); ++i) {
    const auto & a = convex_polygon[i];
    const auto & b = convex_polygon[(i + 1) % convex_polygon.size()];
    twice_area += a[0] * b[1] - b[0] * a[1];
  }
  const double winding = twice_area > 0.0 ? 1.0 : -1.0;  // +1 counter-clockwise
  double margin = std::numeric_limits<double>::infinity();
  for (const auto & p : points) {
    for (size_t i = 0; i < convex_polygon.size(); ++i) {
      const auto & a = convex_polygon[i];
      const auto & b = convex_polygon[(i + 1) % convex_polygon.size()];
      const double ex = b[0] - a[0], ey = b[1] - a[1];
      // Left of a counter-clockwise edge is inside.
      const double distance = winding * (ex * (p[1] - a[1]) - ey * (p[0] - a[0])) / std::hypot(ex, ey);
      margin = std::min(margin, distance);
    }
  }
  return {margin >= 0.0, margin};
}
}  // namespace mobile_manipulator_geometry
