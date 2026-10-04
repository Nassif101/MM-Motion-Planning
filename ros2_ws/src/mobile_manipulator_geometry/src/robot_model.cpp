#include "mobile_manipulator_geometry/robot_model.hpp"

#include <stdexcept>

#include <urdf/model.h>

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
}  // namespace

std::vector<Primitive> load_primitives(const std::string & urdf_text,
                                       const std::optional<Payload> & payload)
{
  urdf::Model model;
  if (!model.initString(urdf_text)) throw std::invalid_argument("Could not parse the URDF");
  std::vector<Primitive> primitives;
  for (const auto & [name, link] : model.links_) {
    for (const auto & collision : link->collision_array) {
      const auto pose = to_isometry(collision->origin);
      const auto & geometry = collision->geometry;
      if (geometry->type == urdf::Geometry::BOX) {
        const auto & box = static_cast<const urdf::Box &>(*geometry);
        primitives.push_back({name, pose, Shape::Box,
                              Eigen::Vector3d(box.dim.x, box.dim.y, box.dim.z) / 2.0});
      } else if (geometry->type == urdf::Geometry::CYLINDER) {
        const auto & cylinder = static_cast<const urdf::Cylinder &>(*geometry);
        primitives.push_back({name, pose, Shape::Cylinder,
                              Eigen::Vector3d(cylinder.radius, cylinder.length / 2.0, 0.0)});
      } else {
        throw std::invalid_argument("Unsupported collision geometry on link " + name);
      }
    }
  }
  if (payload) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = payload->center;
    primitives.push_back({payload->link, pose, Shape::Box, payload->size / 2.0});
  }
  return primitives;
}
}  // namespace mobile_manipulator_geometry
