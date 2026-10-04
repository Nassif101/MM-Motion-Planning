#pragma once
// Ground-plane projection of the robot and attached panel for a joint state (no ROS graph).
//
// The geometry is the lidar self-filter's model: every URDF collision primitive plus the
// rigid payload box. Boxes contribute their 8 corners and cylinders a circumscribed polygon
// at both ends (64 sides by default, as in test_footprint_profiles.py), so the projected
// bounds of a qualified pose reproduce the generated footprint profiles.
#include <array>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <urdf/model.h>

#include "mobile_manipulator_geometry/polygon.hpp"
#include "mobile_manipulator_geometry/robot_model.hpp"

namespace mobile_manipulator_geometry
{
using JointMap = std::map<std::string, double>;

// The reference panel of qualified_payload.json, rigidly attached to tool0.
Payload payload_from_json(const nlohmann::ordered_json & qualified_payload);

struct ProjectionOptions
{
  int cylinder_sides = 64;      // circumscribed polygon per cylinder end
  std::set<std::string> links;  // links to project; empty = all
  bool payload = true;          // include the payload box
};

class FootprintProjector
{
public:
  FootprintProjector(const std::string & urdf_text, const Payload & payload);

  // base_footprint -> link; joints not listed are at zero.
  Eigen::Isometry3d link_pose(const std::string & link, const JointMap & joints) const;
  // base_footprint -> panel centre (axes of the payload link).
  Eigen::Isometry3d panel_pose(const JointMap & joints) const;
  // Ground projection (x, y in base_footprint) of points whose hull contains the robot.
  std::vector<Point2> projected_points(const JointMap & joints) const;
  std::vector<Point2> projected_points(const JointMap & joints, const ProjectionOptions & options) const;

  const std::vector<Primitive> & primitives() const { return primitives_; }
  const Payload & payload() const { return payload_; }
  const urdf::Model & model() const { return model_; }

private:
  urdf::Model model_;
  Payload payload_;
  std::vector<Primitive> primitives_;
};

// URDF roll, pitch, yaw (R = Rz(yaw) Ry(pitch) Rx(roll)). At pitch = +/-90 deg yaw is set
// to zero and the remaining rotation is carried by roll, so the result always rebuilds R.
std::array<double, 3> rpy_of(const Eigen::Matrix3d & rotation);
}  // namespace mobile_manipulator_geometry
