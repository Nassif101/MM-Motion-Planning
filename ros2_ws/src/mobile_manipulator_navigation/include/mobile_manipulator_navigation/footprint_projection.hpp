#pragma once
// Ground-plane projection of the robot and attached panel for a joint state, and
// containment in a footprint polygon (no ROS graph).
//
// The geometry is the lidar self-filter's model: every URDF collision primitive plus the
// rigid payload box. Boxes contribute their 8 corners and cylinders a circumscribed 64-gon
// at both ends, as in test_footprint_profiles.py, so the projected bounds of a qualified
// pose reproduce the generated footprint profiles.
#include <array>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <urdf/model.h>

#include "mobile_manipulator_navigation/lidar_robot_filter.hpp"
#include "mobile_manipulator_navigation/scenario_spec.hpp"
#include "mobile_manipulator_navigation/yaml_json.hpp"

namespace mobile_manipulator_navigation
{
using JointMap = std::map<std::string, double>;

// The reference panel of qualified_payload.json, rigidly attached to tool0.
Payload payload_from_json(const Json & qualified_payload);

class FootprintProjector
{
public:
  FootprintProjector(const std::string & urdf_text, const Payload & payload);

  // base_footprint -> link; joints not listed are at zero.
  Eigen::Isometry3d link_pose(const std::string & link, const JointMap & joints) const;
  // base_footprint -> panel centre (axes of the payload link).
  Eigen::Isometry3d panel_pose(const JointMap & joints) const;
  // Ground projection (x, y in base_footprint) of points whose hull contains the robot.
  std::vector<std::array<double, 2>> projected_points(const JointMap & joints) const;

private:
  urdf::Model model_;
  Payload payload_;
  std::vector<Primitive> primitives_;
};

struct Containment
{
  bool inside;
  // Minimum over points of the signed distance to the polygon's edge lines, positive
  // inside. For points outside it is the overshoot past the nearest violated edge.
  double margin_m;
};

// URDF roll, pitch, yaw (R = Rz(yaw) Ry(pitch) Rx(roll)). At pitch = +/-90 deg yaw is set
// to zero and the remaining rotation is carried by roll, so the result always rebuilds R.
std::array<double, 3> rpy_of(const Eigen::Matrix3d & rotation);

// Containment of points in a convex polygon given in either winding.
Containment contains(const Polygon & convex_polygon, const std::vector<std::array<double, 2>> & points);
}  // namespace mobile_manipulator_navigation
