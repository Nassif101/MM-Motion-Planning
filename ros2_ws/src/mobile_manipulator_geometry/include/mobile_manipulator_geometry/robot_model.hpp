#pragma once
// Collision primitives of the robot (every URDF collision geometry) and the rigidly
// attached payload box; no ROS graph.
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace mobile_manipulator_geometry
{
enum class Shape { Box, Cylinder };

// Box: dims = half extents (x, y, z). Cylinder: dims = (radius, half length, unused) along
// the primitive's local z.
struct Primitive
{
  std::string link;
  Eigen::Isometry3d pose;  // link -> primitive
  Shape shape;
  Eigen::Vector3d dims;
};

// Rigid payload box: full size and centre in `link` coordinates.
struct Payload
{
  std::string link;
  Eigen::Vector3d size;
  Eigen::Vector3d center;
};

// Collision primitives of every link in the URDF, plus the payload box if given.
std::vector<Primitive> load_primitives(const std::string & urdf_text,
                                       const std::optional<Payload> & payload);
}  // namespace mobile_manipulator_geometry
