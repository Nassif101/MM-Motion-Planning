#pragma once
// Point selection for the Livox robot filter (no ROS dependencies).
//
// Self-returns are removed with a geometric self-model: the URDF collision primitives and
// the rigidly attached payload panel, each posed from TF. Because the simulated lidar's
// noise is along the ray (Gaussian range noise), a point is a self-return when its ray
// from the sensor first hits the robot and the point lies no more than `noise_band` in
// front of that surface. Points inside a primitive enlarged by `margin` are also removed.
// Obstacles between the sensor and the robot, or off the robot's rays, are kept even
// inside the footprint rectangle, so safety consumers still see them.
#include <Eigen/Geometry>
#include <optional>
#include <string>
#include <vector>

namespace mobile_manipulator_navigation
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

struct PosedPrimitive
{
  Eigen::Isometry3d pose;     // base -> primitive
  Shape shape;
  Eigen::Vector3d dims;
  Eigen::Isometry3d inverse;  // primitive -> base, computed once per scan
  double bound;               // bounding-sphere radius
};

// A primitive posed in the base frame, with its inverse and bounding radius precomputed.
PosedPrimitive posed_primitive(const Eigen::Isometry3d & pose, Shape shape, const Eigen::Vector3d & dims);

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

// True for points (base frame) inside any posed primitive enlarged by margin.
bool inside_any(const Eigen::Vector3d & point, const std::vector<PosedPrimitive> & posed, double margin);

// Distance along a unit ray to the nearest posed primitive (infinity if none).
double first_self_hit(const Eigen::Vector3d & origin, const Eigen::Vector3d & direction,
                      const std::vector<PosedPrimitive> & posed);

// Whether to keep a point given in the sensor frame: not a UnitySensors miss (0, 0, 0) and
// not a robot self-return. Ground and overhead returns are kept: consumers apply their own
// marking heights, and ground rays are what clear low obstacles near the robot.
bool keep_point(const Eigen::Vector3d & point_sensor, const Eigen::Isometry3d & sensor_to_base,
                const std::vector<PosedPrimitive> & posed, double margin, double noise_band);
}  // namespace mobile_manipulator_navigation
