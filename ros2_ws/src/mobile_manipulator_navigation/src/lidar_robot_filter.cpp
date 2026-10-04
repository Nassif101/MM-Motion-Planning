#include "mobile_manipulator_navigation/lidar_robot_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mobile_manipulator_navigation
{
namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();

// Entry distance of a ray (primitive frame) into a box of half extents, or infinity.
double ray_box(const Eigen::Vector3d & o, const Eigen::Vector3d & d, const Eigen::Vector3d & half)
{
  double t_near = -kInf, t_far = kInf;
  for (int axis = 0; axis < 3; ++axis) {
    if (d[axis] == 0.0) {
      if (std::abs(o[axis]) > half[axis]) return kInf;  // parallel and outside this slab
      continue;
    }
    const double t1 = (-half[axis] - o[axis]) / d[axis];
    const double t2 = (half[axis] - o[axis]) / d[axis];
    t_near = std::max(t_near, std::min(t1, t2));
    t_far = std::min(t_far, std::max(t1, t2));
  }
  if (t_near > t_far || t_far <= 0.0) return kInf;
  return t_near > 0.0 ? t_near : 0.0;
}

// Entry distance of a ray (primitive frame) into a z-axis cylinder, or infinity.
double ray_cylinder(const Eigen::Vector3d & o, const Eigen::Vector3d & d, double radius,
                    double half_length)
{
  double best = kInf;
  const double a = d.x() * d.x() + d.y() * d.y();
  const double b = 2.0 * (o.x() * d.x() + o.y() * d.y());
  const double c = o.x() * o.x() + o.y() * o.y() - radius * radius;
  const double disc = b * b - 4.0 * a * c;
  if (a > 0.0 && disc >= 0.0) {
    const double root = std::sqrt(disc);
    for (const double side : {(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)}) {
      if (side > 0.0 && std::abs(o.z() + side * d.z()) <= half_length) best = std::min(best, side);
    }
  }
  if (d.z() != 0.0) {
    for (const double cap : {-half_length, half_length}) {
      const double side = (cap - o.z()) / d.z();
      const double x = o.x() + side * d.x(), y = o.y() + side * d.y();
      if (side > 0.0 && x * x + y * y <= radius * radius) best = std::min(best, side);
    }
  }
  return best;
}
}  // namespace

PosedPrimitive posed_primitive(const Eigen::Isometry3d & pose, Shape shape, const Eigen::Vector3d & dims)
{
  const double bound = shape == Shape::Box ? dims.norm() : std::hypot(dims.x(), dims.y());
  return {pose, shape, dims, pose.inverse(), bound};
}

bool inside_any(const Eigen::Vector3d & point, const std::vector<PosedPrimitive> & posed, double margin)
{
  for (const auto & primitive : posed) {
    const Eigen::Vector3d local = primitive.inverse * point;
    if (primitive.shape == Shape::Box) {
      if ((local.cwiseAbs().array() <= primitive.dims.array() + margin).all()) return true;
    } else if (std::hypot(local.x(), local.y()) <= primitive.dims.x() + margin &&
               std::abs(local.z()) <= primitive.dims.y() + margin) {
      return true;
    }
  }
  return false;
}

double first_self_hit(const Eigen::Vector3d & origin, const Eigen::Vector3d & direction,
                      const std::vector<PosedPrimitive> & posed)
{
  double nearest = kInf;
  for (const auto & primitive : posed) {
    // Cheap cull: only rays through the primitive's bounding sphere are intersected.
    const double radius = primitive.bound;
    const Eigen::Vector3d offset = primitive.pose.translation() - origin;
    const double along = direction.dot(offset);
    if (along + radius <= 0.0 || offset.squaredNorm() - along * along > radius * radius) continue;
    const Eigen::Vector3d o = primitive.inverse * origin;
    const Eigen::Vector3d d = primitive.inverse.linear() * direction;
    const double t = primitive.shape == Shape::Box
      ? ray_box(o, d, primitive.dims)
      : ray_cylinder(o, d, primitive.dims.x(), primitive.dims.y());
    nearest = std::min(nearest, t);
  }
  return nearest;
}

bool keep_point(const Eigen::Vector3d & point_sensor, const Eigen::Isometry3d & sensor_to_base,
                const std::vector<PosedPrimitive> & posed, double margin, double noise_band)
{
  if (point_sensor.isZero(0.0)) return false;  // UnitySensors miss
  const Eigen::Vector3d base = sensor_to_base * point_sensor;
  if (inside_any(base, posed, margin)) return false;
  if (noise_band > 0.0) {
    const double range = point_sensor.norm();
    const Eigen::Vector3d origin = sensor_to_base.translation();
    const double surface = first_self_hit(origin, (base - origin) / range, posed);
    if (range >= surface - noise_band) return false;
  }
  return true;
}
}  // namespace mobile_manipulator_navigation
