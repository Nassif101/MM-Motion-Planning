#include "mobile_manipulator_mpc/world.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace mobile_manipulator_mpc
{
Bounds bounds_of(const ObstacleBox & box)
{
  // The support of the box along each map axis, as the collision step's bounding boxes.
  const Eigen::Vector3d half = box.size / 2.0;
  Eigen::Vector3d e;
  for (int i = 0; i < 3; ++i) e(i) = (box.pose.linear().transpose() * Eigen::Vector3d::Unit(i)).cwiseAbs().dot(half);
  return {box.pose.translation() - e, box.pose.translation() + e};
}

YawedBox yawed_box(const Eigen::Vector3d & center, const Eigen::Quaterniond & orientation, const Eigen::Vector3d & size)
{
  const Eigen::Matrix3d r = orientation.normalized().toRotationMatrix();
  // The tilt is the angle between the box's own z axis and the map's.
  const double tilt = std::acos(std::clamp(r(2, 2), -1.0, 1.0));
  if (tilt < kUprightTiltRad) return {center, std::atan2(r(1, 0), r(0, 0)), size};
  return {center, 0.0, r.cwiseAbs() * size};
}

void World::set_static(const std::vector<mobile_manipulator_navigation::Box> & boxes)
{
  static_.clear();
  static_.reserve(boxes.size());
  static_bounds_.clear();
  static_bounds_.reserve(boxes.size());
  for (const auto & box : boxes) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = box.center;
    pose.linear() = Eigen::AngleAxisd(box.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    static_.push_back({box.name, pose, box.size});
    static_bounds_.push_back(bounds_of(static_.back()));
  }
}

void World::update_mover(const std::string & name, const Eigen::Vector2d & position, const Eigen::Vector3d & size,
                         double stamp)
{
  update_box(name, Eigen::Vector3d(position.x(), position.y(), size.z() / 2.0), 0.0, size, stamp);
}

void World::update_box(const std::string & name, const Eigen::Vector3d & center, double yaw,
                       const Eigen::Vector3d & size, double stamp)
{
  auto [it, inserted] = movers_.try_emplace(name, Mover{center, Eigen::Vector3d::Zero(), yaw, size, stamp});
  Mover & mover = it->second;
  if (!inserted) {
    const double elapsed = stamp - mover.stamp;
    const double jump = (center - mover.center).norm();
    if (elapsed > kMoverStaleS || jump > kMoverJumpM) {
      mover.velocity.setZero();
    } else if (elapsed > 0.0) {
      mover.velocity = (center - mover.center) / elapsed;
    }
    mover.center = center;
    mover.yaw = yaw;
    mover.size = size;
    mover.stamp = std::max(stamp, mover.stamp);
  }
  // Forget movers that went stale relative to this sample.
  for (auto other = movers_.begin(); other != movers_.end();) {
    if (stamp - other->second.stamp > kMoverStaleS) other = movers_.erase(other);
    else ++other;
  }
}

std::vector<ObstacleBox> World::at(double t_ahead, double now) const
{
  std::vector<ObstacleBox> boxes(static_);
  std::vector<ObstacleBox> movers = movers_at(t_ahead, now);
  boxes.insert(boxes.end(), std::make_move_iterator(movers.begin()), std::make_move_iterator(movers.end()));
  return boxes;
}

std::vector<ObstacleBox> World::movers_at(double t_ahead, double now) const
{
  std::vector<ObstacleBox> boxes;
  boxes.reserve(movers_.size());
  for (const auto & [name, mover] : movers_) {
    if (now - mover.stamp > kMoverStaleS) continue;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = mover.center + mover.velocity * (now + t_ahead - mover.stamp);
    if (mover.yaw != 0.0) pose.linear() = Eigen::AngleAxisd(mover.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    boxes.push_back({name, pose, mover.size});
  }
  return boxes;
}
}  // namespace mobile_manipulator_mpc
