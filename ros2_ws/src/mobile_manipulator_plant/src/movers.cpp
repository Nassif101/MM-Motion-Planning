#include "mobile_manipulator_plant/movers.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mobile_manipulator_plant
{
MoverMotion::MoverMotion(const MoverSpec & spec) : spec_(spec), length_((spec.end - spec.start).norm()), position_(spec.start)
{
  if (!spec.start.allFinite() || !spec.end.allFinite()) throw std::out_of_range(spec.name + ": mover end points must be finite");
  if (!(spec.speed_mps > 0.0 && spec.speed_mps <= 2.0)) throw std::out_of_range(spec.name + ": mover speed must be in (0, 2] m/s");
  if (!(spec.trigger_distance_m > 0.0 && spec.trigger_distance_m <= 20.0))
    throw std::out_of_range(spec.name + ": trigger distance must be in (0, 20] m");
  if (spec.crossings < 1 || spec.crossings > 10) throw std::out_of_range(spec.name + ": crossings must be 1 to 10");
  if (!(length_ >= 0.1)) throw std::out_of_range(spec.name + ": mover segment must be at least 0.1 m long");
}

Eigen::Vector2d MoverMotion::next(double t, const Eigen::Vector2d & robot)
{
  if (!trigger_time_) {
    if ((robot - 0.5 * (spec_.start + spec_.end)).norm() > spec_.trigger_distance_m) {
      previous_time_ = t;
      return position_;
    }
    trigger_time_ = t;
    previous_time_ = t;
  }
  const double step = previous_time_ ? std::max(0.0, t - *previous_time_) : 0.0;
  return position_at(std::min(walked_ + spec_.speed_mps * step, spec_.crossings * length_));
}

void MoverMotion::commit(double t, const Eigen::Vector2d & p)
{
  if (trigger_time_ && previous_time_)
    walked_ = std::min(walked_ + spec_.speed_mps * std::max(0.0, t - *previous_time_), spec_.crossings * length_);
  previous_time_ = t;
  position_ = p;
}

void MoverMotion::hold(double t)
{
  if (trigger_time_ && previous_time_ && !finished()) blocked_s_ += std::max(0.0, t - *previous_time_);
  previous_time_ = t;
}

Eigen::Vector2d MoverMotion::position_at(double distance) const
{
  const double clamped = std::clamp(distance, 0.0, spec_.crossings * length_);
  const int leg = std::min(static_cast<int>(std::floor(clamped / length_)), spec_.crossings - 1);
  double along = (clamped - leg * length_) / length_;
  if (leg % 2 == 1) along = 1.0 - along;  // return leg: end -> start
  return spec_.start + along * (spec_.end - spec_.start);
}
}  // namespace mobile_manipulator_plant
