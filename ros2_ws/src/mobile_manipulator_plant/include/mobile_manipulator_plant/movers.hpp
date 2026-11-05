#pragma once
// A scenario mover's motion, ported from Unity's ScenarioMoverMotion.cs (MuJoCo plant spec Section 3.1): the mover waits
// at its start until the robot comes within the trigger distance of the segment's midpoint, then walks the segment at
// constant speed for the given number of crossings (alternating direction) and stays at the last end. A blocked mover
// holds its position; the held time does not count as walking. Time is the plant's.
#include <Eigen/Core>

#include <optional>

#include "mobile_manipulator_plant/types.hpp"

namespace mobile_manipulator_plant
{
// Unity ScenarioMover.StandOff: a mover waits while the robot is within this distance of its next box (m).
constexpr double kMoverStandOffM = 0.1;

class MoverMotion
{
public:
  // Throws std::out_of_range for non-finite end points, speed outside (0, 2] m/s, trigger outside (0, 20] m,
  // crossings outside 1..10 or a segment shorter than 0.1 m (the C# class's ranges).
  explicit MoverMotion(const MoverSpec & spec);

  // The position the mover would take at `t` given the robot's base position; commit() moves there, hold() keeps it.
  Eigen::Vector2d next(double t, const Eigen::Vector2d & robot);
  void commit(double t, const Eigen::Vector2d & p);
  void hold(double t);

  // The position after walking `distance` along the alternating crossings.
  Eigen::Vector2d position_at(double distance) const;

  const MoverSpec & spec() const { return spec_; }
  const Eigen::Vector2d & position() const { return position_; }
  std::optional<double> trigger_time() const { return trigger_time_; }
  double walked() const { return walked_; }
  double blocked_s() const { return blocked_s_; }
  bool finished() const { return walked_ >= spec_.crossings * length_; }

private:
  MoverSpec spec_;
  double length_;
  Eigen::Vector2d position_;
  std::optional<double> trigger_time_, previous_time_;
  double walked_ = 0.0, blocked_s_ = 0.0;
};
}  // namespace mobile_manipulator_plant
