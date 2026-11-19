// Software License Agreement (BSD License). Copyright (c) 2010, Rice University. All rights reserved.
// See third_party/NOTICE.md for the full licence. Author of the original: Mark Moll.
//
// Ported from OMPL 1.7.0 (eabb4d0f): src/ompl/base/spaces/DubinsStateSpace.h and src/DubinsStateSpace.cpp, the
// distance and interpolation used by nav2_smac_planner, plus SO2StateSpace::enforceBounds. The classification code is
// copied unchanged, including its float (sqrtf, atan2f) steps. Spec 2 Amendment 1 / ADR 0019.
#pragma once
#include <limits>
#include <vector>

namespace mobile_manipulator_routing::smac
{
class DubinsStateSpace
{
public:
  enum DubinsPathSegmentType
  {
    DUBINS_LEFT = 0,
    DUBINS_STRAIGHT = 1,
    DUBINS_RIGHT = 2
  };
  static const std::vector<std::vector<DubinsPathSegmentType>> dubinsPathType;
  class DubinsPath
  {
  public:
    DubinsPath(const std::vector<DubinsPathSegmentType> * type = &dubinsPathType[0], double t = 0.,
               double p = std::numeric_limits<double>::max(), double q = 0.)
    : type_(type)
    {
      length_[0] = t;
      length_[1] = p;
      length_[2] = q;
    }
    double length() const { return length_[0] + length_[1] + length_[2]; }
    const std::vector<DubinsPathSegmentType> * type_;
    double length_[3];
    bool reverse_{false};
  };
  // An SE(2) state: x, y and yaw, as ompl::base::SE2StateSpace::StateType's reals.
  struct State { double x = 0.0, y = 0.0, yaw = 0.0; };

  explicit DubinsStateSpace(double turningRadius = 1.0, bool isSymmetric = false)
  : rho_(turningRadius), isSymmetric_(isSymmetric) {}
  double distance(const State & state1, const State & state2) const;
  void interpolate(const State & from, const State & to, double t, State & state) const;
  DubinsPath dubins(const State & state1, const State & state2) const;
  static DubinsPath dubins(const State & state1, const State & state2, double radius);
  double turningRadius() const { return rho_; }

private:
  void interpolate(const State & from, const DubinsPath & path, double t, State & state, double radius) const;
  double rho_;
  bool isSymmetric_;
};
}  // namespace mobile_manipulator_routing::smac
