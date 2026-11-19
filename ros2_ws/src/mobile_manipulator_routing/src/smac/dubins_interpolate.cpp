// Software License Agreement (BSD License). Copyright (c) 2010, Rice University. All rights reserved.
// See third_party/NOTICE.md for the full licence. Author of the original: Mark Moll.
//
// Ported from OMPL 1.7.0 (eabb4d0f) src/ompl/base/spaces/src/DubinsStateSpace.cpp: interpolation along a Dubins path,
// and SO2StateSpace::enforceBounds. This file is compiled without floating-point contraction: the installed OMPL
// binary (ros-jazzy-ompl 1.7.0-2noble, aarch64) fuses only the two final multiply-adds of this function, which are
// written here as std::fma. The rest of the Dubins code keeps the compiler's default, as OMPL's build does.
#include <cmath>

#include "mobile_manipulator_routing/smac/dubins.hpp"

namespace
{
// ompl::base::SO2StateSpace::enforceBounds
double enforce_bounds(double value)
{
    double v = fmod(value, 2.0 * M_PI);
    if (v < -M_PI)
        v += 2.0 * M_PI;
    else if (v >= M_PI)
        v -= 2.0 * M_PI;
    return v;
}
}  // namespace

namespace mobile_manipulator_routing::smac
{
void DubinsStateSpace::interpolate(const State & from, const DubinsPath & path, double t, State & state,
                                   double radius) const
{
    State s;
    double seg = t * path.length(), phi, v;

    s.x = 0.;
    s.y = 0.;
    s.yaw = from.yaw;
    if (!path.reverse_)
    {
        for (unsigned int i = 0; i < 3 && seg > 0; ++i)
        {
            v = std::min(seg, path.length_[i]);
            phi = s.yaw;
            seg -= v;
            switch (path.type_->at(i))
            {
                case DUBINS_LEFT:
                    s.x = s.x + sin(phi + v) - sin(phi); s.y = s.y - cos(phi + v) + cos(phi);
                    s.yaw = phi + v;
                    break;
                case DUBINS_RIGHT:
                    s.x = s.x - sin(phi - v) + sin(phi); s.y = s.y + cos(phi - v) - cos(phi);
                    s.yaw = phi - v;
                    break;
                case DUBINS_STRAIGHT:
                    s.x = s.x + v * cos(phi); s.y = s.y + v * sin(phi);
                    break;
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < 3 && seg > 0; ++i)
        {
            v = std::min(seg, path.length_[2 - i]);
            phi = s.yaw;
            seg -= v;
            switch (path.type_->at(2 - i))
            {
                case DUBINS_LEFT:
                    s.x = s.x + sin(phi - v) - sin(phi); s.y = s.y - cos(phi - v) + cos(phi);
                    s.yaw = phi - v;
                    break;
                case DUBINS_RIGHT:
                    s.x = s.x - sin(phi + v) + sin(phi); s.y = s.y + cos(phi + v) - cos(phi);
                    s.yaw = phi + v;
                    break;
                case DUBINS_STRAIGHT:
                    s.x = s.x - v * cos(phi); s.y = s.y - v * sin(phi);
                    break;
            }
        }
    }
    // The installed OMPL binary (aarch64) fuses these two into fmadd and no other operation of this function
    // (objdump of ros-jazzy-ompl 1.7.0-2noble); the file is compiled without contraction and fuses them explicitly.
    state.x = std::fma(s.x, radius, from.x);
    state.y = std::fma(s.y, radius, from.y);
    state.yaw = enforce_bounds(s.yaw);
}

}  // namespace mobile_manipulator_routing::smac
