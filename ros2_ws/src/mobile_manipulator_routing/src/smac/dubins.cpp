// Software License Agreement (BSD License). Copyright (c) 2010, Rice University. All rights reserved.
// See third_party/NOTICE.md for the full licence. Author of the original: Mark Moll.
//
// Ported from OMPL 1.7.0 (eabb4d0f) src/ompl/base/spaces/src/DubinsStateSpace.cpp. The anonymous namespace below is
// the upstream text unchanged except for the pi constants (boost's double values, written as M_PI). The state-space
// methods follow upstream with ompl::base::State replaced by DubinsStateSpace::State.
#include "mobile_manipulator_routing/smac/dubins.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

using mobile_manipulator_routing::smac::DubinsStateSpace;

namespace
{
    constexpr double twopi = 2. * M_PI;
    constexpr double onepi = M_PI;
    constexpr double halfpi = M_PI / 2.;
    const double DUBINS_EPS = 1e-6;
    const double DUBINS_ZERO = -1e-7;

    enum DubinsClass
    {
        A11 = 0,
        A12 = 1,
        A13 = 2,
        A14 = 3,
        A21 = 4,
        A22 = 5,
        A23 = 6,
        A24 = 7,
        A31 = 8,
        A32 = 9,
        A33 = 10,
        A34 = 11,
        A41 = 12,
        A42 = 13,
        A43 = 14,
        A44 = 15
    };

    inline double mod2pi(double x)
    {
        if (x < 0 && x > DUBINS_ZERO)
            return 0;
        double xm = x - twopi * floor(x / twopi);
        if (twopi - xm < .5 * DUBINS_EPS)
            xm = 0.;
        return xm;
    }

    inline double t_lsr(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb));
        const double p = sqrtf(std::max(tmp, 0.0));
        const double theta = atan2f(-ca - cb, d + sa + sb) - atan2f(-2.0, p);
        return mod2pi(-alpha + theta);  // t
    }

    inline double p_lsr(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb));
        return sqrtf(std::max(tmp, 0.0));  // p
    }

    inline double q_lsr(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb));
        const double p = sqrtf(std::max(tmp, 0.0));
        const double theta = atan2f(-ca - cb, d + sa + sb) - atan2f(-2.0, p);
        return mod2pi(-beta + theta);  // q
    }

    inline double t_rsl(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb));
        const double p = sqrtf(std::max(tmp, 0.0));
        const double theta = atan2f(ca + cb, d - sa - sb) - atan2f(2.0, p);
        return mod2pi(alpha - theta);  // t
    }

    inline double p_rsl(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb));
        return sqrtf(std::max(tmp, 0.0));  // p
    }

    inline double q_rsl(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb));
        const double p = sqrtf(std::max(tmp, 0.));
        const double theta = atan2f(ca + cb, d - sa - sb) - atan2f(2.0, p);
        return mod2pi(beta - theta);  // q
    }

    inline double t_rsr(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double theta = atan2f(ca - cb, d - sa + sb);
        return mod2pi(alpha - theta);  // t
    }

    inline double p_rsr(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sb - sa));
        return sqrtf(std::max(tmp, 0.0));  // p
    }

    inline double q_rsr(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double theta = atan2f(ca - cb, d - sa + sb);
        return mod2pi(-beta + theta);  // q
    }

    inline double t_lsl(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double theta = atan2f(cb - ca, d + sa - sb);
        return mod2pi(-alpha + theta);  // t
    }

    inline double p_lsl(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sa - sb));
        return sqrtf(std::max(tmp, 0.0));  // p
    }

    inline double q_lsl(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double theta = atan2f(cb - ca, d + sa - sb);
        return mod2pi(beta - theta);  // q
    }

    inline double s_12(double d, double alpha, double beta)
    {
        return p_rsr(d, alpha, beta) - p_rsl(d, alpha, beta) - 2.0 * (q_rsl(d, alpha, beta) - onepi);
    }

    inline double s_13(double d, double alpha, double beta)
    {  // t_rsr - pi
        return t_rsr(d, alpha, beta) - onepi;
    }

    inline double s_14_1(double d, double alpha, double beta)
    {
        return t_rsr(d, alpha, beta) - onepi;
    }

    inline double s_21(double d, double alpha, double beta)
    {
        return p_lsl(d, alpha, beta) - p_rsl(d, alpha, beta) - 2.0 * (t_rsl(d, alpha, beta) - onepi);
    }

    inline double s_22_1(double d, double alpha, double beta)
    {
        return p_lsl(d, alpha, beta) - p_rsl(d, alpha, beta) - 2.0 * (t_rsl(d, alpha, beta) - onepi);
    }

    inline double s_22_2(double d, double alpha, double beta)
    {
        return p_rsr(d, alpha, beta) - p_rsl(d, alpha, beta) - 2.0 * (q_rsl(d, alpha, beta) - onepi);
    }

    inline double s_24(double d, double alpha, double beta)
    {
        return q_rsr(d, alpha, beta) - onepi;
    }

    inline double s_31(double d, double alpha, double beta)
    {
        return q_lsl(d, alpha, beta) - onepi;
    }

    inline double s_33_1(double d, double alpha, double beta)
    {
        return p_rsr(d, alpha, beta) - p_lsr(d, alpha, beta) - 2.0 * (t_lsr(d, alpha, beta) - onepi);
    }

    inline double s_33_2(double d, double alpha, double beta)
    {
        return p_lsl(d, alpha, beta) - p_lsr(d, alpha, beta) - 2.0 * (q_lsr(d, alpha, beta) - onepi);
    }

    inline double s_34(double d, double alpha, double beta)
    {
        return p_rsr(d, alpha, beta) - p_lsr(d, alpha, beta) - 2.0 * (t_lsr(d, alpha, beta) - onepi);
    }

    inline double s_41_1(double d, double alpha, double beta)
    {
        return t_lsl(d, alpha, beta) - onepi;
    }

    inline double s_41_2(double d, double alpha, double beta)
    {
        return q_lsl(d, alpha, beta) - onepi;
    }

    inline double s_42(double d, double alpha, double beta)
    {
        return t_lsl(d, alpha, beta) - onepi;
    }

    inline double s_43(double d, double alpha, double beta)
    {
        return p_lsl(d, alpha, beta) - p_lsr(d, alpha, beta) - 2.0 * (q_lsr(d, alpha, beta) - onepi);
    }

    DubinsStateSpace::DubinsPath dubinsLSL(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sa - sb));
        if (tmp >= DUBINS_ZERO)
        {
            double theta = atan2(cb - ca, d + sa - sb);
            double t = mod2pi(-alpha + theta);
            double p = sqrt(std::max(tmp, 0.));
            double q = mod2pi(beta - theta);
            assert(fabs(p * cos(alpha + t) - sa + sb - d) < (1 + p) * DUBINS_EPS);
            assert(fabs(p * sin(alpha + t) + ca - cb) < (1 + p) * DUBINS_EPS);
            assert(mod2pi(alpha + t + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(&DubinsStateSpace::dubinsPathType[0], t, p, q);
        }
        return {};
    }

    DubinsStateSpace::DubinsPath dubinsRSR(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sb - sa));
        if (tmp >= DUBINS_ZERO)
        {
            double theta = atan2(ca - cb, d - sa + sb);
            double t = mod2pi(alpha - theta);
            double p = sqrt(std::max(tmp, 0.));
            double q = mod2pi(-beta + theta);
            assert(fabs(p * cos(alpha - t) + sa - sb - d) < (1 + p) * DUBINS_EPS);
            assert(fabs(p * sin(alpha - t) - ca + cb) < (1 + p) * DUBINS_EPS);
            assert(mod2pi(alpha - t - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(&DubinsStateSpace::dubinsPathType[1], t, p, q);
        }
        return {};
    }

    DubinsStateSpace::DubinsPath dubinsRSL(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = d * d - 2. + 2. * (ca * cb + sa * sb - d * (sa + sb));
        if (tmp >= DUBINS_ZERO)
        {
            double p = sqrt(std::max(tmp, 0.));
            double theta = atan2(ca + cb, d - sa - sb) - atan2(2., p);
            double t = mod2pi(alpha - theta);
            double q = mod2pi(beta - theta);
            assert(fabs(p * cos(alpha - t) - 2. * sin(alpha - t) + sa + sb - d) < 2 * DUBINS_EPS);
            assert(fabs(p * sin(alpha - t) + 2. * cos(alpha - t) - ca - cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha - t + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(&DubinsStateSpace::dubinsPathType[2], t, p, q);
        }
        return {};
    }

    DubinsStateSpace::DubinsPath dubinsLSR(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = -2. + d * d + 2. * (ca * cb + sa * sb + d * (sa + sb));
        if (tmp >= DUBINS_ZERO)
        {
            double p = sqrt(std::max(tmp, 0.));
            double theta = atan2(-ca - cb, d + sa + sb) - atan2(-2., p);
            double t = mod2pi(-alpha + theta);
            double q = mod2pi(-beta + theta);
            assert(fabs(p * cos(alpha + t) + 2. * sin(alpha + t) - sa - sb - d) < 2 * DUBINS_EPS);
            assert(fabs(p * sin(alpha + t) - 2. * cos(alpha + t) + ca + cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha + t - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(&DubinsStateSpace::dubinsPathType[3], t, p, q);
        }
        return {};
    }

    DubinsStateSpace::DubinsPath dubinsRLR(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb + d * (sa - sb)));
        if (fabs(tmp) < 1.)
        {
            double p = twopi - acos(tmp);
            double theta = atan2(ca - cb, d - sa + sb);
            double t = mod2pi(alpha - theta + .5 * p);
            double q = mod2pi(alpha - beta - t + p);
            assert(fabs(2. * sin(alpha - t + p) - 2. * sin(alpha - t) - d + sa - sb) < 2 * DUBINS_EPS);
            assert(fabs(-2. * cos(alpha - t + p) + 2. * cos(alpha - t) - ca + cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha - t + p - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(&DubinsStateSpace::dubinsPathType[4], t, p, q);
        }
        return {};
    }

    DubinsStateSpace::DubinsPath dubinsLRL(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb - d * (sa - sb)));
        if (fabs(tmp) < 1.)
        {
            double p = twopi - acos(tmp);
            double theta = atan2(-ca + cb, d + sa - sb);
            double t = mod2pi(-alpha + theta + .5 * p);
            double q = mod2pi(beta - alpha - t + p);
            assert(fabs(-2. * sin(alpha + t - p) + 2. * sin(alpha + t) - d - sa + sb) < 2 * DUBINS_EPS);
            assert(fabs(2. * cos(alpha + t - p) - 2. * cos(alpha + t) + ca - cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha + t - p + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(&DubinsStateSpace::dubinsPathType[5], t, p, q);
        }
        return {};
    }

    bool isLongPath(double d, double alpha, double beta)
    {
        return (std::abs(std::sin(alpha)) + std::abs(std::sin(beta)) +
                std::sqrt(4 - std::pow(std::cos(alpha) + std::cos(beta), 2)) - d) < 0;
    }

    DubinsStateSpace::DubinsPath dubinsExhaustive(const double d, const double alpha, const double beta)
    {
        if (d < DUBINS_EPS && fabs(alpha - beta) < DUBINS_EPS)
            return {&DubinsStateSpace::dubinsPathType[0], 0, d, 0};

        DubinsStateSpace::DubinsPath path(dubinsLSL(d, alpha, beta)), tmp(dubinsRSR(d, alpha, beta));
        double len, minLength = path.length();

        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsRSL(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsLSR(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsRLR(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsLRL(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
            path = tmp;
        return path;
    }

    DubinsClass getDubinsClass(const double alpha, const double beta)
    {
        int row(0), column(0);
        if (0 <= alpha && alpha <= halfpi)
        {
            row = 1;
        }
        else if (halfpi < alpha && alpha <= onepi)
        {
            row = 2;
        }
        else if (onepi < alpha && alpha <= 3 * halfpi)
        {
            row = 3;
        }
        else if (3 * halfpi < alpha && alpha <= twopi)
        {
            row = 4;
        }

        if (0 <= beta && beta <= halfpi)
        {
            column = 1;
        }
        else if (halfpi < beta && beta <= onepi)
        {
            column = 2;
        }
        else if (onepi < beta && beta <= 3 * halfpi)
        {
            column = 3;
        }
        else if (3 * halfpi < beta && beta <= 2.0 * onepi)
        {
            column = 4;
        }

        assert(row >= 1 && row <= 4 &&
               "alpha is not in the range of [0,2pi] in classifyPath(double alpha, double beta).");
        assert(column >= 1 && column <= 4 &&
               "beta is not in the range of [0,2pi] in classifyPath(double alpha, double beta).");
        assert((column - 1) + 4 * (row - 1) >= 0 && (column - 1) + 4 * (row - 1) <= 15 &&
               "class is not in range [0,15].");
        return (DubinsClass)((column - 1) + 4 * (row - 1));
    }

    DubinsStateSpace::DubinsPath dubinsClassification(const double d, const double alpha, const double beta)
    {
        if (d < DUBINS_EPS && fabs(alpha - beta) < DUBINS_EPS)
            return {&DubinsStateSpace::dubinsPathType[0], 0, d, 0};
        // Dubins set classification scheme
        // Shkel, Andrei M., and Vladimir Lumelsky. "Classification of the Dubins set."
        //   Robotics and Autonomous Systems 34.4 (2001): 179-202.
        // Lim, Jaeyoung, et al. "Circling Back: Dubins set Classification Revisited."
        //   Workshop on Energy Efficient Aerial Robotic Systems, International Conference on Robotics and Automation
        //   2023.
        DubinsStateSpace::DubinsPath path;
        auto dubins_class = getDubinsClass(alpha, beta);
        switch (dubins_class)
        {
            case DubinsClass::A11:
            {
                path = dubinsRSL(d, alpha, beta);
                break;
            }
            case DubinsClass::A12:
            {
                if (s_13(d, alpha, beta) < 0.0)
                {
                    path = (s_12(d, alpha, beta) < 0.0) ? dubinsRSR(d, alpha, beta) : dubinsRSL(d, alpha, beta);
                }
                else
                {
                    path = dubinsLSR(d, alpha, beta);
                    DubinsStateSpace::DubinsPath tmp = dubinsRSL(d, alpha, beta);
                    if (path.length() > tmp.length())
                    {
                        path = tmp;
                    }
                }
                break;
            }
            case DubinsClass::A13:
            {
                if (s_13(d, alpha, beta) < 0.0)
                {
                    path = dubinsRSR(d, alpha, beta);
                }
                else
                {
                    path = dubinsLSR(d, alpha, beta);
                }
                break;
            }
            case DubinsClass::A14:
            {
                if (s_14_1(d, alpha, beta) > 0.0)
                {
                    path = dubinsLSR(d, alpha, beta);
                }
                else if (s_24(d, alpha, beta) > 0.0)
                {
                    path = dubinsRSL(d, alpha, beta);
                }
                else
                {
                    path = dubinsRSR(d, alpha, beta);
                }
                break;
            }
            case DubinsClass::A21:
            {
                if (s_31(d, alpha, beta) < 0.0)
                {
                    if (s_21(d, alpha, beta) < 0.0)
                    {
                        path = dubinsLSL(d, alpha, beta);
                    }
                    else
                    {
                        path = dubinsRSL(d, alpha, beta);
                    }
                }
                else
                {
                    path = dubinsLSR(d, alpha, beta);
                    DubinsStateSpace::DubinsPath tmp = dubinsRSL(d, alpha, beta);
                    if (path.length() > tmp.length())
                    {
                        path = tmp;
                    }
                }
                break;
            }
            case DubinsClass::A22:
            {
                if (alpha > beta)
                {
                    path = (s_22_1(d, alpha, beta) < 0.0) ? dubinsLSL(d, alpha, beta) : dubinsRSL(d, alpha, beta);
                }
                else
                {
                    path = (s_22_2(d, alpha, beta) < 0.0) ? dubinsRSR(d, alpha, beta) : dubinsRSL(d, alpha, beta);
                }
                break;
            }
            case DubinsClass::A23:
            {
                path = dubinsRSR(d, alpha, beta);
                break;
            }
            case DubinsClass::A24:
            {
                if (s_24(d, alpha, beta) < 0.0)
                {
                    path = dubinsRSR(d, alpha, beta);
                }
                else
                {
                    path = dubinsRSL(d, alpha, beta);
                }
                break;
            }
            case DubinsClass::A31:
            {
                if (s_31(d, alpha, beta) < 0.0)
                {
                    path = dubinsLSL(d, alpha, beta);
                }
                else
                {
                    path = dubinsLSR(d, alpha, beta);
                }
                break;
            }
            case DubinsClass::A32:
            {
                path = dubinsLSL(d, alpha, beta);
                break;
            }
            case DubinsClass::A33:
            {
                if (alpha < beta)
                {
                    if (s_33_1(d, alpha, beta) < 0.0)
                    {
                        path = dubinsRSR(d, alpha, beta);
                    }
                    else
                    {
                        path = dubinsLSR(d, alpha, beta);
                    }
                }
                else
                {
                    if (s_33_2(d, alpha, beta) < 0.0)
                    {
                        path = dubinsLSL(d, alpha, beta);
                    }
                    else
                    {
                        path = dubinsLSR(d, alpha, beta);
                    }
                }
                break;
            }
            case DubinsClass::A34:
            {
                if (s_24(d, alpha, beta) < 0.0)
                {
                    if (s_34(d, alpha, beta) < 0.0)
                    {
                        path = dubinsRSR(d, alpha, beta);
                    }
                    else
                    {
                        path = dubinsLSR(d, alpha, beta);
                    }
                }
                else
                {
                    path = dubinsLSR(d, alpha, beta);
                    DubinsStateSpace::DubinsPath tmp = dubinsRSL(d, alpha, beta);
                    if (path.length() > tmp.length())
                    {
                        path = tmp;
                    }
                }
                break;
            }
            case DubinsClass::A41:
            {
                if (s_41_1(d, alpha, beta) > 0.0)
                {
                    path = dubinsRSL(d, alpha, beta);
                }
                else if (s_41_2(d, alpha, beta) > 0.0)
                {
                    path = dubinsLSR(d, alpha, beta);
                }
                else
                {
                    path = dubinsLSL(d, alpha, beta);
                }
                break;
            }
            case DubinsClass::A42:
            {
                if (s_42(d, alpha, beta) < 0.0)
                {
                    path = dubinsLSL(d, alpha, beta);
                }
                else
                {
                    path = dubinsRSL(d, alpha, beta);
                }
                break;
            }
            case DubinsClass::A43:
            {
                if (s_42(d, alpha, beta) < 0.0)
                {
                    if (s_43(d, alpha, beta) < 0.0)
                    {
                        path = dubinsLSL(d, alpha, beta);
                    }
                    else
                    {
                        path = dubinsLSR(d, alpha, beta);
                    }
                }
                else
                {
                    path = dubinsLSR(d, alpha, beta);
                    DubinsStateSpace::DubinsPath tmp = dubinsRSL(d, alpha, beta);
                    if (path.length() > tmp.length())
                    {
                        path = tmp;
                    }
                }
                break;
            }
            case DubinsClass::A44:
            {
                path = dubinsLSR(d, alpha, beta);
                break;
            }
        }
        return path;
    }

}  // namespace

namespace
{
DubinsStateSpace::DubinsPath dubins(double d, double alpha, double beta)
{
    if (d < DUBINS_EPS && fabs(alpha - beta) < DUBINS_EPS)
        return {&DubinsStateSpace::dubinsPathType[0], 0, d, 0};
    alpha = mod2pi(alpha);
    beta = mod2pi(beta);
    return isLongPath(d, alpha, beta) ? ::dubinsClassification(d, alpha, beta) : ::dubinsExhaustive(d, alpha, beta);
}

}  // namespace

namespace mobile_manipulator_routing::smac
{
const std::vector<std::vector<DubinsStateSpace::DubinsPathSegmentType> > DubinsStateSpace::dubinsPathType {{
    {{DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_LEFT}},
    {{DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_RIGHT}},
    {{DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_LEFT}},
    {{DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_RIGHT}},
    {{DUBINS_RIGHT, DUBINS_LEFT, DUBINS_RIGHT}},
    {{DUBINS_LEFT, DUBINS_RIGHT, DUBINS_LEFT}}
  }};

double DubinsStateSpace::distance(const State & state1, const State & state2) const
{
    if (isSymmetric_)
        return rho_ * std::min(dubins(state1, state2, rho_).length(), dubins(state2, state1, rho_).length());
    return rho_ * dubins(state1, state2, rho_).length();
}

void DubinsStateSpace::interpolate(const State & from, const State & to, const double t, State & state) const
{
    if (t >= 1.)
    {
        state = to;
        return;
    }
    if (t <= 0.)
    {
        state = from;
        return;
    }
    DubinsPath path = dubins(from, to);
    if (isSymmetric_)
    {
        DubinsPath path2(dubins(to, from));
        if (path2.length() < path.length())
        {
            path2.reverse_ = true;
            path = path2;
        }
    }
    interpolate(from, path, t, state, rho_);
}

DubinsStateSpace::DubinsPath DubinsStateSpace::dubins(const State & state1, const State & state2) const
{
    return dubins(state1, state2, rho_);
}

DubinsStateSpace::DubinsPath DubinsStateSpace::dubins(const State & s1, const State & s2, double radius)
{
    double x1 = s1.x, y1 = s1.y, th1 = s1.yaw;
    double x2 = s2.x, y2 = s2.y, th2 = s2.yaw;
    double dx = x2 - x1, dy = y2 - y1, d = sqrt(dx * dx + dy * dy) / radius, th = atan2(dy, dx);
    double alpha = mod2pi(th1 - th), beta = mod2pi(th2 - th);
    return ::dubins(d, alpha, beta);
}
}  // namespace mobile_manipulator_routing::smac
