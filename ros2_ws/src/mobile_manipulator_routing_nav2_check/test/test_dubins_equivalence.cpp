// S3 (spec 2 Amendment 1): the port's Dubins distance and interpolation equal OMPL 1.7.0's DubinsStateSpace exactly,
// at the radii Smac uses (the lattice radius in cells, the analytic-expansion refinements, 0.5 m for the smoother).
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "mobile_manipulator_routing/smac/dubins.hpp"
#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/DubinsStateSpace.h"

namespace sm = mobile_manipulator_routing::smac;

TEST(DubinsEquivalence, DistanceAndInterpolationMatchOmplExactly)
{
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> position(-60.0, 60.0), yaw(0.0, 2.0 * M_PI), unit(0.0, 1.0);
  std::size_t compared = 0;
  for (const double radius : {9.99999985, 10.5, 25.0, 0.5}) {
    auto space = std::make_shared<ompl::base::DubinsStateSpace>(radius);
    const sm::DubinsStateSpace port(radius);
    ompl::base::ScopedState<> from(space), to(space), s(space);
    for (int i = 0; i < 50000; ++i) {
      const double scale = radius < 1.0 ? 0.05 : 1.0;
      sm::DubinsStateSpace::State a{position(rng) * scale, position(rng) * scale, yaw(rng)};
      sm::DubinsStateSpace::State b{position(rng) * scale, position(rng) * scale, yaw(rng)};
      if (i % 10 == 0) b = {a.x + 0.3 * scale, a.y, a.yaw};  // short and degenerate cases
      if (i % 17 == 0) b = a;
      from[0] = a.x; from[1] = a.y; from[2] = a.yaw;
      to[0] = b.x; to[1] = b.y; to[2] = b.yaw;
      ASSERT_EQ(port.distance(a, b), space->distance(from(), to())) << i;
      const double t = (i % 7 == 0) ? 1.0 : unit(rng);
      sm::DubinsStateSpace::State out;
      port.interpolate(a, b, t, out);
      space->interpolate(from(), to(), t, s());
      const auto reals = s.reals();
      ASSERT_EQ(out.x, reals[0]) << i;
      ASSERT_EQ(out.y, reals[1]) << i;
      ASSERT_EQ(out.yaw, reals[2]) << i;
      ++compared;
    }
  }
  EXPECT_EQ(compared, 200000u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
