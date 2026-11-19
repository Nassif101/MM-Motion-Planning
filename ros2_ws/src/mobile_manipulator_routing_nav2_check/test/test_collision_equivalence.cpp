// S2 (spec 2 Amendment 1): the port's GridCollisionChecker and FootprintCollisionChecker give the same collision
// outcomes and costs as nav2_smac_planner's and nav2_costmap_2d's at seeded random poses on the V2 costmap.
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "common.hpp"
#include "mobile_manipulator_routing/smac/collision_checker.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sm = mobile_manipulator_routing::smac;

namespace
{
sm::Footprint irregular()
{
  sm::Footprint f;
  for (int i = 0; i < 16; ++i) {
    const double a = 2.0 * M_PI * i / 16.0 + 0.07;
    const double rx = std::cos(a) > 0 ? 0.58 : 0.71, ry = std::sin(a) > 0 ? 0.43 : 0.47;
    f.push_back({rx * std::cos(a) + 0.03, ry * std::sin(a)});
  }
  return f;
}
}  // namespace

TEST(CollisionEquivalence, GridCheckerMatchesSmacAtRandomPoses)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("collision_equivalence");
  for (const auto & footprint : {check::home_footprint(), irregular()}) {
    const auto layered = check::v2_costmap(footprint);
    const auto nav2_map = check::to_nav2(layered.costmap);
    const double possible = sm::findCircumscribedCost(layered);
    ASSERT_GT(possible, 0.0);

    sm::GridCollisionChecker port(&layered.costmap, 72u);
    port.setFootprint(layered.padded_footprint, false, possible);
    nav2_smac_planner::GridCollisionChecker smac(nullptr, 72u, node);
    smac.setCostmap(nav2_map.get());
    smac.setFootprint(check::to_msg(layered.padded_footprint), false, possible);

    std::mt19937 rng(7);
    const float sx = layered.costmap.getSizeInCellsX(), sy = layered.costmap.getSizeInCellsY();
    // Upstream reads the cell at (x + 0.5, y + 0.5) after a range check against the size itself, so coordinates in
    // (size - 0.5, size] read outside the map. The samples stay below that band.
    std::uniform_real_distribution<float> ux(-2.0f, sx - 0.6f), uy(-2.0f, sy - 0.6f), ua(0.0f, 71.999f);
    std::size_t collisions = 0, checked_footprints = 0;
    for (int i = 0; i < 200000; ++i) {
      const float x = ux(rng), y = uy(rng), a = ua(rng);
      for (const bool unknown : {false, true}) {
        const bool p = port.inCollision(x, y, a, unknown);
        const bool s = smac.inCollision(x, y, a, unknown);
        ASSERT_EQ(p, s) << "x " << x << " y " << y << " bin " << a;
        if (x >= 0 && y >= 0) {
          ASSERT_EQ(port.getCost(), smac.getCost()) << "x " << x << " y " << y;
          if (port.getCost() >= possible && port.getCost() < sm::INSCRIBED) ++checked_footprints;
        }
        collisions += p;
      }
    }
    EXPECT_GT(collisions, 1000u);
    EXPECT_GT(checked_footprints, 1000u);  // the full-footprint branch is exercised
  }
}

TEST(CollisionEquivalence, FootprintCostMatchesNav2AtRandomPoses)
{
  const auto layered = check::v2_costmap(irregular());
  const auto nav2_map = check::to_nav2(layered.costmap);
  sm::FootprintCollisionChecker port(&layered.costmap);
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> nav2(nav2_map.get());
  const auto msg = check::to_msg(layered.padded_footprint);
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> ux(-26.0, 26.0), uy(-21.0, 21.0), ua(-M_PI, M_PI);
  for (int i = 0; i < 100000; ++i) {
    const double x = ux(rng), y = uy(rng), a = ua(rng);
    ASSERT_EQ(port.footprintCostAtPose(x, y, a, layered.padded_footprint), nav2.footprintCostAtPose(x, y, a, msg))
      << x << " " << y << " " << a;
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
