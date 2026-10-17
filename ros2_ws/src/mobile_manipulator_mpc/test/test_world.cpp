#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "mobile_manipulator_mpc/world.hpp"

namespace mmn = mobile_manipulator_navigation;
using namespace mobile_manipulator_mpc;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace
{
const ObstacleBox * find(const std::vector<ObstacleBox> & boxes, const std::string & name)
{
  for (const auto & box : boxes)
    if (box.name == name) return &box;
  return nullptr;
}

World world_with_one_static()
{
  World world;
  world.set_static({mmn::Box{"post", Vector3d(1.0, 2.0, 1.2), Vector3d(0.4, 0.4, 2.4), 0.5}});
  return world;
}
}  // namespace

TEST(World, StaticBoxKeepsCentreSizeAndYaw)
{
  const auto boxes = world_with_one_static().at(1.0, 0.0);
  ASSERT_EQ(boxes.size(), 1u);
  EXPECT_LT((boxes[0].pose.translation() - Vector3d(1.0, 2.0, 1.2)).norm(), 1e-12);
  EXPECT_LT((boxes[0].pose.linear() - Eigen::AngleAxisd(0.5, Vector3d::UnitZ()).toRotationMatrix()).norm(), 1e-12);
  EXPECT_LT((boxes[0].size - Vector3d(0.4, 0.4, 2.4)).norm(), 1e-12);
}

TEST(World, StaticBoundsAreTheYawedBoxesBounds)
{
  // The 0.4 x 0.4 m post turned by 0.5 rad reaches 0.2 (|cos| + |sin|) from its centre in x and y.
  const World world = world_with_one_static();
  ASSERT_EQ(world.statics().size(), 1u);
  ASSERT_EQ(world.static_bounds().size(), 1u);
  const double reach = 0.2 * (std::cos(0.5) + std::sin(0.5));
  EXPECT_LT((world.static_bounds()[0].lo - Vector3d(1.0 - reach, 2.0 - reach, 0.0)).norm(), 1e-12);
  EXPECT_LT((world.static_bounds()[0].hi - Vector3d(1.0 + reach, 2.0 + reach, 2.4)).norm(), 1e-12);
}

TEST(World, AtIsTheStaticsThenTheMovers)
{
  World world = world_with_one_static();
  world.update_mover("worker", Vector2d(0.0, 0.0), Vector3d(0.5, 0.5, 1.8), 0.0);
  world.update_mover("worker", Vector2d(0.05, 0.0), Vector3d(0.5, 0.5, 1.8), 0.1);
  const auto all = world.at(1.0, 0.2);
  const auto movers = world.movers_at(1.0, 0.2);
  ASSERT_EQ(movers.size(), 1u);
  ASSERT_EQ(all.size(), 2u);
  EXPECT_EQ(all[0].name, world.statics()[0].name);
  EXPECT_EQ(all[1].name, movers[0].name);
  EXPECT_EQ(all[1].pose.translation(), movers[0].pose.translation());
  EXPECT_TRUE(world.movers_at(0.0, 2.0).empty());  // stale, as in at()
}

TEST(World, MoverPredictedAtConstantVelocity)
{
  World world = world_with_one_static();
  const Vector3d size(0.5, 0.5, 1.8);
  world.update_mover("worker", Vector2d(0.0, 0.0), size, 10.0);
  world.update_mover("worker", Vector2d(0.05, -0.02), size, 10.1);  // v = (0.5, -0.2)

  const auto boxes = world.at(1.0, 10.2);  // 1.1 s after the last sample
  ASSERT_EQ(boxes.size(), 2u);
  const ObstacleBox * worker = find(boxes, "worker");
  ASSERT_NE(worker, nullptr);
  EXPECT_LT((worker->pose.translation() - Vector3d(0.05 + 0.55, -0.02 - 0.22, 0.9)).norm(), 1e-9);
  EXPECT_LT((worker->pose.linear() - Eigen::Matrix3d::Identity()).norm(), 1e-12);
  EXPECT_LT((worker->size - size).norm(), 1e-12);

  // Prediction starts from the last sample, so at t_ahead = 0 and now = stamp it is the sample.
  EXPECT_LT((find(world.at(0.0, 10.1), "worker")->pose.translation() - Vector3d(0.05, -0.02, 0.9)).norm(), 1e-12);
}

TEST(World, FirstSampleHasZeroVelocity)
{
  World world;
  world.update_mover("worker", Vector2d(3.0, 4.0), Vector3d(0.5, 0.5, 1.8), 1.0);
  EXPECT_LT((world.at(2.0, 1.5).at(0).pose.translation().head<2>() - Vector2d(3.0, 4.0)).norm(), 1e-12);
}

TEST(World, MoverJumpResetsVelocity)
{
  World world;
  const Vector3d size(0.5, 0.5, 1.8);
  world.update_mover("worker", Vector2d(0.0, 0.0), size, 0.0);
  world.update_mover("worker", Vector2d(0.05, 0.0), size, 0.1);
  world.update_mover("worker", Vector2d(2.0, 0.0), size, 0.2);  // teleport: 1.95 m in 0.1 s
  EXPECT_LT((world.at(1.0, 0.2).at(0).pose.translation().head<2>() - Vector2d(2.0, 0.0)).norm(), 1e-12);

  // The next regular sample estimates the velocity again from the last two positions.
  world.update_mover("worker", Vector2d(2.05, 0.0), size, 0.3);
  EXPECT_LT((world.at(1.0, 0.3).at(0).pose.translation().head<2>() - Vector2d(2.55, 0.0)).norm(), 1e-9);
}

TEST(World, StaleMoverDropped)
{
  World world = world_with_one_static();
  world.update_mover("worker", Vector2d(0.0, 0.0), Vector3d(0.5, 0.5, 1.8), 5.0);
  EXPECT_NE(find(world.at(0.0, 5.9), "worker"), nullptr);
  const auto later = world.at(0.0, 6.1);
  EXPECT_EQ(find(later, "worker"), nullptr);
  EXPECT_NE(find(later, "post"), nullptr);  // known boxes never go stale

  // A fresh sample brings it back without a velocity from the stale one.
  world.update_mover("worker", Vector2d(0.3, 0.0), Vector3d(0.5, 0.5, 1.8), 7.0);
  EXPECT_LT((find(world.at(1.0, 7.0), "worker")->pose.translation().head<2>() - Vector2d(0.3, 0.0)).norm(), 1e-12);
}

TEST(World, SampleWithoutTimeProgressKeepsVelocity)
{
  World world;
  const Vector3d size(0.5, 0.5, 1.8);
  world.update_mover("worker", Vector2d(0.0, 0.0), size, 0.0);
  world.update_mover("worker", Vector2d(0.1, 0.0), size, 0.1);  // v = (1, 0)
  world.update_mover("worker", Vector2d(0.1, 0.0), size, 0.1);  // repeated sample
  EXPECT_LT((world.at(1.0, 0.1).at(0).pose.translation().head<2>() - Vector2d(1.1, 0.0)).norm(), 1e-9);
}
