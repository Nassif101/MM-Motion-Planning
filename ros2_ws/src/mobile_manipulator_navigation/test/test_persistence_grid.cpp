#include <gtest/gtest.h>
#include <cmath>

#include "mobile_manipulator_navigation/persistence_grid.hpp"

using mobile_manipulator_navigation::PersistenceGrid;

namespace
{
// 10 Hz lidar seeing `cell` from t0 to t1, with every `every`-th scan hitting it.
void observe(PersistenceGrid & grid, unsigned cell, double t0, double t1, int every = 1)
{
  const int scans = static_cast<int>(std::lround((t1 - t0) / 0.1));
  for (int scan = 0; scan <= scans; ++scan) {
    if (scan % every == 0) grid.observe(cell, t0 + scan * 0.1);  // no accumulated rounding
  }
}
}  // namespace

TEST(PersistenceGrid, StationaryObstacleIsConfirmedAfterThePersistenceTime)
{
  PersistenceGrid grid;
  observe(grid, 7, 0.0, 1.9);
  EXPECT_FALSE(grid.obstacle(7, 1.9));
  observe(grid, 7, 2.0, 2.0);
  EXPECT_TRUE(grid.obstacle(7, 2.0));
}

TEST(PersistenceGrid, SparseHitsWithinTheGapToleranceStillCount)
{
  PersistenceGrid grid;
  observe(grid, 7, 0.0, 3.0, 8);  // one hit every 0.8 s
  EXPECT_TRUE(grid.obstacle(7, 3.0));
}

TEST(PersistenceGrid, WalkingWorkerIsNeverConfirmed)
{
  // A 0.5 m worker at 0.8 m/s crossing 5 m: each 0.05 m cell along its path is inside the
  // worker for 0.625 s, then never again.
  PersistenceGrid grid;
  for (unsigned cell = 0; cell < 100; ++cell) {
    const double enter = cell * 0.05 / 0.8;
    observe(grid, cell, enter, enter + 0.6);
  }
  EXPECT_TRUE(grid.obstacles(7.0).empty());
  EXPECT_TRUE(grid.prune(8.0).empty());
  EXPECT_EQ(grid.tracked(), 0u);
}

TEST(PersistenceGrid, AGapLongerThanTheToleranceRestartsTheCount)
{
  PersistenceGrid grid;
  observe(grid, 7, 0.0, 1.5);
  observe(grid, 7, 3.0, 4.5);  // 1.5 s gap
  EXPECT_FALSE(grid.obstacle(7, 4.5));
  observe(grid, 7, 4.6, 5.0);
  EXPECT_TRUE(grid.obstacle(7, 5.0));
}

TEST(PersistenceGrid, ConfirmedObstacleDecaysAfterItIsNoLongerSeen)
{
  PersistenceGrid grid;
  observe(grid, 7, 0.0, 3.0);
  ASSERT_TRUE(grid.obstacle(7, 3.0));
  EXPECT_TRUE(grid.obstacle(7, 12.9));   // gaps after confirmation do not matter until decay
  EXPECT_TRUE(grid.prune(12.9).empty());
  EXPECT_FALSE(grid.obstacle(7, 13.1));
  EXPECT_EQ(grid.prune(13.1), std::vector<unsigned>{7});
  EXPECT_EQ(grid.tracked(), 0u);
}
