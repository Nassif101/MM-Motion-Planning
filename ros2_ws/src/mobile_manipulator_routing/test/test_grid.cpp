#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
#include "mobile_manipulator_routing/grid.hpp"

namespace mr = mobile_manipulator_routing;

TEST(RoutingGrid, ConvertsMetricCoordinatesAndRejectsOutsideBeforeIntegerConversion)
{
  const mr::GridSnapshot map(3, 2, 0.5, {-2.0, -1.0}, std::vector<mr::Cell>(6, mr::Cell::Free));
  ASSERT_TRUE(map.world_to_cell({-2.0, -1.0}));
  EXPECT_EQ(map.world_to_cell({-2.0, -1.0})->x, 0);
  EXPECT_EQ(map.world_to_cell({-1.01, -0.01})->x, 1);
  EXPECT_EQ(map.world_to_cell({-1.01, -0.01})->y, 1);
  const auto center = map.cell_center(2, 1);
  EXPECT_DOUBLE_EQ(center.x, -0.75);
  EXPECT_DOUBLE_EQ(center.y, -0.25);
  EXPECT_FALSE(map.world_to_cell({-0.5, -0.5}));
  EXPECT_FALSE(map.world_to_cell({-1.0, 0.0}));
  EXPECT_FALSE(map.world_to_cell({std::nextafter(-2.0, -3.0), -0.5}));
  EXPECT_FALSE(map.world_to_cell({std::numeric_limits<double>::max(), 0.0}));
  EXPECT_FALSE(map.world_to_cell({std::numeric_limits<double>::quiet_NaN(), 0.0}));
  EXPECT_THROW(map.cell_center(3, 0), std::out_of_range);
  EXPECT_TRUE(map.blocked(-1, 0));
  EXPECT_EQ(map.occupancy(3, 0), mr::Cell::Unknown);
  EXPECT_DOUBLE_EQ(map.cost(-1, 0), 1.0);
}

TEST(RoutingGrid, ObservationsCannotEraseKnownWallsOrKnownUnknownSpace)
{
  const std::vector<mr::Cell> known{mr::Cell::Occupied, mr::Cell::Free, mr::Cell::Unknown, mr::Cell::Free};
  const mr::GridSnapshot before(4, 1, 1.0, {}, known,
    {mr::Cell::Free, mr::Cell::Occupied, mr::Cell::Free, mr::Cell::Unknown}, {0.0, 5.0}, 8);
  EXPECT_EQ(before.revision(), 8u);
  EXPECT_EQ(before.occupancy(0, 0), mr::Cell::Occupied);
  EXPECT_EQ(before.occupancy(1, 0), mr::Cell::Occupied);
  EXPECT_EQ(before.occupancy(2, 0), mr::Cell::Unknown);
  EXPECT_EQ(before.occupancy(3, 0), mr::Cell::Unknown);
  const mr::GridSnapshot after(4, 1, 1.0, {}, known, {}, {0.0, 5.0}, 9);
  EXPECT_EQ(after.occupancy(0, 0), mr::Cell::Occupied);
  EXPECT_EQ(after.occupancy(1, 0), mr::Cell::Free);
  EXPECT_EQ(after.occupancy(2, 0), mr::Cell::Unknown);
  EXPECT_EQ(after.occupancy(3, 0), mr::Cell::Free);
  EXPECT_GT(after.storage_bytes(), 4u * sizeof(mr::Cell));
  EXPECT_EQ(before.occupancy(1, 0), mr::Cell::Occupied);  // the previous snapshot is unchanged
}

TEST(RoutingGrid, InflationMatchesBruteForceEuclideanDistances)
{
  constexpr int width = 9, height = 7;
  constexpr double resolution = 0.2, radius = 0.7, scaling = 2.3;
  std::vector<mr::Cell> cells(width * height, mr::Cell::Free);
  const std::vector<mr::CellIndex> occupied{{1, 1}, {7, 2}, {3, 6}};
  for (const auto & p : occupied) cells[p.y * width + p.x] = mr::Cell::Occupied;
  cells[0] = mr::Cell::Unknown;
  const mr::GridSnapshot map(width, height, resolution, {}, cells, {}, {radius, scaling});
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (cells[y * width + x] != mr::Cell::Free) {
        EXPECT_DOUBLE_EQ(map.cost(x, y), 1.0);
        continue;
      }
      double nearest = std::numeric_limits<double>::infinity();
      for (const auto & obstacle : occupied) {
        nearest = std::min(nearest, std::hypot(x - obstacle.x, y - obstacle.y) * resolution);
      }
      const double expected = nearest <= radius ? std::exp(-scaling * nearest) : 0.0;
      EXPECT_NEAR(map.cost(x, y), expected, 1e-7) << x << ", " << y;
    }
  }
}

TEST(RoutingGrid, InflationUsesObservedObstaclesAndDoesNotMakeFreeCellsLethal)
{
  const mr::GridSnapshot map(5, 1, 0.5, {}, std::vector<mr::Cell>(5, mr::Cell::Free),
    {mr::Cell::Free, mr::Cell::Free, mr::Cell::Occupied, mr::Cell::Free, mr::Cell::Free}, {0.5, 0.0});
  EXPECT_DOUBLE_EQ(map.cost(0, 0), 0.0);
  EXPECT_DOUBLE_EQ(map.cost(1, 0), 1.0);
  EXPECT_FALSE(map.blocked(1, 0));
  EXPECT_TRUE(map.blocked(2, 0));
  EXPECT_DOUBLE_EQ(map.cost(4, 0), 0.0);
  const mr::GridSnapshot empty(4, 3, 0.05, {}, std::vector<mr::Cell>(12, mr::Cell::Free));
  EXPECT_DOUBLE_EQ(empty.cost(0, 0), 0.0);
  EXPECT_DOUBLE_EQ(empty.cost(3, 2), 0.0);
}

TEST(RoutingGrid, RejectsInvalidDimensionsGeometryLayersAndInflation)
{
  const auto free = mr::Cell::Free;
  EXPECT_THROW(mr::GridSnapshot(0, 1, 1.0, {}, {}), std::invalid_argument);
  EXPECT_THROW(mr::GridSnapshot(-1, 1, 1.0, {}, {}), std::invalid_argument);
  EXPECT_THROW(mr::GridSnapshot(65537, 1, 1.0, {}, {}), std::invalid_argument);
  EXPECT_THROW(mr::GridSnapshot(65536, 65536, 1.0, {}, {}), std::invalid_argument);
  EXPECT_THROW(mr::GridSnapshot(1, 1, 0.0, {}, {free}), std::invalid_argument);
  EXPECT_THROW(mr::GridSnapshot(1, 1, std::numeric_limits<double>::infinity(), {}, {free}), std::invalid_argument);
  EXPECT_THROW(mr::GridSnapshot(1, 1, 1.0, {std::numeric_limits<double>::quiet_NaN(), 0.0}, {free}), std::invalid_argument);
  EXPECT_THROW(mr::GridSnapshot(2, 1, std::numeric_limits<double>::max(), {}, {free, free}), std::invalid_argument);
  EXPECT_THROW(mr::GridSnapshot(1, 1, 1.0, {1e300, 0.0}, {free}), std::invalid_argument);
  EXPECT_THROW(mr::GridSnapshot(2, 1, 1.0, {}, {free}), std::invalid_argument);
  EXPECT_THROW(mr::GridSnapshot(1, 1, 1.0, {}, {free}, {free, free}), std::invalid_argument);
  EXPECT_THROW(mr::GridSnapshot(1, 1, 1.0, {}, {static_cast<mr::Cell>(99)}), std::invalid_argument);
  EXPECT_THROW(mr::GridSnapshot(1, 1, 1.0, {}, {free}, {}, {-1.0, 1.0}), std::invalid_argument);
  EXPECT_THROW(mr::GridSnapshot(1, 1, 1.0, {}, {free}, {}, {1.0, -1.0}), std::invalid_argument);
}
