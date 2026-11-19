#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>
#include "mobile_manipulator_routing/types.hpp"

namespace mobile_manipulator_routing
{
enum class Cell : std::uint8_t { Free = 0, Occupied = 1, Unknown = 2 };
struct CellIndex { int x, y; };
struct InflationParameters { double radius_m = 1.0, scaling = 5.0; };

// Immutable after construction. Row zero is at the map origin, not the first PGM row.
class GridSnapshot
{
public:
  GridSnapshot(int width, int height, double resolution, Point2 origin, std::vector<Cell> known,
               std::vector<Cell> observed = {}, InflationParameters inflation = {}, std::uint64_t revision = 1);
  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  Point2 origin() const { return origin_; }
  std::uint64_t revision() const { return revision_; }
  bool contains(int x, int y) const;
  std::optional<CellIndex> world_to_cell(Point2 point) const;
  Point2 cell_center(int x, int y) const;
  Cell occupancy(int x, int y) const;
  bool blocked(int x, int y) const { return occupancy(x,y) != Cell::Free; }
  double cost(int x, int y) const;
  std::size_t storage_bytes() const;
  const std::vector<Cell> & known() const { return known_; }
  const std::vector<Cell> & observed() const { return observed_; }
private:
  int width_, height_;
  double resolution_;
  Point2 origin_;
  std::uint64_t revision_;
  std::vector<Cell> known_, observed_;
  std::vector<float> costs_;
};
}  // namespace mobile_manipulator_routing
