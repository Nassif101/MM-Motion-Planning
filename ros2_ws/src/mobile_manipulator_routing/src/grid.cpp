#include "mobile_manipulator_routing/grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace mobile_manipulator_routing
{
namespace
{
constexpr int kMaxDimension = 65536;
constexpr std::size_t kMaxCells = 16u * 1024u * 1024u;

bool valid_cell(Cell cell)
{
  return cell == Cell::Free || cell == Cell::Occupied || cell == Cell::Unknown;
}

// Lower envelope of parabolas: exact squared Euclidean distance on a 1D grid.
// Infinite source entries are omitted, avoiding infinity-minus-infinity intersections.
void distance_transform(const std::vector<double> & input, std::vector<double> & output, int count,
                        std::vector<int> & sites, std::vector<double> & boundaries)
{
  int last = -1;
  for (int q = 0; q < count; ++q) {
    if (!std::isfinite(input[static_cast<std::size_t>(q)])) continue;
    double crossing = -std::numeric_limits<double>::infinity();
    while (last >= 0) {
      const int p = sites[static_cast<std::size_t>(last)];
      crossing = ((input[static_cast<std::size_t>(q)] + static_cast<double>(q) * q) -
                  (input[static_cast<std::size_t>(p)] + static_cast<double>(p) * p)) /
                 (2.0 * (q - p));
      if (crossing > boundaries[static_cast<std::size_t>(last)]) break;
      --last;
    }
    ++last;
    sites[static_cast<std::size_t>(last)] = q;
    boundaries[static_cast<std::size_t>(last)] = last == 0
      ? -std::numeric_limits<double>::infinity() : crossing;
    boundaries[static_cast<std::size_t>(last + 1)] = std::numeric_limits<double>::infinity();
  }
  if (last < 0) {
    std::fill(output.begin(), output.begin() + count, std::numeric_limits<double>::infinity());
    return;
  }
  int site = 0;
  for (int q = 0; q < count; ++q) {
    while (site < last && boundaries[static_cast<std::size_t>(site + 1)] < q) ++site;
    const int p = sites[static_cast<std::size_t>(site)];
    const double delta = static_cast<double>(q - p);
    output[static_cast<std::size_t>(q)] = delta * delta + input[static_cast<std::size_t>(p)];
  }
}
}  // namespace

GridSnapshot::GridSnapshot(int width, int height, double resolution, Point2 origin,
                           std::vector<Cell> known, std::vector<Cell> observed,
                           InflationParameters inflation, std::uint64_t revision)
: width_(width), height_(height), resolution_(resolution), origin_(origin), revision_(revision),
  known_(std::move(known)), observed_(std::move(observed))
{
  if (width <= 0 || height <= 0 || width > kMaxDimension || height > kMaxDimension ||
      static_cast<std::size_t>(width) > kMaxCells / static_cast<std::size_t>(height)) {
    throw std::invalid_argument("grid dimensions exceed the supported positive bounds");
  }
  const std::size_t cells = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (!std::isfinite(resolution) || resolution <= 0.0 || !std::isfinite(origin.x) ||
      !std::isfinite(origin.y)) {
    throw std::invalid_argument("grid resolution and origin must be finite, with positive resolution");
  }
  const double end_x = origin.x + width * resolution, end_y = origin.y + height * resolution;
  if (!std::isfinite(end_x) || !std::isfinite(end_y) || end_x <= origin.x || end_y <= origin.y) {
    throw std::invalid_argument("grid extent is not finite and representable");
  }
  if (!std::isfinite(inflation.radius_m) || inflation.radius_m < 0.0 ||
      !std::isfinite(inflation.scaling) || inflation.scaling < 0.0) {
    throw std::invalid_argument("inflation radius and scaling must be finite and nonnegative");
  }
  if (known_.size() != cells || (!observed_.empty() && observed_.size() != cells)) {
    throw std::invalid_argument("grid layer size does not match its dimensions");
  }
  if (!std::all_of(known_.begin(), known_.end(), valid_cell) ||
      !std::all_of(observed_.begin(), observed_.end(), valid_cell)) {
    throw std::invalid_argument("grid contains an invalid occupancy value");
  }
  if (observed_.empty()) observed_.assign(cells, Cell::Free);
  costs_.assign(cells, 0.0f);

  // One image-sized scratch array plus line buffers; no per-obstacle flood or radius-sized kernel.
  std::vector<double> distances(cells, std::numeric_limits<double>::infinity());
  bool have_obstacle = false;
  for (std::size_t index = 0; index < cells; ++index) {
    if (known_[index] == Cell::Occupied || observed_[index] == Cell::Occupied) {
      distances[index] = 0.0;
      have_obstacle = true;
    }
  }
  if (have_obstacle && inflation.radius_m > 0.0) {
    const std::size_t line_length = static_cast<std::size_t>(std::max(width, height));
    std::vector<double> input(line_length), output(line_length), boundaries(line_length + 1);
    std::vector<int> sites(line_length);
    for (int y = 0; y < height; ++y) {
      const auto begin = distances.begin() + static_cast<std::size_t>(y) * width;
      std::copy(begin, begin + width, input.begin());
      distance_transform(input, output, width, sites, boundaries);
      std::copy(output.begin(), output.begin() + width, begin);
    }
    for (int x = 0; x < width; ++x) {
      for (int y = 0; y < height; ++y) {
        input[static_cast<std::size_t>(y)] = distances[static_cast<std::size_t>(y) * width + x];
      }
      distance_transform(input, output, height, sites, boundaries);
      for (int y = 0; y < height; ++y) {
        distances[static_cast<std::size_t>(y) * width + x] = output[static_cast<std::size_t>(y)];
      }
    }
  }
  for (std::size_t index = 0; index < cells; ++index) {
    if (known_[index] != Cell::Free || observed_[index] != Cell::Free) {
      costs_[index] = 1.0f;
      continue;
    }
    const double distance_m = std::sqrt(distances[index]) * resolution;
    if (distance_m <= inflation.radius_m) {
      costs_[index] = static_cast<float>(std::exp(-inflation.scaling * distance_m));
    }
  }
}

bool GridSnapshot::contains(int x, int y) const
{
  return x >= 0 && y >= 0 && x < width_ && y < height_;
}

std::optional<CellIndex> GridSnapshot::world_to_cell(Point2 point) const
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y)) return std::nullopt;
  const double x = (point.x - origin_.x) / resolution_;
  const double y = (point.y - origin_.y) / resolution_;
  // Check the floating-point range before converting to int, including overflow to infinity.
  if (!(x >= 0.0 && x < width_ && y >= 0.0 && y < height_)) return std::nullopt;
  return CellIndex{static_cast<int>(std::floor(x)), static_cast<int>(std::floor(y))};
}

Point2 GridSnapshot::cell_center(int x, int y) const
{
  if (!contains(x, y)) throw std::out_of_range("cell center requested outside the grid");
  return {origin_.x + (x + 0.5) * resolution_, origin_.y + (y + 0.5) * resolution_};
}

Cell GridSnapshot::occupancy(int x, int y) const
{
  if (!contains(x, y)) return Cell::Unknown;
  const std::size_t index = static_cast<std::size_t>(y) * width_ + x;
  const Cell known = known_[index], observed = observed_[index];
  if (known == Cell::Occupied || observed == Cell::Occupied) return Cell::Occupied;
  if (known == Cell::Unknown || observed == Cell::Unknown) return Cell::Unknown;
  return Cell::Free;
}

double GridSnapshot::cost(int x, int y) const
{
  if (!contains(x, y)) return 1.0;
  return costs_[static_cast<std::size_t>(y) * width_ + x];
}

std::size_t GridSnapshot::storage_bytes() const
{
  return sizeof(*this) + known_.capacity() * sizeof(Cell) + observed_.capacity() * sizeof(Cell) +
         costs_.capacity() * sizeof(float);
}
}  // namespace mobile_manipulator_routing
