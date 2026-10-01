#pragma once
// Metrics shared by the scenario tasks (no ROS dependencies).
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace mobile_manipulator_navigation
{
using Point2 = std::array<double, 2>;

// Rounds like Python's round(value, digits) for reported values.
inline double round_digits(double value, int digits)
{
  const double scale = std::pow(10.0, digits);
  return std::nearbyint(value * scale) / scale;
}

// Centres of occupied cells (value >= 65) of an occupancy grid, row-major from the origin.
inline std::vector<Point2> occupied_points(const std::vector<int8_t> & data, uint32_t width, uint32_t height,
                                           double resolution, double origin_x, double origin_y)
{
  std::vector<Point2> points;
  for (uint32_t row = 0; row < height; ++row) {
    for (uint32_t col = 0; col < width; ++col) {
      if (data[row * width + col] >= 65) {
        points.push_back({origin_x + (col + 0.5) * resolution, origin_y + (row + 0.5) * resolution});
      }
    }
  }
  return points;
}

inline double polyline_length(const std::vector<Point2> & points)
{
  double length = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    length += std::hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]);
  }
  return length;
}

// Minimum distance between path points and obstacle points, considering obstacles within
// 3 m of the path's bounding box (3.0 when none is that close); empty for an empty input.
inline std::optional<double> min_clearance(const std::vector<Point2> & path, const std::vector<Point2> & obstacles)
{
  if (path.empty() || obstacles.empty()) return std::nullopt;
  Point2 lo = path[0], hi = path[0];
  for (const auto & p : path) {
    lo = {std::min(lo[0], p[0]), std::min(lo[1], p[1])};
    hi = {std::max(hi[0], p[0]), std::max(hi[1], p[1])};
  }
  lo = {lo[0] - 3.0, lo[1] - 3.0};
  hi = {hi[0] + 3.0, hi[1] + 3.0};
  std::vector<Point2> near;
  for (const auto & o : obstacles) {
    if (o[0] >= lo[0] && o[0] <= hi[0] && o[1] >= lo[1] && o[1] <= hi[1]) near.push_back(o);
  }
  if (near.empty()) return 3.0;
  double best = std::numeric_limits<double>::infinity();
  for (const auto & p : path) {
    for (const auto & o : near) best = std::min(best, std::hypot(o[0] - p[0], o[1] - p[1]));
  }
  return round_digits(best, 3);
}

inline double yaw_of(double x, double y, double z, double w)
{
  return std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

// Axis-aligned footprint bounds in the robot frame.
struct Bounds
{
  double x0, x1, y0, y1;
};

// Distance from a robot-frame point to the footprint rectangle (0 inside).
inline double rectangle_distance(const Point2 & p, const Bounds & b)
{
  const double dx = std::max(std::max(b.x0 - p[0], p[0] - b.x1), 0.0);
  const double dy = std::max(std::max(b.y0 - p[1], p[1] - b.y1), 0.0);
  return std::hypot(dx, dy);
}

// Minimum distance between the footprint posed at (x, y, yaw) and the map-frame points
// closer than `radius` to (x, y); empty when there are none.
inline std::optional<double> footprint_clearance(const std::vector<Point2> & points, double x, double y,
                                                 double yaw, const Bounds & bounds,
                                                 double radius = std::numeric_limits<double>::infinity())
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  std::optional<double> best;
  for (const auto & p : points) {
    const double dx = p[0] - x, dy = p[1] - y;
    if (!(std::hypot(dx, dy) < radius)) continue;
    const double d = rectangle_distance({dx * c + dy * s, -dx * s + dy * c}, bounds);
    if (!best || d < *best) best = d;
  }
  return best;
}

// Points along the outline of an axis-aligned box in the map frame: bottom, top, left and
// right edges, each sampled at about `spacing` including both corners.
inline std::vector<Point2> box_outline(double cx, double cy, double size_x, double size_y, double spacing = 0.02)
{
  const double x0 = cx - size_x / 2, x1 = cx + size_x / 2, y0 = cy - size_y / 2, y1 = cy + size_y / 2;
  const auto steps = [spacing](double a, double b) {
    std::vector<double> values;
    const int n = std::max(2, static_cast<int>(std::nearbyint((b - a) / spacing)) + 1);
    for (int i = 0; i < n; ++i) values.push_back(i == n - 1 ? b : a + i * (b - a) / (n - 1));
    return values;
  };
  const auto xs = steps(x0, x1), ys = steps(y0, y1);
  std::vector<Point2> points;
  for (const double x : xs) points.push_back({x, y0});
  for (const double x : xs) points.push_back({x, y1});
  for (const double y : ys) points.push_back({x0, y});
  for (const double y : ys) points.push_back({x1, y});
  return points;
}

struct Summary
{
  double mean, p95, max;
};

// Mean, 95th percentile (nearest rank below) and maximum, rounded to 4 digits.
inline std::optional<Summary> summarize(std::vector<double> values)
{
  if (values.empty()) return std::nullopt;
  std::sort(values.begin(), values.end());
  double sum = 0.0;
  for (const double v : values) sum += v;
  const size_t index = std::min(values.size() - 1, static_cast<size_t>(0.95 * values.size()));
  return Summary{round_digits(sum / values.size(), 4), round_digits(values[index], 4), round_digits(values.back(), 4)};
}
}  // namespace mobile_manipulator_navigation
