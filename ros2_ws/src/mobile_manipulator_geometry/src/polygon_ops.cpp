#include "mobile_manipulator_geometry/polygon_ops.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace mobile_manipulator_geometry
{
namespace
{
double cross(const Point2 & o, const Point2 & a, const Point2 & b)
{
  return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
}

double signed_area(const Polygon & polygon)
{
  double twice = 0.0;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    twice += a[0] * b[1] - b[0] * a[1];
  }
  return twice / 2.0;
}

double segment_distance(const Point2 & a, const Point2 & b, const Point2 & p)
{
  const double ex = b[0] - a[0], ey = b[1] - a[1];
  const double length2 = ex * ex + ey * ey;
  const double t = length2 > 0.0 ? std::clamp(((p[0] - a[0]) * ex + (p[1] - a[1]) * ey) / length2, 0.0, 1.0) : 0.0;
  return std::hypot(p[0] - a[0] - t * ex, p[1] - a[1] - t * ey);
}

// Boundary points at most `spacing` apart, vertices included.
std::vector<Point2> boundary_samples(const Polygon & polygon, double spacing)
{
  std::vector<Point2> samples;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    const int steps = std::max(1, static_cast<int>(std::ceil(std::hypot(b[0] - a[0], b[1] - a[1]) / spacing)));
    for (int k = 0; k < steps; ++k) {
      const double t = static_cast<double>(k) / steps;
      samples.push_back({a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])});
    }
  }
  return samples;
}

double directed_hausdorff(const Polygon & from, const Polygon & to)
{
  double worst = 0.0;
  for (const auto & p : boundary_samples(from, 0.005)) {
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < to.size(); ++i) best = std::min(best, segment_distance(to[i], to[(i + 1) % to.size()], p));
    worst = std::max(worst, best);
  }
  return worst;
}

// Outward unit normal of a counter-clockwise edge a -> b.
Point2 outward_normal(const Point2 & a, const Point2 & b)
{
  const double ex = b[0] - a[0], ey = b[1] - a[1];
  const double length = std::hypot(ex, ey);
  return {ey / length, -ex / length};
}
}  // namespace

Polygon convex_hull(const std::vector<Point2> & input)
{
  std::vector<Point2> points(input);
  std::sort(points.begin(), points.end());
  points.erase(std::unique(points.begin(), points.end()), points.end());
  if (points.size() < 3) throw std::invalid_argument("convex_hull needs at least 3 distinct points");
  Polygon hull(2 * points.size());
  size_t k = 0;
  // Lower then upper chain; popping on cross <= 0 drops collinear points.
  for (const auto & p : points) {
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], p) <= 0.0) --k;
    hull[k++] = p;
  }
  for (size_t i = points.size() - 1, lower = k + 1; i-- > 0;) {
    while (k >= lower && cross(hull[k - 2], hull[k - 1], points[i]) <= 0.0) --k;
    hull[k++] = points[i];
  }
  hull.resize(k - 1);
  if (hull.size() < 3) throw std::invalid_argument("convex_hull: the points are collinear");
  return hull;
}

Polygon offset_outward(const Polygon & polygon, double delta)
{
  if (polygon.size() < 3) throw std::invalid_argument("offset_outward needs at least 3 vertices");
  if (signed_area(polygon) <= 0.0) throw std::invalid_argument("offset_outward needs a counter-clockwise polygon");
  if (delta < 0.0) throw std::invalid_argument("offset_outward needs delta >= 0");
  Polygon result;
  const size_t n = polygon.size();
  for (size_t i = 0; i < n; ++i) {
    const auto & prev = polygon[(i + n - 1) % n];
    const auto & v = polygon[i];
    const auto & next = polygon[(i + 1) % n];
    const auto n1 = outward_normal(prev, v);
    const auto n2 = outward_normal(v, next);
    const double c = n1[0] * n2[0] + n1[1] * n2[1];  // cos of the turn between the normals
    // The mitre lies delta / cos(turn / 2) from the vertex: within 2 * delta for turns up to 120 deg.
    if (1.0 + c >= 0.5) {
      const double scale = delta / (1.0 + c);
      result.push_back({v[0] + scale * (n1[0] + n2[0]), v[1] + scale * (n1[1] + n2[1])});
      continue;
    }
    // Bevel: the line tangent to the delta circle at the bisector b, cut with each shifted edge.
    const double bx = n1[0] + n2[0], by = n1[1] + n2[1], bl = std::hypot(bx, by);
    const Point2 b{bx / bl, by / bl};
    for (const auto & [normal, from, to] : {std::tuple{n1, prev, v}, std::tuple{n2, v, next}}) {
      const double dx = to[0] - from[0], dy = to[1] - from[1], dl = std::hypot(dx, dy);
      const Point2 d{dx / dl, dy / dl};
      const double t = delta * (1.0 - (normal[0] * b[0] + normal[1] * b[1])) / (d[0] * b[0] + d[1] * b[1]);
      result.push_back({v[0] + delta * normal[0] + t * d[0], v[1] + delta * normal[1] + t * d[1]});
    }
  }
  return result;
}

double hausdorff(const Polygon & a, const Polygon & b)
{
  return std::max(directed_hausdorff(a, b), directed_hausdorff(b, a));
}

double area(const Polygon & polygon) { return std::abs(signed_area(polygon)); }

double circumscribed_radius(const Polygon & polygon)
{
  double radius = 0.0;
  for (const auto & [x, y] : polygon) radius = std::max(radius, std::hypot(x, y));
  return radius;
}
}  // namespace mobile_manipulator_geometry
