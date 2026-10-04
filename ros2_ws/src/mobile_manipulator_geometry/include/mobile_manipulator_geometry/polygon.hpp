#pragma once
// Ground-plane points and polygons (x, y in base_footprint unless stated), and containment.
#include <array>
#include <vector>

namespace mobile_manipulator_geometry
{
using Point2 = std::array<double, 2>;
using Polygon = std::vector<Point2>;

struct Containment
{
  bool inside;
  // Minimum over points of the signed distance to the polygon's edge lines, positive
  // inside. For points outside it is the overshoot past the nearest violated edge.
  double margin_m;
};

// Containment of points in a convex polygon given in either winding.
Containment contains(const Polygon & convex_polygon, const std::vector<Point2> & points);
}  // namespace mobile_manipulator_geometry
