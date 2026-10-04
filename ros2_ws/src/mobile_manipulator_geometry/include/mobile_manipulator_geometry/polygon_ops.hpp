#pragma once
// Convex hull, exact outward offset and measures of ground-plane polygons (no ROS graph).
#include <vector>

#include "mobile_manipulator_geometry/polygon.hpp"

namespace mobile_manipulator_geometry
{
// Counter-clockwise convex hull without collinear or duplicate vertices (Andrew's monotone
// chain). Throws std::invalid_argument for fewer than 3 non-collinear points.
Polygon convex_hull(const std::vector<Point2> & points);

// Outward offset of a counter-clockwise convex polygon: every edge moves out by `delta`
// along its normal and neighbouring edges are re-intersected. Where that mitre would lie
// more than 2 * delta from the vertex (interior angle below 60 deg), the corner is cut by
// the line tangent to the delta circle around the vertex, perpendicular to its bisector,
// so every edge of the result lies at least `delta` from the polygon. (Sagar et al. push
// each vertex away from the centroid instead, which gives a thin polygon's long edges far
// less than `delta`.)
Polygon offset_outward(const Polygon & convex_ccw, double delta);

// Symmetric Hausdorff distance between the two polygon boundaries (sampled at <= 5 mm).
double hausdorff(const Polygon & a, const Polygon & b);

double area(const Polygon & polygon);
// Largest vertex distance from the origin (base_footprint): Nav2's circumscribed radius.
double circumscribed_radius(const Polygon & polygon);
}  // namespace mobile_manipulator_geometry
