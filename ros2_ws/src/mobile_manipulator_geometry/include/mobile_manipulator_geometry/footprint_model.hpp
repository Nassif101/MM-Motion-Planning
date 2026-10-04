#pragma once
// Configuration-dependent footprint models (Sagar, Long, Garcia Santiago, CoDIT 2026): the
// ground projection of the robot and its panel for a joint state, hulled and offset.
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "mobile_manipulator_geometry/footprint_projection.hpp"
#include "mobile_manipulator_geometry/polygon_ops.hpp"

namespace mobile_manipulator_geometry
{
class FootprintModel
{
public:
  virtual ~FootprintModel() = default;
  // Ground points (base_footprint) whose convex hull is the unpadded footprint.
  virtual std::vector<Point2> points(const JointMap & joints) const = 0;
  // offset_outward(convex_hull(points(joints)), delta).
  Polygon footprint(const JointMap & joints, double delta) const;
};

// The paper's mesh variant: every URDF collision primitive plus the panel box.
class MeshHullModel : public FootprintModel
{
public:
  explicit MeshHullModel(std::shared_ptr<const FootprintProjector> projector, int cylinder_sides = 16);
  std::vector<Point2> points(const JointMap & joints) const override;

private:
  std::shared_ptr<const FootprintProjector> projector_;
  int cylinder_sides_;
};

struct DiscLink
{
  std::string link;
  std::optional<double> radius_m;  // empty: computed from the URDF
};

struct DiscModelConfig
{
  std::vector<DiscLink> links;       // kinematic chain order, shoulder to tool
  int disc_samples = 16;             // N_d: sides of each circumscribed disc polygon
  int segment_samples = 3;           // N_s: segment discs at j = 0..N_s between origins
  std::set<std::string> base_links;  // links of the base point set B (projected at zero joints)
};

// The paper's disc method: base points B, a disc of radius r_i at each projected link
// origin, discs of radius r_i at N_s + 1 samples along each segment to the next origin, and
// (for this robot) the eight projected corners of the panel box. A computed r_i is the
// largest 3D distance from link i's collision points to the segment from its origin to the
// next link's origin (the last link: to its own origin), rounded up to 0.005 m; that capsule
// is rigid in link i, so its projection encloses the link's in every pose.
class DiscHullModel : public FootprintModel
{
public:
  DiscHullModel(std::shared_ptr<const FootprintProjector> projector, DiscModelConfig config);
  std::vector<Point2> points(const JointMap & joints) const override;
  const std::vector<double> & radii() const { return radii_; }

private:
  std::shared_ptr<const FootprintProjector> projector_;
  DiscModelConfig config_;
  std::vector<double> radii_;
  std::vector<Point2> base_points_;
};
}  // namespace mobile_manipulator_geometry
