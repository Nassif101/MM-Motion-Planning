#include "mobile_manipulator_geometry/footprint_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mobile_manipulator_geometry
{
namespace
{
double segment_distance(const Eigen::Vector3d & a, const Eigen::Vector3d & b, const Eigen::Vector3d & p)
{
  const Eigen::Vector3d e = b - a;
  const double length2 = e.squaredNorm();
  const double t = length2 > 0.0 ? std::clamp((p - a).dot(e) / length2, 0.0, 1.0) : 0.0;
  return (p - a - t * e).norm();
}

// Centre plus a circumscribed `sides`-gon of radius r (just the centre when r is 0).
void add_disc(std::vector<Point2> & points, const Point2 & centre, double r, int sides)
{
  points.push_back(centre);
  if (r <= 0.0) return;
  const double outer = r / std::cos(M_PI / sides);
  for (int k = 0; k < sides; ++k) {
    const double angle = 2.0 * M_PI * k / sides;
    points.push_back({centre[0] + outer * std::cos(angle), centre[1] + outer * std::sin(angle)});
  }
}
}  // namespace

Polygon FootprintModel::footprint(const JointMap & joints, double delta) const
{
  return offset_outward(convex_hull(points(joints)), delta);
}

MeshHullModel::MeshHullModel(std::shared_ptr<const FootprintProjector> projector, int cylinder_sides)
: projector_(std::move(projector)), cylinder_sides_(cylinder_sides)
{
  if (!projector_) throw std::invalid_argument("MeshHullModel needs a projector");
  if (cylinder_sides_ < 3) throw std::invalid_argument("cylinder_sides must be at least 3");
}

std::vector<Point2> MeshHullModel::points(const JointMap & joints) const
{
  return projector_->projected_points(joints, {cylinder_sides_, {}, true});
}

DiscHullModel::DiscHullModel(std::shared_ptr<const FootprintProjector> projector, DiscModelConfig config)
: projector_(std::move(projector)), config_(std::move(config))
{
  if (!projector_) throw std::invalid_argument("DiscHullModel needs a projector");
  if (config_.links.empty()) throw std::invalid_argument("DiscHullModel needs at least one link");
  if (config_.base_links.empty()) throw std::invalid_argument("DiscHullModel needs base links");
  if (config_.disc_samples < 3) throw std::invalid_argument("disc_samples must be at least 3");
  if (config_.segment_samples < 0) throw std::invalid_argument("segment_samples must be at least 0");
  base_points_ = projector_->projected_points({}, {16, config_.base_links, false});
  if (base_points_.empty()) throw std::invalid_argument("the base links have no collision geometry");

  // Capsule radius of each link about the segment from its origin to the next link's origin.
  // A child link's origin is its joint origin, fixed in the parent link whatever the joint
  // angle, so the zero-pose relative transform gives it.
  const JointMap zero;
  for (size_t i = 0; i < config_.links.size(); ++i) {
    const auto & link = config_.links[i];
    if (link.radius_m) {
      radii_.push_back(*link.radius_m);
      continue;
    }
    const Eigen::Isometry3d inverse = projector_->link_pose(link.link, zero).inverse();
    Eigen::Vector3d end = Eigen::Vector3d::Zero();
    if (i + 1 < config_.links.size()) end = (inverse * projector_->link_pose(config_.links[i + 1].link, zero)).translation();
    double radius = 0.0;
    const auto & primitives = projector_->primitives();
    for (size_t k = 0; k + 1 < primitives.size(); ++k) {  // the payload box is the last primitive
      const auto & primitive = primitives[k];
      if (primitive.link != link.link) continue;
      for (const auto & local : primitive_points(primitive, 64)) {
        radius = std::max(radius, segment_distance(Eigen::Vector3d::Zero(), end, primitive.pose * local));
      }
    }
    radii_.push_back(std::ceil(radius / 0.005 - 1e-9) * 0.005);
  }
}

std::vector<Point2> DiscHullModel::points(const JointMap & joints) const
{
  std::vector<Point2> points = base_points_;
  std::vector<Point2> origins;
  for (const auto & link : config_.links) {
    const Eigen::Vector3d p = projector_->link_pose(link.link, joints).translation();
    origins.push_back({p.x(), p.y()});
  }
  for (size_t i = 0; i < origins.size(); ++i) {
    add_disc(points, origins[i], radii_[i], config_.disc_samples);
    if (i + 1 == origins.size()) continue;
    for (int j = 0; j <= config_.segment_samples; ++j) {
      const double t = config_.segment_samples ? static_cast<double>(j) / config_.segment_samples : 0.0;
      add_disc(points, {origins[i][0] + t * (origins[i + 1][0] - origins[i][0]),
                        origins[i][1] + t * (origins[i + 1][1] - origins[i][1])},
               radii_[i], config_.disc_samples);
    }
  }
  // The panel box (the paper's disc model has no payload).
  const auto & payload = projector_->payload();
  const Eigen::Isometry3d frame = projector_->link_pose(payload.link, joints);
  for (const double sx : {-0.5, 0.5})
    for (const double sy : {-0.5, 0.5})
      for (const double sz : {-0.5, 0.5}) {
        const Eigen::Vector3d corner = frame * (payload.center + Eigen::Vector3d(sx * payload.size.x(), sy * payload.size.y(),
                                                                                sz * payload.size.z()));
        points.push_back({corner.x(), corner.y()});
      }
  return points;
}
}  // namespace mobile_manipulator_geometry
