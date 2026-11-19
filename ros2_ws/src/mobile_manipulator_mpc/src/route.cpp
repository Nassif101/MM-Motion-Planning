#include "mobile_manipulator_mpc/route.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mobile_manipulator_mpc
{
namespace
{
constexpr double kDuplicateM = 1e-9;
}

Route::Route(std::vector<Eigen::Vector2d> points)
{
  for (const auto & p : points) {
    if (!points_.empty() && (p - points_.back()).norm() <= kDuplicateM) continue;
    s_.push_back(points_.empty() ? 0.0 : s_.back() + (p - points_.back()).norm());
    points_.push_back(p);
  }
  if (points_.size() < 2) throw std::invalid_argument("a route needs at least two distinct points");
}

Route::Route(const Eigen::Vector2d & anchor, double target_yaw)
: points_{anchor}, s_{0.0}, stationary_yaw_(std::remainder(target_yaw, 2.0 * M_PI))
{
  if (!anchor.allFinite() || !std::isfinite(target_yaw))
    throw std::invalid_argument("a stationary route needs a finite anchor and target yaw");
}

Route Route::stationary(const Eigen::Vector2d & anchor, double target_yaw)
{
  return Route(anchor, target_yaw);
}

size_t Route::segment_of(double s) const
{
  // First segment whose end lies beyond s, so a vertex belongs to the outgoing segment.
  const auto it = std::upper_bound(s_.begin() + 1, s_.end() - 1, s);
  return static_cast<size_t>(it - (s_.begin() + 1));
}

Route::Projection Route::project(const Eigen::Vector2d & p, double s_hint, double window) const
{
  return project_range(p, s_hint - window, s_hint + window);
}

Route::Projection Route::project_range(const Eigen::Vector2d & p, double s_lo, double s_hi) const
{
  if (is_stationary()) {
    const Eigen::Vector2d offset = p - points_.front();
    return {0.0, points_.front(), stationary_yaw_,
            std::cos(stationary_yaw_) * offset.y() - std::sin(stationary_yaw_) * offset.x()};
  }
  s_lo = std::clamp(s_lo, 0.0, length());
  s_hi = std::clamp(s_hi, 0.0, length());
  if (s_lo > s_hi) std::swap(s_lo, s_hi);
  Projection best{0.0, points_.front(), 0.0, 0.0};
  double best_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 1 < points_.size(); ++i) {
    if (s_[i + 1] < s_lo || s_[i] > s_hi) continue;
    const Eigen::Vector2d d = points_[i + 1] - points_[i];
    const double segment = s_[i + 1] - s_[i];
    const double lo = std::max(s_lo, s_[i]) - s_[i], hi = std::min(s_hi, s_[i + 1]) - s_[i];
    const double t = std::clamp((p - points_[i]).dot(d) / segment, lo, hi);
    const Eigen::Vector2d q = points_[i] + t * d / segment;
    const double distance = (p - q).norm();
    if (distance < best_distance) {
      best_distance = distance;
      const Eigen::Vector2d u = d / segment;
      best = {s_[i] + t, q, std::atan2(u.y(), u.x()), u.x() * (p - q).y() - u.y() * (p - q).x()};
    }
  }
  return best;
}

Eigen::Vector2d Route::at(double s) const
{
  if (is_stationary()) return points_.front();
  s = std::clamp(s, 0.0, length());
  const size_t i = segment_of(s);
  return points_[i] + (s - s_[i]) / (s_[i + 1] - s_[i]) * (points_[i + 1] - points_[i]);
}

double Route::heading_at(double s) const
{
  if (is_stationary()) return stationary_yaw_;
  const size_t i = segment_of(std::clamp(s, 0.0, length()));
  const Eigen::Vector2d d = points_[i + 1] - points_[i];
  return std::atan2(d.y(), d.x());
}
}  // namespace mobile_manipulator_mpc
