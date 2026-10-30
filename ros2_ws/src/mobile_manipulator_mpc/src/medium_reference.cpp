#include "mobile_manipulator_mpc/medium_reference.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mobile_manipulator_mpc
{
namespace
{
constexpr double kHintWindowM = 1.0;  // start_hint searches the samples within this of the start
constexpr double kSameM = 1e-9;

std::vector<Eigen::Vector2d> route_points(const std::vector<MediumSample> & samples, const Route & nav2, double splice_s,
                                          bool & remainder)
{
  if (samples.size() < 2) throw std::invalid_argument("MediumReference: fewer than two samples");
  for (const auto & s : samples)
    if (!std::isfinite(s.x) || !std::isfinite(s.y) || !std::isfinite(s.yaw) || !std::isfinite(s.speed) ||
        !s.q.allFinite())
      throw std::invalid_argument("MediumReference: a sample is not finite");
  std::vector<Eigen::Vector2d> points;
  for (const auto & sample : samples) points.emplace_back(sample.x, sample.y);
  remainder = splice_s < nav2.length() - kSameM;
  if (remainder) {
    points.push_back(nav2.at(splice_s));
    double s = 0.0;
    const auto & p = nav2.points();
    for (size_t i = 1; i < p.size(); ++i) {
      s += (p[i] - p[i - 1]).norm();
      if (s > splice_s + kSameM) points.push_back(p[i]);
    }
  }
  return points;
}
}  // namespace

MediumReference::MediumReference(std::vector<MediumSample> samples, const Route & nav2_route, double splice_s,
                                 const Eigen::Matrix<double, 6, 1> & comfort)
: samples_(std::move(samples)),
  route_(route_points(samples_, nav2_route, splice_s, remainder_)),
  comfort_(comfort)
{
  s_.resize(samples_.size());
  s_[0] = 0.0;
  for (size_t i = 1; i < samples_.size(); ++i)
    s_[i] = s_[i - 1] + std::hypot(samples_[i].x - samples_[i - 1].x, samples_[i].y - samples_[i - 1].y);
  if (s_.back() <= kSameM) throw std::invalid_argument("MediumReference: the samples do not span a route");
}

size_t MediumReference::index_at(double s) const
{
  const auto after = std::upper_bound(s_.begin(), s_.end(), s + kSameM);
  return after == s_.begin() ? 0 : static_cast<size_t>(after - s_.begin()) - 1;
}

double MediumReference::yaw_at(double s) const
{
  if (remainder_ && s > samples_length() + kSameM) return route_.heading_at(s);
  const size_t i = index_at(s);
  if (i + 1 >= samples_.size()) return samples_[i].yaw;
  const double u = (s - s_[i]) / (s_[i + 1] - s_[i]);
  return samples_[i].yaw + u * std::remainder(samples_[i + 1].yaw - samples_[i].yaw, 2.0 * M_PI);
}

double MediumReference::speed_at(double s) const
{
  if (remainder_ && s > samples_length() + kSameM) return std::numeric_limits<double>::infinity();
  const size_t i = index_at(s);
  if (i + 1 >= samples_.size()) return samples_[i].speed;
  const double u = (s - s_[i]) / (s_[i + 1] - s_[i]);
  return samples_[i].speed + u * (samples_[i + 1].speed - samples_[i].speed);
}

Eigen::Matrix<double, 6, 1> MediumReference::posture_at(double s) const
{
  if (remainder_ && s > samples_length() + kSameM) return comfort_;
  const size_t i = index_at(s);
  if (i + 1 >= samples_.size()) return samples_[i].q;
  const double u = (s - s_[i]) / (s_[i + 1] - s_[i]);
  return samples_[i].q + u * (samples_[i + 1].q - samples_[i].q);
}

double MediumReference::start_hint(const Eigen::Vector2d & base) const
{
  double best = std::numeric_limits<double>::infinity(), hint = 0.0;
  for (size_t i = 0; i < samples_.size() && s_[i] <= kHintWindowM + kSameM; ++i) {
    const double d = (Eigen::Vector2d(samples_[i].x, samples_[i].y) - base).norm();
    if (d < best) {
      best = d;
      hint = s_[i];
    }
  }
  return hint;
}

MediumFallback medium_fallback(double plan_age_s, uint64_t plan_route, uint64_t active_route, double offset_m,
                               const MpcConfig & config)
{
  if (std::isnan(plan_age_s)) return MediumFallback::NoPlan;
  if (plan_route != active_route) return MediumFallback::RouteId;
  if (plan_age_s > config.medium_stale_s) return MediumFallback::Stale;
  if (offset_m > config.medium_max_offset_m) return MediumFallback::Offset;
  return MediumFallback::None;
}

const char * to_string(MediumFallback reason)
{
  switch (reason) {
    case MediumFallback::None: return "";
    case MediumFallback::NoPlan: return "no_plan";
    case MediumFallback::Stale: return "stale";
    case MediumFallback::RouteId: return "route_id";
    case MediumFallback::Offset: return "offset";
  }
  return "";
}
}  // namespace mobile_manipulator_mpc
