#include "mobile_manipulator_navigation/dynamic_footprint.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

#include "mobile_manipulator_geometry/polygon_ops.hpp"
#include "mobile_manipulator_navigation/mission.hpp"

namespace mmg = mobile_manipulator_geometry;

namespace
{
// Whether `point` is inside the counter-clockwise convex `polygon`.
bool contains(const mobile_manipulator_navigation::Polygon & polygon, const mmg::Point2 & point)
{
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    if ((b[0] - a[0]) * (point[1] - a[1]) - (b[1] - a[1]) * (point[0] - a[0]) < 0.0) return false;
  }
  return true;
}

bool contains_all(const mobile_manipulator_navigation::Polygon & outer,
                  const mobile_manipulator_navigation::Polygon & inner)
{
  for (const auto & point : inner) {
    if (!contains(outer, point)) return false;
  }
  return true;
}
}  // namespace

namespace mobile_manipulator_navigation
{
DynamicFootprint::DynamicFootprint(std::shared_ptr<const mmg::FootprintModel> model, std::vector<std::string> arm_joints,
                                   std::vector<double> zone_margins_m, double padding_m, double change_threshold_m,
                                   double stale_after_s, double inflation_radius_m, double nav2_padding_m)
: model_(std::move(model)), arm_joints_(std::move(arm_joints)), zone_margins_m_(std::move(zone_margins_m)),
  padding_m_(padding_m), change_threshold_m_(change_threshold_m), stale_after_s_(stale_after_s),
  inflation_radius_m_(inflation_radius_m), nav2_padding_m_(nav2_padding_m)
{
  if (!model_) throw std::invalid_argument("DynamicFootprint needs a footprint model");
  if (arm_joints_.empty()) throw std::invalid_argument("DynamicFootprint needs the arm joint names");
}

void DynamicFootprint::joints(double receive_time, const std::vector<std::string> & names,
                              const std::vector<double> & positions)
{
  any_message_ = true;
  for (size_t i = 0; i < names.size() && i < positions.size(); ++i) {
    if (!std::isfinite(positions[i])) continue;
    positions_[names[i]] = positions[i];
    received_[names[i]] = receive_time;
  }
}

void DynamicFootprint::base_motion(double receive_time, double linear_mps, double yaw_radps)
{
  if (std::abs(linear_mps) >= kMovingLinearMps || std::abs(yaw_radps) >= kMovingYawRadps) last_motion_ = receive_time;
}

bool DynamicFootprint::moving(double now) const { return last_motion_ && now - *last_motion_ <= kMotionHoldS; }

DynamicFootprint::Health DynamicFootprint::health(double now) const
{
  if (!any_message_) return Health::NoData;
  bool stale = false;
  for (const auto & name : arm_joints_) {
    const auto found = received_.find(name);
    if (found == received_.end()) return Health::MissingJoint;
    stale = stale || now - found->second > stale_after_s_;
  }
  return stale ? Health::Stale : Health::Fresh;
}

std::optional<FootprintUpdate> DynamicFootprint::tick(double now)
{
  if (health(now) != Health::Fresh) return std::nullopt;
  const auto start = std::chrono::steady_clock::now();
  const auto points = model_->points(positions_);
  const auto footprint = mmg::offset_outward(mmg::convex_hull(points), padding_m_);
  stats_ = {std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(), points.size(),
            footprint.size(),
            mmg::circumscribed_radius(nav2_padded(footprint, nav2_padding_m_)) > inflation_radius_m_};
  Polygon published = footprint;
  PublishReason reason = current_ ? PublishReason::Standstill : PublishReason::Initial;
  if (current_ && moving(now)) {
    reason = PublishReason::Grow;
    // Grow only: a hull inside the current one (plus the threshold) is sway, not a new pose.
    if (contains_all(mmg::offset_outward(*current_, change_threshold_m_), footprint)) return std::nullopt;
    std::vector<mmg::Point2> both(*current_);
    both.insert(both.end(), footprint.begin(), footprint.end());
    published = mmg::convex_hull(both);
  } else if (current_ && mmg::hausdorff(footprint, *current_) <= change_threshold_m_) {
    return std::nullopt;
  }
  current_ = published;
  FootprintUpdate update{published, {}, reason};
  for (const double margin : zone_margins_m_) update.zones.push_back(mmg::offset_outward(published, margin));
  return update;
}
}  // namespace mobile_manipulator_navigation
