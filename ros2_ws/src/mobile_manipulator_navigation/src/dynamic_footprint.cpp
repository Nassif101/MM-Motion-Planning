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
Polygon swept_hull(const mmg::FootprintModel & model,
                   const std::vector<std::pair<Pose2, mmg::JointMap>> & predicted_in_base, double padding_m)
{
  if (predicted_in_base.empty()) throw std::invalid_argument("swept_hull needs a configuration");
  std::vector<mmg::Point2> all;
  for (const auto & [pose, joints] : predicted_in_base) {
    const double c = std::cos(pose[2]), s = std::sin(pose[2]);
    for (const auto & [x, y] : model.points(joints)) all.push_back({pose[0] + c * x - s * y, pose[1] + s * x + c * y});
  }
  return mmg::offset_outward(mmg::convex_hull(all), padding_m);
}

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
  FootprintUpdate update{published, {}, reason, true, {}};
  for (const double margin : zone_margins_m_) update.zones.push_back(mmg::offset_outward(published, margin));
  return update;
}

void DynamicFootprint::use_wholebody(std::shared_ptr<const mmg::FootprintModel> base_only, size_t slowdown_zone)
{
  if (!base_only) throw std::invalid_argument("wholebody mode needs a base-only footprint model");
  if (slowdown_zone >= zone_margins_m_.size()) throw std::invalid_argument("slowdown zone index out of range");
  base_only_ = std::move(base_only);
  slowdown_zone_ = slowdown_zone;
}

void DynamicFootprint::plan(double receive_time, std::vector<std::pair<Pose2, mmg::JointMap>> predicted_in_base)
{
  plan_time_ = receive_time;
  plan_ = std::move(predicted_in_base);
}

std::optional<FootprintUpdate> DynamicFootprint::tick_wholebody(double now)
{
  if (!base_only_) throw std::logic_error("tick_wholebody needs use_wholebody");
  if (health(now) != Health::Fresh) return std::nullopt;
  const auto start = std::chrono::steady_clock::now();
  const auto points = base_only_->points(positions_);
  const auto base = mmg::offset_outward(mmg::convex_hull(points), padding_m_);
  std::vector<std::pair<Pose2, mmg::JointMap>> configurations;
  if (plan_time_ && now - *plan_time_ <= kPlanFreshS) configurations = plan_;
  configurations.push_back({{0.0, 0.0, 0.0}, positions_});
  const auto swept = swept_hull(*model_, configurations, padding_m_);
  stats_ = {std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(), points.size(),
            base.size(), mmg::circumscribed_radius(nav2_padded(base, nav2_padding_m_)) > inflation_radius_m_};

  FootprintUpdate update{base, {}, current_ ? PublishReason::Standstill : PublishReason::Initial, true, {}};
  update.footprint_changed = !current_ || mmg::hausdorff(base, *current_) > change_threshold_m_;
  const bool slowdown_changed = !slowdown_ || mmg::hausdorff(swept, *slowdown_) > change_threshold_m_;
  if (!update.footprint_changed && !slowdown_changed) return std::nullopt;
  if (update.footprint_changed) current_ = base;
  if (slowdown_changed) slowdown_ = swept;
  for (size_t i = 0; i < zone_margins_m_.size(); ++i) {
    update.zones.push_back(mmg::offset_outward(i == slowdown_zone_ ? *slowdown_ : *current_, zone_margins_m_[i]));
    update.zone_changed.push_back(i == slowdown_zone_ ? slowdown_changed : update.footprint_changed);
  }
  update.footprint = *current_;
  return update;
}
}  // namespace mobile_manipulator_navigation
