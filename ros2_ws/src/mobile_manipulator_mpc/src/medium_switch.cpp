#include "mobile_manipulator_mpc/medium_switch.hpp"

#include <cmath>
#include <limits>

namespace mobile_manipulator_mpc
{
void MediumSwitch::set_plan(MediumReference plan, uint64_t plan_id, uint64_t route_id, double stamp)
{
  pending_.emplace(Held{std::move(plan), plan_id, route_id, stamp});
}

double MediumSwitch::plan_age(double now) const
{
  const Held * h = latest();
  return h != nullptr ? now - h->stamp : std::numeric_limits<double>::quiet_NaN();
}

uint64_t MediumSwitch::plan_id() const { return latest() != nullptr ? latest()->plan_id : 0; }
uint64_t MediumSwitch::plan_route_id() const { return latest() != nullptr ? latest()->route_id : 0; }

MediumFallback MediumSwitch::update(Controller & controller, double now, const Eigen::Vector2d & base,
                                    uint64_t active_route, const Route & nav2_route, double nav2_goal_s)
{
  const Held * h = latest();
  double offset = 0.0;
  if (h != nullptr) {
    const Route & r = h->plan.route();
    offset = (r.project_range(base, 0.0, r.length()).point - base).norm();
  }
  const MediumFallback reason =
    medium_fallback(plan_age(now), h != nullptr ? h->route_id : 0, active_route, offset, config_);
  if (reason == MediumFallback::None) {
    const bool new_plan = pending_.has_value();
    if (new_plan) {
      active_ = std::move(pending_);
      pending_.reset();
    }
    if (!following_ || new_plan || applied_plan_ != active_->plan_id) {
      const MediumReference & plan = active_->plan;
      controller.replace_route(plan.route(), plan.goal_s(), plan.start_hint(base));
      controller.set_medium(&plan);
      applied_plan_ = active_->plan_id;
      following_ = true;
    }
  } else if (following_) {
    controller.set_medium(nullptr);
    controller.replace_route(nav2_route, nav2_goal_s, nav2_route.project_range(base, 0.0, nav2_route.length()).s);
    following_ = false;
  }
  return reason;
}
}  // namespace mobile_manipulator_mpc
