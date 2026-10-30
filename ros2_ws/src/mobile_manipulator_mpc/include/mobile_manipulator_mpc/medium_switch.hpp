#pragma once
// The per-cycle choice between the medium layer's plan and the frozen B5 (Phase 5A, reference_source: medium), without
// ROS so the controller tests can run it: the node hands each /medium/plan to set_plan and calls update before every
// Controller::step.
#include <cstdint>
#include <optional>

#include "mobile_manipulator_mpc/controller.hpp"
#include "mobile_manipulator_mpc/medium_reference.hpp"

namespace mobile_manipulator_mpc
{
class MediumSwitch
{
public:
  explicit MediumSwitch(const MpcConfig & config) : config_(config) {}

  // A plan for route `route_id` planned from the state at `stamp` (s, the controller's clock). It takes effect at the
  // next update (the controller keeps a pointer to the plan it follows until then).
  void set_plan(MediumReference plan, uint64_t plan_id, uint64_t route_id, double stamp);

  // One cycle: medium_fallback for the latest plan against the active route (`active_route`, the Nav2 route and its
  // goal's arc length) and the measured base's distance to the plan's route. Following, the controller tracks the
  // plan's route with its references (a new plan: replace_route at its start_hint); otherwise the Nav2 route with
  // none (the frozen B5), swapped back with replace_route only when the source changes (hint: the base's projection).
  MediumFallback update(Controller & controller, double now, const Eigen::Vector2d & base, uint64_t active_route,
                        const Route & nav2_route, double nav2_goal_s);

  // The controller's route changed under the switch (a new or replanned goal: start or update_goal set the Nav2
  // route): the plan it followed is dropped from the controller; the next update follows a matching plan again.
  void route_changed(Controller & controller);

  bool following() const { return following_; }
  // The latest plan's age at `now` (NaN without one), its ids (0 without one).
  double plan_age(double now) const;
  uint64_t plan_id() const;
  uint64_t plan_route_id() const;

private:
  struct Held
  {
    MediumReference plan;
    uint64_t plan_id, route_id;
    double stamp;
  };
  const Held * latest() const { return pending_ ? &*pending_ : active_ ? &*active_ : nullptr; }

  MpcConfig config_;
  std::optional<Held> active_;   // the plan the controller may point at
  std::optional<Held> pending_;  // a newer plan, swapped in by update
  bool following_ = false;
  uint64_t applied_plan_ = 0;    // the plan_id the controller follows
};
}  // namespace mobile_manipulator_mpc
