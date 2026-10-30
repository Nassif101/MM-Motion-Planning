#pragma once
// B5's references in reference_source: medium (Phase 5A spec Section 2, plan Task 10): the medium layer's plan as the
// route B5 tracks (its samples, then the Nav2 route's remainder from the splice) and, by arc length on that route, the
// heading, speed and posture the plan asks for; and the per-cycle rule that falls back to the frozen B5.
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "mobile_manipulator_mpc/formulation.hpp"
#include "mobile_manipulator_mpc/route.hpp"

namespace mobile_manipulator_mpc
{
// One sample of a medium plan (mobile_manipulator_interfaces/MediumPlanSample without ROS).
struct MediumSample
{
  double s, x, y, yaw, speed;
  Eigen::Matrix<double, 6, 1> q;
};

class MediumReference
{
public:
  // The route: the samples' positions, then (when splice_s is before its end) the Nav2 route from splice_s on, joined
  // at its point there. Arc length on it is the samples' own distance along their positions (consecutive samples at
  // one position, a turn on the spot, share it). `comfort`: the posture beyond the samples. Throws
  // std::invalid_argument for fewer than two samples or samples that do not span a route.
  MediumReference(std::vector<MediumSample> samples, const Route & nav2_route, double splice_s,
                  const Eigen::Matrix<double, 6, 1> & comfort);

  const Route & route() const { return route_; }
  double goal_s() const { return route_.length(); }  // the Nav2 goal is the route's end
  double samples_length() const { return s_.back(); }  // where the samples end on the route

  // At arc length s on route(): between samples linear in s (yaw by its shortest turn), at a run of samples at one s
  // the last of them; beyond the samples with a Nav2 remainder the route's tangent, infinity (B5's own progress speed)
  // and the comfort pose, without one the last sample's values.
  double yaw_at(double s) const;
  double speed_at(double s) const;
  Eigen::Matrix<double, 6, 1> posture_at(double s) const;
  // The arc length of the sample nearest `base` among those within the first metre (the first cycle's progress hint).
  double start_hint(const Eigen::Vector2d & base) const;

private:
  // The index of the last sample at or before s (the run's last at equal s), and whether s lies beyond the samples.
  size_t index_at(double s) const;

  std::vector<MediumSample> samples_;
  std::vector<double> s_;  // each sample's arc length on route_
  Route route_;
  bool remainder_;
  Eigen::Matrix<double, 6, 1> comfort_;
};

enum class MediumFallback { None, NoPlan, Stale, RouteId, Offset };
// Why a cycle in reference_source: medium falls back to the frozen B5 (None: it follows the plan): no plan (age NaN),
// a plan for another route than the active one, a plan older than medium_stale_s, or the measured base farther than
// medium_max_offset_m from the plan's route.
MediumFallback medium_fallback(double plan_age_s, uint64_t plan_route, uint64_t active_route, double offset_m,
                               const MpcConfig & config);
const char * to_string(MediumFallback reason);
}  // namespace mobile_manipulator_mpc
