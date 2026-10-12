#pragma once
// The dynamic footprint of baseline B4 (no ROS graph): the padded convex hull of base, arm and
// panel from the latest joint state, republished only when it moves by more than a threshold.
// While the base moves the footprint only grows: a cantilevered panel sways under acceleration,
// and republishing every swing re-inflates both costmaps mid-drive. A hull inside the current
// one (grown by the threshold) is held, a hull outside it publishes the union; the footprint
// shrinks by the threshold rule once the base has been still for kMotionHoldS.
// footprint_mode:=wholebody (baseline B5) keeps that node but not the arm-dependent footprint: the
// whole-body MPC plans the arm and base together and keeps the arm and panel clear of the known
// obstacles itself, so both costmaps and the stop zone get the base-only footprint and only the
// slowdown zone follows the arm: the hull swept by the configurations of the latest MPC plan.
// Times are receive times on a steady clock, never message stamps, so a simulation epoch or
// teleport (sim time going backwards) cannot make fresh data look stale or the reverse.
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mobile_manipulator_geometry/footprint_model.hpp"
#include "mobile_manipulator_navigation/scenario_spec.hpp"

namespace mobile_manipulator_navigation
{
// The padded convex hull, in the current base frame, of the model's ground points for each
// predicted configuration placed at its pose (x, y, yaw) in that frame. Throws
// std::invalid_argument without a configuration.
Polygon swept_hull(const mobile_manipulator_geometry::FootprintModel & model,
                   const std::vector<std::pair<Pose2, mobile_manipulator_geometry::JointMap>> & predicted_in_base,
                   double padding_m);

// Why a footprint is published: the first one, a grow while the base moves, or a change found
// by the Hausdorff rule at standstill (arm moved, or the shrink once the base has been still).
enum class PublishReason { Initial, Grow, Standstill };

inline const char * to_string(PublishReason reason)
{
  switch (reason) {
    case PublishReason::Initial: return "initial";
    case PublishReason::Grow: return "grow";
    case PublishReason::Standstill: return "standstill";
  }
  return "";
}

struct FootprintUpdate
{
  Polygon footprint;           // padded hull, base_footprint
  std::vector<Polygon> zones;  // offset_outward(footprint, margin) per zone margin
  PublishReason reason = PublishReason::Initial;
  // wholebody mode only: whether the costmap footprint and each zone changed since the last
  // update (unchanged ones are not republished: a republished footprint re-inflates the costmaps).
  bool footprint_changed = true;
  std::vector<bool> zone_changed;  // empty: every zone changed
};

struct TickStats
{
  double compute_s = 0.0;
  size_t points = 0;
  size_t vertices = 0;
  bool beyond_inflation = false;  // circumscribed radius (with Nav2's padding) above the inflation radius
};

class DynamicFootprint
{
public:
  enum class Health { NoData, Fresh, Stale, MissingJoint };
  static constexpr double kMovingLinearMps = 0.01;
  static constexpr double kMovingYawRadps = 0.02;
  static constexpr double kMotionHoldS = 0.5;

  DynamicFootprint(std::shared_ptr<const mobile_manipulator_geometry::FootprintModel> model,
                   std::vector<std::string> arm_joints, std::vector<double> zone_margins_m, double padding_m,
                   double change_threshold_m, double stale_after_s, double inflation_radius_m,
                   double nav2_padding_m = 0.0);

  // Merge a (possibly partial) joint state received at `receive_time`; non-finite values are
  // ignored (neither stored nor counted as received).
  void joints(double receive_time, const std::vector<std::string> & names, const std::vector<double> & positions);
  // Report the commanded base velocity (/cmd_vel) at `receive_time`. A speed at or above
  // kMovingLinearMps or kMovingYawRadps counts as moving for kMotionHoldS afterwards. The command
  // is used rather than /odom because arm motion alone rocks the simulated base past those
  // thresholds.
  void base_motion(double receive_time, double linear_mps, double yaw_radps);
  bool moving(double now) const;

  // footprint_mode:=wholebody. `base_only` gives the footprint of the costmaps and the stop zone
  // (whatever the joints); zone `slowdown_zone` (an index into the zone margins) follows the swept
  // hull instead. A plan is used while it is at most kPlanFreshS old.
  static constexpr double kPlanFreshS = 0.5;
  void use_wholebody(std::shared_ptr<const mobile_manipulator_geometry::FootprintModel> base_only,
                     size_t slowdown_zone);
  // The latest plan received at `receive_time`: the predicted configurations placed relative to the
  // current base pose (stage 0 first), with every model joint set.
  void plan(double receive_time,
            std::vector<std::pair<Pose2, mobile_manipulator_geometry::JointMap>> predicted_in_base);
  // The wholebody counterpart of tick(): the base-only footprint and zones once, then the slowdown
  // zone whenever the swept hull (of the plan and the measured configuration; without a fresh plan
  // of the measured configuration alone) moves by more than the change threshold.
  std::optional<FootprintUpdate> tick_wholebody(double now);
  const std::map<std::string, double> & positions() const { return positions_; }
  // The footprint and zones to publish, or nothing (no change, missing or stale joints).
  std::optional<FootprintUpdate> tick(double now);
  const std::optional<Polygon> & current() const { return current_; }
  Health health(double now) const;
  const TickStats & last_stats() const { return stats_; }

private:
  std::shared_ptr<const mobile_manipulator_geometry::FootprintModel> model_;
  std::vector<std::string> arm_joints_;
  std::vector<double> zone_margins_m_;
  double padding_m_, change_threshold_m_, stale_after_s_, inflation_radius_m_, nav2_padding_m_;
  bool any_message_ = false;
  std::map<std::string, double> positions_;
  std::map<std::string, double> received_;
  std::optional<Polygon> current_;
  std::optional<double> last_motion_;
  std::shared_ptr<const mobile_manipulator_geometry::FootprintModel> base_only_;
  size_t slowdown_zone_ = 0;
  std::optional<double> plan_time_;
  std::vector<std::pair<Pose2, mobile_manipulator_geometry::JointMap>> plan_;
  std::optional<Polygon> slowdown_;
  TickStats stats_;
};
}  // namespace mobile_manipulator_navigation
