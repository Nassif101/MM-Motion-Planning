#pragma once
// The dynamic footprint of baseline B4 (no ROS graph): the padded convex hull of base, arm and
// panel from the latest joint state, republished only when it moves by more than a threshold.
// Times are receive times on a steady clock, never message stamps, so a simulation epoch or
// teleport (sim time going backwards) cannot make fresh data look stale or the reverse.
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "mobile_manipulator_geometry/footprint_model.hpp"
#include "mobile_manipulator_navigation/scenario_spec.hpp"

namespace mobile_manipulator_navigation
{
struct FootprintUpdate
{
  Polygon footprint;           // padded hull, base_footprint
  std::vector<Polygon> zones;  // offset_outward(footprint, margin) per zone margin
};

struct TickStats
{
  double compute_s = 0.0;
  size_t points = 0;
  size_t vertices = 0;
  bool beyond_inflation = false;  // circumscribed radius above the costmap inflation radius
};

class DynamicFootprint
{
public:
  enum class Health { NoData, Fresh, Stale, MissingJoint };

  DynamicFootprint(std::shared_ptr<const mobile_manipulator_geometry::FootprintModel> model,
                   std::vector<std::string> arm_joints, std::vector<double> zone_margins_m, double padding_m,
                   double change_threshold_m, double stale_after_s, double inflation_radius_m);

  // Merge a (possibly partial) joint state received at `receive_time`.
  void joints(double receive_time, const std::vector<std::string> & names, const std::vector<double> & positions);
  // The footprint and zones to publish, or nothing (no change, missing or stale joints).
  std::optional<FootprintUpdate> tick(double now);
  const std::optional<Polygon> & current() const { return current_; }
  Health health(double now) const;
  const TickStats & last_stats() const { return stats_; }

private:
  std::shared_ptr<const mobile_manipulator_geometry::FootprintModel> model_;
  std::vector<std::string> arm_joints_;
  std::vector<double> zone_margins_m_;
  double padding_m_, change_threshold_m_, stale_after_s_, inflation_radius_m_;
  bool any_message_ = false;
  std::map<std::string, double> positions_;
  std::map<std::string, double> received_;
  std::optional<Polygon> current_;
  TickStats stats_;
};
}  // namespace mobile_manipulator_navigation
