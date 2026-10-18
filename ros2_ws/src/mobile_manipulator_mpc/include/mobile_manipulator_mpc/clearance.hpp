#pragma once
// Minimum 3D clearance of a recorded run: the robot's collision primitives (the MPC's
// CollisionModel, without margin) against the known boxes over a series of robot states. Scores
// B1-B5 alike; the 2D footprint clearance of the navigate task cannot see an overhead beam or the
// arm and panel.
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mobile_manipulator_mpc/collision.hpp"

namespace mobile_manipulator_mpc
{
struct ClearanceSample
{
  double time_s;  // simulation time of the state
  State state;    // base_footprint pose in map and the six arm joints (measured)
  // The JTC's commanded reference at that time (model joint order), when the bag has one.
  std::optional<Eigen::Matrix<double, 6, 1>> reference = std::nullopt;
};

struct Clearance
{
  double distance_m = std::numeric_limits<double>::infinity();  // signed; infinity without a pair
  std::string link, obstacle;                                     // at the minimum
  double time_s = std::numeric_limits<double>::quiet_NaN();       // sample time of the minimum
  size_t samples = 0;                                             // states evaluated
};

Clearance minimum_clearance(const CollisionModel & model, const std::vector<ClearanceSample> & samples,
                            const std::vector<ObstacleBox> & boxes);

// Phase 4B-A spec Section 5: does the commanded-to-measured arm lag explain clearance below the margin?
struct ShortfallVerdict
{
  size_t samples_below_margin = 0;  // samples whose measured clearance is below the margin
  double worst_shortfall_m = 0.0;   // largest margin - measured
  double covered_fraction = 1.0;    // smallest (reference - measured) / (margin - measured) over them
  // Every sample below the margin has a reference clearance of at least margin - 0.001 and a
  // difference covering at least 80 % of its shortfall (false without any sample below the margin).
  bool explained = false;
};
// `pairs`: (measured, reference) clearance per sample.
ShortfallVerdict judge_shortfall(const std::vector<std::pair<double, double>> & pairs, double margin_m);

struct ReferenceComparison
{
  double measured_min = std::numeric_limits<double>::infinity();  // over samples with a reference
  double reference_at_measured_min = std::numeric_limits<double>::infinity();
  double time_s = std::numeric_limits<double>::quiet_NaN();       // of the measured minimum
  Eigen::Matrix<double, 6, 1> lag_at_measured_min = Eigen::Matrix<double, 6, 1>::Zero();  // reference - measured
  size_t samples_with_reference = 0;
  ShortfallVerdict verdict;
};
// The minimum clearance of each sample with a reference, once with the measured joints and once with
// the reference joints at the same base pose; samples without a reference are skipped.
ReferenceComparison compare_reference(const CollisionModel & model, const std::vector<ClearanceSample> & samples,
                                      const std::vector<ObstacleBox> & boxes, double margin_m);
}  // namespace mobile_manipulator_mpc
