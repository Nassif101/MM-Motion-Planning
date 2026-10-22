#include "mobile_manipulator_mpc/clearance.hpp"

#include <algorithm>

namespace mobile_manipulator_mpc
{
Clearance minimum_clearance(const CollisionModel & model, const std::vector<ClearanceSample> & samples,
                            const std::vector<ObstacleBox> & boxes)
{
  Clearance result;
  for (const auto & sample : samples) {
    ++result.samples;
    const auto pair = model.nearest(sample.state, boxes);
    if (pair && pair->distance < result.distance_m) {
      result.distance_m = pair->distance;
      result.link = pair->link;
      result.obstacle = pair->obstacle;
      result.time_s = sample.time_s;
    }
  }
  return result;
}

ShortfallVerdict judge_shortfall(const std::vector<std::pair<double, double>> & pairs, double margin_m)
{
  constexpr double kReferenceTolerance = 0.001, kCovered = 0.8;
  ShortfallVerdict v;
  bool all = true;
  for (const auto & [measured, reference] : pairs) {
    if (!(measured < margin_m)) continue;
    ++v.samples_below_margin;
    const double shortfall = margin_m - measured;
    v.worst_shortfall_m = std::max(v.worst_shortfall_m, shortfall);
    const double covered = std::max(0.0, reference - measured) / shortfall;
    v.covered_fraction = std::min(v.covered_fraction, covered);
    all = all && reference >= margin_m - kReferenceTolerance && covered >= kCovered;
  }
  if (v.samples_below_margin == 0) v.covered_fraction = 1.0;
  v.explained = v.samples_below_margin > 0 && all;
  return v;
}

ReferenceComparison compare_reference(const CollisionModel & model, const std::vector<ClearanceSample> & samples,
                                      const std::vector<ObstacleBox> & boxes, double margin_m)
{
  ReferenceComparison c;
  std::vector<std::pair<double, double>> pairs;
  for (const auto & sample : samples) {
    if (!sample.reference) continue;
    const auto measured = model.nearest(sample.state, boxes);
    if (!measured) continue;
    State reference_state = sample.state;
    reference_state.tail<6>() = *sample.reference;
    const auto reference = model.nearest(reference_state, boxes);
    if (!reference) continue;
    ++c.samples_with_reference;
    pairs.emplace_back(measured->distance, reference->distance);
    if (measured->distance < c.measured_min) {
      c.measured_min = measured->distance;
      c.reference_at_measured_min = reference->distance;
      c.time_s = sample.time_s;
      c.lag_at_measured_min = *sample.reference - sample.state.tail<6>();
    }
  }
  c.verdict = judge_shortfall(pairs, margin_m);
  return c;
}
Clearance minimum_self_clearance(const CollisionModel & model, const std::vector<ClearanceSample> & samples)
{
  Clearance out;
  for (const auto & sample : samples) {
    const auto nearest = model.nearest_self(sample.state);
    if (!nearest) continue;
    ++out.samples;
    if (nearest->distance < out.distance_m) {
      out.distance_m = nearest->distance;
      out.link = nearest->link;
      out.obstacle = nearest->obstacle;
      out.time_s = sample.time_s;
    }
  }
  return out;
}
}  // namespace mobile_manipulator_mpc
