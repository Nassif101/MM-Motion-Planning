#pragma once
// Spec 2 Task 10: the plant's scans as the routing core's observations. The production observation carries no ground
// truth labels; the plant's per-point geom and class go to ScanLabels, for validation and diagnostics only.
#include <cstdint>
#include <deque>
#include <vector>

#include "mobile_manipulator_plant/lidar.hpp"
#include "mobile_manipulator_routing/observation.hpp"

namespace mobile_manipulator_campaign
{
// Validation-only labels, aligned with the adapted observation's points.
struct ScanLabels
{
  std::vector<int> geom;
  std::vector<mobile_manipulator_plant::HitClass> cls;
};

struct AdaptedScan
{
  mobile_manipulator_routing::LidarObservation observation;
  ScanLabels labels;
  std::size_t dropped_batches = 0, dropped_points = 0;  // batches without an acquisition state (never substituted)
};

// A plant scan drained at simulation time `t_received`. A batch without its acquisition state is dropped with its
// points, never given another time's pose.
AdaptedScan adapt_scan(const mobile_manipulator_plant::PlantScan & scan, double t_received);

// Session-side accounting and the bounded hand-off to the observation consumers (the perception phase's filter and
// layer: plan Task 11, deferred on 2026-11-19).
struct ObservationCounts
{
  std::uint64_t scans = 0, points = 0;        // accepted
  std::uint64_t missing_state_batches = 0, missing_state_points = 0;
  std::uint64_t out_of_order = 0;             // refused: not newer than the last accepted scan
  std::uint64_t overflow = 0;                 // dropped from a full pending queue (a consumer not draining)
};

// Every drained scan once, in acquisition order: observations newer than the last accepted one are queued for the
// consumer (at most `capacity`; the oldest is dropped and counted beyond it); older or equal ones are refused.
class ObservationQueue
{
public:
  explicit ObservationQueue(std::size_t capacity = 64) : capacity_(capacity) {}
  // true when accepted
  bool push(AdaptedScan scan);
  std::vector<mobile_manipulator_routing::LidarObservation> take();
  const ObservationCounts & counts() const { return counts_; }
  void reset();

private:
  std::size_t capacity_;
  std::deque<mobile_manipulator_routing::LidarObservation> pending_;
  ObservationCounts counts_;
  double last_t_end_ = -1.0;
  bool any_ = false;
};
}  // namespace mobile_manipulator_campaign
