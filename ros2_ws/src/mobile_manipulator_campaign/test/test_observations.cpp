// Spec 2 Task 10: acquisition-time observations and the session as the one scan owner.
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "mobile_manipulator_campaign/observation_adapter.hpp"
#include "mobile_manipulator_campaign/session.hpp"
#include "mobile_manipulator_plant/frames.hpp"

namespace mmc = mobile_manipulator_campaign;
namespace mmp = mobile_manipulator_plant;
namespace mr = mobile_manipulator_routing;

namespace
{
mmp::PlantScan synthetic_scan(double t0, bool known_middle = true)
{
  mmp::PlantScan s;
  s.scan = 3;
  s.t_start = t0;
  s.t_end = t0 + 0.1;
  s.rays = 9;
  for (int b = 0; b < 3; ++b) {
    mmp::CastBatch batch;
    batch.t = t0 + 0.02 * (b + 1);
    batch.first_point = static_cast<int>(s.points.size());
    batch.first_ray = 3 * b;
    batch.rays = 3;
    batch.sensor_position = Eigen::Vector3d(1.0 + b, 2.0, 0.5);
    batch.robot.known = b != 1 || known_middle;
    batch.robot.x = 1.0 + b;
    for (int i = 0; i < 2; ++i) {
      s.points.push_back(Eigen::Vector3f(1.0f + b, 0.1f * i, 0.0f));
      s.dt.push_back(static_cast<float>(batch.t - t0));
      s.ray.push_back(3 * b + i);
      s.geom.push_back(10 * b + i);
      s.cls.push_back(b == 2 ? mmp::HitClass::Panel : mmp::HitClass::Site);
    }
    batch.points = 2;
    s.batches.push_back(batch);
  }
  return s;
}

std::vector<mr::LidarObservation> run(mmc::SessionConfig c, int ticks, int & scan_frames)
{
  c.lidar = true;
  mmc::Session s(c);
  s.reset("v2_open_yard_nav", "", std::nullopt, 1);
  std::vector<mr::LidarObservation> out;
  scan_frames = 0;
  for (int k = 0; k < ticks && s.step_cycle(); ++k) {
    for (auto & o : s.take_observations()) out.push_back(std::move(o));
    for (const auto & f : s.take_frames()) {
      mmp::FrameType type{};
      nlohmann::json h;
      std::vector<std::vector<float>> b;
      size_t used = 0;
      if (mmp::decode(f, type, h, b, used) && type == mmp::FrameType::Scan) ++scan_frames;
    }
  }
  EXPECT_EQ(s.observation_counts().scans, out.size());
  EXPECT_EQ(s.observation_counts().missing_state_batches, 0u);
  return out;
}
}  // namespace

TEST(ObservationAdapter, ABatchWithoutItsAcquisitionStateIsDroppedNotSubstituted)
{
  const auto adapted = mmc::adapt_scan(synthetic_scan(1.0, false), 1.12);
  const auto & o = adapted.observation;
  EXPECT_EQ(adapted.dropped_batches, 1u);
  EXPECT_EQ(adapted.dropped_points, 2u);
  ASSERT_EQ(o.batches.size(), 2u);
  ASSERT_EQ(o.points.size(), 4u);
  EXPECT_EQ(o.batches[1].first_point, 2u);              // re-indexed after the gap
  EXPECT_DOUBLE_EQ(o.batches[1].robot.base.x, 3.0);     // the third batch keeps its own pose
  EXPECT_DOUBLE_EQ(o.batches[1].sensor.x, 3.0);
  EXPECT_FLOAT_EQ(o.points[2][0], 3.0f);
  EXPECT_DOUBLE_EQ(o.t_received, 1.12);
  // labels stay aligned with the kept points, outside the observation
  ASSERT_EQ(adapted.labels.geom.size(), 4u);
  EXPECT_EQ(adapted.labels.geom[2], 20);
  EXPECT_EQ(adapted.labels.cls[2], mmp::HitClass::Panel);
  EXPECT_NO_THROW(mr::validate(o));
}

TEST(ObservationAdapter, AMalformedPlantScanIsRefusedBeforeItIsRead)
{
  // Task 10 review M2: these used to read out of range, duplicate points or report a wrapped drop count.
  auto overlap = synthetic_scan(1.0);
  overlap.batches[1].first_point = 0;  // two batches selecting the same points
  EXPECT_THROW(mmc::adapt_scan(overlap, 1.2), std::invalid_argument);
  auto outside = synthetic_scan(1.0);
  outside.batches[2].first_point = 100;
  EXPECT_THROW(mmc::adapt_scan(outside, 1.2), std::invalid_argument);
  auto negative = synthetic_scan(1.0, false);
  negative.batches[1].points = -1;  // also on a batch that would be dropped
  EXPECT_THROW(mmc::adapt_scan(negative, 1.2), std::invalid_argument);
  auto unlabeled = synthetic_scan(1.0);
  unlabeled.geom.clear();
  EXPECT_THROW(mmc::adapt_scan(unlabeled, 1.2), std::invalid_argument);
  auto uncovered = synthetic_scan(1.0);
  uncovered.batches.pop_back();  // points outside every batch
  EXPECT_THROW(mmc::adapt_scan(uncovered, 1.2), std::invalid_argument);
  EXPECT_NO_THROW(mmc::adapt_scan(synthetic_scan(1.0, false), 1.2));  // a missing state is a drop, not an error
}

TEST(ObservationQueue, RefusesOutOfOrderScansAndBoundsWhatWaits)
{
  mmc::ObservationQueue q(2);
  EXPECT_TRUE(q.push(mmc::adapt_scan(synthetic_scan(1.0), 1.1)));
  EXPECT_FALSE(q.push(mmc::adapt_scan(synthetic_scan(1.0), 1.2)));  // not newer: refused
  EXPECT_FALSE(q.push(mmc::adapt_scan(synthetic_scan(0.5), 1.2)));  // older: refused
  EXPECT_TRUE(q.push(mmc::adapt_scan(synthetic_scan(1.1), 1.3)));
  EXPECT_TRUE(q.push(mmc::adapt_scan(synthetic_scan(1.2), 1.4)));   // the first is dropped from the full queue
  EXPECT_EQ(q.counts().out_of_order, 2u);
  EXPECT_EQ(q.counts().overflow, 1u);
  const auto taken = q.take();
  ASSERT_EQ(taken.size(), 2u);
  EXPECT_DOUBLE_EQ(taken[0].t_start, 1.1);
  EXPECT_TRUE(q.take().empty());
}

TEST(SessionObservations, MappingGetsEveryScanWhateverTheFrameDecimation)
{
  mmc::SessionConfig c;
  c.queue_frames = true;
  c.scan_decimation = 5;
  int frames = 0;
  const auto observations = run(c, 100, frames);  // 2 s: 20 scans at 10 Hz
  EXPECT_EQ(observations.size(), 20u);
  EXPECT_EQ(frames, 4);  // every fifth scan in the frames
  for (const auto & o : observations) {
    EXPECT_NO_THROW(mr::validate(o));
    EXPECT_FALSE(o.batches.empty());
    EXPECT_GT(o.points.size(), 1000u);
    EXPECT_GE(o.t_received, o.t_end);
  }
}

TEST(SessionObservations, AViewerOrItsDecimationDoesNotChangeTheObservations)
{
  mmc::SessionConfig streamed;
  streamed.queue_frames = true;
  streamed.scan_decimation = 3;
  mmc::SessionConfig headless;  // no frames queued, every scan in the (absent) frames
  int a = 0, b = 0;
  const auto with_viewer = run(streamed, 60, a), without = run(headless, 60, b);
  ASSERT_EQ(with_viewer.size(), without.size());
  for (size_t i = 0; i < without.size(); ++i) {
    ASSERT_EQ(with_viewer[i].points, without[i].points) << i;
    ASSERT_EQ(with_viewer[i].batches.size(), without[i].batches.size()) << i;
    for (size_t k = 0; k < without[i].batches.size(); ++k) {
      EXPECT_EQ(with_viewer[i].batches[k].t, without[i].batches[k].t);
      EXPECT_EQ(with_viewer[i].batches[k].sensor.x, without[i].batches[k].sensor.x);
      EXPECT_EQ(with_viewer[i].batches[k].robot.q, without[i].batches[k].robot.q);
    }
  }
}
