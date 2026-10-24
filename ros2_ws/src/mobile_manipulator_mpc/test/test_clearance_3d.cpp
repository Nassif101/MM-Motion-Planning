// Minimum 3D clearance between the robot primitives and the known boxes (clearance_3d): the
// pure computation against CollisionModel's own distance rows, and the bag reader on a small
// synthetic bag.
#include <gtest/gtest.h>

#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "mobile_manipulator_mpc/clearance.hpp"
#include "mobile_manipulator_mpc/clearance_bag.hpp"
#include "test_support.hpp"

using namespace mpc_test;
namespace mmn = mobile_manipulator_navigation;
using Eigen::Vector3d;

namespace
{
Joints pose(const std::string & name)
{
  Joints q;
  for (int i = 0; i < 6; ++i) q(i) = payload()["poses_rad"][name][i].get<double>();
  return q;
}

CollisionModel unlimited_model()
{
  CollisionConfig config;
  config.pairs_per_stage = 1000;
  return CollisionModel(primitives(), config);
}

mmn::Box box(const std::string & name, const Vector3d & center, const Vector3d & size)
{
  return mmn::Box{name, center, size, 0.0};
}

std::vector<ObstacleBox> boxes_of(const std::vector<mmn::Box> & boxes)
{
  return world_of(boxes).at(0.0, 0.0);
}

// The closest distance of CollisionModel's rows to a scenario obstacle (no margin: the rows carry
// the plain signed distance).
double row_minimum(const CollisionModel & model, const State & x, const std::vector<mmn::Box> & boxes,
                   std::string * obstacle)
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto & row : model.rows({x}, 0.1, 0.0, world_of(boxes), nullptr)) {
    if (row.obstacle == "floor" || row.obstacle.rfind("self/", 0) == 0 || row.distance >= best) continue;
    best = row.distance;
    *obstacle = row.obstacle;
  }
  return best;
}
}  // namespace

TEST(Clearance3d, MatchesCollisionModelDistance)
{
  // The 1.05 m gate posts (0.4 m wide, 2.4 m high) and a wall piece, approached along the gate.
  const std::vector<mmn::Box> boxes = {box("left", Vector3d(-7.225, -7.0, 1.2), Vector3d(0.4, 0.4, 2.4)),
                                       box("right", Vector3d(-7.225, -8.45, 1.2), Vector3d(0.4, 0.4, 2.4)),
                                       box("wall", Vector3d(-5.0, -7.7, 1.0), Vector3d(0.2, 4.0, 2.0))};
  const CollisionModel model = unlimited_model();
  const std::vector<ClearanceSample> samples = {
    {10.0, state(-10.0, -7.7, 0.0, pose("home"))},
    {10.5, state(-8.4, -7.7, 0.0, pose("vertical_carry"))},
    {11.0, state(-7.6, -7.65, 0.2, pose("vertical_carry"))},
    {11.5, state(-6.4, -7.7, 0.0, pose("home"))}};

  const Clearance result = minimum_clearance(model, samples, boxes_of(boxes));

  double expected = std::numeric_limits<double>::infinity();
  size_t expected_index = 0;
  std::string expected_obstacle;
  for (size_t i = 0; i < samples.size(); ++i) {
    std::string obstacle;
    const double d = row_minimum(model, samples[i].state, boxes, &obstacle);
    if (d < expected) {
      expected = d;
      expected_index = i;
      expected_obstacle = obstacle;
    }
  }
  ASSERT_TRUE(std::isfinite(expected));
  EXPECT_NEAR(result.distance_m, expected, 1e-9);
  EXPECT_EQ(result.obstacle, expected_obstacle);
  EXPECT_DOUBLE_EQ(result.time_s, samples[expected_index].time_s);
  EXPECT_EQ(result.samples, samples.size());
  EXPECT_FALSE(result.link.empty());
  // The same value per state through the model's own nearest pair.
  const auto nearest = model.nearest(samples[expected_index].state, boxes_of(boxes));
  ASSERT_TRUE(nearest.has_value());
  EXPECT_NEAR(nearest->distance, expected, 1e-9);
  EXPECT_EQ(nearest->link, result.link);
}

TEST(Clearance3d, PenetrationIsNegative)
{
  const std::vector<mmn::Box> boxes = {box("pillar", Vector3d(0.3, 0.0, 1.0), Vector3d(0.4, 0.4, 2.0))};
  const Clearance result = minimum_clearance(unlimited_model(), {{1.0, state(0.0, 0.0, 0.0, pose("home"))}},
                                             boxes_of(boxes));
  EXPECT_LT(result.distance_m, 0.0);
  EXPECT_EQ(result.obstacle, "pillar");
}

TEST(Clearance3d, AnOverheadBeamIsMeasuredAboveTheRobot)
{
  // A beam over the whole robot: the base primitives pass under it, so the rows (which skip pairs
  // the base cannot reach by driving) do not list it, but the 3D clearance is the vertical gap.
  const CollisionModel model = unlimited_model();
  const State x = state(0.0, 0.0, 0.0, pose("vertical_carry"));
  const auto at = [&](double bottom) {
    return minimum_clearance(model, {{2.0, x}},
                             boxes_of({box("beam", Vector3d(0.0, 0.0, bottom + 0.1), Vector3d(4.0, 4.0, 0.2))}));
  };
  const Clearance low = at(3.0), high = at(3.5);
  EXPECT_GT(low.distance_m, 0.0);
  EXPECT_NEAR(high.distance_m - low.distance_m, 0.5, 1e-6);
  EXPECT_EQ(low.obstacle, "beam");
  EXPECT_EQ(low.link, high.link);
}

TEST(Clearance3d, NoSamplesOrNoBoxesGiveNoDistance)
{
  const CollisionModel model = unlimited_model();
  const auto beam = boxes_of({box("beam", Vector3d(0.0, 0.0, 3.1), Vector3d(4.0, 4.0, 0.2))});
  EXPECT_FALSE(std::isfinite(minimum_clearance(model, {}, beam).distance_m));
  EXPECT_FALSE(std::isfinite(minimum_clearance(model, {{0.0, state(0, 0, 0, pose("home"))}}, {}).distance_m));
}

namespace
{
geometry_msgs::msg::TransformStamped transform(const std::string & parent, const std::string & child, double x,
                                               double y, double yaw)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.rotation.z = std::sin(yaw / 2.0);
  t.transform.rotation.w = std::cos(yaw / 2.0);
  return t;
}

sensor_msgs::msg::JointState joint_state(double stamp, const std::array<std::string, 6> & order, const Joints & q)
{
  sensor_msgs::msg::JointState m;
  m.header.stamp.sec = static_cast<int32_t>(stamp);
  m.header.stamp.nanosec = static_cast<uint32_t>((stamp - std::floor(stamp)) * 1e9);
  // Reversed order and an unrelated joint: names, not positions in the message, give the joint.
  m.name.push_back("front_left_wheel_joint");
  m.position.push_back(7.0);
  for (int i = 5; i >= 0; --i) {
    m.name.push_back(order[static_cast<size_t>(i)]);
    m.position.push_back(q(i));
  }
  return m;
}
}  // namespace

TEST(Clearance3d, BagReaderBuildsStatesFromTfAndJointStates)
{
  const auto uri = std::filesystem::temp_directory_path() / ("clearance_3d_bag_" + std::to_string(::getpid()));
  std::filesystem::remove_all(uri);
  std::array<std::string, 6> order;
  for (int i = 0; i < 6; ++i) order[static_cast<size_t>(i)] = payload()["joint_order"][i].get<std::string>();
  const Joints q = pose("vertical_carry");
  {
    rosbag2_cpp::Writer writer;
    writer.open(uri.string());
    const auto ns = [](double s) { return rclcpp::Time(static_cast<int64_t>(s * 1e9)); };
    // A joint state before any transform exists is skipped; map -> odom is static.
    writer.write(joint_state(0.5, order, q), "/joint_states", ns(0.5));
    tf2_msgs::msg::TFMessage fixed;
    fixed.transforms.push_back(transform("map", "odom", 2.0, 0.0, 0.0));
    writer.write(fixed, "/tf_static", ns(0.6));
    tf2_msgs::msg::TFMessage moving;
    moving.transforms.push_back(transform("odom", "base_footprint", 1.0, 0.5, M_PI / 2.0));
    moving.transforms.push_back(transform("base_footprint", "base_link", 0.0, 0.0, 0.0));
    writer.write(moving, "/tf", ns(1.0));
    writer.write(joint_state(1.01, order, q), "/joint_states", ns(1.01));
    writer.write(joint_state(1.02, order, q), "/joint_states", ns(1.02));  // inside the 0.05 s period
    tf2_msgs::msg::TFMessage later;
    later.transforms.push_back(transform("odom", "base_footprint", 1.5, 0.5, M_PI / 2.0));
    writer.write(later, "/tf", ns(1.05));
    writer.write(joint_state(1.10, order, q), "/joint_states", ns(1.10));
  }
  const auto samples = read_clearance_samples(uri.string(), order, 0.05);
  std::filesystem::remove_all(uri);

  ASSERT_EQ(samples.size(), 2u);
  EXPECT_NEAR(samples[0].time_s, 1.01, 1e-6);
  EXPECT_NEAR(samples[1].time_s, 1.10, 1e-6);
  const State first = state(3.0, 0.5, M_PI / 2.0, q), second = state(3.5, 0.5, M_PI / 2.0, q);
  EXPECT_LT((samples[0].state - first).norm(), 1e-9);
  EXPECT_LT((samples[1].state - second).norm(), 1e-9);
}

TEST(CompareReference, LagExplainsShortfall)
{
  // Measured 0.045 m (0.005 short of the 0.05 margin), the reference 0.006 m farther.
  const ShortfallVerdict v = judge_shortfall({{0.20, 0.20}, {0.045, 0.051}, {0.048, 0.0505}}, 0.05);
  EXPECT_EQ(v.samples_below_margin, 2u);
  EXPECT_TRUE(v.explained);
  EXPECT_NEAR(v.worst_shortfall_m, 0.005, 1e-12);
  EXPECT_GE(v.covered_fraction, 0.8);
}

TEST(CompareReference, NoLagDoesNotExplain)
{
  const ShortfallVerdict v = judge_shortfall({{0.045, 0.045}}, 0.05);
  EXPECT_FALSE(v.explained);
  EXPECT_NEAR(v.covered_fraction, 0.0, 1e-12);
}

TEST(CompareReference, ReferenceBelowTheMarginDoesNotExplain)
{
  // The difference covers 80 % but the reference itself is below 0.049 m.
  EXPECT_FALSE(judge_shortfall({{0.030, 0.047}}, 0.05).explained);
}

TEST(CompareReference, NoShortfallIsNotExplained)
{
  const ShortfallVerdict v = judge_shortfall({{0.10, 0.10}}, 0.05);
  EXPECT_EQ(v.samples_below_margin, 0u);
  EXPECT_FALSE(v.explained);
}

TEST(CompareReference, UsesTheReferenceJointsAtTheSameBasePose)
{
  // A beam over the robot: the measured arm in home, the reference in vertical carry (taller).
  const CollisionModel model = unlimited_model();
  const auto beam = boxes_of({box("beam", Vector3d(0.0, 0.0, 2.6), Vector3d(4.0, 4.0, 0.2))});
  ClearanceSample sample{1.0, state(0.0, 0.0, 0.0, pose("home"))};
  sample.reference = pose("vertical_carry");
  const ReferenceComparison c = compare_reference(model, {sample}, beam, 0.05);
  const double measured = minimum_clearance(model, {{1.0, state(0.0, 0.0, 0.0, pose("home"))}}, beam).distance_m;
  const double reference =
    minimum_clearance(model, {{1.0, state(0.0, 0.0, 0.0, pose("vertical_carry"))}}, beam).distance_m;
  EXPECT_NEAR(c.measured_min, measured, 1e-9);
  EXPECT_NEAR(c.reference_at_measured_min, reference, 1e-9);
  EXPECT_LT(c.reference_at_measured_min, c.measured_min);
}

// The commanded reference's own minimum over the run, whenever it happens (Phase 4B-B shortfall analysis).
TEST(CompareReference, ReportsTheReferencesOwnMinimum)
{
  const CollisionModel model = unlimited_model();
  const auto beam = boxes_of({box("beam", Vector3d(0.0, 0.0, 2.6), Vector3d(4.0, 4.0, 0.2))});
  ClearanceSample low{1.0, state(0.0, 0.0, 0.0, pose("vertical_carry"))};  // the measured minimum
  low.reference = pose("home");
  ClearanceSample high{2.0, state(0.0, 0.0, 0.0, pose("home"))};
  high.reference = pose("vertical_carry");                                   // the reference's minimum
  const ReferenceComparison c = compare_reference(model, {low, high}, beam, 0.05);
  const double vertical =
    minimum_clearance(model, {{1.0, state(0.0, 0.0, 0.0, pose("vertical_carry"))}}, beam).distance_m;
  EXPECT_NEAR(c.reference_min, vertical, 1e-9);
  EXPECT_DOUBLE_EQ(c.reference_min_time_s, 2.0);
  EXPECT_DOUBLE_EQ(c.time_s, 1.0);
}

TEST(CompareReference, SamplesWithoutReferenceAreNotCompared)
{
  const CollisionModel model = unlimited_model();
  const auto pillar = boxes_of({box("pillar", Vector3d(0.3, 0.0, 1.0), Vector3d(0.4, 0.4, 2.0))});
  const ReferenceComparison c = compare_reference(model, {{1.0, state(0.0, 0.0, 0.0, pose("home"))}}, pillar, 0.05);
  EXPECT_EQ(c.samples_with_reference, 0u);
  EXPECT_FALSE(c.verdict.explained);
}

TEST(ClearanceBag, ReadsControllerStateReference)
{
  const auto uri = std::filesystem::temp_directory_path() / ("clearance_ref_bag_" + std::to_string(::getpid()));
  std::filesystem::remove_all(uri);
  std::array<std::string, 6> order;
  for (int i = 0; i < 6; ++i) order[static_cast<size_t>(i)] = payload()["joint_order"][i].get<std::string>();
  const Joints q = pose("home"), r = pose("carry");
  {
    rosbag2_cpp::Writer writer;
    writer.open(uri.string());
    const auto ns = [](double s) { return rclcpp::Time(static_cast<int64_t>(s * 1e9)); };
    tf2_msgs::msg::TFMessage fixed;
    fixed.transforms.push_back(transform("map", "base_footprint", 0.0, 0.0, 0.0));
    writer.write(fixed, "/tf_static", ns(0.1));
    writer.write(joint_state(0.9, order, q), "/joint_states", ns(0.9));  // before any reference
    control_msgs::msg::JointTrajectoryControllerState state_msg;
    for (int i = 5; i >= 0; --i) {  // names, not positions, give the joint
      state_msg.joint_names.push_back(order[static_cast<size_t>(i)]);
      state_msg.reference.positions.push_back(r(i));
    }
    writer.write(state_msg, "/arm_controller/controller_state", ns(0.95));
    writer.write(joint_state(1.0, order, q), "/joint_states", ns(1.0));
  }
  const auto samples = read_clearance_samples(uri.string(), order, 0.05);
  std::filesystem::remove_all(uri);
  ASSERT_EQ(samples.size(), 2u);
  EXPECT_FALSE(samples[0].reference.has_value());
  ASSERT_TRUE(samples[1].reference.has_value());
  EXPECT_LT((*samples[1].reference - r).norm(), 1e-9);
  EXPECT_LT((samples[1].state.tail<6>() - q).norm(), 1e-9);
}

TEST(Clearance3d, ReportsTheSelfMinimum)
{
  // Phase 4B-B: the panel against the configured arm links over the samples, the smallest with its link and time.
  CollisionConfig c;
  c.self_links = {"forearm_link", "shoulder_pan_link", "upper_arm_link", "wrist_1_link"};
  const CollisionModel model(primitives(), c);
  std::vector<ClearanceSample> samples;
  auto pose = [](const char * name) {
    const auto q = payload().at("poses_rad").at(name).get<std::vector<double>>();
    return Eigen::Map<const Eigen::Matrix<double, 6, 1>>(q.data()).eval();
  };
  samples.push_back({1.0, state(0, 0, 0, pose("carry"))});
  samples.push_back({2.0, state(0, 0, 0, pose("vertical_carry"))});
  const Clearance self = minimum_self_clearance(model, samples);
  const auto expected = model.nearest_self(samples[1].state);
  ASSERT_TRUE(expected.has_value());
  EXPECT_NEAR(self.distance_m, std::min(model.nearest_self(samples[0].state)->distance, expected->distance), 1e-12);
  EXPECT_EQ(self.samples, 2u);
  EXPECT_FALSE(self.obstacle.empty());
  EXPECT_EQ(self.link, "panel");
  EXPECT_FALSE(std::isfinite(minimum_self_clearance(CollisionModel(primitives(), CollisionConfig{}), samples).distance_m))
    << "no self links configured";
}
