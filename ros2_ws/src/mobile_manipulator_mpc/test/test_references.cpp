#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mobile_manipulator_mpc/model.hpp"
#include "mobile_manipulator_mpc/references.hpp"

namespace mmg = mobile_manipulator_geometry;
namespace mmn = mobile_manipulator_navigation;
using namespace mobile_manipulator_mpc;
using Eigen::Vector2d;

namespace
{
std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

const Json & payload()
{
  static const Json json = Json::parse(read_file(
    ament_index_cpp::get_package_share_directory("mobile_manipulator_control") +
    "/config/qualified_payload.json"));
  return json;
}

const mmg::FootprintProjector & projector()
{
  static const mmg::FootprintProjector instance(
    read_file(ament_index_cpp::get_package_share_directory("mobile_manipulator_description") +
              "/urdf/mobile_manipulator.urdf"),
    mmg::payload_from_json(payload()));
  return instance;
}

Json scenario(const std::string & name)
{
  static const Json all = mmn::load_yaml_file(
    ament_index_cpp::get_package_share_directory("mobile_manipulator_navigation") + "/config/scenarios.yaml");
  return all.at("scenarios").at(name);
}

mmg::JointMap joints_of(const std::string & pose)
{
  mmg::JointMap joints;
  for (int i = 0; i < 6; ++i) {
    joints[payload().at("joint_order").at(i).get<std::string>()] = payload().at("poses_rad").at(pose).at(i).get<double>();
  }
  return joints;
}

double rotation_gap(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b)
{
  return Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle();
}
}  // namespace

TEST(References, WeightRampsByRouteProgress)
{
  const PanelReference r{Eigen::Isometry3d::Identity(), 10.0, 20.0, 2.0};
  EXPECT_DOUBLE_EQ(reference_weight(r, 8.0), 0.0);
  EXPECT_DOUBLE_EQ(reference_weight(r, 5.0), 0.0);
  EXPECT_DOUBLE_EQ(reference_weight(r, 9.0), 0.5);
  EXPECT_DOUBLE_EQ(reference_weight(r, 10.0), 1.0);
  EXPECT_DOUBLE_EQ(reference_weight(r, 15.0), 1.0);
  EXPECT_DOUBLE_EQ(reference_weight(r, 20.0), 1.0);
  EXPECT_DOUBLE_EQ(reference_weight(r, 21.0), 0.5);
  EXPECT_DOUBLE_EQ(reference_weight(r, 22.0), 0.0);
  EXPECT_DOUBLE_EQ(reference_weight(r, 30.0), 0.0);
}

TEST(References, NarrowGateMissionGivesVerticalThenHome)
{
  const auto scenario_json = scenario("narrow_gate_mission");
  const auto refs = mission_references(scenario_json, projector(), payload());
  ASSERT_EQ(refs.size(), 2u);
  EXPECT_EQ(refs[0].start, Vector2d(-4.2, -7.725));
  EXPECT_EQ(refs[0].end, Vector2d(-9.0, -7.725));
  EXPECT_EQ(refs[1].start, Vector2d(-9.0, -7.725));
  EXPECT_EQ(refs[1].end, Vector2d(-11.0, -7.725));

  // The first reference is the mission's panel_pose, which is the qualified vertical carry.
  const auto & pose = scenario_json.at("steps").at(1).at("reconfigure").at("panel_pose");
  EXPECT_NEAR(refs[0].panel_in_base.translation().x(), pose.at("xyz").at(0).get<double>(), 1e-12);
  EXPECT_NEAR(refs[0].panel_in_base.translation().y(), pose.at("xyz").at(1).get<double>(), 1e-12);
  EXPECT_NEAR(refs[0].panel_in_base.translation().z(), pose.at("xyz").at(2).get<double>(), 1e-12);
  const auto vertical = projector().panel_pose(joints_of("vertical_carry"));
  EXPECT_LT((refs[0].panel_in_base.translation() - vertical.translation()).norm(), 1e-3);
  EXPECT_LT(rotation_gap(refs[0].panel_in_base, vertical), 1e-3);
  // The second is home.
  const auto home = projector().panel_pose(joints_of("home"));
  EXPECT_LT((refs[1].panel_in_base.translation() - home.translation()).norm(), 1e-9);
  EXPECT_LT(rotation_gap(refs[1].panel_in_base, home), 1e-9);
}

TEST(References, NamedStateUsesQualifiedPose)
{
  Json spec = Json::parse(R"({
    "start": [0.0, 0.0, 0.0], "goal": [6.0, 0.0, 0.0],
    "steps": [
      {"reconfigure": {"named_state": "level_extension", "footprint_profile": "home"}},
      {"navigate": [3.0, 0.0, 0.0]},
      {"navigate": [5.0, 0.0, 0.0]},
      {"reconfigure": {"named_state": "home", "footprint_profile": "home"}}
    ]})");
  const auto refs = mission_references(spec, projector(), payload());
  ASSERT_EQ(refs.size(), 2u);
  EXPECT_EQ(refs[0].start, Vector2d(0, 0));  // no drive before it: the scenario start
  EXPECT_EQ(refs[0].end, Vector2d(5, 0));
  EXPECT_EQ(refs[1].start, Vector2d(5, 0));
  EXPECT_EQ(refs[1].end, Vector2d(6, 0));
  const auto level = projector().panel_pose(joints_of("level_extension"));
  EXPECT_LT((refs[0].panel_in_base.translation() - level.translation()).norm(), 1e-9);
  EXPECT_LT(rotation_gap(refs[0].panel_in_base, level), 1e-9);

  spec["steps"][0]["reconfigure"]["named_state"] = "no_such_pose";
  EXPECT_THROW(mission_references(spec, projector(), payload()), std::invalid_argument);

  // The reference reproduces the model's own panel pose for that joint vector.
  State x = State::Zero();
  for (int i = 0; i < 6; ++i) x(3 + i) = payload().at("poses_rad").at("level_extension").at(i).get<double>();
  EXPECT_LT(panel_error(x, refs[0].panel_in_base).e.norm(), 1e-6);
}

TEST(References, AnchorsOnRouteGiveArcLengths)
{
  const Route route({Vector2d(0, 0), Vector2d(10, 0), Vector2d(10, 5)});
  const std::vector<AnchoredReference> anchored{
    {Eigen::Isometry3d::Identity(), Vector2d(2.0, 0.1), Vector2d(10.0, 3.0)}};
  const auto refs = anchor_on_route(anchored, route, 1.5);
  ASSERT_EQ(refs.size(), 1u);
  EXPECT_NEAR(refs[0].start_s, 2.0, 1e-12);
  EXPECT_NEAR(refs[0].end_s, 13.0, 1e-12);
  EXPECT_DOUBLE_EQ(refs[0].ramp_m, 1.5);
  EXPECT_DOUBLE_EQ(anchor_on_route(anchored, route)[0].ramp_m, 1.5);
}

TEST(References, AnchorOutsideRouteThrows)
{
  const Route route({Vector2d(0, 0), Vector2d(10, 0)});
  const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  EXPECT_THROW(anchor_on_route({{pose, Vector2d(2, 0.6), Vector2d(8, 0)}}, route, 1.5), std::invalid_argument);
  EXPECT_THROW(anchor_on_route({{pose, Vector2d(2, 0), Vector2d(8, -0.6)}}, route, 1.5), std::invalid_argument);
  EXPECT_NO_THROW(anchor_on_route({{pose, Vector2d(2, 0.4), Vector2d(8, -0.4)}}, route, 1.5));
}

TEST(References, RemainingRouteDropsPassedReferences)
{
  // A path replanned from the robot's position part-way through the mission (FollowPath after a
  // recovery): anchors behind the route start are passed, not off the route.
  const Route route({Vector2d(5, 0), Vector2d(10, 0), Vector2d(10, 5)});
  const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  const std::vector<AnchoredReference> anchored{
    {pose, Vector2d(2, 0), Vector2d(4, 0)},     // over: dropped
    {pose, Vector2d(3, 0), Vector2d(10, 3)},    // running: active from the route start
    {pose, Vector2d(10, 1), Vector2d(10, 5)}};  // ahead: as anchor_on_route
  EXPECT_THROW(anchor_on_route(anchored, route, 1.5), std::invalid_argument);
  const auto refs = anchor_on_remaining_route(anchored, route, 1.5);
  ASSERT_EQ(refs.size(), 2u);
  EXPECT_DOUBLE_EQ(refs[0].start_s, 0.0);
  EXPECT_NEAR(refs[0].end_s, 8.0, 1e-12);
  EXPECT_DOUBLE_EQ(reference_weight(refs[0], 0.0), 1.0);
  EXPECT_NEAR(refs[1].start_s, 6.0, 1e-12);
  EXPECT_NEAR(refs[1].end_s, 10.0, 1e-12);
  EXPECT_DOUBLE_EQ(refs[1].ramp_m, 1.5);
  // The whole mission on its full route: the same as anchor_on_route.
  const Route full({Vector2d(0, 0), Vector2d(10, 0), Vector2d(10, 5)});
  const std::vector<AnchoredReference> whole{{pose, Vector2d(2, 0.1), Vector2d(10, 3)}};
  EXPECT_DOUBLE_EQ(anchor_on_remaining_route(whole, full)[0].start_s, anchor_on_route(whole, full)[0].start_s);
  EXPECT_DOUBLE_EQ(anchor_on_remaining_route(whole, full)[0].end_s, anchor_on_route(whole, full)[0].end_s);
  // Anchors beside the route ahead of its start still throw.
  EXPECT_THROW(anchor_on_remaining_route({{pose, Vector2d(8, 0.6), Vector2d(10, 3)}}, route), std::invalid_argument);
  EXPECT_THROW(anchor_on_remaining_route({{pose, Vector2d(3, 0), Vector2d(8, -0.6)}}, route), std::invalid_argument);
}

TEST(References, PostureTargetsComeFromQualifiedPoses)
{
  const auto targets = posture_targets(payload());
  EXPECT_EQ(targets.comfort, (Eigen::Matrix<double, 6, 1>::Zero().eval()));
  Eigen::Matrix<double, 6, 1> vertical;
  vertical << M_PI / 2, 0, 0, 0, M_PI / 2, 0;
  EXPECT_LT((targets.tight - vertical).norm(), 1e-12);
  EXPECT_EQ(targets.mode, PostureMode::Blend);  // the recorded B5
}

TEST(References, ComfortPoseIsHomeOrAQualifiedCarry)
{
  // The comfort pose is home or the bent carry, both taken from poses_rad.
  EXPECT_EQ(posture_targets(payload(), "home", PostureMode::ComfortOnly).mode, PostureMode::ComfortOnly);
  const auto carry = posture_targets(payload(), "carry");
  Eigen::Matrix<double, 6, 1> expected;
  expected << 0.0, 0.968, -1.735, 0.0, 0.767, 0.0;
  EXPECT_LT((carry.comfort - expected).norm(), 1e-12);
  EXPECT_LT((carry.tight - posture_targets(payload()).tight).norm(), 1e-12);
  // A payload without the pose is refused by name.
  Json without_carry = payload();
  without_carry["poses_rad"].erase("carry");
  try {
    posture_targets(without_carry, "carry");
    FAIL() << "a missing carry pose was accepted";
  } catch (const std::invalid_argument & error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("'carry'"), std::string::npos) << message;
    EXPECT_NE(message.find("poses_rad"), std::string::npos) << message;
  }
  // Only home and carry are comfort poses (vertical_carry is the tight attractor).
  EXPECT_THROW(posture_targets(payload(), "vertical_carry"), std::invalid_argument);
  EXPECT_THROW(posture_targets(payload(), "level_extension"), std::invalid_argument);
}

TEST(References, PostureModeNames)
{
  for (const auto mode : {PostureMode::Blend, PostureMode::ComfortOnly, PostureMode::None}) {
    EXPECT_EQ(parse_posture_mode(to_string(mode)), mode);
  }
  EXPECT_THROW(parse_posture_mode("comfort"), std::invalid_argument);
  EXPECT_THROW(parse_posture_mode(""), std::invalid_argument);
}

TEST(References, PayloadJointOrderMustBeTheModels)
{
  // R33-M7: the node, the posture targets and the limits use the model's joint order; a payload
  // in another order (two joints swapped, or a joint missing) fails fast.
  EXPECT_NO_THROW(require_model_joint_order(payload()));
  Json swapped = payload();
  std::swap(swapped["joint_order"][0], swapped["joint_order"][1]);
  EXPECT_THROW(require_model_joint_order(swapped), std::runtime_error);
  EXPECT_THROW(posture_targets(swapped), std::runtime_error);
  Json short_order = payload();
  short_order["joint_order"].erase(5);
  EXPECT_THROW(require_model_joint_order(short_order), std::runtime_error);
}

// Phase 4B-A spec Section 3: the tight poses the side picker chooses from, in the configured order.
TEST(References, TightPosesResolveInOrder)
{
  const auto targets = posture_targets(payload(), "carry", PostureMode::Blend, {"vertical_carry", "vertical_carry_right"});
  ASSERT_EQ(targets.tights.size(), 2u);
  EXPECT_NEAR(targets.tights[0](0), M_PI / 2, 1e-9);
  EXPECT_NEAR(targets.tights[1](0), -M_PI / 2, 1e-9);
  EXPECT_LT((targets.tight - targets.tights[0]).norm(), 1e-12);  // the first is the single tight pose
  const auto single = posture_targets(payload(), "carry");
  ASSERT_EQ(single.tights.size(), 1u);
  EXPECT_LT((single.tights[0] - single.tight).norm(), 1e-12);
}

TEST(References, UnknownTightPoseThrowsByName)
{
  try {
    posture_targets(payload(), "carry", PostureMode::Blend, {"vertical_carry", "sideways"});
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument & e) {
    EXPECT_NE(std::string(e.what()).find("sideways"), std::string::npos) << e.what();
  }
  EXPECT_THROW(posture_targets(payload(), "carry", PostureMode::Blend, {}), std::invalid_argument);
  // The side picker chooses between two (left, right): more is a configuration error.
  EXPECT_THROW(posture_targets(payload(), "carry", PostureMode::Blend, {"vertical_carry", "vertical_carry_right", "carry"}),
               std::invalid_argument);
}
