#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

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

TEST(References, PostureTargetsComeFromQualifiedPoses)
{
  const auto targets = posture_targets(payload());
  EXPECT_EQ(targets.home, (Eigen::Matrix<double, 6, 1>::Zero().eval()));
  Eigen::Matrix<double, 6, 1> vertical;
  vertical << M_PI / 2, 0, 0, 0, M_PI / 2, 0;
  EXPECT_LT((targets.tight - vertical).norm(), 1e-12);
}
