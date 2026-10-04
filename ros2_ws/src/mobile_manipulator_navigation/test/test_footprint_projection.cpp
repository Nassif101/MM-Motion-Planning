#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mobile_manipulator_geometry/footprint_projection.hpp"
#include "mobile_manipulator_navigation/scenario_spec.hpp"

namespace mmg = mobile_manipulator_geometry;
namespace mmn = mobile_manipulator_navigation;

namespace
{
std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

const mmn::Json & qualified_payload()
{
  static const auto json = mmn::Json::parse(read_file(
    ament_index_cpp::get_package_share_directory("mobile_manipulator_control") +
    "/config/qualified_payload.json"));
  return json;
}

const mmg::FootprintProjector & projector()
{
  static const mmg::FootprintProjector instance(
    read_file(ament_index_cpp::get_package_share_directory("mobile_manipulator_description") +
              "/urdf/mobile_manipulator.urdf"),
    mmg::payload_from_json(qualified_payload()));
  return instance;
}

mmg::JointMap pose(const std::string & name)
{
  mmg::JointMap joints;
  const auto & order = qualified_payload()["joint_order"];
  const auto & values = qualified_payload()["poses_rad"][name];
  for (size_t i = 0; i < order.size(); ++i) joints[order[i].get<std::string>()] = values[i].get<double>();
  return joints;
}

mmn::Polygon profile(const std::string & name)
{
  static const mmn::ScenarioConfig config(PACKAGE_SOURCE_DIR);
  return mmn::polygon_of(config.load("footprint_profiles.yaml")["profiles"][name]["polygon"]);
}
}  // namespace

TEST(Projection, NamedProfilesContainTheirPoseWithTheAllowance)
{
  for (const std::string name : {"home", "vertical_carry"}) {
    SCOPED_TRACE(name);
    const auto result = mmg::contains(profile(name), projector().projected_points(pose(name)));
    EXPECT_TRUE(result.inside);
    EXPECT_GE(result.margin_m, 0.019);
    EXPECT_LE(result.margin_m, 0.026);
  }
}

TEST(Projection, LevelExtensionDoesNotFitHome)
{
  const auto result = mmg::contains(profile("home"), projector().projected_points(pose("level_extension")));
  EXPECT_FALSE(result.inside);
  EXPECT_LT(result.margin_m, 0.0);
}

TEST(Projection, PanelOrientation)
{
  EXPECT_GT(std::abs(projector().panel_pose(pose("home")).linear()(2, 2)), 0.999);
  EXPECT_LT(std::abs(projector().panel_pose(pose("vertical_carry")).linear()(2, 2)), 1e-6);
}

TEST(Containment, UnitSquare)
{
  const mmn::Polygon square = {{1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
  const auto centre = mmg::contains(square, {{0.0, 0.0}});
  EXPECT_TRUE(centre.inside);
  EXPECT_NEAR(centre.margin_m, 1.0, 1e-12);
  const auto outside = mmg::contains(square, {{0.0, 0.0}, {1.5, 0.0}});
  EXPECT_FALSE(outside.inside);
  EXPECT_NEAR(outside.margin_m, -0.5, 1e-12);
}

TEST(Rpy, RoundTripsIncludingGimbalLock)
{
  const auto rebuild = [](const std::array<double, 3> & rpy) {
    return Eigen::Matrix3d((Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix());
  };
  for (const auto & input : std::vector<std::array<double, 3>>{
         {0.3, -0.4, 1.2}, {0.2, M_PI / 2, -0.7}, {-1.1, -M_PI / 2, 0.4}}) {
    const Eigen::Matrix3d rotation = rebuild(input);
    EXPECT_TRUE(rebuild(mmg::rpy_of(rotation)).isApprox(rotation, 1e-9));
  }
  const Eigen::Matrix3d panel = projector().panel_pose(pose("vertical_carry")).linear();
  EXPECT_TRUE(rebuild(mmg::rpy_of(panel)).isApprox(panel, 1e-9));
}
