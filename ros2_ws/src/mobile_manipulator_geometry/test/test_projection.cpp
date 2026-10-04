#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mobile_manipulator_geometry/footprint_projection.hpp"

namespace mmg = mobile_manipulator_geometry;

namespace
{
std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

const nlohmann::ordered_json & qualified_payload()
{
  static const auto json = nlohmann::ordered_json::parse(read_file(
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

size_t point_count(const std::vector<mmg::Primitive> & primitives, int sides, bool payload)
{
  size_t count = 0;
  for (size_t i = 0; i < primitives.size(); ++i) {
    if (!payload && i + 1 == primitives.size()) break;  // the payload is the last primitive
    count += primitives[i].shape == mmg::Shape::Box ? 8 : 2 * sides;
  }
  return count;
}
}  // namespace

TEST(Projection, DefaultOptionsMatchTheOneArgumentOverload)
{
  const auto joints = pose("home");
  const auto legacy = projector().projected_points(joints);
  const auto options = projector().projected_points(joints, mmg::ProjectionOptions{});
  ASSERT_EQ(legacy.size(), options.size());
  for (size_t i = 0; i < legacy.size(); ++i) {
    EXPECT_EQ(legacy[i], options[i]);
  }
}

TEST(Projection, CylinderSidesSetThePointCount)
{
  const auto & primitives = projector().primitives();
  EXPECT_EQ(projector().projected_points(pose("home"), {16, {}, true}).size(), point_count(primitives, 16, true));
  EXPECT_EQ(projector().projected_points(pose("home"), {64, {}, true}).size(), point_count(primitives, 64, true));
}

TEST(Projection, LinkFilterWithoutPayloadKeepsOnlyThoseLinks)
{
  // The base box and wheels lie within x +/-0.44 m, y +/-0.365 m (test_footprint_profiles.py
  // bounds of the non-arm links); the panel in home reaches y +/-0.60 m.
  const mmg::ProjectionOptions base{64,
                                    {"base_link", "front_left_wheel_link", "front_right_wheel_link",
                                     "rear_left_wheel_link", "rear_right_wheel_link", "arm_mount_link"},
                                    false};
  const auto points = projector().projected_points(pose("home"), base);
  ASSERT_FALSE(points.empty());
  double x = 0.0, y = 0.0;
  for (const auto & p : points) {
    x = std::max(x, std::abs(p[0]));
    y = std::max(y, std::abs(p[1]));
  }
  EXPECT_NEAR(x, 0.440, 0.002);
  EXPECT_NEAR(y, 0.365, 0.002);
}

TEST(Projection, PayloadSwitchAddsThePanelCorners)
{
  const auto with = projector().projected_points(pose("home"), {64, {}, true});
  const auto without = projector().projected_points(pose("home"), {64, {}, false});
  EXPECT_EQ(with.size(), without.size() + 8);
}
