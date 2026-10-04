#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "mobile_manipulator_navigation/lidar_robot_filter.hpp"

namespace mmg = mobile_manipulator_geometry;
namespace mmn = mobile_manipulator_navigation;
using Eigen::Vector3d;

namespace
{
constexpr double kMargin = 0.03;

std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

std::vector<mmg::Primitive> robot_primitives()
{
  const auto description = ament_index_cpp::get_package_share_directory("mobile_manipulator_description");
  const auto control = ament_index_cpp::get_package_share_directory("mobile_manipulator_control");
  const auto payload = YAML::LoadFile(control + "/config/qualified_payload.json")["payload"];
  auto vec = [](const YAML::Node & n) { return Vector3d(n[0].as<double>(), n[1].as<double>(), n[2].as<double>()); };
  return mmg::load_primitives(read_file(description + "/urdf/mobile_manipulator.urdf"),
                              mmg::Payload{"tool0", vec(payload["dimensions_tool_ros_m"]),
                                           vec(payload["com_tool_ros_m"])});
}

Eigen::Isometry3d at(double x, double y, double z)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Vector3d(x, y, z);
  return pose;
}

// Links at the base origin except those given.
std::vector<mmn::PosedPrimitive> posed_at_origin(const std::map<std::string, Eigen::Isometry3d> & links)
{
  std::vector<mmn::PosedPrimitive> posed;
  for (const auto & p : robot_primitives()) {
    const auto link = links.count(p.link) ? links.at(p.link) : Eigen::Isometry3d::Identity();
    posed.push_back(mmn::posed_primitive(link * p.pose, p.shape, p.dims));
  }
  return posed;
}
}  // namespace

TEST(LidarRobotFilter, LoadsEveryCollisionPrimitivePlusThePanel)
{
  const auto primitives = robot_primitives();
  ASSERT_EQ(primitives.size(), 13u);
  const auto boxes = std::count_if(primitives.begin(), primitives.end(),
                                   [](const auto & p) { return p.shape == mmg::Shape::Box; });
  EXPECT_EQ(boxes, 4);
  const auto & panel = primitives.back();
  EXPECT_EQ(panel.link, "tool0");
  EXPECT_TRUE(panel.dims.isApprox(Vector3d(0.6, 0.6, 0.02)));
  EXPECT_TRUE(panel.pose.translation().isApprox(Vector3d(0, 0, 0.035)));
}

TEST(LidarRobotFilter, BoxAndCylinderMembershipWithMargin)
{
  const std::vector<mmn::PosedPrimitive> box{mmn::posed_primitive(at(1.0, 0.0, 0.0), mmg::Shape::Box, Vector3d(0.5, 0.2, 0.1))};
  EXPECT_TRUE(mmn::inside_any(Vector3d(1.52, 0, 0), box, 0.03));
  EXPECT_FALSE(mmn::inside_any(Vector3d(1.56, 0, 0), box, 0.03));
  EXPECT_TRUE(mmn::inside_any(Vector3d(1.0, 0.22, 0.12), box, 0.03));
  // Cylinder rolled by pi/2: axis along base y, radius in x/z, half length in y.
  Eigen::Isometry3d rolled = Eigen::Isometry3d::Identity();
  rolled.linear() = Eigen::AngleAxisd(M_PI / 2, Vector3d::UnitX()).toRotationMatrix();
  const std::vector<mmn::PosedPrimitive> cylinder{mmn::posed_primitive(rolled, mmg::Shape::Cylinder, Vector3d(0.14, 0.045, 0))};
  EXPECT_TRUE(mmn::inside_any(Vector3d(0.16, 0, 0), cylinder, 0.03));
  EXPECT_TRUE(mmn::inside_any(Vector3d(0, 0.07, 0), cylinder, 0.03));
  EXPECT_FALSE(mmn::inside_any(Vector3d(0, 0.08, 0), cylinder, 0.03));
}

TEST(LidarRobotFilter, ObstacleInsideTheFootprintRectangleButOffTheRobotIsKept)
{
  // base_link box at its joint offset; a post at (0.5, 0.5) is inside the home footprint
  // rectangle (x <= 0.54, |y| <= 0.62) but outside every primitive.
  const auto posed = posed_at_origin({{"base_link", at(0, 0, 0.21)}});
  EXPECT_FALSE(mmn::inside_any(Vector3d(0.5, 0.5, 0.5), posed, kMargin));
  EXPECT_FALSE(mmn::inside_any(Vector3d(0.5, 0.5, 1.0), posed, kMargin));
  EXPECT_TRUE(mmn::inside_any(Vector3d(0.2, 0.1, 0.32), posed, kMargin));  // deck
}

TEST(LidarRobotFilter, KeepPointDropsMissesAndSelfReturnsOnly)
{
  const auto sensor = at(0.24, 0.0, 0.387);
  const auto posed = posed_at_origin({{"base_link", at(0, 0, 0.21)}});
  EXPECT_FALSE(mmn::keep_point(Vector3d(0, 0, 0), sensor, posed, kMargin, 0.0));        // miss
  EXPECT_FALSE(mmn::keep_point(Vector3d(-0.1, 0, -0.08), sensor, posed, kMargin, 0.0));  // deck
  EXPECT_TRUE(mmn::keep_point(Vector3d(3.0, 0, -0.38), sensor, posed, kMargin, 0.0));    // ground
  EXPECT_TRUE(mmn::keep_point(Vector3d(0.26, 0.5, 0.1), sensor, posed, kMargin, 0.0));   // obstacle
}

TEST(LidarRobotFilter, RayRuleRemovesNoisySelfReturnsButKeepsObstaclesInFront)
{
  const auto sensor = at(0.24, 0.0, 0.387);
  // Arm pedestal: cylinder r 0.112 around (-0.08, 0), z 0.32..0.50 in base_footprint.
  const std::vector<mmn::PosedPrimitive> pedestal{mmn::posed_primitive(at(-0.08, 0, 0.41), mmg::Shape::Cylinder, Vector3d(0.112, 0.09, 0))};
  const double surface = mmn::first_self_hit(sensor.translation(), Vector3d(-1, 0, 0), pedestal);
  EXPECT_NEAR(surface, 0.24 - (-0.08 + 0.112), 1e-9);  // 0.208 m
  // A return 0.07 m short of the surface (3.5 sigma noise), the surface, and an obstacle
  // 0.15 m in front of the pedestal.
  EXPECT_FALSE(mmn::keep_point(Vector3d(-(surface - 0.07), 0, 0), sensor, pedestal, 0.03, 0.08));
  EXPECT_FALSE(mmn::keep_point(Vector3d(-surface, 0, 0), sensor, pedestal, 0.03, 0.08));
  EXPECT_TRUE(mmn::keep_point(Vector3d(-(surface - 0.15), 0, 0), sensor, pedestal, 0.03, 0.08));
}

TEST(LidarRobotFilter, RayRuleHitsBoxesAndIgnoresRaysThatMissTheRobot)
{
  const std::vector<mmn::PosedPrimitive> box{mmn::posed_primitive(at(1.0, 0, 0), mmg::Shape::Box, Vector3d(0.1, 0.1, 0.1))};
  EXPECT_NEAR(mmn::first_self_hit(Vector3d::Zero(), Vector3d(1, 0, 0), box), 0.9, 1e-12);
  EXPECT_TRUE(std::isinf(mmn::first_self_hit(Vector3d::Zero(), Vector3d(0, 1, 0), box)));
}
