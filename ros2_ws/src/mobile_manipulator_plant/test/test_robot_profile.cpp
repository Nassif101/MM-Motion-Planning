// The robot profile loader (MuJoCo plant plan Task 3): references resolved through a package-share lookup, the
// generated robot model compiles with its 9 joints and 9 actuators.
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>

#include "mobile_manipulator_plant/robot_profile.hpp"

namespace mmp = mobile_manipulator_plant;

namespace
{
// Package sources stand in for their share directories (the referenced files sit at the same relative paths).
std::string source_share(const std::string & package) { return std::string(PACKAGES_SRC) + "/" + package; }
}  // namespace

TEST(RobotProfile, LoadsAndResolvesReferences)
{
  const mmp::RobotProfile p = mmp::load_profile(PROFILE_YAML, source_share);
  EXPECT_EQ(p.robot, "mobile_manipulator");
  const std::array<std::string, 6> joints{"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  EXPECT_EQ(p.arm_joints, joints);
  EXPECT_TRUE(std::isnan(p.torque_limits_nm(0)));
  EXPECT_DOUBLE_EQ(p.torque_limits_nm(1), 96.0);
  EXPECT_DOUBLE_EQ(p.payload_mass_kg, 3.0);
  EXPECT_TRUE(p.payload_size.isApprox(Eigen::Vector3d(1.2, 1.2, 0.04)));
  EXPECT_TRUE(p.payload_center.isApprox(Eigen::Vector3d(0.0, 0.0, 0.035)));
  ASSERT_EQ(p.poses.count("carry"), 1u);
  EXPECT_EQ(p.lidar.site, "livox_frame");
  EXPECT_DOUBLE_EQ(p.lidar.rate_hz, 10.0);
  EXPECT_EQ(p.lidar.points_per_scan, 20000);
  EXPECT_DOUBLE_EQ(p.lidar.min_range_m, 0.1);
  EXPECT_DOUBLE_EQ(p.lidar.max_range_m, 70.0);
  EXPECT_DOUBLE_EQ(p.lidar.sigma_m, 0.02);
  EXPECT_DOUBLE_EQ(p.lidar.dropout, 0.0);
  EXPECT_DOUBLE_EQ(p.base_limits.v_max_fwd, 0.8);
  EXPECT_DOUBLE_EQ(p.base_limits.stale_s, 0.5);
  EXPECT_DOUBLE_EQ(p.breakaway_linear_mps, 0.0375);
  EXPECT_DOUBLE_EQ(p.breakaway_yaw_radps, 0.07);
  EXPECT_DOUBLE_EQ(p.kp(1), 6000.0);
  EXPECT_DOUBLE_EQ(p.kd(5), 28.0);
  EXPECT_EQ(p.damping, mmp::Vec6::Zero());
  EXPECT_EQ(p.frictionloss, mmp::Vec6::Zero());
  EXPECT_DOUBLE_EQ(p.rest_linear_mps, 0.01);
  EXPECT_DOUBLE_EQ(p.rest_yaw_radps, 0.02);
  EXPECT_FALSE(p.ideal_breakaway);
  EXPECT_DOUBLE_EQ(p.command_delay_s, 0.04);
}

TEST(RobotProfile, GeneratedModelCompiles)
{
  const mmp::RobotProfile p = mmp::load_profile(PROFILE_YAML, source_share);
  char error[1000] = "";
  mjModel * m = mj_loadXML(p.model_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_NE(m, nullptr) << error;
  EXPECT_EQ(m->njnt, 9);  // base_x, base_y, base_yaw + 6 arm joints
  EXPECT_EQ(m->nu, 9);    // 6 arm servos + 3 base drives
  for (int k = 0; k < 6; ++k) EXPECT_GE(mj_name2id(m, mjOBJ_JOINT, p.arm_joints[k].c_str()), 0);
  EXPECT_GE(mj_name2id(m, mjOBJ_SITE, "livox_frame"), 0);
  EXPECT_GE(mj_name2id(m, mjOBJ_GEOM, "robot/panel"), 0);
  mj_deleteModel(m);
}

namespace
{
// The profile with one change, written next to the original (its relative references stay valid); the loader's error.
std::string load_error(const std::function<void(YAML::Node &)> & change)
{
  YAML::Node root = YAML::LoadFile(PROFILE_YAML);
  change(root);
  const std::string path = std::filesystem::path(PROFILE_YAML).parent_path().string() + "/test_profile_variant.yaml";
  std::ofstream(path) << root;
  try {
    mmp::load_profile(path, source_share);
  } catch (const std::runtime_error & e) {
    std::filesystem::remove(path);
    return e.what();
  }
  std::filesystem::remove(path);
  return "";
}
}  // namespace

TEST(RobotProfile, NamesTheFileAndKeyOfABadEntry)
{
  EXPECT_NE(load_error([](YAML::Node & r) { r["arm"].remove("kd"); }).find("arm.kd"), std::string::npos);
  EXPECT_NE(load_error([](YAML::Node & r) { r["arm"]["kp"].push_back(1.0); }).find("arm.kp"), std::string::npos);
  EXPECT_NE(load_error([](YAML::Node & r) { r["arm"]["kp"][2] = "stiff"; }).find("arm.kp"), std::string::npos);
  EXPECT_NE(load_error([](YAML::Node & r) { r["arm_joints"][0] = YAML::Node(YAML::NodeType::Map); }).find("arm_joints"),
            std::string::npos);
  EXPECT_NE(load_error([](YAML::Node & r) { r["lidar"].remove("sigma_m"); }).find("lidar.sigma_m"), std::string::npos);
  EXPECT_NE(load_error([](YAML::Node & r) { r["arm"].remove("damping"); }).find("arm.damping"), std::string::npos);
}

TEST(RobotProfile, RejectsOutOfRangeValues)
{
  EXPECT_NE(load_error([](YAML::Node & r) { r["arm"]["kp"][0] = 0.0; }).find("arm.kp"), std::string::npos);
  EXPECT_NE(load_error([](YAML::Node & r) { r["arm"]["kd"][0] = -1.0; }).find("arm.kd"), std::string::npos);
  EXPECT_NE(load_error([](YAML::Node & r) { r["lidar"]["dropout"] = 1.5; }).find("lidar.dropout"), std::string::npos);
  EXPECT_NE(load_error([](YAML::Node & r) { r["lidar"]["rate_hz"] = 0.0; }).find("lidar.rate_hz"), std::string::npos);
  EXPECT_NE(load_error([](YAML::Node & r) { r["lidar"]["min_range_m"] = 80.0; }).find("lidar.min_range_m"), std::string::npos);
  EXPECT_NE(load_error([](YAML::Node & r) { r["realistic"]["command_delay_s"] = -0.01; }).find("realistic.command_delay_s"),
            std::string::npos);
  EXPECT_NE(load_error([](YAML::Node & r) { r["model"] = "missing.xml"; }).find("missing.xml"), std::string::npos);
  EXPECT_EQ(load_error([](YAML::Node &) {}), "");  // the profile itself loads
}

TEST(RobotProfile, RejectsMissingKeys)
{
  EXPECT_THROW(mmp::load_profile(std::string(PACKAGES_SRC) + "/mobile_manipulator_plant/package.xml", source_share),
               std::runtime_error);
}
