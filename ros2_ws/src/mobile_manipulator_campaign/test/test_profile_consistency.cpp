// The robot profile and the generated robot model against B5's and F2's own configs (MuJoCo plant plan Task 3,
// spec Section 3.3): joint order, joint limits, breakaway, static-torque limits and the collision primitives.
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <fstream>
#include <sstream>

#include "mobile_manipulator_geometry/footprint_projection.hpp"
#include "mobile_manipulator_geometry/robot_model.hpp"
#include "mobile_manipulator_medium/problem.hpp"
#include "mobile_manipulator_mpc/formulation.hpp"
#include "mobile_manipulator_mpc/model.hpp"
#include "mobile_manipulator_plant/robot_profile.hpp"

namespace mmp = mobile_manipulator_plant;
namespace mmm = mobile_manipulator_mpc;
namespace mmg = mobile_manipulator_geometry;

namespace
{
std::string share(const std::string & package) { return ament_index_cpp::get_package_share_directory(package); }

std::string read_file(const std::string & path)
{
  std::ifstream in(path);
  std::stringstream s;
  s << in.rdbuf();
  return s.str();
}

mmp::RobotProfile profile() { return mmp::load_profile(share("mobile_manipulator_plant") + "/robots/mobile_manipulator/profile.yaml", share); }

Eigen::Isometry3d body_world(const mjData * d, int body)
{
  Eigen::Isometry3d x = Eigen::Isometry3d::Identity();
  x.translation() = Eigen::Map<const Eigen::Vector3d>(d->xpos + 3 * body);
  x.linear() = Eigen::Quaterniond(d->xquat[4 * body], d->xquat[4 * body + 1], d->xquat[4 * body + 2], d->xquat[4 * body + 3]).toRotationMatrix();
  return x;
}

Eigen::Isometry3d geom_world(const mjData * d, int geom)
{
  Eigen::Isometry3d x = Eigen::Isometry3d::Identity();
  x.translation() = Eigen::Map<const Eigen::Vector3d>(d->geom_xpos + 3 * geom);
  x.linear() = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(d->geom_xmat + 9 * geom);
  return x;
}
}  // namespace

TEST(ProfileConsistency, JointOrderMatchesB5)
{
  const auto p = profile();
  for (int i = 0; i < 6; ++i) EXPECT_EQ(p.arm_joints[i], std::string(mmm::kArmJoints[i]));
}

TEST(ProfileConsistency, LimitsAndBreakawayMatchB5)
{
  const auto p = profile();
  const mmm::Limits limits = mmm::load_limits(share("mobile_manipulator_navigation") + "/config/nav_operating_envelope.yaml",
                                              read_file(p.urdf_path));
  EXPECT_DOUBLE_EQ(limits.breakaway.linear_mps, p.breakaway_linear_mps);
  EXPECT_DOUBLE_EQ(limits.breakaway.yaw_radps, p.breakaway_yaw_radps);
  char error[1000] = "";
  mjModel * m = mj_loadXML(p.model_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_NE(m, nullptr) << error;
  for (int i = 0; i < 6; ++i) {
    const int j = mj_name2id(m, mjOBJ_JOINT, p.arm_joints[i].c_str());
    ASSERT_GE(j, 0);
    // B5's bounds are the URDF's intersected with its own joint_bounds: inside the plant's hard limits
    EXPECT_GE(limits.q_min(i), m->jnt_range[2 * j] - 1e-12) << p.arm_joints[i];
    EXPECT_LE(limits.q_max(i), m->jnt_range[2 * j + 1] + 1e-12) << p.arm_joints[i];
  }
  mj_deleteModel(m);
}

TEST(ProfileConsistency, TorqueLimitsMatchF2)
{
  const auto p = profile();
  const auto config = mobile_manipulator_medium::load_medium_config(share("mobile_manipulator_medium") + "/config/medium.yaml");
  for (int i = 0; i < 6; ++i) {
    if (std::isnan(p.torque_limits_nm(i))) EXPECT_TRUE(std::isinf(config.torque_limits_nm(i))) << i;
    else EXPECT_DOUBLE_EQ(p.torque_limits_nm(i), config.torque_limits_nm(i)) << i;
  }
}

TEST(ProfileConsistency, CollisionGeomsAreB5Primitives)
{
  const auto p = profile();
  const auto payload = mmg::payload_from_json(nlohmann::ordered_json::parse(read_file(p.payload_json_path)));
  const auto primitives = mmg::load_primitives(read_file(p.urdf_path), payload);
  char error[1000] = "";
  mjModel * m = mj_loadXML(p.model_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_NE(m, nullptr) << error;
  mjData * d = mj_makeData(m);
  mj_kinematics(m, d);  // qpos0: every frame defined
  int robot_geoms = 0;
  for (int g = 0; g < m->ngeom; ++g) {
    const char * name = mj_id2name(m, mjOBJ_GEOM, g);
    robot_geoms += name && std::string(name).rfind("robot/", 0) == 0;
  }
  EXPECT_EQ(robot_geoms, static_cast<int>(primitives.size()));
  for (const auto & prim : primitives) {
    const int body = mj_name2id(m, mjOBJ_BODY, prim.link.c_str());
    ASSERT_GE(body, 0) << prim.link;
    const Eigen::Isometry3d expected = body_world(d, body) * prim.pose;
    const int type = prim.shape == mmg::Shape::Box ? mjGEOM_BOX : mjGEOM_CYLINDER;
    int matches = 0;
    for (int g = 0; g < m->ngeom; ++g) {
      if (m->geom_type[g] != type) continue;
      const Eigen::Vector3d size(m->geom_size[3 * g], m->geom_size[3 * g + 1], m->geom_size[3 * g + 2]);
      const bool same_dims = type == mjGEOM_BOX ? size.isApprox(prim.dims, 1e-12)
                                                : std::abs(size(0) - prim.dims(0)) < 1e-12 && std::abs(size(1) - prim.dims(1)) < 1e-12;
      if (!same_dims) continue;
      const Eigen::Isometry3d actual = geom_world(d, g);
      // a cylinder is symmetric about its axis: compare the centre and the axis only
      const bool same_pose = (actual.translation() - expected.translation()).norm() < 1e-9 &&
                             (type == mjGEOM_BOX ? actual.linear().isApprox(expected.linear(), 1e-9)
                                                 : std::abs(actual.linear().col(2).dot(expected.linear().col(2))) > 1 - 1e-12);
      matches += same_pose;
    }
    EXPECT_EQ(matches, 1) << prim.link;
  }
  mj_deleteData(d);
  mj_deleteModel(m);
}
