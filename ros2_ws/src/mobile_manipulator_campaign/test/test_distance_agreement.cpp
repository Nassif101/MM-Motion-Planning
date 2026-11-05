// G3 (MuJoCo plant spec Section 8, plan Task 3): MuJoCo's geom distances between the robot's collision geoms and the
// known boxes agree with B5's FCL distances within 1 mm, over 10,000 seeded robot states near the course fixtures.
// The test model is the generated robot plus the fixture scenarios' known boxes (site + scenario, as B5 loads them).
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <fstream>
#include <map>
#include <random>
#include <sstream>

#include "mobile_manipulator_geometry/footprint_projection.hpp"
#include "mobile_manipulator_geometry/robot_model.hpp"
#include "mobile_manipulator_mpc/collision.hpp"
#include "mobile_manipulator_mpc/model.hpp"
#include "mobile_manipulator_mpc/scene.hpp"
#include "mobile_manipulator_mpc/world.hpp"
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

constexpr double kNear = 0.3;  // pairs compared below this distance (m)
constexpr double kTolerance = 0.001;

struct Fixture
{
  std::string scenario;
  int samples;
};

// The robot model plus `boxes` (by name) as static world geoms.
mjModel * build(const std::string & robot_xml, const std::vector<mmm::ObstacleBox> & boxes)
{
  char error[1000] = "";
  mjSpec * spec = mj_parseXML(robot_xml.c_str(), nullptr, error, sizeof(error));
  if (!spec) throw std::runtime_error(error);
  mjsBody * world = mjs_findBody(spec, "world");
  for (const auto & box : boxes) {
    mjsGeom * g = mjs_addGeom(world, nullptr);
    mjs_setName(g->element, ("site/" + box.name).c_str());
    g->type = mjGEOM_BOX;
    for (int i = 0; i < 3; ++i) {
      g->size[i] = box.size(i) / 2.0;
      g->pos[i] = box.pose.translation()(i);
    }
    const Eigen::Quaterniond q(box.pose.linear());
    g->quat[0] = q.w();
    g->quat[1] = q.x();
    g->quat[2] = q.y();
    g->quat[3] = q.z();
    g->contype = 1;
    g->conaffinity = 2;
  }
  mjModel * m = mj_compile(spec, nullptr);
  mj_deleteSpec(spec);
  if (!m) throw std::runtime_error("compile failed");
  return m;
}
}  // namespace

TEST(DistanceAgreement, MujocoMatchesCollisionModel)
{
  const auto p = mmp::load_profile(share("mobile_manipulator_plant") + "/robots/mobile_manipulator/profile.yaml", share);
  const std::string urdf = read_file(p.urdf_path);
  const auto payload = mmg::payload_from_json(nlohmann::ordered_json::parse(read_file(p.payload_json_path)));
  const mmm::CollisionModel b5(mmg::load_primitives(urdf, payload), mmm::CollisionConfig{});
  const std::vector<Fixture> fixtures{{"v2_gate_right_swing_nav", 2500}, {"v2_door_then_beam_nav", 2500},
                                      {"v2_low_beam_123_nav", 2500}, {"v2_door_105_nav", 2500}};
  const std::vector<std::string> poses{"carry", "vertical_carry", "home"};
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> unit(0.0, 1.0), jitter(-1.0, 1.0);
  double max_diff = 0.0;
  int compared = 0, overall = 0, penetrating = 0;
  for (const auto & fixture : fixtures) {
    const mmm::Scene scene = mmm::load_scene(share("mobile_manipulator_navigation"), "", fixture.scenario, "");
    mmm::World world;
    world.set_static(scene.boxes);
    const auto & spec = *scene.scenario;
    const Eigen::Vector2d a(spec.at("start").at(0).get<double>(), spec.at("start").at(1).get<double>());
    const Eigen::Vector2d b(spec.at("goal").at(0).get<double>(), spec.at("goal").at(1).get<double>());
    // the boxes near the drive (within 4 m of the start-goal segment)
    std::vector<mmm::ObstacleBox> near;
    for (const auto & box : world.statics()) {
      const Eigen::Vector2d c = box.pose.translation().head<2>();
      const double u = std::clamp((c - a).dot(b - a) / (b - a).squaredNorm(), 0.0, 1.0);
      if ((a + u * (b - a) - c).norm() < 4.0 + box.size.head<2>().norm() / 2.0) near.push_back(box);
    }
    mjModel * m = build(p.model_path, near);
    mjData * d = mj_makeData(m);
    std::map<std::string, std::vector<int>> link_geoms;  // B5 link name -> robot geoms on that body
    std::vector<int> box_geoms;
    for (int g = 0; g < m->ngeom; ++g) {
      const std::string name = mj_id2name(m, mjOBJ_GEOM, g);
      if (name.rfind("site/", 0) == 0) box_geoms.push_back(g);
      else if (name.rfind("robot/", 0) == 0) link_geoms[mj_id2name(m, mjOBJ_BODY, m->geom_bodyid[g])].push_back(g);
    }
    const double heading = std::atan2(b.y() - a.y(), b.x() - a.x());
    const Eigen::Vector2d lateral(-std::sin(heading), std::cos(heading));
    for (int s = 0; s < fixture.samples; ++s) {
      mmm::State x;
      const Eigen::Vector2d xy = a + unit(rng) * (b - a) + 0.4 * jitter(rng) * lateral;
      x << xy.x(), xy.y(), heading + 0.4 * jitter(rng), p.poses.at(poses[s % poses.size()]);
      for (int j = 0; j < 6; ++j) {
        const int jid = mj_name2id(m, mjOBJ_JOINT, p.arm_joints[j].c_str());
        x(3 + j) = std::clamp(x(3 + j) + 0.3 * jitter(rng), m->jnt_range[2 * jid], m->jnt_range[2 * jid + 1]);
      }
      d->qpos[0] = x(0);
      d->qpos[1] = x(1);
      d->qpos[2] = x(2);
      for (int j = 0; j < 6; ++j) d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, p.arm_joints[j].c_str())]] = x(3 + j);
      mj_kinematics(m, d);
      double mujoco_min = std::numeric_limits<double>::infinity();
      for (const auto & [link, geoms] : link_geoms) {
        for (int box : box_geoms) {
          double mj_dist = std::numeric_limits<double>::infinity();
          for (int g : geoms) mj_dist = std::min(mj_dist, mj_geomDistance(m, d, g, box, kNear, nullptr));
          mujoco_min = std::min(mujoco_min, mj_dist);
          if (mj_dist >= kNear) continue;
          mmm::DistanceRow row;
          row.link = mmm::link_index(link);
          row.obstacle = mj_id2name(m, mjOBJ_GEOM, box) + 5;  // without "site/"
          const double fcl = b5.pair_distance(x, row, world, 0.0);
          if (mj_dist <= 0.0 || fcl <= 0.0) {
            ++penetrating;  // penetration depths are defined differently: both must say "touching"
            EXPECT_LE(std::max(mj_dist, fcl), kTolerance) << link << " / " << row.obstacle;
            continue;
          }
          ++compared;
          max_diff = std::max(max_diff, std::abs(mj_dist - fcl));
          EXPECT_NEAR(mj_dist, fcl, kTolerance) << fixture.scenario << " " << link << " / " << row.obstacle;
        }
      }
      // the overall nearest pair, whichever side found it
      const auto nearest = b5.nearest(x, near);
      if (nearest && nearest->distance < kNear && nearest->distance > 0.0) {
        ++overall;
        EXPECT_NEAR(mujoco_min, nearest->distance, kTolerance) << fixture.scenario;
      }
    }
    mj_deleteData(d);
    mj_deleteModel(m);
  }
  std::cout << "compared " << compared << " pairs (" << overall << " overall minima, " << penetrating
            << " touching), max |MuJoCo - FCL| = " << max_diff << " m\n";
  EXPECT_GT(compared, 1000);
}
