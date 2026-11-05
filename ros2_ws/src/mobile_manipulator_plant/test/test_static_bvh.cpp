// The static-geometry BVH against MuJoCo's own ray casts on small models (Task 5 BVH review): exact ties, invisible
// geoms, finite planes, rays along a face plane, the unsupported-geom error.
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <random>
#include <stdexcept>
#include <string>

#include "mobile_manipulator_plant/static_bvh.hpp"

namespace mmp = mobile_manipulator_plant;

namespace
{
struct Xml
{
  mjModel * m = nullptr;
  mjData * d = nullptr;
  explicit Xml(const std::string & body)
  {
    const std::string xml = "<mujoco><worldbody>" + body + "</worldbody></mujoco>";
    mjSpec * spec = mj_parseXMLString(xml.c_str(), nullptr, nullptr, 0);
    m = mj_compile(spec, nullptr);
    mj_deleteSpec(spec);
    d = mj_makeData(m);
    mj_kinematics(m, d);
  }
  ~Xml()
  {
    mj_deleteData(d);
    mj_deleteModel(m);
  }
  // MuJoCo's answer among every geom
  double mujoco(const Eigen::Vector3d & o, const Eigen::Vector3d & v, int & geom) const
  {
    const mjtNum p[3] = {o.x(), o.y(), o.z()}, vec[3] = {v.x(), v.y(), v.z()};
    return mj_ray(m, d, p, vec, nullptr, 1, -1, &geom, nullptr);
  }
};
}  // namespace

TEST(StaticBvh, ExactTieGoesToTheLowestGeomLikeMujoco)
{
  // two boxes whose front faces (x = 1) are coplanar and overlap for 0 < y < 1: a ray there hits both at the same t
  for (const std::string order : {"ab", "ba"}) {
    const std::string a = R"(<geom name="a" type="box" pos="2 -0.5 1" size="1 1 1"/>)";
    const std::string b = R"(<geom name="b" type="box" pos="2 1 1" size="1 1 1"/>)";
    const Xml x(order == "ab" ? a + b : b + a);
    const mmp::StaticBvh bvh(x.m, x.d);
    int g_bvh = -1, g_mj = -1;
    const Eigen::Vector3d o(0.0, 0.5, 1.0), v(1.0, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(bvh.ray(o, v, g_bvh), x.mujoco(o, v, g_mj)) << order;
    EXPECT_EQ(g_bvh, g_mj) << order;
    EXPECT_EQ(g_bvh, 0) << order;  // the lower geom id
  }
}

TEST(StaticBvh, MatchesMujocoOnRandomRays)
{
  std::string body = R"(<geom name="floor" type="plane" size="0 0 0.05"/><geom name="pad" type="plane" pos="0 0 0.5" size="1 1 0.05"/>)";
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> u(-5.0, 5.0), s(0.1, 1.5), yaw(-3.2, 3.2);
  for (int i = 0; i < 60; ++i)
    body += "<geom type=\"box\" pos=\"" + std::to_string(u(rng)) + " " + std::to_string(u(rng)) + " " + std::to_string(s(rng)) +
            "\" size=\"" + std::to_string(s(rng)) + " " + std::to_string(s(rng)) + " " + std::to_string(s(rng)) +
            "\" euler=\"0 0 " + std::to_string(yaw(rng)) + "\"/>";
  body += R"(<geom name="ghost" type="box" pos="0 0 3" size="10 10 0.1" rgba="1 1 1 0"/>)";  // invisible: skipped by both
  const Xml x(body);
  const mmp::StaticBvh bvh(x.m, x.d);
  std::normal_distribution<double> n(0.0, 1.0);
  int hits = 0;
  for (int k = 0; k < 20000; ++k) {
    const Eigen::Vector3d o(u(rng), u(rng), 0.3 + 2.0 * std::abs(n(rng)));
    Eigen::Vector3d v(n(rng), n(rng), n(rng));
    if (k % 50 == 0) v.z() = 0.0;  // rays along horizontal face planes
    v.normalize();
    int g_bvh = -1, g_mj = -1;
    const double t_bvh = bvh.ray(o, v, g_bvh), t_mj = x.mujoco(o, v, g_mj);
    ASSERT_EQ(g_bvh, g_mj) << k;
    if (g_mj >= 0) {
      ASSERT_NEAR(t_bvh, t_mj, 1e-12) << k;
      ++hits;
    }
  }
  EXPECT_GT(hits, 10000);
}

TEST(StaticBvh, UnsupportedStaticGeomNamesTheWayOut)
{
  const Xml x(R"(<geom name="pipe" type="cylinder" size="0.1 1"/>)");
  try {
    mmp::StaticBvh bvh(x.m, x.d);
    FAIL() << "no error";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("pipe"), std::string::npos);
    EXPECT_NE(std::string(e.what()).find("static_bvh: false"), std::string::npos);
  }
}
