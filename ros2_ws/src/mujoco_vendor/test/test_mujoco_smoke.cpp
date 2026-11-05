// Smoke test of the vendored MuJoCo (MuJoCo plant plan Task 1): the pinned library loads a model from memory and
// steps it, and its version is the pinned one.
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <string>

namespace
{
constexpr const char * kFallingBox = R"(
<mujoco>
  <option timestep="0.002"/>
  <worldbody>
    <geom type="plane" size="5 5 0.1"/>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>)";
}

TEST(MujocoSmoke, FallingBoxLandsOnPlane)
{
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  ASSERT_EQ(mj_addBufferVFS(&vfs, "box.xml", kFallingBox, static_cast<int>(std::string(kFallingBox).size())), 0);
  char error[1000] = "";
  mjModel * m = mj_loadXML("box.xml", &vfs, error, sizeof(error));
  ASSERT_NE(m, nullptr) << error;
  mjData * d = mj_makeData(m);
  for (int i = 0; i < 1000; ++i) mj_step(m, d);  // 2 s
  EXPECT_NEAR(d->qpos[2], 0.1, 0.002);
  mj_deleteData(d);
  mj_deleteModel(m);
  mj_deleteVFS(&vfs);
}

TEST(MujocoSmoke, VersionMatchesPin)
{
  EXPECT_EQ(std::string(mj_versionString()), std::string(MUJOCO_VERSION));
}
