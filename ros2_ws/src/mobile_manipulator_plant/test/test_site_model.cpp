// G1 after MuJoCo's compiler (MuJoCo plant spec Section 8, plan Task 2): the installed site model, compiled, has every
// box of the V2 export at its centre, half size and yaw, and carries the boxes file's checksum.
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <map>
#include <string>

namespace
{
std::string custom_text(const mjModel * m, const std::string & name)
{
  const int id = mj_name2id(m, mjOBJ_TEXT, name.c_str());
  if (id < 0) return "";
  return std::string(m->text_data + m->text_adr[id], m->text_size[id] - 1);
}
}  // namespace

TEST(SiteModel, CompiledGeomsMatchExport)
{
  char error[1000] = "";
  mjModel * m = mj_loadXML(SITE_MODEL, nullptr, error, sizeof(error));
  ASSERT_NE(m, nullptr) << error;
  const YAML::Node boxes = YAML::LoadFile(BOXES_YAML)["boxes"];
  ASSERT_EQ(boxes.size(), 604u);
  int site_geoms = 0;
  for (int g = 0; g < m->ngeom; ++g) {
    const char * name = mj_id2name(m, mjOBJ_GEOM, g);
    if (name && std::string(name).rfind("site/", 0) == 0) ++site_geoms;
  }
  EXPECT_EQ(site_geoms, 604);
  for (const auto & box : boxes) {
    const std::string name = "site/" + box["name"].as<std::string>();
    const int g = mj_name2id(m, mjOBJ_GEOM, name.c_str());
    ASSERT_GE(g, 0) << name;
    EXPECT_EQ(m->geom_type[g], mjGEOM_BOX) << name;
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(m->geom_pos[3 * g + i], box["center"][i].as<double>(), 1e-6) << name;
      EXPECT_NEAR(m->geom_size[3 * g + i], box["size"][i].as<double>() / 2.0, 1e-6) << name;
    }
    const double yaw = box["yaw"].as<double>();
    // q and -q are the same rotation: |q . q_expected| = 1
    const double * q = m->geom_quat + 4 * g;
    const double dot = q[0] * std::cos(yaw / 2.0) + q[3] * std::sin(yaw / 2.0);
    EXPECT_NEAR(std::abs(dot), 1.0, 1e-9) << name;
    EXPECT_NEAR(q[1], 0.0, 1e-9) << name;
    EXPECT_NEAR(q[2], 0.0, 1e-9) << name;
  }
  EXPECT_EQ(custom_text(m, "site_checksum"), std::string("sha256:") + BOXES_SHA256);
  EXPECT_EQ(custom_text(m, "generator"), "generate_site_model 1");
  mj_deleteModel(m);
}
