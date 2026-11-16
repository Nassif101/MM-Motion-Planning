// The scenario fixtures in the compiled plant model (G5 for the scenario boxes, plan Task 4; the site's doors are G5 in
// tools/plant/test_generate_site_model.py): the 1.05 m gate between its posts, the 1.23 / 1.30 m beam undersides.
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mobile_manipulator_campaign/scenario_setup.hpp"
#include "mobile_manipulator_mpc/scene.hpp"
#include "mobile_manipulator_plant/model_builder.hpp"

namespace mmc = mobile_manipulator_campaign;
namespace mmp = mobile_manipulator_plant;

namespace
{
std::string share(const std::string & package) { return ament_index_cpp::get_package_share_directory(package); }

struct Compiled
{
  mjModel * m;
  explicit Compiled(const std::string & scenario)
  {
    const auto profile = mmp::load_profile(share("mobile_manipulator_plant") + "/robots/mobile_manipulator/profile.yaml", share);
    const auto scene = mobile_manipulator_mpc::load_scene(share("mobile_manipulator_navigation"), "", scenario, "");
    m = mmp::build_model(profile, share("mobile_manipulator_plant") + "/models/site_v2.xml", mmc::scenario_setup(scene, scenario));
  }
  ~Compiled() { mj_deleteModel(m); }
  int geom(const std::string & name) const
  {
    const int g = mj_name2id(m, mjOBJ_GEOM, name.c_str());
    EXPECT_GE(g, 0) << name;
    return g;
  }
  double lo(const std::string & name, int axis) const { return m->geom_pos[3 * geom(name) + axis] - m->geom_size[3 * geom(name) + axis]; }
  double hi(const std::string & name, int axis) const { return m->geom_pos[3 * geom(name) + axis] + m->geom_size[3 * geom(name) + axis]; }
};
}  // namespace

TEST(ScenarioFixtures, GateOpeningIs105)
{
  for (const std::string scenario : {"v2_gate_right_swing_nav", "v2_door_then_beam_nav"}) {
    const Compiled c(scenario);
    EXPECT_NEAR(c.lo("scenario/gate_post_left", 1) - c.hi("scenario/gate_post_right", 1), 1.05, 1e-9) << scenario;
  }
}

TEST(ScenarioFixtures, BeamUndersides)
{
  EXPECT_NEAR(Compiled("v2_low_beam_123_nav").lo("scenario/low_beam", 2), 1.23, 1e-9);
  EXPECT_NEAR(Compiled("v2_low_beam_130_nav").lo("scenario/low_beam", 2), 1.30, 1e-9);
  EXPECT_NEAR(Compiled("v2_door_then_beam_nav").lo("scenario/low_beam", 2), 1.23, 1e-9);
}

TEST(ScenarioFixtures, SiteBoxesAreNotDuplicated)
{
  const Compiled c("v2_gate_right_swing_nav");
  int scenario_geoms = 0;
  for (int g = 0; g < c.m->ngeom; ++g) scenario_geoms += std::string(mj_id2name(c.m, mjOBJ_GEOM, g) ? mj_id2name(c.m, mjOBJ_GEOM, g) : "").rfind("scenario/", 0) == 0;
  EXPECT_EQ(scenario_geoms, 3);  // the two posts and the shelf
}

TEST(ScenarioFixtures, MoversBecomeMocapBodies)
{
  const Compiled c("v2_worker_crossing_nav");
  const int body = mj_name2id(c.m, mjOBJ_BODY, "mover/crossing_worker");
  ASSERT_GE(body, 0);
  EXPECT_GE(c.m->body_mocapid[body], 0);
  EXPECT_NEAR(c.m->body_pos[3 * body + 2], 0.9, 1e-12);
}

TEST(ScenarioFixtures, RejectsAnotherSite)
{
  mobile_manipulator_mpc::Scene scene;
  scene.site = "construction_site";
  EXPECT_THROW(mmc::scenario_setup(scene, "v1"), std::invalid_argument);
}

// ---- E3 Task 3: the tightness cells (spec 2026-11-15 Section 3.2) ----

namespace
{
const double kE3Beam[] = {1.25, 1.23, 1.21, 1.19};  // the beam's underside (m)
const double kE3Gap[] = {0.30, 0.27, 0.24, 0.21};   // each brace's face from the route (m)
}  // namespace

TEST(E3Cells, APairHasTheSameObstacleTurnedWithTheRoute)
{
  for (int k = 0; k < 4; ++k) {
    const std::string step = std::to_string(k + 1);
    const Compiled bs("e3_beam_straight_" + step), bt("e3_beam_turn_" + step);
    for (const Compiled * c : {&bs, &bt}) {
      EXPECT_NEAR(c->lo("scenario/low_beam", 2), kE3Beam[k], 1e-9) << step;
      EXPECT_NEAR(c->hi("scenario/low_beam", 2) - c->lo("scenario/low_beam", 2), 0.2, 1e-9);
    }
    // 0.2 m deep along the travel, 3.0 m across it: x then y on the straight approach (east), y then x after the turn (north)
    EXPECT_NEAR(bs.hi("scenario/low_beam", 0) - bs.lo("scenario/low_beam", 0), 0.2, 1e-9);
    EXPECT_NEAR(bs.hi("scenario/low_beam", 1) - bs.lo("scenario/low_beam", 1), 3.0, 1e-9);
    EXPECT_NEAR(bt.hi("scenario/low_beam", 1) - bt.lo("scenario/low_beam", 1), 0.2, 1e-9);
    EXPECT_NEAR(bt.hi("scenario/low_beam", 0) - bt.lo("scenario/low_beam", 0), 3.0, 1e-9);

    const Compiled gs("e3_gap_straight_" + step), gt("e3_gap_turn_" + step);
    // the gap is centred on the route: y = -5.5 on the straight approach, x = -1.5 after the turn
    EXPECT_NEAR(gs.lo("scenario/brace_left", 1) - (-5.5), kE3Gap[k], 1e-9) << step;
    EXPECT_NEAR(-5.5 - gs.hi("scenario/brace_right", 1), kE3Gap[k], 1e-9) << step;
    EXPECT_NEAR(-1.5 - gt.hi("scenario/brace_left", 0), kE3Gap[k], 1e-9) << step;
    EXPECT_NEAR(gt.lo("scenario/brace_right", 0) - (-1.5), kE3Gap[k], 1e-9) << step;
    for (const Compiled * c : {&gs, &gt})
      for (const char * brace : {"scenario/brace_left", "scenario/brace_right"}) {
        EXPECT_NEAR(c->lo(brace, 2), 0.75, 1e-9) << "raised above the base";
        EXPECT_NEAR(c->hi(brace, 2), 2.2, 1e-9);
      }
    EXPECT_NEAR(gs.hi("scenario/brace_left", 0) - gs.lo("scenario/brace_left", 0), 0.4, 1e-9);  // deep along the travel
    EXPECT_NEAR(gt.hi("scenario/brace_left", 1) - gt.lo("scenario/brace_left", 1), 0.4, 1e-9);
  }
}
