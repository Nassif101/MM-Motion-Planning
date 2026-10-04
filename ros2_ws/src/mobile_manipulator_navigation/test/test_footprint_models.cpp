#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mobile_manipulator_geometry/footprint_model.hpp"
#include "mobile_manipulator_navigation/footprint_config.hpp"
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
    ament_index_cpp::get_package_share_directory("mobile_manipulator_control") + "/config/qualified_payload.json"));
  return json;
}

std::shared_ptr<const mmg::FootprintProjector> projector()
{
  static const auto instance = std::make_shared<const mmg::FootprintProjector>(
    read_file(ament_index_cpp::get_package_share_directory("mobile_manipulator_description") +
              "/urdf/mobile_manipulator.urdf"),
    mmg::payload_from_json(qualified_payload()));
  return instance;
}

const mmn::Json & parameters()
{
  static const mmn::ScenarioConfig config(PACKAGE_SOURCE_DIR);
  static const auto json = config.load("dynamic_footprint.yaml").at("dynamic_footprint_node").at("ros__parameters");
  return json;
}

std::vector<double> qualified(const std::string & name)
{
  return qualified_payload()["poses_rad"][name].get<std::vector<double>>();
}

mmg::JointMap joints_of(const std::vector<double> & values)
{
  mmg::JointMap joints;
  const auto & order = qualified_payload()["joint_order"];
  for (size_t i = 0; i < order.size(); ++i) joints[order[i].get<std::string>()] = values[i];
  return joints;
}

// The three qualified poses plus 21 joint-space samples on home -> vertical_carry and on
// home -> level_extension.
std::vector<std::pair<std::string, mmg::JointMap>> sweep()
{
  std::vector<std::pair<std::string, mmg::JointMap>> poses;
  for (const auto & name : {"home", "vertical_carry", "level_extension"}) poses.emplace_back(name, joints_of(qualified(name)));
  for (const auto & target : {"vertical_carry", "level_extension"}) {
    const auto from = qualified("home"), to = qualified(target);
    for (int i = 1; i <= 21; ++i) {
      std::vector<double> q;
      for (size_t j = 0; j < from.size(); ++j) q.push_back(from[j] + (to[j] - from[j]) * i / 22.0);
      poses.emplace_back(std::string("home->") + target + " " + std::to_string(i), joints_of(q));
    }
  }
  return poses;
}

mmn::Polygon profile(const std::string & name)
{
  static const mmn::ScenarioConfig config(PACKAGE_SOURCE_DIR);
  return mmn::polygon_of(config.load("footprint_profiles.yaml")["profiles"][name]["polygon"]);
}

std::array<double, 4> bounds(const mmg::Polygon & polygon)
{
  std::array<double, 4> b{INFINITY, -INFINITY, INFINITY, -INFINITY};
  for (const auto & [x, y] : polygon) {
    b = {std::min(b[0], x), std::max(b[1], x), std::min(b[2], y), std::max(b[3], y)};
  }
  return b;
}

const auto mesh = [] { return mmn::footprint_model("mesh", parameters(), projector()); };
const auto disc = [] { return mmn::footprint_model("disc", parameters(), projector()); };
}  // namespace

TEST(MeshModel, HullInsideMatchingProfile)
{
  for (const auto & name : {"home", "vertical_carry"}) {
    const auto footprint = mesh()->footprint(joints_of(qualified(name)), 0.0);
    EXPECT_TRUE(mmg::contains(profile(name), footprint).inside) << name;
  }
}

TEST(MeshModel, MatchesProjectorBounds)
{
  // The profiles are the projected bounds plus the 0.02 m perimeter allowance.
  for (const auto & name : {"home", "vertical_carry"}) {
    const auto expected = bounds(profile(name));
    const auto actual = bounds(mesh()->footprint(joints_of(qualified(name)), 0.0));
    EXPECT_NEAR(actual[0], expected[0] + 0.02, 0.006) << name;
    EXPECT_NEAR(actual[1], expected[1] - 0.02, 0.006) << name;
    EXPECT_NEAR(actual[2], expected[2] + 0.02, 0.006) << name;
    EXPECT_NEAR(actual[3], expected[3] - 0.02, 0.006) << name;
  }
}

TEST(DiscModel, ContainsMeshOverSweep)
{
  const auto mesh_model = mesh();
  const auto disc_model = disc();
  for (const auto & [name, joints] : sweep()) {
    EXPECT_GE(mmg::contains(disc_model->footprint(joints, 0.0), mesh_model->points(joints)).margin_m, -1e-9) << name;
  }
}

TEST(DiscModel, RadiiFromUrdf)
{
  const auto config = mmn::disc_config_of(parameters().at("disc"));
  const mmg::DiscHullModel model(projector(), config);
  ASSERT_EQ(model.radii().size(), config.links.size());
  for (size_t i = 0; i < config.links.size(); ++i) {
    const double r = model.radii()[i];
    EXPECT_GE(r, 0.0) << config.links[i].link;
    EXPECT_LE(r, 0.25) << config.links[i].link;
    EXPECT_NEAR(std::round(r / 0.005) * 0.005, r, 1e-12) << config.links[i].link;
    // Every link with collision geometry gets a positive radius; tool0 has none.
    if (config.links[i].link != "tool0") EXPECT_GT(r, 0.0) << config.links[i].link;
  }
}

TEST(Models, FiniteOutputOverSweep)
{
  for (const auto & model : {mesh(), disc()}) {
    for (const auto & [name, joints] : sweep()) {
      const auto footprint = model->footprint(joints, 0.02);
      EXPECT_GE(footprint.size(), 3u) << name;
      EXPECT_LE(footprint.size(), 40u) << name;
      EXPECT_GT(mmg::area(footprint), 0.0) << name;
    }
  }
}

TEST(Models, UnknownModelThrows)
{
  EXPECT_THROW(mmn::footprint_model("cylinder", parameters(), projector()), std::invalid_argument);
}
