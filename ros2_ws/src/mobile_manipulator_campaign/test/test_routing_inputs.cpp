#include <gtest/gtest.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include "mobile_manipulator_campaign/routing_inputs.hpp"

namespace mc=mobile_manipulator_campaign;
namespace mr=mobile_manipulator_routing;
namespace fs=std::filesystem;
namespace
{
std::string read(const fs::path & path)
{
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("fixture input missing");
  std::ostringstream value; value<<stream.rdbuf(); return value.str();
}
const fs::path packages=PACKAGES_SOURCE;
const auto navigation=packages/"mobile_manipulator_navigation";
const auto primitives=packages/"mobile_manipulator_routing/data/diff_5cm.json";
std::string robot() { return read(packages/"mobile_manipulator_description/urdf/mobile_manipulator.urdf"); }
nlohmann::ordered_json payload()
{
  return nlohmann::ordered_json::parse(read(packages/"mobile_manipulator_control/config/qualified_payload.json"));
}
struct MapFixture
{
  fs::path root=fs::temp_directory_path()/
    ("mm-routing-inputs-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  MapFixture()
  {
    if (!fs::create_directory(root)) throw std::runtime_error("cannot create test map directory");
    std::ofstream image(root/"map.pgm",std::ios::binary);
    image<<"P5\n60 60\n255\n";
    const std::string white(3600,char(255)); image.write(white.data(),white.size()); image.close();
    std::ofstream(root/"map.yaml")<<"image: map.pgm\nmode: trinary\nresolution: 0.05\n"
      "origin: [-1.5, -1.5, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n";
  }
  ~MapFixture() { std::error_code ignored; fs::remove_all(root,ignored); }
};
}

TEST(RoutingInputs, UsesBaseGeometryAndFingerprintsTheActualV2Inputs)
{
  const auto inputs=mc::load_routing_inputs(navigation.string(),robot(),payload(),"",primitives.string());
  EXPECT_EQ(inputs.grid->width(),1000);
  EXPECT_EQ(inputs.grid->height(),800);
  EXPECT_DOUBLE_EQ(inputs.grid->resolution(),0.05);
  ASSERT_GE(inputs.footprint.size(),4u);
  double xmin=1e9,xmax=-1e9,ymin=1e9,ymax=-1e9;
  for (const auto & p:inputs.footprint) {
    xmin=std::min(xmin,p.x); xmax=std::max(xmax,p.x);
    ymin=std::min(ymin,p.y); ymax=std::max(ymax,p.y);
  }
  EXPECT_GT(xmax-xmin,0.85);
  EXPECT_LT(xmax-xmin,1.1);
  EXPECT_GT(ymax-ymin,0.7);
  EXPECT_LT(ymax-ymin,1.0);  // the 1.24 m wide carry/home payload footprint is excluded
  EXPECT_EQ(inputs.metadata.at("primitive_sha256"),"105b37351d15e340cc2921fa8878c8ea727a65cd7e89c39c977be0654be3b2a9");
  EXPECT_EQ(inputs.metadata.at("known_additions").size(),0u);
  EXPECT_EQ(inputs.metadata.at("map_yaml_sha256").get<std::string>().size(),64u);
  EXPECT_DOUBLE_EQ(inputs.metadata.at("geometry_padding_m"),0.02);
  EXPECT_DOUBLE_EQ(inputs.metadata.at("map_padding_m"),0.01);
}

TEST(RoutingInputs, TheSweptValidationUsesTheGeometricPaddingOnly)
{
  // Spec 2 Amendment 3 (user, 2026-11-19): Smac checks its poses with the costmap's footprint_padding added; the swept
  // validation between them checks the hull with the geometric padding only, the polygon the costmap is configured
  // with. With both paddings, the checker's cell dilation rejected Task 13's clear store_room seed 4 route.
  const auto inputs=mc::load_routing_inputs(navigation.string(),robot(),payload(),"",primitives.string());
  ASSERT_EQ(inputs.footprint.size(),inputs.costmap_footprint.size());
  for (std::size_t i=0;i<inputs.footprint.size();++i) {
    EXPECT_DOUBLE_EQ(inputs.footprint[i].x,inputs.costmap_footprint[i].x);
    EXPECT_DOUBLE_EQ(inputs.footprint[i].y,inputs.costmap_footprint[i].y);
  }
  EXPECT_EQ(inputs.metadata.at("validation_footprint"),"geometry_padding");
}

TEST(RoutingInputs, AddsOnlyExplicitKnownGeometryInTheBaseHeightBand)
{
  MapFixture map;
  mobile_manipulator_navigation::Box low{"known",{0,0,0.2},{0.2,0.2,0.4},0.0};
  auto high=low; high.name="overhead"; high.center={0.5,0.5,2.0};
  const auto empty=mc::load_routing_inputs(navigation.string(),robot(),payload(),(map.root/"map.yaml").string(),primitives.string());
  EXPECT_FALSE(empty.grid->blocked(30,30));
  const auto added=mc::load_routing_inputs(navigation.string(),robot(),payload(),(map.root/"map.yaml").string(),primitives.string(),{low,high});
  EXPECT_TRUE(added.grid->blocked(30,30));
  EXPECT_FALSE(added.grid->blocked(40,40));
  EXPECT_GT(added.grid->revision(),empty.grid->revision());
  low.center.x()=std::numeric_limits<double>::infinity();
  EXPECT_THROW(mc::load_routing_inputs(navigation.string(),robot(),payload(),(map.root/"map.yaml").string(),primitives.string(),{low}),std::invalid_argument);
}

TEST(RoutingInputs, RejectsStaleExportMetadata)
{
  MapFixture map;
  std::ofstream(map.root/"map.metadata.json")<<"{\"pgmSha256\":\"wrong\"}";
  EXPECT_THROW(mc::load_routing_inputs(navigation.string(),robot(),payload(),(map.root/"map.yaml").string(),primitives.string()),std::invalid_argument);
}
