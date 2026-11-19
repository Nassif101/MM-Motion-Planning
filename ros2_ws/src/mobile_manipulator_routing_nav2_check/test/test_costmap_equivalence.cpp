// S1 (spec 2 Amendment 1): the routing core's costmap port equals nav2_costmap_2d's static + inflation layers
// byte for byte, for the reference settings (track_unknown_space false, trinary, padding 0.01 m, inflation 1.0 m
// at scaling 5.0).
#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "mobile_manipulator_routing/map_io.hpp"
#include "mobile_manipulator_routing/smac/costmap.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/static_layer.hpp"
#include "nav2_map_server/map_io.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace mr = mobile_manipulator_routing;
namespace sm = mobile_manipulator_routing::smac;

namespace
{
struct StaticLayer : nav2_costmap_2d::StaticLayer
{
  using nav2_costmap_2d::StaticLayer::incomingMap;
};

const sm::Footprint kHome{{0.54, 0.62}, {0.54, -0.62}, {-0.70, -0.62}, {-0.70, 0.62}};
// An irregular convex polygon like the projected base hull (16 vertices, off-centre).
sm::Footprint irregular()
{
  sm::Footprint f;
  for (int i = 0; i < 16; ++i) {
    const double a = 2.0 * M_PI * i / 16.0 + 0.07;
    const double rx = std::cos(a) > 0 ? 0.58 : 0.71, ry = std::sin(a) > 0 ? 0.43 : 0.47;
    f.push_back({rx * std::cos(a) + 0.03, ry * std::sin(a)});
  }
  return f;
}

std::vector<unsigned char> nav2_costmap(const nav_msgs::msg::OccupancyGrid & map, const sm::Footprint & footprint)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("costmap_equivalence");
  node->declare_parameter("map_topic", rclcpp::ParameterValue(std::string("map")));
  node->declare_parameter("track_unknown_space", rclcpp::ParameterValue(false));
  node->declare_parameter("use_maximum", rclcpp::ParameterValue(false));
  node->declare_parameter("lethal_cost_threshold", rclcpp::ParameterValue(100));
  node->declare_parameter("unknown_cost_value", rclcpp::ParameterValue(255));
  node->declare_parameter("trinary_costmap", rclcpp::ParameterValue(true));
  node->declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));
  node->declare_parameter("inflation.inflation_radius", rclcpp::ParameterValue(1.0));
  node->declare_parameter("inflation.cost_scaling_factor", rclcpp::ParameterValue(5.0));
  node->declare_parameter("inflation.inflate_unknown", rclcpp::ParameterValue(false));
  node->declare_parameter("inflation.inflate_around_unknown", rclcpp::ParameterValue(false));
  tf2_ros::Buffer tf(node->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("map", false, false);
  auto slayer = std::make_shared<StaticLayer>();
  layers.addPlugin(slayer);
  slayer->initialize(&layers, "static", &tf, node, nullptr);
  auto ilayer = std::make_shared<nav2_costmap_2d::InflationLayer>();
  ilayer->initialize(&layers, "inflation", &tf, node, nullptr);
  layers.addPlugin(ilayer);
  slayer->incomingMap(std::make_shared<nav_msgs::msg::OccupancyGrid>(map));
  std::vector<geometry_msgs::msg::Point> polygon;
  for (const auto & p : footprint) {
    geometry_msgs::msg::Point q;
    q.x = p.x;
    q.y = p.y;
    polygon.push_back(q);
  }
  nav2_costmap_2d::padFootprint(polygon, 0.01);  // as Costmap2DROS::setRobotFootprint
  layers.setFootprint(polygon);
  layers.updateMap(0, 0, 0);
  const auto * costmap = layers.getCostmap();
  EXPECT_EQ(costmap->getSizeInCellsX(), map.info.width);
  EXPECT_EQ(costmap->getSizeInCellsY(), map.info.height);
  const auto * data = costmap->getCharMap();
  return std::vector<unsigned char>(data, data + costmap->getSizeInCellsX() * costmap->getSizeInCellsY());
}

void expect_identical(const std::vector<unsigned char> & nav2, const sm::LayeredCostmap & port)
{
  ASSERT_EQ(nav2.size(), port.costmap.data().size());
  std::size_t mismatches = 0, inflated = 0;
  for (std::size_t i = 0; i < nav2.size(); ++i) {
    if (nav2[i] != port.costmap.data()[i] && mismatches++ < 5) {
      ADD_FAILURE() << "cell " << i << ": nav2 " << int(nav2[i]) << ", port " << int(port.costmap.data()[i]);
    }
    if (nav2[i] > 0 && nav2[i] < sm::kLethalObstacle) ++inflated;
  }
  EXPECT_EQ(mismatches, 0u);
  EXPECT_GT(inflated, 0u);  // the comparison covers the inflation field, not only lethal cells
}

nav_msgs::msg::OccupancyGrid boxes(std::uint32_t seed, mr::GridSnapshot *& out_grid,
                                   std::unique_ptr<mr::GridSnapshot> & storage)
{
  const int w = 240, h = 180;
  std::mt19937 rng(seed);
  std::vector<mr::Cell> cells(static_cast<std::size_t>(w) * h, mr::Cell::Free);
  std::uniform_int_distribution<int> x(0, w - 1), y(0, h - 1), side(1, 14), kind(0, 9);
  for (int b = 0; b < 40; ++b) {
    const int x0 = x(rng), y0 = y(rng), sx = side(rng), sy = side(rng);
    const auto value = kind(rng) == 0 ? mr::Cell::Unknown : mr::Cell::Occupied;
    for (int j = y0; j < std::min(h, y0 + sy); ++j)
      for (int i = x0; i < std::min(w, x0 + sx); ++i) cells[static_cast<std::size_t>(j) * w + i] = value;
  }
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.width = w;
  map.info.height = h;
  map.info.resolution = 0.05f;
  map.info.origin.position.x = -3.0;
  map.info.origin.position.y = -2.5;
  map.info.origin.orientation.w = 1.0;
  map.data.resize(cells.size());
  for (std::size_t i = 0; i < cells.size(); ++i)
    map.data[i] = cells[i] == mr::Cell::Occupied ? 100 : cells[i] == mr::Cell::Unknown ? -1 : 0;
  storage = std::make_unique<mr::GridSnapshot>(w, h, 0.05, mr::Point2{-3.0, -2.5}, std::move(cells));
  out_grid = storage.get();
  return map;
}
}  // namespace

TEST(CostmapEquivalence, V2MapMatchesNav2ForBothFootprints)
{
  const std::string yaml = std::string(NAVIGATION_SOURCE) + "/maps/construction_site_v2.yaml";
  nav_msgs::msg::OccupancyGrid map;
  ASSERT_EQ(nav2_map_server::loadMapFromYaml(yaml, map), nav2_map_server::LOAD_MAP_SUCCESS);
  const auto grid = mr::load_map(yaml);
  for (const auto & footprint : {kHome, irregular()}) {
    const auto port = sm::build_costmap(grid, footprint, {});
    expect_identical(nav2_costmap(map, footprint), port);
  }
}

TEST(CostmapEquivalence, SeededBoxMapsWithUnknownCellsMatchNav2)
{
  for (std::uint32_t seed : {1u, 2u, 3u}) {
    mr::GridSnapshot * grid = nullptr;
    std::unique_ptr<mr::GridSnapshot> storage;
    const auto map = boxes(seed, grid, storage);
    for (const auto & footprint : {kHome, irregular()}) {
      SCOPED_TRACE("seed " + std::to_string(seed));
      expect_identical(nav2_costmap(map, footprint), sm::build_costmap(*grid, footprint, {}));
    }
  }
}

TEST(CostmapEquivalence, RadiiMatchNav2)
{
  for (const auto & footprint : {kHome, irregular()}) {
    std::vector<geometry_msgs::msg::Point> polygon;
    for (const auto & p : footprint) {
      geometry_msgs::msg::Point q;
      q.x = p.x;
      q.y = p.y;
      polygon.push_back(q);
    }
    nav2_costmap_2d::padFootprint(polygon, 0.01);
    const auto [inscribed, circumscribed] = nav2_costmap_2d::calculateMinAndMaxDistances(polygon);
    mr::GridSnapshot grid(20, 20, 0.05, {0, 0}, std::vector<mr::Cell>(400, mr::Cell::Free));
    const auto port = sm::build_costmap(grid, footprint, {});
    EXPECT_EQ(port.inscribed_radius, inscribed);
    EXPECT_EQ(port.circumscribed_radius, circumscribed);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
