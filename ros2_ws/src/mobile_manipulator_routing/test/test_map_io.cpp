#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include "mobile_manipulator_routing/map_io.hpp"

namespace mr = mobile_manipulator_routing;

namespace
{
class RoutingMapIO : public ::testing::Test
{
protected:
  void SetUp() override
  {
    static std::atomic<unsigned> sequence{0};
    for (unsigned attempt = 0; attempt < 20; ++attempt) {
      const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
      directory_ = std::filesystem::temp_directory_path() /
        ("mm-routing-map-" + std::to_string(stamp) + "-" + std::to_string(sequence++));
      if (std::filesystem::create_directory(directory_)) return;
    }
    FAIL() << "could not create a unique map fixture directory";
  }
  void TearDown() override
  {
    std::error_code ignored;
    std::filesystem::remove_all(directory_, ignored);
  }
  void image(const std::string & header, const std::vector<unsigned char> & pixels)
  {
    std::ofstream output(directory_ / "map.pgm", std::ios::binary);
    output << header;
    output.write(reinterpret_cast<const char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
    ASSERT_TRUE(output.good());
  }
  std::filesystem::path yaml(const std::string & overrides = "")
  {
    std::ofstream output(directory_ / "map.yaml");
    output << "image: map.pgm\nmode: trinary\nresolution: 0.5\norigin: [-2, -1, 0]\n"
              "negate: 0\noccupied_thresh: 0.8\nfree_thresh: 0.2\n" << overrides;
    output.close();
    return directory_ / "map.yaml";
  }
  void replace_yaml(const std::string & text)
  {
    std::ofstream output(directory_ / "map.yaml");
    output << text;
  }
  std::filesystem::path directory_;
};
}  // namespace

TEST_F(RoutingMapIO, LoadsAsymmetricImageWithRowInversionAndThresholdEdges)
{
  // Image row zero is north/top; probabilities exactly equal to either threshold are unknown.
  image("P5\n# an ordinary map comment\n3 2\n255\n", {255, 0, 204, 51, 128, 255});
  const auto map = mr::load_map(yaml(), {0.0, 0.0});
  EXPECT_EQ(map.width(), 3);
  EXPECT_EQ(map.height(), 2);
  EXPECT_DOUBLE_EQ(map.resolution(), 0.5);
  EXPECT_DOUBLE_EQ(map.origin().x, -2.0);
  EXPECT_EQ(map.occupancy(0, 1), mr::Cell::Free);
  EXPECT_EQ(map.occupancy(1, 1), mr::Cell::Occupied);
  EXPECT_EQ(map.occupancy(2, 1), mr::Cell::Unknown);
  EXPECT_EQ(map.occupancy(0, 0), mr::Cell::Unknown);
  EXPECT_EQ(map.occupancy(1, 0), mr::Cell::Unknown);
  EXPECT_EQ(map.occupancy(2, 0), mr::Cell::Free);
  EXPECT_EQ(map.world_to_cell({-0.75, -0.75})->x, 2);
  EXPECT_EQ(map.world_to_cell({-0.75, -0.75})->y, 0);
}

TEST_F(RoutingMapIO, NegateChangesPixelInterpretationAndModeMayDefaultToTrinary)
{
  image("P5\n2 1\n255\n", {0, 255});
  yaml();
  replace_yaml("image: map.pgm\nresolution: 1\norigin: [0,0,0]\nnegate: 1\n"
               "occupied_thresh: 0.65\nfree_thresh: 0.25\n");
  const auto map = mr::load_map(directory_ / "map.yaml");
  EXPECT_EQ(map.occupancy(0, 0), mr::Cell::Free);
  EXPECT_EQ(map.occupancy(1, 0), mr::Cell::Occupied);
}

TEST_F(RoutingMapIO, PreservesFirstBinaryPixelEvenWhenItIsWhitespaceOrHash)
{
  for (const unsigned char first : {' ', '\n', '\r', '\t', '#'}) {
    image("P5\n2 1\n255\n", {first, 255});
    const auto map = mr::load_map(yaml(), {0.0, 0.0});
    EXPECT_EQ(map.occupancy(0, 0), mr::Cell::Occupied) << static_cast<int>(first);
    EXPECT_EQ(map.occupancy(1, 0), mr::Cell::Free);
  }
}

TEST_F(RoutingMapIO, AcceptsCRLFHeaderWithoutDiscardingFirstPixel)
{
  image("P5\r\n# comment\r\n2 1\r\n255\r\n", {'\n', 255});
  const auto map = mr::load_map(yaml(), {0.0, 0.0});
  EXPECT_EQ(map.occupancy(0, 0), mr::Cell::Occupied);
  EXPECT_EQ(map.occupancy(1, 0), mr::Cell::Free);

  image("P5\n2 1\n255\r", {'\n', 255});
  const auto cr_only = mr::load_map(yaml(), {0.0, 0.0});
  EXPECT_EQ(cr_only.occupancy(0, 0), mr::Cell::Occupied);
  EXPECT_EQ(cr_only.occupancy(1, 0), mr::Cell::Free);
}

TEST_F(RoutingMapIO, RejectsTruncatedExtraUnsupportedAndOverflowingImages)
{
  const auto path = yaml();
  image("P5\n2 2\n255\n", {0, 255, 0});
  EXPECT_THROW(mr::load_map(path), std::runtime_error);
  image("P5\n2 1\n255\n", {0, 255, 0});
  EXPECT_THROW(mr::load_map(path), std::runtime_error);
  image("P2\n2 1\n255\n", {0, 255});
  EXPECT_THROW(mr::load_map(path), std::runtime_error);
  image("P5\n2 1\n65535\n", {0, 255, 0, 255});
  EXPECT_THROW(mr::load_map(path), std::runtime_error);
  image("P5\n0 1\n255\n", {});
  EXPECT_THROW(mr::load_map(path), std::runtime_error);
  image("P5\n65536 65536\n255\n", {});
  EXPECT_THROW(mr::load_map(path), std::runtime_error);
  image("P5\n18446744073709551616 1\n255\n", {});
  EXPECT_THROW(mr::load_map(path), std::runtime_error);
}

TEST_F(RoutingMapIO, RejectsUnsupportedAmbiguousAndInvalidMetadata)
{
  image("P5\n1 1\n255\n", {255});
  auto path = yaml("mode: scale\n");
  EXPECT_THROW(mr::load_map(path), std::invalid_argument);  // also a duplicate key
  const std::string prefix = "image: map.pgm\nresolution: 0.05\nnegate: 0\n"
                             "occupied_thresh: 0.65\nfree_thresh: 0.25\n";
  for (const std::string origin : {"[0,0,0.1]", "[0,0]", "[.nan,0,0]"}) {
    replace_yaml(prefix + "origin: " + origin + "\n");
    EXPECT_THROW(mr::load_map(path), std::invalid_argument);
  }
  replace_yaml(prefix + "origin: [0,0,0]\nmode: raw\n");
  EXPECT_THROW(mr::load_map(path), std::invalid_argument);
  replace_yaml(prefix + "origin: [0,0,0]\nunknown_field: true\n");
  EXPECT_THROW(mr::load_map(path), std::invalid_argument);
  replace_yaml("image: map.pgm\nmode: trinary\nresolution: -1\norigin: [0,0,0]\n"
               "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n");
  EXPECT_THROW(mr::load_map(path), std::invalid_argument);
  replace_yaml("image: map.pgm\nresolution: 1\norigin: [0,0,0]\n"
               "negate: 2\noccupied_thresh: 0.65\nfree_thresh: 0.25\n");
  EXPECT_THROW(mr::load_map(path), std::invalid_argument);
  replace_yaml("image: map.pgm\nresolution: 1\norigin: [0,0,0]\n"
               "negate: 0\noccupied_thresh: 0.2\nfree_thresh: 0.3\n");
  EXPECT_THROW(mr::load_map(path), std::invalid_argument);
  replace_yaml("image: map.pgm\nresolution: 1\norigin: [0,0,0]\nnegate: 0\n");
  EXPECT_THROW(mr::load_map(path), std::invalid_argument);
}

TEST_F(RoutingMapIO, BoundsHeaderLengthBeforeAllocatingRaster)
{
  image("P5\n#" + std::string(65536, 'a') + "\n1 1\n255\n", {255});
  EXPECT_THROW(mr::load_map(yaml()), std::runtime_error);
}
