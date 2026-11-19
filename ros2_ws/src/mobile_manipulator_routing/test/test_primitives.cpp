#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "mobile_manipulator_routing/primitives.hpp"

namespace mr = mobile_manipulator_routing;
using Json = nlohmann::json;

namespace
{
class PrimitiveFile
{
public:
  PrimitiveFile()
  {
    static std::atomic<unsigned> sequence{0};
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    directory_ = std::filesystem::temp_directory_path() /
      ("routing-primitives-" + std::to_string(now) + "-" + std::to_string(sequence++));
    if (!std::filesystem::create_directory(directory_)) throw std::runtime_error("test directory creation failed");
  }
  ~PrimitiveFile()
  {
    std::error_code ignored;
    std::filesystem::remove_all(directory_, ignored);
  }
  std::filesystem::path path() const { return directory_ / "primitives.json"; }
  void write(const Json & data) const
  {
    std::ofstream stream(path(), std::ios::binary);
    stream << data.dump();
    if (!stream) throw std::runtime_error("test primitive write failed");
  }
private:
  std::filesystem::path directory_;
};

Json trajectory(std::size_t id, std::size_t heading, double yaw, double x, double y)
{
  const double length = std::hypot(x, y);
  return {{"trajectory_id", id}, {"start_angle_index", heading}, {"end_angle_index", heading},
    {"left_turn", false}, {"trajectory_radius", 0.0}, {"trajectory_length", length},
    {"arc_length", 0.0}, {"straight_length", length}, {"poses", {{x, y, yaw}}}};
}

Json nonuniform_fixture()
{
  const double oblique = std::atan(0.5);
  const std::vector<double> angles{0.0, oblique, mr::kPi / 2.0, 2.0 * mr::kPi - oblique};
  return {{"version", "1.0"},
    {"lattice_metadata", {{"grid_resolution", 0.05}, {"motion_model", "diff"}, {"turning_radius", 0.5},
      {"num_of_headings", 4}, {"heading_angles", angles}, {"number_of_trajectories", 4}}},
    {"primitives", {trajectory(40, 0, angles[0], 0.05, 0.0),
      trajectory(30, 1, angles[1], 0.10, 0.05), trajectory(20, 2, angles[2], 0.0, 0.05),
      trajectory(10, 3, angles[3], 0.10, -0.05)}}};
}

void add_clockwise_rotation(Json & data)
{
  data["primitives"].push_back({{"trajectory_id", 5}, {"start_angle_index", 0}, {"end_angle_index", 3},
    {"left_turn", false}, {"trajectory_radius", 0.0}, {"trajectory_length", 0.0},
    {"arc_length", 0.0}, {"straight_length", 0.0},
    {"poses", {{0.0, 0.0, 0.0}, {0.0, 0.0, 2.0 * mr::kPi - 0.2},
      {0.0, 0.0, 2.0 * mr::kPi - std::atan(0.5)}}}});
  data["lattice_metadata"]["number_of_trajectories"] = data["primitives"].size();
}
}  // namespace

TEST(Primitives, PreservesNonuniformHeadingBankAndMetricEndpoints)
{
  PrimitiveFile file;
  file.write(nonuniform_fixture());
  const auto data = mr::PrimitiveSet::load(file.path());
  ASSERT_EQ(data.headings.size(), 4u);
  EXPECT_DOUBLE_EQ(data.headings[1], std::atan(0.5));
  EXPECT_EQ(data.nearest_heading(0.22), 0u);
  EXPECT_EQ(data.nearest_heading(0.30), 1u);
  EXPECT_EQ(data.nearest_heading(-0.45), 3u);
  EXPECT_EQ(data.nearest_heading(2.0 * mr::kPi + 0.30), 1u);
  ASSERT_EQ(data.motions.size(), 4u);
  const auto & diagonal = data.motions[data.successors[1].front()];
  EXPECT_EQ(diagonal.dx, 2);
  EXPECT_EQ(diagonal.dy, 1);
  EXPECT_NEAR(diagonal.length_m, std::hypot(0.10, 0.05), 1e-12);
  EXPECT_FALSE(diagonal.pure_rotation);
  EXPECT_FALSE(diagonal.turning);
  EXPECT_GT(data.storage_bytes(), sizeof(mr::PrimitiveSet));
}

TEST(Primitives, PrependsDeclaredOriginWithoutShiftingSourcePositions)
{
  PrimitiveFile file;
  auto source = nonuniform_fixture();
  source["version"] = 1.0;
  file.write(source);
  const auto data = mr::PrimitiveSet::load(file.path());
  const auto & diagonal = data.motions[data.successors[1].front()];
  ASSERT_EQ(diagonal.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(diagonal.poses[0].x, 0.0);
  EXPECT_DOUBLE_EQ(diagonal.poses[0].y, 0.0);
  EXPECT_DOUBLE_EQ(diagonal.poses[0].yaw, data.headings[1]);
  EXPECT_DOUBLE_EQ(diagonal.poses[1].x, 0.10);
  EXPECT_DOUBLE_EQ(diagonal.poses[1].y, 0.05);
}

TEST(Primitives, WrappedRotationHasContinuousPositiveAngularTravel)
{
  PrimitiveFile file;
  auto source = nonuniform_fixture();
  add_clockwise_rotation(source);
  file.write(source);
  const auto data = mr::PrimitiveSet::load(file.path());
  ASSERT_EQ(data.motions.front().id, 5u);
  const auto & rotation = data.motions.front();
  EXPECT_TRUE(rotation.pure_rotation);
  EXPECT_TRUE(rotation.turning);
  EXPECT_DOUBLE_EQ(rotation.length_m, 0.0);
  EXPECT_NEAR(rotation.angular_travel, std::atan(0.5), 1e-12);
  ASSERT_EQ(rotation.poses.size(), 3u);
  EXPECT_NEAR(rotation.poses[1].yaw, -0.2, 1e-12);
  EXPECT_NEAR(rotation.poses.back().yaw, -std::atan(0.5), 1e-12);
  EXPECT_EQ(rotation.dx, 0);
  EXPECT_EQ(rotation.dy, 0);
  const auto & from_zero = data.successors[0];
  ASSERT_EQ(from_zero.size(), 2u);
  EXPECT_LT(data.motions[from_zero[0]].id, data.motions[from_zero[1]].id);
}

TEST(Primitives, IncludesImplicitInitialAngularInterval)
{
  PrimitiveFile file;
  auto source = nonuniform_fixture();
  add_clockwise_rotation(source);
  source["primitives"].back()["poses"].erase(source["primitives"].back()["poses"].begin());
  file.write(source);
  const auto data = mr::PrimitiveSet::load(file.path());
  EXPECT_NEAR(data.motions.front().angular_travel, std::atan(0.5), 1e-12);
  EXPECT_DOUBLE_EQ(data.motions.front().poses.front().yaw, 0.0);
}

TEST(Primitives, RejectsMalformedSchemaAndUnsupportedMotion)
{
  using Mutation = std::pair<std::string, std::function<void(Json &)>>;
  const std::vector<Mutation> mutations{
    {"unsupported version", [](Json & d) { d["version"] = "2.0"; }},
    {"unsupported model", [](Json & d) { d["lattice_metadata"]["motion_model"] = "car"; }},
    {"resolution mismatch", [](Json & d) { d["lattice_metadata"]["grid_resolution"] = 0.1; }},
    {"zero resolution", [](Json & d) { d["lattice_metadata"]["grid_resolution"] = 0.0; }},
    {"negative radius", [](Json & d) { d["lattice_metadata"]["turning_radius"] = -0.5; }},
    {"heading count mismatch", [](Json & d) { d["lattice_metadata"]["num_of_headings"] = 3; }},
    {"duplicate angle", [](Json & d) { d["lattice_metadata"]["heading_angles"][1] = 0.0; }},
    {"angle range", [](Json & d) { d["lattice_metadata"]["heading_angles"][0] = -0.1; }},
    {"missing field", [](Json & d) { d["lattice_metadata"].erase("heading_angles"); }},
    {"trajectory count mismatch", [](Json & d) { d["lattice_metadata"]["number_of_trajectories"] = 5; }},
    {"duplicate ID", [](Json & d) { d["primitives"][1]["trajectory_id"] = d["primitives"][0]["trajectory_id"]; }},
    {"negative ID", [](Json & d) { d["primitives"][0]["trajectory_id"] = -1; }},
    {"fractional index", [](Json & d) { d["primitives"][0]["start_angle_index"] = 0.5; }},
    {"out of range index", [](Json & d) { d["primitives"][0]["end_angle_index"] = 4; }},
    {"endpoint angle mismatch", [](Json & d) { d["primitives"][0]["end_angle_index"] = 1; }},
    {"off lattice endpoint", [](Json & d) { d["primitives"][0]["poses"][0][0] = 0.055; }},
    {"wrong pose arity", [](Json & d) { d["primitives"][0]["poses"][0].push_back(0.0); }},
    {"empty poses", [](Json & d) { d["primitives"][0]["poses"] = Json::array(); }},
    {"nonnumeric pose", [](Json & d) { d["primitives"][0]["poses"][0][0] = "0.05"; }},
    {"negative length", [](Json & d) { d["primitives"][0]["trajectory_length"] = -0.05; }},
    {"length disagreement", [](Json & d) { d["primitives"][0]["trajectory_length"] = 1.0; }},
    {"reverse translation", [](Json & d) { d["primitives"][0]["poses"][0][0] = -0.05; }},
    {"no motion", [](Json & d) {
      auto & m = d["primitives"][0]; m["poses"] = {{0.0, 0.0, 0.0}};
      m["trajectory_length"] = 0.0; m["straight_length"] = 0.0;
    }},
    {"uncovered heading", [](Json & d) {
      d["primitives"].erase(d["primitives"].begin());
      d["lattice_metadata"]["number_of_trajectories"] = d["primitives"].size();
    }}
  };
  PrimitiveFile file;
  for (const auto & [name, mutate] : mutations) {
    SCOPED_TRACE(name);
    auto source = nonuniform_fixture();
    mutate(source);
    file.write(source);
    EXPECT_THROW(mr::PrimitiveSet::load(file.path()), std::invalid_argument);
  }
}

TEST(Primitives, BoundsHeadingAndTrajectoryCountsBeforeMotionAllocation)
{
  PrimitiveFile file;
  auto source = nonuniform_fixture();
  source["lattice_metadata"]["num_of_headings"] = 257;
  source["lattice_metadata"]["heading_angles"] = std::vector<double>(257, 0.0);
  file.write(source);
  EXPECT_THROW(mr::PrimitiveSet::load(file.path()), std::invalid_argument);
  source = nonuniform_fixture();
  source["lattice_metadata"]["number_of_trajectories"] = 65537;
  source["primitives"] = Json::array();
  for (int i = 0; i < 65537; ++i) source["primitives"].push_back(Json::object());
  file.write(source);
  EXPECT_THROW(mr::PrimitiveSet::load(file.path()), std::invalid_argument);
}

TEST(Primitives, BoundsFileSizeAndRejectsMalformedJson)
{
  PrimitiveFile file;
  {
    std::ofstream stream(file.path(), std::ios::binary);
    stream.seekp(8u * 1024u * 1024u);
    stream.put('\0');
  }
  EXPECT_THROW(mr::PrimitiveSet::load(file.path()), std::invalid_argument);
  {
    std::ofstream stream(file.path(), std::ios::binary | std::ios::trunc);
    stream << "{\"version\":1e999}";
  }
  EXPECT_THROW(mr::PrimitiveSet::load(file.path()), std::invalid_argument);
  EXPECT_THROW(mr::PrimitiveSet::load(file.path(), 0.0), std::invalid_argument);
  EXPECT_THROW(mr::PrimitiveSet::load(file.path(), std::numeric_limits<double>::infinity()), std::invalid_argument);
  EXPECT_THROW(mr::PrimitiveSet::load(file.path().parent_path() / "absent.json"), std::runtime_error);
}

TEST(Primitives, NearestHeadingRejectsInvalidQueryAndUsesStableTies)
{
  mr::PrimitiveSet data;
  EXPECT_THROW(data.nearest_heading(0.0), std::invalid_argument);
  data.headings = {0.0, mr::kPi / 2.0};
  EXPECT_EQ(data.nearest_heading(mr::kPi / 4.0), 0u);
  EXPECT_THROW(data.nearest_heading(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}

TEST(Primitives, PinnedDifferentialDriveDataMatchesItsDeclaredContract)
{
  const auto path = std::filesystem::path(__FILE__).parent_path().parent_path() / "data" / "diff_5cm.json";
  const auto data = mr::PrimitiveSet::load(path);
  EXPECT_DOUBLE_EQ(data.resolution, 0.05);
  ASSERT_EQ(data.headings.size(), 16u);
  ASSERT_EQ(data.motions.size(), 112u);
  EXPECT_NEAR(data.headings[1], std::atan(0.5), 1e-12);
  EXPECT_EQ(data.nearest_heading(0.22), 0u);
  EXPECT_EQ(std::count_if(data.motions.begin(), data.motions.end(),
    [](const auto & m) { return m.pure_rotation; }), 32);
  for (std::size_t heading = 0; heading < data.headings.size(); ++heading) {
    ASSERT_EQ(data.successors[heading].size(), 7u);
    for (auto index : data.successors[heading]) {
      const auto & m = data.motions[index];
      EXPECT_EQ(m.start_heading, heading);
      EXPECT_DOUBLE_EQ(m.poses.front().x, 0.0);
      EXPECT_DOUBLE_EQ(m.poses.front().y, 0.0);
      EXPECT_DOUBLE_EQ(m.poses.front().yaw, data.headings[heading]);
      if (m.pure_rotation) EXPECT_GT(m.angular_travel, 0.0);
      else EXPECT_GE(m.length_m + 1e-12, std::hypot(m.poses.back().x, m.poses.back().y));
    }
  }
}
