// The plant's scans at a fixed pose for gate L (MuJoCo plant plan Task 10): the robot held still in its scenario's arm
// pose, 20 scans, the primitives the comparison labels Unity's points with.
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <set>

#include "mobile_manipulator_campaign/lidar_fixture.hpp"
#include "mobile_manipulator_plant/frames.hpp"
#include "mobile_manipulator_plant/run_log.hpp"

namespace mmc = mobile_manipulator_campaign;
namespace mmp = mobile_manipulator_plant;
using json = nlohmann::json;

namespace
{
mmc::SessionConfig realistic()
{
  mmc::SessionConfig c;
  c.mode = mmp::PlantMode::Realistic;
  return c;
}
}  // namespace

TEST(LidarFixture, HoldsStillAndScans)
{
  const mmp::Pose2 pose{-6.0, -3.5, -0.523599};  // the open yard
  const auto r = mmc::lidar_fixture(realistic(), "v2_open_yard_nav", pose, 20, 1);
  ASSERT_EQ(r.scans.size(), 20u);
  for (const auto & s : r.scans) EXPECT_GT(s.points.size(), 5000u);  // of 20000 rays: the open sky returns nothing
  EXPECT_LT(r.header.at("drift_m").get<double>(), 1e-6);  // standing on its wheels, holding still (2e-11 m measured)
  EXPECT_EQ(r.header.at("contacts").get<int>(), 0);
  const auto base = r.header.at("base");  // 4 x 4, row-major nested
  EXPECT_NEAR(base[0][3].get<double>(), pose.x, 0.002);
  EXPECT_NEAR(base[1][3].get<double>(), pose.y, 0.002);
  EXPECT_NEAR(std::atan2(base[1][0].get<double>(), base[0][0].get<double>()), pose.yaw, 0.002);
  const auto lidar = r.header.at("lidar_in_base");
  EXPECT_NEAR(lidar[0][3].get<double>(), 0.24, 0.05);  // the Mid-360 on the top sensor mount, ahead of the centre
  std::set<std::string> labels;
  for (const auto & p : r.header.at("prims")) labels.insert(p.at("label").get<std::string>());
  EXPECT_TRUE(labels.count("site") && labels.count("robot") && labels.count("panel"));
  EXPECT_EQ(r.header.at("q").size(), 6u);
}

TEST(LidarFixture, CarriesTheScenarioBoxes)
{
  // the 1.23 m beam ahead: the scenario's own box is a primitive and some points hit it
  const auto r = mmc::lidar_fixture(realistic(), "v2_low_beam_123_nav", {-1.2, -5.5, 0.0}, 2, 1);
  bool beam = false;
  for (const auto & p : r.header.at("prims")) beam = beam || p.at("label") == "scenario";
  EXPECT_TRUE(beam);
  int hits = 0;
  for (const auto & s : r.scans)
    for (auto c : s.cls) hits += c == mmp::HitClass::Scenario;
  EXPECT_GT(hits, 0);
}

TEST(LidarFixture, WritesARunLog)
{
  const auto r = mmc::lidar_fixture(realistic(), "v2_open_yard_nav", {-6.0, -3.5, -0.523599}, 2, 1);
  const auto path = std::filesystem::temp_directory_path() / "lidar_fixture_test.mmlog";
  mmc::write_lidar_fixture(r, path.string());
  mmp::RunLogReader log(path.string());
  EXPECT_EQ(log.header().at("scenario"), "v2_open_yard_nav");
  std::string frame;
  int scans = 0;
  while (log.next(frame)) {
    mmp::FrameType type;
    json header;
    std::vector<std::vector<float>> blocks;
    size_t used = 0;
    ASSERT_TRUE(mmp::decode(frame, type, header, blocks, used));
    if (type != mmp::FrameType::Scan) continue;
    ++scans;
    ASSERT_EQ(blocks.size(), 2u);
    EXPECT_EQ(blocks[0].size(), 3 * blocks[1].size());
  }
  EXPECT_EQ(scans, 2);
  std::filesystem::remove(path);
}
