// Frames (one schema for the demo stream and the run log) and the run log (MuJoCo plant spec Section 6, plan Task 8).
#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

#include "mobile_manipulator_plant/frames.hpp"
#include "mobile_manipulator_plant/run_log.hpp"

namespace mmp = mobile_manipulator_plant;
using json = nlohmann::json;

namespace
{
std::string temp(const std::string & name) { return (std::filesystem::temp_directory_path() / name).string(); }
std::string bytes_of(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(in), {});
}
}  // namespace

TEST(Frames, RoundTrip)
{
  const json header = {{"t", 1.5}, {"name", "door_105"}, {"nested", {{"a", {1, 2}}}}};
  const std::vector<std::vector<float>> blocks{{1.0f, 2.0f, 3.0f}, {}, {4.5f}};
  const std::string frame = mmp::encode(mmp::FrameType::State, header, blocks);
  mmp::FrameType type{};
  json h;
  std::vector<std::vector<float>> b;
  size_t consumed = 0;
  ASSERT_TRUE(mmp::decode(frame, type, h, b, consumed));
  EXPECT_EQ(type, mmp::FrameType::State);
  EXPECT_EQ(h, header);
  EXPECT_EQ(b, blocks);
  EXPECT_EQ(consumed, frame.size());
}

TEST(Frames, DecodesAStreamFrameByFrame)
{
  const std::string stream = mmp::encode(mmp::FrameType::Hello, {{"schema", mmp::kSchemaVersion}}, {}) +
                             mmp::encode(mmp::FrameType::Event, {{"event", "goal_reached"}}, {{0.5f}});
  mmp::FrameType type{};
  json h;
  std::vector<std::vector<float>> b;
  size_t consumed = 0;
  ASSERT_TRUE(mmp::decode(stream, type, h, b, consumed));
  EXPECT_EQ(type, mmp::FrameType::Hello);
  EXPECT_EQ(h.at("schema"), mmp::kSchemaVersion);
  ASSERT_TRUE(mmp::decode(std::string_view(stream).substr(consumed), type, h, b, consumed));
  EXPECT_EQ(type, mmp::FrameType::Event);
  EXPECT_EQ(h.at("event"), "goal_reached");
}

TEST(Frames, RejectsTruncated)
{
  const std::string frame = mmp::encode(mmp::FrameType::Scan, {{"scan", 3}}, {{1.0f, 2.0f}, {3.0f}});
  for (size_t n = 0; n < frame.size(); ++n) {
    mmp::FrameType type{};
    json h;
    std::vector<std::vector<float>> b;
    size_t consumed = 7;
    EXPECT_FALSE(mmp::decode(std::string_view(frame).substr(0, n), type, h, b, consumed)) << n;
    EXPECT_EQ(consumed, 7u);  // untouched: wait for more bytes
  }
}

TEST(Frames, RejectsMalformed)
{
  std::string frame = mmp::encode(mmp::FrameType::State, {{"t", 1.0}}, {{1.0f}});
  frame[10] = '}';  // inside the JSON
  mmp::FrameType type{};
  json h;
  std::vector<std::vector<float>> b;
  size_t consumed = 0;
  EXPECT_THROW(mmp::decode(frame, type, h, b, consumed), std::runtime_error);
}

TEST(RunLog, RoundTripAndHeader)
{
  const std::string path = temp("test_run_log.mmlog");
  const json header = {{"schema", mmp::kSchemaVersion}, {"scenario", "v2_door_105_nav"}, {"seed", 7}};
  std::vector<std::string> frames;
  for (int k = 0; k < 3000; ++k)  // more than one 1 MiB chunk
    frames.push_back(mmp::encode(mmp::FrameType::State, {{"t", 0.02 * k}}, {std::vector<float>(100, static_cast<float>(k))}));
  {
    mmp::RunLogWriter w(path, header);
    for (const auto & f : frames) w.write(f);
    w.close();
  }
  mmp::RunLogReader r(path);
  EXPECT_EQ(r.header(), header);
  std::string frame;
  size_t n = 0;
  while (r.next(frame)) {
    ASSERT_LT(n, frames.size());
    EXPECT_EQ(frame, frames[n]);
    ++n;
  }
  EXPECT_EQ(n, frames.size());
  std::filesystem::remove(path);
}

TEST(RunLog, SameFramesSameBytes)
{
  // E2 for logs: the same run writes the same file
  const std::string a = temp("test_run_log_a.mmlog"), b = temp("test_run_log_b.mmlog");
  for (const std::string & path : {a, b}) {
    mmp::RunLogWriter w(path, {{"seed", 1}});
    for (int k = 0; k < 500; ++k) w.write(mmp::encode(mmp::FrameType::State, {{"t", 0.02 * k}}, {{static_cast<float>(k)}}));
    w.close();
  }
  EXPECT_EQ(bytes_of(a), bytes_of(b));
  std::filesystem::remove(a);
  std::filesystem::remove(b);
}

TEST(RunLog, RejectsAFileThatIsNotALog)
{
  const std::string path = temp("test_not_a_log.mmlog");
  std::ofstream(path) << "hello";
  EXPECT_THROW(mmp::RunLogReader{path}, std::runtime_error);
  std::filesystem::remove(path);
}
