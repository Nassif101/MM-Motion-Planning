#include <gtest/gtest.h>
#include "mobile_manipulator_control/tcp_protocol.hpp"

using namespace mobile_manipulator_control;

TEST(TcpProtocol, FramesAreLengthPrefixedLittleEndian)
{
  const uint8_t payload[] = {0xAA, 0xBB, 0xCC};
  const auto frame = encode_frame("/clock", payload, 3);
  ASSERT_EQ(frame.size(), 4u + 6u + 4u + 3u);
  EXPECT_EQ(read_u32(frame.data()), 6u);
  EXPECT_EQ(std::string(frame.begin() + 4, frame.begin() + 10), "/clock");
  EXPECT_EQ(read_u32(frame.data() + 10), 3u);
  EXPECT_EQ(frame.back(), 0xCC);
  EXPECT_EQ(read_u32(std::vector<uint8_t>{0x01, 0x02, 0x00, 0x01}.data()), 0x01000201u);
}

TEST(TcpProtocol, CommandPayloadDropsTheTrailingNul)
{
  const std::string json = R"({"topic": "/clock", "message_name": "rosgraph_msgs/Clock"})";
  std::vector<uint8_t> payload(json.begin(), json.end());
  payload.push_back(0);
  EXPECT_EQ(command_json(payload), json);
  EXPECT_EQ(command_json(std::vector<uint8_t>(json.begin(), json.end())), json);
}

TEST(TcpProtocol, MessageNamesMapBetweenUnityAndRos2)
{
  EXPECT_EQ(ros_type("sensor_msgs/JointState"), "sensor_msgs/msg/JointState");
  EXPECT_EQ(ros_type("std_srvs/Trigger", "srv"), "std_srvs/srv/Trigger");
  EXPECT_EQ(ros_type("sensor_msgs/msg/JointState"), "sensor_msgs/msg/JointState");
  EXPECT_EQ(unity_type("geometry_msgs/msg/Twist"), "geometry_msgs/Twist");
}

TEST(TcpProtocol, TopicNamesRejectMisreadPayloads)
{
  EXPECT_TRUE(valid_topic_name("/mm/telemetry/path"));
  EXPECT_TRUE(valid_topic_name("/livox/lidar"));
  EXPECT_FALSE(valid_topic_name(""));
  EXPECT_FALSE(valid_topic_name(R"({"topic":"/clock"})"));
  EXPECT_FALSE(valid_topic_name(std::string("\x10\x00\x00\x00", 4)));
}

TEST(TcpProtocol, CommandsMustCarryTheirOwnKeys)
{
  const auto subscribe = nlohmann::json::parse(R"({"topic":"/mm/telemetry","message_name":"std_msgs/String"})");
  const auto publish = nlohmann::json::parse(
    R"({"topic":"/clock","message_name":"rosgraph_msgs/Clock","queue_size":10,"latch":false})");
  EXPECT_TRUE(command_fits("subscribe", subscribe));
  EXPECT_TRUE(command_fits("publish", publish));
  EXPECT_TRUE(command_fits("topic_list", nlohmann::json::object()));
  // Interleaved registrations: a command name paired with another command's JSON.
  EXPECT_FALSE(command_fits("publish", subscribe));
  EXPECT_FALSE(command_fits("subscribe", publish));
  EXPECT_FALSE(command_fits("topic_list", subscribe));
  EXPECT_FALSE(command_fits("subscribe", nlohmann::json::parse(R"({"topic":"a b","message_name":"x/Y"})")));
  EXPECT_FALSE(command_fits("subscribe", nlohmann::json::array()));
}
