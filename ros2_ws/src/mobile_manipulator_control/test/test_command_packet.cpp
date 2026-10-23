#include <gtest/gtest.h>
#include "mobile_manipulator_control/command_packet.hpp"
TEST(CommandPacket, IsOneStampedPointWithAccelerations) {
  builtin_interfaces::msg::Time stamp; stamp.sec = 12; stamp.nanosec = 340000000;
  auto packet = mobile_manipulator_control::command_packet(stamp, {"a", "b"}, {0.1, 0.2}, {0.3, 0.4}, {0.5, 0.6});
  EXPECT_EQ(packet.header.stamp, stamp);
  EXPECT_EQ(packet.joint_names, (std::vector<std::string>{"a", "b"}));
  ASSERT_EQ(packet.points.size(), 1u);
  EXPECT_EQ(packet.points[0].positions, (std::vector<double>{0.1, 0.2}));
  EXPECT_EQ(packet.points[0].velocities, (std::vector<double>{0.3, 0.4}));
  EXPECT_EQ(packet.points[0].accelerations, (std::vector<double>{0.5, 0.6}));
  EXPECT_EQ(packet.points[0].time_from_start.sec, 0);
  EXPECT_EQ(packet.points[0].time_from_start.nanosec, 0u);
}
