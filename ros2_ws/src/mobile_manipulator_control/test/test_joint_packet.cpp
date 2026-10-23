#include <gtest/gtest.h>
#include "mobile_manipulator_control/joint_packet.hpp"
using mobile_manipulator_control::packet_mapping;
TEST(Packet, ResolvesNamesInsteadOfOrder) {
  EXPECT_EQ(packet_mapping({"a","b"},{"b","a"},{2,1},{0,0}), (std::vector<size_t>{1,0}));
}
TEST(Packet, RejectsMalformedAndNonfinite) {
  EXPECT_THROW(packet_mapping({"a","b"},{"a","a"},{1,2},{0,0}),std::invalid_argument);
  EXPECT_THROW(packet_mapping({"a","b"},{"a","c"},{1,2},{0,0}),std::invalid_argument);
  EXPECT_THROW(packet_mapping({"a"},{"a"},{NAN},{0}),std::invalid_argument);
  EXPECT_THROW(packet_mapping({"a"},{"a"},{1},{}),std::invalid_argument);
}
using mobile_manipulator_control::invalid_command_joint;
TEST(Command, AcceptsATargetWithinLimits) {
  EXPECT_EQ(invalid_command_joint({0.5,0},{1,-1},{3,-3},{-1,-1},{1,1},{1.5,1.5}), -1);
}
TEST(Command, NamesTheFirstJointOutOfLimitsOrNonfinite) {
  EXPECT_EQ(invalid_command_joint({0.5,1.2},{0,0},{0,0},{-1,-1},{1,1},{1,1}), 1);
  EXPECT_EQ(invalid_command_joint({0,0},{0,1.2},{0,0},{-1,-1},{1,1},{1,1}), 1);
  EXPECT_EQ(invalid_command_joint({0,0},{0,0},{NAN,0},{-1,-1},{1,1},{1,1}), 0);
  EXPECT_EQ(invalid_command_joint({0,0},{0,0},{0,INFINITY},{-1,-1},{1,1},{1,1}), 1);
}
