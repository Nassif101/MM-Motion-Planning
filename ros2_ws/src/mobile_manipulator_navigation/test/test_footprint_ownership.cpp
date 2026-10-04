#include <gtest/gtest.h>

#include "mobile_manipulator_navigation/mission.hpp"

namespace mmn = mobile_manipulator_navigation;

namespace
{
std::map<std::string, std::vector<std::string>> all(const std::vector<std::string> & nodes)
{
  std::map<std::string, std::vector<std::string>> publishers;
  for (const auto & topic : mmn::kFootprintTopics) publishers[topic] = nodes;
  return publishers;
}

bool mentions(const std::vector<std::string> & problems, const std::string & text)
{
  for (const auto & p : problems) if (p.find(text) != std::string::npos) return true;
  return false;
}
}  // namespace

TEST(Ownership, StaticNeedsNoPublisher)
{
  EXPECT_TRUE(mmn::ownership_problems("static", all({})).empty());
  auto publishers = all({});
  publishers["/local_costmap/footprint"] = {"reconfigure_panel_server"};
  const auto problems = mmn::ownership_problems("static", publishers);
  ASSERT_EQ(problems.size(), 1u);
  EXPECT_TRUE(mentions(problems, "/local_costmap/footprint"));
}

TEST(Ownership, ProfilesNeedsTheServerOnly)
{
  EXPECT_TRUE(mmn::ownership_problems("profiles", all({"reconfigure_panel_server"})).empty());
  EXPECT_EQ(mmn::ownership_problems("profiles", all({})).size(), mmn::kFootprintTopics.size());
}

TEST(Ownership, DynamicNeedsTheNodeOnly)
{
  EXPECT_TRUE(mmn::ownership_problems("dynamic", all({"dynamic_footprint_node"})).empty());
  const auto two = mmn::ownership_problems("dynamic", all({"dynamic_footprint_node", "reconfigure_panel_server"}));
  EXPECT_EQ(two.size(), mmn::kFootprintTopics.size());
  EXPECT_TRUE(mentions(two, "reconfigure_panel_server"));
  EXPECT_FALSE(mmn::ownership_problems("dynamic", all({"reconfigure_panel_server"})).empty());
}

TEST(Ownership, UnknownMode)
{
  const auto problems = mmn::ownership_problems("adaptive", all({}));
  ASSERT_EQ(problems.size(), 1u);
  EXPECT_TRUE(mentions(problems, "adaptive"));
}
