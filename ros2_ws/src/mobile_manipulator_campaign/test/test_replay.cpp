// plant_replay's engine (MuJoCo plant plan Task 6): recorded commands replayed open loop through the realistic plant.
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "mobile_manipulator_campaign/replay.hpp"

namespace mmc = mobile_manipulator_campaign;
namespace mmp = mobile_manipulator_plant;
using nlohmann::json;

namespace
{
std::string share(const std::string & package) { return ament_index_cpp::get_package_share_directory(package); }
mmp::RobotProfile profile() { return mmp::load_profile(share("mobile_manipulator_plant") + "/robots/mobile_manipulator/profile.yaml", share); }
std::string site() { return share("mobile_manipulator_plant") + "/models/site_v2.xml"; }

json input(const mmp::RobotProfile & p)
{
  json in;
  std::vector<double> q(p.poses.at("carry").data(), p.poses.at("carry").data() + 6);
  in["start"] = {{"base", {-5.0, -5.5, 0.0}}, {"q", q}};
  in["duration_s"] = 3.0;
  in["sample_period_s"] = 0.02;
  in["base_commands"] = json::array();
  for (int k = 0; k <= 150; ++k) in["base_commands"].push_back({0.02 * k, 0.3, 0.0});
  std::vector<double> q1 = q;
  q1[0] += 0.4;
  in["arm_streams"] = json::array({{{"t", 0.5}, {"start", 0.5},
                                    {"points", {{{"t", 0.0}, {"q", q}, {"v", std::vector<double>(6, 0.0)}},
                                                {{"t", 1.0}, {"q", q1}, {"v", std::vector<double>(6, 0.0)}}}}}});
  return in;
}
}  // namespace

TEST(Replay, ReplaysBaseAndArmCommands)
{
  const auto p = profile();
  const json out = mmc::replay(input(p), p, site());
  ASSERT_EQ(out.at("t").size(), 151u);  // 0 .. 3 s every 0.02 s
  EXPECT_NEAR(out.at("twist").back().at(0).get<double>(), 0.3, 0.01);
  EXPECT_NEAR(out.at("q").back().at(0).get<double>(), p.poses.at("carry")(0) + 0.4, 3e-3);
  EXPECT_NEAR(out.at("base").back().at(0).get<double>(), -5.0 + 0.3 * 3.0 - 0.5 * 0.3 * 0.6, 0.03);  // the 0.6 s ramp
}

TEST(Replay, ParamsOverrideTheProfile)
{
  const auto p = profile();
  json in = input(p);
  in["params"] = {{"command_delay_s", 0.1}, {"kp.0", 1000.0}, {"base_slip", {0.9, 1.0}}};
  const json out = mmc::replay(in, p, site());
  EXPECT_DOUBLE_EQ(out.at("params").at("command_delay_s").get<double>(), 0.1);
  EXPECT_DOUBLE_EQ(out.at("params").at("kp").at(0).get<double>(), 1000.0);
  EXPECT_NEAR(out.at("twist").back().at(0).get<double>(), 0.27, 0.01);
  // with the delay the base starts later: at 0.1 s it has not moved
  EXPECT_DOUBLE_EQ(out.at("base").at(5).at(0).get<double>(), -5.0);
}

TEST(Replay, RejectsUnknownParams)
{
  auto p = profile();
  EXPECT_THROW(mmc::apply_overrides(p, json{{"kq", 1.0}}), std::invalid_argument);
}
