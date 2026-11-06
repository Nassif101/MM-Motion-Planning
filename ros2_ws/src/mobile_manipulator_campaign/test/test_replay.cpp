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
// The Unity-calibrated planar profile with its calibrated base and delays neutralized: these tests check the replay's
// mechanics.
mmp::RobotProfile profile()
{
  auto p = mmp::load_profile(share("mobile_manipulator_plant") + "/robots/mobile_manipulator/profile_unity.yaml", share);
  p.skid_share = {0.0};
  p.skid_v = {1.0};
  p.skid_w = {1.0};
  p.skid_w_in_place = 1.0;
  p.base_lag_s.setZero();
  p.base_command_delay_s = 0.0;
  p.arm_command_delay_s = 0.0;
  p.base_state_delay_s = 0.0;
  p.arm_state_delay_s = 0.0;
  p.base_slip.setOnes();
  p.kp << 2000.0, 6000.0, 3500.0, 500.0, 650.0, 300.0;  // Unity's drives
  p.kd << 240.0, 360.0, 180.0, 40.0, 35.0, 28.0;
  return p;
}
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
  in["params"] = {{"base_command_delay_s", 0.1}, {"kp.0", 1000.0}, {"base_slip", {0.9, 1.0}}};
  const json out = mmc::replay(in, p, site());
  EXPECT_DOUBLE_EQ(out.at("params").at("base_command_delay_s").get<double>(), 0.1);
  EXPECT_DOUBLE_EQ(out.at("params").at("kp").at(0).get<double>(), 1000.0);
  EXPECT_NEAR(out.at("twist").back().at(0).get<double>(), 0.27, 0.01);
  // with the delay the base starts later: at 0.1 s it has not moved
  EXPECT_DOUBLE_EQ(out.at("base").at(5).at(0).get<double>(), -5.0);
}

TEST(Replay, RejectsUnknownOrInvalidParams)
{
  auto p = profile();
  EXPECT_THROW(mmc::apply_overrides(p, json{{"kq", 1.0}}), std::invalid_argument);
  EXPECT_THROW(mmc::apply_overrides(p, json{{"kp.2abc", 1.0}}), std::invalid_argument);
  EXPECT_THROW(mmc::apply_overrides(p, json{{"gravity_ff.1", true}}), std::invalid_argument);
  EXPECT_THROW(mmc::apply_overrides(p, json{{"kp.0", 0.0}}), std::invalid_argument);  // the servo divides by kp
  EXPECT_THROW(mmc::apply_overrides(p, json{{"arm_command_delay_s", -0.01}}), std::invalid_argument);
}

TEST(Replay, AGapInTheCommandsLetsTheWatchdogStop)
{
  // Unity's watchdog: no base command for more than 0.5 s brakes the base, as a ROS-TCP stall in a recording would
  const auto p = profile();
  json in = input(p);
  in["base_commands"] = json::array({{0.0, 0.3, 0.0}});  // one command, then nothing
  const json out = mmc::replay(in, p, site());
  EXPECT_NEAR(out.at("twist").back().at(0).get<double>(), 0.0, 0.01);
}

TEST(Replay, UnstampedStreamsStartOnReceiptAfterTheArmDelay)
{
  // a stream without "start" (the node's stamp 0) starts when it arrives; the JTC's output reaches the drives after the
  // arm delay, which shifts the whole motion once
  double moved_at[2];
  for (const double delay : {0.0, 0.1}) {
    const auto p = profile();
    json in = input(p);
    in["arm_streams"][0].erase("start");
    in["params"] = {{"arm_command_delay_s", delay}};
    in["sample_period_s"] = 0.002;
    const json out = mmc::replay(in, p, site());
    const double q0 = out.at("q").at(0).at(0).get<double>();
    moved_at[delay > 0.0] = -1.0;
    for (size_t k = 0; k < out.at("t").size(); ++k)
      if (out.at("q").at(k).at(0).get<double>() > q0 + 0.01) {
        moved_at[delay > 0.0] = out.at("t").at(k).get<double>();
        break;
      }
  }
  EXPECT_NEAR(moved_at[1] - moved_at[0], 0.1, 0.006);
}

TEST(Replay, TheGroundWorldHasNoSiteBoxes)
{
  // driving into the site's ColumnE (centre x 18.8, 0.4 m wide): the site stops the base, the ground world does not
  const auto p = profile();
  double reached[2];
  for (const std::string world : {"site", "ground"}) {
    json in = input(p);
    in["start"]["base"] = {17.6, 11.25, 0.0};
    in["duration_s"] = 6.0;
    in["world"] = world;
    in["base_commands"] = json::array();
    for (int k = 0; k <= 300; ++k) in["base_commands"].push_back({0.02 * k, 0.3, 0.0});
    const json out = mmc::replay(in, p, site());
    reached[world == "ground"] = out.at("base").back().at(0).get<double>();
    ASSERT_EQ(out.at("q_ref").back().size(), 6u);
  }
  EXPECT_LT(reached[0], 18.8 - 0.2 - 0.4);  // the wheels (x + 0.44) at the column's face
  EXPECT_GT(reached[1], 18.8);
}

TEST(Replay, TakesASkidMap)
{
  const auto p = profile();
  json in = input(p);
  in["params"] = {{"base_skid", {{"share", {0.0, 1.0}}, {"v", {0.5, 0.5}}, {"w", {1.0, 1.0}}, {"w_in_place", 1.0}}}};
  const json out = mmc::replay(in, p, site());
  EXPECT_NEAR(out.at("twist").back().at(0).get<double>(), 0.15, 0.01);  // straight at half the commanded 0.3 m/s
  EXPECT_EQ(out.at("params").at("base_skid").at("v").at(0).get<double>(), 0.5);
  json bad = in;
  bad["params"]["base_skid"]["share"] = {1.0, 0.0};
  EXPECT_THROW(mmc::replay(bad, p, site()), std::invalid_argument);
  json negative = in;
  negative["params"]["base_skid"]["v"] = {0.5, -0.5};
  EXPECT_THROW(mmc::replay(negative, p, site()), std::invalid_argument);
  json malformed = in;
  malformed["params"]["base_skid"] = {{"share", "x"}};
  EXPECT_THROW(mmc::replay(malformed, p, site()), std::invalid_argument);
}

TEST(Replay, OverridesTheWheelsBaseAndReportsResolvedGains)
{
  // the URDF-based default (amendment 2026-11-06): its wheels base and gain rule take overrides for commissioning
  const auto p = mmp::load_profile(share("mobile_manipulator_plant") + "/robots/mobile_manipulator/profile.yaml", share);
  json in = input(p);
  in["world"] = "ground";
  in["duration_s"] = 0.2;
  in["params"] = {{"effective_track_m", 2.0}, {"wheel_speed_max", 10.0}, {"wheel_kv", 500.0}, {"contact_mu", 0.5},
                  {"contact_impratio", 5.0}, {"gain_bandwidth_hz", 8.0}};
  const json out = mmc::replay(in, p, site());
  const json & params = out.at("params");
  EXPECT_DOUBLE_EQ(params.at("effective_track_m").get<double>(), 2.0);
  EXPECT_DOUBLE_EQ(params.at("wheel_speed_max").get<double>(), 10.0);
  EXPECT_DOUBLE_EQ(params.at("wheel_kv").get<double>(), 500.0);
  EXPECT_DOUBLE_EQ(params.at("contact_mu").get<double>(), 0.5);
  EXPECT_DOUBLE_EQ(params.at("contact_impratio").get<double>(), 5.0);
  EXPECT_DOUBLE_EQ(params.at("gain_bandwidth_hz").get<double>(), 8.0);
  EXPECT_EQ(params.at("base_model").get<std::string>(), "wheels");
  EXPECT_GT(params.at("kp").at(1).get<double>(), 100.0);  // resolved by the rule, not the unit placeholders
}

TEST(Replay, RejectsExplicitGainsUnderAGainRule)
{
  const auto p = mmp::load_profile(share("mobile_manipulator_plant") + "/robots/mobile_manipulator/profile.yaml", share);
  json in = input(p);
  in["params"] = {{"kp.0", 3000.0}};
  EXPECT_THROW(mmc::replay(in, p, site()), std::invalid_argument);
  in["params"] = {{"wheel_kv", 0.0}};
  EXPECT_THROW(mmc::replay(in, p, site()), std::invalid_argument);
}
