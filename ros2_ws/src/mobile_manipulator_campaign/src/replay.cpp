#include "mobile_manipulator_campaign/replay.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

#include "mobile_manipulator_plant/model_builder.hpp"
#include "mobile_manipulator_plant/plant.hpp"

namespace mobile_manipulator_campaign
{
namespace mmp = mobile_manipulator_plant;
using nlohmann::json;

namespace
{
mmp::Vec6 vec6(const json & j)
{
  if (!j.is_array() || j.size() != 6) throw std::invalid_argument("replay: expected six numbers, got " + j.dump());
  mmp::Vec6 v;
  for (int i = 0; i < 6; ++i) v(i) = j.at(i).get<double>();
  return v;
}

std::vector<double> list(const mmp::Vec6 & v) { return {v.data(), v.data() + 6}; }

// A named field of the profile: its storage and length (1, 2 or 6 numbers, or a flag).
struct Field
{
  double * values = nullptr;
  int size = 0;
  bool * flag = nullptr;
};

std::map<std::string, Field> fields(mmp::RobotProfile & p)
{
  return {{"base_command_delay_s", {&p.base_command_delay_s, 1}},
          {"arm_command_delay_s", {&p.arm_command_delay_s, 1}},
          {"base_state_delay_s", {&p.base_state_delay_s, 1}},
          {"arm_state_delay_s", {&p.arm_state_delay_s, 1}},
          {"jtc_rate_hz", {&p.jtc_rate_hz, 1}},
          {"ff_accel_limit", {p.ff_accel_limit.data(), 6}},
          {"base_lag_s", {p.base_lag_s.data(), 2}},
          {"base_slip", {p.base_slip.data(), 2}},
          {"kp", {p.kp.data(), 6}},
          {"kd", {p.kd.data(), 6}},
          {"armature", {p.armature.data(), 6}},
          {"damping", {p.damping.data(), 6}},
          {"frictionloss", {p.frictionloss.data(), 6}},
          {"base_breakaway", {nullptr, 0, &p.base_breakaway}},
          {"gravity_ff", {nullptr, 0, &p.gravity_ff}},
          {"motion_ff", {nullptr, 0, &p.motion_ff}}};
}
}  // namespace

void apply_overrides(mmp::RobotProfile & profile, const json & params)
{
  auto known = fields(profile);
  for (const auto & [key, value] : params.items()) {
    if (key == "base_skid") {  // the skid map: {"share": [..], "v": [..], "w": [..], "w_in_place": x}
      try {
        profile.skid_share = value.at("share").get<std::vector<double>>();
        profile.skid_v = value.at("v").get<std::vector<double>>();
        profile.skid_w = value.at("w").get<std::vector<double>>();
        profile.skid_w_in_place = value.value("w_in_place", 1.0);
      } catch (const nlohmann::json::exception & e) {
        throw std::invalid_argument(std::string("replay: base_skid: ") + e.what());
      }
      continue;
    }
    const auto dot = key.find('.');
    const std::string name = key.substr(0, dot);
    const auto it = known.find(name);
    if (it == known.end()) throw std::invalid_argument("replay: unknown parameter " + key);
    Field & f = it->second;
    if (f.flag) {
      if (dot != std::string::npos) throw std::invalid_argument("replay: " + name + " is a flag, not a list: " + key);
      *f.flag = value.get<bool>();
    } else if (dot != std::string::npos) {
      const std::string index = key.substr(dot + 1);
      if (index.empty() || index.find_first_not_of("0123456789") != std::string::npos)
        throw std::invalid_argument("replay: bad index in " + key);
      const int i = std::stoi(index);
      if (i < 0 || i >= f.size) throw std::invalid_argument("replay: index out of range in " + key);
      f.values[i] = value.get<double>();
    } else if (f.size == 1) {
      f.values[0] = value.get<double>();
    } else {
      if (!value.is_array() || static_cast<int>(value.size()) != f.size)
        throw std::invalid_argument("replay: " + key + " needs " + std::to_string(f.size) + " numbers");
      for (int i = 0; i < f.size; ++i) f.values[i] = value.at(i).get<double>();
    }
  }
  // what the servo and the queues need (the profile loader's own checks)
  auto bad = [](const std::string & what) { throw std::invalid_argument("replay: " + what); };
  if (!(profile.kp.array() > 0.0).all()) bad("kp must be positive");
  if (!(profile.kd.array() >= 0.0).all()) bad("kd must not be negative");
  if (!(profile.armature.array() >= 0.0).all() || !(profile.damping.array() >= 0.0).all() || !(profile.frictionloss.array() >= 0.0).all())
    bad("armature, damping and frictionloss must not be negative");
  if (profile.base_command_delay_s < 0.0 || profile.arm_command_delay_s < 0.0 || profile.base_state_delay_s < 0.0 ||
      profile.arm_state_delay_s < 0.0)
    bad("delays must not be negative");
  if (!(profile.base_lag_s.array() >= 0.0).all()) bad("base_lag_s must not be negative");
  if (!(profile.jtc_rate_hz > 0.0)) bad("jtc_rate_hz must be positive");
  if (!(profile.ff_accel_limit.array() >= 0.0).all()) bad("ff_accel_limit must not be negative");
  if (profile.skid_share.empty() || profile.skid_v.size() != profile.skid_share.size() || profile.skid_w.size() != profile.skid_share.size())
    bad("base_skid: share, v and w need equal, non-zero lengths");
  for (size_t i = 1; i < profile.skid_share.size(); ++i)
    if (!(profile.skid_share[i] > profile.skid_share[i - 1])) bad("base_skid: share knots must ascend");
  auto positive = [](double x) { return std::isfinite(x) && x > 0.0; };
  if (!(profile.skid_share.front() >= 0.0) || !std::all_of(profile.skid_v.begin(), profile.skid_v.end(), positive) ||
      !std::all_of(profile.skid_w.begin(), profile.skid_w.end(), positive) || !positive(profile.skid_w_in_place))
    bad("base_skid: shares non-negative, factors finite and positive");
}

json replay(const json & input, mmp::RobotProfile profile, const std::string & site_mjcf)
{
  if (input.contains("params")) apply_overrides(profile, input.at("params"));
  const auto & start = input.at("start");
  const mmp::Pose2 base{start.at("base").at(0).get<double>(), start.at("base").at(1).get<double>(), start.at("base").at(2).get<double>()};
  const double duration = input.at("duration_s").get<double>();
  const double period = input.value("sample_period_s", 0.02);
  const long every = std::lround(period / mmp::kPhysicsStepS);
  if (every < 1 || std::abs(every * mmp::kPhysicsStepS - period) > 1e-9)
    throw std::invalid_argument("replay: sample_period_s must be a multiple of 2 ms");

  // the recorded arm streams, by receipt time; one without "start" (the node's stamp 0) starts on receipt, after the
  // arm's delay, as the closed loop's do
  std::vector<std::pair<double, mmp::ArmReference>> streams;
  for (const auto & s : input.value("arm_streams", json::array())) {
    mmp::ArmReference r;
    const double receipt = s.at("t").get<double>();
    r.start_time = s.contains("start") && !s.at("start").is_null() ? s.at("start").get<double>() : receipt;  // the JTC samples on receipt
    for (const auto & p : s.at("points")) r.points.push_back({p.at("t").get<double>(), vec6(p.at("q")), vec6(p.at("v"))});
    streams.emplace_back(receipt, std::move(r));
  }
  std::stable_sort(streams.begin(), streams.end(), [](const auto & a, const auto & b) { return a.first < b.first; });
  const json & commands = input.value("base_commands", json::array());

  // "world": "site" (the V2 site, default) or "ground" (the ground alone, beside the site model: calibration replays)
  const std::string world = input.value("world", std::string("site"));
  std::string model = site_mjcf;
  if (world == "ground") model = site_mjcf.substr(0, site_mjcf.find_last_of('/') + 1) + "ground.xml";
  else if (world != "site") throw std::invalid_argument("replay: world must be site or ground");
  mmp::Plant plant(profile, model, mmp::PlantMode::Realistic);
  plant.reset({}, base, vec6(start.at("q")), input.value("seed", 1));
  json out = {{"t", json::array()},  {"base", json::array()}, {"twist", json::array()},
              {"q", json::array()},  {"qd", json::array()},   {"q_ref", json::array()}};
  auto record = [&](const mmp::PlantState & s) {
    out["t"].push_back(s.t);
    out["base"].push_back({s.base.x, s.base.y, s.base.yaw});
    out["twist"].push_back({s.v, s.omega});
    out["q"].push_back(list(s.q));
    out["qd"].push_back(list(s.qd));
    out["q_ref"].push_back(list(s.q_ref));
  };
  record(plant.state());
  size_t next_command = 0, next_stream = 0;
  mmp::PlantCommand c;
  bool changed = true;
  const long steps = std::lround(duration / mmp::kPhysicsStepS);
  for (long k = 0; k < steps; ++k) {
    const double t = k * mmp::kPhysicsStepS;
    while (next_command < commands.size() && commands[next_command].at(0).get<double>() <= t + 1e-9) {
      c.v = commands[next_command].at(1).get<double>();
      c.omega = commands[next_command].at(2).get<double>();
      ++next_command;
      changed = true;
    }
    while (next_stream < streams.size() && streams[next_stream].first <= t + 1e-9) {
      c.arm = streams[next_stream].second;
      ++next_stream;
      changed = true;
    }
    if (changed) c.stamp = t;  // a new command arrives now (the plant's command delay follows)
    changed = false;
    plant.step(c, mmp::kPhysicsStepS);
    if ((k + 1) % every == 0) record(plant.state());
  }
  auto & p = out["params"];
  p["base_command_delay_s"] = profile.base_command_delay_s;
  p["arm_command_delay_s"] = profile.arm_command_delay_s;
  p["base_state_delay_s"] = profile.base_state_delay_s;
  p["arm_state_delay_s"] = profile.arm_state_delay_s;
  p["base_lag_s"] = {profile.base_lag_s(0), profile.base_lag_s(1)};
  p["base_slip"] = {profile.base_slip(0), profile.base_slip(1)};
  p["kp"] = list(profile.kp);
  p["kd"] = list(profile.kd);
  p["jtc_rate_hz"] = profile.jtc_rate_hz;
  p["ff_accel_limit"] = list(profile.ff_accel_limit);
  p["armature"] = list(profile.armature);
  p["damping"] = list(profile.damping);
  p["frictionloss"] = list(profile.frictionloss);
  p["base_breakaway"] = profile.base_breakaway;
  p["base_skid"] = {{"share", profile.skid_share}, {"v", profile.skid_v}, {"w", profile.skid_w}, {"w_in_place", profile.skid_w_in_place}};
  p["gravity_ff"] = profile.gravity_ff;
  p["motion_ff"] = profile.motion_ff;
  return out;
}
}  // namespace mobile_manipulator_campaign
