#include "mobile_manipulator_campaign/session.hpp"

#include <openssl/evp.h>

#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>

#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

#include "mobile_manipulator_campaign/scenario_setup.hpp"
#include "mobile_manipulator_mpc/replay.hpp"
#include "mobile_manipulator_mpc/replay_bag.hpp"
#include "mobile_manipulator_plant/frames.hpp"

namespace mobile_manipulator_campaign
{
namespace mmd = mobile_manipulator_medium;
namespace mmm = mobile_manipulator_mpc;
namespace mmp = mobile_manipulator_plant;

namespace
{
// SHA-256 of a file's bytes, lowercase hex ("" when it cannot be read): the Hello's provenance (spec Section 6.4)
std::string sha256_file(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) return "";
  const std::string bytes(std::istreambuf_iterator<char>(in), {});
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int n = 0;
  EVP_Digest(bytes.data(), bytes.size(), digest, &n, EVP_sha256(), nullptr);
  std::ostringstream hex;
  for (unsigned int i = 0; i < n; ++i) hex << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(digest[i]);
  return hex.str();
}
}  // namespace
using json = nlohmann::json;

namespace
{
constexpr double kCycleS = 0.02;  // B5's period (50 Hz)
constexpr double kPerturbation = 1e-3;  // the seed's start-pose perturbation: +-1 mm, +-1 mrad (CP1)

double now_s() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

std::vector<float> floats(const Eigen::VectorXd & v) { return std::vector<float>(v.data(), v.data() + v.size()); }

json finite_or_null(double v) { return std::isfinite(v) ? json(v) : json(nullptr); }

std::string text_of(const mjModel * m, const char * name)
{
  const int id = mj_name2id(m, mjOBJ_TEXT, name);
  return id < 0 ? "" : std::string(m->text_data + m->text_adr[id]);
}

std::string hostname()
{
  char name[256] = "";
  gethostname(name, sizeof(name) - 1);
  return name;
}
}  // namespace

Session::Session(SessionConfig config) : config_(std::move(config))
{
  const std::string share = mmd::package_share("mobile_manipulator_plant");
  profile_path_ = config_.profile.empty() ? share + "/robots/mobile_manipulator/profile.yaml" : config_.profile;
  site_mjcf_ = share + "/models/site_v2.xml";
  if (config_.scan_decimation < 1) throw std::invalid_argument("Session: scan_decimation must be at least 1");
}

Session::~Session() = default;

std::vector<std::string> Session::list_scenarios()
{
  const YAML::Node root = YAML::LoadFile(mmd::package_share("mobile_manipulator_navigation") + "/config/scenarios_site_v2.yaml");
  std::vector<std::string> names;
  for (const auto & item : root["scenarios"]) names.push_back(item.first.as<std::string>());
  return names;
}

void Session::reset(const std::string & scenario, const std::string & bag, std::optional<mmp::Pose2> start, uint64_t seed)
{
  stepper_.reset();
  log_.reset();
  frames_.clear();
  events_.clear();
  records_.clear();
  scans_ = 0;
  last_contacts_ = 0;
  last_fallback_.clear();
  wall_s_ = 0.0;
  scenario_ = scenario;
  bag_ = bag;
  seed_ = seed;

  case_ = make_case(scenario, bag, config_.medium_config);
  goals_.clear();
  if (!bag.empty()) {  // every recorded goal in order (make_case took the first one's route and start)
    std::array<std::string, 6> joints;
    for (int i = 0; i < 6; ++i) joints[i] = mmm::kArmJoints[i];
    for (const auto & g : mmm::read_recording(bag, joints).goals)
      if (g.points.size() >= 2) goals_.push_back({mmm::Route(g.points), g.goal});
  } else {
    goals_.push_back({case_.route, case_.goal});
  }
  start_ = case_.start;
  if (start) start_.head<3>() << start->x, start->y, start->yaw;
  perturbation_.setZero();
  if (config_.perturb_start) {
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> u(-kPerturbation, kPerturbation);
    for (int i = 0; i < 3; ++i) perturbation_(i) = u(rng);
    start_.head<3>() += perturbation_;
  }

  const auto & bench = *case_.bench;
  const auto profile = mmp::load_profile(profile_path_, [](const std::string & p) { return mmd::package_share(p); });
  plant_ = std::make_unique<MujocoPlant>(profile, site_mjcf_, config_.mode);
  plant_->configure_arm_path(bench.limits(), case_.b5_config, bench.primitives());
  const mmp::ScenarioSetup setup = scenario_setup(bench.scene(), scenario);
  plant_->reset(setup, start_, seed);
  if (config_.lidar) plant_->plant().enable_lidar();
  b5_ = mmm::build_controller(case_.b5_config, bench.limits(), bench.urdf(), bench.payload(), 50.0, false);
  medium_ = std::make_unique<mmd::MediumPlanner>(bench.planner());
  medium_->set_iteration_budget(config_.iteration_budget);
  if (config_.iteration_budget > 0) medium_->set_solve_cap_s(10.0);  // a budget alone cuts the cycles
  posture_ = mmm::posture_targets(bench.payload(), case_.b5_config.comfort_pose, case_.b5_config.posture_mode,
                                  case_.b5_config.tight_poses);

  json boxes = json::array(), movers = json::array();  // what the viewer draws beyond the site
  for (const auto & b : setup.boxes)
    boxes.push_back({{"name", b.name}, {"center", {b.center.x(), b.center.y(), b.center.z()}},
                     {"size", {b.size.x(), b.size.y(), b.size.z()}}, {"yaw", b.yaw}});
  for (const auto & m : setup.movers) movers.push_back({{"name", m.name}, {"size", {m.size.x(), m.size.y(), m.size.z()}}});
  json goals = json::array();
  for (const auto & g : goals_) goals.push_back({g.goal[0], g.goal[1], g.goal[2]});
  const json hello = {{"schema", mmp::kSchemaVersion},
                      {"scenario", scenario},
                      {"bag", bag},
                      {"seed", seed},
                      {"mode", config_.mode == mmp::PlantMode::Realistic ? "realistic" : "ideal"},
                      {"medium", config_.medium},
                      {"profile", profile_path_},
                      {"site_checksum", text_of(plant_->plant().model(), "site_checksum")},
                      {"start", {start_(0), start_(1), start_(2)}},
                      {"goals", goals},
                      {"boxes", boxes},
                      {"movers", movers},
                      {"provenance", config_.provenance},
                      {"profile_sha256", sha256_file(profile_path_)},
                      {"robot_model_sha256", sha256_file(profile.base_model == mmp::BaseModel::Wheels ? profile.model_wheels_path
                                                                                                       : profile.model_path)},
                      {"site_model_sha256", sha256_file(site_mjcf_)},
                      {"config", {{"medium", config_.medium},
                                  {"medium_config", config_.medium_config},
                                  {"lidar", config_.lidar},
                                  {"scan_decimation", config_.scan_decimation},
                                  {"timeout_s", config_.timeout_s},
                                  {"iteration_budget", config_.iteration_budget},
                                  {"perturb_start", config_.perturb_start}}}};
  if (!config_.log_path.empty()) log_ = std::make_unique<mmp::RunLogWriter>(config_.log_path, hello);  // writes the Hello
  if (config_.queue_frames) frames_.push_back(mmp::encode(mmp::FrameType::Hello, hello, {}));
  goal_index_ = 0;
  start_next_goal();
}

void Session::set_goal(const mobile_manipulator_navigation::Pose2 & goal)
{
  if (!plant_) throw std::logic_error("Session: set_goal before reset");
  const Eigen::Vector2d from = plant_->plant().true_state().base.x == 0.0 && plant_->plant().true_state().base.y == 0.0
                                 ? Eigen::Vector2d(start_(0), start_(1))
                                 : Eigen::Vector2d(plant_->plant().true_state().base.x, plant_->plant().true_state().base.y);
  set_route(mmm::Route({from, Eigen::Vector2d(goal[0], goal[1])}), goal);
}

nlohmann::json git_provenance()
{
  auto run = [](const char * cmd) {
    std::string out;
    if (FILE * f = popen(cmd, "r")) {
      char buf[256];
      while (fgets(buf, sizeof buf, f)) out += buf;
      pclose(f);
    }
    return out;
  };
  std::string commit = run("git rev-parse HEAD 2>/dev/null");
  while (!commit.empty() && (commit.back() == '\n' || commit.back() == '\r')) commit.pop_back();
  if (commit.empty()) return nlohmann::json::object();
  return {{"git_commit", commit}, {"git_dirty", !run("git status --porcelain --untracked-files=no 2>/dev/null").empty()}};
}

void Session::set_route(const mmm::Route & route, const mobile_manipulator_navigation::Pose2 & goal)
{
  if (!plant_) throw std::logic_error("Session: set_route before reset");
  goals_ = {{route, goal}};
  goal_index_ = 0;
  start_next_goal();
}

void Session::start_next_goal()
{
  stepper_.reset();
  if (goal_index_ >= goals_.size()) return;
  const Goal & g = goals_[goal_index_];
  const auto & bench = *case_.bench;
  mmd::ClosedLoopOptions co;
  co.timeout_s = config_.timeout_s;
  co.medium = config_.medium;
  if (config_.iteration_budget > 0) co.fixed_latency_s = bench.config().solve_cap_s;
  co.start_time_s = std::round(time() / kCycleS) * kCycleS;  // the plant's clock: its commands are stamped with it
  co.on_plan = [this](double t, const mmm::State &, const mmd::PlanOutput & out) {
    std::vector<float> samples;
    for (const auto & s : out.samples) {
      samples.insert(samples.end(), {static_cast<float>(s.x), static_cast<float>(s.y), static_cast<float>(s.yaw)});
      for (int j = 0; j < 6; ++j) samples.push_back(static_cast<float>(s.q(j)));
    }
    emit(mmp::FrameType::Plans,
         {{"t", t}, {"plan_id", out.plan_id}, {"route_id", out.route_id}, {"status", out.status}, {"homotopy", out.homotopy},
          {"capped", out.capped}, {"layout", "x y yaw q0..q5"}},  // no solve_s: wall clock (E2; the summary has it)
         {samples});
  };
  goal_wall_start_ = wall_s_;
  stepper_ = std::make_unique<mmd::ClosedLoopStepper>(*b5_, *medium_, case_.b5_config, posture_, g.route, g.goal, *plant_,
                                                      bench.world(), bench.primitives(), co);
}

bool Session::step_cycle()
{
  if (!stepper_) return false;
  const double wall0 = now_s();
  const bool stepped = stepper_->step();
  wall_s_ += now_s() - wall0;
  if (stepped) {  // the plant moved one cycle
    const mmp::PlantState & truth = plant_->plant().true_state();
    Eigen::Matrix<double, 9, 1> pose;
    pose << truth.base.x, truth.base.y, truth.base.yaw, truth.q;
    const int contacts = plant_->contacts();
    emit(mmp::FrameType::State,
         {{"t", truth.t}, {"goal", goal_index_}, {"status", mmm::to_string(stepper_->last_status())},
          {"fallback", stepper_->last_fallback()}, {"contacts", contacts}, {"movers", mover_xy()},
          {"layout", "true x y yaw q; b5 state; executed"}},
         {floats(pose), floats(plant_->state()), floats(plant_->executed())});
    if (contacts > 0 && last_contacts_ == 0) {
      const auto & c = plant_->truth().contacts.front();
      event("contact", {{"robot_geom", c.robot_geom}, {"other_geom", c.other_geom}, {"depth", c.depth}});
    }
    last_contacts_ = contacts;
    const std::string & fallback = stepper_->last_fallback();
    if (!fallback.empty() && fallback != last_fallback_) event("fallback", {{"reason", fallback}});
    last_fallback_ = fallback;
    while (auto scan = plant_->plant().scan()) {
      if (scans_++ % config_.scan_decimation != 0) continue;
      std::vector<float> points, classes;
      for (size_t i = 0; i < scan->points.size(); ++i) {
        points.insert(points.end(), {scan->points[i].x(), scan->points[i].y(), scan->points[i].z()});
        classes.push_back(static_cast<float>(scan->cls[i]));
      }
      emit(mmp::FrameType::Scan,
           {{"scan", scan->scan}, {"t_start", scan->t_start}, {"t_end", scan->t_end}, {"rays", scan->rays},
            {"layout", "points xyz in the lidar frame at each ray's time; classes"}},
           {points, classes});
    }
    return true;
  }
  finish_goal();
  return active();
}

void Session::finish_goal()
{
  const auto & r = stepper_->result();
  const Goal & g = goals_[goal_index_];
  records_.push_back(record(r, g.goal, wall_s_ - goal_wall_start_, stepper_->timed_out()));
  event(r.reached ? "goal_reached" : stepper_->timed_out() ? "timeout" : "failed",
        {{"goal", goal_index_}, {"end_status", r.end_status}, {"time_s", r.time_s}});
  if (r.reached && goal_index_ + 1 < goals_.size()) {
    ++goal_index_;
    start_next_goal();
  } else {
    stepper_.reset();  // the log stays open: a goal set now goes into it too (it closes at the next reset or the end)
  }
}

json Session::mover_xy() const
{
  json out = json::array();
  for (const auto & [name, p] : plant_->plant().mover_positions()) out.push_back({name, p.x(), p.y()});
  return out;
}

double Session::time() const { return plant_ ? plant_->plant().true_state().t : 0.0; }

void Session::move_body(const std::string & name, const Eigen::Vector3d & center, double yaw)
{
  if (!plant_) throw std::logic_error("Session: move_body before reset");
  plant_->plant().move_body(name, center, yaw);
}

std::vector<std::string> Session::take_frames() { return std::exchange(frames_, {}); }

std::vector<json> Session::take_events() { return std::exchange(events_, {}); }

void Session::emit(mmp::FrameType type, const json & header, const std::vector<std::vector<float>> & blocks)
{
  if (!log_ && !config_.queue_frames) return;
  const std::string frame = mmp::encode(type, header, blocks);
  if (log_) log_->write(frame);
  if (config_.queue_frames) frames_.push_back(frame);
}

void Session::event(const std::string & name, json details)
{
  details["event"] = name;
  details["t"] = time();
  events_.push_back(details);
  emit(mmp::FrameType::Event, details);
}

json Session::record(const mmd::ClosedLoopResult & r, const mobile_manipulator_navigation::Pose2 & goal, double wall_s,
                     bool timed_out) const
{
  // medium_bench closed's record (medium_bench.cpp run_closed) without the trace
  json fallbacks = json::object();
  for (const auto & [reason, count] : r.fallbacks) fallbacks[reason] = count;
  return {{"kind", config_.medium ? "f2" : "b5"},
          {"reached", r.reached},
          {"end_status", r.end_status},
          {"timed_out", timed_out},
          {"medium_period_s", 1.0 / case_.bench->config().rate_hz},
          {"time_s", r.time_s},
          {"b5_cycles", r.b5_cycles},
          {"following_cycles", r.following_cycles},
          {"fallbacks", fallbacks},
          {"executed_slack_plans", r.executed_slack_plans},
          {"max_executed_slack_m", r.max_executed_slack_m},
          {"min_clearance_m", finite_or_null(r.min_clearance_m)},
          {"max_torque_ratio", r.max_torque_ratio},
          {"medium_cycles", r.medium_cycles},
          {"medium_capped", r.medium_capped},
          {"medium_infeasible", r.medium_infeasible},
          {"medium_solve_s", r.medium_solve_s},
          {"first_side", r.first_side},
          {"homotopy_switches_after_first", r.homotopy_switches_after_first},
          {"side_sequence", r.side_sequence},
          {"followed_infeasible_cycles", r.followed_infeasible_cycles},
          {"followed_infeasible_plans", r.followed_infeasible_plans},
          {"final", {r.final_state(0), r.final_state(1), r.final_state(2)}},
          {"goal", {goal[0], goal[1], goal[2]}},
          {"wall_s", wall_s},
          {"plant_min_clearance_m", finite_or_null(r.plant_min_clearance_m)},
          {"plant_contacts", r.plant_contacts}};
}

json Session::summary() const
{
  // the goals' records, the run's as a whole: one goal's record for a single goal; a mission's sums, extremes and the
  // last goal's status
  std::vector<json> goals = records_;
  if (stepper_)
    goals.push_back(record(stepper_->result(), goals_[goal_index_].goal, wall_s_ - goal_wall_start_, stepper_->timed_out()));
  json s = goals.empty() ? json::object() : goals.front();
  if (!goals.empty()) {
    s["reached"] = std::all_of(goals.begin(), goals.end(), [](const json & g) { return g.at("reached").get<bool>(); }) &&
                   goals.size() == goals_.size() && !stepper_;
    s["end_status"] = goals.back().at("end_status");
    s["timed_out"] = goals.back().at("timed_out");
    s["goal"] = goals.back().at("goal");
    s["final"] = goals.back().at("final");
    for (const char * key : {"b5_cycles", "following_cycles", "executed_slack_plans", "medium_cycles", "medium_capped",
                             "medium_infeasible", "homotopy_switches_after_first", "followed_infeasible_cycles",
                             "followed_infeasible_plans", "plant_contacts"}) {
      int total = 0;
      for (const auto & g : goals) total += g.at(key).get<int>();
      s[key] = total;
    }
    double min_clearance = std::numeric_limits<double>::infinity(), plant_min = min_clearance, slack = 0.0, torque = 0.0;
    json fallbacks = json::object(), solve = json::array(), sides = json::array();
    for (const auto & g : goals) {
      if (!g.at("min_clearance_m").is_null()) min_clearance = std::min(min_clearance, g.at("min_clearance_m").get<double>());
      if (!g.at("plant_min_clearance_m").is_null()) plant_min = std::min(plant_min, g.at("plant_min_clearance_m").get<double>());
      slack = std::max(slack, g.at("max_executed_slack_m").get<double>());
      torque = std::max(torque, g.at("max_torque_ratio").get<double>());
      for (const auto & [reason, count] : g.at("fallbacks").items()) fallbacks[reason] = fallbacks.value(reason, 0) + count.get<int>();
      for (const auto & x : g.at("medium_solve_s")) solve.push_back(x);
      for (const auto & x : g.at("side_sequence")) sides.push_back(x);
    }
    s["min_clearance_m"] = finite_or_null(min_clearance);
    s["plant_min_clearance_m"] = finite_or_null(plant_min);
    s["max_executed_slack_m"] = slack;
    s["max_torque_ratio"] = torque;
    s["fallbacks"] = fallbacks;
    s["medium_solve_s"] = solve;
    s["side_sequence"] = sides;
  }
  s["time_s"] = time();
  s["wall_s"] = wall_s_;
  s["cell"] = scenario_;
  s["bag"] = bag_;
  s["seed"] = seed_;
  s["plant_profile"] = config_.mode == mmp::PlantMode::Realistic ? "mujoco-realistic" : "mujoco-ideal";
  s["profile"] = profile_path_;
  s["lidar"] = config_.lidar;
  s["host"] = hostname();
  s["site_checksum"] = plant_ ? text_of(plant_->plant().model(), "site_checksum") : "";
  s["start_perturbation"] = {perturbation_(0), perturbation_(1), perturbation_(2)};
  s["real_time_factor"] = wall_s_ > 0.0 ? time() / wall_s_ : 0.0;
  s["goals"] = goals;
  return s;
}
}  // namespace mobile_manipulator_campaign
