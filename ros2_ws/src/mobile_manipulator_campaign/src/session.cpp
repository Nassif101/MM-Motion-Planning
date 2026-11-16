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
#include "mobile_manipulator_medium/bench_setup.hpp"
#include "mobile_manipulator_mpc/tracking_model.hpp"
#include "mobile_manipulator_mpc/replay.hpp"
#include "mobile_manipulator_mpc/replay_bag.hpp"
#include "mobile_manipulator_plant/frames.hpp"
#include "mobile_manipulator_plant/model_builder.hpp"

namespace mobile_manipulator_campaign
{
namespace mmd = mobile_manipulator_medium;
namespace mmm = mobile_manipulator_mpc;
namespace mmp = mobile_manipulator_plant;

namespace
{
// E1's Plans trace: [name, merit, feasible 0/1, progress_m, merged 0/1] per seed (SeedStats without its timings; merged:
// its solution is an earlier seed's, CP1)
nlohmann::json seed_rows(const std::vector<mmd::SeedStats> & seeds)
{
  nlohmann::json rows = nlohmann::json::array();
  for (const auto & st : seeds) rows.push_back({st.name, st.merit, st.feasible ? 1 : 0, st.progress_m, st.merged ? 1 : 0});
  return rows;
}

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
constexpr double kRowTracePeriodS = 0.1;  // E3 Task 1: one row trace per 0.1 s of the plant's clock
constexpr double kStallBaseM = 0.05, kStallYawRad = 0.1, kStallJointRad = 0.05;  // what counts as progress
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
  // E3 Task 1 (review I1): the row trace's analysis needs the command sent each cycle and the tracking estimate
  // (diag_cmd, diag_track), which the diagnostics carry: a traced run records both
  if (config_.row_trace) config_.diagnostics = true;
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
  // everything that can throw (an unknown scenario, a bag that cannot be read, a scenario outside the V2 site, the
  // profile) is built into locals first: a failed reset leaves the previous run whole (final review)
  ClosedLoopCase c = make_case(scenario, bag, config_.medium_config, config_.b5_config);
  const std::string b5_sha = sha256_file(b5_config_path(config_.b5_config));
  if (config_.tracking_model == "none") {  // E2 Task 7: the run's tracking model and its online correction
    // the unicycle: no correction either (final review C2: an estimator would learn a turn map onto the identity
    // prior); a correction asked for with "none" contradicts it
    if (config_.tracking_adaptation >= 1)
      throw std::invalid_argument("--tracking-model none is the unicycle: it takes no --tracking-adaptation on|gain");
    c.b5_config.tracking_model.clear();
    c.b5_config.tracking = mmm::TrackingModel{};
    c.b5_config.tracking_adaptation = false;
  } else if (!config_.tracking_model.empty()) {
    c.b5_config.tracking = mmm::load_tracking_model(config_.tracking_model);  // throws on a missing or bad file
    c.b5_config.tracking_model = config_.tracking_model;
  } else if (c.b5_config.tracking_model.empty()) {
    // E2 Task 11 (CP4, user 2026-11-14): a plant run whose B5 config names no model predicts with the installed
    // MuJoCo model and corrects its turn gain alone (the ROS node keeps its config's identity until E5)
    c.b5_config.tracking_model = mmd::package_share("mobile_manipulator_mpc") + "/config/tracking_model_mujoco.yaml";
    c.b5_config.tracking = mmm::load_tracking_model(c.b5_config.tracking_model);
    if (config_.tracking_adaptation < 0) {
      c.b5_config.tracking_adaptation = true;
      c.b5_config.tracking_adapt_sideslip = false;
    }
  }
  if (config_.tracking_adaptation >= 0) c.b5_config.tracking_adaptation = config_.tracking_adaptation >= 1;
  if (config_.tracking_adaptation == 2) c.b5_config.tracking_adapt_sideslip = false;  // Task 10c: the turn gain alone
  const std::string tracking_sha =
    c.b5_config.tracking_model.empty() ? std::string() : sha256_file(c.b5_config.tracking_model);
  std::vector<Goal> goals_new;
  if (!bag.empty()) {  // every recorded goal in order (make_case took the first one's route and start)
    std::array<std::string, 6> joints;
    for (int i = 0; i < 6; ++i) joints[i] = mmm::kArmJoints[i];
    for (const auto & g : mission_goals(mmm::read_recording(bag, joints).goals)) goals_new.push_back({mmm::Route(g.points), g.goal});
  } else {
    goals_new.push_back({c.route, c.goal});
  }
  mmm::State start_new = c.start;
  if (start) start_new.head<3>() << start->x, start->y, start->yaw;
  Eigen::Vector3d perturbation = Eigen::Vector3d::Zero();
  if (config_.perturb_start) {
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> u(-kPerturbation, kPerturbation);
    for (int i = 0; i < 3; ++i) perturbation(i) = u(rng);
    start_new.head<3>() += perturbation;
  }
  start_new(2) += config_.start_nudge_rad;  // after the perturbation: a twin keeps the seed's noise stream
  const auto & bench = *c.bench;
  const auto profile = mmp::load_profile(profile_path_, [](const std::string & p) { return mmd::package_share(p); });
  auto plant = std::make_unique<MujocoPlant>(profile, site_mjcf_, config_.mode);
  plant->configure_arm_path(bench.limits(), c.b5_config, bench.primitives());
  const mmp::ScenarioSetup setup = scenario_setup(bench.scene(), scenario);
  double timeout = config_.timeout_s > 0.0 ? config_.timeout_s : 120.0;
  if (config_.timeout_s <= 0.0 && bench.scene().scenario && bench.scene().scenario->contains("timeout_s"))
    timeout = bench.scene().scenario->at("timeout_s").get<double>();
  plant->reset(setup, start_new, seed);
  if (config_.lidar) plant->plant().enable_lidar();
  auto b5 = mmm::build_controller(c.b5_config, bench.limits(), bench.urdf(), bench.payload(), 50.0, config_.diagnostics,
                                 config_.row_trace ? kRowTracePeriodS : 0.0);
  auto medium = std::make_unique<mmd::MediumPlanner>(bench.planner());
  medium->set_iteration_budget(config_.iteration_budget);
  if (config_.iteration_budget > 0) medium->set_solve_cap_s(10.0);  // a budget alone cuts the cycles
  const auto posture = mmm::posture_targets(bench.payload(), c.b5_config.comfort_pose, c.b5_config.posture_mode,
                                            c.b5_config.tight_poses);
  // the model the plant built: the wheels model only in realistic mode on a wheels profile (Plant::reset)
  const std::string model_path = config_.mode == mmp::PlantMode::Realistic && profile.base_model == mmp::BaseModel::Wheels
                                   ? profile.model_wheels_path
                                   : profile.model_path;

  // commit: nothing below throws before the run starts (the stepper goes first: it refers to b5_, medium_, plant_)
  stepper_.reset();
  log_.reset();
  frames_.clear();
  events_.clear();
  records_.clear();
  scans_ = 0;
  step_wall_s_.clear();
  b5_solve_s_.clear();
  b5_cycle_s_.clear();
  scan_cast_s_.clear();
  last_contacts_ = 0;
  last_fallback_.clear();
  wall_s_ = 0.0;
  scenario_ = scenario;
  bag_ = bag;
  seed_ = seed;
  case_ = std::move(c);
  goals_ = std::move(goals_new);
  start_ = start_new;
  perturbation_ = perturbation;
  b5_config_sha256_ = b5_sha;
  tracking_model_sha256_ = tracking_sha;
  plant_ = std::move(plant);
  b5_ = std::move(b5);
  medium_ = std::move(medium);
  posture_ = posture;
  timeout_s_ = timeout;

  json boxes = json::array(), movers = json::array();  // what the viewer draws beyond the site
  for (const auto & b : setup.boxes)
    boxes.push_back({{"name", b.name}, {"center", {b.center.x(), b.center.y(), b.center.z()}},
                     {"size", {b.size.x(), b.size.y(), b.size.z()}}, {"yaw", b.yaw}});
  for (const auto & m : setup.movers) movers.push_back({{"name", m.name}, {"size", {m.size.x(), m.size.y(), m.size.z()}}});
  json goals = json::array();
  for (const auto & g : goals_) goals.push_back({g.goal[0], g.goal[1], g.goal[2]});
  json hello = {{"schema", mmp::kSchemaVersion},
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
                      {"reserved", reserved_bodies()},
                      {"provenance", config_.provenance},
                      {"profile_sha256", sha256_file(profile_path_)},
                      {"robot_model_sha256", sha256_file(model_path)},
                      {"site_model_sha256", sha256_file(site_mjcf_)},
                      {"config", {{"medium", config_.medium},
                                  {"medium_config", config_.medium_config},
                                  {"lidar", config_.lidar},
                                  {"scan_decimation", config_.scan_decimation},
                                  {"timeout_s", timeout_s_},
                                  {"iteration_budget", config_.iteration_budget},
                                  {"perturb_start", config_.perturb_start},
                                  {"b5_config", config_.b5_config},
                                  {"b5_config_sha256", b5_config_sha256_},
                                  {"gate_rule", mmm::to_string(case_.b5_config.slack_gate.inside_rule)},
                                  {"tracking_model", case_.b5_config.tracking_model},
                                  {"tracking_model_sha256", tracking_model_sha256_},
                                  {"tracking_adaptation", case_.b5_config.tracking_adaptation},
                                  {"tracking_adapt_sideslip", case_.b5_config.tracking_adapt_sideslip},
                                  {"start_nudge_rad", config_.start_nudge_rad},
                                  {"stall_s", config_.stall_s},
                                  {"diagnostics", config_.diagnostics}}}};
  if (config_.row_trace) {  // E3 Task 1: only when on, so logs without the trace keep their Hello (review M1)
    hello["config"]["row_trace"] = kRowTracePeriodS;                // the trace's period (s)
    hello["config"]["row_trace_stage_s"] = case_.b5_config.dt;       // B5's stage: a row's lookahead is stage x this
  }
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

std::vector<mmm::RecordedGoal> mission_goals(const std::vector<mmm::RecordedGoal> & plans)
{
  constexpr double kSame = 1e-6;  // as MediumPlanner::set_route's same-goal test
  std::vector<mmm::RecordedGoal> out;
  for (const auto & p : plans) {
    if (p.points.size() < 2) continue;
    if (!out.empty() && std::abs(out.back().goal[0] - p.goal[0]) <= kSame && std::abs(out.back().goal[1] - p.goal[1]) <= kSame &&
        std::abs(std::remainder(out.back().goal[2] - p.goal[2], 2.0 * M_PI)) <= kSame)
      continue;  // a replan of the same goal
    out.push_back(p);
  }
  return out;
}

void Session::set_mission(const std::vector<std::pair<mmm::Route, mobile_manipulator_navigation::Pose2>> & goals)
{
  if (!plant_) throw std::logic_error("Session: set_mission before reset");
  if (goals.empty()) throw std::invalid_argument("Session: a mission needs a goal");
  records_.clear();  // the summary covers the goals set last (the run log keeps every goal)
  goals_.clear();
  for (const auto & [route, goal] : goals) goals_.push_back({route, goal});
  goal_index_ = 0;
  start_next_goal();
}

void Session::set_route(const mmm::Route & route, const mobile_manipulator_navigation::Pose2 & goal)
{
  if (!plant_) throw std::logic_error("Session: set_route before reset");
  records_.clear();  // the summary covers the goals set last (the run log keeps every goal)
  goals_ = {{route, goal}};
  goal_index_ = 0;
  start_next_goal();
}

void Session::start_next_goal()
{
  stepper_.reset();
  if (goal_index_ >= goals_.size()) return;
  progress_ref_ = plant_->state();  // the stall detector's reference: the goal starts as progress
  progress_t_ = time();
  const Goal & g = goals_[goal_index_];
  const auto & bench = *case_.bench;
  mmd::ClosedLoopOptions co;
  co.timeout_s = timeout_s_;
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
          {"capped", out.capped}, {"layout", "x y yaw q0..q5"},
          // E1's decision trace: the winner, the cycle's kind and each seed (no timings: same-seed logs stay identical)
          {"seed", out.seed}, {"full_multistart", out.full_multistart}, {"waiting", out.waiting},
          {"homotopy_switched", out.homotopy_switched}, {"seeds", seed_rows(out.seeds)}},  // no solve_s: wall clock (E2; the summary has it)
         {samples});
  };
  goal_wall_start_ = wall_s_;
  stepper_ = std::make_unique<mmd::ClosedLoopStepper>(*b5_, *medium_, case_.b5_config, posture_, g.route, g.goal, *plant_,
                                                      bench.world(), bench.primitives(), co);
}

std::pair<double, double> b5_timing_sample(const mmm::CycleTiming & tm) { return {tm.solve_s, tm.total_s}; }

bool Session::step_cycle()
{
  if (!stepper_) return false;
  const double wall0 = now_s();
  const long b5_before = stepper_->b5_cycles_stepped();
  const bool stepped = stepper_->step();
  wall_s_ += now_s() - wall0;
  if (config_.profile_timing && stepper_->b5_cycles_stepped() > b5_before) {  // E2 Task 9 (T7)
    const auto [solve, cycle] = b5_timing_sample(stepper_->last_b5_timing());
    b5_solve_s_.push_back(solve);
    b5_cycle_s_.push_back(cycle);
  }
  if (stepped) {  // the plant moved one cycle
    if (config_.profile_timing) step_wall_s_.push_back(plant_->plant().last_step_wall_s());
    const mmp::PlantState & truth = plant_->plant().true_state();
    Eigen::Matrix<double, 9, 1> pose;
    pose << truth.base.x, truth.base.y, truth.base.yaw, truth.q;
    const int contacts = plant_->contacts();
    json state_header = {{"t", truth.t}, {"goal", goal_index_}, {"status", mmm::to_string(stepper_->last_status())},
          {"fallback", stepper_->last_fallback()}, {"contacts", contacts}, {"movers", mover_xy()}, {"bodies", body_poses()},
          {"gate", {stepper_->last_gate().slack, stepper_->last_gate().violation, stepper_->last_gate().unsafe ? 1 : 0}},
          {"dwell", stepper_->last_gate().dwell ? 1 : 0},
          {"layout", "true x y yaw q; b5 state; executed"}};
    if (case_.b5_config.slack_gate.depth_guard_m >= 0.0)  // E1 CP3: the guard on: [held 0/1, deepest measured depth m]
      state_header["guard"] = {stepper_->last_gate().guard_tripped ? 1 : 0, stepper_->last_gate().guard_depth};
    if (config_.diagnostics) {  // E1 Task 9: [deepest pair's depth, its stage-1 prediction, linearization error or null]
      const auto & g = stepper_->last_gate();
      state_header["diag"] = {g.depth, g.depth_stage1, std::isfinite(g.lin_error) ? json(g.lin_error) : json()};
      // E2 Task 1: the deepest pair's name, its distance gradient and kink (forward minus backward slope) at B5's
      // state, and the command B5 sent (v, omega, qdot1..6: the requested control; the realistic arm follows the
      // trajectory built from the plan, whose velocity is the executed block's last six). Time: the frame at t
      // carries what was evaluated and sent at t - one cycle, so the transition from the previous frame's pose to
      // this frame's ran under this frame's diag_cmd, from the previous frame's B5 state.
      state_header["diag_pair"] = g.pair;
      state_header["diag_grad"] = std::vector<double>(g.gradient.data(), g.gradient.data() + g.gradient.size());
      state_header["diag_kink"] = std::vector<double>(g.kink.data(), g.kink.data() + g.kink.size());
      const mmm::Control & u = stepper_->last_command();
      state_header["diag_cmd"] = std::vector<double>(u.data(), u.data() + u.size());
      // E2 Task 4: what the plan asked for before the output stage (rate band, boxes, breakaway boost from rest)
      // E2 Task 4b, the store_room probe: [link, obstacle, distance, in stage 1, in any row, rows dropped]
      state_header["diag_nearest"] = {g.probe_link, g.probe_obstacle, g.probe_distance, g.probe_in_stage1 ? 1 : 0,
                                      g.probe_in_rows ? 1 : 0, g.probe_dropped};
      const mmm::Control & p = stepper_->last_planned();
      state_header["diag_plan_u0"] = std::vector<double>(p.data(), p.data() + p.size());
      // E2 Task 7: the tracking model this cycle planned with, [g_inf, g_0, sideslip k] (the online correction's)
      state_header["diag_track"] = {g.tracking(0), g.tracking(1), g.tracking(2)};
    }
    if (config_.row_trace && !stepper_->last_row_trace_x().empty()) {
      // E3 Task 1: the rows B5 traced at this frame's cycle (evaluated at t - one cycle, as diag), one
      // [pair, type, stage, margin, d_lin, d_plan, g0..g8] each; the plan's states at stages 1, 2, 5, 10 and its
      // controls at stages 0..9, flat
      json rows = json::array();
      for (const mmm::RowTrace & r : stepper_->last_row_trace()) {
        json row = {r.pair, r.type, r.stage, r.margin, r.d_lin, r.d_plan};
        for (int i = 0; i < r.gradient.size(); ++i) row.push_back(r.gradient(i));
        rows.push_back(std::move(row));
      }
      std::vector<double> xs, us;
      for (const mmm::State & x : stepper_->last_row_trace_x()) xs.insert(xs.end(), x.data(), x.data() + x.size());
      for (const mmm::Control & u : stepper_->last_row_trace_u()) us.insert(us.end(), u.data(), u.data() + u.size());
      state_header["rows"] = std::move(rows);
      state_header["rows_x"] = xs;
      state_header["rows_u"] = us;
    }
    emit(mmp::FrameType::State, state_header, {floats(pose), floats(plant_->state()), floats(plant_->executed())});
    if (contacts > 0 && last_contacts_ == 0) {
      const auto & c = plant_->truth().contacts.front();
      event("contact", {{"robot_geom", c.robot_geom}, {"other_geom", c.other_geom}, {"depth", c.depth}});
    }
    last_contacts_ = contacts;
    const std::string & fallback = stepper_->last_fallback();
    if (!fallback.empty() && fallback != last_fallback_) event("fallback", {{"reason", fallback}});
    last_fallback_ = fallback;
    while (auto scan = plant_->plant().scan()) {
      if (config_.profile_timing) scan_cast_s_.push_back(scan->cast_s);
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
    if (config_.stall_s > 0.0) {  // progress: the base 5 cm or 0.1 rad, or an arm joint 0.05 rad, from the reference
      const mmm::State & x = plant_->state();
      const bool moved = (x.head<2>() - progress_ref_.head<2>()).norm() > kStallBaseM ||
                         std::abs(std::remainder(x(2) - progress_ref_(2), 2.0 * M_PI)) > kStallYawRad ||
                         (x.tail<6>() - progress_ref_.tail<6>()).cwiseAbs().maxCoeff() > kStallJointRad;
      if (moved) {
        progress_ref_ = x;
        progress_t_ = time();
      } else if (time() - progress_t_ >= config_.stall_s) {
        finish_goal(true);
        return active();
      }
    }
    return true;
  }
  finish_goal();
  return active();
}

void Session::finish_goal(bool stalled)
{
  const auto & r = stepper_->result();
  const Goal & g = goals_[goal_index_];
  records_.push_back(record(r, g.goal, wall_s_ - goal_wall_start_, stepper_->timed_out()));
  records_.back()["stalled"] = stalled;
  event(r.reached ? "goal_reached" : stalled ? "stalled" : stepper_->timed_out() ? "timeout" : "failed",
        {{"goal", goal_index_}, {"end_status", r.end_status}, {"time_s", r.time_s}});
  if (r.reached && !stalled && goal_index_ + 1 < goals_.size()) {
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

json Session::reserved_bodies()
{
  json out = json::array();
  for (int i = 0; i < mmp::kReservedBodies; ++i)
    out.push_back({{"name", "reserved/" + std::to_string(i)}, {"size", {mmp::kReservedSize.x(), mmp::kReservedSize.y(), mmp::kReservedSize.z()}}});
  return out;
}

json Session::body_poses() const
{
  json out = json::array();
  for (const auto & b : plant_->plant().reserved_positions()) out.push_back({b.name, b.x, b.y, b.yaw});
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
          {"stalled", false},
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
    s["stalled"] = goals.back().at("stalled");
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
  s["time_s"] = goals.empty() ? time() : goals.back().at("time_s").get<double>();  // as the goal's record (medium_bench)
  s["wall_s"] = wall_s_;
  s["cell"] = config_.cell.empty() ? scenario_ : config_.cell;
  s["bag"] = bag_;
  s["seed"] = seed_;
  s["plant_profile"] = config_.mode == mmp::PlantMode::Realistic ? "mujoco-realistic" : "mujoco-ideal";
  s["profile"] = profile_path_;
  s["lidar"] = config_.lidar;
  s["host"] = hostname();
  s["site_checksum"] = plant_ ? text_of(plant_->plant().model(), "site_checksum") : "";
  s["start_perturbation"] = {perturbation_(0), perturbation_(1), perturbation_(2)};
  s["start_nudge_rad"] = config_.start_nudge_rad;
  s["stall_s"] = config_.stall_s;  // 0: no stall detector (as every run before 5bd4300, the baseline replicas)
  s["b5_config"] = config_.b5_config;
  s["b5_config_sha256"] = b5_config_sha256_;
  s["tracking_model"] = case_.b5_config.tracking_model;
  s["tracking_model_sha256"] = tracking_model_sha256_;
  s["tracking_adaptation"] = case_.b5_config.tracking_adaptation;
  s["tracking_adapt_sideslip"] = case_.b5_config.tracking_adapt_sideslip;
  s["gate_rule"] = mmm::to_string(case_.b5_config.slack_gate.inside_rule);
  s["real_time_factor"] = wall_s_ > 0.0 ? time() / wall_s_ : 0.0;
  s["goals"] = goals;
  if (config_.profile_timing) {  // gate P (spec Section 8): wall times on this host
    auto ms = [](std::vector<double> v) {
      json out = {{"n", v.size()}};
      if (v.empty()) return out;
      std::sort(v.begin(), v.end());
      auto at = [&](double q) { return 1000.0 * v[std::min(v.size() - 1, static_cast<size_t>(q * (v.size() - 1) + 0.5))]; };
      out["p50"] = at(0.5);
      out["p99"] = at(0.99);
      out["max"] = 1000.0 * v.back();
      return out;
    };
    double mean = 0.0;
    for (double x : step_wall_s_) mean += x;
    mean = step_wall_s_.empty() ? 0.0 : mean / static_cast<double>(step_wall_s_.size());
    s["timing"] = {{"plant_step_ms", ms(step_wall_s_)}, {"plant_step_fraction_of_real_time", mean / kCycleS},
                   {"scan_cast_ms", ms(scan_cast_s_)},
                   // E2 Task 9 (T7): B5's QP solve, and its cycle without the diagnostics (wall clock, this host)
                   {"b5_solve_ms", ms(b5_solve_s_)}, {"b5_cycle_ms", ms(b5_cycle_s_)}};
  }
  return s;
}
}  // namespace mobile_manipulator_campaign
