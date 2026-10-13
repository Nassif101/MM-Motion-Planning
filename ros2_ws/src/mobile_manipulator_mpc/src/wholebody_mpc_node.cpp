// wholebody_mpc_node: the B5 local whole-body MPC as Nav2's FollowPath server, in its own process
// (Phase 4 spec, Sections 2, 3, 4 and 6). Nav2's behaviour tree sends the route on the action
// `wholebody_mpc/follow_path`; each cycle the node reads the measured state, runs one
// mobile_manipulator_mpc::Controller step and sends
//   - the base command (v, omega) on `cmd_vel_wholebody` (the launch remaps it to the collision
//     monitor's input): the controller's sent command, which is authoritative (the plan's first
//     control may differ from it at the node's rate),
//   - the predicted arm motion to the JTC on /arm_controller/joint_trajectory (ruling R24): point 0
//     at time 0 on the reference the JTC commands now (the last trajectory sent, sampled now:
//     positions and velocities), so the JTC continues its reference instead of splining from the
//     measured joints (with interpolate_from_desired_state false it only uses the measured state
//     before a first point that lies in the future); then the plan's stages 1..N with positions
//     and velocities, then a braking tail that ends at rest (JTC rejects a trajectory that ends
//     moving), header.stamp 0 (start on receipt),
//   - the plan on `wholebody_mpc/plan` (WholeBodyPlan) and diagnostics on `wholebody_mpc/cycle`
//     (MpcCycle).
// State: TF map -> base_footprint (latest, without waiting), the six arm joints merged by name
// from /joint_states (non-finite values ignored), /odom for the measured velocity (at rest for the
// breakaway stage). The arm part of the MPC's state is the commanded reference (above) while every
// joint is within kReanchorRad of the measured joints (or the joint states are stale); otherwise
// the measured joints (a re-anchor, counted in the statistics and flagged in MpcCycle). Before the
// node has sent anything the reference is the measured joints at rest. Joint states and odometry older than 0.5 s (receive time, steady clock) or a
// transform older than 0.5 s (node clock) are stale: the controller brakes and the goal fails.
// A goal without a complete state (an arm joint never received, no transform, no odometry) fails
// at once with a zero command: the node never commands from missing data.
// World: the map boxes (boxes_file) plus the obstacles of the `scenario` parameter, and the
// scenario's movers from /scenario/movers (a PoseArray of box centres sorted by mover name, as
// Unity publishes it). Panel references (references:=scripted) come from the scenario's mission
// steps; the default none is the unscripted, primary B5 configuration.
//
// Goals: a new goal preempts the active one in place (the old one is aborted; Controller::
// update_goal swaps route, goal and references; the command, its rate band, the failures in a
// row, the warm plan and the no-progress window continue). Every stop (cancel, goal end, braking,
// stale state, error, shutdown) sends a zero base command, resets the controller's sent command
// and stops the arm with a braking hold from the reference (q, v) now (point 0 at time 0) to zero
// velocity at q + v T / 2, T = max(0.5 s, max_j |v_j| / qdd_max_j): constant deceleration within
// the acceleration limit. A holding cycle (the slack gate: the plan would push deeper into an
// obstacle) sends the controller's braking base command and the same arm hold once per hold, keeps
// the goal and plans again every cycle; tracking resumes with the next stream. The
// goal succeeds at the goal tolerance and fails when the controller fails (solver failures or
// missed deadlines beyond the fallback cycles: fallback_s of wholebody_mpc.yaml at rate_hz, ruling
// R27; stale state) or without progress (less than
// 0.3 m and 0.5 rad of base motion in 10 s). A cycle whose total time exceeds the period counts
// as a controller failure.
//
// The control loop runs on a wall timer in its own MutuallyExclusive callback group; the action
// server and the subscriptions share another; the timer alone touches the controller, the world
// and the outgoing messages (and the pre-shutdown callback, under the same lock). An exception in
// a cycle ends the active goal and the node keeps running. Message buffers and the statistics are
// allocated at start-up; the QP solver is set up by the first real linearization (first cycle of
// the first goal). On shutdown the node writes the timing summary to `stats_file` (JSON;
// "" = none), with the options it used and `arm_trace`: what it chose for the arm, sampled at 5 Hz
// (node clock) while a goal is active: time t, the measured joints q (model order), the panel's
// roll, pitch and yaw in base_footprint from the model's panel pose at those joints (panel_rpy,
// URDF convention) and the tightness.
//
// Parameters: rate_hz (20), scenario (""), references (none|scripted), boxes_file, envelope_file
// (defaults: the navigation package's map boxes and operating envelope), stats_file (""), and the
// posture preference's posture_mode (blend|comfort_only|none) and comfort_pose (home|carry), the
// clearance cost's clearance_weight and clearance_target_m and the slack gate's slack_gate,
// slack_gate_horizon_s, slack_gate_resume_horizon_s, slack_gate_resume_dwell_s and
// slack_gate_tolerance_m, whose defaults are wholebody_mpc.yaml's (blend, carry, 0 = off, 0.25 m;
// gate on, 1 s, 2 s, 0.2 s, 5 mm; the recorded B5 ran home without the gate). The statistics count the
// holding cycles and hold events.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "mobile_manipulator_geometry/footprint_projection.hpp"
#include "mobile_manipulator_geometry/robot_model.hpp"
#include "mobile_manipulator_interfaces/msg/mpc_cycle.hpp"
#include "mobile_manipulator_interfaces/msg/whole_body_plan.hpp"
#include "mobile_manipulator_mpc/controller.hpp"
#include "mobile_manipulator_mpc/osqp_solver.hpp"
#include "mobile_manipulator_navigation/known_world.hpp"
#include "mobile_manipulator_navigation/scenario_spec.hpp"

namespace mmg = mobile_manipulator_geometry;
namespace mmm = mobile_manipulator_mpc;
namespace mmn = mobile_manipulator_navigation;
using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowPath>;
using Steady = std::chrono::steady_clock;
using Joints = Eigen::Matrix<double, 6, 1>;

namespace
{
constexpr double kStaleS = 0.5;          // joint states, odometry and TF older than this are stale
constexpr double kTailS = 0.5;           // shortest braking tail after the horizon
constexpr double kMinLeadS = 0.005;      // a streamed point closer than this to now is already due
constexpr double kHoldMinS = mmm::kArmHoldMinS;  // shortest stop: a hold point is reached at least this late
constexpr double kProgressWindowS = 10.0;  // no-progress failure: less motion than below in this window
constexpr double kProgressM = 0.3;       // (Nav2's progress checker: 0.3 m in 10 s)
constexpr double kProgressRad = 0.5;     // in-place turns count as progress
constexpr double kReanchorRad = 0.15;    // reference farther than this from the measured joints: re-anchor
constexpr double kNearMissFraction = 0.8;  // a cycle above this fraction of the period is a near miss
constexpr double kStatsReserveS = 3600.0;  // an hour at rate_hz before the statistics reallocate (R33-M8)
constexpr double kArmTraceS = 0.2;         // arm trace period (5 Hz) while a goal is active

std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot read " + path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

std::string share(const std::string & package) { return ament_index_cpp::get_package_share_directory(package); }

double seconds(Steady::time_point t) { return std::chrono::duration<double>(t.time_since_epoch()).count(); }
double seconds(Steady::duration d) { return std::chrono::duration<double>(d).count(); }

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

// Summary of one per-cycle quantity: mean, median, p90, p95, p99 (nearest rank), max and, for a
// time, the number of samples above the period.
mmn::Json summary(std::vector<double> values, std::optional<double> period)
{
  mmn::Json out = mmn::Json::object();
  if (values.empty()) return out;
  std::sort(values.begin(), values.end());
  const auto rank = [&](double p) {
    const auto n = static_cast<double>(values.size());
    return values[std::min(values.size() - 1, static_cast<size_t>(std::max(0.0, std::ceil(p * n) - 1.0)))];
  };
  out["mean"] = std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
  out["median"] = rank(0.5);
  out["p90"] = rank(0.9);
  out["p95"] = rank(0.95);
  out["p99"] = rank(0.99);
  out["max"] = values.back();
  if (period) out["misses"] = std::count_if(values.begin(), values.end(), [&](double v) { return v > *period; });
  return out;
}

enum class End { Succeeded, Aborted, Canceled };
}  // namespace

class WholeBodyMpcNode : public rclcpp::Node
{
public:
  WholeBodyMpcNode()
  : Node("wholebody_mpc"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const std::string navigation = share("mobile_manipulator_navigation");
    rate_hz_ = declare_parameter("rate_hz", 20.0);
    const std::string scenario_name = declare_parameter("scenario", std::string(""));
    const std::string references = declare_parameter("references", std::string("none"));
    const std::string boxes_file =
      declare_parameter("boxes_file", navigation + "/maps/construction_site.boxes.yaml");
    const std::string envelope_file =
      declare_parameter("envelope_file", navigation + "/config/nav_operating_envelope.yaml");
    stats_file_ = declare_parameter("stats_file", std::string(""));
    if (!(rate_hz_ > 0.0)) throw std::invalid_argument("rate_hz must be positive");
    if (references != "none" && references != "scripted") {
      throw std::invalid_argument("references must be none or scripted, not '" + references + "'");
    }
    period_s_ = 1.0 / rate_hz_;

    // The controller (formulation, collision model, OSQP backend) from the installed description,
    // payload, operating envelope and MPC configuration.
    const std::string urdf = read_file(share("mobile_manipulator_description") + "/urdf/mobile_manipulator.urdf");
    const mmn::Json payload = mmn::Json::parse(read_file(share("mobile_manipulator_control") +
                                                         "/config/qualified_payload.json"));
    // The state, posture targets, limits and the streamed trajectory use the model's joint order
    // (R33-M7): a payload in another order fails here, before anything is built.
    mmm::require_model_joint_order(payload);
    const mmm::Limits limits = mmm::load_limits(envelope_file, urdf);
    mmm::MpcConfig config = mmm::load_mpc_config(share("mobile_manipulator_mpc") + "/config/wholebody_mpc.yaml");
    // Posture preference: wholebody_mpc.yaml's values unless overridden (launch arguments).
    config.posture_mode = mmm::parse_posture_mode(
      declare_parameter("posture_mode", std::string(mmm::to_string(config.posture_mode))));
    config.comfort_pose = declare_parameter("comfort_pose", config.comfort_pose);
    // Clearance cost: wholebody_mpc.yaml's weight (0: off) and target unless overridden.
    config.weights.clearance = declare_parameter("clearance_weight", config.weights.clearance);
    config.weights.clearance_target_m = declare_parameter("clearance_target_m", config.weights.clearance_target_m);
    if (!(config.weights.clearance >= 0.0) || !std::isfinite(config.weights.clearance) ||
        !(config.weights.clearance_target_m > 0.0)) {
      throw std::invalid_argument("clearance_weight must be >= 0 and clearance_target_m > 0");
    }
    // Slack gate: wholebody_mpc.yaml's unless overridden (launch argument slack_gate).
    mmm::SlackGate & gate = config.slack_gate;
    gate.enabled = declare_parameter("slack_gate", gate.enabled);
    gate.horizon_s = declare_parameter("slack_gate_horizon_s", gate.horizon_s);
    gate.resume_horizon_s = declare_parameter("slack_gate_resume_horizon_s", gate.resume_horizon_s);
    gate.resume_dwell_s = declare_parameter("slack_gate_resume_dwell_s", gate.resume_dwell_s);
    gate.tolerance_m = declare_parameter("slack_gate_tolerance_m", gate.tolerance_m);
    if (!(gate.horizon_s > 0.0) || !(gate.resume_horizon_s > 0.0) || !(gate.resume_dwell_s >= 0.0) ||
        !(gate.tolerance_m >= 0.0) || !std::isfinite(gate.tolerance_m)) {
      throw std::invalid_argument("slack gate: horizons must be positive, dwell and tolerance >= 0");
    }
    // Before the controller: a comfort pose missing from the payload fails here, by name.
    posture_ = mmm::posture_targets(payload, config.comfort_pose, config.posture_mode);
    mmm::OsqpOptions options;
    options.eps_abs = options.eps_rel = config.solver_eps;
    options.max_iter = config.solver_max_iter;
    fallback_cycles_ = mmm::fallback_cycles_for(config.fallback_s, rate_hz_);
    controller_ = std::make_unique<mmm::Controller>(
      mmm::Formulation(config.horizon, config.dt, limits, config.weights),
      mmm::CollisionModel(mmg::load_primitives(urdf, mmg::payload_from_json(payload)), mmm::CollisionConfig{}),
      std::make_unique<mmm::OsqpSolver>(options), limits.breakaway, fallback_cycles_, mmm::kGoalXyM, mmm::kGoalYawRad,
      config.slack_gate);
    limits_ = limits;
    horizon_ = config.horizon;
    dt_ = config.dt;
    config_ = config;
    std::copy(mmm::kArmJoints.begin(), mmm::kArmJoints.end(), joint_names_.begin());

    // World and references from the scenario (none: the map boxes only, unscripted).
    std::optional<mmn::Json> scenario;
    if (!scenario_name.empty()) scenario = mmn::ScenarioConfig(navigation).resolve(scenario_name).at("scenario");
    const auto boxes = mmn::known_boxes(boxes_file, scenario ? &*scenario : nullptr);
    world_.set_static(boxes);
    if (scenario) {
      for (const auto & mover : scenario->value("movers", mmn::Json::array())) {
        movers_.push_back({mover.at("name").get<std::string>(),
                           Eigen::Vector3d(mover.at("size_x").get<double>(), mover.at("size_y").get<double>(),
                                           mover.at("height").get<double>())});
      }
      std::sort(movers_.begin(), movers_.end(), [](const Mover & a, const Mover & b) { return a.name < b.name; });
    }
    mover_positions_.assign(movers_.size(), Eigen::Vector2d::Zero());
    if (references == "scripted") {
      if (!scenario || scenario->at("task") != "mission") {
        throw std::invalid_argument("references:=scripted needs a mission scenario (scenario:=" + scenario_name + ")");
      }
      const mmg::FootprintProjector projector(urdf, mmg::payload_from_json(payload));
      anchored_ = mmm::mission_references(*scenario, projector, payload);
      RCLCPP_INFO(get_logger(), "Loaded %zu panel references from scenario %s (references:=scripted)",
                  anchored_.size(), scenario_name.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "No panel references (references:=none): unscripted");
    }
    RCLCPP_INFO(get_logger(), "World: %zu known boxes, %zu movers (scenario '%s')", boxes.size(), movers_.size(),
                scenario_name.c_str());
    RCLCPP_INFO(get_logger(), "Posture preference: %s, comfort pose %s; clearance cost weight %g, target %.3f m",
                mmm::to_string(config.posture_mode), config.comfort_pose.c_str(), config.weights.clearance,
                config.weights.clearance_target_m);
    RCLCPP_INFO(get_logger(), "Slack gate: %s (first %.2f s of the plan, tolerance %.4f m; resume after %.2f s of plans "
                "passing over %.2f s)", gate.enabled ? "on" : "off", gate.horizon_s, gate.tolerance_m, gate.resume_dwell_s,
                gate.resume_horizon_s);

    allocate_messages();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_wholebody", 10);
    trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 10);
    plan_pub_ = create_publisher<mobile_manipulator_interfaces::msg::WholeBodyPlan>("wholebody_mpc/plan", 10);
    cycle_pub_ = create_publisher<mobile_manipulator_interfaces::msg::MpcCycle>("wholebody_mpc/cycle", 10);

    io_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    loop_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions io;
    io.callback_group = io_group_;
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 50, [this](const sensor_msgs::msg::JointState & m) { on_joints(m); }, io);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 20, [this](const nav_msgs::msg::Odometry & m) { on_odom(m); }, io);
    movers_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/scenario/movers", 10, [this](const geometry_msgs::msg::PoseArray & m) { on_movers(m); }, io);
    action_server_ = rclcpp_action::create_server<FollowPath>(
      this, "wholebody_mpc/follow_path",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowPath::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](std::shared_ptr<GoalHandle> handle) { on_accepted(std::move(handle)); }, rcl_action_server_get_default_options(),
      io_group_);
    timer_ = create_wall_timer(std::chrono::duration<double>(period_s_), [this] { cycle(); }, loop_group_);
    shutdown_callback_ = get_node_base_interface()->get_context()->add_pre_shutdown_callback([this] { on_shutdown(); });
    RCLCPP_INFO(get_logger(),
                "Whole-body MPC at %.0f Hz (N = %d, dt = %.2f s, fallback %d cycles = %.2f s), serving "
                "wholebody_mpc/follow_path", rate_hz_, horizon_, dt_, fallback_cycles_, config.fallback_s);
  }

  ~WholeBodyMpcNode() override
  {
    get_node_base_interface()->get_context()->remove_pre_shutdown_callback(shutdown_callback_);
  }

  void write_stats() const
  {
    if (stats_file_.empty()) return;
    mmn::Json stats{{"rate_hz", rate_hz_}, {"period_s", period_s_}, {"fallback_cycles", fallback_cycles_},
                    {"cycles", stats_.total.size()},
                    {"deadline_misses", deadline_misses_}, {"near_misses", near_misses_},
                    {"near_miss_fraction", kNearMissFraction}};
    mmn::Json statuses = mmn::Json::object();
    for (int s = 0; s < kStatuses; ++s) {
      statuses[mmm::to_string(static_cast<mmm::CycleStatus>(s))] = status_counts_[static_cast<size_t>(s)];
    }
    stats["statuses"] = statuses;
    stats["linearize_s"] = summary(stats_.linearize, period_s_);
    stats["collision_s"] = summary(stats_.collision, period_s_);
    stats["setup_s"] = summary(stats_.setup, period_s_);
    stats["solve_s"] = summary(stats_.solve, period_s_);
    stats["total_s"] = summary(stats_.total, period_s_);
    stats["state_to_command_s"] = summary(stats_.state_to_command, period_s_);
    stats["iterations"] = summary(stats_.iterations, std::nullopt);
    stats["arm_reanchors"] = arm_reanchors_;
    // Slack gate: cycles held, holds (entries into holding) and cycles whose plan failed the gate
    // (also counted with the gate off: the plans the recorded B5 would have executed).
    stats["holding_cycles"] = status_counts_[static_cast<size_t>(mmm::CycleStatus::Holding)];
    stats["holding_events"] = holding_events_;
    stats["gate_unsafe_cycles"] = gate_unsafe_cycles_;
    stats["gate_max_excess_m"] = gate_max_excess_;
    stats["arm_reference_offset_rad"] = summary(stats_.arm_offset, std::nullopt);
    // The options this run used (wholebody_mpc.yaml, or the node parameters that override it).
    // What the MPC chose for the arm (5 Hz while a goal was active), rounded for size.
    const auto round_to = [](double value, double step) { return std::round(value / step) * step; };
    mmn::Json trace = mmn::Json::array();
    for (const auto & sample : arm_trace_) {
      mmn::Json q = mmn::Json::array(), rpy = mmn::Json::array();
      for (int j = 0; j < 6; ++j) q.push_back(round_to(sample.q(j), 1e-4));
      for (int j = 0; j < 3; ++j) rpy.push_back(round_to(sample.panel_rpy(j), 1e-4));
      trace.push_back({{"t", round_to(sample.t, 1e-3)}, {"q", q}, {"panel_rpy", rpy},
                       {"tightness", round_to(sample.tightness, 1e-4)}});
    }
    stats["arm_trace"] = trace;
    stats["options"] = {{"posture_mode", mmm::to_string(config_.posture_mode)},
                        {"comfort_pose", config_.comfort_pose},
                        {"clearance_weight", config_.weights.clearance},
                        {"clearance_target_m", config_.weights.clearance_target_m},
                        {"slack_gate", config_.slack_gate.enabled},
                        {"slack_gate_horizon_s", config_.slack_gate.horizon_s},
                        {"slack_gate_resume_horizon_s", config_.slack_gate.resume_horizon_s},
                        {"slack_gate_resume_dwell_s", config_.slack_gate.resume_dwell_s},
                        {"slack_gate_tolerance_m", config_.slack_gate.tolerance_m}};
    std::ofstream(stats_file_) << stats.dump(2) << "\n";
    RCLCPP_INFO(get_logger(), "Wrote the timing summary of %zu cycles to %s", stats_.total.size(), stats_file_.c_str());
  }

private:
  static constexpr int kStatuses = 7;  // mmm::CycleStatus values

  struct Mover
  {
    std::string name;
    Eigen::Vector3d size;
  };

  struct Measured
  {
    mmm::State x = mmm::State::Zero();
    bool complete = false;  // every part received at least once
    bool fresh = false;
    std::string missing, stale;  // descriptions for the result (set on failure only)
    bool tf_problem = false;
    double v = 0.0, omega = 0.0;
  };

  // ---- inputs (io callback group) ----

  void on_joints(const sensor_msgs::msg::JointState & m)
  {
    const double now = seconds(Steady::now());
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t i = 0; i < m.name.size() && i < m.position.size(); ++i) {
      if (!std::isfinite(m.position[i])) continue;
      for (size_t j = 0; j < joint_names_.size(); ++j) {
        if (m.name[i] != joint_names_[j]) continue;
        joints_(static_cast<int>(j)) = m.position[i];
        joint_received_[j] = now;
        break;
      }
    }
  }

  void on_odom(const nav_msgs::msg::Odometry & m)
  {
    const double now = seconds(Steady::now());
    std::lock_guard<std::mutex> lock(state_mutex_);
    odom_v_ = std::hypot(m.twist.twist.linear.x, m.twist.twist.linear.y);
    odom_omega_ = m.twist.twist.angular.z;
    odom_received_ = now;
  }

  void on_movers(const geometry_msgs::msg::PoseArray & m)
  {
    if (m.poses.empty()) return;
    if (m.poses.size() != movers_.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "/scenario/movers has %zu boxes but scenario has %zu movers: ignored", m.poses.size(),
                           movers_.size());
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t i = 0; i < m.poses.size(); ++i) mover_positions_[i] = {m.poses[i].position.x, m.poses[i].position.y};
    mover_stamp_ = rclcpp::Time(m.header.stamp).seconds();
    movers_new_ = true;
  }

  void on_accepted(std::shared_ptr<GoalHandle> handle)
  {
    std::shared_ptr<GoalHandle> superseded;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      superseded = std::move(pending_);
      pending_ = std::move(handle);
    }
    // Two goals within one cycle: the older never started.
    if (superseded) superseded->abort(result(FollowPath::Result::NONE, "preempted by a newer goal before it started"));
  }

  // ---- the control loop (loop callback group) ----

  // One timer tick. Any exception ends the active goal (zero command, braking hold, abort) and
  // leaves the node running; nothing runs once shutdown has begun.
  void cycle()
  {
    std::lock_guard<std::mutex> lock(loop_mutex_);
    if (shutting_down_) return;
    try {
      run_cycle();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "MPC cycle failed: %s", error.what());
      stop_goal(std::string("internal error: ") + error.what());
    }
  }

  // Ends the active goal after an error or at shutdown, never throwing.
  void stop_goal(const std::string & message)
  {
    try {
      if (active_) end_goal(End::Aborted, FollowPath::Result::UNKNOWN, message);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Could not end the goal cleanly: %s", error.what());
      active_.reset();
    }
  }

  // Pre-shutdown (the context is still valid): zero command, braking hold and abort for an
  // active goal; pending goals are aborted; the loop stops.
  void on_shutdown()
  {
    std::lock_guard<std::mutex> lock(loop_mutex_);
    shutting_down_ = true;
    if (timer_) timer_->cancel();
    std::shared_ptr<GoalHandle> pending;
    {
      std::lock_guard<std::mutex> goals(goal_mutex_);
      pending = std::move(pending_);
    }
    try {
      if (pending) pending->abort(result(FollowPath::Result::UNKNOWN, "the MPC node is shutting down"));
    } catch (const std::exception &) {
    }
    stop_goal("the MPC node is shutting down");
  }

  void run_cycle()
  {
    const auto cycle_start = Steady::now();
    take_goal();
    if (!active_) return;
    if (active_->is_canceling()) {
      end_goal(End::Canceled, FollowPath::Result::NONE, "canceled");
      return;
    }

    const auto state_time = Steady::now();
    const Measured state = measure();
    if (!state.complete) {
      end_goal(End::Aborted, state.tf_problem ? FollowPath::Result::TF_ERROR : FollowPath::Result::NO_VALID_CONTROL,
               "no complete state: " + state.missing);
      return;
    }
    if (!state.fresh) {
      stale_ = state.stale;  // the reason if the controller fails while braking
      stale_tf_ = state.tf_problem;
    }
    update_movers();
    const mmm::State x = arm_reference(state);
    const rclcpp::Time stamp = now();
    const double t = stamp.seconds();
    const mmm::CycleOutput & out = controller_->step(
      t, x, state.fresh, world_, mmm::base_at_rest(state.v, state.omega, limits_.breakaway));
    trace_arm(t, state, out.tightness);

    // The sent command is authoritative for the base.
    cmd_.linear.x = out.command(0);
    cmd_.angular.z = out.command(1);
    cmd_pub_->publish(cmd_);
    const double state_to_command = seconds(Steady::now() - state_time);

    switch (out.status) {
      case mmm::CycleStatus::Ok:
      case mmm::CycleStatus::Fallback:
        stream(out.plan, state_to_command);  // stage 0 is the state measured this cycle
        break;
      case mmm::CycleStatus::Braking:
      case mmm::CycleStatus::StaleState:
      case mmm::CycleStatus::Holding:  // one braking hold per hold; the next stream resumes tracking
        if (!arm_held_) hold_arm(state);
        break;
      case mmm::CycleStatus::Failed:
      case mmm::CycleStatus::GoalReached:
        break;  // the goal ends below
    }
    publish_plan(out.plan, stamp);

    // Goal outcome.
    bool ended = true;
    if (out.status == mmm::CycleStatus::GoalReached) {
      end_goal(End::Succeeded, FollowPath::Result::NONE, "goal reached", &state);
    } else if (out.status == mmm::CycleStatus::Failed) {
      if (!stale_.empty()) {
        end_goal(End::Aborted, stale_tf_ ? FollowPath::Result::TF_ERROR : FollowPath::Result::NO_VALID_CONTROL,
                 "stale state (" + stale_ + "): braked and stopped", &state);
      } else {
        end_goal(End::Aborted, FollowPath::Result::NO_VALID_CONTROL,
                 "no valid control (solver failures or missed deadlines): braked and stopped", &state);
      }
    } else if (no_progress(state.x, t)) {
      end_goal(End::Aborted, FollowPath::Result::FAILED_TO_MAKE_PROGRESS,
               "no progress: less than 0.3 m and 0.5 rad in 10 s", &state);
    } else {
      ended = false;
      feedback_->distance_to_goal = static_cast<float>(
        route_->length() - route_->project_range(state.x.head<2>(), 0.0, route_->length()).s);
      feedback_->speed = static_cast<float>(state.v);
      active_->publish_feedback(feedback_);
    }

    const double total = seconds(Steady::now() - cycle_start);
    const bool missed = total > period_s_;
    if (missed && !ended) controller_->note_failure();
    record(out, total, state_to_command, missed, stamp);
  }

  // One arm_trace sample per kArmTraceS (node clock) while a goal is active: the measured joints,
  // the panel's orientation in base_footprint at them and the tightness. Within the reserve (an
  // hour) it does not allocate.
  void trace_arm(double t, const Measured & state, double tightness)
  {
    if (last_trace_ && t - *last_trace_ < kArmTraceS - 1e-6 && t >= *last_trace_) return;
    last_trace_ = t;
    const Joints q = state.x.tail<6>();
    arm_trace_.push_back({t, q, mmm::panel_rpy_in_base(q), tightness});
  }

  // A pending goal becomes the active one; it preempts the active goal in place.
  void take_goal()
  {
    std::shared_ptr<GoalHandle> next;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      next = std::move(pending_);
    }
    if (!next) return;
    const bool preempt = static_cast<bool>(active_);
    if (preempt) {
      active_->abort(result(FollowPath::Result::NONE, "preempted by a new goal"));
      RCLCPP_INFO(get_logger(), "FollowPath preempted by a new goal");
    }
    active_ = std::move(next);
    std::string error;
    if (!start_goal(*active_->get_goal(), preempt, error)) {
      end_goal(End::Aborted, FollowPath::Result::INVALID_PATH, error);
    }
  }

  // A fresh goal starts the controller; a preempting one (ruling R22: the behaviour tree resends
  // FollowPath on every replan) only swaps route, goal and references, so the no-progress window,
  // the failures in a row, the tightness and the warm plan carry over.
  bool start_goal(const FollowPath::Goal & goal, bool preempt, std::string & error)
  {
    const auto & path = goal.path;
    if (!path.header.frame_id.empty() && path.header.frame_id != "map") {
      error = "the path must be in map, not " + path.header.frame_id;
      return false;
    }
    if (path.poses.empty()) {
      error = "empty path";
      return false;
    }
    std::vector<Eigen::Vector2d> points;
    points.reserve(path.poses.size() + 1);
    for (const auto & pose : path.poses) points.emplace_back(pose.pose.position.x, pose.pose.position.y);
    const auto & last = path.poses.back().pose;
    const mmn::Pose2 goal_pose{last.position.x, last.position.y, yaw_of(last.orientation)};
    try {
      // A path of one point (the robot is about at the goal): from the robot to it.
      if (std::all_of(points.begin(), points.end(), [&](const Eigen::Vector2d & p) { return (p - points[0]).norm() < 1e-6; })) {
        const Measured state = measure();
        if (state.complete) points.insert(points.begin(), state.x.head<2>());
      }
      route_.emplace(std::move(points));
      std::vector<mmm::PanelReference> refs;
      if (!anchored_.empty()) refs = mmm::anchor_on_remaining_route(anchored_, *route_);
      const size_t ref_count = refs.size();
      if (preempt) {
        controller_->update_goal(*route_, goal_pose, std::move(refs));
      } else {
        controller_->start(*route_, goal_pose, std::move(refs), posture_);
      }
      RCLCPP_INFO(get_logger(), "FollowPath%s: %zu poses, %.2f m to (%.2f, %.2f, %.2f), %zu panel references",
                  preempt ? " (in place)" : "", path.poses.size(), route_->length(), goal_pose[0], goal_pose[1],
                  goal_pose[2], ref_count);
    } catch (const std::exception & e) {
      error = std::string("invalid path: ") + e.what();
      return false;
    }
    if (preempt) return true;
    progress_time_.reset();
    arm_held_ = false;
    goal_streamed_ = false;
    stale_.clear();
    stale_tf_ = false;
    return true;
  }

  // Ends the active goal: zero base command (the controller's band restarts from it), the arm
  // held where it is if this goal moved it, and the result.
  void end_goal(End how, uint16_t code, const std::string & message, const Measured * state = nullptr)
  {
    cmd_.linear.x = 0.0;
    cmd_.angular.z = 0.0;
    cmd_pub_->publish(cmd_);
    controller_->reset_command();
    if (goal_streamed_ && !arm_held_) {
      if (state) {
        hold_arm(*state);
      } else {
        const Measured now_state = measure();
        if (now_state.complete) hold_arm(now_state);
      }
    }
    const auto r = result(code, message);
    switch (how) {
      case End::Succeeded:
        active_->succeed(r);
        RCLCPP_INFO(get_logger(), "FollowPath succeeded: %s", message.c_str());
        break;
      case End::Aborted:
        active_->abort(r);
        RCLCPP_WARN(get_logger(), "FollowPath failed (error %u): %s", code, message.c_str());
        break;
      case End::Canceled:
        active_->canceled(r);
        RCLCPP_INFO(get_logger(), "FollowPath canceled");
        break;
    }
    active_.reset();
  }

  static std::shared_ptr<FollowPath::Result> result(uint16_t code, const std::string & message)
  {
    auto r = std::make_shared<FollowPath::Result>();
    r->error_code = code;
    r->error_msg = message;
    return r;
  }

  // The measured state: TF pose, arm joints, odometry, with their completeness and freshness.
  Measured measure()
  {
    Measured m;
    const double steady = seconds(Steady::now());
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      m.x.tail<6>() = joints_;
      m.v = odom_v_;
      m.omega = odom_omega_;
      m.complete = true;
      m.fresh = true;
      for (size_t j = 0; j < joint_names_.size(); ++j) {
        if (!std::isfinite(joint_received_[j])) {
          m.complete = false;
          m.missing += (m.missing.empty() ? "" : ", ") + joint_names_[j];
        } else if (steady - joint_received_[j] > kStaleS) {
          m.fresh = false;
          m.stale += (m.stale.empty() ? "joint states " : ", ") + joint_names_[j];
        }
      }
      if (!std::isfinite(odom_received_)) {
        m.complete = false;
        m.missing += (m.missing.empty() ? "" : ", ") + std::string("odometry");
      } else if (steady - odom_received_ > kStaleS) {
        m.fresh = false;
        m.stale += (m.stale.empty() ? "" : "; ") + std::string("odometry");
      }
    }
    try {
      if (tf_buffer_.canTransform("map", "base_footprint", tf2::TimePointZero)) {
        const auto t = tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);
        pose_ = {t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)};
        pose_stamp_ = rclcpp::Time(t.header.stamp).seconds();
      }
    } catch (const tf2::TransformException &) {
      // the last pose stands in, and it ages
    }
    if (!pose_) {
      m.complete = false;
      m.tf_problem = true;
      m.missing += (m.missing.empty() ? "" : ", ") + std::string("TF map -> base_footprint");
    } else {
      m.x(0) = (*pose_)[0];
      m.x(1) = (*pose_)[1];
      m.x(2) = (*pose_)[2];
      if (now().seconds() - pose_stamp_ > kStaleS) {
        m.fresh = false;
        m.tf_problem = true;
        m.stale += (m.stale.empty() ? "" : "; ") + std::string("TF map -> base_footprint");
      }
    }
    return m;
  }

  void update_movers()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!movers_new_) return;
    for (size_t i = 0; i < movers_.size(); ++i) world_.update_mover(movers_[i].name, mover_positions_[i], movers_[i].size, mover_stamp_);
    movers_new_ = false;
  }

  // No progress: the base moved less than kProgressM and turned less than kProgressRad within
  // kProgressWindowS (node clock).
  bool no_progress(const mmm::State & x, double t)
  {
    if (!progress_time_ || (x.head<2>() - progress_pose_.head<2>()).norm() >= kProgressM ||
        std::abs(wrap(x(2) - progress_pose_(2))) >= kProgressRad || t < *progress_time_) {
      progress_pose_ = x.head<3>();
      progress_time_ = t;
      return false;
    }
    return t - *progress_time_ > kProgressWindowS;
  }

  // ---- outputs ----

  void allocate_messages()
  {
    const auto n = static_cast<size_t>(horizon_);
    trajectory_.joint_names.assign(joint_names_.begin(), joint_names_.end());
    trajectory_.points.resize(n + 2);  // the reference now, stages 1..N and the braking tail
    for (auto & p : trajectory_.points) {
      p.positions.resize(6);
      p.velocities.resize(6);
    }
    hold_.joint_names = trajectory_.joint_names;
    hold_.points.resize(2);  // the reference now, then the braking target at rest
    for (auto & p : hold_.points) {
      p.positions.resize(6);
      p.velocities.assign(6, 0.0);
    }
    plan_.header.frame_id = "map";
    plan_.joint_names = trajectory_.joint_names;
    plan_.time_from_start.resize(n + 1);
    for (size_t k = 0; k <= n; ++k) plan_.time_from_start[k] = static_cast<double>(k) * dt_;
    plan_.base.resize(n + 1);
    plan_.positions.resize((n + 1) * 6);
    cycle_msg_.status.reserve(16);
    feedback_ = std::make_shared<FollowPath::Feedback>();
    for (auto * v : {&stats_.linearize, &stats_.collision, &stats_.setup, &stats_.solve, &stats_.total,
                     &stats_.state_to_command, &stats_.iterations, &stats_.arm_offset}) {
      v->reserve(static_cast<size_t>(mmm::cycles_in(kStatsReserveS, rate_hz_)));
    }
    arm_trace_.reserve(static_cast<size_t>(mmm::cycles_in(kStatsReserveS, 1.0 / kArmTraceS)));
  }

  // The arm reference (ruling R24): the last trajectory sent, sampled now. The MPC's arm state is
  // that reference while every joint is within kReanchorRad of the measured joints, or while the
  // joint states are stale (the measured joints are then old); otherwise the measured joints (a
  // re-anchor), and the stream then starts there. Before anything was sent: the measured joints at
  // rest. Sets reference_v_ (the velocity a re-anchored stream starts with), reanchored_ and
  // reference_offset_ (largest |reference - measured|) for this cycle.
  mmm::State arm_reference(const Measured & state)
  {
    mmm::State x = state.x;
    reanchored_ = false;
    reference_offset_ = 0.0;
    reference_v_.setZero();
    if (!sent_) return x;
    Joints q, v;
    sample(*sent_, seconds(Steady::now() - sent_at_), q, v);
    reference_offset_ = (q - state.x.tail<6>()).cwiseAbs().maxCoeff();
    reference_v_ = v;
    const bool joints_stale = state.stale.find("joint") != std::string::npos;
    if (reference_offset_ <= kReanchorRad || joints_stale) {
      x.tail<6>() = q;
      return x;
    }
    reanchored_ = true;
    ++arm_reanchors_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Arm reference %.3f rad from the measured joints (limit %.2f): re-anchored on the measured "
                         "joints (%zu re-anchors)", reference_offset_, kReanchorRad, arm_reanchors_);
    return x;
  }

  // The plan for the JTC (ruling R24): point 0 at time 0 on the reference now (the plan's stage 0
  // is the reference at the state time, so point 0 is the reference sampled at publishing, or
  // after a re-anchor the measured joints advanced by `offset` at the reference velocity), then
  // stage k at k dt after the plan's stage 0 less `offset` (the state-to-command latency); stages
  // already due are skipped. Knot velocities are the mean of the adjacent stages' joint
  // velocities, then a braking tail that ends at rest: constant deceleration from the last
  // stage's velocity, at least kTailS and long enough for every joint's acceleration limit.
  void stream(const mmm::Plan & plan, double offset)
  {
    const int n = horizon_;
    Joints q0, v0;
    if (sent_ && !reanchored_) {
      sample(*sent_, seconds(Steady::now() - sent_at_), q0, v0);  // before trajectory_ is overwritten
    } else {
      v0 = reference_v_;
      q0 = plan.x[0].tail<6>() + offset * v0;
    }
    int first = 1;
    while (first < n && first * dt_ - offset <= kMinLeadS) ++first;
    const auto count = static_cast<size_t>(n - first + 3);
    if (trajectory_.points.size() != count) {
      // Shrinking keeps the buffers; growing back (after a skipped stage) allocates the new points.
      trajectory_.points.resize(count);
      for (auto & p : trajectory_.points) {
        p.positions.resize(6);
        p.velocities.resize(6);
      }
    }
    auto & start = trajectory_.points.front();
    for (int j = 0; j < 6; ++j) {
      start.positions[static_cast<size_t>(j)] = q0(j);
      start.velocities[static_cast<size_t>(j)] = v0(j);
    }
    start.time_from_start = rclcpp::Duration::from_seconds(0.0);
    for (int k = first; k <= n; ++k) {
      auto & point = trajectory_.points[static_cast<size_t>(k - first + 1)];
      const auto velocity = k < n ? Joints(0.5 * (plan.u[k - 1].tail<6>() + plan.u[k].tail<6>()))
                                  : Joints(plan.u[n - 1].tail<6>());
      for (int j = 0; j < 6; ++j) {
        point.positions[static_cast<size_t>(j)] = plan.x[static_cast<size_t>(k)](3 + j);
        point.velocities[static_cast<size_t>(j)] = velocity(j);
      }
      point.time_from_start = rclcpp::Duration::from_seconds(std::max(k * dt_ - offset, kMinLeadS));
    }
    const Joints v = plan.u[n - 1].tail<6>();
    const double tail = std::max(kTailS, (v.cwiseAbs().array() / limits_.qdd_max.array()).maxCoeff());
    const Joints end = (plan.x[static_cast<size_t>(n)].tail<6>() + 0.5 * tail * v).cwiseMax(limits_.q_min).cwiseMin(limits_.q_max);
    auto & last = trajectory_.points.back();
    for (int j = 0; j < 6; ++j) {
      last.positions[static_cast<size_t>(j)] = end(j);
      last.velocities[static_cast<size_t>(j)] = 0.0;
    }
    last.time_from_start = rclcpp::Duration::from_seconds(std::max(n * dt_ - offset, kMinLeadS) + tail);
    trajectory_pub_->publish(trajectory_);
    sent_ = &trajectory_;
    sent_at_ = Steady::now();
    goal_streamed_ = true;
    arm_held_ = false;
  }

  // Stops the arm within its acceleration limit (rulings R21, R24): point 0 at time 0 on the
  // reference now (q, v), then zero velocity at the braking target q + v T / 2 after
  // T = max(kHoldMinS, max_j |v_j| / qdd_max_j) (one T, so the joints stay synchronized), clamped
  // into the joint bounds: a constant deceleration v / T. The reference is the last trajectory
  // sent, sampled now; the measured joints (keeping the reference velocity) when it is farther
  // than kReanchorRad from them while they are fresh; the measured joints at rest when nothing
  // was sent. Never commands from missing data (the caller has a complete state).
  void hold_arm(const Measured & state)
  {
    Joints q = state.x.tail<6>(), v = Joints::Zero();
    if (sent_) {
      Joints q_ref;
      sample(*sent_, seconds(Steady::now() - sent_at_), q_ref, v);
      const bool joints_stale = state.stale.find("joint") != std::string::npos;
      if ((q_ref - q).cwiseAbs().maxCoeff() <= kReanchorRad || joints_stale) q = q_ref;
    }
    const double duration = std::max(kHoldMinS, (v.cwiseAbs().array() / limits_.qdd_max.array()).maxCoeff());
    const Joints target = (q + 0.5 * duration * v).cwiseMax(limits_.q_min).cwiseMin(limits_.q_max);
    for (int j = 0; j < 6; ++j) {
      hold_.points[0].positions[static_cast<size_t>(j)] = q(j);
      hold_.points[0].velocities[static_cast<size_t>(j)] = v(j);
      hold_.points[1].positions[static_cast<size_t>(j)] = target(j);
    }
    hold_.points[0].time_from_start = rclcpp::Duration::from_seconds(0.0);
    hold_.points[1].time_from_start = rclcpp::Duration::from_seconds(duration);
    trajectory_pub_->publish(hold_);
    sent_ = &hold_;
    sent_at_ = Steady::now();
    arm_held_ = true;
  }

  // Position and velocity of a sent trajectory `t` seconds after it was sent, as the JTC samples
  // it from positions and velocities: a cubic Hermite spline between points; the first point
  // before it (a first point at time 0 is never before), at rest at the last point after it.
  static void sample(const trajectory_msgs::msg::JointTrajectory & trajectory, double t, Joints & q, Joints & v)
  {
    const auto & points = trajectory.points;
    const auto time = [](const trajectory_msgs::msg::JointTrajectoryPoint & p) {
      return rclcpp::Duration(p.time_from_start).seconds();
    };
    const auto at = [](const std::vector<double> & values) {
      return Eigen::Map<const Joints>(values.data());
    };
    if (t <= time(points.front())) {
      q = at(points.front().positions);
      v = at(points.front().velocities);
      return;
    }
    size_t i = 0;
    while (i + 1 < points.size() && time(points[i + 1]) < t) ++i;
    if (i + 1 >= points.size()) {
      q = at(points.back().positions);
      v = Joints::Zero();
      return;
    }
    const double t0 = time(points[i]), h = time(points[i + 1]) - t0;
    if (!(h > 0.0)) {
      q = at(points[i + 1].positions);
      v = at(points[i + 1].velocities);
      return;
    }
    const double s = std::clamp((t - t0) / h, 0.0, 1.0), s2 = s * s, s3 = s2 * s;
    const Joints p0 = at(points[i].positions), p1 = at(points[i + 1].positions);
    const Joints m0 = h * at(points[i].velocities), m1 = h * at(points[i + 1].velocities);
    q = (2 * s3 - 3 * s2 + 1) * p0 + (s3 - 2 * s2 + s) * m0 + (-2 * s3 + 3 * s2) * p1 + (s3 - s2) * m1;
    v = ((6 * s2 - 6 * s) * p0 + (3 * s2 - 4 * s + 1) * m0 + (-6 * s2 + 6 * s) * p1 + (3 * s2 - 2 * s) * m1) / h;
  }

  void publish_plan(const mmm::Plan & plan, const rclcpp::Time & stamp)
  {
    plan_.header.stamp = stamp;
    for (size_t k = 0; k < plan.x.size() && k < plan_.base.size(); ++k) {
      const auto & x = plan.x[k];
      plan_.base[k].x = x(0);
      plan_.base[k].y = x(1);
      plan_.base[k].theta = x(2);
      for (int j = 0; j < 6; ++j) plan_.positions[k * 6 + static_cast<size_t>(j)] = x(3 + j);
    }
    plan_pub_->publish(plan_);
  }

  void record(const mmm::CycleOutput & out, double total, double state_to_command, bool missed,
              const rclcpp::Time & stamp)
  {
    auto & c = cycle_msg_;
    c.header.stamp = stamp;
    c.status = mmm::to_string(out.status);
    c.linearize_s = out.timing.linearize_s;
    c.collision_s = out.timing.collision_s;
    c.setup_s = out.timing.setup_s;
    c.solve_s = out.timing.solve_s;
    c.total_s = total;
    c.state_to_command_s = state_to_command;
    c.iterations = out.iterations;
    c.active_pairs = out.collision.active_pairs;
    c.dropped_pairs = out.collision.dropped_pairs;
    c.max_slack = out.max_slack;
    c.gate_slack = out.gate_slack;
    c.gate_violation = out.gate_violation;
    c.manipulability = out.manipulability;
    c.tightness = out.tightness;
    c.deadline_missed = missed;
    c.arm_reanchored = reanchored_;
    c.arm_reference_offset = reference_offset_;
    cycle_pub_->publish(c);

    stats_.linearize.push_back(c.linearize_s);
    stats_.collision.push_back(c.collision_s);
    stats_.setup.push_back(c.setup_s);
    stats_.solve.push_back(c.solve_s);
    stats_.total.push_back(total);
    stats_.state_to_command.push_back(state_to_command);
    stats_.iterations.push_back(out.iterations);
    stats_.arm_offset.push_back(reference_offset_);
    ++status_counts_[static_cast<size_t>(out.status)];
    const bool holding = out.status == mmm::CycleStatus::Holding;
    if (holding && !holding_) {
      ++holding_events_;
      RCLCPP_WARN(get_logger(), "Holding: the plan's slack %.3f m exceeds its pair's current violation %.3f m by more "
                  "than %.3f m (hold %zu)", out.gate_slack, out.gate_violation, config_.slack_gate.tolerance_m,
                  holding_events_);
    } else if (!holding && holding_ && out.status == mmm::CycleStatus::Ok) {
      RCLCPP_INFO(get_logger(), "Hold %zu ended: tracking resumed", holding_events_);
    }
    holding_ = holding;
    const double excess = out.gate_slack - out.gate_violation;
    if (excess > config_.slack_gate.tolerance_m) ++gate_unsafe_cycles_;
    gate_max_excess_ = std::max(gate_max_excess_, excess);
    if (missed) {
      ++deadline_misses_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "MPC cycle took %.1f ms (period %.1f ms)", 1e3 * total,
                           1e3 * period_s_);
    }
    if (total > kNearMissFraction * period_s_) ++near_misses_;
  }

  // Parameters and the controller (the loop's own).
  double rate_hz_ = 20.0, period_s_ = 0.05;
  int fallback_cycles_ = mmm::kDefaultFallbackCycles;  // fallback_s of wholebody_mpc.yaml at rate_hz
  std::string stats_file_;
  std::unique_ptr<mmm::Controller> controller_;
  mmm::Limits limits_{};
  int horizon_ = 0;
  double dt_ = 0.1;
  mmm::PostureTargets posture_{};
  mmm::MpcConfig config_{};  // as used: wholebody_mpc.yaml with the parameter overrides
  std::array<std::string, 6> joint_names_;
  mmm::World world_;
  std::vector<Mover> movers_;
  std::vector<mmm::AnchoredReference> anchored_;

  // Inputs (state_mutex_), written by the io callbacks.
  std::mutex state_mutex_;
  Joints joints_ = Joints::Zero();
  std::array<double, 6> joint_received_{NAN, NAN, NAN, NAN, NAN, NAN};
  double odom_v_ = 0.0, odom_omega_ = 0.0, odom_received_ = NAN;
  std::vector<Eigen::Vector2d> mover_positions_;
  double mover_stamp_ = 0.0;
  bool movers_new_ = false;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::optional<std::array<double, 3>> pose_;  // the last transform (loop only)
  double pose_stamp_ = 0.0;

  // Goals: pending_ (goal_mutex_) from the action server, active_ in the loop.
  std::mutex goal_mutex_;
  std::shared_ptr<GoalHandle> pending_, active_;
  std::optional<mmm::Route> route_;
  std::optional<double> progress_time_;
  Eigen::Vector3d progress_pose_ = Eigen::Vector3d::Zero();
  bool goal_streamed_ = false, arm_held_ = false, stale_tf_ = false;
  std::string stale_;

  // Outgoing messages, allocated once.
  geometry_msgs::msg::Twist cmd_;
  trajectory_msgs::msg::JointTrajectory trajectory_, hold_;
  // The last trajectory sent (trajectory_ or hold_; null before the first) and when: the JTC's
  // reference (ruling R24).
  const trajectory_msgs::msg::JointTrajectory * sent_ = nullptr;
  Steady::time_point sent_at_;
  Joints reference_v_ = Joints::Zero();  // this cycle's reference velocity
  bool reanchored_ = false;
  double reference_offset_ = 0.0;
  mobile_manipulator_interfaces::msg::WholeBodyPlan plan_;
  mobile_manipulator_interfaces::msg::MpcCycle cycle_msg_;
  std::shared_ptr<FollowPath::Feedback> feedback_;

  // Statistics.
  struct
  {
    std::vector<double> linearize, collision, setup, solve, total, state_to_command, iterations, arm_offset;
  } stats_;
  std::array<size_t, kStatuses> status_counts_{};
  struct ArmSample
  {
    double t;
    Joints q;
    Eigen::Vector3d panel_rpy;
    double tightness;
  };
  std::vector<ArmSample> arm_trace_;
  std::optional<double> last_trace_;  // node time of the last arm_trace sample
  size_t deadline_misses_ = 0, near_misses_ = 0, arm_reanchors_ = 0;
  size_t holding_events_ = 0, gate_unsafe_cycles_ = 0;  // slack gate
  double gate_max_excess_ = 0.0;  // largest gate_slack - gate_violation of a cycle (m)
  bool holding_ = false;          // the last cycle held

  rclcpp::CallbackGroup::SharedPtr io_group_, loop_group_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<mobile_manipulator_interfaces::msg::WholeBodyPlan>::SharedPtr plan_pub_;
  rclcpp::Publisher<mobile_manipulator_interfaces::msg::MpcCycle>::SharedPtr cycle_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr movers_sub_;
  rclcpp_action::Server<FollowPath>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::PreShutdownCallbackHandle shutdown_callback_;
  std::mutex loop_mutex_;  // the loop's state: held by each cycle and by the pre-shutdown callback
  bool shutting_down_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<WholeBodyMpcNode> node;
  try {
    node = std::make_shared<WholeBodyMpcNode>();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("wholebody_mpc"), "Cannot start: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  // Two threads: the control loop and the inputs/action server never wait for each other.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  node->write_stats();
  rclcpp::shutdown();
  return 0;
}
