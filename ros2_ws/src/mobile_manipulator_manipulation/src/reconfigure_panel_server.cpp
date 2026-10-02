// ReconfigurePanel action server (baseline B3; spec section 4).
//
// With the base stopped, plans a collision-free arm motion with move_group to a panel pose
// or a named SRDF state, checks that the planned final state's robot + panel ground
// projection fits the requested footprint profile, executes through the arm JTC, repeats
// the check on the measured state, and publishes the profile to both Nav2 costmaps.
// Never publishes /cmd_vel or sends Nav2 goals.
//
// Parameters: payload_file, profiles_file, initial_footprint_profile (the profile Nav2 was
// launched with). Timing is node time (simulation time with Unity); deadlines are wall
// time so a stalled move_group or controller can never hang a mission.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/list_hardware_components.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <yaml-cpp/yaml.h>

#include "mobile_manipulator_interfaces/action/reconfigure_panel.hpp"
#include "mobile_manipulator_manipulation/reconfigure_logic.hpp"
#include "mobile_manipulator_navigation/footprint_projection.hpp"
#include "mobile_manipulator_navigation/mission.hpp"
#include "mobile_manipulator_navigation/scenario_spec.hpp"

namespace mmm = mobile_manipulator_manipulation;
namespace mmn = mobile_manipulator_navigation;
using Reconfigure = mobile_manipulator_interfaces::action::ReconfigurePanel;
using GoalHandle = rclcpp_action::ServerGoalHandle<Reconfigure>;
using MoveGroup = moveit_msgs::action::MoveGroup;
using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
using Result = Reconfigure::Result;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

namespace
{
const std::vector<std::string> kArmJoints = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
constexpr char kHardware[] = "UnityArm";

std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot read " + path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

double seconds(const builtin_interfaces::msg::Duration & d) { return d.sec + d.nanosec * 1e-9; }

// The request was canceled or a result is ready; thrown to unwind the goal thread.
struct Finished
{
  uint8_t code;
  std::string message;
};
}  // namespace

class ReconfigurePanelServer : public rclcpp::Node
{
public:
  ReconfigurePanelServer()
  : Node("reconfigure_panel_server")
  {
    const auto share = [](const std::string & p) { return ament_index_cpp::get_package_share_directory(p); };
    const auto payload_file = declare_parameter("payload_file", share("mobile_manipulator_control") +
                                                "/config/qualified_payload.json");
    const auto navigation = share("mobile_manipulator_navigation");
    current_profile_ = declare_parameter("initial_footprint_profile", std::string("home"));

    const auto urdf_text = read_file(share("mobile_manipulator_description") + "/urdf/mobile_manipulator.urdf");
    projector_ = std::make_unique<mmn::FootprintProjector>(
      urdf_text, mmn::payload_from_json(mmn::Json::parse(read_file(payload_file))));
    const mmn::ScenarioConfig navigation_config(navigation);
    zones_ = mmn::monitor_zones(navigation_config.load("nav2_navigation.yaml"));
    const auto profiles = navigation_config.load("footprint_profiles.yaml");
    for (const auto & item : profiles.at("profiles").items()) {
      profiles_[item.key()] = mmn::polygon_of(item.value()["polygon"]);
    }
    if (!profiles_.count(current_profile_)) throw std::invalid_argument("unknown initial profile " + current_profile_);
    const auto urdf_model = urdf::parseURDF(urdf_text);
    auto srdf_model = std::make_shared<srdf::Model>();
    srdf_model->initString(*urdf_model, read_file(share("mobile_manipulator_moveit_config") +
                                                  "/config/mobile_manipulator.srdf"));
    robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
    // A request scaling of 0 makes time parameterization fall back to 1.0, not to the
    // configured defaults, so the configured factors are sent with every request.
    const auto limits = YAML::LoadFile(share("mobile_manipulator_moveit_config") + "/config/joint_limits.yaml");
    velocity_scaling_ = limits["default_velocity_scaling_factor"].as<double>();
    acceleration_scaling_ = limits["default_acceleration_scaling_factor"].as<double>();

    sensors_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;
    options.callback_group = sensors_;
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 20, [this](const nav_msgs::msg::Odometry & m) {
        std::lock_guard<std::mutex> lock(mutex_);
        base_.add(rclcpp::Time(m.header.stamp).seconds(), std::hypot(m.twist.twist.linear.x, m.twist.twist.linear.y),
                  m.twist.twist.angular.z);
      }, options);
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 50, [this](const sensor_msgs::msg::JointState & m) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < m.name.size() && i < m.position.size(); ++i) joints_[m.name[i]] = m.position[i];
        joint_stamp_ = m.header.stamp;
        if (hold_target_) {
          for (size_t i = 0; i < kArmJoints.size(); ++i) {
            if (joints_.count(kArmJoints[i])) {
              hold_error_ = std::max(hold_error_, std::abs(joints_[kArmJoints[i]] - (*hold_target_)[i]));
            }
          }
        }
      }, options);
    state_sub_ = create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "/arm_controller/controller_state", 20, [this](const control_msgs::msg::JointTrajectoryControllerState & m) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!tracking_) return;
        for (const double e : m.error.positions) path_error_ = std::max(path_error_, std::abs(e));
      }, options);

    stop_pub_ = create_publisher<std_msgs::msg::String>("/trajectory_execution_event", 10);
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    footprint_pubs_ = {create_publisher<geometry_msgs::msg::Polygon>("/global_costmap/footprint", latched),
                       create_publisher<geometry_msgs::msg::Polygon>("/local_costmap/footprint", latched)};
    // The collision monitor's stop and slowdown zones are sized from the profile too; with
    // dynamic_monitor_zones:=true they follow these (latched) topics.
    for (const auto & zone : zones_) {
      zone_pubs_.push_back(create_publisher<geometry_msgs::msg::PolygonStamped>(zone.polygon_topic, latched));
    }
    publish_zones(current_profile_);

    // One mutually exclusive group per action client: in a reentrant group two executor
    // threads can service the same action client at once and drop its responses, which
    // hung goals (cancel and execution result never delivered).
    clients_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    move_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    execute_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    move_client_ = rclcpp_action::create_client<MoveGroup>(this, "/move_action", move_group_);
    execute_client_ = rclcpp_action::create_client<ExecuteTrajectory>(this, "/execute_trajectory", execute_group_);
    controllers_client_ = create_client<controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers", rclcpp::ServicesQoS(), clients_);
    hardware_client_ = create_client<controller_manager_msgs::srv::ListHardwareComponents>(
      "/controller_manager/list_hardware_components", rclcpp::ServicesQoS(), clients_);
    scene_client_ = create_client<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene", rclcpp::ServicesQoS(), clients_);

    server_ = rclcpp_action::create_server<Reconfigure>(
      this, "/reconfigure_panel",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Reconfigure::Goal> goal) {
        if (busy_.exchange(true)) {
          RCLCPP_WARN(get_logger(), "Rejected a goal: a reconfiguration is already running");
          return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(get_logger(), "Goal: %s -> profile '%s'",
                    goal->target_type == Reconfigure::Goal::NAMED_STATE ? goal->named_state.c_str() : "panel pose",
                    goal->footprint_profile.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const std::shared_ptr<GoalHandle> handle) {
        std::thread([this, handle] { run(handle); busy_ = false; }).detach();
      });
    RCLCPP_INFO(get_logger(), "ReconfigurePanel ready; active footprint profile '%s'", current_profile_.c_str());
  }

private:
  void phase(const std::shared_ptr<GoalHandle> & handle, uint8_t value)
  {
    auto feedback = std::make_shared<Reconfigure::Feedback>();
    feedback->phase = value;
    handle->publish_feedback(feedback);
  }

  void check_cancel(const std::shared_ptr<GoalHandle> & handle)
  {
    if (handle->is_canceling()) throw Finished{Result::CANCELED, "canceled"};
  }

  // Waits for a future, polling for cancellation; nullopt on deadline.
  template<typename Future>
  bool wait(const std::shared_ptr<GoalHandle> & handle, Future & future, Clock::time_point deadline,
            const std::function<void()> & on_cancel = {})
  {
    while (future.wait_for(20ms) != std::future_status::ready) {
      if (handle->is_canceling()) {
        if (on_cancel) on_cancel();
        throw Finished{Result::CANCELED, "canceled"};
      }
      if (Clock::now() > deadline) return false;
    }
    return true;
  }

  bool arm_active(const std::shared_ptr<GoalHandle> & handle)
  {
    const auto deadline = Clock::now() + 3s;
    if (!controllers_client_->wait_for_service(2s) || !hardware_client_->wait_for_service(1s)) return false;
    auto controllers = controllers_client_->async_send_request(
      std::make_shared<controller_manager_msgs::srv::ListControllers::Request>());
    auto hardware = hardware_client_->async_send_request(
      std::make_shared<controller_manager_msgs::srv::ListHardwareComponents::Request>());
    if (!wait(handle, controllers, deadline) || !wait(handle, hardware, deadline)) return false;
    // Service futures hand over sole ownership of the response: keep it alive here.
    const auto controller_list = controllers.get();
    const auto hardware_list = hardware.get();
    const auto & c = controller_list->controller;
    const bool controller = std::any_of(c.begin(), c.end(), [](const auto & s) {
      return s.name == "arm_controller" && s.state == "active"; });
    const auto & h = hardware_list->component;
    const bool component = std::any_of(h.begin(), h.end(), [](const auto & s) {
      return s.name == kHardware && s.state.label == "active"; });
    return controller && component;
  }

  mmn::JointMap arm_joints(const std::vector<std::string> & names, const std::vector<double> & positions) const
  {
    mmn::JointMap joints;
    for (size_t i = 0; i < names.size() && i < positions.size(); ++i) joints[names[i]] = positions[i];
    return joints;
  }

  std::pair<mmn::JointMap, builtin_interfaces::msg::Time> measured()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mmn::JointMap joints;
    for (const auto & name : kArmJoints) if (joints_.count(name)) joints[name] = joints_.at(name);
    return {joints, joint_stamp_};
  }

  // Minimum distance between the robot (incl. panel) and the world over the trajectory.
  double min_clearance(const std::shared_ptr<GoalHandle> & handle, const moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    using Components = moveit_msgs::msg::PlanningSceneComponents;
    request->components.components = Components::SCENE_SETTINGS | Components::ROBOT_STATE |
      Components::ROBOT_STATE_ATTACHED_OBJECTS | Components::WORLD_OBJECT_NAMES |
      Components::WORLD_OBJECT_GEOMETRY | Components::OCTOMAP | Components::TRANSFORMS |
      Components::ALLOWED_COLLISION_MATRIX;
    auto future = scene_client_->async_send_request(request);
    if (!wait(handle, future, Clock::now() + 5s)) return std::nan("");
    planning_scene::PlanningScene scene(robot_model_);
    scene.setPlanningSceneMsg(future.get()->scene);
    moveit::core::RobotState state = scene.getCurrentState();
    // Clearance to obstacles: the raised floor only bounds panel ground clearance (and the
    // arm's distance to it would dominate the minimum), so it is not counted.
    auto acm = scene.getAllowedCollisionMatrix();
    acm.setDefaultEntry("floor", true);
    double clearance = std::numeric_limits<double>::infinity();
    const auto & names = trajectory.joint_trajectory.joint_names;
    for (const auto & point : trajectory.joint_trajectory.points) {
      for (size_t i = 0; i < names.size(); ++i) state.setVariablePosition(names[i], point.positions[i]);
      state.update();
      clearance = std::min(clearance, scene.distanceToCollision(state, acm));
    }
    return clearance;
  }

  void run(const std::shared_ptr<GoalHandle> handle)
  {
    const auto goal = handle->get_goal();
    auto result = std::make_shared<Result>();
    result->applied_footprint_profile = current_profile_;
    const std::string previous = current_profile_;
    bool moved = false;
    try {
      phase(handle, Reconfigure::Feedback::CHECKING);
      if (!profiles_.count(goal->footprint_profile)) {
        throw Finished{Result::UNKNOWN_PROFILE, "unknown footprint profile '" + goal->footprint_profile + "'"};
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!base_.stopped(now().seconds())) throw Finished{Result::BASE_NOT_STOPPED, "base is not stopped"};
      }
      if (!arm_active(handle)) throw Finished{Result::ARM_NOT_ACTIVE, "arm controller or hardware not active"};

      // Plan.
      phase(handle, Reconfigure::Feedback::PLANNING);
      MoveGroup::Goal plan;
      auto & request = plan.request;
      request.group_name = "arm";
      request.pipeline_id = "ompl";
      request.planner_id = "RRTConnectkConfigDefault";
      request.num_planning_attempts = std::max(1, goal->planning_attempts);
      request.allowed_planning_time = goal->planning_time_s;
      request.start_state.is_diff = true;
      request.max_velocity_scaling_factor = velocity_scaling_;
      request.max_acceleration_scaling_factor = acceleration_scaling_;
      if (goal->target_type == Reconfigure::Goal::PANEL_POSE) {
        if (goal->panel_pose.header.frame_id != "base_footprint") {
          throw Finished{Result::PLANNING_FAILED, "panel_pose must be given in base_footprint"};
        }
        request.goal_constraints.push_back(
          mmm::panel_goal_constraints(goal->panel_pose, goal->position_tolerance, goal->orientation_tolerance));
      } else {
        std::map<std::string, double> named;
        if (!robot_model_->getJointModelGroup("arm")->getVariableDefaultPositions(goal->named_state, named)) {
          throw Finished{Result::PLANNING_FAILED, "unknown named state '" + goal->named_state + "'"};
        }
        moveit_msgs::msg::Constraints joints;
        for (const auto & [name, value] : named) {
          moveit_msgs::msg::JointConstraint c;
          c.joint_name = name;
          c.position = value;
          c.tolerance_above = c.tolerance_below = 1e-3;
          c.weight = 1.0;
          joints.joint_constraints.push_back(c);
        }
        request.goal_constraints.push_back(joints);
      }
      plan.planning_options.plan_only = true;
      const auto plan_start = now();
      const auto plan_deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(goal->planning_time_s + 5.0));
      if (!move_client_->wait_for_action_server(1s)) throw Finished{Result::PLANNING_FAILED, "move_group not available"};
      auto sent = move_client_->async_send_goal(plan);
      if (!wait(handle, sent, plan_deadline) || !sent.get()) throw Finished{Result::PLANNING_FAILED, "move_group did not accept the request"};
      const auto plan_handle = sent.get();
      auto planned = move_client_->async_get_result(plan_handle);
      // A cancel during planning lets the plan finish (nothing moves) and discards it:
      // canceling move_group's goal can race with its own completion and abort move_group
      // ("invalid transition from state EXECUTING with event CANCELED").
      if (!wait(handle, planned, plan_deadline, [&] { planned.wait_until(plan_deadline); })) {
        throw Finished{Result::PLANNING_FAILED, "planning deadline exceeded"};
      }
      result->planning_time_s = (now() - plan_start).seconds();
      const auto plan_result = planned.get().result;
      using Codes = moveit_msgs::msg::MoveItErrorCodes;
      if (!plan_result || plan_result->error_code.val != Codes::SUCCESS) {
        const int code = plan_result ? plan_result->error_code.val : 0;
        throw Finished{code == Codes::NO_IK_SOLUTION || code == Codes::GOAL_CONSTRAINTS_VIOLATED
                       ? Result::NO_IK : Result::PLANNING_FAILED,
                       "planning failed with MoveIt error " + std::to_string(code)};
      }
      const auto & trajectory = plan_result->planned_trajectory;
      const auto & points = trajectory.joint_trajectory.points;
      if (points.empty()) throw Finished{Result::PLANNING_FAILED, "empty trajectory"};
      result->trajectory_duration_s = seconds(points.back().time_from_start);
      result->joint_path_length_rad = mmm::joint_path_length(trajectory.joint_trajectory);

      // Verify the plan's final state against the requested profile before moving.
      phase(handle, Reconfigure::Feedback::VERIFYING_PLAN);
      const auto planned_final = arm_joints(trajectory.joint_trajectory.joint_names, points.back().positions);
      if (goal->target_type == Reconfigure::Goal::PANEL_POSE &&
          !mmm::panel_goal_met(projector_->panel_pose(planned_final), goal->panel_pose,
                               goal->position_tolerance, goal->orientation_tolerance)) {
        throw Finished{Result::PLANNING_FAILED, "plan does not reach the requested panel pose"};
      }
      const auto planned_fit = mmn::contains(profiles_.at(goal->footprint_profile),
                                             projector_->projected_points(planned_final));
      result->planned_containment_margin_m = planned_fit.margin_m;
      if (!planned_fit.inside) {
        std::ostringstream detail;
        detail << "planned state leaves profile '" << goal->footprint_profile << "' by "
               << -planned_fit.margin_m << " m at joints [";
        for (size_t i = 0; i < kArmJoints.size(); ++i) detail << (i ? ", " : "") << planned_final.at(kArmJoints[i]);
        detail << "]";
        throw Finished{Result::PROFILE_TOO_SMALL, detail.str()};
      }
      result->min_planned_clearance_m = min_clearance(handle, trajectory);
      check_cancel(handle);

      // Execute.
      phase(handle, Reconfigure::Feedback::EXECUTING);
      ExecuteTrajectory::Goal execute;
      execute.trajectory = trajectory;
      const auto exec_deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(result->trajectory_duration_s + 8.0));
      if (!execute_client_->wait_for_action_server(1s)) throw Finished{Result::EXECUTION_FAILED, "execute_trajectory not available"};
      {
        std::lock_guard<std::mutex> lock(mutex_);
        path_error_ = 0.0;
        tracking_ = true;
      }
      moved = true;
      const auto exec_start = now();
      auto exec_sent = execute_client_->async_send_goal(execute);
      // No cancel check here: a cancel before the handle exists would leave the arm moving.
      // Once accepted, a pending cancel is honoured through cancel_execution below.
      if (exec_sent.wait_for(5s) != std::future_status::ready || !exec_sent.get()) {
        throw Finished{Result::EXECUTION_FAILED, "execution was not accepted"};
      }
      const auto exec_handle = exec_sent.get();
      auto executed = execute_client_->async_get_result(exec_handle);
      const auto cancel_execution = [&] {
        // Canceling execute_trajectory alone let the trajectory run to its end; the
        // trajectory execution manager stops the controllers on a "stop" event.
        std_msgs::msg::String stop;
        stop.data = "stop";
        stop_pub_->publish(stop);
        auto cancel = execute_client_->async_cancel_goal(exec_handle);
        cancel.wait_for(3s);
        executed.wait_for(3s);
      };
      if (!wait(handle, executed, exec_deadline, cancel_execution)) {
        cancel_execution();
        throw Finished{Result::EXECUTION_FAILED, "execution deadline exceeded"};
      }
      result->execution_time_s = (now() - exec_start).seconds();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        tracking_ = false;
        result->max_path_error_rad = path_error_;
      }
      const auto exec_result = executed.get().result;
      if (!exec_result || exec_result->error_code.val != Codes::SUCCESS) {
        throw Finished{arm_active(handle) ? Result::EXECUTION_FAILED : Result::ARM_FAULT,
                       "execution failed with MoveIt error " + std::to_string(exec_result ? exec_result->error_code.val : 0)};
      }

      // Verify the measured state, then hold for 1 s to measure the hold error.
      phase(handle, Reconfigure::Feedback::VERIFYING_STATE);
      const auto end_stamp = static_cast<builtin_interfaces::msg::Time>(now());
      const auto fresh_deadline = Clock::now() + 2s;
      while (!mmm::fresh_after(measured().second, end_stamp)) {
        if (Clock::now() > fresh_deadline) throw Finished{Result::EXECUTION_FAILED, "no joint state after execution"};
        std::this_thread::sleep_for(10ms);
      }
      std::vector<double> target;
      for (const auto & name : kArmJoints) target.push_back(planned_final.at(name));
      {
        std::lock_guard<std::mutex> lock(mutex_);
        hold_error_ = 0.0;
        hold_target_ = target;
      }
      const auto hold_end = now() + rclcpp::Duration::from_seconds(1.0);
      const auto hold_deadline = Clock::now() + 5s;
      while (now() < hold_end && Clock::now() < hold_deadline) std::this_thread::sleep_for(10ms);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        hold_target_.reset();
        result->hold_error_rad = hold_error_;
      }
      const auto reached = measured().first;
      const auto measured_fit = mmn::contains(profiles_.at(goal->footprint_profile),
                                              projector_->projected_points(reached));
      result->measured_containment_margin_m = measured_fit.margin_m;
      fill_reached(*result, reached);
      if (!measured_fit.inside) {
        result->profile_violated = !mmn::contains(profiles_.at(previous), projector_->projected_points(reached)).inside;
        throw Finished{Result::PROFILE_VIOLATED_AFTER_EXECUTION,
                       "measured state leaves profile '" + goal->footprint_profile + "'"};
      }

      // Switch the Nav2 footprint.
      phase(handle, Reconfigure::Feedback::SWITCHING_FOOTPRINT);
      const auto switch_start = now();
      geometry_msgs::msg::Polygon polygon;
      for (const auto & [x, y] : profiles_.at(goal->footprint_profile)) {
        geometry_msgs::msg::Point32 p;
        p.x = static_cast<float>(x);
        p.y = static_cast<float>(y);
        polygon.points.push_back(p);
      }
      for (const auto & pub : footprint_pubs_) pub->publish(polygon);
      publish_zones(goal->footprint_profile);
      current_profile_ = goal->footprint_profile;
      result->applied_footprint_profile = current_profile_;
      result->footprint_switch_time_s = (now() - switch_start).seconds();
      result->error_code = Result::SUCCESS;
      result->message = "reconfigured; footprint profile '" + current_profile_ + "'";
      RCLCPP_INFO(get_logger(), "ReconfigurePanel: %s (plan %.2f s, execute %.2f s)", result->message.c_str(),
                  result->planning_time_s, result->execution_time_s);
      handle->succeed(result);
    } catch (const Finished & finished) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        tracking_ = false;
        hold_target_.reset();
      }
      result->error_code = finished.code;
      result->message = finished.message;
      if (moved && finished.code != Result::PROFILE_VIOLATED_AFTER_EXECUTION) {
        const auto reached = measured().first;
        fill_reached(*result, reached);
        result->profile_violated = !mmn::contains(profiles_.at(previous), projector_->projected_points(reached)).inside;
      }
      RCLCPP_WARN(get_logger(), "ReconfigurePanel: %s", finished.message.c_str());
      if (finished.code == Result::CANCELED) handle->canceled(result);
      else handle->abort(result);
    } catch (const std::exception & error) {
      result->error_code = Result::EXECUTION_FAILED;
      result->message = error.what();
      RCLCPP_ERROR(get_logger(), "ReconfigurePanel: %s", error.what());
      handle->abort(result);
    }
  }

  void publish_zones(const std::string & profile)
  {
    for (size_t i = 0; i < zones_.size(); ++i) {
      geometry_msgs::msg::PolygonStamped zone;
      zone.header.frame_id = "base_footprint";
      zone.header.stamp = now();
      for (const auto & [x, y] : mmn::padded_rectangle(profiles_.at(profile), zones_[i].margin_m)) {
        geometry_msgs::msg::Point32 p;
        p.x = static_cast<float>(x);
        p.y = static_cast<float>(y);
        zone.polygon.points.push_back(p);
      }
      zone_pubs_[i]->publish(zone);
    }
  }

  void fill_reached(Result & result, const mmn::JointMap & reached) const
  {
    result.reached_joint_positions.clear();
    for (const auto & name : kArmJoints) result.reached_joint_positions.push_back(reached.count(name) ? reached.at(name) : std::nan(""));
    const auto pose = projector_->panel_pose(reached);
    const Eigen::Quaterniond q(pose.linear());
    result.reached_panel_pose.position.x = pose.translation().x();
    result.reached_panel_pose.position.y = pose.translation().y();
    result.reached_panel_pose.position.z = pose.translation().z();
    result.reached_panel_pose.orientation.x = q.x();
    result.reached_panel_pose.orientation.y = q.y();
    result.reached_panel_pose.orientation.z = q.z();
    result.reached_panel_pose.orientation.w = q.w();
  }

  std::unique_ptr<mmn::FootprintProjector> projector_;
  std::map<std::string, mmn::Polygon> profiles_;
  std::string current_profile_;
  double velocity_scaling_ = 0.5, acceleration_scaling_ = 0.5;
  moveit::core::RobotModelConstPtr robot_model_;
  std::atomic<bool> busy_{false};

  std::mutex mutex_;
  mmm::BaseMotionWindow base_;
  std::map<std::string, double> joints_;
  builtin_interfaces::msg::Time joint_stamp_;
  bool tracking_ = false;
  double path_error_ = 0.0;
  std::optional<std::vector<double>> hold_target_;
  double hold_error_ = 0.0;

  rclcpp::CallbackGroup::SharedPtr sensors_, clients_, move_group_, execute_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr state_sub_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr> footprint_pubs_;
  std::vector<mmn::MonitorZone> zones_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr> zone_pubs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stop_pub_;
  rclcpp_action::Client<MoveGroup>::SharedPtr move_client_;
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr execute_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr controllers_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListHardwareComponents>::SharedPtr hardware_client_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;
  rclcpp_action::Server<Reconfigure>::SharedPtr server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReconfigurePanelServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
