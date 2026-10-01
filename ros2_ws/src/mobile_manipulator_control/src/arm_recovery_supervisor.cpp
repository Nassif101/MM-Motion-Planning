// Re-activate the Unity arm hardware and its controllers after a stale-feedback pause.
//
// The UnityArm hardware deactivates when Unity's arm feedback pauses for more than its
// 0.5 s state timeout (ADR 0005); the controller manager then also deactivates the arm
// controllers, aborting any trajectory. Unity's own watchdog holds the arm meanwhile. Once
// fresh feedback with advancing stamps has flowed for `fresh_for_s`, this node activates the
// hardware again (which restarts commands from the actual joint positions) and then the
// controllers. A new simulation epoch stays latched in the hardware: activation is refused
// and arm control must be restarted. Runs on wall time so a /clock pause cannot stop it.
#include <chrono>
#include <string>
#include <vector>

#include <controller_manager_msgs/srv/list_hardware_components.hpp>
#include <controller_manager_msgs/srv/set_hardware_component_state.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "mobile_manipulator_control/feedback_freshness.hpp"

using namespace std::chrono_literals;
using controller_manager_msgs::srv::ListHardwareComponents;
using controller_manager_msgs::srv::SetHardwareComponentState;
using controller_manager_msgs::srv::SwitchController;
using lifecycle_msgs::msg::State;

namespace
{
double monotonic_s()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

class ArmRecoverySupervisor : public rclcpp::Node
{
public:
  ArmRecoverySupervisor()
  : Node("arm_recovery_supervisor"),
    hardware_(declare_parameter("hardware", std::string("UnityArm"))),
    controllers_(declare_parameter("controllers",
                                   std::vector<std::string>{"arm_joint_state_broadcaster", "arm_controller"})),
    retry_s_(declare_parameter("retry_s", 3.0)),
    freshness_(declare_parameter("fresh_for_s", 1.0))
  {
    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/arm/state", rclcpp::QoS(1).reliable(), [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        freshness_.on_state(monotonic_s(), msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
      });
    const std::string prefix = "/controller_manager/";
    list_client_ = create_client<ListHardwareComponents>(prefix + "list_hardware_components");
    set_client_ = create_client<SetHardwareComponentState>(prefix + "set_hardware_component_state");
    switch_client_ = create_client<SwitchController>(prefix + "switch_controller");
    timer_ = create_wall_timer(500ms, [this] { tick(); });
  }

private:
  void tick()
  {
    if (busy_ || !list_client_->service_is_ready()) return;
    busy_ = true;
    list_client_->async_send_request(
      std::make_shared<ListHardwareComponents::Request>(),
      [this](rclcpp::Client<ListHardwareComponents>::SharedFuture future) { on_list(future.get()); });
  }

  void on_list(const ListHardwareComponents::Response::SharedPtr & result)
  {
    const controller_manager_msgs::msg::HardwareComponentState * component = nullptr;
    if (result) {
      for (const auto & candidate : result->component) {
        if (candidate.name == hardware_) component = &candidate;
      }
    }
    const double now = monotonic_s();
    if (!component || component->state.id == State::PRIMARY_STATE_ACTIVE || !freshness_.fresh(now) ||
        now - last_attempt_ < retry_s_) {
      busy_ = false;
      return;
    }
    last_attempt_ = now;
    RCLCPP_WARN(get_logger(), "%s is %s with fresh Unity feedback; re-activating", hardware_.c_str(),
                component->state.label.c_str());
    auto request = std::make_shared<SetHardwareComponentState::Request>();
    request->name = hardware_;
    request->target_state.id = State::PRIMARY_STATE_ACTIVE;
    request->target_state.label = "active";
    set_client_->async_send_request(
      request, [this](rclcpp::Client<SetHardwareComponentState>::SharedFuture future) { on_set(future.get()); });
  }

  void on_set(const SetHardwareComponentState::Response::SharedPtr & result)
  {
    if (!result || !result->ok) {
      RCLCPP_ERROR(get_logger(), "%s refused activation; if the simulation epoch changed, restart arm control",
                   hardware_.c_str());
      busy_ = false;
      return;
    }
    auto request = std::make_shared<SwitchController::Request>();
    request->activate_controllers = controllers_;
    request->strictness = SwitchController::Request::STRICT;
    request->activate_asap = true;
    request->timeout.sec = 5;
    switch_client_->async_send_request(
      request, [this](rclcpp::Client<SwitchController>::SharedFuture future) { on_switch(future.get()); });
  }

  void on_switch(const SwitchController::Response::SharedPtr & result)
  {
    if (result && result->ok) {
      ++recoveries_;
      RCLCPP_WARN(get_logger(), "Arm control recovered (%d since start)", recoveries_);
    } else {
      std::string names;
      for (const auto & name : controllers_) names += (names.empty() ? "" : ", ") + name;
      RCLCPP_ERROR(get_logger(), "Could not re-activate [%s]: %s", names.c_str(),
                   result ? result->message.c_str() : "no response");
    }
    busy_ = false;
  }

  const std::string hardware_;
  const std::vector<std::string> controllers_;
  const double retry_s_;
  mobile_manipulator_control::FeedbackFreshness freshness_;
  double last_attempt_ = 0.0;
  bool busy_ = false;
  int recoveries_ = 0;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Client<ListHardwareComponents>::SharedPtr list_client_;
  rclcpp::Client<SetHardwareComponentState>::SharedPtr set_client_;
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmRecoverySupervisor>());
  rclcpp::shutdown();
  return 0;
}
