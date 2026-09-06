#include "mobile_manipulator_control/joint_packet.hpp"
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <algorithm>
#include <chrono>
#include <limits>
#include <thread>

namespace mobile_manipulator_control
{
class UnityArmSystem : public hardware_interface::SystemInterface
{
  using Packet = sensor_msgs::msg::JointState;
  using Steady = std::chrono::steady_clock;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<Packet>::SharedPtr subscription_;
  rclcpp::Publisher<Packet>::SharedPtr publisher_;
  std::vector<std::string> names_;
  std::vector<double> q_, v_, cq_, cv_, lower_, upper_, vmax_;
  Steady::time_point received_{};
  double last_stamp_ = -1, timeout_ = 0.5;
  int64_t last_command_nanoseconds_ = -1;
  bool have_state_ = false, active_ = false, claimed_ = false, fault_ = false;
  bool fresh() const {
    return have_state_ && !fault_ && std::chrono::duration<double>(Steady::now()-received_).count() < timeout_;
  }
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override {
    const auto & info = params.hardware_info;
    if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
      return hardware_interface::CallbackReturn::ERROR;
    try {
      if (info.joints.size() != 6) throw std::invalid_argument("Unity arm requires six joints");
      for (const auto & joint : info.joints) {
        if (joint.command_interfaces.size()!=2 || joint.state_interfaces.size()!=2)
          throw std::invalid_argument("Requires position and velocity command/state interfaces");
        for (const auto * interfaces : {&joint.command_interfaces, &joint.state_interfaces}) {
          std::vector<std::string> n;
          for (const auto & interface : *interfaces) n.push_back(interface.name);
          std::sort(n.begin(), n.end());
          if (n != std::vector<std::string>{"position", "velocity"}) throw std::invalid_argument("Invalid interfaces");
        }
        names_.push_back(joint.name);
        lower_.push_back(std::stod(joint.parameters.at("lower")));
        upper_.push_back(std::stod(joint.parameters.at("upper")));
        vmax_.push_back(std::stod(joint.parameters.at("velocity")));
        if (!std::isfinite(lower_.back()) || !std::isfinite(upper_.back()) ||
            !std::isfinite(vmax_.back()) || lower_.back()>=upper_.back() || vmax_.back()<=0)
          throw std::invalid_argument("Invalid physical limits: " + joint.name);
      }
      timeout_ = std::stod(info.hardware_parameters.at("state_timeout"));
      if (!std::isfinite(timeout_) || timeout_ <= 0) throw std::invalid_argument("Invalid state timeout");
      packet_mapping(names_, names_, std::vector<double>(6,0), std::vector<double>(6,0));
      q_.assign(6, std::numeric_limits<double>::quiet_NaN()); v_=cq_=cv_=q_;
      node_ = std::make_shared<rclcpp::Node>("unity_arm_hardware_transport");
      publisher_=node_->create_publisher<Packet>(info.hardware_parameters.at("command_topic"),rclcpp::QoS(1).reliable());
      subscription_=node_->create_subscription<Packet>(info.hardware_parameters.at("state_topic"),rclcpp::QoS(1).reliable(),
        [this](Packet::ConstSharedPtr msg) {
          try {
            auto map=packet_mapping(names_,msg->name,msg->position,msg->velocity);
            double stamp=msg->header.stamp.sec+1e-9*msg->header.stamp.nanosec;
            if (stamp <= last_stamp_) {
              if (stamp < last_stamp_) fault_=true; // reset requires manager restart
              return;
            }
            for (size_t i=0;i<6;++i) {q_[i]=msg->position[map[i]]; v_[i]=msg->velocity[map[i]];}
            last_stamp_=stamp; received_=Steady::now(); have_state_=true;
          } catch (const std::exception & e) {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "%s", e.what());
          }
        });
      executor_.add_node(node_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("UnityArmSystem"), "%s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> result;
    for(size_t i=0;i<6;++i) {result.emplace_back(names_[i],"position",&q_[i]); result.emplace_back(names_[i],"velocity",&v_[i]);}
    return result;
  }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> result;
    for(size_t i=0;i<6;++i) {result.emplace_back(names_[i],"position",&cq_[i]); result.emplace_back(names_[i],"velocity",&cv_[i]);}
    return result;
  }
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    auto deadline=Steady::now()+std::chrono::seconds(10);
    while (!fresh() && Steady::now()<deadline && rclcpp::ok()) {
      executor_.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!fresh()) {
      RCLCPP_ERROR(node_->get_logger(),"No fresh Unity arm feedback; enter Play before starting controller_manager");
      return hardware_interface::CallbackReturn::ERROR;
    }
    cq_=q_; cv_.assign(6,0); active_=true; last_command_nanoseconds_=-1;
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    active_=claimed_=false;
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override {
    auto ours=[this](const std::string & s) {
      return std::any_of(names_.begin(),names_.end(),[&](const auto & n){return s==n+"/position" || s==n+"/velocity";});
    };
    if (std::any_of(stop.begin(),stop.end(),ours)) claimed_=false;
    if (std::any_of(start.begin(),start.end(),ours)) { cq_=q_; cv_.assign(6,0); claimed_=true; }
    return hardware_interface::return_type::OK;
  }
  hardware_interface::return_type read(const rclcpp::Time & time,const rclcpp::Duration &) override {
    executor_.spin_some();
    if (active_ && (!fresh() || time.seconds()-last_stamp_ > timeout_ || last_stamp_-time.seconds()>timeout_)) {
      fault_=true; claimed_=false;
      RCLCPP_ERROR(node_->get_logger(),"Unity feedback stale or simulation epoch changed; restart controller manager");
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }
  hardware_interface::return_type write(const rclcpp::Time & time,const rclcpp::Duration &) override {
    if (!active_ || !claimed_ || !fresh()) return hardware_interface::return_type::OK;
    // A burst of /clock delivery can wake the manager twice at the same ROS time.
    // Unity deliberately rejects non-advancing stamps; do not manufacture those packets.
    if (time.nanoseconds()<=last_command_nanoseconds_) return hardware_interface::return_type::OK;
    for(size_t i=0;i<6;++i) {
      if (!std::isfinite(cq_[i]) || !std::isfinite(cv_[i]) || cq_[i]<lower_[i]-1e-6 ||
          cq_[i]>upper_[i]+1e-6 || std::abs(cv_[i])>vmax_[i]+1e-6) {
        fault_=true; claimed_=false;
        RCLCPP_ERROR(node_->get_logger(),"Rejected out-of-range arm target for %s",names_[i].c_str());
        return hardware_interface::return_type::ERROR;
      }
    }
    Packet msg; msg.header.stamp=time; msg.name=names_; msg.position=cq_; msg.velocity=cv_;
    publisher_->publish(msg);
    last_command_nanoseconds_=time.nanoseconds();
    return hardware_interface::return_type::OK;
  }
};
}
PLUGINLIB_EXPORT_CLASS(mobile_manipulator_control::UnityArmSystem, hardware_interface::SystemInterface)
