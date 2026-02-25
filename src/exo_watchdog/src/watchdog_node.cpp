#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "exo_interfaces/msg/fault_event.hpp"
#include "exo_interfaces/msg/fused_limb_state.hpp"
#include "exo_interfaces/msg/gait_phase.hpp"
#include "exo_interfaces/msg/raw_sensor_data.hpp"
#include "exo_interfaces/msg/torque_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class WatchdogNode : public rclcpp::Node {
public:
  WatchdogNode()
  : Node("watchdog_node"),
    raw_seen_(false),
    fused_seen_(false),
    phase_seen_(false),
    torque_seen_(false),
    raw_faulted_(false),
    fused_faulted_(false),
    phase_faulted_(false),
    torque_faulted_(false),
    mode_seen_(false),
    current_mode_("STARTUP") {
    raw_sensor_topic_ = this->declare_parameter<std::string>("raw_sensor_topic", "/exo/sensing/raw");
    fused_state_topic_ = this->declare_parameter<std::string>("fused_state_topic", "/exo/state/fused");
    gait_phase_topic_ = this->declare_parameter<std::string>("gait_phase_topic", "/exo/gait/phase");
    torque_command_topic_ = this->declare_parameter<std::string>("torque_command_topic", "/exo/control/torque_command");
    fault_event_topic_ = this->declare_parameter<std::string>("fault_event_topic", "/exo/fault/event");
    mode_state_topic_ = this->declare_parameter<std::string>("mode_state_topic", "/exo/system/mode");

    check_period_ms_ = this->declare_parameter<int>("check_period_ms", 20);
    mode_change_grace_period_ms_ = this->declare_parameter<int>("mode_change_grace_period_ms", 250);
    raw_timeout_ms_ = this->declare_parameter<int>("raw_sensor_timeout_ms", 100);
    fused_timeout_ms_ = this->declare_parameter<int>("fused_state_timeout_ms", 150);
    phase_timeout_ms_ = this->declare_parameter<int>("gait_phase_timeout_ms", 200);
    torque_timeout_ms_ = this->declare_parameter<int>("torque_command_timeout_ms", 300);

    mode_last_changed_ = this->now();

    fault_pub_ = this->create_publisher<exo_interfaces::msg::FaultEvent>(fault_event_topic_, 10);

    raw_sub_ = this->create_subscription<exo_interfaces::msg::RawSensorData>(
      raw_sensor_topic_, 10, [this](const exo_interfaces::msg::RawSensorData::SharedPtr) {
        raw_last_seen_ = this->now();
        raw_seen_ = true;
      });

    fused_sub_ = this->create_subscription<exo_interfaces::msg::FusedLimbState>(
      fused_state_topic_, 10, [this](const exo_interfaces::msg::FusedLimbState::SharedPtr) {
        fused_last_seen_ = this->now();
        fused_seen_ = true;
      });

    phase_sub_ = this->create_subscription<exo_interfaces::msg::GaitPhase>(
      gait_phase_topic_, 10, [this](const exo_interfaces::msg::GaitPhase::SharedPtr) {
        phase_last_seen_ = this->now();
        phase_seen_ = true;
      });

    torque_sub_ = this->create_subscription<exo_interfaces::msg::TorqueCommand>(
      torque_command_topic_, 10, [this](const exo_interfaces::msg::TorqueCommand::SharedPtr) {
        torque_last_seen_ = this->now();
        torque_seen_ = true;
      });

    rclcpp::QoS mode_qos(1);
    mode_qos.transient_local();
    mode_qos.reliable();
    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      mode_state_topic_, mode_qos, std::bind(&WatchdogNode::on_mode, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(std::max(1, check_period_ms_)),
      std::bind(&WatchdogNode::check_health, this));

    RCLCPP_INFO(this->get_logger(), "Watchdog monitoring raw/fused/phase/torque streams");
  }

private:
  void on_mode(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data != current_mode_) {
      current_mode_ = msg->data;
      mode_last_changed_ = this->now();
      raw_faulted_ = false;
      fused_faulted_ = false;
      phase_faulted_ = false;
      torque_faulted_ = false;
      RCLCPP_INFO(this->get_logger(), "Watchdog mode update: %s", current_mode_.c_str());
    }
    mode_seen_ = true;
  }

  bool monitoring_enabled() const {
    if (!mode_seen_) {
      return false;
    }

    if (current_mode_ == "STARTUP" || current_mode_ == "FAULT") {
      return false;
    }

    const double since_mode_change_ms = (this->now() - mode_last_changed_).seconds() * 1000.0;
    return since_mode_change_ms >= static_cast<double>(mode_change_grace_period_ms_);
  }

  void check_timeout(
    const std::string & stream_name,
    const rclcpp::Time & last_seen,
    bool seen,
    int timeout_ms,
    bool & faulted_flag) {
    if (!seen) {
      const double since_mode_change_ms = (this->now() - mode_last_changed_).seconds() * 1000.0;
      if (!faulted_flag && since_mode_change_ms > static_cast<double>(timeout_ms)) {
        publish_fault(stream_name, "timeout", 2, "No message received in current mode window");
        faulted_flag = true;
      }
      return;
    }

    const double elapsed_ms = (this->now() - last_seen).seconds() * 1000.0;
    if (elapsed_ms > static_cast<double>(timeout_ms)) {
      if (!faulted_flag) {
        publish_fault(stream_name, "timeout", 2, "No message within timeout window");
        faulted_flag = true;
      }
    } else {
      faulted_flag = false;
    }
  }

  void check_health() {
    if (!monitoring_enabled()) {
      return;
    }

    check_timeout("raw_sensor", raw_last_seen_, raw_seen_, raw_timeout_ms_, raw_faulted_);
    check_timeout("fused_state", fused_last_seen_, fused_seen_, fused_timeout_ms_, fused_faulted_);
    check_timeout("gait_phase", phase_last_seen_, phase_seen_, phase_timeout_ms_, phase_faulted_);

    if (current_mode_ == "ASSISTIVE") {
      check_timeout("torque_command", torque_last_seen_, torque_seen_, torque_timeout_ms_, torque_faulted_);
    } else {
      torque_faulted_ = false;
    }
  }

  void publish_fault(
    const std::string & source_node,
    const std::string & failure_type,
    uint8_t severity,
    const std::string & detail) {
    exo_interfaces::msg::FaultEvent event;
    event.header.stamp = this->now();
    event.source_node = source_node;
    event.failure_type = failure_type;
    event.severity = severity;
    event.detail = detail;
    fault_pub_->publish(event);

    RCLCPP_WARN(this->get_logger(), "Watchdog fault: %s - %s", source_node.c_str(), detail.c_str());
  }

  std::string raw_sensor_topic_;
  std::string fused_state_topic_;
  std::string gait_phase_topic_;
  std::string torque_command_topic_;
  std::string fault_event_topic_;
  std::string mode_state_topic_;

  int check_period_ms_;
  int mode_change_grace_period_ms_;
  int raw_timeout_ms_;
  int fused_timeout_ms_;
  int phase_timeout_ms_;
  int torque_timeout_ms_;

  bool raw_seen_;
  bool fused_seen_;
  bool phase_seen_;
  bool torque_seen_;

  bool raw_faulted_;
  bool fused_faulted_;
  bool phase_faulted_;
  bool torque_faulted_;

  bool mode_seen_;
  std::string current_mode_;
  rclcpp::Time mode_last_changed_;

  rclcpp::Time raw_last_seen_;
  rclcpp::Time fused_last_seen_;
  rclcpp::Time phase_last_seen_;
  rclcpp::Time torque_last_seen_;

  rclcpp::Subscription<exo_interfaces::msg::RawSensorData>::SharedPtr raw_sub_;
  rclcpp::Subscription<exo_interfaces::msg::FusedLimbState>::SharedPtr fused_sub_;
  rclcpp::Subscription<exo_interfaces::msg::GaitPhase>::SharedPtr phase_sub_;
  rclcpp::Subscription<exo_interfaces::msg::TorqueCommand>::SharedPtr torque_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;

  rclcpp::Publisher<exo_interfaces::msg::FaultEvent>::SharedPtr fault_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WatchdogNode>());
  rclcpp::shutdown();
  return 0;
}
