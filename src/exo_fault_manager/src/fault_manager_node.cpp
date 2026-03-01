#include <algorithm>
#include <cctype>
#include <memory>
#include <string>

#include "exo_interfaces/msg/fault_event.hpp"
#include "exo_interfaces/msg/calibration_status.hpp"
#include "exo_interfaces/srv/set_operating_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class FaultManagerNode : public rclcpp::Node {
public:
  FaultManagerNode()
  : Node("fault_manager_node"),
    last_published_mode_(""),
    has_calibration_status_(false),
    calibration_status_(exo_interfaces::msg::CalibrationStatus::UNKNOWN)
  {
    fault_event_topic_ = this->declare_parameter<std::string>("fault_event_topic", "/exo/fault/event");
    mode_state_topic_ = this->declare_parameter<std::string>("mode_state_topic", "/exo/system/mode");
    calibration_status_topic_ = this->declare_parameter<std::string>(
      "calibration_status_topic", "/exo/system/calibration_status");
    set_mode_service_ = this->declare_parameter<std::string>("set_mode_service", "/exo/system/set_mode");
    initial_mode_ = this->declare_parameter<std::string>("initial_mode", "OFFLINE");
    mode_publish_rate_hz_ = this->declare_parameter<double>("mode_publish_rate_hz", 5.0);
    fault_severity_threshold_ = this->declare_parameter<int>("fault_severity_threshold", 2);
    startup_mode_duration_ms_ = this->declare_parameter<int>("startup_mode_duration_ms", 1000);
    startup_auto_transition_enabled_ = this->declare_parameter<bool>("startup_auto_transition_enabled", false);
    require_valid_calibration_for_assistive_ = this->declare_parameter<bool>(
      "require_valid_calibration_for_assistive", true);
    calibration_status_timeout_s_ = std::max(
      0.05, this->declare_parameter<double>("calibration_status_timeout_s", 2.0));

    current_mode_ = normalize_mode(initial_mode_);
    if (!is_valid_mode(current_mode_)) {
      RCLCPP_WARN(this->get_logger(), "Invalid initial_mode '%s', defaulting to OFFLINE", initial_mode_.c_str());
      current_mode_ = "OFFLINE";
    }

    rclcpp::QoS mode_qos(1);
    mode_qos.transient_local();
    mode_qos.reliable();

    mode_pub_ = this->create_publisher<std_msgs::msg::String>(mode_state_topic_, mode_qos);

    fault_sub_ = this->create_subscription<exo_interfaces::msg::FaultEvent>(
      fault_event_topic_,
      10,
      std::bind(&FaultManagerNode::on_fault_event, this, std::placeholders::_1));

    calibration_status_sub_ = this->create_subscription<exo_interfaces::msg::CalibrationStatus>(
      calibration_status_topic_,
      10,
      std::bind(&FaultManagerNode::on_calibration_status, this, std::placeholders::_1));

    mode_srv_ = this->create_service<exo_interfaces::srv::SetOperatingMode>(
      set_mode_service_,
      std::bind(&FaultManagerNode::on_set_mode_request, this, std::placeholders::_1, std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, mode_publish_rate_hz_));
    mode_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&FaultManagerNode::publish_mode, this));

    if (startup_auto_transition_enabled_ && current_mode_ == "STARTUP") {
      startup_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(std::max(1, startup_mode_duration_ms_)),
        std::bind(&FaultManagerNode::on_startup_complete, this));
    }

    publish_mode();

    RCLCPP_INFO(
      this->get_logger(),
      "Fault manager ready: mode=%s, fault_topic=%s, mode_topic=%s, service=%s",
      current_mode_.c_str(),
      fault_event_topic_.c_str(),
      mode_state_topic_.c_str(),
      set_mode_service_.c_str());
  }

private:
  static std::string normalize_mode(const std::string & mode) {
    std::string normalized = mode;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
      return static_cast<char>(std::toupper(c));
    });
    return normalized;
  }

  static bool is_valid_mode(const std::string & mode) {
    return mode == "OFFLINE" || mode == "STARTUP" || mode == "IDLE" || mode == "CALIBRATION" || mode == "ASSISTIVE" || mode == "FAULT";
  }

  bool can_transition(const std::string & from, const std::string & to) const {
    if (!is_valid_mode(to)) {
      return false;
    }

    if (from == to) {
      return true;
    }

    if (from == "OFFLINE") {
      return to == "STARTUP" || to == "FAULT";
    }
    if (from == "STARTUP") {
      return to == "IDLE" || to == "CALIBRATION" || to == "FAULT";
    }
    if (from == "IDLE") {
      return to == "CALIBRATION" || to == "FAULT" || to == "OFFLINE";
    }
    if (from == "CALIBRATION") {
      return to == "ASSISTIVE" || to == "IDLE" || to == "FAULT";
    }
    if (from == "ASSISTIVE") {
      return to == "IDLE" || to == "FAULT";
    }
    if (from == "FAULT") {
      // Deterministic recovery: explicit manual reset back to IDLE only.
      return to == "IDLE";
    }
    return false;
  }

  void transition_to(const std::string & next_mode, const std::string & reason) {
    if (!can_transition(current_mode_, next_mode)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Rejected mode transition %s -> %s (%s)",
        current_mode_.c_str(),
        next_mode.c_str(),
        reason.c_str());
      return;
    }

    if (current_mode_ != next_mode) {
      RCLCPP_WARN(
        this->get_logger(),
        "[MODE CHANGE] %s -> %s",
        current_mode_.c_str(),
        next_mode.c_str());
      current_mode_ = next_mode;
      publish_mode();
    }
  }

  void on_startup_complete() {
    if (current_mode_ == "STARTUP") {
      transition_to("IDLE", "startup_complete");
    }
    startup_timer_.reset();
  }

  void on_fault_event(const exo_interfaces::msg::FaultEvent::SharedPtr msg) {
    if (static_cast<int>(msg->severity) >= fault_severity_threshold_) {
      transition_to("FAULT", "fault_event:" + msg->source_node + ":" + msg->failure_type);
    }
  }

  void on_calibration_status(const exo_interfaces::msg::CalibrationStatus::SharedPtr msg) {
    calibration_status_ = msg->status;
    has_calibration_status_ = true;
    last_calibration_status_rx_time_ = this->now();
  }

  bool can_enter_assistive(std::string & reason) const {
    if (!require_valid_calibration_for_assistive_) {
      return true;
    }

    if (!has_calibration_status_) {
      reason = "calibration status unavailable";
      return false;
    }

    const double status_age_s = (this->now() - last_calibration_status_rx_time_).seconds();
    if (status_age_s > calibration_status_timeout_s_) {
      reason = "calibration status stale";
      return false;
    }

    if (calibration_status_ != exo_interfaces::msg::CalibrationStatus::VALID) {
      reason = "calibration not valid";
      return false;
    }

    return true;
  }

  void on_set_mode_request(
    const std::shared_ptr<exo_interfaces::srv::SetOperatingMode::Request> request,
    std::shared_ptr<exo_interfaces::srv::SetOperatingMode::Response> response) {
    const std::string requested_mode = normalize_mode(request->mode);
    if (!is_valid_mode(requested_mode)) {
      response->accepted = false;
      response->message = "Invalid mode for request: " + request->mode;
      return;
    }

    if (!can_transition(current_mode_, requested_mode)) {
      response->accepted = false;
      response->message = "Invalid transition: " + current_mode_ + " -> " + requested_mode;
      return;
    }

    if (requested_mode == "ASSISTIVE") {
      std::string denial_reason;
      if (!can_enter_assistive(denial_reason)) {
        response->accepted = false;
        response->message = "Denied ASSISTIVE: " + denial_reason;
        return;
      }
    }

    transition_to(requested_mode, "service_request");
    response->accepted = true;
    response->message = "Mode set to " + current_mode_;
  }

  void publish_mode() {
    std_msgs::msg::String mode_msg;
    mode_msg.data = current_mode_;
    mode_pub_->publish(mode_msg);

    if (current_mode_ != last_published_mode_) {
      RCLCPP_INFO(this->get_logger(), "Operating mode: %s", current_mode_.c_str());
      last_published_mode_ = current_mode_;
    }
  }

  std::string fault_event_topic_;
  std::string mode_state_topic_;
  std::string calibration_status_topic_;
  std::string set_mode_service_;
  std::string initial_mode_;
  std::string current_mode_;
  std::string last_published_mode_;
  double mode_publish_rate_hz_;
  bool require_valid_calibration_for_assistive_;
  double calibration_status_timeout_s_;
  int fault_severity_threshold_;
  int startup_mode_duration_ms_;
  bool startup_auto_transition_enabled_;
  bool has_calibration_status_;
  uint8_t calibration_status_;
  rclcpp::Time last_calibration_status_rx_time_;

  rclcpp::Subscription<exo_interfaces::msg::FaultEvent>::SharedPtr fault_sub_;
  rclcpp::Subscription<exo_interfaces::msg::CalibrationStatus>::SharedPtr calibration_status_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Service<exo_interfaces::srv::SetOperatingMode>::SharedPtr mode_srv_;
  rclcpp::TimerBase::SharedPtr mode_timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FaultManagerNode>());
  rclcpp::shutdown();
  return 0;
}
