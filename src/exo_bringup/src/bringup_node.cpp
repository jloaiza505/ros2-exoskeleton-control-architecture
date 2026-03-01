#include <chrono>
#include <memory>
#include <string>

#include "exo_interfaces/msg/calibration_status.hpp"
#include "exo_interfaces/msg/fused_limb_state.hpp"
#include "exo_interfaces/msg/gait_phase.hpp"
#include "exo_interfaces/msg/raw_sensor_data.hpp"
#include "exo_interfaces/msg/user_message.hpp"
#include "exo_interfaces/srv/set_operating_mode.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class ExoBringupNode : public rclcpp::Node {
public:
  ExoBringupNode()
  : Node("bringup_node"),
    current_mode_("UNKNOWN"),
    has_calibration_status_(false),
    calibration_status_(exo_interfaces::msg::CalibrationStatus::UNKNOWN),
    raw_seen_(false),
    fused_seen_(false),
    gait_seen_(false)
  {
    mode_state_topic_ = this->declare_parameter<std::string>("mode_state_topic", "/exo/system/mode");
    calibration_status_topic_ = this->declare_parameter<std::string>(
      "calibration_status_topic", "/exo/system/calibration_status");
    user_message_topic_ = this->declare_parameter<std::string>("user_message_topic", "/exo/system/user_message");
    set_mode_service_ = this->declare_parameter<std::string>("set_mode_service", "/exo/system/set_mode");
    start_unit_service_ = this->declare_parameter<std::string>("start_unit_service", "/exo/system/start_unit");
    shutdown_unit_service_ = this->declare_parameter<std::string>("shutdown_unit_service", "/exo/system/shutdown_unit");

    raw_sensor_topic_ = this->declare_parameter<std::string>("raw_sensor_topic", "/exo/sensing/raw");
    fused_state_topic_ = this->declare_parameter<std::string>("fused_state_topic", "/exo/state/fused");
    gait_phase_topic_ = this->declare_parameter<std::string>("gait_phase_topic", "/exo/gait/phase");

    offline_mode_value_ = this->declare_parameter<std::string>("offline_mode_value", "OFFLINE");
    startup_mode_value_ = this->declare_parameter<std::string>("startup_mode_value", "STARTUP");
    idle_mode_value_ = this->declare_parameter<std::string>("idle_mode_value", "IDLE");
    calibration_mode_value_ = this->declare_parameter<std::string>("calibration_mode_value", "CALIBRATION");
    startup_mode_value_ = this->declare_parameter<std::string>("startup_mode_target", startup_mode_value_);
    offline_mode_value_ = this->declare_parameter<std::string>("shutdown_mode_target", offline_mode_value_);

    sensor_check_timeout_s_ = std::max(0.1, this->declare_parameter<double>("sensor_check_timeout_s", 2.0));
    sensor_freshness_timeout_s_ = std::max(0.05, this->declare_parameter<double>("sensor_freshness_timeout_s", 0.30));

    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      mode_state_topic_, 10, std::bind(&ExoBringupNode::on_mode, this, std::placeholders::_1));
    calibration_sub_ = this->create_subscription<exo_interfaces::msg::CalibrationStatus>(
      calibration_status_topic_, 10, std::bind(&ExoBringupNode::on_calibration_status, this, std::placeholders::_1));

    raw_sub_ = this->create_subscription<exo_interfaces::msg::RawSensorData>(
      raw_sensor_topic_, 10, std::bind(&ExoBringupNode::on_raw, this, std::placeholders::_1));
    fused_sub_ = this->create_subscription<exo_interfaces::msg::FusedLimbState>(
      fused_state_topic_, 10, std::bind(&ExoBringupNode::on_fused, this, std::placeholders::_1));
    gait_sub_ = this->create_subscription<exo_interfaces::msg::GaitPhase>(
      gait_phase_topic_, 10, std::bind(&ExoBringupNode::on_gait, this, std::placeholders::_1));

    user_message_pub_ = this->create_publisher<exo_interfaces::msg::UserMessage>(user_message_topic_, 10);

    client_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    mode_client_ = this->create_client<exo_interfaces::srv::SetOperatingMode>(
      set_mode_service_,
      rmw_qos_profile_services_default,
      client_callback_group_);

    start_srv_ = this->create_service<std_srvs::srv::Trigger>(
      start_unit_service_,
      std::bind(&ExoBringupNode::on_start_unit, this, std::placeholders::_1, std::placeholders::_2));

    shutdown_srv_ = this->create_service<std_srvs::srv::Trigger>(
      shutdown_unit_service_,
      std::bind(&ExoBringupNode::on_shutdown_unit, this, std::placeholders::_1, std::placeholders::_2));

    publish_user_message(
      exo_interfaces::msg::UserMessage::INFO,
      "UNIT_OFFLINE_READY",
      "Unit is OFFLINE and waiting for start command.");

    RCLCPP_INFO(
      this->get_logger(),
      "Bringup orchestrator ready (start=%s, shutdown=%s)",
      start_unit_service_.c_str(),
      shutdown_unit_service_.c_str());
  }

private:
  void publish_user_message(uint8_t severity, const std::string & code, const std::string & text)
  {
    exo_interfaces::msg::UserMessage msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.severity = severity;
    msg.code = code;
    msg.text = text;
    user_message_pub_->publish(msg);
  }

  static std::string calibration_status_to_string(uint8_t status)
  {
    if (status == exo_interfaces::msg::CalibrationStatus::MISSING) {
      return "MISSING";
    }
    if (status == exo_interfaces::msg::CalibrationStatus::VALID) {
      return "VALID";
    }
    if (status == exo_interfaces::msg::CalibrationStatus::INVALID) {
      return "INVALID";
    }
    return "UNKNOWN";
  }

  void on_mode(const std_msgs::msg::String::SharedPtr msg)
  {
    current_mode_ = msg->data;
  }

  void on_calibration_status(const exo_interfaces::msg::CalibrationStatus::SharedPtr msg)
  {
    has_calibration_status_ = true;
    calibration_status_ = msg->status;
  }

  void on_raw(const exo_interfaces::msg::RawSensorData::SharedPtr msg)
  {
    (void)msg;
    raw_seen_ = true;
    raw_last_seen_ = this->now();
  }

  void on_fused(const exo_interfaces::msg::FusedLimbState::SharedPtr msg)
  {
    (void)msg;
    fused_seen_ = true;
    fused_last_seen_ = this->now();
  }

  void on_gait(const exo_interfaces::msg::GaitPhase::SharedPtr msg)
  {
    (void)msg;
    gait_seen_ = true;
    gait_last_seen_ = this->now();
  }

  bool are_sensors_fresh() const
  {
    if (!raw_seen_ || !fused_seen_ || !gait_seen_) {
      return false;
    }

    const rclcpp::Time now = this->now();
    const double raw_age_s = (now - raw_last_seen_).seconds();
    const double fused_age_s = (now - fused_last_seen_).seconds();
    const double gait_age_s = (now - gait_last_seen_).seconds();

    return raw_age_s <= sensor_freshness_timeout_s_ &&
           fused_age_s <= sensor_freshness_timeout_s_ &&
           gait_age_s <= sensor_freshness_timeout_s_;
  }

  bool wait_for_sensor_check()
  {
    const rclcpp::Time deadline = this->now() + rclcpp::Duration::from_seconds(sensor_check_timeout_s_);
    rclcpp::Rate rate(20.0);

    while (rclcpp::ok() && this->now() < deadline) {
      if (are_sensors_fresh()) {
        return true;
      }
      rate.sleep();
    }

    return are_sensors_fresh();
  }

  bool request_mode_change(const std::string & mode, std::string & result_message)
  {
    if (!mode_client_->wait_for_service(1s)) {
      result_message = "set mode service unavailable";
      return false;
    }

    auto request = std::make_shared<exo_interfaces::srv::SetOperatingMode::Request>();
    request->mode = mode;
    auto future = mode_client_->async_send_request(request);

    if (future.wait_for(2s) != std::future_status::ready) {
      result_message = "set mode service call timeout";
      return false;
    }

    const auto response = future.get();
    result_message = response->message;
    return response->accepted;
  }

  void on_start_unit(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    std::string mode_message;
    if (!request_mode_change(startup_mode_value_, mode_message)) {
      response->success = false;
      response->message = "Failed to enter STARTUP: " + mode_message;
      publish_user_message(
        exo_interfaces::msg::UserMessage::ERROR,
        "UNIT_START_FAILED",
        response->message);
      return;
    }

    publish_user_message(
      exo_interfaces::msg::UserMessage::INFO,
      "UNIT_STARTUP_SEQUENCE",
      "Startup sequence initiated: sensor check -> calibration check.");

    if (!wait_for_sensor_check()) {
      response->success = false;
      response->message = "Sensor check failed (missing/stale streams)";
      publish_user_message(
        exo_interfaces::msg::UserMessage::ERROR,
        "SENSOR_CHECK_FAILED",
        "Startup blocked: sensor check failed.");
      return;
    }

    publish_user_message(
      exo_interfaces::msg::UserMessage::INFO,
      "SENSOR_CHECK_PASSED",
      "Sensor check passed.");

    if (!has_calibration_status_ || calibration_status_ == exo_interfaces::msg::CalibrationStatus::UNKNOWN) {
      request_mode_change(calibration_mode_value_, mode_message);
      response->success = true;
      response->message = "Calibration parameters missing/outdated; moved to CALIBRATION";
      publish_user_message(
        exo_interfaces::msg::UserMessage::WARNING,
        "CALIBRATION_REQUIRED",
        "Calibration parameters are missing or outdated. Please run full calibration.");
      return;
    }

    if (calibration_status_ == exo_interfaces::msg::CalibrationStatus::VALID) {
      if (!request_mode_change(idle_mode_value_, mode_message)) {
        response->success = false;
        response->message = "Failed to enter IDLE after checks: " + mode_message;
        publish_user_message(
          exo_interfaces::msg::UserMessage::ERROR,
          "UNIT_START_FAILED",
          response->message);
        return;
      }

      response->success = true;
      response->message = "Startup complete: IDLE (ready for ASSISTIVE)";
      publish_user_message(
        exo_interfaces::msg::UserMessage::INFO,
        "UNIT_READY_FOR_ASSISTIVE",
        "Startup checks passed. System is in IDLE and ready for ASSISTIVE.");
      return;
    }

    request_mode_change(calibration_mode_value_, mode_message);
    response->success = true;
    response->message = "Calibration " + calibration_status_to_string(calibration_status_) + "; moved to CALIBRATION";
    publish_user_message(
      exo_interfaces::msg::UserMessage::WARNING,
      "CALIBRATION_REQUIRED",
      "Calibration is invalid or outdated. Please run full calibration.");
  }

  void on_shutdown_unit(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    std::string mode_message;
    if (!request_mode_change(idle_mode_value_, mode_message)) {
      response->success = false;
      response->message = "Failed to enter IDLE for shutdown: " + mode_message;
      publish_user_message(
        exo_interfaces::msg::UserMessage::ERROR,
        "UNIT_SHUTDOWN_FAILED",
        response->message);
      return;
    }

    if (!request_mode_change(offline_mode_value_, mode_message)) {
      response->success = false;
      response->message = "Failed to enter OFFLINE: " + mode_message;
      publish_user_message(
        exo_interfaces::msg::UserMessage::ERROR,
        "UNIT_SHUTDOWN_FAILED",
        response->message);
      return;
    }

    response->success = true;
    response->message = "Unit is now OFFLINE";
    publish_user_message(
      exo_interfaces::msg::UserMessage::INFO,
      "UNIT_OFFLINE",
      "Unit is OFFLINE and waiting for next start command.");
  }

  std::string mode_state_topic_;
  std::string calibration_status_topic_;
  std::string user_message_topic_;
  std::string set_mode_service_;
  std::string start_unit_service_;
  std::string shutdown_unit_service_;

  std::string raw_sensor_topic_;
  std::string fused_state_topic_;
  std::string gait_phase_topic_;

  std::string offline_mode_value_;
  std::string startup_mode_value_;
  std::string idle_mode_value_;
  std::string calibration_mode_value_;

  double sensor_check_timeout_s_;
  double sensor_freshness_timeout_s_;

  std::string current_mode_;

  bool has_calibration_status_;
  uint8_t calibration_status_;

  bool raw_seen_;
  bool fused_seen_;
  bool gait_seen_;

  rclcpp::Time raw_last_seen_;
  rclcpp::Time fused_last_seen_;
  rclcpp::Time gait_last_seen_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<exo_interfaces::msg::CalibrationStatus>::SharedPtr calibration_sub_;
  rclcpp::Subscription<exo_interfaces::msg::RawSensorData>::SharedPtr raw_sub_;
  rclcpp::Subscription<exo_interfaces::msg::FusedLimbState>::SharedPtr fused_sub_;
  rclcpp::Subscription<exo_interfaces::msg::GaitPhase>::SharedPtr gait_sub_;

  rclcpp::Publisher<exo_interfaces::msg::UserMessage>::SharedPtr user_message_pub_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::Client<exo_interfaces::srv::SetOperatingMode>::SharedPtr mode_client_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr shutdown_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExoBringupNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
