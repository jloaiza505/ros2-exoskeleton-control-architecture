#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include "exo_interfaces/msg/calibration_status.hpp"
#include "exo_interfaces/msg/fused_limb_state.hpp"
#include "exo_interfaces/msg/gait_phase.hpp"
#include "exo_interfaces/msg/user_message.hpp"
#include "exo_interfaces/srv/set_operating_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

class CalibrationNode : public rclcpp::Node {
public:
  CalibrationNode()
  : Node("calibration_node"),
    current_mode_("OFFLINE"),
    has_baseline_(false),
    current_status_(exo_interfaces::msg::CalibrationStatus::MISSING),
    current_detail_("calibration_required")
  {
    fused_state_topic_ = this->declare_parameter<std::string>("fused_state_topic", "/exo/state/fused");
    gait_phase_topic_ = this->declare_parameter<std::string>("gait_phase_topic", "/exo/gait/phase");
    mode_state_topic_ = this->declare_parameter<std::string>("mode_state_topic", "/exo/system/mode");
    calibration_status_topic_ = this->declare_parameter<std::string>(
      "calibration_status_topic", "/exo/system/calibration_status");
    run_full_calibration_service_ = this->declare_parameter<std::string>(
      "run_full_calibration_service", "/exo/system/run_full_calibration");
    validate_calibration_service_ = this->declare_parameter<std::string>(
      "validate_calibration_service", "/exo/system/validate_calibration");
    user_message_topic_ = this->declare_parameter<std::string>("user_message_topic", "/exo/system/user_message");
    set_mode_service_ = this->declare_parameter<std::string>("set_mode_service", "/exo/system/set_mode");

    status_publish_rate_hz_ = std::max(0.2, this->declare_parameter<double>("status_publish_rate_hz", 2.0));
    analysis_window_s_ = std::max(0.5, this->declare_parameter<double>("analysis_window_s", 3.0));
    min_samples_ = std::max<int64_t>(5, this->declare_parameter<int64_t>("min_samples", 30));
    max_velocity_ratio_delta_ = std::max(
      0.01, this->declare_parameter<double>("max_velocity_ratio_delta", 0.35));
    max_confidence_delta_ = std::max(0.01, this->declare_parameter<double>("max_confidence_delta", 0.20));
    max_contact_delta_ = std::max(0.01, this->declare_parameter<double>("max_contact_delta", 0.25));
    require_calibration_mode_for_full_ = this->declare_parameter<bool>(
      "require_calibration_mode_for_full", true);
    baseline_store_path_ = this->declare_parameter<std::string>(
      "baseline_store_path", "/tmp/exo_calibration_baseline.yaml");
    auto_load_baseline_ = this->declare_parameter<bool>("auto_load_baseline", true);
    auto_save_baseline_ = this->declare_parameter<bool>("auto_save_baseline", true);
    auto_validate_on_idle_startup_ = this->declare_parameter<bool>("auto_validate_on_idle_startup", true);
    startup_validation_delay_s_ = std::max(0.1, this->declare_parameter<double>("startup_validation_delay_s", 3.0));
    startup_validation_mode_ = this->declare_parameter<std::string>("startup_validation_mode", "auto");
    auto_request_calibration_mode_ = this->declare_parameter<bool>("auto_request_calibration_mode", true);

    rclcpp::QoS status_qos(1);
    status_qos.transient_local();
    status_qos.reliable();

    status_pub_ = this->create_publisher<exo_interfaces::msg::CalibrationStatus>(
      calibration_status_topic_, status_qos);
    user_message_pub_ = this->create_publisher<exo_interfaces::msg::UserMessage>(user_message_topic_, 10);

    fused_sub_ = this->create_subscription<exo_interfaces::msg::FusedLimbState>(
      fused_state_topic_, 20, std::bind(&CalibrationNode::on_fused_state, this, std::placeholders::_1));
    gait_sub_ = this->create_subscription<exo_interfaces::msg::GaitPhase>(
      gait_phase_topic_, 20, std::bind(&CalibrationNode::on_gait_phase, this, std::placeholders::_1));
    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      mode_state_topic_, 10, std::bind(&CalibrationNode::on_mode, this, std::placeholders::_1));

    full_calibration_srv_ = this->create_service<std_srvs::srv::Trigger>(
      run_full_calibration_service_,
      std::bind(&CalibrationNode::on_run_full_calibration, this, std::placeholders::_1, std::placeholders::_2));

    validate_calibration_srv_ = this->create_service<std_srvs::srv::Trigger>(
      validate_calibration_service_,
      std::bind(&CalibrationNode::on_validate_calibration, this, std::placeholders::_1, std::placeholders::_2));
    mode_client_ = this->create_client<exo_interfaces::srv::SetOperatingMode>(set_mode_service_);

    const auto period = std::chrono::duration<double>(1.0 / status_publish_rate_hz_);
    status_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CalibrationNode::publish_status, this));

    if (auto_load_baseline_) {
      std::string load_reason;
      if (load_baseline_from_file(load_reason)) {
        set_status(exo_interfaces::msg::CalibrationStatus::UNKNOWN, "baseline_loaded_validation_required");
        startup_validation_pending_ = auto_validate_on_idle_startup_;
        publish_user_message(
          exo_interfaces::msg::UserMessage::INFO,
          "CALIBRATION_BASELINE_LOADED",
          "Calibration baseline loaded. Waiting for quick validation.");
      } else {
        RCLCPP_INFO(this->get_logger(), "No baseline loaded at startup: %s", load_reason.c_str());
        set_status(exo_interfaces::msg::CalibrationStatus::MISSING, "baseline_missing");
        calibration_mode_request_pending_ = auto_request_calibration_mode_;
        publish_user_message(
          exo_interfaces::msg::UserMessage::WARNING,
          "CALIBRATION_REQUIRED",
          "No calibration baseline found. Calibration is required.");
      }
    }

    publish_status();

    RCLCPP_INFO(
      this->get_logger(),
      "Calibration node ready (status topic: %s, full service: %s, validate service: %s)",
      calibration_status_topic_.c_str(),
      run_full_calibration_service_.c_str(),
      validate_calibration_service_.c_str());
  }

private:
  struct TimedSample {
    rclcpp::Time stamp;
    double value;
  };

  struct WindowFeatures {
    double mean_abs_velocity = 0.0;
    double mean_confidence = 0.0;
    double mean_contact_probability = 0.0;
  };

  static double clamp_unit(double value) {
    return std::clamp(value, 0.0, 1.0);
  }

  void on_fused_state(const exo_interfaces::msg::FusedLimbState::SharedPtr msg) {
    const rclcpp::Time stamp = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0
      ? this->now()
      : rclcpp::Time(msg->header.stamp);
    velocity_samples_.push_back({stamp, std::fabs(msg->joint_velocity_rad_s)});
    contact_samples_.push_back({stamp, clamp_unit(msg->contact_probability)});
    prune_old_samples();
  }

  void on_gait_phase(const exo_interfaces::msg::GaitPhase::SharedPtr msg) {
    const rclcpp::Time stamp = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0
      ? this->now()
      : rclcpp::Time(msg->header.stamp);
    confidence_samples_.push_back({stamp, clamp_unit(msg->confidence)});
    prune_old_samples();
  }

  void on_mode(const std_msgs::msg::String::SharedPtr msg) {
    current_mode_ = msg->data;
    if (current_mode_ == "IDLE" && !idle_tracking_active_) {
      idle_since_ = this->now();
      idle_tracking_active_ = true;
    }
    if (current_mode_ != "IDLE") {
      idle_tracking_active_ = false;
    }
  }

  void prune_queue(std::deque<TimedSample> & queue, const rclcpp::Time & cutoff) {
    while (!queue.empty() && queue.front().stamp < cutoff) {
      queue.pop_front();
    }
  }

  void prune_old_samples() {
    const rclcpp::Time cutoff = this->now() - rclcpp::Duration::from_seconds(analysis_window_s_);
    prune_queue(velocity_samples_, cutoff);
    prune_queue(confidence_samples_, cutoff);
    prune_queue(contact_samples_, cutoff);
  }

  double mean_of(const std::deque<TimedSample> & samples) const {
    if (samples.empty()) {
      return 0.0;
    }
    double sum = 0.0;
    for (const auto & sample : samples) {
      sum += sample.value;
    }
    return sum / static_cast<double>(samples.size());
  }

  bool collect_window_features(WindowFeatures & out, std::string & reason) {
    prune_old_samples();

    if (
      static_cast<int64_t>(velocity_samples_.size()) < min_samples_ ||
      static_cast<int64_t>(confidence_samples_.size()) < min_samples_ ||
      static_cast<int64_t>(contact_samples_.size()) < min_samples_)
    {
      std::ostringstream oss;
      oss << "insufficient samples in " << analysis_window_s_ << "s window (need " << min_samples_ << ")";
      reason = oss.str();
      return false;
    }

    out.mean_abs_velocity = mean_of(velocity_samples_);
    out.mean_confidence = mean_of(confidence_samples_);
    out.mean_contact_probability = mean_of(contact_samples_);
    return true;
  }

  void set_status(uint8_t status, const std::string & detail) {
    current_status_ = status;
    current_detail_ = detail;
    publish_status();
  }

  bool load_baseline_from_file(std::string & reason) {
    std::ifstream in(baseline_store_path_);
    if (!in.is_open()) {
      reason = "baseline file not found";
      return false;
    }

    WindowFeatures loaded;
    bool got_velocity = false;
    bool got_confidence = false;
    bool got_contact = false;

    std::string line;
    while (std::getline(in, line)) {
      const auto sep = line.find(':');
      if (sep == std::string::npos) {
        continue;
      }

      const std::string key = line.substr(0, sep);
      const std::string value = line.substr(sep + 1);
      try {
        const double parsed = std::stod(value);
        if (key == "mean_abs_velocity") {
          loaded.mean_abs_velocity = parsed;
          got_velocity = true;
        } else if (key == "mean_confidence") {
          loaded.mean_confidence = parsed;
          got_confidence = true;
        } else if (key == "mean_contact_probability") {
          loaded.mean_contact_probability = parsed;
          got_contact = true;
        }
      } catch (const std::exception &) {
        continue;
      }
    }

    if (!got_velocity || !got_confidence || !got_contact) {
      reason = "baseline file missing required fields";
      return false;
    }

    baseline_ = loaded;
    has_baseline_ = true;
    reason = "baseline loaded";
    RCLCPP_INFO(
      this->get_logger(),
      "Loaded calibration baseline from %s",
      baseline_store_path_.c_str());
    return true;
  }

  bool save_baseline_to_file(std::string & reason) {
    const std::filesystem::path baseline_path(baseline_store_path_);
    const std::filesystem::path parent = baseline_path.parent_path();
    if (!parent.empty()) {
      std::error_code ec;
      std::filesystem::create_directories(parent, ec);
      if (ec) {
        reason = "failed to create baseline directory";
        return false;
      }
    }

    std::ofstream out(baseline_store_path_, std::ios::trunc);
    if (!out.is_open()) {
      reason = "failed to open baseline file for write";
      return false;
    }

    out << "mean_abs_velocity:" << baseline_.mean_abs_velocity << "\n";
    out << "mean_confidence:" << baseline_.mean_confidence << "\n";
    out << "mean_contact_probability:" << baseline_.mean_contact_probability << "\n";
    out.flush();

    if (!out.good()) {
      reason = "failed to flush baseline file";
      return false;
    }

    reason = "baseline saved";
    RCLCPP_INFO(
      this->get_logger(),
      "Saved calibration baseline to %s",
      baseline_store_path_.c_str());
    return true;
  }

  void publish_status() {
    maybe_run_startup_validation();
    maybe_request_calibration_mode();

    exo_interfaces::msg::CalibrationStatus msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "knee_joint";
    msg.status = current_status_;
    msg.detail = current_detail_;
    status_pub_->publish(msg);
  }

  void publish_user_message(uint8_t severity, const std::string & code, const std::string & text) {
    exo_interfaces::msg::UserMessage msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.severity = severity;
    msg.code = code;
    msg.text = text;
    user_message_pub_->publish(msg);
  }

  bool request_mode_change(const std::string & mode) {
    if (!mode_client_->wait_for_service(std::chrono::milliseconds(100))) {
      RCLCPP_WARN(this->get_logger(), "Mode service unavailable for request: %s", mode.c_str());
      return false;
    }

    auto request = std::make_shared<exo_interfaces::srv::SetOperatingMode::Request>();
    request->mode = mode;
    mode_client_->async_send_request(request);
    return true;
  }

  void maybe_request_calibration_mode() {
    if (!calibration_mode_request_pending_ || !auto_request_calibration_mode_) {
      return;
    }
    if (current_mode_ != "IDLE") {
      return;
    }
    if (request_mode_change("CALIBRATION")) {
      calibration_mode_request_pending_ = false;
      publish_user_message(
        exo_interfaces::msg::UserMessage::WARNING,
        "CALIBRATION_MODE_REQUESTED",
        "Switching to CALIBRATION mode. Please run full calibration.");
    }
  }

  void maybe_run_startup_validation() {
    if (!startup_validation_pending_ || current_mode_ != "IDLE") {
      return;
    }
    if (!idle_tracking_active_) {
      return;
    }
    const double idle_elapsed_s = (this->now() - idle_since_).seconds();
    if (idle_elapsed_s < startup_validation_delay_s_) {
      return;
    }

    startup_validation_pending_ = false;

    bool valid = false;
    std::string detail_message;
    if (!validate_baseline(valid, detail_message, true)) {
      set_status(exo_interfaces::msg::CalibrationStatus::INVALID, "startup_validation_data_insufficient");
      calibration_mode_request_pending_ = true;
      publish_user_message(
        exo_interfaces::msg::UserMessage::WARNING,
        "CALIBRATION_VALIDATION_INSUFFICIENT",
        "Quick calibration check could not complete. Calibration is required.");
      return;
    }

    if (valid) {
      set_status(exo_interfaces::msg::CalibrationStatus::VALID, "startup_validation_pass");
      publish_user_message(
        exo_interfaces::msg::UserMessage::INFO,
        "SYSTEM_READY_FOR_ASSISTIVE",
        "Calibration validated. System is ready to enter ASSISTIVE from IDLE.");
      return;
    }

    set_status(exo_interfaces::msg::CalibrationStatus::INVALID, "startup_validation_fail");
    calibration_mode_request_pending_ = true;
    publish_user_message(
      exo_interfaces::msg::UserMessage::WARNING,
      "CALIBRATION_INVALID",
      "Calibration check failed. Recalibration is required.");
  }

  bool validate_baseline(bool & valid, std::string & detail, bool apply_startup_override) {
    if (!has_baseline_) {
      detail = "baseline missing";
      return false;
    }

    WindowFeatures current;
    if (!collect_window_features(current, detail)) {
      return false;
    }

    const double velocity_ref = std::max(0.1, baseline_.mean_abs_velocity);
    const double velocity_ratio_delta = std::fabs(current.mean_abs_velocity - baseline_.mean_abs_velocity) / velocity_ref;
    const double confidence_delta = std::fabs(current.mean_confidence - baseline_.mean_confidence);
    const double contact_delta = std::fabs(current.mean_contact_probability - baseline_.mean_contact_probability);

    valid = velocity_ratio_delta <= max_velocity_ratio_delta_ &&
      confidence_delta <= max_confidence_delta_ &&
      contact_delta <= max_contact_delta_;

    if (apply_startup_override && startup_validation_mode_ == "force_valid") {
      valid = true;
    } else if (apply_startup_override && startup_validation_mode_ == "force_invalid") {
      valid = false;
    }

    std::ostringstream oss;
    oss << "dv=" << velocity_ratio_delta
        << ", dc=" << confidence_delta
        << ", dp=" << contact_delta;
    detail = oss.str();
    return true;
  }

  void on_run_full_calibration(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    publish_user_message(
      exo_interfaces::msg::UserMessage::INFO,
      "CALIBRATION_STARTED",
      "Calibration started. Estimating baseline parameters.");

    if (require_calibration_mode_for_full_ && current_mode_ != "CALIBRATION") {
      response->success = false;
      response->message = "Full calibration requires CALIBRATION mode";
      publish_user_message(
        exo_interfaces::msg::UserMessage::WARNING,
        "CALIBRATION_MODE_REQUIRED",
        "Full calibration requires CALIBRATION mode.");
      return;
    }

    WindowFeatures features;
    std::string reason;
    if (!collect_window_features(features, reason)) {
      response->success = false;
      response->message = "Full calibration failed: " + reason;
      set_status(exo_interfaces::msg::CalibrationStatus::MISSING, "calibration_data_insufficient");
      publish_user_message(
        exo_interfaces::msg::UserMessage::WARNING,
        "CALIBRATION_DATA_INSUFFICIENT",
        "Full calibration failed due to insufficient data.");
      return;
    }

    baseline_ = features;
    has_baseline_ = true;

    publish_user_message(
      exo_interfaces::msg::UserMessage::INFO,
      "CALIBRATION_ESTIMATION_COMPLETE",
      "Calibration parameter estimation completed. Running validation.");

    if (auto_save_baseline_) {
      std::string save_reason;
      if (!save_baseline_to_file(save_reason)) {
        RCLCPP_WARN(this->get_logger(), "Full calibration completed but persistence failed: %s", save_reason.c_str());
      }
    }

    bool valid = false;
    std::string validation_detail;
    if (!validate_baseline(valid, validation_detail, false)) {
      response->success = false;
      response->message = "Calibration validation failed: " + validation_detail;
      set_status(exo_interfaces::msg::CalibrationStatus::INVALID, "validation_data_insufficient");
      calibration_mode_request_pending_ = true;
      publish_user_message(
        exo_interfaces::msg::UserMessage::WARNING,
        "CALIBRATION_VALIDATION_INSUFFICIENT",
        "Calibration validation had insufficient data. Please run full calibration again.");
      return;
    }

    if (!valid) {
      response->success = false;
      response->message = "Calibration validation failed";
      set_status(exo_interfaces::msg::CalibrationStatus::INVALID, "validation_fail:" + validation_detail);
      calibration_mode_request_pending_ = true;
      publish_user_message(
        exo_interfaces::msg::UserMessage::WARNING,
        "CALIBRATION_VALIDATION_FAILED",
        "Calibration validation unsuccessful. Please run full calibration again.");
      return;
    }

    set_status(exo_interfaces::msg::CalibrationStatus::VALID, "validation_pass:" + validation_detail);
    startup_validation_pending_ = false;

    if (!request_mode_change("IDLE")) {
      response->success = false;
      response->message = "Calibration succeeded but failed to request IDLE";
      publish_user_message(
        exo_interfaces::msg::UserMessage::ERROR,
        "CALIBRATION_COMPLETE_IDLE_REQUEST_FAILED",
        "Calibration succeeded but could not request IDLE mode.");
      return;
    }

    response->success = true;
    response->message = "Calibration and validation successful; requesting IDLE";
    publish_user_message(
      exo_interfaces::msg::UserMessage::INFO,
      "CALIBRATION_VALIDATION_SUCCESS",
      "Calibration ended successfully. Transitioning to IDLE.");
    publish_user_message(
      exo_interfaces::msg::UserMessage::INFO,
      "SYSTEM_READY_FOR_ASSISTIVE",
      "System is in IDLE and ready to enter ASSISTIVE mode.");
  }

  void on_validate_calibration(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    if (!has_baseline_) {
      response->success = false;
      response->message = "Calibration baseline missing";
      set_status(exo_interfaces::msg::CalibrationStatus::MISSING, "baseline_missing");
      publish_user_message(
        exo_interfaces::msg::UserMessage::WARNING,
        "CALIBRATION_REQUIRED",
        "Calibration baseline is missing. Full calibration is required.");
      return;
    }

    bool valid = false;
    std::string reason;
    if (!validate_baseline(valid, reason, false)) {
      response->success = false;
      response->message = "Validation failed: " + reason;
      set_status(exo_interfaces::msg::CalibrationStatus::INVALID, "validation_data_insufficient");
      publish_user_message(
        exo_interfaces::msg::UserMessage::WARNING,
        "CALIBRATION_VALIDATION_INSUFFICIENT",
        "Calibration validation did not have enough data.");
      return;
    }

    if (valid) {
      response->success = true;
      response->message = "Calibration validation passed";
      set_status(exo_interfaces::msg::CalibrationStatus::VALID, "validation_pass:" + reason);
      publish_user_message(
        exo_interfaces::msg::UserMessage::INFO,
        "SYSTEM_READY_FOR_ASSISTIVE",
        "Calibration validated. System is ready to enter ASSISTIVE from IDLE.");
      return;
    }

    response->success = false;
    response->message = "Calibration validation failed";
    set_status(exo_interfaces::msg::CalibrationStatus::INVALID, "validation_fail:" + reason);
    calibration_mode_request_pending_ = true;
    publish_user_message(
      exo_interfaces::msg::UserMessage::WARNING,
      "CALIBRATION_INVALID",
      "Calibration validation failed. Recalibration is required.");
  }

  std::string fused_state_topic_;
  std::string gait_phase_topic_;
  std::string mode_state_topic_;
  std::string calibration_status_topic_;
  std::string run_full_calibration_service_;
  std::string validate_calibration_service_;
  std::string user_message_topic_;
  std::string set_mode_service_;

  double status_publish_rate_hz_;
  double analysis_window_s_;
  int64_t min_samples_;
  double max_velocity_ratio_delta_;
  double max_confidence_delta_;
  double max_contact_delta_;
  bool require_calibration_mode_for_full_;
  std::string baseline_store_path_;
  bool auto_load_baseline_;
  bool auto_save_baseline_;
  bool auto_validate_on_idle_startup_;
  double startup_validation_delay_s_;
  std::string startup_validation_mode_;
  bool auto_request_calibration_mode_;

  std::string current_mode_;
  bool has_baseline_;
  WindowFeatures baseline_;
  uint8_t current_status_;
  std::string current_detail_;
  bool startup_validation_pending_{false};
  bool calibration_mode_request_pending_{false};
  bool idle_tracking_active_{false};
  rclcpp::Time idle_since_;

  std::deque<TimedSample> velocity_samples_;
  std::deque<TimedSample> confidence_samples_;
  std::deque<TimedSample> contact_samples_;

  rclcpp::Subscription<exo_interfaces::msg::FusedLimbState>::SharedPtr fused_sub_;
  rclcpp::Subscription<exo_interfaces::msg::GaitPhase>::SharedPtr gait_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Publisher<exo_interfaces::msg::CalibrationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<exo_interfaces::msg::UserMessage>::SharedPtr user_message_pub_;
  rclcpp::Client<exo_interfaces::srv::SetOperatingMode>::SharedPtr mode_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr full_calibration_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr validate_calibration_srv_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalibrationNode>());
  rclcpp::shutdown();
  return 0;
}
