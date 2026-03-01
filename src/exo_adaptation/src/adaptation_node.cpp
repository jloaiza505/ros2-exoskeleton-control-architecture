#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "exo_interfaces/msg/adaptive_parameters.hpp"
#include "exo_interfaces/msg/fused_limb_state.hpp"
#include "exo_interfaces/msg/gait_phase.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class AdaptationNode : public rclcpp::Node {
public:
  AdaptationNode()
  : Node("adaptation_node"),
    has_fused_(false),
    has_phase_(false),
    current_mode_("OFFLINE")
  {
    fused_state_topic_ = this->declare_parameter<std::string>("fused_state_topic", "/exo/state/fused");
    gait_phase_topic_ = this->declare_parameter<std::string>("gait_phase_topic", "/exo/gait/phase");
    mode_state_topic_ = this->declare_parameter<std::string>("mode_state_topic", "/exo/system/mode");
    adaptive_parameters_topic_ = this->declare_parameter<std::string>(
      "adaptive_parameters_topic", "/exo/adaptation/parameters");

    publish_rate_hz_ = std::max(1.0, this->declare_parameter<double>("publish_rate_hz", 5.0));
    input_timeout_s_ = std::max(0.05, this->declare_parameter<double>("input_timeout_s", 0.5));
    nominal_velocity_rad_s_ = std::max(
      0.1, this->declare_parameter<double>("nominal_velocity_rad_s", 2.0));

    calibration_complete_ = this->declare_parameter<bool>("calibration_complete", true);

    base_assistance_gain_ = this->declare_parameter<double>("base_assistance_gain", 1.0);
    base_torque_profile_scale_ = this->declare_parameter<double>("base_torque_profile_scale", 1.0);
    base_timing_offset_s_ = this->declare_parameter<double>("base_timing_offset_s", 0.0);
    base_safety_scale_ = this->declare_parameter<double>("base_safety_scale", 1.0);

    min_assistance_gain_ = this->declare_parameter<double>("min_assistance_gain", 0.0);
    max_assistance_gain_ = this->declare_parameter<double>("max_assistance_gain", 1.5);
    min_torque_profile_scale_ = this->declare_parameter<double>("min_torque_profile_scale", 0.5);
    max_torque_profile_scale_ = this->declare_parameter<double>("max_torque_profile_scale", 1.5);
    min_timing_offset_s_ = this->declare_parameter<double>("min_timing_offset_s", -0.20);
    max_timing_offset_s_ = this->declare_parameter<double>("max_timing_offset_s", 0.20);
    min_safety_scale_ = this->declare_parameter<double>("min_safety_scale", 0.0);
    max_safety_scale_ = this->declare_parameter<double>("max_safety_scale", 1.0);

    patient_mass_kg_ = this->declare_parameter<double>("patient_mass_kg", 75.0);
    patient_height_m_ = this->declare_parameter<double>("patient_height_m", 1.75);
    exo_lever_length_m_ = this->declare_parameter<double>("exo_lever_length_m", 0.35);

    adaptation_pub_ = this->create_publisher<exo_interfaces::msg::AdaptiveParameters>(
      adaptive_parameters_topic_, 10);

    fused_sub_ = this->create_subscription<exo_interfaces::msg::FusedLimbState>(
      fused_state_topic_, 10, std::bind(&AdaptationNode::on_fused_state, this, std::placeholders::_1));
    phase_sub_ = this->create_subscription<exo_interfaces::msg::GaitPhase>(
      gait_phase_topic_, 10, std::bind(&AdaptationNode::on_gait_phase, this, std::placeholders::_1));
    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      mode_state_topic_, 10, std::bind(&AdaptationNode::on_mode, this, std::placeholders::_1));

    initialize_safe_parameters();

    const auto period_ms = std::chrono::milliseconds(
      static_cast<int>(std::lround(1000.0 / publish_rate_hz_)));
    timer_ = this->create_wall_timer(period_ms, std::bind(&AdaptationNode::on_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Adaptation publishing %s at %.1f Hz (calibration_complete=%s)",
      adaptive_parameters_topic_.c_str(),
      publish_rate_hz_,
      calibration_complete_ ? "true" : "false");
  }

private:
  static double clamp_value(double value, double min_value, double max_value)
  {
    return std::clamp(value, min_value, max_value);
  }

  void initialize_safe_parameters()
  {
    last_safe_.assistance_gain = clamp_value(base_assistance_gain_, min_assistance_gain_, max_assistance_gain_);
    last_safe_.torque_profile_scale = clamp_value(
      base_torque_profile_scale_, min_torque_profile_scale_, max_torque_profile_scale_);
    last_safe_.timing_offset_s = clamp_value(base_timing_offset_s_, min_timing_offset_s_, max_timing_offset_s_);
    last_safe_.safety_scale = clamp_value(base_safety_scale_, min_safety_scale_, max_safety_scale_);
  }

  void on_fused_state(const exo_interfaces::msg::FusedLimbState::SharedPtr msg)
  {
    last_fused_ = *msg;
    has_fused_ = true;
    last_fused_rx_time_ = this->now();
  }

  void on_gait_phase(const exo_interfaces::msg::GaitPhase::SharedPtr msg)
  {
    last_phase_ = *msg;
    has_phase_ = true;
    last_phase_rx_time_ = this->now();
  }

  void on_mode(const std_msgs::msg::String::SharedPtr msg)
  {
    current_mode_ = msg->data;
  }

  bool inputs_fresh() const
  {
    if (!has_fused_ || !has_phase_) {
      return false;
    }

    const rclcpp::Time now = this->now();
    const double fused_age_s = (now - last_fused_rx_time_).seconds();
    const double phase_age_s = (now - last_phase_rx_time_).seconds();
    return fused_age_s <= input_timeout_s_ && phase_age_s <= input_timeout_s_;
  }

  void on_timer()
  {
    exo_interfaces::msg::AdaptiveParameters out;
    out.header.stamp = this->now();
    out.header.frame_id = "knee_joint";

    if (!inputs_fresh()) {
      out.assistance_gain = last_safe_.assistance_gain;
      out.torque_profile_scale = last_safe_.torque_profile_scale;
      out.timing_offset_s = last_safe_.timing_offset_s;
      out.safety_scale = last_safe_.safety_scale;
      adaptation_pub_->publish(out);
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Adaptation inputs stale; holding last safe values");
      return;
    }

    const double confidence = clamp_value(last_phase_.confidence, 0.0, 1.0);
    const double contact_probability = clamp_value(last_fused_.contact_probability, 0.0, 1.0);
    const double velocity_abs = std::fabs(last_fused_.joint_velocity_rad_s);
    const double velocity_scale = clamp_value(velocity_abs / nominal_velocity_rad_s_, 0.6, 1.2);

    const double mass_scale = clamp_value(1.0 + 0.15 * ((patient_mass_kg_ - 75.0) / 75.0), 0.8, 1.2);
    const double height_scale = clamp_value(1.0 + 0.10 * ((patient_height_m_ - 1.75) / 1.75), 0.9, 1.1);
    const double lever_scale = clamp_value(
      1.0 + 0.10 * ((exo_lever_length_m_ - 0.35) / 0.35), 0.9, 1.1);
    const double anthropometric_scale = mass_scale * height_scale * lever_scale;

    double assistance_gain = base_assistance_gain_ * anthropometric_scale * velocity_scale;
    double torque_profile_scale = base_torque_profile_scale_ * (0.8 + 0.2 * confidence);
    double timing_offset_s = base_timing_offset_s_ + 0.02 * (0.5 - contact_probability);
    double safety_scale = base_safety_scale_ * (0.5 + 0.5 * confidence * contact_probability);

    if (!calibration_complete_) {
      assistance_gain *= 0.5;
      torque_profile_scale *= 0.8;
      safety_scale *= 0.5;
    }

    if (current_mode_ == "FAULT") {
      safety_scale = 0.0;
    }

    out.assistance_gain = clamp_value(assistance_gain, min_assistance_gain_, max_assistance_gain_);
    out.torque_profile_scale = clamp_value(
      torque_profile_scale, min_torque_profile_scale_, max_torque_profile_scale_);
    out.timing_offset_s = clamp_value(timing_offset_s, min_timing_offset_s_, max_timing_offset_s_);
    out.safety_scale = clamp_value(safety_scale, min_safety_scale_, max_safety_scale_);

    last_safe_.assistance_gain = out.assistance_gain;
    last_safe_.torque_profile_scale = out.torque_profile_scale;
    last_safe_.timing_offset_s = out.timing_offset_s;
    last_safe_.safety_scale = out.safety_scale;

    adaptation_pub_->publish(out);
  }

  std::string fused_state_topic_;
  std::string gait_phase_topic_;
  std::string mode_state_topic_;
  std::string adaptive_parameters_topic_;
  double publish_rate_hz_;
  double input_timeout_s_;
  double nominal_velocity_rad_s_;

  bool calibration_complete_;

  double base_assistance_gain_;
  double base_torque_profile_scale_;
  double base_timing_offset_s_;
  double base_safety_scale_;

  double min_assistance_gain_;
  double max_assistance_gain_;
  double min_torque_profile_scale_;
  double max_torque_profile_scale_;
  double min_timing_offset_s_;
  double max_timing_offset_s_;
  double min_safety_scale_;
  double max_safety_scale_;

  double patient_mass_kg_;
  double patient_height_m_;
  double exo_lever_length_m_;

  bool has_fused_;
  bool has_phase_;
  std::string current_mode_;
  rclcpp::Time last_fused_rx_time_;
  rclcpp::Time last_phase_rx_time_;
  exo_interfaces::msg::FusedLimbState last_fused_;
  exo_interfaces::msg::GaitPhase last_phase_;
  exo_interfaces::msg::AdaptiveParameters last_safe_;

  rclcpp::Subscription<exo_interfaces::msg::FusedLimbState>::SharedPtr fused_sub_;
  rclcpp::Subscription<exo_interfaces::msg::GaitPhase>::SharedPtr phase_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Publisher<exo_interfaces::msg::AdaptiveParameters>::SharedPtr adaptation_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdaptationNode>());
  rclcpp::shutdown();
  return 0;
}
