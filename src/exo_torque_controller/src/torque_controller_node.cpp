#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "exo_interfaces/msg/adaptive_parameters.hpp"
#include "exo_interfaces/msg/fused_limb_state.hpp"
#include "exo_interfaces/msg/gait_phase.hpp"
#include "exo_interfaces/msg/torque_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TorqueControllerNode : public rclcpp::Node {
public:
  TorqueControllerNode()
  : Node("torque_controller_node"),
    has_fused_(false),
    has_phase_(false),
    has_adaptation_(false),
    current_mode_("OFFLINE")
  {
    fused_state_topic_ = this->declare_parameter<std::string>("fused_state_topic", "/exo/state/fused");
    gait_phase_topic_ = this->declare_parameter<std::string>("gait_phase_topic", "/exo/gait/phase");
    mode_state_topic_ = this->declare_parameter<std::string>("mode_state_topic", "/exo/system/mode");
    adaptive_parameters_topic_ = this->declare_parameter<std::string>(
      "adaptive_parameters_topic", "/exo/adaptation/parameters");
    torque_command_topic_ = this->declare_parameter<std::string>("torque_command_topic", "/exo/control/torque_command");
    max_torque_nm_ = this->declare_parameter<double>("max_torque_nm", 25.0);
    velocity_damping_gain_ = this->declare_parameter<double>("velocity_damping_gain", 1.5);
    adaptation_timeout_s_ = std::max(0.05, this->declare_parameter<double>("adaptation_timeout_s", 0.5));
    nominal_gait_cycle_s_ = std::max(0.2, this->declare_parameter<double>("nominal_gait_cycle_s", 1.2));

    torque_pub_ = this->create_publisher<exo_interfaces::msg::TorqueCommand>(torque_command_topic_, 10);

    fused_sub_ = this->create_subscription<exo_interfaces::msg::FusedLimbState>(
      fused_state_topic_, 10, std::bind(&TorqueControllerNode::on_fused_state, this, std::placeholders::_1));

    phase_sub_ = this->create_subscription<exo_interfaces::msg::GaitPhase>(
      gait_phase_topic_, 10, std::bind(&TorqueControllerNode::on_phase, this, std::placeholders::_1));

    adaptation_sub_ = this->create_subscription<exo_interfaces::msg::AdaptiveParameters>(
      adaptive_parameters_topic_, 10, std::bind(&TorqueControllerNode::on_adaptation, this, std::placeholders::_1));

    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      mode_state_topic_, 10, std::bind(&TorqueControllerNode::on_mode, this, std::placeholders::_1));

    // Before mode manager publishes, keep controller in a safe pre-operational state.
    publish_command(0.0, false, false);

    RCLCPP_INFO(
      this->get_logger(),
      "Torque controller publishing %s (mode topic: %s, adaptation topic: %s, initial_mode=%s)",
      torque_command_topic_.c_str(),
      mode_state_topic_.c_str(),
      adaptive_parameters_topic_.c_str(),
      current_mode_.c_str());
  }

private:
  static double clamp_unit(double value) {
    return std::clamp(value, 0.0, 1.0);
  }

  double compute_phase_scale(uint8_t phase_label, double phase_continuous) const {
    const double phase = clamp_unit(phase_continuous);
    if (phase_label == exo_interfaces::msg::GaitPhase::STANCE) {
      return 0.6 + 0.4 * phase;
    }
    if (phase_label == exo_interfaces::msg::GaitPhase::TRANSITION) {
      return 0.35;
    }
    return 0.1;
  }

  void on_fused_state(const exo_interfaces::msg::FusedLimbState::SharedPtr msg) {
    last_fused_ = *msg;
    has_fused_ = true;
    maybe_publish_torque();
  }

  void on_phase(const exo_interfaces::msg::GaitPhase::SharedPtr msg) {
    last_phase_ = *msg;
    has_phase_ = true;
    maybe_publish_torque();
  }

  void on_adaptation(const exo_interfaces::msg::AdaptiveParameters::SharedPtr msg) {
    last_adaptation_ = *msg;
    has_adaptation_ = true;
    last_adaptation_rx_time_ = this->now();
  }

  void on_mode(const std_msgs::msg::String::SharedPtr msg) {
    const std::string previous_mode = current_mode_;
    current_mode_ = msg->data;

    if (current_mode_ != previous_mode) {
      RCLCPP_INFO(
        this->get_logger(), "Mode update: %s -> %s", previous_mode.c_str(), current_mode_.c_str());
    }

    // Force immediate zero-torque publish on non-assistive modes.
    if (current_mode_ != "ASSISTIVE") {
      publish_command(0.0, false, current_mode_ != "FAULT");
    }
  }

  void maybe_publish_torque() {
    if (!has_fused_ || !has_phase_) {
      return;
    }

    if (current_mode_ != "ASSISTIVE") {
      publish_command(0.0, false, current_mode_ != "FAULT");
      return;
    }

    double assistance_gain = 1.0;
    double torque_profile_scale = 1.0;
    double timing_offset_s = 0.0;
    double safety_scale = 1.0;

    bool adaptation_fresh = false;
    if (has_adaptation_) {
      const double adaptation_age_s = (this->now() - last_adaptation_rx_time_).seconds();
      adaptation_fresh = adaptation_age_s <= adaptation_timeout_s_;
    }

    if (adaptation_fresh) {
      assistance_gain = std::clamp(last_adaptation_.assistance_gain, 0.0, 2.0);
      torque_profile_scale = std::clamp(last_adaptation_.torque_profile_scale, 0.0, 2.0);
      timing_offset_s = std::clamp(last_adaptation_.timing_offset_s, -0.5, 0.5);
      safety_scale = std::clamp(last_adaptation_.safety_scale, 0.0, 1.0);
    } else if (has_adaptation_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Adaptive parameters stale; using neutral controller scaling");
    }

    const double phase_offset = timing_offset_s / nominal_gait_cycle_s_;
    const double adjusted_phase = clamp_unit(last_phase_.phase_continuous + phase_offset);
    const double phase_scale = compute_phase_scale(last_phase_.phase_label, adjusted_phase);

    double desired = max_torque_nm_ * phase_scale * assistance_gain * torque_profile_scale * safety_scale;
    desired -= velocity_damping_gain_ * last_fused_.joint_velocity_rad_s;

    const double bounded = std::clamp(desired, -max_torque_nm_, max_torque_nm_);
    publish_command(bounded, bounded != desired, true);
  }

  void publish_command(double desired_torque_nm, bool saturated, bool safety_ok) {
    exo_interfaces::msg::TorqueCommand out;
    out.header.stamp = this->now();
    out.header.frame_id = "knee_joint";
    out.desired_torque_nm = desired_torque_nm;
    out.saturated = saturated;
    out.safety_ok = safety_ok;
    torque_pub_->publish(out);
  }

  std::string fused_state_topic_;
  std::string gait_phase_topic_;
  std::string mode_state_topic_;
  std::string adaptive_parameters_topic_;
  std::string torque_command_topic_;
  double max_torque_nm_;
  double velocity_damping_gain_;
  double adaptation_timeout_s_;
  double nominal_gait_cycle_s_;

  bool has_fused_;
  bool has_phase_;
  bool has_adaptation_;
  std::string current_mode_;
  exo_interfaces::msg::FusedLimbState last_fused_;
  exo_interfaces::msg::GaitPhase last_phase_;
  exo_interfaces::msg::AdaptiveParameters last_adaptation_;
  rclcpp::Time last_adaptation_rx_time_;

  rclcpp::Subscription<exo_interfaces::msg::FusedLimbState>::SharedPtr fused_sub_;
  rclcpp::Subscription<exo_interfaces::msg::GaitPhase>::SharedPtr phase_sub_;
  rclcpp::Subscription<exo_interfaces::msg::AdaptiveParameters>::SharedPtr adaptation_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Publisher<exo_interfaces::msg::TorqueCommand>::SharedPtr torque_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TorqueControllerNode>());
  rclcpp::shutdown();
  return 0;
}
