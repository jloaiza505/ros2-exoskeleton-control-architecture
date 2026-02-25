#include <algorithm>
#include <cmath>
#include <memory>

#include "exo_interfaces/msg/fused_limb_state.hpp"
#include "exo_interfaces/msg/gait_phase.hpp"
#include "rclcpp/rclcpp.hpp"

class GaitPhaseDetectorNode : public rclcpp::Node {
public:
  GaitPhaseDetectorNode() : Node("gait_phase_detector_node") {
    fused_state_topic_ = this->declare_parameter<std::string>("fused_state_topic", "/exo/state/fused");
    gait_phase_topic_ = this->declare_parameter<std::string>("gait_phase_topic", "/exo/gait/phase");
    contact_threshold_ = this->declare_parameter<double>("contact_threshold", 0.5);
    velocity_transition_threshold_ =
      this->declare_parameter<double>("velocity_transition_threshold_rad_s", 0.35);

    gait_pub_ = this->create_publisher<exo_interfaces::msg::GaitPhase>(gait_phase_topic_, 10);
    fused_sub_ = this->create_subscription<exo_interfaces::msg::FusedLimbState>(
      fused_state_topic_,
      10,
      std::bind(&GaitPhaseDetectorNode::on_fused_state, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Gait phase detector subscribed to %s", fused_state_topic_.c_str());
  }

private:
  void on_fused_state(const exo_interfaces::msg::FusedLimbState::SharedPtr msg) {
    exo_interfaces::msg::GaitPhase out;
    out.header = msg->header;

    const bool in_contact = msg->contact_probability >= contact_threshold_;
    const bool fast_motion = std::abs(msg->joint_velocity_rad_s) > velocity_transition_threshold_;

    if (fast_motion) {
      out.phase_label = exo_interfaces::msg::GaitPhase::TRANSITION;
    } else {
      out.phase_label = in_contact ? exo_interfaces::msg::GaitPhase::STANCE : exo_interfaces::msg::GaitPhase::SWING;
    }

    out.phase_continuous = std::clamp(msg->phase_progress_hint, 0.0, 1.0);
    out.confidence = std::clamp(2.0 * std::abs(msg->contact_probability - 0.5), 0.0, 1.0);

    gait_pub_->publish(out);
  }

  std::string fused_state_topic_;
  std::string gait_phase_topic_;
  double contact_threshold_;
  double velocity_transition_threshold_;

  rclcpp::Subscription<exo_interfaces::msg::FusedLimbState>::SharedPtr fused_sub_;
  rclcpp::Publisher<exo_interfaces::msg::GaitPhase>::SharedPtr gait_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GaitPhaseDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
