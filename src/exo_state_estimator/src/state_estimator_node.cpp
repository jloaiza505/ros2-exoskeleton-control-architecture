#include <algorithm>
#include <cmath>
#include <memory>

#include "exo_interfaces/msg/fused_limb_state.hpp"
#include "exo_interfaces/msg/raw_sensor_data.hpp"
#include "rclcpp/rclcpp.hpp"

class StateEstimatorNode : public rclcpp::Node {
public:
  StateEstimatorNode()
  : Node("state_estimator_node"),
    has_previous_(false),
    prev_angle_(0.0),
    prev_velocity_(0.0),
    filtered_contact_prob_(0.0) {
    raw_sensor_topic_ = this->declare_parameter<std::string>("raw_sensor_topic", "/exo/sensing/raw");
    fused_state_topic_ = this->declare_parameter<std::string>("fused_state_topic", "/exo/state/fused");
    contact_alpha_ = this->declare_parameter<double>("contact_alpha", 0.2);

    fused_pub_ = this->create_publisher<exo_interfaces::msg::FusedLimbState>(fused_state_topic_, 10);
    raw_sub_ = this->create_subscription<exo_interfaces::msg::RawSensorData>(
      raw_sensor_topic_,
      10,
      std::bind(&StateEstimatorNode::on_raw_sensor, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "State estimator subscribed to %s", raw_sensor_topic_.c_str());
  }

private:
  void on_raw_sensor(const exo_interfaces::msg::RawSensorData::SharedPtr msg) {
    const rclcpp::Time stamp = msg->header.stamp;

    double velocity = 0.0;
    double acceleration = 0.0;

    if (has_previous_) {
      const double dt = (stamp - prev_stamp_).seconds();
      if (dt > 1e-6) {
        velocity = (msg->joint_angle_rad - prev_angle_) / dt;
        acceleration = (velocity - prev_velocity_) / dt;
      }
    }

    const double contact_measurement = msg->contact_state ? 1.0 : 0.0;
    filtered_contact_prob_ =
      std::clamp((1.0 - contact_alpha_) * filtered_contact_prob_ + contact_alpha_ * contact_measurement, 0.0, 1.0);

    exo_interfaces::msg::FusedLimbState out;
    out.header = msg->header;
    out.joint_angle_rad = msg->joint_angle_rad;
    out.joint_velocity_rad_s = velocity;
    out.joint_acceleration_rad_s2 = acceleration;
    out.contact_probability = filtered_contact_prob_;

    constexpr double two_pi = 2.0 * M_PI;
    const double normalized = std::fmod(msg->joint_angle_rad + two_pi, two_pi) / two_pi;
    out.phase_progress_hint = std::clamp(normalized, 0.0, 1.0);

    fused_pub_->publish(out);

    prev_stamp_ = stamp;
    prev_angle_ = msg->joint_angle_rad;
    prev_velocity_ = velocity;
    has_previous_ = true;
  }

  std::string raw_sensor_topic_;
  std::string fused_state_topic_;
  double contact_alpha_;

  bool has_previous_;
  rclcpp::Time prev_stamp_;
  double prev_angle_;
  double prev_velocity_;
  double filtered_contact_prob_;

  rclcpp::Subscription<exo_interfaces::msg::RawSensorData>::SharedPtr raw_sub_;
  rclcpp::Publisher<exo_interfaces::msg::FusedLimbState>::SharedPtr fused_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
