#include <cmath>
#include <memory>

#include "exo_interfaces/msg/raw_sensor_data.hpp"
#include "rclcpp/rclcpp.hpp"

class SimulationNode : public rclcpp::Node {
public:
  SimulationNode() : Node("simulation_node"), phase_(0.0) {
    raw_sensor_topic_ = this->declare_parameter<std::string>("raw_sensor_topic", "/exo/sensing/raw");
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 200.0);
    gait_frequency_hz_ = this->declare_parameter<double>("gait_frequency_hz", 1.0);
    knee_amplitude_rad_ = this->declare_parameter<double>("knee_amplitude_rad", 0.6);

    raw_pub_ = this->create_publisher<exo_interfaces::msg::RawSensorData>(raw_sensor_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SimulationNode::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "Simulation publishing %s at %.1f Hz", raw_sensor_topic_.c_str(), publish_rate_hz_);
  }

private:
  void on_timer() {
    const double dt = 1.0 / std::max(1.0, publish_rate_hz_);
    phase_ += 2.0 * M_PI * gait_frequency_hz_ * dt;
    if (phase_ > 2.0 * M_PI) {
      phase_ -= 2.0 * M_PI;
    }

    exo_interfaces::msg::RawSensorData msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "knee_joint";
    msg.joint_angle_rad = knee_amplitude_rad_ * std::sin(phase_);
    msg.imu_orientation_pitch_rad = 0.5 * msg.joint_angle_rad;
    msg.imu_angular_velocity_y_rad_s =
      knee_amplitude_rad_ * 2.0 * M_PI * gait_frequency_hz_ * std::cos(phase_);
    msg.contact_state = std::sin(phase_) > 0.0;

    raw_pub_->publish(msg);
  }

  std::string raw_sensor_topic_;
  double publish_rate_hz_;
  double gait_frequency_hz_;
  double knee_amplitude_rad_;
  double phase_;

  rclcpp::Publisher<exo_interfaces::msg::RawSensorData>::SharedPtr raw_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulationNode>());
  rclcpp::shutdown();
  return 0;
}
