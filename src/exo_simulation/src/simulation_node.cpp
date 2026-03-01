#include <cmath>
#include <memory>
#include <string>

#include "exo_interfaces/msg/user_message.hpp"
#include "exo_interfaces/msg/raw_sensor_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class SimulationNode : public rclcpp::Node {
public:
  SimulationNode() : Node("simulation_node"), walking_enabled_(true), phase_(0.0) {
    raw_sensor_topic_ = this->declare_parameter<std::string>("raw_sensor_topic", "/exo/sensing/raw");
    user_message_topic_ = this->declare_parameter<std::string>("user_message_topic", "/exo/system/user_message");
    start_simulation_service_ = this->declare_parameter<std::string>("start_simulation_service", "/exo/system/start_simulation");
    stop_simulation_service_ = this->declare_parameter<std::string>("stop_simulation_service", "/exo/system/stop_simulation");
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 200.0);
    gait_frequency_hz_ = this->declare_parameter<double>("gait_frequency_hz", 1.0);
    knee_amplitude_rad_ = this->declare_parameter<double>("knee_amplitude_rad", 0.6);
    auto_start_walking_ = this->declare_parameter<bool>("auto_start_walking", true);
    walking_enabled_ = auto_start_walking_;

    raw_pub_ = this->create_publisher<exo_interfaces::msg::RawSensorData>(raw_sensor_topic_, 10);
    user_message_pub_ = this->create_publisher<exo_interfaces::msg::UserMessage>(user_message_topic_, 10);

    start_srv_ = this->create_service<std_srvs::srv::Trigger>(
      start_simulation_service_,
      std::bind(&SimulationNode::on_start_simulation, this, std::placeholders::_1, std::placeholders::_2));
    stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
      stop_simulation_service_,
      std::bind(&SimulationNode::on_stop_simulation, this, std::placeholders::_1, std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SimulationNode::on_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Simulation publishing %s at %.1f Hz (walking_enabled=%s)",
      raw_sensor_topic_.c_str(),
      publish_rate_hz_,
      walking_enabled_ ? "true" : "false");
  }

private:
  void publish_user_message(uint8_t severity, const std::string & code, const std::string & text) {
    exo_interfaces::msg::UserMessage msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.severity = severity;
    msg.code = code;
    msg.text = text;
    user_message_pub_->publish(msg);
  }

  void on_start_simulation(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    walking_enabled_ = true;
    response->success = true;
    response->message = "Simulation walking started";
    publish_user_message(
      exo_interfaces::msg::UserMessage::INFO,
      "SIMULATION_STARTED",
      "Simulation walking started.");
  }

  void on_stop_simulation(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    walking_enabled_ = false;
    response->success = true;
    response->message = "Simulation walking stopped";
    publish_user_message(
      exo_interfaces::msg::UserMessage::WARNING,
      "SIMULATION_STOPPED",
      "Simulation walking stopped.");
  }

  void on_timer() {
    if (!walking_enabled_) {
      return;
    }

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
  std::string user_message_topic_;
  std::string start_simulation_service_;
  std::string stop_simulation_service_;
  double publish_rate_hz_;
  double gait_frequency_hz_;
  double knee_amplitude_rad_;
  bool auto_start_walking_;
  bool walking_enabled_;
  double phase_;

  rclcpp::Publisher<exo_interfaces::msg::RawSensorData>::SharedPtr raw_pub_;
  rclcpp::Publisher<exo_interfaces::msg::UserMessage>::SharedPtr user_message_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulationNode>());
  rclcpp::shutdown();
  return 0;
}
