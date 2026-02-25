#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Exo_sensor_layerNode : public rclcpp::Node {
public:
  Exo_sensor_layerNode() : Node("sensor_layer_node") {
    timer_ = this->create_wall_timer(1000ms, [this]() {
      RCLCPP_DEBUG(this->get_logger(), "exo_sensor_layer heartbeat");
    });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Exo_sensor_layerNode>());
  rclcpp::shutdown();
  return 0;
}
