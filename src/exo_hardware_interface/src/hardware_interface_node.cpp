#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Exo_hardware_interfaceNode : public rclcpp::Node {
public:
  Exo_hardware_interfaceNode() : Node("hardware_interface_node") {
    timer_ = this->create_wall_timer(1000ms, [this]() {
      RCLCPP_DEBUG(this->get_logger(), "exo_hardware_interface heartbeat");
    });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Exo_hardware_interfaceNode>());
  rclcpp::shutdown();
  return 0;
}
