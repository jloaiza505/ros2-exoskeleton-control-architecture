#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Exo_adaptationNode : public rclcpp::Node {
public:
  Exo_adaptationNode() : Node("adaptation_node") {
    timer_ = this->create_wall_timer(1000ms, [this]() {
      RCLCPP_DEBUG(this->get_logger(), "exo_adaptation heartbeat");
    });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Exo_adaptationNode>());
  rclcpp::shutdown();
  return 0;
}
