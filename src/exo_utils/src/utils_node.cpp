#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Exo_utilsNode : public rclcpp::Node {
public:
  Exo_utilsNode() : Node("utils_node") {
    timer_ = this->create_wall_timer(1000ms, [this]() {
      RCLCPP_DEBUG(this->get_logger(), "exo_utils heartbeat");
    });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Exo_utilsNode>());
  rclcpp::shutdown();
  return 0;
}
