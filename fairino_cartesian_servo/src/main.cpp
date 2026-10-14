#include <rclcpp/rclcpp.hpp>

#include "fairino_cartesian_servo/cartesian_servo_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fairino_cartesian_servo::CartesianServoNode>();
  node->start();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
  executor.add_node(node);
  executor.spin();
  node->stop();
  rclcpp::shutdown();
  return 0;
}
