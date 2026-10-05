// Standalone entry point for motion_controller_node.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "j10_control/motion_controller_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<j10_control::MotionControllerNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
