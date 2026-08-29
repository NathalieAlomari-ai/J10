// Standalone entry point for safety_filter_node.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "j10_safety/safety_filter_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<j10_safety::SafetyFilterNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
