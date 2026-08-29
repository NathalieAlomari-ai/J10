// Standalone entry point for teleop_override_node.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "j10_teleop/teleop_override_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<j10_teleop::TeleopOverrideNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
