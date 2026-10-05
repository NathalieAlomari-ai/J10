// Standalone entry point for vehicle_state_node.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "j10_mavlink/vehicle_state_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<j10_mavlink::VehicleStateNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
