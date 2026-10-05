// Standalone entry point for video_receiver_node.
//
// Goes through the factory rather than naming the class, so this file needs no GStreamer
// headers and there remains exactly one definition of the node, shared with the component
// registration in video_receiver_node.cpp.

#include <rclcpp/rclcpp.hpp>

#include "j10_video/video_receiver_factory.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(j10_video::createVideoReceiverNode(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
