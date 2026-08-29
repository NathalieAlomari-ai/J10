// Factory for VideoReceiverNode.
//
// The node class itself stays inside video_receiver_node.cpp: its members are GStreamer
// types, and declaring it in a header would push a gst dependency onto everything that
// includes it. This one function is all the standalone executable needs, and it keeps a
// single definition of the node shared with the component registration.

#ifndef J10_VIDEO__VIDEO_RECEIVER_FACTORY_HPP_
#define J10_VIDEO__VIDEO_RECEIVER_FACTORY_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace j10_video
{

/// Construct a VideoReceiverNode. Returns the base Node so the header stays gst-free.
std::shared_ptr<rclcpp::Node> createVideoReceiverNode(const rclcpp::NodeOptions & options);

}  // namespace j10_video

#endif  // J10_VIDEO__VIDEO_RECEIVER_FACTORY_HPP_
