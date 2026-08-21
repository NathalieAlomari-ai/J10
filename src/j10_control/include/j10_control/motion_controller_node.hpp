#ifndef J10_CONTROL__MOTION_CONTROLLER_NODE_HPP_
#define J10_CONTROL__MOTION_CONTROLLER_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <j10_interfaces/msg/latency_report.hpp>
#include <j10_interfaces/msg/nav_intent.hpp>

#include "j10_control/motion_shaper.hpp"

namespace j10_control
{

/// The fast layer of the two-rate cascade.
///
/// Holds the most recent NavIntent and converts it into a smooth 30 Hz body-frame velocity,
/// so that a 5-10 Hz model does not leave a 100-200 ms hole in the command stream every
/// cycle. Publishes on a timer rather than on intent arrival, precisely so that the
/// *absence* of intents is something the loop observes and acts on.
///
/// Also emits a CONTROL-stage LatencyReport per cycle that is following an intent, carrying
/// the originating camera frame's capture time. That is what lets the latency monitor stitch
/// the stages together and prove the end-to-end budget rather than assume it.
class MotionControllerNode : public rclcpp::Node
{
public:
  explicit MotionControllerNode(const rclcpp::NodeOptions & options);

private:
  void onIntent(const j10_interfaces::msg::NavIntent::SharedPtr msg);
  void onTimer();
  ShaperLimits loadLimits();

  MotionShaper shaper_;

  rclcpp::Subscription<j10_interfaces::msg::NavIntent>::SharedPtr intent_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<j10_interfaces::msg::LatencyReport>::SharedPtr latency_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  j10_interfaces::msg::NavIntent intent_;
  bool intent_received_{false};

  rclcpp::Time last_step_;
  std::string frame_id_;
  double rate_hz_{30.0};
  uint64_t sequence_{0};
  uint8_t last_reason_{255};
};

}  // namespace j10_control

#endif  // J10_CONTROL__MOTION_CONTROLLER_NODE_HPP_
