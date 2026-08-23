// teleop_override_node -- joystick with a deadman, at the top of the arbitration order.
//
// docs/ARCHITECTURE.md: "Joystick with a deadman button. Publishes manual velocity and the
// E-stop latch. Highest arbitration priority -- human input instantly preempts autonomy."
//
// Two behaviours are worth reading before changing anything:
//
// 1. Publishing runs on a fixed 50 Hz timer, not on Joy arrival. j10_safety arbitrates on
//    message *freshness*, so a manual command that stops being republished must go stale on
//    its own -- that is how releasing the deadman hands control back rather than leaving the
//    last manual command standing. Publishing only from the Joy callback would tie that
//    timing to a device driver's rate, which is not ours to depend on.
//
// 2. The E-stop is latched here and cleared only by an explicit gesture. A momentary topic
//    would unlatch the instant the button was released, which is the opposite of what an
//    emergency stop means.
//
// All the interesting logic lives in joystick_mapping.hpp, which has no ROS in it and is
// unit-tested exhaustively. This class is I/O and latching.

#ifndef J10_TELEOP__TELEOP_OVERRIDE_NODE_HPP_
#define J10_TELEOP__TELEOP_OVERRIDE_NODE_HPP_

#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

#include "j10_teleop/joystick_mapping.hpp"

namespace j10_teleop
{

class TeleopOverrideNode : public rclcpp::Node
{
public:
  explicit TeleopOverrideNode(const rclcpp::NodeOptions & options);

private:
  void onJoy(const sensor_msgs::msg::Joy & msg);
  void onTimer();
  void publishEstop();

  // Parameters
  double publish_rate_hz_{50.0};
  double joy_timeout_sec_{0.5};
  std::string cmd_vel_topic_;
  std::string deadman_topic_;
  std::string estop_topic_;
  std::string joy_topic_;
  std::string frame_id_;
  AxisMap map_;
  ShapingParams shaping_;

  // State
  std::mutex mutex_;
  TeleopCommand last_command_;
  rclcpp::Time last_joy_time_;
  bool have_joy_{false};
  bool estop_latched_{false};
  bool deadman_was_held_{false};

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr deadman_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace j10_teleop

#endif  // J10_TELEOP__TELEOP_OVERRIDE_NODE_HPP_
