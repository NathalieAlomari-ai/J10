// Implementation of TeleopOverrideNode. See the header for why publishing is timer-driven
// and why the E-stop latches.

#include "j10_teleop/teleop_override_node.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

namespace j10_teleop
{

TeleopOverrideNode::TeleopOverrideNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("teleop_override_node", options)
{
  publish_rate_hz_ = declare_parameter("publish_rate_hz", 50.0);
  joy_timeout_sec_ = declare_parameter("joy_timeout_sec", 0.5);
  frame_id_ = declare_parameter("frame_id", std::string("base_link"));

  cmd_vel_topic_ = declare_parameter("cmd_vel_topic", std::string("/j10/cmd_vel_manual"));
  deadman_topic_ = declare_parameter("deadman_topic", std::string("/j10/teleop/deadman"));
  estop_topic_ = declare_parameter("estop_topic", std::string("/j10/safety/estop"));
  joy_topic_ = declare_parameter("joy_topic", std::string("/joy"));

  map_.forward = static_cast<int>(declare_parameter("axis_forward", 1));
  map_.left = static_cast<int>(declare_parameter("axis_left", 0));
  map_.up = static_cast<int>(declare_parameter("axis_up", 4));
  map_.yaw = static_cast<int>(declare_parameter("axis_yaw", 3));
  map_.invert_forward = declare_parameter("invert_forward", false);
  map_.invert_left = declare_parameter("invert_left", false);
  map_.invert_up = declare_parameter("invert_up", false);
  map_.invert_yaw = declare_parameter("invert_yaw", false);
  map_.deadman_button = static_cast<int>(declare_parameter("deadman_button", 4));
  map_.estop_button = static_cast<int>(declare_parameter("estop_button", 1));
  map_.estop_reset_button = static_cast<int>(declare_parameter("estop_reset_button", -1));

  shaping_.deadzone = declare_parameter("deadzone", 0.08);
  shaping_.expo = declare_parameter("expo", 0.5);
  shaping_.max_linear_mps = declare_parameter("max_linear_mps", 0.5);
  shaping_.max_vertical_mps = declare_parameter("max_vertical_mps", 0.3);
  shaping_.max_yaw_rate_rps = declare_parameter("max_yaw_rate_rps", 0.8);

  last_joy_time_ = now();

  // Match what j10_safety subscribes with: the manual command and deadman are realtime
  // signals, the E-stop is latched so a node starting later still learns it is engaged.
  const auto realtime_qos = rclcpp::QoS(1).best_effort();
  const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    cmd_vel_topic_, realtime_qos);
  deadman_pub_ = create_publisher<std_msgs::msg::Bool>(deadman_topic_, realtime_qos);
  estop_pub_ = create_publisher<std_msgs::msg::Bool>(estop_topic_, latched_qos);

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    joy_topic_, rclcpp::QoS(1).best_effort(),
    [this](const sensor_msgs::msg::Joy::SharedPtr msg) {onJoy(*msg);});

  // Publish the initial (not-stopped) E-stop value immediately. A latched topic with
  // nothing on it is indistinguishable from a node that has not started yet.
  publishEstop();

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 1.0)),
    [this]() {onTimer();});

  RCLCPP_INFO(
    get_logger(),
    "teleop_override_node up: %.0f Hz, joy=%s -> %s (deadman button %d, estop button %d)",
    publish_rate_hz_, joy_topic_.c_str(), cmd_vel_topic_.c_str(),
    map_.deadman_button, map_.estop_button);
  RCLCPP_INFO(
    get_logger(), "  limits: %.2f m/s lateral, %.2f m/s vertical, %.2f rad/s yaw",
    shaping_.max_linear_mps, shaping_.max_vertical_mps, shaping_.max_yaw_rate_rps);
  if (map_.estop_reset_button < 0) {
    RCLCPP_INFO(
      get_logger(),
      "  E-stop reset is not bound to a button; use the /j10/safety/reset_estop service.");
  }
}

void TeleopOverrideNode::onJoy(const sensor_msgs::msg::Joy & msg)
{
  const auto command = interpret(msg.axes, msg.buttons, map_, shaping_);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_command_ = command;
    last_joy_time_ = now();
    have_joy_ = true;
  }

  // Edge-triggered, so holding a button neither spams the log nor republishes a latch that
  // is already set.
  if (command.estop_pressed && !estop_latched_) {
    estop_latched_ = true;
    RCLCPP_ERROR(get_logger(), "E-STOP engaged from joystick");
    publishEstop();
  } else if (command.estop_reset_requested && estop_latched_) {
    estop_latched_ = false;
    RCLCPP_WARN(get_logger(), "E-stop released from joystick");
    publishEstop();
  }

  if (command.deadman_held != deadman_was_held_) {
    RCLCPP_INFO(
      get_logger(), "deadman %s -- manual control %s",
      command.deadman_held ? "HELD" : "released",
      command.deadman_held ? "engaged" : "handed back");
    deadman_was_held_ = command.deadman_held;
  }
}

void TeleopOverrideNode::onTimer()
{
  TeleopCommand command;  // defaults to released deadman and zero velocity
  bool joy_is_fresh = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (have_joy_) {
      const double age = (now() - last_joy_time_).seconds();
      joy_is_fresh = age <= joy_timeout_sec_;
      if (joy_is_fresh) {
        command = last_command_;
      }
    }
  }

  // A silent joystick is treated as a released deadman, never as the last command held. An
  // unplugged controller or a crashed driver has to hand control back, and the only way to
  // tell "still holding the stick" from "the device is gone" is that messages stopped.
  if (!joy_is_fresh && deadman_was_held_) {
    RCLCPP_WARN(
      get_logger(), "joystick silent for more than %.2fs -- releasing manual control",
      joy_timeout_sec_);
    deadman_was_held_ = false;
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = frame_id_;
  cmd.twist.linear.x = command.velocity.x;
  cmd.twist.linear.y = command.velocity.y;
  cmd.twist.linear.z = command.velocity.z;
  cmd.twist.angular.z = command.velocity.yaw_rate;
  cmd_vel_pub_->publish(cmd);

  std_msgs::msg::Bool deadman;
  deadman.data = command.deadman_held;
  deadman_pub_->publish(deadman);
}

void TeleopOverrideNode::publishEstop()
{
  std_msgs::msg::Bool msg;
  msg.data = estop_latched_;
  estop_pub_->publish(msg);
}

}  // namespace j10_teleop

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(j10_teleop::TeleopOverrideNode)
