#ifndef J10_SAFETY__SAFETY_FILTER_NODE_HPP_
#define J10_SAFETY__SAFETY_FILTER_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <j10_interfaces/msg/safety_status.hpp>
#include <j10_interfaces/msg/stream_status.hpp>
#include <j10_interfaces/msg/vehicle_state.hpp>

#include "j10_safety/safety_filter.hpp"

namespace j10_safety
{

/// ROS wrapper around SafetyFilter.
///
/// Holds no safety logic of its own: it collects inputs, timestamps them, hands a plain
/// FilterInputs to the filter, and publishes the verdict. All the actual limits live in
/// SafetyFilter, which is testable without any of this.
///
/// Runs at a fixed rate rather than reacting to input, so that loss of input is itself an
/// event the filter sees -- an event-driven design would simply stop producing output when
/// its input died, which is the one behaviour a safety layer must never have.
class SafetyFilterNode : public rclcpp::Node
{
public:
  explicit SafetyFilterNode(const rclcpp::NodeOptions & options);

private:
  void onTimer();
  void onAutonomousCmd(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void onManualCmd(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void onEstop(const std_msgs::msg::Bool::SharedPtr msg);
  void onDeadman(const std_msgs::msg::Bool::SharedPtr msg);
  void onAutonomyEnabled(const std_msgs::msg::Bool::SharedPtr msg);
  void onVehicleState(const j10_interfaces::msg::VehicleState::SharedPtr msg);
  void onStreamStatus(const j10_interfaces::msg::StreamStatus::SharedPtr msg);

  void handleResetEstop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  Limits loadLimits();

  SafetyFilter filter_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr autonomous_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr manual_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomy_sub_;
  rclcpp::Subscription<j10_interfaces::msg::VehicleState>::SharedPtr vehicle_sub_;
  rclcpp::Subscription<j10_interfaces::msg::StreamStatus>::SharedPtr stream_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<j10_interfaces::msg::SafetyStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_estop_srv_;

  std::mutex mutex_;
  geometry_msgs::msg::Twist autonomous_cmd_;
  rclcpp::Time autonomous_time_;
  bool autonomous_received_{false};

  geometry_msgs::msg::Twist manual_cmd_;
  rclcpp::Time manual_time_;
  bool manual_received_{false};

  bool deadman_held_{false};
  bool estop_engaged_{false};
  bool autonomy_enabled_{false};

  j10_interfaces::msg::VehicleState vehicle_state_;
  rclcpp::Time vehicle_time_;
  bool vehicle_received_{false};

  rclcpp::Time stream_time_;
  bool stream_ok_{false};
  bool stream_received_{false};

  rclcpp::Time last_step_;
  std::string frame_id_;
  double rate_hz_{30.0};
  uint8_t last_reported_state_{255};
};

}  // namespace j10_safety

#endif  // J10_SAFETY__SAFETY_FILTER_NODE_HPP_
