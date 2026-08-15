#ifndef J10_MAVLINK__MAVLINK_BRIDGE_NODE_HPP_
#define J10_MAVLINK__MAVLINK_BRIDGE_NODE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <j10_interfaces/msg/vehicle_state.hpp>
#include <j10_interfaces/srv/arm_disarm.hpp>

#include "j10_mavlink/frame_conventions.hpp"

namespace j10_mavlink
{

/// Sole owner of the flight-controller interface.
///
/// Converts `/j10/cmd_vel_safe` into a continuous `mavros_msgs/PositionTarget` stream on
/// `/mavros/setpoint_raw/local`, and owns GUIDED entry, arming, takeoff and mode-loss
/// detection.
///
/// Two invariants drive the whole design:
///
///  1. **The setpoint stream is never silent.** ArduPilot's guided-mode failsafe stops the
///     vehicle after a few seconds without a setpoint, so the timer publishes at a fixed
///     rate whether or not a command has arrived, whether or not the vehicle is armed.
///  2. **Stale input decays to zero, never to the last command.** If `/j10/cmd_vel_safe`
///     goes quiet for longer than `command_timeout_sec`, the bridge publishes zero
///     velocity. Repeating the last command on loss of input is how offboard systems fly
///     into walls.
///
/// @note This node performs blocking service calls (to `/mavros/set_mode`, `/mavros/cmd/*`)
///       from inside its own service callbacks, using separate mutually-exclusive callback
///       groups for servers and clients. It therefore **must** be spun by a
///       `MultiThreadedExecutor` — i.e. loaded into `component_container_mt`, not the
///       single-threaded `component_container`. The provided `mavlink_bridge_node`
///       executable does this correctly.
class MavlinkBridgeNode : public rclcpp::Node
{
public:
  explicit MavlinkBridgeNode(const rclcpp::NodeOptions & options);

private:
  // --- Parameters, resolved once at construction ---
  struct Params
  {
    double setpoint_rate_hz;
    double command_timeout_sec;
    double absolute_max_linear_mps;
    double absolute_max_yaw_rate_rps;
    BodyFrameConvention body_frame_convention;
    std::string guided_mode;
    double takeoff_altitude_m;
    double takeoff_timeout_sec;
    bool suppress_setpoints_during_takeoff;
    bool require_ekf_healthy_to_arm;
    bool allow_force_arm;
    double service_timeout_sec;
    double state_timeout_sec;
  };

  // --- Callbacks ---
  void onCmdVel(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void onVehicleState(const j10_interfaces::msg::VehicleState::SharedPtr msg);
  void onSetpointTimer();

  void handleArmDisarm(
    const std::shared_ptr<j10_interfaces::srv::ArmDisarm::Request> request,
    std::shared_ptr<j10_interfaces::srv::ArmDisarm::Response> response);
  void handleTakeoff(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleSetGuided(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // --- Helpers ---
  /// Blocking call into a MAVROS service. Safe only because clients live in their own
  /// callback group and the node is spun multi-threaded.
  template<typename SrvT>
  bool callService(
    const typename rclcpp::Client<SrvT>::SharedPtr & client,
    const typename SrvT::Request::SharedPtr & request,
    typename SrvT::Response::SharedPtr & response,
    std::string & error);

  bool requestMode(const std::string & mode, std::string & error);
  bool waitForMode(const std::string & mode, double timeout_sec);
  bool waitForArmed(bool armed, double timeout_sec);

  /// Snapshot of the last VehicleState, with a freshness verdict.
  j10_interfaces::msg::VehicleState vehicleState(bool & fresh) const;

  Params params_;

  // --- Interfaces ---
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<j10_interfaces::msg::VehicleState>::SharedPtr vehicle_state_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::TimerBase::SharedPtr setpoint_timer_;

  rclcpp::Service<j10_interfaces::srv::ArmDisarm>::SharedPtr arm_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_guided_srv_;

  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;

  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::CallbackGroup::SharedPtr sub_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr client_group_;

  // --- State ---
  mutable std::mutex cmd_mutex_;
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_;
  bool cmd_ever_received_{false};
  bool holding_zero_{true};       // currently publishing zeros because input is stale

  mutable std::mutex state_mutex_;
  j10_interfaces::msg::VehicleState last_state_;
  rclcpp::Time last_state_time_;
  bool state_ever_received_{false};

  std::atomic<bool> takeoff_in_progress_{false};
  std::atomic<uint64_t> setpoints_published_{0};
  /// True once we have successfully commanded GUIDED, so a later mode change is a loss.
  std::atomic<bool> guided_requested_{false};
  std::string last_seen_mode_;
};

}  // namespace j10_mavlink

#endif  // J10_MAVLINK__MAVLINK_BRIDGE_NODE_HPP_
