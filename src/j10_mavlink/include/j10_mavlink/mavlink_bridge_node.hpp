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
///  1. **Stale input decays to zero, never to the last command.** If `/j10/cmd_vel_safe`
///     goes quiet for longer than `command_timeout_sec`, the bridge publishes zero
///     velocity. Repeating the last command on loss of input is how offboard systems fly
///     into walls.
///  2. **The setpoint stream is never silent while the PC-side chain is alive, and
///     deliberately goes silent once it is not.** These are two different failures and the
///     bridge treats them differently:
///
///       - *Upstream is slow or stalled, but this process and the link are healthy.*
///         Keep streaming at a fixed rate (zeros, per invariant 1). ArduPilot's guided
///         failsafe must NOT trip here — nothing is actually wrong with the vehicle.
///
///       - *The PC-side chain is gone.* `/j10/cmd_vel_safe` is published at a fixed rate by
///         `safety_filter_node` regardless of its own inputs, so that topic falling silent
///         for longer than `stream_stop_timeout_sec` means the filter itself died, the
///         process was killed, or the link dropped. The bridge then **stops publishing
///         entirely** and lets the flight controller's own failsafe take the vehicle.
///
///     The second case is a requirement, not a refinement: task A specifies "stop commands
///     on stream drop -> FC failsafe takes over", with the KPI "Commands stop <= 1 s after
///     link loss; FC failsafe verified". A bridge that keeps emitting zeros forever
///     suppresses the very failsafe that KPI asks to demonstrate.
///
///     The stop latches. Once the flight controller has begun its failsafe response,
///     resuming setpoints underneath it would fight the landing, so recovery is an explicit
///     operator action on `/j10/vehicle/resume_stream`.
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
    double max_setpoint_suppression_sec;
    bool require_ekf_healthy_to_arm;
    bool allow_force_arm;
    double service_timeout_sec;
    double state_timeout_sec;
    bool link_loss_stop_enabled;
    double stream_stop_timeout_sec;
    bool stream_stop_latching;
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
  /// Deliberately stop the setpoint stream so the flight controller's failsafe takes over.
  /// This is what a video-loss or E-stop decision upstream should call.
  void handleStopStream(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  /// Clear the stop latch and resume streaming. Explicit operator action, by design.
  void handleResumeStream(
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

  /// Latch the stream stopped and say so once. Idempotent.
  void stopStream(const std::string & reason);

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
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_stream_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_stream_srv_;

  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;

  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::CallbackGroup::SharedPtr sub_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  /// Takeoff gets its own mutually-exclusive group. It blocks for up to
  /// takeoff_timeout_sec polling for altitude, and sharing service_group_ with
  /// arm/disarm meant that for the whole of a 30 s climb there was no way to disarm.
  rclcpp::CallbackGroup::SharedPtr takeoff_group_;
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

  /// Set once the stream has been deliberately stopped. Latching: cleared only by
  /// `/j10/vehicle/resume_stream`, so a failsafe already in progress is never fought.
  std::atomic<bool> stream_stopped_{false};
  mutable std::mutex stop_mutex_;
  std::string stream_stop_reason_;

  std::atomic<bool> takeoff_in_progress_{false};
  /// steady_clock nanoseconds at which the current takeoff began, so the setpoint
  /// suppression window can be bounded independently of takeoff_timeout_sec.
  std::atomic<int64_t> takeoff_started_ns_{0};
  std::atomic<uint64_t> setpoints_published_{0};
  /// True once we have successfully commanded GUIDED, so a later mode change is a loss.
  std::atomic<bool> guided_requested_{false};
  std::string last_seen_mode_;
};

}  // namespace j10_mavlink

#endif  // J10_MAVLINK__MAVLINK_BRIDGE_NODE_HPP_
