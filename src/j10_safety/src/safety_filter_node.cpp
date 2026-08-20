#include "j10_safety/safety_filter_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <utility>

namespace j10_safety
{

namespace
{

using SafetyStatusMsg = j10_interfaces::msg::SafetyStatus;

// The filter's SafetyState is a standalone enum so the safety logic stays free of ROS.
// These assertions are what stop the two from silently drifting apart if the message
// changes -- a mismatch would mean the published status no longer describes what the
// filter actually did.
static_assert(
  static_cast<uint8_t>(SafetyState::kNominal) == SafetyStatusMsg::STATE_NOMINAL,
  "SafetyState::kNominal must match SafetyStatus::STATE_NOMINAL");
static_assert(
  static_cast<uint8_t>(SafetyState::kLimited) == SafetyStatusMsg::STATE_LIMITED,
  "SafetyState::kLimited must match SafetyStatus::STATE_LIMITED");
static_assert(
  static_cast<uint8_t>(SafetyState::kBraking) == SafetyStatusMsg::STATE_BRAKING,
  "SafetyState::kBraking must match SafetyStatus::STATE_BRAKING");
static_assert(
  static_cast<uint8_t>(SafetyState::kLanding) == SafetyStatusMsg::STATE_LANDING,
  "SafetyState::kLanding must match SafetyStatus::STATE_LANDING");
static_assert(
  static_cast<uint8_t>(SafetyState::kEstop) == SafetyStatusMsg::STATE_ESTOP,
  "SafetyState::kEstop must match SafetyStatus::STATE_ESTOP");

std::chrono::nanoseconds periodFromRate(double rate_hz, double fallback_hz)
{
  const double hz = (rate_hz > 0.0) ? rate_hz : fallback_hz;
  return std::chrono::nanoseconds(static_cast<int64_t>(1e9 / hz));
}

/// Extract yaw from a quaternion. Only yaw matters here -- the geofence is a vertical
/// prism, so roll and pitch do not affect which way "body forward" points on the ground.
double yawFromQuaternion(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

const char * stateName(uint8_t state)
{
  switch (state) {
    case SafetyStatusMsg::STATE_NOMINAL: return "NOMINAL";
    case SafetyStatusMsg::STATE_LIMITED: return "LIMITED";
    case SafetyStatusMsg::STATE_BRAKING: return "BRAKING";
    case SafetyStatusMsg::STATE_LANDING: return "LANDING";
    case SafetyStatusMsg::STATE_ESTOP: return "ESTOP";
    default: return "UNKNOWN";
  }
}

}  // namespace

SafetyFilterNode::SafetyFilterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("safety_filter_node", options),
  autonomous_time_(0, 0, RCL_ROS_TIME),
  manual_time_(0, 0, RCL_ROS_TIME),
  deadman_time_(0, 0, RCL_ROS_TIME),
  vehicle_time_(0, 0, RCL_ROS_TIME),
  stream_time_(0, 0, RCL_ROS_TIME),
  last_step_(0, 0, RCL_ROS_TIME)
{
  rate_hz_ = declare_parameter("rate_hz", 30.0);
  frame_id_ = declare_parameter("frame_id", std::string("base_link"));

  // Autonomy starts DISABLED. The mission manager must explicitly enable it, so a node
  // restart mid-flight cannot silently re-authorise the model.
  autonomy_enabled_ = declare_parameter("autonomy_enabled_on_start", false);

  const auto limits = loadLimits();
  assertEnvelopeIsEnforceable(limits);
  filter_.setLimits(limits);

  const auto realtime_qos = rclcpp::QoS(1).best_effort();
  const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
  const auto status_qos = rclcpp::QoS(10).reliable();

  cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    "/j10/cmd_vel_safe", realtime_qos);
  status_pub_ = create_publisher<j10_interfaces::msg::SafetyStatus>(
    "/j10/safety/status", status_qos);

  autonomous_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "/j10/cmd_vel_raw", realtime_qos,
    std::bind(&SafetyFilterNode::onAutonomousCmd, this, std::placeholders::_1));
  manual_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "/j10/cmd_vel_manual", realtime_qos,
    std::bind(&SafetyFilterNode::onManualCmd, this, std::placeholders::_1));

  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/j10/safety/estop", latched_qos,
    std::bind(&SafetyFilterNode::onEstop, this, std::placeholders::_1));
  deadman_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/j10/teleop/deadman", realtime_qos,
    std::bind(&SafetyFilterNode::onDeadman, this, std::placeholders::_1));
  autonomy_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/j10/mission/autonomy_enabled", latched_qos,
    std::bind(&SafetyFilterNode::onAutonomyEnabled, this, std::placeholders::_1));

  vehicle_sub_ = create_subscription<j10_interfaces::msg::VehicleState>(
    "/j10/vehicle/state", rclcpp::QoS(5).reliable(),
    std::bind(&SafetyFilterNode::onVehicleState, this, std::placeholders::_1));
  stream_sub_ = create_subscription<j10_interfaces::msg::StreamStatus>(
    "/j10/video/status", rclcpp::QoS(5).reliable(),
    std::bind(&SafetyFilterNode::onStreamStatus, this, std::placeholders::_1));

  reset_estop_srv_ = create_service<std_srvs::srv::Trigger>(
    "/j10/safety/reset_estop",
    std::bind(
      &SafetyFilterNode::handleResetEstop, this, std::placeholders::_1,
      std::placeholders::_2));

  timer_ = create_wall_timer(
    periodFromRate(rate_hz_, 30.0), std::bind(&SafetyFilterNode::onTimer, this));

  const auto & l = filter_.limits();
  RCLCPP_INFO(
    get_logger(),
    "safety_filter_node up at %.1f Hz | speed %.2f m/s horiz, %.2f m/s vert, "
    "%.2f rad/s yaw | altitude %.2f-%.2f m | geofence %s",
    rate_hz_, l.max_horizontal_mps, l.max_vertical_mps, l.max_yaw_rate_rps,
    l.min_altitude_m, l.max_altitude_m, l.geofence_enabled ? "on" : "OFF");
  RCLCPP_INFO(
    get_logger(), "autonomy starts %s; nothing reaches the FC until it is enabled",
    autonomy_enabled_ ? "ENABLED" : "disabled");
}

Limits SafetyFilterNode::loadLimits()
{
  Limits l;
  l.max_horizontal_mps = declare_parameter("max_horizontal_mps", l.max_horizontal_mps);
  l.max_vertical_mps = declare_parameter("max_vertical_mps", l.max_vertical_mps);
  l.max_yaw_rate_rps = declare_parameter("max_yaw_rate_rps", l.max_yaw_rate_rps);

  l.max_accel_mps2 = declare_parameter("max_accel_mps2", l.max_accel_mps2);
  l.max_decel_mps2 = declare_parameter("max_decel_mps2", l.max_decel_mps2);
  l.max_yaw_accel_rps2 = declare_parameter("max_yaw_accel_rps2", l.max_yaw_accel_rps2);

  l.min_altitude_m = declare_parameter("min_altitude_m", l.min_altitude_m);
  l.max_altitude_m = declare_parameter("max_altitude_m", l.max_altitude_m);
  l.altitude_margin_m = declare_parameter("altitude_margin_m", l.altitude_margin_m);

  l.geofence_enabled = declare_parameter("geofence_enabled", l.geofence_enabled);
  l.geofence_min_x = declare_parameter("geofence_min_x", l.geofence_min_x);
  l.geofence_max_x = declare_parameter("geofence_max_x", l.geofence_max_x);
  l.geofence_min_y = declare_parameter("geofence_min_y", l.geofence_min_y);
  l.geofence_max_y = declare_parameter("geofence_max_y", l.geofence_max_y);
  l.geofence_margin_m = declare_parameter("geofence_margin_m", l.geofence_margin_m);

  l.brake_accel_mps2 = declare_parameter("brake_accel_mps2", l.brake_accel_mps2);
  l.control_latency_sec = declare_parameter("control_latency_sec", l.control_latency_sec);

  l.proximity_stop_m = declare_parameter("proximity_stop_m", l.proximity_stop_m);
  l.proximity_slow_m = declare_parameter("proximity_slow_m", l.proximity_slow_m);

  l.battery_warn_fraction = declare_parameter("battery_warn_fraction", l.battery_warn_fraction);
  l.battery_land_fraction = declare_parameter("battery_land_fraction", l.battery_land_fraction);
  l.land_speed_mps = declare_parameter("land_speed_mps", l.land_speed_mps);

  l.command_timeout_sec = declare_parameter("command_timeout_sec", l.command_timeout_sec);
  l.state_timeout_sec = declare_parameter("state_timeout_sec", l.state_timeout_sec);
  l.video_timeout_sec = declare_parameter("video_timeout_sec", l.video_timeout_sec);
  l.video_loss_stops_autonomy =
    declare_parameter("video_loss_stops_autonomy", l.video_loss_stops_autonomy);

  l.require_guided = declare_parameter("require_guided", l.require_guided);
  l.guided_mode = declare_parameter("guided_mode", l.guided_mode);
  return l;
}

void SafetyFilterNode::assertEnvelopeIsEnforceable(const Limits & l) const
{
  // An envelope is only a guarantee if the margins are wide enough to stop inside, given
  // the braking authority and the loop delay. Getting this wrong is silent -- the vehicle
  // flies fine right up until someone raises a speed limit and the fence quietly stops
  // being a fence -- so it is checked once, loudly, at startup.
  std::string problem;

  const auto require = [&problem](
    const char * what, double margin, double speed, double brake, double latency) {
      const double needed = requiredMargin(speed, brake, latency);
      if (margin + 1e-9 < needed) {
        char buf[320];
        std::snprintf(
          buf, sizeof buf,
          "%s margin is %.3f m but %.3f m is needed to stop from %.2f m/s at %.2f m/s^2 "
          "with %.0f ms of loop delay; ",
          what, margin, needed, speed, brake, latency * 1e3);
        problem += buf;
      }
    };

  if (l.geofence_enabled) {
    require(
      "geofence", l.geofence_margin_m, l.max_horizontal_mps,
      l.brake_accel_mps2, l.control_latency_sec);
  }
  require(
    "altitude", l.altitude_margin_m, l.max_vertical_mps,
    l.brake_accel_mps2, l.control_latency_sec);

  if (problem.empty()) {
    return;
  }

  const std::string message =
    "refusing to start: the configured envelope cannot be enforced. " + problem +
    "Either widen the margin, lower the speed limit, or raise brake_accel_mps2 to what "
    "the flight controller can actually achieve — but do not raise it to make this "
    "message go away.";
  RCLCPP_FATAL(get_logger(), "%s", message.c_str());
  throw std::runtime_error(message);
}

// ---------------------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------------------

void SafetyFilterNode::onAutonomousCmd(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  autonomous_cmd_ = msg->twist;
  autonomous_time_ = now();
  autonomous_received_ = true;
}

void SafetyFilterNode::onManualCmd(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  manual_cmd_ = msg->twist;
  manual_time_ = now();
  manual_received_ = true;
}

void SafetyFilterNode::onEstop(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  estop_engaged_ = msg->data;
  if (msg->data) {
    RCLCPP_ERROR(get_logger(), "E-STOP engaged");
  }
}

void SafetyFilterNode::onDeadman(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  deadman_held_ = msg->data;
  deadman_time_ = now();
  deadman_received_ = true;
}

void SafetyFilterNode::onAutonomyEnabled(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (autonomy_enabled_ != msg->data) {
    RCLCPP_WARN(get_logger(), "autonomy %s", msg->data ? "ENABLED" : "disabled");
  }
  autonomy_enabled_ = msg->data;
}

void SafetyFilterNode::onVehicleState(const j10_interfaces::msg::VehicleState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  vehicle_state_ = *msg;
  vehicle_time_ = now();
  vehicle_received_ = true;
}

void SafetyFilterNode::onStreamStatus(const j10_interfaces::msg::StreamStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  stream_ok_ = msg->state == j10_interfaces::msg::StreamStatus::STATE_STREAMING ||
    msg->state == j10_interfaces::msg::StreamStatus::STATE_DEGRADED;
  stream_time_ = now();
  stream_received_ = true;
}

void SafetyFilterNode::handleResetEstop(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (estop_engaged_) {
    response->success = false;
    response->message =
      "E-stop is still engaged on /j10/safety/estop -- release it before resetting.";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }
  filter_.reset();
  response->success = true;
  response->message = "E-stop latch cleared";
  RCLCPP_WARN(get_logger(), "E-stop latch cleared by operator request");
}

// ---------------------------------------------------------------------------------------
// The control cycle
// ---------------------------------------------------------------------------------------

void SafetyFilterNode::onTimer()
{
  const rclcpp::Time stamp = now();

  FilterInputs in;
  bool deadman_released = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);

    in.autonomous = autonomous_cmd_;
    in.autonomous_valid = autonomous_received_;
    in.autonomous_age_sec = autonomous_received_ ?
      (stamp - autonomous_time_).seconds() : 0.0;

    in.manual = manual_cmd_;
    in.manual_valid = manual_received_;
    in.manual_age_sec = manual_received_ ? (stamp - manual_time_).seconds() : 0.0;

    in.deadman_held = deadman_held_;
    in.deadman_valid = deadman_received_;
    in.deadman_age_sec = deadman_received_ ? (stamp - deadman_time_).seconds() : 0.0;
    in.estop_engaged = estop_engaged_;
    in.autonomy_enabled = autonomy_enabled_;

    // No StreamStatus publisher exists before Phase 3. Treating "never heard from" as
    // healthy would be wrong once video is real, so this is gated on the video watchdog
    // being enabled at all -- see video_loss_stops_autonomy in the config.
    in.video_valid = stream_received_ && stream_ok_;
    in.video_age_sec = stream_received_ ? (stamp - stream_time_).seconds() : 0.0;

    auto & v = in.vehicle;
    v.valid = vehicle_received_;
    v.age_sec = vehicle_received_ ? (stamp - vehicle_time_).seconds() : 0.0;
    if (vehicle_received_) {
      const auto & s = vehicle_state_;
      v.x = s.pose.pose.position.x;
      v.y = s.pose.pose.position.y;
      v.z = s.pose.pose.position.z;
      v.vx = s.velocity.twist.linear.x;
      v.vy = s.velocity.twist.linear.y;
      v.vz = s.velocity.twist.linear.z;
      v.yaw = yawFromQuaternion(
        s.pose.pose.orientation.x, s.pose.pose.orientation.y,
        s.pose.pose.orientation.z, s.pose.pose.orientation.w);
      v.rangefinder_valid = s.rangefinder_valid;
      v.rangefinder_range = s.rangefinder_range;
      v.ekf_healthy = s.ekf_healthy;
      v.armed = s.armed;
      v.guided = s.guided;
      v.mode = s.mode;
      v.battery_fraction = s.battery_percentage;
    }

    // Release a deadman we can no longer confirm. The filter has already seen the held
    // state for this cycle and braked on it, which makes the transition visible; from the
    // next cycle the ladder proceeds normally instead of latching on a dead publisher.
    if (deadman_held_ && deadman_received_ &&
      (stamp - deadman_time_).seconds() > filter_.limits().command_timeout_sec)
    {
      deadman_held_ = false;
      deadman_released = true;
    }
  }

  if (deadman_released) {
    RCLCPP_WARN(
      get_logger(),
      "/j10/teleop/deadman went silent while held — treating the deadman as released. "
      "Is j10_teleop still running?");
  }

  double dt = 1.0 / ((rate_hz_ > 0.0) ? rate_hz_ : 30.0);
  if (last_step_.nanoseconds() != 0) {
    const double measured = (stamp - last_step_).seconds();
    // Guard against a clock jump or a stalled executor producing a nonsense dt that would
    // let the acceleration limiter pass an arbitrarily large step through.
    if (measured > 0.0 && measured < 1.0) {
      dt = measured;
    }
  }
  last_step_ = stamp;

  const FilterOutput out = filter_.step(in, dt);

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = stamp;
  cmd.header.frame_id = frame_id_;
  cmd.twist = out.commanded;
  cmd_pub_->publish(cmd);

  j10_interfaces::msg::SafetyStatus status;
  status.header.stamp = stamp;
  status.header.frame_id = frame_id_;
  status.state = static_cast<uint8_t>(out.state);
  status.active_limits = out.active_limits;
  status.requested = out.requested;
  status.commanded = out.commanded;
  status.autonomy_enabled = out.autonomy_enabled;
  status.arbitration_source = toString(out.source);
  status_pub_->publish(status);

  if (status.state != last_reported_state_) {
    std::string limits;
    for (const auto & name : out.active_limits) {
      limits += (limits.empty() ? "" : ", ") + name;
    }
    RCLCPP_WARN(
      get_logger(), "safety state -> %s [%s] source=%s",
      stateName(status.state), limits.c_str(), status.arbitration_source.c_str());
    last_reported_state_ = status.state;
  }
}

}  // namespace j10_safety

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(j10_safety::SafetyFilterNode)
