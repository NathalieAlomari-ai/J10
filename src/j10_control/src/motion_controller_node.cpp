#include "j10_control/motion_controller_node.hpp"

#include <chrono>
#include <cmath>

namespace j10_control
{

namespace
{

using NavIntentMsg = j10_interfaces::msg::NavIntent;

// ActionType is a standalone enum so the shaper stays free of the message. These are what
// stop the two from drifting apart if the taxonomy ever changes.
static_assert(
  static_cast<uint8_t>(ActionType::kHold) == NavIntentMsg::ACTION_HOLD, "ACTION_HOLD");
static_assert(
  static_cast<uint8_t>(ActionType::kMove) == NavIntentMsg::ACTION_MOVE, "ACTION_MOVE");
static_assert(
  static_cast<uint8_t>(ActionType::kTurn) == NavIntentMsg::ACTION_TURN, "ACTION_TURN");
static_assert(
  static_cast<uint8_t>(ActionType::kExplore) == NavIntentMsg::ACTION_EXPLORE, "ACTION_EXPLORE");
static_assert(
  static_cast<uint8_t>(ActionType::kLand) == NavIntentMsg::ACTION_LAND, "ACTION_LAND");

std::chrono::nanoseconds periodFromRate(double rate_hz, double fallback_hz)
{
  const double hz = (rate_hz > 0.0) ? rate_hz : fallback_hz;
  return std::chrono::nanoseconds(static_cast<int64_t>(1e9 / hz));
}

/// Map an out-of-range action byte onto HOLD. An unknown action from a model is not a
/// reason to guess -- it is a reason to stop.
ActionType toActionType(uint8_t raw, bool & known)
{
  known = true;
  switch (raw) {
    case NavIntentMsg::ACTION_HOLD: return ActionType::kHold;
    case NavIntentMsg::ACTION_MOVE: return ActionType::kMove;
    case NavIntentMsg::ACTION_TURN: return ActionType::kTurn;
    case NavIntentMsg::ACTION_EXPLORE: return ActionType::kExplore;
    case NavIntentMsg::ACTION_LAND: return ActionType::kLand;
    default: break;
  }
  known = false;
  return ActionType::kHold;
}

}  // namespace

MotionControllerNode::MotionControllerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("motion_controller_node", options),
  last_step_(0, 0, RCL_ROS_TIME)
{
  rate_hz_ = declare_parameter("rate_hz", 30.0);
  frame_id_ = declare_parameter("frame_id", std::string("base_link"));
  shaper_.setLimits(loadLimits());

  cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    "/j10/cmd_vel_raw", rclcpp::QoS(1).best_effort());
  latency_pub_ = create_publisher<j10_interfaces::msg::LatencyReport>(
    "/j10/telemetry/latency", rclcpp::QoS(10).best_effort());

  // KEEP_LAST(1): always act on the newest decision. Queueing intents would mean flying a
  // decision the model made about a scene that has already moved on.
  intent_sub_ = create_subscription<j10_interfaces::msg::NavIntent>(
    "/j10/vla/intent", rclcpp::QoS(1).reliable(),
    std::bind(&MotionControllerNode::onIntent, this, std::placeholders::_1));

  timer_ = create_wall_timer(
    periodFromRate(rate_hz_, 30.0), std::bind(&MotionControllerNode::onTimer, this));

  const auto & l = shaper_.limits();
  RCLCPP_INFO(
    get_logger(),
    "motion_controller_node up at %.1f Hz | accel %.2f / decel %.2f m/s^2 | "
    "min_confidence %.2f | intent capped at %.2f s",
    rate_hz_, l.max_linear_accel_mps2, l.max_linear_decel_mps2,
    l.min_confidence, l.max_intent_age_sec);
}

ShaperLimits MotionControllerNode::loadLimits()
{
  ShaperLimits l;
  l.max_linear_accel_mps2 = declare_parameter("max_linear_accel_mps2", l.max_linear_accel_mps2);
  l.max_linear_decel_mps2 = declare_parameter("max_linear_decel_mps2", l.max_linear_decel_mps2);
  l.max_yaw_accel_rps2 = declare_parameter("max_yaw_accel_rps2", l.max_yaw_accel_rps2);
  l.max_yaw_decel_rps2 = declare_parameter("max_yaw_decel_rps2", l.max_yaw_decel_rps2);
  l.min_confidence = declare_parameter("min_confidence", l.min_confidence);
  l.scale_by_confidence = declare_parameter("scale_by_confidence", l.scale_by_confidence);
  l.land_speed_mps = declare_parameter("land_speed_mps", l.land_speed_mps);
  l.max_intent_age_sec = declare_parameter("max_intent_age_sec", l.max_intent_age_sec);
  return l;
}

void MotionControllerNode::onIntent(const j10_interfaces::msg::NavIntent::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  intent_ = *msg;
  intent_received_ = true;
}

void MotionControllerNode::onTimer()
{
  const rclcpp::Time stamp = now();
  const auto cycle_start = std::chrono::steady_clock::now();

  Intent intent;
  j10_interfaces::msg::NavIntent raw;
  bool have_raw = false;
  bool unknown_action = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (intent_received_) {
      raw = intent_;
      have_raw = true;
    }
  }

  if (have_raw) {
    bool known = false;
    intent.valid = true;
    intent.action = toActionType(raw.action_type, known);
    unknown_action = !known;
    intent.velocity = raw.velocity;
    intent.duration_sec = raw.duration;
    intent.confidence = raw.confidence;

    // Age from the intent's own stamp, so network or scheduling delay counts against its
    // validity window rather than being invisible.
    const rclcpp::Time intent_stamp(raw.header.stamp, RCL_ROS_TIME);
    const double age = (stamp - intent_stamp).seconds();
    // A stamp in the future means clock skew somewhere upstream; treat it as fresh rather
    // than negative-aged, but never as extending the window.
    intent.age_sec = std::max(age, 0.0);
  }

  if (unknown_action) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "NavIntent carried unknown action_type %u -- treating as HOLD", raw.action_type);
  }

  double dt = 1.0 / ((rate_hz_ > 0.0) ? rate_hz_ : 30.0);
  if (last_step_.nanoseconds() != 0) {
    const double measured = (stamp - last_step_).seconds();
    if (measured > 0.0 && measured < 1.0) {
      dt = measured;
    }
  }
  last_step_ = stamp;

  const ShaperOutput out = shaper_.step(intent, dt);

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = stamp;
  cmd.header.frame_id = frame_id_;
  cmd.twist = out.command;
  cmd_pub_->publish(cmd);

  // Only report latency while actually following an intent: a hover cycle traces back to
  // no camera frame, and feeding it into the monitor would dilute the percentiles with
  // measurements that mean nothing.
  if (out.following && have_raw) {
    const auto cycle_end = std::chrono::steady_clock::now();
    j10_interfaces::msg::LatencyReport report;
    report.header.stamp = stamp;
    report.header.frame_id = frame_id_;
    report.stage = "CONTROL";
    report.source_stamp = raw.source_stamp;
    report.stage_latency_ms = static_cast<float>(
      std::chrono::duration<double, std::milli>(cycle_end - cycle_start).count());

    const rclcpp::Time source(raw.source_stamp, RCL_ROS_TIME);
    const double cumulative_ms = (stamp - source).seconds() * 1e3;
    report.cumulative_latency_ms = static_cast<float>(std::max(cumulative_ms, 0.0));
    report.sequence = sequence_++;
    latency_pub_->publish(report);
  }

  if (static_cast<uint8_t>(out.reason) != last_reason_) {
    RCLCPP_INFO(get_logger(), "intent state -> %s", toString(out.reason));
    last_reason_ = static_cast<uint8_t>(out.reason);
  }
}

}  // namespace j10_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(j10_control::MotionControllerNode)
