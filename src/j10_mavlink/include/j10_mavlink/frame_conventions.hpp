// Frame and type-mask helpers for the MAVLink setpoint path.
//
// These are deliberately free functions on plain messages: the sign conventions below are
// the single most common source of "the drone flew sideways" bugs, and they must be
// testable from a gtest binary with no ROS graph, no MAVROS, and no simulator.

#ifndef J10_MAVLINK__FRAME_CONVENTIONS_HPP_
#define J10_MAVLINK__FRAME_CONVENTIONS_HPP_

#include <algorithm>
#include <cmath>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/msg/position_target.hpp>

namespace j10_mavlink
{

/// Axis layout of the velocity written into the outgoing PositionTarget.
enum class BodyFrameConvention
{
  /// Write the ROS body-FLU twist through untouched.
  ///
  /// This is the correct choice for `mavros_msgs/PositionTarget` with
  /// `FRAME_BODY_NED`. The MAVROS `setpoint_raw` plugin already applies
  /// `ftf::transform_frame_baselink_aircraft` (FLU -> FRD) to `velocity`, and
  /// `ftf::transform_frame_ned_enu` to `yaw_rate`, before it builds the MAVLink
  /// packet. Converting here as well would double-negate y, z and yaw_rate and
  /// fly the vehicle in the mirror image of the commanded direction.
  kFlu,

  /// Convert FLU -> FRD in this node.
  ///
  /// Only for a consumer that expects an already-aircraft-framed setpoint and
  /// performs no transform of its own. Provided so that a MAVROS behaviour
  /// change can be corrected with a parameter rather than a patch release.
  kFrd,
};

/// Parse a `body_frame_convention` parameter value. Returns false on an unknown name,
/// leaving `out` untouched.
inline bool parseBodyFrameConvention(const std::string & name, BodyFrameConvention & out)
{
  if (name == "flu") {
    out = BodyFrameConvention::kFlu;
    return true;
  }
  if (name == "frd") {
    out = BodyFrameConvention::kFrd;
    return true;
  }
  return false;
}

inline const char * toString(BodyFrameConvention convention)
{
  return convention == BodyFrameConvention::kFlu ? "flu" : "frd";
}

/// type_mask selecting velocity + yaw_rate: ignore position, acceleration and absolute yaw.
///
/// Equals 1479. The FORCE bit stays clear, so the acceleration field would be read as
/// acceleration if it were not ignored.
inline constexpr uint16_t kVelocityYawRateTypeMask =
  mavros_msgs::msg::PositionTarget::IGNORE_PX |
  mavros_msgs::msg::PositionTarget::IGNORE_PY |
  mavros_msgs::msg::PositionTarget::IGNORE_PZ |
  mavros_msgs::msg::PositionTarget::IGNORE_AFX |
  mavros_msgs::msg::PositionTarget::IGNORE_AFY |
  mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
  mavros_msgs::msg::PositionTarget::IGNORE_YAW;

/// Replace any non-finite component with zero.
///
/// A NaN reaching the flight controller is not a degraded command, it is an undefined one.
/// Returns true when something had to be scrubbed, so the caller can log it.
inline bool sanitize(geometry_msgs::msg::Twist & twist)
{
  bool scrubbed = false;
  const auto fix = [&scrubbed](double & v) {
      if (!std::isfinite(v)) {
        v = 0.0;
        scrubbed = true;
      }
    };
  fix(twist.linear.x);
  fix(twist.linear.y);
  fix(twist.linear.z);
  fix(twist.angular.x);
  fix(twist.angular.y);
  fix(twist.angular.z);
  return scrubbed;
}

/// Last-ditch saturation applied immediately before the setpoint leaves the process.
///
/// This is defence in depth, NOT a substitute for `j10_safety` — the safety filter remains
/// the only node with the authority to veto or shape a command. The limits here exist so
/// that a bug upstream of the filter cannot command an unbounded velocity, and they are
/// deliberately set wide enough in the shipped config that they never bind during normal
/// operation. Linear velocity is scaled as a 3-vector so the commanded *direction* survives
/// saturation; clamping per-axis would silently rotate the command.
///
/// Returns true when either limit actually bound.
inline bool clampBodyVelocity(
  geometry_msgs::msg::Twist & twist,
  double max_linear_mps,
  double max_yaw_rate_rps)
{
  bool clamped = false;

  if (max_linear_mps > 0.0) {
    const double speed = std::sqrt(
      twist.linear.x * twist.linear.x +
      twist.linear.y * twist.linear.y +
      twist.linear.z * twist.linear.z);
    if (speed > max_linear_mps) {
      const double scale = max_linear_mps / speed;
      twist.linear.x *= scale;
      twist.linear.y *= scale;
      twist.linear.z *= scale;
      clamped = true;
    }
  }

  if (max_yaw_rate_rps > 0.0 && std::abs(twist.angular.z) > max_yaw_rate_rps) {
    twist.angular.z = std::copysign(max_yaw_rate_rps, twist.angular.z);
    clamped = true;
  }

  return clamped;
}

/// Fill the velocity and yaw_rate fields of a PositionTarget from a body-FLU twist.
///
/// Does not touch `header`, `coordinate_frame` or `type_mask` — the caller owns those.
inline void fillBodyVelocitySetpoint(
  const geometry_msgs::msg::Twist & body_flu,
  BodyFrameConvention convention,
  mavros_msgs::msg::PositionTarget & target)
{
  const double sign = (convention == BodyFrameConvention::kFlu) ? 1.0 : -1.0;

  // x (forward) is identical in FLU and FRD; y and z invert, as does yaw_rate.
  target.velocity.x = body_flu.linear.x;
  target.velocity.y = sign * body_flu.linear.y;
  target.velocity.z = sign * body_flu.linear.z;
  target.yaw_rate = static_cast<float>(sign * body_flu.angular.z);

  target.position.x = 0.0;
  target.position.y = 0.0;
  target.position.z = 0.0;
  target.acceleration_or_force.x = 0.0;
  target.acceleration_or_force.y = 0.0;
  target.acceleration_or_force.z = 0.0;
  target.yaw = 0.0f;
}

}  // namespace j10_mavlink

#endif  // J10_MAVLINK__FRAME_CONVENTIONS_HPP_
