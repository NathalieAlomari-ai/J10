#include "j10_control/motion_shaper.hpp"

#include <algorithm>

namespace j10_control
{

namespace
{
/// The model's requested validity window, capped by our own ceiling. A model does not get
/// to decide how long it is trusted for.
double effectiveDuration(double requested, const ShaperLimits & limits)
{
  if (limits.max_intent_age_sec > 0.0) {
    return std::min(requested, limits.max_intent_age_sec);
  }
  return requested;
}
}  // namespace

MotionShaper::MotionShaper(const ShaperLimits & limits)
: limits_(limits)
{
}

void MotionShaper::reset()
{
  current_ = geometry_msgs::msg::Twist();
}

ShaperOutput MotionShaper::step(const Intent & intent, double dt)
{
  ShaperOutput out;

  // ---------------------------------------------------------------------------------
  // Decide what we are aiming for. Every rejection path aims at zero -- the fast layer's
  // default output is hover, never the last command.
  // ---------------------------------------------------------------------------------
  if (!intent.valid) {
    out.reason = ShaperReason::kNoIntent;
  } else if (intent.duration_sec <= 0.0) {
    // NavIntent's contract says duration must be > 0. A zero or negative window is a
    // malformed intent, not an infinitely valid one.
    out.reason = ShaperReason::kExpired;
  } else if (intent.age_sec > effectiveDuration(intent.duration_sec, limits_)) {
    out.reason = ShaperReason::kExpired;
  } else if (intent.confidence < limits_.min_confidence) {
    out.reason = ShaperReason::kLowConfidence;
  } else {
    switch (intent.action) {
      case ActionType::kHold:
        out.reason = ShaperReason::kHold;
        break;

      case ActionType::kLand:
        out.reason = ShaperReason::kLand;
        out.target.linear.z = -limits_.land_speed_mps;
        out.following = true;
        break;

      case ActionType::kMove:
      case ActionType::kTurn:
      case ActionType::kExplore:
        out.reason = ShaperReason::kActive;
        out.target = intent.velocity;
        out.following = true;
        break;
    }
  }

  // A NaN from the model must never become a target.
  sanitize(out.target);

  if (out.following && limits_.scale_by_confidence) {
    const double scale = std::min(std::max(intent.confidence, 0.0), 1.0);
    out.target.linear.x *= scale;
    out.target.linear.y *= scale;
    out.target.linear.z *= scale;
    out.target.angular.z *= scale;
  }

  // ---------------------------------------------------------------------------------
  // Ease toward it. Trapezoidal: bounded acceleration in, bounded deceleration out, so a
  // new intent never steps the command and an expiry never slams it to zero.
  // ---------------------------------------------------------------------------------
  approach(
    current_.linear.x, out.target.linear.x,
    limits_.max_linear_accel_mps2, limits_.max_linear_decel_mps2, dt);
  approach(
    current_.linear.y, out.target.linear.y,
    limits_.max_linear_accel_mps2, limits_.max_linear_decel_mps2, dt);
  approach(
    current_.linear.z, out.target.linear.z,
    limits_.max_linear_accel_mps2, limits_.max_linear_decel_mps2, dt);
  approach(
    current_.angular.z, out.target.angular.z,
    limits_.max_yaw_accel_rps2, limits_.max_yaw_decel_rps2, dt);

  out.command = current_;
  return out;
}

}  // namespace j10_control
