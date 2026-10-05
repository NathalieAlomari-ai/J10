// Intent -> smoothed velocity shaping.
//
// Pure logic, no ROS: same reasoning as j10_safety. The fast layer's behaviour on intent
// expiry and model silence is the thing most worth testing exhaustively, and it should not
// need a simulator to do it.
//
// NOTE ON THE DUPLICATED RATE LIMITER
// This file rate-limits velocity, and so does j10_safety. That duplication is deliberate.
// The shaping here exists for *smoothness* -- so a new intent does not step the command --
// while the limiting in j10_safety exists for *safety* and must stay independent of
// everything upstream of it. Sharing the code would couple the guardian to the thing it
// guards, which is exactly what boundary rule 3 forbids.

#ifndef J10_CONTROL__MOTION_SHAPER_HPP_
#define J10_CONTROL__MOTION_SHAPER_HPP_

#include <cmath>
#include <cstdint>

#include <geometry_msgs/msg/twist.hpp>

namespace j10_control
{

/// Mirrors j10_interfaces/NavIntent ACTION_* so the shaper stays free of the message.
/// The node static_asserts these against the real constants.
enum class ActionType : uint8_t
{
  kHold = 0,
  kMove = 1,
  kTurn = 2,
  kExplore = 3,
  kLand = 4,
};

/// Why the shaper is producing what it is producing. Surfaced for logging and for the
/// mission manager, and checked directly in tests.
enum class ShaperReason : uint8_t
{
  kActive,          ///< following a live intent
  kNoIntent,        ///< nothing has arrived yet, or the model went silent
  kExpired,         ///< the intent outlived its own validity window
  kLowConfidence,   ///< the model was not sure enough to be trusted
  kHold,            ///< the intent explicitly asked to hold station
  kLand,            ///< the intent asked to land
};

inline const char * toString(ShaperReason reason)
{
  switch (reason) {
    case ShaperReason::kActive: return "ACTIVE";
    case ShaperReason::kNoIntent: return "NO_INTENT";
    case ShaperReason::kExpired: return "EXPIRED";
    case ShaperReason::kLowConfidence: return "LOW_CONFIDENCE";
    case ShaperReason::kHold: return "HOLD";
    case ShaperReason::kLand: return "LAND";
  }
  return "UNKNOWN";
}

struct ShaperLimits
{
  /// Deceleration is separate from, and normally larger than, acceleration: easing *into*
  /// motion should be gentle, but coming to a stop should not be sluggish.
  double max_linear_accel_mps2{0.5};
  double max_linear_decel_mps2{1.0};
  double max_yaw_accel_rps2{1.0};
  double max_yaw_decel_rps2{2.0};

  /// Intents below this confidence are ignored entirely, and the command decays to hover.
  double min_confidence{0.0};

  /// Scale the target velocity by the model's own confidence. Off by default: a
  /// low-confidence intent is better rejected outright than flown slowly.
  bool scale_by_confidence{false};

  /// Descent rate for ACTION_LAND. The safety filter still bounds this.
  double land_speed_mps{0.3};

  /// Hard ceiling on how long any single intent may be followed, regardless of the
  /// `duration` the model asked for. Without this, a model that emits duration=1000 grants
  /// itself unlimited authority and the expiry path -- the entire point of the two-rate
  /// design -- never fires. Zero disables the cap.
  double max_intent_age_sec{1.0};
};

/// One intent as the shaper sees it, flattened out of NavIntent.
struct Intent
{
  bool valid{false};
  ActionType action{ActionType::kHold};
  geometry_msgs::msg::Twist velocity;   ///< body FLU
  double age_sec{0.0};                  ///< since the intent's own header.stamp
  double duration_sec{0.0};             ///< validity window; must be > 0 to be usable
  double confidence{0.0};
};

struct ShaperOutput
{
  geometry_msgs::msg::Twist command;    ///< shaped, body FLU
  geometry_msgs::msg::Twist target;     ///< what the intent asked for, pre-shaping
  ShaperReason reason{ShaperReason::kNoIntent};
  bool following{false};                ///< true only while flying a live intent
};

/// Converts a slow stream of semantic intents into a smooth 30 Hz velocity command.
///
/// The single most important property, and the one the tests hammer: when an intent
/// expires or the model goes silent, the command decays to **zero**, not to the last
/// value. Holding the last command on loss of input is how offboard systems fly into
/// walls. It decays smoothly rather than stepping, because a step to zero at 30 Hz is
/// itself a disturbance -- but it always decays.
class MotionShaper
{
public:
  MotionShaper() = default;
  explicit MotionShaper(const ShaperLimits & limits);

  ShaperOutput step(const Intent & intent, double dt);
  void reset();

  const ShaperLimits & limits() const {return limits_;}
  void setLimits(const ShaperLimits & limits) {limits_ = limits;}
  const geometry_msgs::msg::Twist & current() const {return current_;}

private:
  ShaperLimits limits_{};
  geometry_msgs::msg::Twist current_{};
};

// --------------------------------------------------------------------------------------
// Shared helpers (also used by the tests)
// --------------------------------------------------------------------------------------

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

/// Move `current` toward `target`, spending at most accel*dt when speeding up and
/// decel*dt when slowing down. Returns true when the limit bound.
inline bool approach(double & current, double target, double accel, double decel, double dt)
{
  if (dt <= 0.0) {
    current = target;
    return false;
  }
  const double delta = target - current;
  if (delta == 0.0) {
    return false;
  }
  const bool slowing = std::abs(target) < std::abs(current) ||
    (current != 0.0 && target != 0.0 && ((current > 0.0) != (target > 0.0)));
  const double budget = (slowing ? decel : accel) * dt;
  if (budget <= 0.0 || std::abs(delta) <= budget) {
    current = target;
    return false;
  }
  current += std::copysign(budget, delta);
  return true;
}

}  // namespace j10_control

#endif  // J10_CONTROL__MOTION_SHAPER_HPP_
