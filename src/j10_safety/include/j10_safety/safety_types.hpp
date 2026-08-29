// Plain types and pure helpers for the safety filter.
//
// Deliberately free of ROS: boundary rule 3 in docs/ARCHITECTURE.md requires j10_safety to
// be unit-testable from a plain gtest binary in under a second, with no simulator, no
// MAVROS and no GPU. The filter therefore works on a plain VehicleSnapshot rather than the
// j10_interfaces/VehicleState message, and the node converts at the boundary. That keeps
// the tests trivial to write and stops message evolution from churning the safety logic.

#ifndef J10_SAFETY__SAFETY_TYPES_HPP_
#define J10_SAFETY__SAFETY_TYPES_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>

namespace j10_safety
{

/// Escalation ladder. Values mirror j10_interfaces/SafetyStatus STATE_* exactly; the node
/// static_asserts that they still match, so a change to the message cannot silently
/// desynchronise from the filter.
enum class SafetyState : uint8_t
{
  kNominal = 0,   ///< command passed through unmodified
  kLimited = 1,   ///< command clamped or shaped, still flying the intent
  kBraking = 2,   ///< commanding zero velocity, holding position
  kLanding = 3,   ///< safety-initiated descent
  kEstop = 4,     ///< latched stop, requires explicit operator reset
};

/// Which input won arbitration. Ordered by priority, highest first.
enum class ArbitrationSource : uint8_t
{
  kEstop = 0,
  kManual = 1,
  kOverride = 2,
  kAutonomous = 3,
};

inline const char * toString(ArbitrationSource source)
{
  switch (source) {
    case ArbitrationSource::kEstop: return "ESTOP";
    case ArbitrationSource::kManual: return "MANUAL";
    case ArbitrationSource::kOverride: return "OVERRIDE";
    case ArbitrationSource::kAutonomous: return "AUTONOMOUS";
  }
  return "UNKNOWN";
}

/// Every limit the filter can enforce. All distances metres, speeds m/s, times seconds.
struct Limits
{
  // --- Velocity clamps ---
  double max_horizontal_mps{0.5};
  double max_vertical_mps{0.3};
  double max_yaw_rate_rps{0.5};

  // --- Acceleration limits (applied to the change in commanded velocity) ---
  // Deceleration is allowed to be more aggressive than acceleration: slowing down is
  // always the safer direction, and rate-limiting a brake would defeat the whole point.
  double max_accel_mps2{0.5};
  double max_decel_mps2{1.5};
  double max_yaw_accel_rps2{1.0};

  // --- Altitude envelope (metres above the takeoff plane) ---
  double min_altitude_m{0.3};
  double max_altitude_m{2.0};
  /// Distance from the boundary at which the outward component starts scaling to zero.
  double altitude_margin_m{0.3};

  // --- Envelope barrier ---
  /// Deceleration available for stopping the vehicle at a boundary, m/s^2.
  ///
  /// This is what the FLIGHT CONTROLLER can actually achieve, not what this filter
  /// rate-limits commands to. The filter can only stop *asking* for outward motion; the
  /// vehicle is brought to a halt by the guided velocity controller, so the number that
  /// governs how much room a stop needs is the autopilot's, and it is deliberately
  /// conservative here.
  double brake_accel_mps2{1.0};
  /// Round-trip control latency, seconds. The vehicle keeps moving for roughly this long
  /// at its present velocity before any new command takes effect, so this much travel is
  /// reserved out of the distance to every boundary. Matches the 300 ms end-to-end budget.
  double control_latency_sec{0.3};

  // --- Virtual geofence, an axis-aligned box in the ENU local frame ---
  bool geofence_enabled{true};
  double geofence_min_x{-2.0};
  double geofence_max_x{2.0};
  double geofence_min_y{-2.0};
  double geofence_max_y{2.0};
  double geofence_margin_m{0.5};

  // --- Downward rangefinder proximity ---
  /// Below this AGL reading, descent is refused outright.
  double proximity_stop_m{0.25};
  /// Between stop and slow, descent is scaled down.
  double proximity_slow_m{0.6};

  // --- Battery failsafe (fraction, 0.0-1.0; negative percentage means unknown) ---
  double battery_warn_fraction{0.25};
  double battery_land_fraction{0.15};
  double land_speed_mps{0.3};

  // --- Input freshness watchdogs ---
  double command_timeout_sec{0.3};
  double state_timeout_sec{0.5};
  double video_timeout_sec{0.5};
  /// When true, loss of video revokes autonomy (the VLA is blind without frames).
  /// Manual teleop is unaffected -- a human on the sticks does not need the video link.
  bool video_loss_stops_autonomy{true};

  // --- Offboard authority ---
  /// When true, an armed vehicle that is not in `guided_mode` cannot be commanded: the
  /// flight controller has stopped acting on our setpoints, so continuing to compute them
  /// as though it were is a fiction. Empty `guided_mode` checks only the boolean flag.
  bool require_guided{true};
  std::string guided_mode{"GUIDED"};
};

/// Everything the filter needs to know about the vehicle, flattened out of VehicleState.
struct VehicleSnapshot
{
  bool valid{false};              ///< a VehicleState has been received at all
  double age_sec{0.0};            ///< seconds since that message arrived

  // Position and velocity in the ENU local frame (EKF origin).
  double x{0.0};
  double y{0.0};
  double z{0.0};
  // Measured velocity, ENU. The envelope barrier reads these: a boundary check that looks
  // only at the commanded velocity ignores the momentum the vehicle already carries, and
  // stops asking for outward motion at exactly the moment it can no longer help.
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};
  double yaw{0.0};                ///< radians, CCW from ENU +x

  bool rangefinder_valid{false};
  double rangefinder_range{0.0};  ///< metres AGL

  bool ekf_healthy{false};
  bool armed{false};
  /// Flight controller is in a mode that acts on offboard setpoints.
  bool guided{false};
  /// Raw flight-controller mode string, e.g. "GUIDED", "LOITER", "LAND".
  std::string mode;

  /// 0.0-1.0, or negative when unknown. Unknown is NOT treated as empty.
  double battery_fraction{-1.0};
};

/// One control cycle's worth of inputs.
struct FilterInputs
{
  // --- Autonomous path (motion controller output) ---
  geometry_msgs::msg::Twist autonomous;
  bool autonomous_valid{false};
  double autonomous_age_sec{0.0};

  // --- Manual path (teleop) ---
  geometry_msgs::msg::Twist manual;
  bool manual_valid{false};
  double manual_age_sec{0.0};
  /// Deadman state, and how old that state is. A held deadman with no further messages is
  /// not a held deadman -- it is a dead publisher, and treating it as held latches the
  /// vehicle into the manual branch with no way out.
  bool deadman_held{false};
  double deadman_age_sec{0.0};
  /// Defaults true: a caller that supplies `deadman_held` without freshness information is
  /// taken at its word, which keeps a held deadman meaning manual control. The node always
  /// supplies both, so in the running system this is never the default.
  bool deadman_valid{true};

  // --- Latches and gates ---
  bool estop_engaged{false};      ///< rising edge latches; clears only via reset()
  bool autonomy_enabled{true};    ///< mission manager gate

  // --- Health ---
  VehicleSnapshot vehicle;
  bool video_valid{false};
  double video_age_sec{0.0};
};

/// The filter's verdict for one cycle.
struct FilterOutput
{
  geometry_msgs::msg::Twist commanded;   ///< what actually goes to the flight controller
  geometry_msgs::msg::Twist requested;   ///< arbitration winner, before any filtering
  SafetyState state{SafetyState::kNominal};
  ArbitrationSource source{ArbitrationSource::kOverride};
  std::vector<std::string> active_limits;
  bool autonomy_enabled{false};
};

// --------------------------------------------------------------------------------------
// Pure helpers
// --------------------------------------------------------------------------------------

/// Replace any non-finite component with zero. Returns true if anything was scrubbed.
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

inline void zero(geometry_msgs::msg::Twist & twist)
{
  twist = geometry_msgs::msg::Twist();
}

/// Raise `state` to `candidate` if candidate is more severe. The ladder only goes up
/// within a cycle; it is recomputed from scratch each cycle.
inline void escalate(SafetyState & state, SafetyState candidate)
{
  if (static_cast<uint8_t>(candidate) > static_cast<uint8_t>(state)) {
    state = candidate;
  }
}

/// Record a limit name once, preserving insertion order.
inline void note(std::vector<std::string> & limits, const std::string & name)
{
  if (std::find(limits.begin(), limits.end(), name) == limits.end()) {
    limits.push_back(name);
  }
}

/// Clamp a velocity along one axis against a positional envelope.
///
/// Returns the allowed velocity: unchanged when moving inward or comfortably inside,
/// scaled linearly to zero across `margin` as the boundary is approached, and hard zero
/// once outside. This is the single primitive behind both the geofence and the altitude
/// envelope, so their behaviour cannot drift apart.
inline double limitAgainstBounds(
  double position, double lower, double upper, double margin, double velocity)
{
  if (velocity > 0.0) {
    const double distance = upper - position;
    if (distance <= 0.0) {
      return 0.0;
    }
    if (margin > 0.0 && distance < margin) {
      return velocity * (distance / margin);
    }
  } else if (velocity < 0.0) {
    const double distance = position - lower;
    if (distance <= 0.0) {
      return 0.0;
    }
    if (margin > 0.0 && distance < margin) {
      return velocity * (distance / margin);
    }
  }
  return velocity;
}

/// Braking-distance form of `limitAgainstBounds`, and the one the envelope actually uses.
///
/// `limitAgainstBounds` scales the *commanded* velocity to zero across the margin. That is
/// not a containment guarantee: the vehicle's real velocity is not the commanded one, the
/// autopilot tracks the command with its own lag, and the whole loop carries roughly
/// `latency_sec` of delay. A command that reaches zero exactly at the boundary belongs to a
/// vehicle that is still moving outward.
///
/// This is the discrete form of the control-barrier condition. With `d` the distance to the
/// boundary, the outward command is capped so that the remaining room is enough to stop:
///
///     d_usable = d - v_measured_outward * latency_sec       (travel during the delay)
///     v_max    = sqrt(2 * brake_accel * max(0, d_usable))   (braking distance)
///
/// It never *raises* the allowed velocity: the heuristic margin scaling is applied first and
/// acts as a ceiling, so an envelope tuned against the old behaviour cannot loosen.
///
/// @note This bounds the command; it does not command a correction. If the vehicle is
///       already drifting outward faster than `v_max`, the filter returns zero and the
///       autopilot's own velocity controller does the stopping. Actively pushing back
///       inward is a controller's job, not a veto layer's, which is why `brake_accel`
///       should be set to what the FLIGHT CONTROLLER can achieve.
inline double limitAgainstBoundsBraking(
  double position, double lower, double upper, double margin,
  double velocity, double measured_velocity,
  double brake_accel, double latency_sec)
{
  // Never looser than the margin rule.
  const double allowed = limitAgainstBounds(position, lower, upper, margin, velocity);
  if (allowed == 0.0 || brake_accel <= 0.0) {
    return allowed;
  }

  const bool outward_positive = allowed > 0.0;
  const double distance = outward_positive ? (upper - position) : (position - lower);
  if (distance <= 0.0) {
    return 0.0;
  }

  // Only outward measured motion eats into the room available. Drifting inward does not
  // earn extra outward budget, hence the max() rather than a signed term.
  const double measured_outward = outward_positive ? measured_velocity : -measured_velocity;
  const double reserved = std::max(0.0, measured_outward) * std::max(0.0, latency_sec);
  const double usable = distance - reserved;
  if (usable <= 0.0) {
    return 0.0;
  }

  const double v_max = std::sqrt(2.0 * brake_accel * usable);
  if (std::abs(allowed) <= v_max) {
    return allowed;
  }
  return std::copysign(v_max, allowed);
}

/// The smallest margin that can hold `max_speed` inside a boundary, given the braking
/// authority and the loop delay. Configuring a margin below this means the envelope cannot
/// be enforced at the configured speed, so the node refuses to start rather than flying an
/// envelope it cannot keep.
inline double requiredMargin(double max_speed, double brake_accel, double latency_sec)
{
  if (max_speed <= 0.0) {
    return 0.0;
  }
  const double braking = (brake_accel > 0.0) ?
    (max_speed * max_speed) / (2.0 * brake_accel) :
    std::numeric_limits<double>::infinity();
  return braking + max_speed * std::max(0.0, latency_sec);
}

/// Rotate a body-FLU horizontal velocity into the ENU local frame.
inline void bodyToEnu(double vx_body, double vy_body, double yaw, double & vx, double & vy)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  vx = vx_body * c - vy_body * s;
  vy = vx_body * s + vy_body * c;
}

/// Inverse of bodyToEnu.
inline void enuToBody(double vx_enu, double vy_enu, double yaw, double & vx, double & vy)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  vx = vx_enu * c + vy_enu * s;
  vy = -vx_enu * s + vy_enu * c;
}

/// Scale a horizontal vector so its magnitude never exceeds `limit`, preserving direction.
/// Clamping per-axis instead would silently rotate the commanded heading.
inline bool clampHorizontal(double & x, double & y, double limit)
{
  if (limit <= 0.0) {
    return false;
  }
  const double speed = std::hypot(x, y);
  if (speed <= limit) {
    return false;
  }
  const double scale = limit / speed;
  x *= scale;
  y *= scale;
  return true;
}

/// Clamp a scalar to +/- limit. Returns true if it bound.
inline bool clampScalar(double & value, double limit)
{
  if (limit <= 0.0 || std::abs(value) <= limit) {
    return false;
  }
  value = std::copysign(limit, value);
  return true;
}

/// Rate-limit one component toward `target`, allowing `decel` when moving toward zero and
/// `accel` otherwise. Returns true if the limit bound.
inline bool rateLimit(double & current, double target, double accel, double decel, double dt)
{
  if (dt <= 0.0) {
    // A non-positive dt means the clock went backwards, the executor stalled, or this is
    // the very first cycle. Previously this assigned the target outright AND reported that
    // nothing bound -- the acceleration limiter silently disabled itself at exactly the
    // moment the timing was untrustworthy. Hold instead, and say that it bound, so the
    // condition is visible in active_limits rather than invisible in the command.
    return true;
  }
  const double delta = target - current;
  // Moving toward zero (or reversing) is deceleration; away from zero is acceleration.
  const bool decelerating = std::abs(target) < std::abs(current) ||
    (current != 0.0 && target != 0.0 && ((current > 0.0) != (target > 0.0)));
  const double budget = (decelerating ? decel : accel) * dt;
  if (budget <= 0.0 || std::abs(delta) <= budget) {
    current = target;
    return false;
  }
  current += std::copysign(budget, delta);
  return true;
}

/// Rate-limit a horizontal velocity toward its target as a 2-VECTOR.
///
/// Limiting x and y independently lets a 45-degree command accelerate at sqrt(2) times the
/// configured limit, for the same reason `clampHorizontal` scales speed as a vector rather
/// than per axis. Reduces exactly to `rateLimit` on a single axis.
inline bool rateLimitHorizontal(
  double & cx, double & cy, double tx, double ty, double accel, double decel, double dt)
{
  if (dt <= 0.0) {
    return true;   // hold; see the note in rateLimit
  }
  const double dx = tx - cx;
  const double dy = ty - cy;
  const double delta = std::hypot(dx, dy);
  // Shrinking the speed is deceleration, whatever the direction change.
  const bool decelerating = std::hypot(tx, ty) < std::hypot(cx, cy);
  const double budget = (decelerating ? decel : accel) * dt;
  if (budget <= 0.0 || delta <= budget) {
    cx = tx;
    cy = ty;
    return false;
  }
  const double scale = budget / delta;
  cx += dx * scale;
  cy += dy * scale;
  return true;
}

}  // namespace j10_safety

#endif  // J10_SAFETY__SAFETY_TYPES_HPP_
