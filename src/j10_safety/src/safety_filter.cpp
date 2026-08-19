#include "j10_safety/safety_filter.hpp"

#include <algorithm>
#include <cmath>

namespace j10_safety
{

namespace
{
/// Velocity changes below this are numerical noise, not enforcement.
///
/// Rotating a body command through yaw goes via cos/sin, and cos(pi/2) is 6.1e-17 rather
/// than 0. Without this threshold, a command flown exactly parallel to a fence produces a
/// sub-nanometre-per-second "outward" component, which the envelope clamps to zero and
/// which would then be reported as an active GEOFENCE limit -- escalating the state to
/// BRAKING while the vehicle is in fact flying safely. Report a limit only when it
/// actually changed the command by an amount that could matter.
constexpr double kVelocityEpsilon = 1e-9;

bool changed(double before, double after)
{
  return std::abs(after - before) > kVelocityEpsilon;
}
}  // namespace

SafetyFilter::SafetyFilter(const Limits & limits)
: limits_(limits)
{
}

void SafetyFilter::reset()
{
  estop_latched_ = false;
  zero(last_commanded_);
}

FilterOutput SafetyFilter::step(const FilterInputs & inputs, double dt)
{
  FilterOutput out;
  out.autonomy_enabled = inputs.autonomy_enabled;

  // ---------------------------------------------------------------------------------
  // 1. E-stop. Latching, absolute, and cleared only by an explicit reset() call.
  // ---------------------------------------------------------------------------------
  if (inputs.estop_engaged) {
    estop_latched_ = true;
  }
  if (estop_latched_) {
    zero(out.commanded);
    zero(out.requested);
    out.state = SafetyState::kEstop;
    out.source = ArbitrationSource::kEstop;
    note(out.active_limits, "ESTOP");
    // The acceleration limiter must not ease out of an E-stop on the next cycle.
    zero(last_commanded_);
    return out;
  }

  SafetyState state = SafetyState::kNominal;

  // ---------------------------------------------------------------------------------
  // 2. Arbitration. A human on the deadman outranks autonomy, always.
  // ---------------------------------------------------------------------------------
  const bool manual_fresh = inputs.manual_valid &&
    inputs.manual_age_sec <= limits_.command_timeout_sec;
  const bool autonomous_fresh = inputs.autonomous_valid &&
    inputs.autonomous_age_sec <= limits_.command_timeout_sec;

  // Loss of video blinds the VLA, so its output stops being trustworthy. Manual teleop is
  // deliberately unaffected -- a human on the sticks does not need the video link.
  const bool video_ok = !limits_.video_loss_stops_autonomy || inputs.video_valid;
  const bool video_fresh = video_ok &&
    (!limits_.video_loss_stops_autonomy || inputs.video_age_sec <= limits_.video_timeout_sec);

  if (inputs.deadman_held) {
    // The operator has asserted control, so manual owns the vehicle from here. If their
    // input has gone stale we brake -- we do NOT fall back to autonomy. The human
    // believes they are flying it, and silently handing the vehicle back to the model
    // while they hold the deadman is precisely the surprise this ladder exists to stop.
    if (manual_fresh) {
      out.source = ArbitrationSource::kManual;
      out.requested = inputs.manual;
    } else {
      out.source = ArbitrationSource::kOverride;
      zero(out.requested);
      note(out.active_limits, "MANUAL_TIMEOUT");
      escalate(state, SafetyState::kBraking);
    }
  } else if (inputs.autonomy_enabled && autonomous_fresh && video_fresh) {
    out.source = ArbitrationSource::kAutonomous;
    out.requested = inputs.autonomous;
  } else {
    // Nothing is both eligible and fresh, so the filter itself is flying: hold.
    out.source = ArbitrationSource::kOverride;
    zero(out.requested);

    if (!inputs.autonomy_enabled) {
      note(out.active_limits, "AUTONOMY_DISABLED");
      escalate(state, SafetyState::kBraking);
    } else if (!autonomous_fresh) {
      note(out.active_limits, inputs.autonomous_valid ? "CMD_TIMEOUT" : "NO_INPUT");
      escalate(state, SafetyState::kBraking);
    } else if (!video_fresh) {
      note(out.active_limits, "VIDEO_TIMEOUT");
      escalate(state, SafetyState::kBraking);
    }
  }

  geometry_msgs::msg::Twist cmd = out.requested;
  if (sanitize(cmd)) {
    // A NaN reaching the flight controller is not a degraded command, it is an undefined
    // one. Scrub it and treat the cycle as degraded.
    note(out.active_limits, "NON_FINITE");
    escalate(state, SafetyState::kLimited);
  }

  // ---------------------------------------------------------------------------------
  // 3-4. Vehicle health. Without a fresh, trusted state estimate the envelope below
  // cannot be enforced at all -- so the only safe command is no command.
  // ---------------------------------------------------------------------------------
  const auto & v = inputs.vehicle;
  const bool state_fresh = v.valid && v.age_sec <= limits_.state_timeout_sec;

  if (!state_fresh) {
    note(out.active_limits, v.valid ? "STATE_TIMEOUT" : "STATE_INVALID");
    escalate(state, SafetyState::kBraking);
    zero(cmd);
  } else if (!v.ekf_healthy) {
    note(out.active_limits, "EKF_UNHEALTHY");
    escalate(state, SafetyState::kBraking);
    zero(cmd);
  }

  // ---------------------------------------------------------------------------------
  // 5. Battery failsafe. Unknown charge (negative) is not treated as empty.
  // ---------------------------------------------------------------------------------
  bool forced_landing = false;
  if (state_fresh && v.battery_fraction >= 0.0) {
    if (v.battery_fraction <= limits_.battery_land_fraction) {
      note(out.active_limits, "BATTERY_LAND");
      escalate(state, SafetyState::kLanding);
      forced_landing = true;
      // Controlled descent, no horizontal motion, no yaw.
      zero(cmd);
      cmd.linear.z = -limits_.land_speed_mps;
    } else if (v.battery_fraction <= limits_.battery_warn_fraction) {
      note(out.active_limits, "BATTERY_WARN");
      escalate(state, SafetyState::kLimited);
    }
  }

  // ---------------------------------------------------------------------------------
  // 6. Absolute velocity clamps.
  // ---------------------------------------------------------------------------------
  if (clampHorizontal(cmd.linear.x, cmd.linear.y, limits_.max_horizontal_mps)) {
    note(out.active_limits, "SPEED_H");
    escalate(state, SafetyState::kLimited);
  }
  if (clampScalar(cmd.linear.z, limits_.max_vertical_mps)) {
    note(out.active_limits, "SPEED_V");
    escalate(state, SafetyState::kLimited);
  }
  if (clampScalar(cmd.angular.z, limits_.max_yaw_rate_rps)) {
    note(out.active_limits, "YAW_RATE");
    escalate(state, SafetyState::kLimited);
  }

  if (state_fresh) {
    // -------------------------------------------------------------------------------
    // 7. Altitude envelope. The rangefinder is the authoritative AGL source indoors --
    // it is a direct measurement, where EKF z is an integrated estimate that drifts.
    // -------------------------------------------------------------------------------
    const bool use_rangefinder = v.rangefinder_valid;
    const double altitude = use_rangefinder ? v.rangefinder_range : v.z;

    // Ground proximity: refuse descent below the stop distance, scale it down above.
    if (use_rangefinder && cmd.linear.z < 0.0) {
      if (v.rangefinder_range <= limits_.proximity_stop_m) {
        cmd.linear.z = 0.0;
        note(out.active_limits, "PROXIMITY");
        escalate(state, SafetyState::kBraking);
      } else if (v.rangefinder_range < limits_.proximity_slow_m) {
        const double span = limits_.proximity_slow_m - limits_.proximity_stop_m;
        if (span > 0.0) {
          const double scale = (v.rangefinder_range - limits_.proximity_stop_m) / span;
          cmd.linear.z *= scale;
          note(out.active_limits, "PROXIMITY");
          escalate(state, SafetyState::kLimited);
        }
      }
    }

    // A battery landing is allowed through the floor -- the vehicle is supposed to reach
    // the ground. Every other descent respects it.
    if (!forced_landing) {
      const double before = cmd.linear.z;
      cmd.linear.z = limitAgainstBounds(
        altitude, limits_.min_altitude_m, limits_.max_altitude_m,
        limits_.altitude_margin_m, cmd.linear.z);
      if (changed(before, cmd.linear.z)) {
        note(out.active_limits, before < 0.0 ? "ALT_FLOOR" : "ALT_CEILING");
        escalate(
          state, cmd.linear.z == 0.0 ? SafetyState::kBraking : SafetyState::kLimited);
      }
    }

    // -------------------------------------------------------------------------------
    // 8. Geofence. The command is body-FLU but the fence is ENU, so rotate through the
    // vehicle's yaw, constrain, and rotate back.
    // -------------------------------------------------------------------------------
    if (limits_.geofence_enabled) {
      double vx_enu = 0.0;
      double vy_enu = 0.0;
      bodyToEnu(cmd.linear.x, cmd.linear.y, v.yaw, vx_enu, vy_enu);

      const double vx_limited = limitAgainstBounds(
        v.x, limits_.geofence_min_x, limits_.geofence_max_x,
        limits_.geofence_margin_m, vx_enu);
      const double vy_limited = limitAgainstBounds(
        v.y, limits_.geofence_min_y, limits_.geofence_max_y,
        limits_.geofence_margin_m, vy_enu);

      if (changed(vx_enu, vx_limited)) {
        note(out.active_limits, "GEOFENCE_X");
        escalate(
          state, vx_limited == 0.0 ? SafetyState::kBraking : SafetyState::kLimited);
      }
      if (changed(vy_enu, vy_limited)) {
        note(out.active_limits, "GEOFENCE_Y");
        escalate(
          state, vy_limited == 0.0 ? SafetyState::kBraking : SafetyState::kLimited);
      }

      enuToBody(vx_limited, vy_limited, v.yaw, cmd.linear.x, cmd.linear.y);
    }
  }

  // ---------------------------------------------------------------------------------
  // 9. Acceleration limiting, last so that nothing above can be exceeded on the way to
  // the target, and asymmetric so a brake is never rate-limited into a slow roll.
  // ---------------------------------------------------------------------------------
  bool accel_bound = false;
  accel_bound |= rateLimit(
    last_commanded_.linear.x, cmd.linear.x, limits_.max_accel_mps2, limits_.max_decel_mps2, dt);
  accel_bound |= rateLimit(
    last_commanded_.linear.y, cmd.linear.y, limits_.max_accel_mps2, limits_.max_decel_mps2, dt);
  accel_bound |= rateLimit(
    last_commanded_.linear.z, cmd.linear.z, limits_.max_accel_mps2, limits_.max_decel_mps2, dt);
  const bool yaw_bound = rateLimit(
    last_commanded_.angular.z, cmd.angular.z,
    limits_.max_yaw_accel_rps2, limits_.max_yaw_accel_rps2, dt);

  if (accel_bound) {
    note(out.active_limits, "ACCEL");
    escalate(state, SafetyState::kLimited);
  }
  if (yaw_bound) {
    note(out.active_limits, "YAW_ACCEL");
    escalate(state, SafetyState::kLimited);
  }

  out.commanded = last_commanded_;
  out.state = state;
  return out;
}

}  // namespace j10_safety
