#ifndef J10_SAFETY__SAFETY_FILTER_HPP_
#define J10_SAFETY__SAFETY_FILTER_HPP_

#include <geometry_msgs/msg/twist.hpp>

#include "j10_safety/safety_types.hpp"

namespace j10_safety
{

/// The independent guardian. The only component with authority to veto a command.
///
/// Pure logic, no ROS, no clocks of its own -- `step()` is handed the elapsed time. That
/// makes every limit, watchdog and escalation path testable deterministically without a
/// simulator, which is the whole reason this class exists separately from the node.
///
/// Order of enforcement, and it matters:
///
///   1. E-stop latch          -- absolute, overrides everything, requires explicit reset
///   2. Arbitration           -- ESTOP > MANUAL (deadman) > OVERRIDE > AUTONOMOUS
///   3. Input freshness       -- a stale winner commands nothing
///   4. Vehicle health        -- no fresh state, or unhealthy EKF, means we cannot verify
///                               the envelope, so we must not fly
///   5. Battery failsafe      -- may force a descent
///   6. Velocity clamps       -- absolute speed ceilings
///   7. Altitude envelope     -- floor and ceiling, rangefinder preferred over EKF z
///   8. Geofence              -- box in ENU, applied to the rotated horizontal command
///   9. Acceleration limits   -- applied last so every clamp above is still respected,
///                               and asymmetric so braking is never rate-limited away
///
/// The default output is zero velocity. Every failure path decays to hover rather than
/// repeating the last command -- repeating stale commands is how offboard systems fly
/// into walls.
class SafetyFilter
{
public:
  SafetyFilter() = default;
  explicit SafetyFilter(const Limits & limits);

  /// Run one cycle. `dt` is the elapsed time since the previous call, in seconds.
  FilterOutput step(const FilterInputs & inputs, double dt);

  /// Clear the E-stop latch and the acceleration-limiter memory. This is the explicit
  /// operator reset the E-stop contract requires; nothing else clears the latch.
  void reset();

  const Limits & limits() const {return limits_;}
  void setLimits(const Limits & limits) {limits_ = limits;}
  bool estopLatched() const {return estop_latched_;}

private:
  /// True when the flight controller is still in a mode that acts on offboard setpoints.
  bool offboardAccepted(const VehicleSnapshot & v) const;

  Limits limits_{};
  geometry_msgs::msg::Twist last_commanded_{};
  bool estop_latched_{false};
};

}  // namespace j10_safety

#endif  // J10_SAFETY__SAFETY_FILTER_HPP_
