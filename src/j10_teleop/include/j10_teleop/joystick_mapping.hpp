// Joystick -> body velocity, as pure functions.
//
// docs/ARCHITECTURE.md gives teleop_override_node the highest arbitration priority: human
// input instantly preempts autonomy. That makes the mapping below safety-relevant in a way
// a joystick usually is not -- an axis that sticks, a deadman that latches the wrong way, or
// a button index that shifts between controller models all fail in the direction of
// unrequested motion.
//
// So it lives here, free of ROS, and is unit-tested exhaustively. The node does I/O only.

#ifndef J10_TELEOP__JOYSTICK_MAPPING_HPP_
#define J10_TELEOP__JOYSTICK_MAPPING_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace j10_teleop
{

/// One body-FLU velocity request: x forward, y left, z up, yaw counter-clockwise.
struct BodyVelocity
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw_rate{0.0};

  bool isZero() const
  {
    return x == 0.0 && y == 0.0 && z == 0.0 && yaw_rate == 0.0;
  }
};

/// Which entries of a sensor_msgs/Joy carry what.
///
/// Indices rather than names because Joy has none -- the mapping is per controller model,
/// which is exactly why it is configuration and not a constant.
struct AxisMap
{
  int forward{1};        ///< left stick vertical
  int left{0};           ///< left stick horizontal
  int up{4};             ///< right stick vertical
  int yaw{3};            ///< right stick horizontal

  bool invert_forward{false};
  bool invert_left{false};
  bool invert_up{false};
  bool invert_yaw{false};

  int deadman_button{4};  ///< left shoulder
  int estop_button{1};    ///< B / circle

  /// Optional. -1 disables; when set, releasing latched E-stop needs this held too, so a
  /// single fumbled press cannot clear a stop that was deliberately triggered.
  int estop_reset_button{-1};
};

struct ShapingParams
{
  /// Below this, an axis reads as exactly zero. Sticks do not return to a true centre, and
  /// without a deadzone a resting controller commands a slow permanent drift.
  double deadzone{0.08};

  /// 0 = linear, 1 = fully cubic. Cubic gives fine control near centre while keeping full
  /// authority at the stops -- the difference between nudging a drone across a room and
  /// fighting it.
  double expo{0.5};

  double max_linear_mps{0.5};
  double max_vertical_mps{0.3};
  double max_yaw_rate_rps{0.8};
};

/// Rescale so the usable range starts at the deadzone edge rather than jumping.
///
/// A naive `abs(v) < deadzone ? 0 : v` steps discontinuously from 0 to `deadzone` the moment
/// the stick crosses the threshold. Rescaling makes the first millimetre of travel produce
/// the smallest command, which is what "fine control" actually means.
inline double applyDeadzone(double value, double deadzone)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  const double dz = std::clamp(deadzone, 0.0, 0.99);
  const double magnitude = std::abs(value);
  if (magnitude <= dz) {
    return 0.0;
  }
  const double scaled = (magnitude - dz) / (1.0 - dz);
  return std::copysign(std::min(scaled, 1.0), value);
}

/// Blend linear and cubic response. Sign-preserving, and fixed at the endpoints.
inline double applyExpo(double value, double expo)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  const double e = std::clamp(expo, 0.0, 1.0);
  return (1.0 - e) * value + e * value * value * value;
}

/// Read one axis, or 0 when the index is out of range or the value is not finite.
///
/// Out-of-range is a real case, not defensive noise: plugging in a controller with fewer
/// axes than the config expects would otherwise read past the end of the array. Returning
/// zero degrades to "that axis is centred", which is the safe reading.
inline double readAxis(const std::vector<float> & axes, int index, bool invert)
{
  if (index < 0 || static_cast<std::size_t>(index) >= axes.size()) {
    return 0.0;
  }
  const double value = static_cast<double>(axes[static_cast<std::size_t>(index)]);
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return invert ? -value : value;
}

/// Read one button as pressed/not, false when the index is out of range.
inline bool readButton(const std::vector<int32_t> & buttons, int index)
{
  if (index < 0 || static_cast<std::size_t>(index) >= buttons.size()) {
    return false;
  }
  return buttons[static_cast<std::size_t>(index)] != 0;
}

/// Map raw axes to a body velocity. Deadzone, then expo, then scale.
///
/// The order matters. Expo before deadzone would compress the deadzone itself, making the
/// threshold depend on the curve; scaling before expo would apply the curve to metres per
/// second rather than to normalised stick travel, so the shape would change whenever a
/// limit changed.
inline BodyVelocity mapAxes(
  const std::vector<float> & axes, const AxisMap & map, const ShapingParams & shaping)
{
  const auto shape = [&shaping](double raw) {
      return applyExpo(applyDeadzone(raw, shaping.deadzone), shaping.expo);
    };

  BodyVelocity out;
  out.x = shape(readAxis(axes, map.forward, map.invert_forward)) *
    std::abs(shaping.max_linear_mps);
  out.y = shape(readAxis(axes, map.left, map.invert_left)) *
    std::abs(shaping.max_linear_mps);
  out.z = shape(readAxis(axes, map.up, map.invert_up)) *
    std::abs(shaping.max_vertical_mps);
  out.yaw_rate = shape(readAxis(axes, map.yaw, map.invert_yaw)) *
    std::abs(shaping.max_yaw_rate_rps);
  return out;
}

/// What one Joy message means.
struct TeleopCommand
{
  BodyVelocity velocity;      ///< zero unless the deadman is held
  bool deadman_held{false};
  bool estop_pressed{false};
  bool estop_reset_requested{false};
};

/// Interpret a Joy message.
///
/// The deadman rule is absolute: with the button released the velocity is zero, whatever
/// the sticks say. A deadman that merely lowers a limit, or that is checked downstream, is
/// not a deadman -- it has to zero the command at the point the command is created.
inline TeleopCommand interpret(
  const std::vector<float> & axes,
  const std::vector<int32_t> & buttons,
  const AxisMap & map,
  const ShapingParams & shaping)
{
  TeleopCommand cmd;
  cmd.deadman_held = readButton(buttons, map.deadman_button);
  cmd.estop_pressed = readButton(buttons, map.estop_button);
  cmd.estop_reset_requested = map.estop_reset_button >= 0 &&
    readButton(buttons, map.estop_reset_button) && cmd.deadman_held;

  if (cmd.deadman_held) {
    cmd.velocity = mapAxes(axes, map, shaping);
  }
  return cmd;
}

}  // namespace j10_teleop

#endif  // J10_TELEOP__JOYSTICK_MAPPING_HPP_
