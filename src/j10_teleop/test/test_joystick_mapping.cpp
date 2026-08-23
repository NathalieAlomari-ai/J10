// Tests for the joystick mapping.
//
// Teleop has the highest arbitration priority in the stack, so a bug here outranks the
// safety filter's opinion in exactly the way a bug should not. The cases below concentrate
// on the ways a joystick fails toward *motion*: a stick that never quite centres, a
// controller with fewer axes than configured, a NaN from a disconnecting device, and a
// deadman that stops being absolute.

#include <cmath>
#include <vector>

#include "gtest/gtest.h"

#include "j10_teleop/joystick_mapping.hpp"

using j10_teleop::AxisMap;
using j10_teleop::BodyVelocity;
using j10_teleop::ShapingParams;
using j10_teleop::applyDeadzone;
using j10_teleop::applyExpo;
using j10_teleop::interpret;
using j10_teleop::mapAxes;
using j10_teleop::readAxis;
using j10_teleop::readButton;

namespace
{

/// Axes long enough for the default map, all centred.
std::vector<float> centredAxes()
{
  return std::vector<float>(6, 0.0f);
}

/// Buttons long enough for the default map, none pressed.
std::vector<int32_t> noButtons()
{
  return std::vector<int32_t>(8, 0);
}

ShapingParams linearShaping()
{
  ShapingParams s;
  s.expo = 0.0;       // isolate scaling from curve shape
  s.deadzone = 0.0;
  return s;
}

}  // namespace

// --- Deadzone -------------------------------------------------------------------------

TEST(Deadzone, CentreReadsAsExactlyZero) {
  EXPECT_DOUBLE_EQ(applyDeadzone(0.0, 0.1), 0.0);
}

TEST(Deadzone, RestingStickDriftIsSuppressed) {
  // The case the deadzone exists for: sticks do not return to a true centre, and without
  // this a controller sitting on a desk commands a slow permanent drift.
  EXPECT_DOUBLE_EQ(applyDeadzone(0.05, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(applyDeadzone(-0.05, 0.1), 0.0);
}

TEST(Deadzone, ExactlyAtTheThresholdIsStillZero) {
  EXPECT_DOUBLE_EQ(applyDeadzone(0.1, 0.1), 0.0);
}

TEST(Deadzone, OutputRescalesFromZeroRatherThanSteppingToTheThreshold) {
  // A naive implementation jumps from 0 to 0.1 the instant the stick crosses the deadzone.
  // Rescaling means the first movement past the threshold produces the *smallest* command,
  // which is what fine control means.
  const double just_past = applyDeadzone(0.1001, 0.1);
  EXPECT_GT(just_past, 0.0);
  EXPECT_LT(just_past, 0.01) << "should rescale from zero, not step to the deadzone value";
}

TEST(Deadzone, FullDeflectionStillReachesFullScale) {
  // The other half of rescaling: the deadzone must not cost authority at the stops.
  EXPECT_DOUBLE_EQ(applyDeadzone(1.0, 0.1), 1.0);
  EXPECT_DOUBLE_EQ(applyDeadzone(-1.0, 0.1), -1.0);
}

TEST(Deadzone, SignIsPreserved) {
  EXPECT_LT(applyDeadzone(-0.5, 0.1), 0.0);
  EXPECT_GT(applyDeadzone(0.5, 0.1), 0.0);
}

TEST(Deadzone, OverTravelIsClampedToUnit) {
  // Some drivers report slightly beyond +/-1.0 on a hard deflection.
  EXPECT_DOUBLE_EQ(applyDeadzone(1.5, 0.1), 1.0);
  EXPECT_DOUBLE_EQ(applyDeadzone(-1.5, 0.1), -1.0);
}

TEST(Deadzone, NonFiniteInputBecomesZero) {
  // A disconnecting device can emit NaN. Untreated it propagates all the way to a setpoint.
  EXPECT_DOUBLE_EQ(applyDeadzone(std::nan(""), 0.1), 0.0);
  EXPECT_DOUBLE_EQ(applyDeadzone(INFINITY, 0.1), 0.0);
}

TEST(Deadzone, AbsurdDeadzoneIsClampedRatherThanDisablingTheStick) {
  // deadzone >= 1.0 would divide by zero in the rescale. Clamping keeps some travel usable.
  const double out = applyDeadzone(1.0, 5.0);
  EXPECT_TRUE(std::isfinite(out));
}

TEST(Deadzone, NegativeDeadzoneIsTreatedAsZero) {
  EXPECT_DOUBLE_EQ(applyDeadzone(0.5, -1.0), 0.5);
}

// --- Expo -----------------------------------------------------------------------------

TEST(Expo, ZeroExpoIsIdentity) {
  EXPECT_DOUBLE_EQ(applyExpo(0.5, 0.0), 0.5);
}

TEST(Expo, FullExpoIsCubic) {
  EXPECT_DOUBLE_EQ(applyExpo(0.5, 1.0), 0.125);
}

TEST(Expo, EndpointsAreUnchangedAtAnyExpo) {
  // Expo must buy sensitivity near centre without costing authority at the stops.
  for (double e : {0.0, 0.25, 0.5, 0.75, 1.0}) {
    EXPECT_NEAR(applyExpo(1.0, e), 1.0, 1e-12) << "expo=" << e;
    EXPECT_NEAR(applyExpo(-1.0, e), -1.0, 1e-12) << "expo=" << e;
    EXPECT_NEAR(applyExpo(0.0, e), 0.0, 1e-12) << "expo=" << e;
  }
}

TEST(Expo, SoftensTheMiddleOfTheRange) {
  EXPECT_LT(applyExpo(0.5, 0.5), 0.5);
}

TEST(Expo, IsSignSymmetric) {
  EXPECT_DOUBLE_EQ(applyExpo(-0.5, 0.7), -applyExpo(0.5, 0.7));
}

TEST(Expo, IsMonotonic) {
  // A non-monotonic curve would make the drone slow down as the stick is pushed further --
  // easy to introduce with a careless blend, and deeply confusing to fly.
  double previous = applyExpo(-1.0, 0.5);
  for (int i = -99; i <= 100; ++i) {
    const double current = applyExpo(i / 100.0, 0.5);
    EXPECT_GE(current, previous) << "at " << i / 100.0;
    previous = current;
  }
}

TEST(Expo, OutOfRangeExpoIsClamped) {
  EXPECT_DOUBLE_EQ(applyExpo(0.5, -1.0), 0.5);
  EXPECT_DOUBLE_EQ(applyExpo(0.5, 2.0), 0.125);
}

TEST(Expo, NonFiniteInputBecomesZero) {
  EXPECT_DOUBLE_EQ(applyExpo(std::nan(""), 0.5), 0.0);
}

// --- Axis and button reads --------------------------------------------------------------

// Joy carries float32 while the mapping works in double, so these use values that are
// exactly representable in binary floating point (0.25, 0.5). 0.2f widened to double is
// 0.20000000298..., which would fail an exact comparison for reasons that have nothing to
// do with the code under test.
TEST(ReadAxis, ReadsTheConfiguredIndex) {
  const std::vector<float> axes{0.125f, 0.25f, 0.5f};
  EXPECT_DOUBLE_EQ(readAxis(axes, 1, false), 0.25);
}

TEST(ReadAxis, InvertFlipsTheSign) {
  const std::vector<float> axes{0.125f, 0.25f, 0.5f};
  EXPECT_DOUBLE_EQ(readAxis(axes, 1, true), -0.25);
}

TEST(ReadAxis, OutOfRangeIndexReadsAsCentred) {
  // Plugging in a controller with fewer axes than the config expects must not read past the
  // end of the array. Zero degrades to "that axis is centred", which is the safe reading.
  const std::vector<float> axes{0.125f, 0.25f};
  EXPECT_DOUBLE_EQ(readAxis(axes, 7, false), 0.0);
  EXPECT_DOUBLE_EQ(readAxis(axes, -1, false), 0.0);
}

TEST(ReadAxis, EmptyAxesReadAsCentred) {
  EXPECT_DOUBLE_EQ(readAxis({}, 0, false), 0.0);
}

TEST(ReadAxis, NonFiniteAxisValueBecomesZero) {
  const std::vector<float> axes{std::nanf("")};
  EXPECT_DOUBLE_EQ(readAxis(axes, 0, false), 0.0);
}

TEST(ReadButton, ReadsPressedAndReleased) {
  const std::vector<int32_t> buttons{0, 1, 0};
  EXPECT_FALSE(readButton(buttons, 0));
  EXPECT_TRUE(readButton(buttons, 1));
}

TEST(ReadButton, OutOfRangeIndexIsNotPressed) {
  // Critically, this means a misconfigured deadman index reads as *released*, so the
  // vehicle refuses to move -- rather than as held, which would remove the deadman entirely.
  const std::vector<int32_t> buttons{1, 1};
  EXPECT_FALSE(readButton(buttons, 9));
  EXPECT_FALSE(readButton(buttons, -1));
}

TEST(ReadButton, AnyNonZeroCountsAsPressed) {
  const std::vector<int32_t> buttons{2};
  EXPECT_TRUE(readButton(buttons, 0));
}

// --- Axis mapping -------------------------------------------------------------------------

TEST(MapAxes, FullForwardGivesMaxLinearSpeed) {
  AxisMap map;
  ShapingParams shaping = linearShaping();
  shaping.max_linear_mps = 0.5;
  auto axes = centredAxes();
  axes[static_cast<std::size_t>(map.forward)] = 1.0f;

  const BodyVelocity out = mapAxes(axes, map, shaping);
  EXPECT_DOUBLE_EQ(out.x, 0.5);
  EXPECT_DOUBLE_EQ(out.y, 0.0);
  EXPECT_DOUBLE_EQ(out.z, 0.0);
  EXPECT_DOUBLE_EQ(out.yaw_rate, 0.0);
}

TEST(MapAxes, EachAxisDrivesOnlyItsOwnComponent) {
  // Guards against a copy-paste slip in the mapping -- the kind that flies sideways when
  // asked to climb and is very hard to read back from a log.
  AxisMap map;
  ShapingParams shaping = linearShaping();

  struct Case { int index; const char * name; };
  const Case cases[] = {
    {map.forward, "forward"}, {map.left, "left"}, {map.up, "up"}, {map.yaw, "yaw"},
  };

  for (const auto & c : cases) {
    auto axes = centredAxes();
    axes[static_cast<std::size_t>(c.index)] = 1.0f;
    const BodyVelocity out = mapAxes(axes, map, shaping);
    int nonzero = 0;
    for (double v : {out.x, out.y, out.z, out.yaw_rate}) {
      if (v != 0.0) {++nonzero;}
    }
    EXPECT_EQ(nonzero, 1) << c.name << " axis moved more than one component";
  }
}

TEST(MapAxes, VerticalAndYawUseTheirOwnLimits) {
  AxisMap map;
  ShapingParams shaping = linearShaping();
  shaping.max_linear_mps = 0.5;
  shaping.max_vertical_mps = 0.3;
  shaping.max_yaw_rate_rps = 0.8;

  auto axes = centredAxes();
  axes[static_cast<std::size_t>(map.up)] = 1.0f;
  axes[static_cast<std::size_t>(map.yaw)] = 1.0f;

  const BodyVelocity out = mapAxes(axes, map, shaping);
  EXPECT_DOUBLE_EQ(out.z, 0.3);
  EXPECT_DOUBLE_EQ(out.yaw_rate, 0.8);
}

TEST(MapAxes, InversionFlipsTheCommandedDirection) {
  AxisMap map;
  map.invert_forward = true;
  ShapingParams shaping = linearShaping();
  auto axes = centredAxes();
  axes[static_cast<std::size_t>(map.forward)] = 1.0f;

  EXPECT_LT(mapAxes(axes, map, shaping).x, 0.0);
}

TEST(MapAxes, CentredSticksCommandNothing) {
  EXPECT_TRUE(mapAxes(centredAxes(), AxisMap{}, ShapingParams{}).isZero());
}

TEST(MapAxes, NegativeLimitsCannotInvertTheMapping) {
  // A negative max in config is a typo, not a request to reverse the controls. Taking the
  // magnitude means the worst outcome is the configured speed, never a mirrored axis.
  AxisMap map;
  ShapingParams shaping = linearShaping();
  shaping.max_linear_mps = -0.5;
  auto axes = centredAxes();
  axes[static_cast<std::size_t>(map.forward)] = 1.0f;

  EXPECT_DOUBLE_EQ(mapAxes(axes, map, shaping).x, 0.5);
}

TEST(MapAxes, ShortAxisVectorYieldsZeroRatherThanReadingPastTheEnd) {
  const std::vector<float> axes{1.0f};  // shorter than the default map needs
  const BodyVelocity out = mapAxes(axes, AxisMap{}, linearShaping());
  EXPECT_TRUE(std::isfinite(out.x));
  EXPECT_TRUE(std::isfinite(out.y));
  EXPECT_TRUE(std::isfinite(out.z));
  EXPECT_TRUE(std::isfinite(out.yaw_rate));
}

TEST(MapAxes, OutputNeverExceedsTheConfiguredLimits) {
  // Sweep the whole stick envelope, including over-travel, and confirm nothing escapes.
  AxisMap map;
  ShapingParams shaping;
  shaping.max_linear_mps = 0.5;
  shaping.max_vertical_mps = 0.3;
  shaping.max_yaw_rate_rps = 0.8;

  for (int i = -15; i <= 15; ++i) {
    const float v = static_cast<float>(i) / 10.0f;  // -1.5 .. 1.5
    std::vector<float> axes(6, v);
    const BodyVelocity out = mapAxes(axes, map, shaping);
    EXPECT_LE(std::abs(out.x), 0.5 + 1e-9) << "at " << v;
    EXPECT_LE(std::abs(out.y), 0.5 + 1e-9) << "at " << v;
    EXPECT_LE(std::abs(out.z), 0.3 + 1e-9) << "at " << v;
    EXPECT_LE(std::abs(out.yaw_rate), 0.8 + 1e-9) << "at " << v;
  }
}

// --- Deadman ------------------------------------------------------------------------------

TEST(Deadman, ReleasedMeansZeroVelocityWhateverTheSticksSay) {
  // The single most important assertion in this file. A deadman that only lowers a limit,
  // or that is enforced downstream, is not a deadman.
  AxisMap map;
  std::vector<float> axes(6, 1.0f);  // every stick at full deflection
  const auto cmd = interpret(axes, noButtons(), map, ShapingParams{});

  EXPECT_FALSE(cmd.deadman_held);
  EXPECT_TRUE(cmd.velocity.isZero());
}

TEST(Deadman, HeldPassesTheSticksThrough) {
  AxisMap map;
  auto axes = centredAxes();
  axes[static_cast<std::size_t>(map.forward)] = 1.0f;
  auto buttons = noButtons();
  buttons[static_cast<std::size_t>(map.deadman_button)] = 1;

  const auto cmd = interpret(axes, buttons, map, linearShaping());
  EXPECT_TRUE(cmd.deadman_held);
  EXPECT_GT(cmd.velocity.x, 0.0);
}

TEST(Deadman, MisconfiguredButtonIndexFailsToReleasedNotHeld) {
  // Fail-safe direction check: a bad index must remove authority, never grant it.
  AxisMap map;
  map.deadman_button = 99;
  std::vector<float> axes(6, 1.0f);
  std::vector<int32_t> buttons(8, 1);  // everything pressed

  const auto cmd = interpret(axes, buttons, map, ShapingParams{});
  EXPECT_FALSE(cmd.deadman_held);
  EXPECT_TRUE(cmd.velocity.isZero());
}

// --- E-stop -------------------------------------------------------------------------------

TEST(Estop, IsReportedIndependentlyOfTheDeadman) {
  // Stopping must not require also holding the deadman -- the operator reaching for E-stop
  // is the operator letting go of everything else.
  AxisMap map;
  auto buttons = noButtons();
  buttons[static_cast<std::size_t>(map.estop_button)] = 1;

  const auto cmd = interpret(centredAxes(), buttons, map, ShapingParams{});
  EXPECT_TRUE(cmd.estop_pressed);
  EXPECT_FALSE(cmd.deadman_held);
}

TEST(Estop, NotPressedByDefault) {
  EXPECT_FALSE(interpret(centredAxes(), noButtons(), AxisMap{}, ShapingParams{}).estop_pressed);
}

TEST(EstopReset, IsDisabledWhenUnconfigured) {
  AxisMap map;
  map.estop_reset_button = -1;
  std::vector<int32_t> buttons(8, 1);

  EXPECT_FALSE(
    interpret(centredAxes(), buttons, map, ShapingParams{}).estop_reset_requested);
}

TEST(EstopReset, RequiresTheDeadmanToBeHeldAsWell) {
  // A two-handed gesture on purpose: clearing a stop that someone deliberately triggered
  // should not be reachable by one fumbled press.
  AxisMap map;
  map.estop_reset_button = 6;
  auto buttons = noButtons();
  buttons[6] = 1;

  EXPECT_FALSE(
    interpret(centredAxes(), buttons, map, ShapingParams{}).estop_reset_requested);

  buttons[static_cast<std::size_t>(map.deadman_button)] = 1;
  EXPECT_TRUE(
    interpret(centredAxes(), buttons, map, ShapingParams{}).estop_reset_requested);
}

// --- Invariant ------------------------------------------------------------------------------

TEST(Invariant, NoInputCombinationEverProducesANonFiniteCommand) {
  // Exhaustive-ish sweep over hostile inputs. Whatever arrives, what leaves must be a
  // number -- a NaN here would reach the safety filter and then the flight controller.
  const std::vector<float> hostile{
    0.0f, 1.0f, -1.0f, 1e9f, -1e9f, std::nanf(""), INFINITY, -INFINITY};

  AxisMap map;
  ShapingParams shaping;
  for (float value : hostile) {
    for (std::size_t length : {std::size_t{0}, std::size_t{1}, std::size_t{6}}) {
      const std::vector<float> axes(length, value);
      for (int deadman : {0, 1}) {
        std::vector<int32_t> buttons(8, 0);
        buttons[static_cast<std::size_t>(map.deadman_button)] = deadman;
        const auto cmd = interpret(axes, buttons, map, shaping);
        EXPECT_TRUE(std::isfinite(cmd.velocity.x));
        EXPECT_TRUE(std::isfinite(cmd.velocity.y));
        EXPECT_TRUE(std::isfinite(cmd.velocity.z));
        EXPECT_TRUE(std::isfinite(cmd.velocity.yaw_rate));
      }
    }
  }
}
