// Unit tests for the motion shaper.
//
// No ROS graph, no simulator. The behaviour under test is what the vehicle does when the
// VLA stalls, returns garbage, or goes silent -- which is precisely the behaviour that is
// hardest to provoke on demand in simulation and most expensive to get wrong in flight.

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "j10_control/motion_shaper.hpp"

using j10_control::ActionType;
using j10_control::Intent;
using j10_control::MotionShaper;
using j10_control::ShaperLimits;
using j10_control::ShaperReason;

namespace
{

/// Limits with shaping effectively disabled, so a test probing target selection sees the
/// target directly rather than one acceleration step toward it.
ShaperLimits instant()
{
  ShaperLimits l;
  l.max_linear_accel_mps2 = 1e6;
  l.max_linear_decel_mps2 = 1e6;
  l.max_yaw_accel_rps2 = 1e6;
  l.max_yaw_decel_rps2 = 1e6;
  return l;
}

Intent moveIntent(double vx, double duration = 1.0, double confidence = 0.9)
{
  Intent i;
  i.valid = true;
  i.action = ActionType::kMove;
  i.velocity.linear.x = vx;
  i.age_sec = 0.0;
  i.duration_sec = duration;
  i.confidence = confidence;
  return i;
}

constexpr double kDt = 1.0 / 30.0;

}  // namespace

// =======================================================================================
// Following a live intent
// =======================================================================================

TEST(Shaper, FollowsAValidIntent)
{
  MotionShaper shaper(instant());
  const auto out = shaper.step(moveIntent(0.4), kDt);

  EXPECT_EQ(out.reason, ShaperReason::kActive);
  EXPECT_TRUE(out.following);
  EXPECT_DOUBLE_EQ(out.command.linear.x, 0.4);
}

TEST(Shaper, NewIntentReplacesTheOld)
{
  MotionShaper shaper(instant());
  shaper.step(moveIntent(0.4), kDt);
  const auto out = shaper.step(moveIntent(-0.2), kDt);

  EXPECT_DOUBLE_EQ(out.command.linear.x, -0.2);
}

TEST(Shaper, TurnAndExploreAreFlownLikeMove)
{
  MotionShaper shaper(instant());

  auto turn = moveIntent(0.0);
  turn.action = ActionType::kTurn;
  turn.velocity.angular.z = 0.3;
  auto out = shaper.step(turn, kDt);
  EXPECT_EQ(out.reason, ShaperReason::kActive);
  EXPECT_DOUBLE_EQ(out.command.angular.z, 0.3);

  auto explore = moveIntent(0.25);
  explore.action = ActionType::kExplore;
  out = shaper.step(explore, kDt);
  EXPECT_EQ(out.reason, ShaperReason::kActive);
  EXPECT_DOUBLE_EQ(out.command.linear.x, 0.25);
}

// =======================================================================================
// Expiry and silence -- the reason this layer exists
// =======================================================================================

TEST(Expiry, IntentPastItsDurationDecaysToZero)
{
  MotionShaper shaper(instant());
  ASSERT_DOUBLE_EQ(shaper.step(moveIntent(0.4, 1.0), kDt).command.linear.x, 0.4);

  auto stale = moveIntent(0.4, 1.0);
  stale.age_sec = 1.5;
  const auto out = shaper.step(stale, kDt);

  EXPECT_EQ(out.reason, ShaperReason::kExpired);
  EXPECT_FALSE(out.following);
  EXPECT_DOUBLE_EQ(out.command.linear.x, 0.0);
}

TEST(Expiry, IntentExactlyAtItsDurationIsStillValid)
{
  MotionShaper shaper(instant());
  auto edge = moveIntent(0.4, 1.0);
  edge.age_sec = 1.0;

  const auto out = shaper.step(edge, kDt);

  EXPECT_EQ(out.reason, ShaperReason::kActive);
  EXPECT_DOUBLE_EQ(out.command.linear.x, 0.4);
}

TEST(Expiry, ZeroOrNegativeDurationIsMalformedNotInfinite)
{
  // NavIntent's contract says duration must be > 0. A zero window must not be read as
  // "valid forever".
  MotionShaper shaper(instant());

  auto zero_duration = moveIntent(0.4, 0.0);
  EXPECT_EQ(shaper.step(zero_duration, kDt).reason, ShaperReason::kExpired);

  auto negative = moveIntent(0.4, -5.0);
  EXPECT_EQ(shaper.step(negative, kDt).reason, ShaperReason::kExpired);
}

TEST(Expiry, ModelSilenceDecaysToZeroAndStaysThere)
{
  MotionShaper shaper(instant());
  ASSERT_DOUBLE_EQ(shaper.step(moveIntent(0.5), kDt).command.linear.x, 0.5);

  Intent silence;   // valid == false
  for (int i = 0; i < 20; ++i) {
    const auto out = shaper.step(silence, kDt);
    EXPECT_EQ(out.reason, ShaperReason::kNoIntent);
    EXPECT_DOUBLE_EQ(out.command.linear.x, 0.0) << "cycle " << i;
  }
}

TEST(Expiry, NeverHoldsTheLastCommand)
{
  // The single property this whole two-rate design exists to guarantee.
  ShaperLimits limits;
  limits.max_linear_decel_mps2 = 1.0;
  MotionShaper shaper(limits);

  for (int i = 0; i < 200; ++i) {
    shaper.step(moveIntent(0.5), kDt);
  }
  ASSERT_NEAR(shaper.current().linear.x, 0.5, 1e-6);

  Intent silence;
  double last = shaper.current().linear.x;
  for (int i = 0; i < 200; ++i) {
    const double now = shaper.step(silence, kDt).command.linear.x;
    ASSERT_LE(now, last + 1e-12) << "must be monotonically decaying, cycle " << i;
    last = now;
  }
  EXPECT_DOUBLE_EQ(last, 0.0);
}

// =======================================================================================
// Confidence
// =======================================================================================

TEST(Confidence, LowConfidenceIntentIsRejected)
{
  auto limits = instant();
  limits.min_confidence = 0.5;
  MotionShaper shaper(limits);

  const auto out = shaper.step(moveIntent(0.4, 1.0, 0.2), kDt);

  EXPECT_EQ(out.reason, ShaperReason::kLowConfidence);
  EXPECT_FALSE(out.following);
  EXPECT_DOUBLE_EQ(out.command.linear.x, 0.0);
}

TEST(Confidence, AtThresholdIsAccepted)
{
  auto limits = instant();
  limits.min_confidence = 0.5;
  MotionShaper shaper(limits);

  const auto out = shaper.step(moveIntent(0.4, 1.0, 0.5), kDt);

  EXPECT_EQ(out.reason, ShaperReason::kActive);
  EXPECT_DOUBLE_EQ(out.command.linear.x, 0.4);
}

TEST(Confidence, OptionalScalingIsOffByDefault)
{
  auto limits = instant();
  MotionShaper shaper(limits);

  const auto out = shaper.step(moveIntent(0.4, 1.0, 0.5), kDt);
  EXPECT_DOUBLE_EQ(out.command.linear.x, 0.4) << "unscaled unless explicitly enabled";
}

TEST(Confidence, ScalingAppliesWhenEnabled)
{
  auto limits = instant();
  limits.scale_by_confidence = true;
  MotionShaper shaper(limits);

  const auto out = shaper.step(moveIntent(0.4, 1.0, 0.5), kDt);
  EXPECT_NEAR(out.command.linear.x, 0.2, 1e-12);
}

// =======================================================================================
// Action taxonomy
// =======================================================================================

TEST(Action, HoldCommandsZeroWithoutBeingAnError)
{
  MotionShaper shaper(instant());
  auto hold = moveIntent(0.4);
  hold.action = ActionType::kHold;

  const auto out = shaper.step(hold, kDt);

  EXPECT_EQ(out.reason, ShaperReason::kHold);
  EXPECT_FALSE(out.following);
  EXPECT_DOUBLE_EQ(out.command.linear.x, 0.0)
    << "HOLD must ignore any velocity the model attached";
}

TEST(Action, LandDescendsAtTheConfiguredRateWithNoHorizontalMotion)
{
  auto limits = instant();
  limits.land_speed_mps = 0.25;
  MotionShaper shaper(limits);

  auto land = moveIntent(0.4);
  land.action = ActionType::kLand;
  land.velocity.linear.y = 0.3;

  const auto out = shaper.step(land, kDt);

  EXPECT_EQ(out.reason, ShaperReason::kLand);
  EXPECT_DOUBLE_EQ(out.command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(out.command.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(out.command.linear.z, -0.25);
}

// =======================================================================================
// Trapezoidal shaping
// =======================================================================================

TEST(Shaping, RampsRatherThanSteppingOnANewIntent)
{
  ShaperLimits limits;
  limits.max_linear_accel_mps2 = 0.5;
  MotionShaper shaper(limits);

  const auto first = shaper.step(moveIntent(1.0), 0.1);
  EXPECT_NEAR(first.command.linear.x, 0.05, 1e-12) << "0.5 m/s^2 over 0.1 s";

  const auto second = shaper.step(moveIntent(1.0), 0.1);
  EXPECT_NEAR(second.command.linear.x, 0.10, 1e-12);
}

TEST(Shaping, DecelerationUsesItsOwnLargerBudget)
{
  ShaperLimits limits;
  limits.max_linear_accel_mps2 = 0.5;
  limits.max_linear_decel_mps2 = 2.0;
  MotionShaper shaper(limits);

  for (int i = 0; i < 100; ++i) {
    shaper.step(moveIntent(1.0), 0.1);
  }
  ASSERT_NEAR(shaper.current().linear.x, 1.0, 1e-9);

  // 1.0 m/s at 2.0 m/s^2 over 0.1 s -> 0.2 removed, not 0.05.
  const auto out = shaper.step(moveIntent(0.0), 0.1);
  EXPECT_NEAR(out.command.linear.x, 0.8, 1e-12);
}

TEST(Shaping, TargetIsReportedSeparatelyFromTheShapedCommand)
{
  ShaperLimits limits;
  limits.max_linear_accel_mps2 = 0.5;
  MotionShaper shaper(limits);

  const auto out = shaper.step(moveIntent(1.0), 0.1);

  EXPECT_DOUBLE_EQ(out.target.linear.x, 1.0) << "what the intent asked for";
  EXPECT_NEAR(out.command.linear.x, 0.05, 1e-12) << "what shaping allowed this cycle";
}

TEST(Shaping, ResetClearsAccumulatedVelocity)
{
  MotionShaper shaper(instant());
  shaper.step(moveIntent(0.5), kDt);
  ASSERT_DOUBLE_EQ(shaper.current().linear.x, 0.5);

  shaper.reset();
  EXPECT_DOUBLE_EQ(shaper.current().linear.x, 0.0);
}

TEST(Shaping, ZeroDtDoesNotDivideOrStall)
{
  MotionShaper shaper(ShaperLimits{});
  const auto out = shaper.step(moveIntent(0.4), 0.0);
  EXPECT_TRUE(std::isfinite(out.command.linear.x));
}

// =======================================================================================
// Hostile model output
// =======================================================================================

TEST(Hostile, NonFiniteVelocityIsScrubbedNotPropagated)
{
  MotionShaper shaper(instant());
  auto bad = moveIntent(std::numeric_limits<double>::quiet_NaN());
  bad.velocity.linear.z = std::numeric_limits<double>::infinity();

  const auto out = shaper.step(bad, kDt);

  EXPECT_TRUE(std::isfinite(out.command.linear.x));
  EXPECT_TRUE(std::isfinite(out.command.linear.z));
  EXPECT_DOUBLE_EQ(out.command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(out.command.linear.z, 0.0);
}

TEST(Hostile, OutputStaysFiniteAcrossAbsurdInput)
{
  MotionShaper shaper(instant());
  const double hostile[] = {
    0.0, 1e9, -1e9,
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
  };

  for (double v : hostile) {
    for (double d : hostile) {
      auto bad = moveIntent(v);
      bad.duration_sec = d;
      bad.confidence = v;
      const auto out = shaper.step(bad, kDt);
      ASSERT_TRUE(std::isfinite(out.command.linear.x));
      ASSERT_TRUE(std::isfinite(out.command.linear.y));
      ASSERT_TRUE(std::isfinite(out.command.linear.z));
      ASSERT_TRUE(std::isfinite(out.command.angular.z));
    }
  }
}

TEST(Expiry, ModelCannotGrantItselfUnlimitedAuthority)
{
  // A model emitting a huge duration must not be followed indefinitely -- that would
  // disable the expiry path the whole two-rate design depends on.
  auto limits = instant();
  limits.max_intent_age_sec = 1.0;
  MotionShaper shaper(limits);

  auto greedy = moveIntent(0.4, 1000.0);
  greedy.age_sec = 0.5;
  EXPECT_EQ(shaper.step(greedy, kDt).reason, ShaperReason::kActive);

  greedy.age_sec = 1.5;   // past our ceiling, far inside the model's claim
  const auto out = shaper.step(greedy, kDt);
  EXPECT_EQ(out.reason, ShaperReason::kExpired);
  EXPECT_DOUBLE_EQ(out.command.linear.x, 0.0);
}

TEST(Expiry, CapCanBeDisabled)
{
  auto limits = instant();
  limits.max_intent_age_sec = 0.0;   // disabled
  MotionShaper shaper(limits);

  auto long_intent = moveIntent(0.4, 100.0);
  long_intent.age_sec = 50.0;
  EXPECT_EQ(shaper.step(long_intent, kDt).reason, ShaperReason::kActive);
}
