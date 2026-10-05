// Tests for video link health.
//
// j10_interfaces/StreamStatus states the stakes: "The safety filter treats loss of video as
// loss of autonomy: no frames means the VLA is blind, so its intents must stop being
// trusted." The failure to avoid is therefore asymmetric -- reporting STREAMING while
// nothing arrives leaves j10_safety trusting a blind model, which is far worse than an
// occasional false DEGRADED.

#include "gtest/gtest.h"

#include "j10_video/stream_health.hpp"

using j10_video::HealthLimits;
using j10_video::StreamHealth;
using j10_video::StreamState;

namespace
{

/// Feed *count* frames at *fps* starting at *start*, returning the last arrival time.
double feed(StreamHealth & health, double start, int count, double fps)
{
  double t = start;
  for (int i = 0; i < count; ++i) {
    health.onFrame(t);
    t += 1.0 / fps;
  }
  return t - 1.0 / fps;
}

}  // namespace

// --- Initial state -------------------------------------------------------------------------

TEST(StreamHealth, StartsDisconnectedNotLost) {
  // The distinction matters: LOST means something that was working has stopped, which is
  // the case worth alarming on. A node that starts before the sender is merely waiting.
  StreamHealth health;
  EXPECT_EQ(health.state(100.0), StreamState::kDisconnected);
}

TEST(StreamHealth, DisconnectedIsNotUsableForAutonomy) {
  StreamHealth health;
  EXPECT_FALSE(health.usableForAutonomy(100.0));
}

TEST(StreamHealth, NeverConnectedReportsZeroAgeRatherThanAHugeNumber) {
  // "Never" is communicated by the state, not by an enormous age that downstream code
  // would have to special-case.
  StreamHealth health;
  EXPECT_DOUBLE_EQ(health.timeSinceLastFrame(1e6), 0.0);
}

// --- Healthy streaming ----------------------------------------------------------------------

TEST(StreamHealth, SteadyThirtyHzIsStreaming) {
  StreamHealth health;
  const double last = feed(health, 100.0, 30, 30.0);
  EXPECT_EQ(health.state(last), StreamState::kStreaming);
  EXPECT_TRUE(health.usableForAutonomy(last));
}

TEST(StreamHealth, FpsIsMeasuredFromTheArrivalSpanNotACountDividedByTheWindow) {
  // Early in a stream the window is only partly filled. Dividing the count by the nominal
  // window would report a rate far below the true one and declare a healthy link degraded
  // in its first second.
  StreamHealth health;
  const double last = feed(health, 100.0, 5, 30.0);  // only ~130 ms of a 1 s window
  EXPECT_NEAR(health.fps(last), 30.0, 1.0);
}

TEST(StreamHealth, TheFirstFrameDoesNotYieldAnFpsFigure) {
  StreamHealth health;
  health.onFrame(100.0);
  EXPECT_DOUBLE_EQ(health.fps(100.0), 0.0);
}

TEST(StreamHealth, TheFirstFramesAreNotJudgedAsDegraded) {
  // With too few samples to measure a rate from, the honest answer is "streaming", not a
  // degradation warning derived from insufficient evidence.
  StreamHealth health;
  health.onFrame(100.0);
  EXPECT_EQ(health.state(100.0), StreamState::kStreaming);
  health.onFrame(100.033);
  EXPECT_EQ(health.state(100.033), StreamState::kStreaming);
}

// --- Loss ------------------------------------------------------------------------------------

TEST(StreamHealth, SilencePastTheTimeoutIsLost) {
  StreamHealth health;
  const double last = feed(health, 100.0, 30, 30.0);
  EXPECT_EQ(health.state(last + 0.6), StreamState::kLost);
}

TEST(StreamHealth, LossIsDetectedWithinTheDocumentedFiveHundredMilliseconds) {
  // The architecture doc's Phase 3 exit criterion, asserted directly.
  HealthLimits limits;
  limits.loss_timeout_sec = 0.5;
  StreamHealth health(limits);
  const double last = feed(health, 100.0, 30, 30.0);

  EXPECT_NE(health.state(last + 0.49), StreamState::kLost) << "must not fire early";
  EXPECT_EQ(health.state(last + 0.51), StreamState::kLost) << "must fire within 500 ms";
}

TEST(StreamHealth, LostIsNotUsableForAutonomy) {
  StreamHealth health;
  const double last = feed(health, 100.0, 30, 30.0);
  EXPECT_FALSE(health.usableForAutonomy(last + 1.0));
}

TEST(StreamHealth, RecoveryReturnsToStreaming) {
  StreamHealth health;
  double last = feed(health, 100.0, 30, 30.0);
  EXPECT_EQ(health.state(last + 1.0), StreamState::kLost);

  last = feed(health, last + 2.0, 30, 30.0);
  EXPECT_EQ(health.state(last), StreamState::kStreaming);
}

TEST(StreamHealth, AgeIsNeverNegativeWhenTheClockStepsBackwards) {
  StreamHealth health;
  health.onFrame(100.0);
  EXPECT_GE(health.timeSinceLastFrame(99.0), 0.0);
}

// --- Degradation --------------------------------------------------------------------------------

TEST(StreamHealth, AHalvedFrameRateIsDegraded) {
  HealthLimits limits;
  limits.expected_fps = 30.0;
  limits.degraded_fps_fraction = 0.6;  // floor at 18 fps
  StreamHealth health(limits);

  const double last = feed(health, 100.0, 10, 12.0);
  EXPECT_EQ(health.state(last), StreamState::kDegraded);
}

TEST(StreamHealth, DegradedIsNotUsableForAutonomy) {
  // Deliberate: the question is not "are frames arriving" but "should autonomy be
  // permitted", and a model seeing a third of its expected frames is guessing.
  HealthLimits limits;
  limits.expected_fps = 30.0;
  StreamHealth health(limits);
  const double last = feed(health, 100.0, 10, 10.0);

  EXPECT_EQ(health.state(last), StreamState::kDegraded);
  EXPECT_FALSE(health.usableForAutonomy(last));
}

TEST(StreamHealth, ARateJustAboveTheFloorIsStillStreaming) {
  HealthLimits limits;
  limits.expected_fps = 30.0;
  limits.degraded_fps_fraction = 0.6;  // floor at 18 fps
  StreamHealth health(limits);

  const double last = feed(health, 100.0, 25, 25.0);
  EXPECT_EQ(health.state(last), StreamState::kStreaming);
}

TEST(StreamHealth, RepeatedDecodeErrorsDegradeTheLink) {
  StreamHealth health;
  double last = feed(health, 100.0, 30, 30.0);
  for (int i = 0; i < 3; ++i) {
    health.onDecodeError(last + 0.001 * i);
  }
  EXPECT_EQ(health.state(last + 0.01), StreamState::kDegraded);
}

TEST(StreamHealth, AnIsolatedDecodeErrorDoesNotDegradeTheLink) {
  // One corrupt frame on a wireless link is normal. Flagging it would train the operator to
  // ignore the warning.
  StreamHealth health;
  const double last = feed(health, 100.0, 30, 30.0);
  health.onDecodeError(last);
  EXPECT_EQ(health.state(last), StreamState::kStreaming);
}

TEST(StreamHealth, OldDecodeErrorsAgeOutOfTheWindow) {
  StreamHealth health;
  double last = feed(health, 100.0, 30, 30.0);
  for (int i = 0; i < 5; ++i) {
    health.onDecodeError(last);
  }
  EXPECT_EQ(health.state(last), StreamState::kDegraded);

  last = feed(health, last + 2.0, 30, 30.0);
  EXPECT_EQ(health.state(last), StreamState::kStreaming) << "errors should not be permanent";
}

TEST(StreamHealth, LossOutranksDegradation) {
  // A link that is both slow and silent is lost, not degraded -- the more severe reading
  // is the correct one.
  StreamHealth health;
  const double last = feed(health, 100.0, 10, 5.0);
  EXPECT_EQ(health.state(last + 1.0), StreamState::kLost);
}

// --- Counters ---------------------------------------------------------------------------------

TEST(StreamHealth, CountersTrackEachCategorySeparately) {
  StreamHealth health;
  feed(health, 100.0, 10, 30.0);
  health.onDropped();
  health.onDropped();
  health.onDecodeError(100.4);

  EXPECT_EQ(health.framesReceived(), 10u);
  EXPECT_EQ(health.framesDropped(), 2u);
  EXPECT_EQ(health.decodeErrors(), 1u);
}

TEST(StreamHealth, DroppedFramesDoNotCountAgainstHealth) {
  // Dropping stale frames is the intended behaviour of a KEEP_LAST(1) path. Treating it as
  // degradation would flag a correctly-working pipeline under load.
  StreamHealth health;
  const double last = feed(health, 100.0, 30, 30.0);
  for (int i = 0; i < 100; ++i) {
    health.onDropped();
  }
  EXPECT_EQ(health.state(last), StreamState::kStreaming);
}

TEST(StreamHealth, ResetForgetsHistoryButNotLifetimeCounters) {
  // Lifetime totals describe the run; arrival history describes the link. A pipeline
  // restart invalidates the second, not the first.
  StreamHealth health;
  feed(health, 100.0, 30, 30.0);
  health.reset();

  EXPECT_EQ(health.state(200.0), StreamState::kDisconnected);
  EXPECT_EQ(health.framesReceived(), 30u);
}

// --- Enum contract -------------------------------------------------------------------------------

TEST(StreamHealth, StateValuesMatchTheMessageConstants) {
  // StreamState duplicates StreamStatus.msg's STATE_* so this header stays ROS-free. The
  // node static_asserts the same pairing; this catches a drift even without ROS present.
  EXPECT_EQ(static_cast<uint8_t>(StreamState::kDisconnected), 0);
  EXPECT_EQ(static_cast<uint8_t>(StreamState::kStreaming), 1);
  EXPECT_EQ(static_cast<uint8_t>(StreamState::kDegraded), 2);
  EXPECT_EQ(static_cast<uint8_t>(StreamState::kLost), 3);
}
