// Tests for RTP timestamp -> capture time reconstruction.
//
// Getting this wrong does not crash anything; it quietly makes the latency budget
// unmeasurable. Stamping arrival time instead of capture time reports ~0 ms glass-to-ROS
// however bad the link is, so the pipeline would look healthiest exactly when it was
// failing. These tests exist to keep that from creeping back in.

#include <cmath>
#include <cstdint>

#include "gtest/gtest.h"

#include "j10_video/rtp_clock.hpp"

using j10_video::RtpClock;
using j10_video::kH264ClockHz;

namespace
{
/// RTP ticks for a given number of seconds at the H.264 90 kHz clock rate.
uint32_t ticks(double seconds)
{
  return static_cast<uint32_t>(seconds * kH264ClockHz);
}
}  // namespace

// --- Anchoring ---------------------------------------------------------------------------

TEST(RtpClock, StartsUnanchored) {
  EXPECT_FALSE(RtpClock().anchored());
}

TEST(RtpClock, FirstFrameAnchorsAndReturnsArrivalTime) {
  // Nothing to measure a difference against yet, so arrival time is the honest answer.
  RtpClock clock;
  EXPECT_DOUBLE_EQ(clock.toCaptureTime(1000, 100.0), 100.0);
  EXPECT_TRUE(clock.anchored());
}

TEST(RtpClock, SenderSpacingIsPreservedNotArrivalSpacing) {
  // The core behaviour. The sender captured frames 33 ms apart; they arrived 100 ms apart
  // because the link is congested. The stamps must reflect the sender.
  RtpClock clock;
  clock.toCaptureTime(0, 100.0);
  const double capture = clock.toCaptureTime(ticks(0.033), 100.100);
  EXPECT_NEAR(capture, 100.033, 1e-6);
}

TEST(RtpClock, TransportDelayShowsUpAsLatencyRatherThanVanishing) {
  // The failure this class prevents: with arrival-time stamping this difference would be
  // zero, and the measured pipeline latency would be zero no matter how slow the link.
  RtpClock clock;
  clock.toCaptureTime(0, 100.0);
  const double arrival = 100.150;
  const double capture = clock.toCaptureTime(ticks(0.033), arrival);
  EXPECT_GT(arrival - capture, 0.1) << "the 117 ms of extra transport delay must be visible";
}

TEST(RtpClock, ArbitrarySenderOriginDoesNotMatter) {
  // Only differences are meaningful; the sender's clock origin is unrelated to ours.
  RtpClock clock;
  clock.toCaptureTime(3'000'000'000u, 500.0);
  const double capture = clock.toCaptureTime(3'000'000'000u + ticks(0.1), 500.12);
  EXPECT_NEAR(capture, 500.1, 1e-6);
}

TEST(RtpClock, ResetDiscardsTheAnchor) {
  RtpClock clock;
  clock.toCaptureTime(1000, 100.0);
  clock.reset();
  EXPECT_FALSE(clock.anchored());
  EXPECT_DOUBLE_EQ(clock.toCaptureTime(5000, 200.0), 200.0);
}

// --- Steady state ------------------------------------------------------------------------

TEST(RtpClock, ThirtyHzHoldsForASustainedRun) {
  // Accumulated error is what would show up here: a small per-frame mistake compounds over
  // hundreds of frames into stamps that are visibly wrong.
  RtpClock clock;
  clock.toCaptureTime(0, 1000.0);
  for (int i = 1; i <= 300; ++i) {
    const double sender_time = i / 30.0;
    const double capture = clock.toCaptureTime(ticks(sender_time), 1000.0 + sender_time + 0.05);
    EXPECT_NEAR(capture, 1000.0 + sender_time, 1e-4) << "frame " << i;
  }
  EXPECT_EQ(clock.resyncCount(), 0u) << "a steady stream should never need to re-anchor";
}

// --- Wrap --------------------------------------------------------------------------------

TEST(RtpClock, WrapIsNotReadAsAJumpBackwards) {
  // The 32-bit RTP counter wraps about every 13h15m at 90 kHz. Untreated, the frame after a
  // wrap reconstructs as ~13 hours in the past.
  RtpClock clock;
  const uint32_t before = UINT32_MAX - ticks(0.01);  // 10 ms before wrapping
  clock.toCaptureTime(before, 1000.0);

  const uint32_t after = ticks(0.023);  // 33 ms later, having wrapped through zero
  const double capture = clock.toCaptureTime(after, 1000.033 + 0.05);

  EXPECT_GT(capture, 1000.0) << "must move forward through the wrap";
  EXPECT_NEAR(capture, 1000.033, 1e-3);
  EXPECT_EQ(clock.wrapCount(), 1u);
}

TEST(RtpClock, WrapDoesNotTriggerAResync) {
  // If wrap handling were missing, the resulting huge error would be papered over by a
  // re-anchor and the bug would hide as "occasional resyncs" instead of failing loudly.
  RtpClock clock;
  clock.toCaptureTime(UINT32_MAX - ticks(0.01), 1000.0);
  clock.toCaptureTime(ticks(0.023), 1000.083);
  EXPECT_EQ(clock.resyncCount(), 0u);
}

TEST(RtpClock, MultipleWrapsAccumulateCorrectly) {
  RtpClock clock;
  clock.toCaptureTime(UINT32_MAX - ticks(0.01), 1000.0);
  double local = 1000.0;
  for (int wrap = 0; wrap < 3; ++wrap) {
    // Step forward in large increments, crossing the boundary each time.
    for (int i = 0; i < 4; ++i) {
      local += 0.033;
      clock.toCaptureTime(
        static_cast<uint32_t>(UINT32_MAX - ticks(0.01) + ticks(0.033 * (wrap * 4 + i + 1))),
        local + 0.05);
    }
  }
  EXPECT_GE(clock.wrapCount(), 1u);
  EXPECT_EQ(clock.resyncCount(), 0u);
}

TEST(RtpClock, SmallBackwardsStepIsTreatedAsReorderingNotAWrap) {
  // RTP does not guarantee ordering. A frame arriving slightly out of order must not be
  // mistaken for a 13-hour wrap.
  RtpClock clock;
  clock.toCaptureTime(ticks(10.0), 1000.0);
  clock.toCaptureTime(ticks(10.033), 1000.033 + 0.05);
  const double reordered = clock.toCaptureTime(ticks(10.020), 1000.045 + 0.05);

  EXPECT_EQ(clock.wrapCount(), 0u);
  EXPECT_LT(reordered, 1000.033) << "an earlier capture should stamp earlier";
}

// --- Drift and resync -----------------------------------------------------------------------

TEST(RtpClock, LargeDriftForcesAResync) {
  RtpClock clock(kH264ClockHz, 2.0);
  clock.toCaptureTime(0, 1000.0);
  // Sender says 100 ms elapsed; locally 10 s have passed. Something is badly wrong.
  const double capture = clock.toCaptureTime(ticks(0.1), 1010.0);
  EXPECT_DOUBLE_EQ(capture, 1010.0);
  EXPECT_EQ(clock.resyncCount(), 1u);
}

TEST(RtpClock, CaptureTimeInTheFutureForcesAResync) {
  // A frame cannot be captured after it arrived. Left alone this yields a negative latency
  // on every subsequent frame, which would poison the percentile windows downstream --
  // j10_telemetry rejects negative samples, so the stage would silently stop reporting.
  RtpClock clock;
  clock.toCaptureTime(0, 1000.0);
  const double capture = clock.toCaptureTime(ticks(5.0), 1000.1);
  EXPECT_DOUBLE_EQ(capture, 1000.1);
  EXPECT_EQ(clock.resyncCount(), 1u);
}

TEST(RtpClock, NormalTransportDelayDoesNotResync) {
  // The threshold must sit far above real pipeline latency. If ordinary delay triggered a
  // re-anchor, every frame would be stamped with its arrival time -- silently reverting to
  // the exact behaviour this class exists to avoid.
  RtpClock clock(kH264ClockHz, 2.0);
  clock.toCaptureTime(0, 1000.0);
  for (int i = 1; i <= 100; ++i) {
    const double sender = i / 30.0;
    clock.toCaptureTime(ticks(sender), 1000.0 + sender + 0.25);  // a hefty 250 ms
  }
  EXPECT_EQ(clock.resyncCount(), 0u);
}

TEST(RtpClock, ResyncRecoversRatherThanStayingBroken) {
  RtpClock clock;
  clock.toCaptureTime(0, 1000.0);
  clock.toCaptureTime(ticks(5.0), 1000.1);  // forces a resync, re-anchored at 1000.1

  const double capture = clock.toCaptureTime(ticks(5.033), 1000.133 + 0.05);
  EXPECT_NEAR(capture, 1000.133, 1e-3);
}

// --- Construction ----------------------------------------------------------------------------

TEST(RtpClock, NonPositiveClockRateFallsBackToTheH264Default) {
  // A zero rate would divide by zero on the first frame.
  RtpClock clock(0.0);
  clock.toCaptureTime(0, 1000.0);
  const double capture = clock.toCaptureTime(ticks(0.033), 1000.083);
  EXPECT_TRUE(std::isfinite(capture));
  EXPECT_NEAR(capture, 1000.033, 1e-3);
}

TEST(RtpClock, EveryOutputIsFinite) {
  RtpClock clock;
  const uint32_t hostile[] = {0u, 1u, UINT32_MAX, UINT32_MAX / 2, 12345u};
  double local = 1000.0;
  for (uint32_t value : hostile) {
    local += 0.033;
    EXPECT_TRUE(std::isfinite(clock.toCaptureTime(value, local)));
  }
}
