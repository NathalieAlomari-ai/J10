// Reconstructing frame capture time from an RTP timestamp.
//
// docs/ARCHITECTURE.md is specific about this, and it is the whole reason the file exists:
//
//     Converts frames to sensor_msgs/Image, stamping the header with **capture time
//     reconstructed from the RTP timestamp**, not arrival time.
//
// Arrival time is the easy thing to stamp and it is wrong. It folds every millisecond of
// encode, WiFi transport and decode into "now", which means the measured glass-to-ROS
// latency comes out as approximately zero no matter how bad the link is -- the pipeline
// would report a healthy budget precisely when it stopped meeting it. Section 6's whole
// premise is measuring that path, so the stamp has to come from the sender's clock.
//
// Three things make this less trivial than it sounds, and each is tested:
//
//   * The H.264 RTP clock runs at 90 kHz and is a 32-bit unsigned value, so it wraps
//     roughly every 13 hours 15 minutes. A wrap must not read as a 13-hour jump backwards.
//   * The sender's clock has an arbitrary origin. Only *differences* are meaningful, so the
//     first frame establishes an anchor and everything after is measured from it.
//   * The two clocks drift. Left uncorrected the reconstructed stamps slide away from local
//     time indefinitely, so the anchor is re-established when the error grows too large.

#ifndef J10_VIDEO__RTP_CLOCK_HPP_
#define J10_VIDEO__RTP_CLOCK_HPP_

#include <cmath>
#include <cstdint>
#include <optional>

namespace j10_video
{

/// Standard RTP clock rate for H.264 payloads (RFC 6184).
constexpr double kH264ClockHz = 90000.0;

/// One past the largest RTP timestamp; the value the 32-bit counter wraps at.
constexpr double kRtpWrapPeriod = 4294967296.0;  // 2^32

/// Converts 32-bit RTP timestamps into local wall-clock capture times.
///
/// Not thread-safe: intended to be driven from the single GStreamer appsink callback.
class RtpClock
{
public:
  /// \param clock_hz RTP clock rate. 90 kHz for H.264.
  /// \param resync_threshold_sec Re-anchor when the reconstructed time drifts this far from
  ///   arrival time. Generous by default: it must be far larger than any plausible true
  ///   pipeline latency, or normal transport delay would be mistaken for drift and
  ///   continuously re-anchored away -- which would silently turn this back into an
  ///   arrival-time stamp, the exact failure the class exists to avoid.
  explicit RtpClock(double clock_hz = kH264ClockHz, double resync_threshold_sec = 2.0)
  : clock_hz_(clock_hz > 0.0 ? clock_hz : kH264ClockHz),
    resync_threshold_sec_(std::abs(resync_threshold_sec))
  {
  }

  /// Feed one frame's RTP timestamp and its local arrival time; get the capture time.
  ///
  /// The first frame defines the anchor and necessarily returns *arrival_sec* -- there is
  /// nothing yet to measure a difference against. Every frame after is
  /// `anchor_local + (rtp - anchor_rtp) / clock_hz`, which carries the sender's spacing
  /// rather than ours.
  double toCaptureTime(uint32_t rtp_timestamp, double arrival_sec)
  {
    if (!anchored_) {
      anchor(rtp_timestamp, arrival_sec);
      return arrival_sec;
    }

    const double elapsed = elapsedSeconds(rtp_timestamp);
    const double capture = anchor_local_sec_ + elapsed;

    // A reconstructed capture time in the future means the clocks disagree in the one
    // direction that cannot be real -- a frame cannot be captured after it arrived. Rather
    // than emit a negative latency for every subsequent frame, treat it as a lost anchor.
    // The same branch catches large drift in either direction.
    const double error = arrival_sec - capture;
    if (error < 0.0 || error > resync_threshold_sec_) {
      ++resync_count_;
      anchor(rtp_timestamp, arrival_sec);
      return arrival_sec;
    }

    last_rtp_ = rtp_timestamp;
    return capture;
  }

  /// Seconds from the anchor to *rtp_timestamp*, accounting for 32-bit wrap.
  ///
  /// Wrap detection uses the *previous* timestamp rather than the anchor, because
  /// consecutive frames are milliseconds apart while the anchor may be hours behind. A
  /// backwards step of more than half the counter range is read as a wrap; anything smaller
  /// is read as reordering and left alone, since RTP does not guarantee ordering and a
  /// genuinely reordered frame must not be mistaken for a 13-hour jump.
  double elapsedSeconds(uint32_t rtp_timestamp)
  {
    if (!anchored_) {
      return 0.0;
    }
    const double previous = static_cast<double>(last_rtp_);
    const double current = static_cast<double>(rtp_timestamp);
    double delta = current - previous;
    if (delta < -kRtpWrapPeriod / 2.0) {
      ++wrap_count_;
      delta += kRtpWrapPeriod;
    } else if (delta > kRtpWrapPeriod / 2.0) {
      // Backwards across a wrap boundary: a late frame from just before the wrap.
      delta -= kRtpWrapPeriod;
    }
    accumulated_ticks_ += delta;
    return accumulated_ticks_ / clock_hz_;
  }

  /// Discard the anchor. The next frame establishes a new one.
  void reset()
  {
    anchored_ = false;
    accumulated_ticks_ = 0.0;
  }

  bool anchored() const {return anchored_;}
  uint64_t wrapCount() const {return wrap_count_;}
  uint64_t resyncCount() const {return resync_count_;}

private:
  void anchor(uint32_t rtp_timestamp, double local_sec)
  {
    anchor_rtp_ = rtp_timestamp;
    last_rtp_ = rtp_timestamp;
    anchor_local_sec_ = local_sec;
    accumulated_ticks_ = 0.0;
    anchored_ = true;
  }

  double clock_hz_;
  double resync_threshold_sec_;

  bool anchored_{false};
  uint32_t anchor_rtp_{0};
  uint32_t last_rtp_{0};
  double anchor_local_sec_{0.0};
  /// Signed tick count since the anchor, wrap-corrected. Double rather than integer because
  /// it is fed straight into a division and may legitimately go slightly negative on a
  /// reordered frame.
  double accumulated_ticks_{0.0};

  uint64_t wrap_count_{0};
  uint64_t resync_count_{0};
};

}  // namespace j10_video

#endif  // J10_VIDEO__RTP_CLOCK_HPP_
