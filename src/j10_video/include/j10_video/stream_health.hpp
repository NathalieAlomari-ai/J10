// Video link health, as a pure state machine.
//
// docs/ARCHITECTURE.md requires stream loss to be declared within 500 ms, and
// j10_interfaces/StreamStatus explains why it matters:
//
//     The safety filter treats loss of video as loss of autonomy: no frames means the VLA
//     is blind, so its intents must stop being trusted.
//
// That makes this small class safety-relevant. The failure to avoid is a health signal that
// reports STREAMING while nothing is arriving -- j10_safety would keep trusting a model that
// has not seen anything for seconds.

#ifndef J10_VIDEO__STREAM_HEALTH_HPP_
#define J10_VIDEO__STREAM_HEALTH_HPP_

#include <cstdint>
#include <deque>

namespace j10_video
{

/// Mirrors j10_interfaces/msg/StreamStatus STATE_* constants.
///
/// Duplicated so this header stays free of ROS and testable on its own; the node
/// static_asserts the two against each other so they cannot drift silently.
enum class StreamState : uint8_t
{
  kDisconnected = 0,  ///< no frame has ever arrived, or the pipeline is down
  kStreaming = 1,     ///< frames arriving within the freshness window
  kDegraded = 2,      ///< frames arriving, but late, dropping, or erroring
  kLost = 3,          ///< was streaming, now silent past the timeout
};

struct HealthLimits
{
  /// Silence longer than this declares loss. 0.5 s is the figure in the architecture doc.
  double loss_timeout_sec{0.5};

  /// Below this fraction of the expected rate the link is degraded rather than healthy.
  /// A stream limping at a third of its rate is not "fine" -- the model is acting on
  /// stale scenery -- but it is not lost either, and conflating the two would either
  /// suppress a real warning or trigger a false autonomy drop.
  double degraded_fps_fraction{0.6};

  double expected_fps{30.0};

  /// Window over which fps is measured. Long enough to be stable, short enough to react.
  double fps_window_sec{1.0};

  /// Decode errors within the window before the link counts as degraded. One corrupt frame
  /// on a wireless link is normal; a steady trickle is not.
  uint32_t degraded_decode_errors{3};
};

/// Tracks arrival timing and error counts, and derives a StreamState.
///
/// Not thread-safe; driven from the single appsink callback plus a timer.
class StreamHealth
{
public:
  explicit StreamHealth(const HealthLimits & limits = HealthLimits())
  : limits_(limits)
  {
  }

  /// Record a successfully decoded frame arriving at *now_sec*.
  void onFrame(double now_sec)
  {
    ++frames_received_;
    last_frame_sec_ = now_sec;
    have_frame_ = true;
    arrivals_.push_back(now_sec);
    trim(now_sec);
  }

  /// Record a frame dropped by the receiver because a newer one was already available.
  ///
  /// Not an error and not counted against health: dropping stale frames is the intended
  /// behaviour of a KEEP_LAST(1) path, and treating it as degradation would flag a
  /// correctly-working pipeline under load.
  void onDropped() {++frames_dropped_;}

  /// Record a frame that failed to decode.
  void onDecodeError(double now_sec)
  {
    ++decode_errors_;
    decode_error_times_.push_back(now_sec);
    trim(now_sec);
  }

  /// Frames per second over the trailing window.
  ///
  /// Computed from the span between the oldest and newest arrival rather than
  /// count / window: early in a stream the window is only partly filled, and dividing by
  /// the nominal window would report a rate far below the true one and spuriously declare
  /// the link degraded in its first second.
  double fps(double now_sec)
  {
    trim(now_sec);
    if (arrivals_.size() < 2) {
      return 0.0;
    }
    const double span = arrivals_.back() - arrivals_.front();
    if (span <= 0.0) {
      return 0.0;
    }
    return static_cast<double>(arrivals_.size() - 1) / span;
  }

  double timeSinceLastFrame(double now_sec) const
  {
    if (!have_frame_) {
      return 0.0;  // "never" is reported through kDisconnected, not as a huge age
    }
    const double age = now_sec - last_frame_sec_;
    return age > 0.0 ? age : 0.0;
  }

  /// The link's current state.
  ///
  /// Order matters: never-connected is checked before silence, so a node that starts before
  /// the sender reports DISCONNECTED rather than LOST. The two mean different things --
  /// LOST says something that was working has stopped, which is the one worth alarming on.
  StreamState state(double now_sec)
  {
    if (!have_frame_) {
      return StreamState::kDisconnected;
    }
    if (timeSinceLastFrame(now_sec) > limits_.loss_timeout_sec) {
      return StreamState::kLost;
    }

    trim(now_sec);
    if (decode_error_times_.size() >= limits_.degraded_decode_errors) {
      return StreamState::kDegraded;
    }

    const double rate = fps(now_sec);
    const double floor_fps = limits_.expected_fps * limits_.degraded_fps_fraction;
    // Only judge the rate once there is enough of a window to judge it from; otherwise the
    // first two frames of a healthy stream would read as degraded.
    if (arrivals_.size() >= 3 && rate < floor_fps) {
      return StreamState::kDegraded;
    }
    return StreamState::kStreaming;
  }

  /// True when the link is healthy enough for a model to be trusted with it.
  ///
  /// Degraded counts as not-usable on purpose. The question this answers is not "are
  /// frames arriving" but "should autonomy be permitted", and a model acting on a third of
  /// the frames it was built around is guessing more than it is seeing.
  bool usableForAutonomy(double now_sec)
  {
    return state(now_sec) == StreamState::kStreaming;
  }

  uint64_t framesReceived() const {return frames_received_;}
  uint64_t framesDropped() const {return frames_dropped_;}
  uint64_t decodeErrors() const {return decode_errors_;}

  /// Forget all history. For a pipeline restart, where old arrivals say nothing about the
  /// new link.
  void reset()
  {
    arrivals_.clear();
    decode_error_times_.clear();
    have_frame_ = false;
    last_frame_sec_ = 0.0;
  }

private:
  void trim(double now_sec)
  {
    const double cutoff = now_sec - limits_.fps_window_sec;
    while (!arrivals_.empty() && arrivals_.front() < cutoff) {
      arrivals_.pop_front();
    }
    while (!decode_error_times_.empty() && decode_error_times_.front() < cutoff) {
      decode_error_times_.pop_front();
    }
  }

  HealthLimits limits_;
  std::deque<double> arrivals_;
  std::deque<double> decode_error_times_;

  bool have_frame_{false};
  double last_frame_sec_{0.0};

  uint64_t frames_received_{0};
  uint64_t frames_dropped_{0};
  uint64_t decode_errors_{0};
};

}  // namespace j10_video

#endif  // J10_VIDEO__STREAM_HEALTH_HPP_
