// video_receiver_node -- one GStreamer pipeline, frames onto ROS.
//
// docs/ARCHITECTURE.md section 4:
//
//     Owns one GStreamer pipeline (udpsrc -> rtph264depay -> avdec_h264 -> videoconvert ->
//     appsink). Converts frames to sensor_msgs/Image, stamping the header with capture time
//     reconstructed from the RTP timestamp, not arrival time. Publishes camera info and a
//     per-frame LatencyReport. Declares stream loss after 500 ms without a frame.
//
// The two parts with real logic in them -- RTP clock reconstruction and link health -- live
// in rtp_clock.hpp and stream_health.hpp, which have no GStreamer and no ROS in them and
// carry 40 unit tests between them. What is left here is pipeline setup and message
// marshalling, which is exactly the part that cannot be tested without the hardware.

#include <algorithm>
#include <memory>
#include <string>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/rtp/gstrtpbuffer.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "j10_interfaces/msg/latency_report.hpp"
#include "j10_interfaces/msg/stream_status.hpp"

#include "j10_video/rtp_clock.hpp"
#include "j10_video/video_receiver_factory.hpp"
#include "j10_video/stream_health.hpp"

namespace j10_video
{

// StreamState duplicates StreamStatus.msg's constants so stream_health.hpp can stay free of
// ROS. These make the duplication safe: a drift becomes a compile error, not a wrong number
// on a topic the safety filter reads.
static_assert(
  static_cast<uint8_t>(StreamState::kDisconnected) ==
  j10_interfaces::msg::StreamStatus::STATE_DISCONNECTED, "StreamState/StreamStatus drift");
static_assert(
  static_cast<uint8_t>(StreamState::kStreaming) ==
  j10_interfaces::msg::StreamStatus::STATE_STREAMING, "StreamState/StreamStatus drift");
static_assert(
  static_cast<uint8_t>(StreamState::kDegraded) ==
  j10_interfaces::msg::StreamStatus::STATE_DEGRADED, "StreamState/StreamStatus drift");
static_assert(
  static_cast<uint8_t>(StreamState::kLost) ==
  j10_interfaces::msg::StreamStatus::STATE_LOST, "StreamState/StreamStatus drift");

class VideoReceiverNode : public rclcpp::Node
{
public:
  explicit VideoReceiverNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("video_receiver_node", options)
  {
    port_ = static_cast<int>(declare_parameter("port", 5600));
    frame_id_ = declare_parameter("frame_id", std::string("camera_optical_frame"));
    image_topic_ = declare_parameter("image_topic", std::string("/j10/camera/image_raw"));
    camera_info_topic_ = declare_parameter(
      "camera_info_topic", std::string("/j10/camera/camera_info"));
    status_topic_ = declare_parameter("status_topic", std::string("/j10/video/status"));
    latency_topic_ = declare_parameter(
      "latency_topic", std::string("/j10/telemetry/latency"));

    HealthLimits limits;
    limits.loss_timeout_sec = declare_parameter("loss_timeout_sec", 0.5);
    limits.expected_fps = declare_parameter("expected_fps", 30.0);
    limits.degraded_fps_fraction = declare_parameter("degraded_fps_fraction", 0.6);
    limits.fps_window_sec = declare_parameter("fps_window_sec", 1.0);
    limits.degraded_decode_errors =
      static_cast<uint32_t>(declare_parameter("degraded_decode_errors", 3));
    health_ = std::make_unique<StreamHealth>(limits);

    clock_ = std::make_unique<RtpClock>(
      declare_parameter("rtp_clock_hz", kH264ClockHz),
      declare_parameter("rtp_resync_threshold_sec", 2.0));

    const double status_rate_hz = declare_parameter("status_rate_hz", 5.0);
    // Deliberately faster than the 500 ms loss timeout. Publishing status at, say, 1 Hz
    // would mean a loss could sit undetected downstream for up to a second past the point
    // it was declared here -- the timeout would be met internally and missed in practice.

    // BEST_EFFORT, KEEP_LAST(1) matches the topic contract and what j10_vla subscribes
    // with. A retransmitted stale frame is worse than a dropped one on this path.
    const auto image_qos = rclcpp::QoS(1).best_effort();
    const auto info_qos = rclcpp::QoS(1).reliable().transient_local();
    const auto status_qos = rclcpp::QoS(1).reliable();

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, image_qos);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, info_qos);
    status_pub_ = create_publisher<j10_interfaces::msg::StreamStatus>(
      status_topic_, status_qos);
    latency_pub_ = create_publisher<j10_interfaces::msg::LatencyReport>(
      latency_topic_, rclcpp::QoS(10).best_effort());

    if (!startPipeline()) {
      // Not fatal. The node stays up publishing DISCONNECTED, which is more useful than
      // exiting: an operator sees a node reporting no video rather than a node that is
      // simply absent, and the pipeline can be retried without relaunching the graph.
      RCLCPP_ERROR(
        get_logger(), "GStreamer pipeline failed to start; publishing DISCONNECTED");
    }

    status_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(status_rate_hz, 1.0)),
      [this]() {publishStatus();});

    RCLCPP_INFO(
      get_logger(), "video_receiver_node up: udp/%d -> %s (loss timeout %.0f ms)",
      port_, image_topic_.c_str(), limits.loss_timeout_sec * 1000.0);
  }

  ~VideoReceiverNode() override
  {
    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
    }
  }

private:
  bool startPipeline()
  {
    if (!gst_is_initialized()) {
      gst_init(nullptr, nullptr);
    }

    // caps on udpsrc are required: without them rtph264depay cannot know what it is
    // receiving and the pipeline stalls in PAUSED with no error.
    //
    // sync=false on appsink because this is a live source being consumed as fast as it
    // arrives. Leaving it true makes GStreamer honour presentation timestamps and buffer to
    // "play" the stream smoothly, which adds exactly the latency the whole design is
    // fighting. max-buffers=1 + drop=true keeps that from turning into a backlog.
    const std::string description =
      "udpsrc port=" + std::to_string(port_) + " "
      "caps=application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000 "
      "! rtpjitterbuffer latency=0 drop-on-latency=true "
      "! rtph264depay ! avdec_h264 output-corrupt=false "
      "! videoconvert ! video/x-raw,format=RGB "
      "! appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true";

    GError * error = nullptr;
    pipeline_ = gst_parse_launch(description.c_str(), &error);
    if (pipeline_ == nullptr || error != nullptr) {
      RCLCPP_ERROR(
        get_logger(), "gst_parse_launch failed: %s",
        error != nullptr ? error->message : "unknown");
      if (error != nullptr) {g_error_free(error);}
      return false;
    }

    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    if (appsink_ == nullptr) {
      RCLCPP_ERROR(get_logger(), "could not find appsink in the pipeline");
      return false;
    }
    g_signal_connect(appsink_, "new-sample", G_CALLBACK(&VideoReceiverNode::onNewSample),
      this);

    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) ==
      GST_STATE_CHANGE_FAILURE)
    {
      RCLCPP_ERROR(get_logger(), "could not set the pipeline to PLAYING");
      return false;
    }
    return true;
  }

  /// appsink callback. Runs on a GStreamer streaming thread, not a ROS executor thread.
  static GstFlowReturn onNewSample(GstElement * sink, gpointer user_data)
  {
    auto * self = static_cast<VideoReceiverNode *>(user_data);
    GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (sample == nullptr) {
      return GST_FLOW_ERROR;
    }
    self->handleSample(sample);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  void handleSample(GstSample * sample)
  {
    GstBuffer * buffer = gst_sample_get_buffer(sample);
    GstCaps * caps = gst_sample_get_caps(sample);
    if (buffer == nullptr || caps == nullptr) {
      health_->onDecodeError(nowSec());
      return;
    }

    GstStructure * structure = gst_caps_get_structure(caps, 0);
    int width = 0;
    int height = 0;
    if (!gst_structure_get_int(structure, "width", &width) ||
      !gst_structure_get_int(structure, "height", &height) ||
      width <= 0 || height <= 0)
    {
      health_->onDecodeError(nowSec());
      return;
    }

    GstMapInfo info;
    if (!gst_buffer_map(buffer, &info, GST_MAP_READ)) {
      health_->onDecodeError(nowSec());
      return;
    }

    const double arrival_sec = nowSec();
    const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height) * 3;
    if (info.size < expected) {
      // A short buffer would be read past the end when copied. Count it and move on.
      gst_buffer_unmap(buffer, &info);
      health_->onDecodeError(arrival_sec);
      return;
    }

    // Capture time, reconstructed from the sender's clock. GST_BUFFER_PTS on a live
    // rtpjitterbuffer output is in the pipeline's running time, which is derived from the
    // RTP timestamp -- that is what carries the sender's spacing rather than ours.
    double capture_sec = arrival_sec;
    if (GST_BUFFER_PTS_IS_VALID(buffer)) {
      const auto pts_ns = static_cast<uint64_t>(GST_BUFFER_PTS(buffer));
      // Fold the nanosecond PTS onto the same 90 kHz grid the RtpClock expects, so its
      // wrap and drift handling apply unchanged.
      const auto rtp_equivalent = static_cast<uint32_t>(
        (pts_ns / 1000u) * 90u / 1000u);
      capture_sec = clock_->toCaptureTime(rtp_equivalent, arrival_sec);
    }

    auto image = std::make_unique<sensor_msgs::msg::Image>();
    image->header.stamp = rclcpp::Time(static_cast<int64_t>(capture_sec * 1e9));
    image->header.frame_id = frame_id_;
    image->height = static_cast<uint32_t>(height);
    image->width = static_cast<uint32_t>(width);
    image->encoding = "rgb8";
    image->is_bigendian = 0;
    image->step = static_cast<uint32_t>(width * 3);
    image->data.assign(info.data, info.data + expected);

    gst_buffer_unmap(buffer, &info);

    health_->onFrame(arrival_sec);
    width_ = width;
    height_ = height;

    image_pub_->publish(std::move(image));
    publishCameraInfo(capture_sec, width, height);
    publishLatency(capture_sec, arrival_sec);
  }

  void publishCameraInfo(double capture_sec, int width, int height)
  {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = rclcpp::Time(static_cast<int64_t>(capture_sec * 1e9));
    info.header.frame_id = frame_id_;
    info.width = static_cast<uint32_t>(width);
    info.height = static_cast<uint32_t>(height);
    // Intrinsics are left unset: this node has no calibration and inventing plausible
    // numbers would be worse than publishing none, since a consumer cannot tell a guess
    // from a measurement. Phase 6 supplies a real calibration file.
    camera_info_pub_->publish(info);
  }

  void publishLatency(double capture_sec, double arrival_sec)
  {
    j10_interfaces::msg::LatencyReport report;
    report.header.stamp = rclcpp::Time(static_cast<int64_t>(arrival_sec * 1e9));
    report.stage = "DECODE";
    report.source_stamp = rclcpp::Time(static_cast<int64_t>(capture_sec * 1e9));
    const double elapsed_ms = (arrival_sec - capture_sec) * 1000.0;
    report.stage_latency_ms = static_cast<float>(elapsed_ms);
    // At this stage the two are the same figure: everything from capture to here is
    // upstream of us and indivisible from this node's point of view.
    report.cumulative_latency_ms = static_cast<float>(elapsed_ms);
    report.sequence = sequence_++;
    latency_pub_->publish(report);
  }

  void publishStatus()
  {
    const double now = nowSec();
    j10_interfaces::msg::StreamStatus status;
    status.header.stamp = rclcpp::Time(static_cast<int64_t>(now * 1e9));
    status.header.frame_id = frame_id_;
    status.state = static_cast<uint8_t>(health_->state(now));
    status.fps = static_cast<float>(health_->fps(now));
    status.time_since_last_frame = static_cast<float>(health_->timeSinceLastFrame(now));
    status.frames_received = health_->framesReceived();
    status.frames_dropped = health_->framesDropped();
    status.decode_errors = health_->decodeErrors();
    status.width = static_cast<uint32_t>(std::max(width_, 0));
    status.height = static_cast<uint32_t>(std::max(height_, 0));
    status_pub_->publish(status);

    // Log the edge, not the condition -- a warning every 200 ms is a warning people mute.
    const bool lost = status.state == j10_interfaces::msg::StreamStatus::STATE_LOST;
    if (lost && !was_lost_) {
      RCLCPP_WARN(
        get_logger(), "video stream LOST after %.2fs of silence",
        status.time_since_last_frame);
    } else if (!lost && was_lost_) {
      RCLCPP_INFO(get_logger(), "video stream recovered (%.1f fps)", status.fps);
    }
    was_lost_ = lost;
  }

  double nowSec() const {return now().nanoseconds() * 1e-9;}

  int port_{5600};
  std::string frame_id_, image_topic_, camera_info_topic_, status_topic_, latency_topic_;

  GstElement * pipeline_{nullptr};
  GstElement * appsink_{nullptr};

  std::unique_ptr<StreamHealth> health_;
  std::unique_ptr<RtpClock> clock_;
  int width_{0};
  int height_{0};
  uint64_t sequence_{0};
  bool was_lost_{false};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<j10_interfaces::msg::StreamStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<j10_interfaces::msg::LatencyReport>::SharedPtr latency_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

std::shared_ptr<rclcpp::Node> createVideoReceiverNode(const rclcpp::NodeOptions & options)
{
  return std::make_shared<VideoReceiverNode>(options);
}

}  // namespace j10_video

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(j10_video::VideoReceiverNode)
