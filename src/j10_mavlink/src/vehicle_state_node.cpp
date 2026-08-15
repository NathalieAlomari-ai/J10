#include "j10_mavlink/vehicle_state_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace j10_mavlink
{

namespace
{
constexpr double kQuietNaN = std::numeric_limits<double>::quiet_NaN();

std::chrono::nanoseconds periodFromRate(double rate_hz, double fallback_hz)
{
  const double hz = (rate_hz > 0.0) ? rate_hz : fallback_hz;
  return std::chrono::nanoseconds(static_cast<int64_t>(1e9 / hz));
}
}  // namespace

VehicleStateNode::VehicleStateNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("vehicle_state_node", options)
{
  publish_rate_hz_ = declare_parameter("publish_rate_hz", 20.0);
  frame_id_ = declare_parameter("frame_id", std::string("map"));
  pose_timeout_sec_ = declare_parameter("pose_timeout_sec", 0.5);
  rangefinder_timeout_sec_ = declare_parameter("rangefinder_timeout_sec", 0.5);
  flow_timeout_sec_ = declare_parameter("flow_timeout_sec", 0.5);
  estimator_timeout_sec_ = declare_parameter("estimator_timeout_sec", 2.0);
  rangefinder_min_range_m_ = declare_parameter("rangefinder_min_range_m", 0.05);
  rangefinder_max_range_m_ = declare_parameter("rangefinder_max_range_m", 8.0);
  flow_quality_threshold_ = declare_parameter("flow_quality_threshold", 0.2);
  // Indoors there is no GPS, so absolute horizontal position is never valid. Requiring it
  // would make ekf_healthy permanently false and block arming.
  require_pos_horiz_abs_ = declare_parameter("require_pos_horiz_abs", false);

  state_pub_ = create_publisher<j10_interfaces::msg::VehicleState>(
    "/j10/vehicle/state", rclcpp::QoS(5).reliable());

  const auto sensor_qos = rclcpp::SensorDataQoS();
  // MAVROS publishes state/battery/estimator_status with TRANSIENT_LOCAL durability and
  // reliable QoS; a plain reliable subscription is compatible with both.
  const auto status_qos = rclcpp::QoS(10).reliable();

  mav_state_sub_ = create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", status_qos,
    [this](const mavros_msgs::msg::State::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      mav_state_.set(*msg, now());
    });

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", sensor_qos,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      pose_.set(*msg, now());
    });

  // velocity_local is the ENU local-frame twist, which is what the VehicleState contract
  // asks for. velocity_body (FLU) is deliberately not used here.
  velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "/mavros/local_position/velocity_local", sensor_qos,
    [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      velocity_.set(*msg, now());
    });

  battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
    "/mavros/battery", sensor_qos,
    [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      battery_.set(*msg, now());
    });

  rangefinder_sub_ = create_subscription<sensor_msgs::msg::Range>(
    "/mavros/rangefinder/rangefinder", sensor_qos,
    [this](const sensor_msgs::msg::Range::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      rangefinder_.set(*msg, now());
    });

  estimator_sub_ = create_subscription<mavros_msgs::msg::EstimatorStatus>(
    "/mavros/estimator_status", status_qos,
    [this](const mavros_msgs::msg::EstimatorStatus::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      estimator_.set(*msg, now());
    });

  // ArduPilot emits OPTICAL_FLOW (#100); the MAVROS px4flow plugin consumes
  // OPTICAL_FLOW_RAD (#106). Depending on the MAVROS build and plugin set this topic may
  // never be published, in which case flow_valid simply stays false. Phase 1 does not
  // depend on it — the MTF-01 flow path is exercised properly in Phase 6.
  flow_sub_ = create_subscription<mavros_msgs::msg::OpticalFlowRad>(
    "/mavros/px4flow/raw/optical_flow_rad", sensor_qos,
    [this](const mavros_msgs::msg::OpticalFlowRad::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      flow_.set(*msg, now());
    });

  publish_timer_ = create_wall_timer(
    periodFromRate(publish_rate_hz_, 20.0),
    std::bind(&VehicleStateNode::onPublishTimer, this));

  RCLCPP_INFO(
    get_logger(), "vehicle_state_node up: publishing /j10/vehicle/state at %.1f Hz",
    publish_rate_hz_);
}

void VehicleStateNode::onPublishTimer()
{
  const rclcpp::Time stamp = now();
  j10_interfaces::msg::VehicleState msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;

  // Tracks the age of the freshest MAVROS input seen, which is what the contract defines
  // staleness_sec to be. Consumers use this instead of comparing clocks themselves.
  double newest_age = std::numeric_limits<double>::infinity();
  const auto note_age = [&newest_age](const std::optional<double> & age) {
      if (age.has_value()) {
        newest_age = std::min(newest_age, *age);
      }
    };

  std::lock_guard<std::mutex> lock(mutex_);

  // --- Link and arming ---
  if (mav_state_.valid) {
    msg.connected = mav_state_.value.connected;
    msg.armed = mav_state_.value.armed;
    msg.guided = mav_state_.value.guided;
    msg.mode = mav_state_.value.mode;
    note_age(mav_state_.age(stamp));
  } else {
    msg.connected = false;
    msg.armed = false;
    msg.guided = false;
    msg.mode = "";
  }

  // --- Pose and motion (ENU local, EKF origin) ---
  // VehicleState carries no pose_valid flag — staleness_sec is the contract for that — but
  // a stale pose is worth saying out loud, because the bridge's takeoff completion check
  // reads pose.position.z and would otherwise wait silently on a frozen value.
  if (pose_.valid) {
    msg.pose = pose_.value;
    const auto age = pose_.age(stamp);
    note_age(age);
    if (age.has_value() && *age > pose_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "/mavros/local_position/pose is %.2f s old (limit %.2f s) — EKF may have lost its "
        "position estimate", *age, pose_timeout_sec_);
    }
  }
  if (velocity_.valid) {
    msg.velocity = velocity_.value;
    note_age(velocity_.age(stamp));
  }

  // --- Rangefinder ---
  msg.rangefinder_range = static_cast<float>(kQuietNaN);
  msg.rangefinder_valid = false;
  if (rangefinder_.valid) {
    const auto age = rangefinder_.age(stamp);
    note_age(age);
    const double range = rangefinder_.value.range;
    const bool fresh = age.has_value() && *age <= rangefinder_timeout_sec_;
    const bool in_band = std::isfinite(range) &&
      range >= rangefinder_min_range_m_ && range <= rangefinder_max_range_m_;
    if (fresh && in_band) {
      msg.rangefinder_range = static_cast<float>(range);
      msg.rangefinder_valid = true;
    }
  }

  // --- Optical flow ---
  msg.flow_quality = 0.0f;
  msg.flow_valid = false;
  if (flow_.valid) {
    const auto age = flow_.age(stamp);
    note_age(age);
    const double quality = static_cast<double>(flow_.value.quality) / 255.0;
    msg.flow_quality = static_cast<float>(quality);
    msg.flow_valid = age.has_value() && *age <= flow_timeout_sec_ &&
      quality >= flow_quality_threshold_;
  }

  // --- EKF health ---
  // Indoors the vehicle flies on optical flow plus a rangefinder, so absolute horizontal
  // position is never available. Health means: attitude solved, horizontal velocity solved,
  // relative horizontal position solved, and absolute vertical position solved.
  msg.ekf_healthy = false;
  if (estimator_.valid) {
    const auto age = estimator_.age(stamp);
    note_age(age);
    const auto & e = estimator_.value;
    const bool fresh = age.has_value() && *age <= estimator_timeout_sec_;
    bool healthy = e.attitude_status_flag &&
      e.velocity_horiz_status_flag &&
      e.pos_horiz_rel_status_flag &&
      e.pos_vert_abs_status_flag &&
      !e.accel_error_status_flag;
    if (require_pos_horiz_abs_) {
      healthy = healthy && e.pos_horiz_abs_status_flag;
    }
    msg.ekf_healthy = fresh && healthy;
  }

  // --- Power ---
  msg.battery_voltage = 0.0f;
  msg.battery_percentage = -1.0f;
  if (battery_.valid) {
    note_age(battery_.age(stamp));
    const auto & b = battery_.value;
    if (std::isfinite(b.voltage)) {
      msg.battery_voltage = b.voltage;
    }
    // sensor_msgs/BatteryState uses NaN for "unknown"; the contract uses -1.0.
    msg.battery_percentage = std::isfinite(b.percentage) ? b.percentage : -1.0f;
  }

  msg.staleness_sec = std::isfinite(newest_age) ?
    static_cast<float>(newest_age) :
    std::numeric_limits<float>::infinity();

  state_pub_->publish(msg);
}

}  // namespace j10_mavlink

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(j10_mavlink::VehicleStateNode)
