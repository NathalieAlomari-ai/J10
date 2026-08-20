#ifndef J10_MAVLINK__VEHICLE_STATE_NODE_HPP_
#define J10_MAVLINK__VEHICLE_STATE_NODE_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/estimator_status.hpp>
#include <mavros_msgs/msg/optical_flow_rad.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <j10_interfaces/msg/vehicle_state.hpp>

namespace j10_mavlink
{

/// Aggregates the scattered `/mavros/*` topics into one coherent `VehicleState` snapshot.
///
/// This node exists so that boundary rule 2 in `docs/ARCHITECTURE.md` can hold: nothing
/// outside `j10_mavlink` subscribes to MAVROS. Every consumer reads `/j10/vehicle/state`,
/// which makes a PX4 migration or a MAVROS version bump a one-package change.
///
/// Publishing is timer-driven rather than event-driven so that consumers see a fixed 20 Hz
/// heartbeat and can distinguish "the vehicle is unchanged" from "this node died". Each
/// input carries its own freshness deadline; an input that goes stale degrades its
/// corresponding validity flag rather than stalling the whole snapshot.
class VehicleStateNode : public rclcpp::Node
{
public:
  explicit VehicleStateNode(const rclcpp::NodeOptions & options);

private:
  /// A subscribed value plus the time it arrived, so staleness is explicit everywhere.
  template<typename T>
  struct Stamped
  {
    T value;
    rclcpp::Time received;
    bool valid{false};

    void set(const T & v, const rclcpp::Time & now)
    {
      value = v;
      received = now;
      valid = true;
    }

    /// Seconds since arrival, or nullopt if nothing has ever arrived.
    std::optional<double> age(const rclcpp::Time & now) const
    {
      if (!valid) {
        return std::nullopt;
      }
      return (now - received).seconds();
    }
  };

  void onPublishTimer();

  rclcpp::Publisher<j10_interfaces::msg::VehicleState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mav_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rangefinder_sub_;
  rclcpp::Subscription<mavros_msgs::msg::EstimatorStatus>::SharedPtr estimator_sub_;
  rclcpp::Subscription<mavros_msgs::msg::OpticalFlowRad>::SharedPtr flow_sub_;

  mutable std::mutex mutex_;
  Stamped<mavros_msgs::msg::State> mav_state_;
  Stamped<geometry_msgs::msg::PoseStamped> pose_;
  Stamped<geometry_msgs::msg::TwistStamped> velocity_;
  Stamped<sensor_msgs::msg::BatteryState> battery_;
  Stamped<sensor_msgs::msg::Range> rangefinder_;
  Stamped<mavros_msgs::msg::EstimatorStatus> estimator_;
  Stamped<mavros_msgs::msg::OpticalFlowRad> flow_;

  // --- Parameters ---
  double publish_rate_hz_;
  std::string frame_id_;
  double pose_timeout_sec_;
  double rangefinder_timeout_sec_;
  double flow_timeout_sec_;
  double estimator_timeout_sec_;
  double rangefinder_min_range_m_;
  double rangefinder_max_range_m_;
  double flow_quality_threshold_;
  bool require_pos_horiz_abs_;
  /// Kept so the "no ESTIMATOR_STATUS ever arrived" warning can name the topic it is
  /// waiting on — the usual cause is a topic name with no publisher behind it.
  std::string estimator_status_topic_;
};

}  // namespace j10_mavlink

#endif  // J10_MAVLINK__VEHICLE_STATE_NODE_HPP_
