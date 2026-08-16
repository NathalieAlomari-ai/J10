#include "j10_mavlink/mavlink_bridge_node.hpp"

#include <chrono>
#include <cmath>
#include <thread>
#include <utility>

using namespace std::chrono_literals;

namespace j10_mavlink
{

namespace
{
/// Convert a rate in Hz to a timer period, guarding against a zero/negative parameter.
std::chrono::nanoseconds periodFromRate(double rate_hz, double fallback_hz)
{
  const double hz = (rate_hz > 0.0) ? rate_hz : fallback_hz;
  return std::chrono::nanoseconds(static_cast<int64_t>(1e9 / hz));
}
}  // namespace

MavlinkBridgeNode::MavlinkBridgeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mavlink_bridge_node", options),
  last_cmd_time_(0, 0, RCL_ROS_TIME),
  last_state_time_(0, 0, RCL_ROS_TIME)
{
  // --- Parameters ---
  params_.setpoint_rate_hz = declare_parameter("setpoint_rate_hz", 30.0);
  params_.command_timeout_sec = declare_parameter("command_timeout_sec", 0.3);
  params_.absolute_max_linear_mps = declare_parameter("absolute_max_linear_mps", 2.0);
  params_.absolute_max_yaw_rate_rps = declare_parameter("absolute_max_yaw_rate_rps", 1.5);
  params_.guided_mode = declare_parameter("guided_mode", std::string("GUIDED"));
  params_.takeoff_altitude_m = declare_parameter("takeoff_altitude_m", 1.5);
  params_.takeoff_timeout_sec = declare_parameter("takeoff_timeout_sec", 30.0);
  params_.suppress_setpoints_during_takeoff =
    declare_parameter("suppress_setpoints_during_takeoff", true);
  params_.require_ekf_healthy_to_arm = declare_parameter("require_ekf_healthy_to_arm", true);
  params_.allow_force_arm = declare_parameter("allow_force_arm", false);
  params_.service_timeout_sec = declare_parameter("service_timeout_sec", 5.0);
  params_.state_timeout_sec = declare_parameter("state_timeout_sec", 1.0);

  const auto convention_name = declare_parameter("body_frame_convention", std::string("flu"));
  params_.body_frame_convention = BodyFrameConvention::kFlu;
  if (!parseBodyFrameConvention(convention_name, params_.body_frame_convention)) {
    RCLCPP_ERROR(
      get_logger(),
      "body_frame_convention '%s' is not one of {flu, frd}; falling back to 'flu'",
      convention_name.c_str());
  }

  // --- Callback groups ---
  // The service handlers below block on MAVROS service calls. Servers and clients must sit
  // in different mutually-exclusive groups, and the node must be spun multi-threaded, or
  // the response can never be delivered and every arm request times out.
  timer_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sub_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // --- Topic and service names ---
  //
  // Parameters, because MAVROS's own naming is not uniform. The setpoint_raw plugin
  // declares its subscription as "~/local", which resolves against the UAS node's
  // fully-qualified name -- /mavros/mavros under the conventional launch (node named
  // "mavros" inside namespace "mavros"). Publishing to /mavros/setpoint_raw/local instead
  // fails *silently*: the publisher is created, the topic appears in `ros2 topic list`,
  // and the flight controller simply never receives a setpoint. Verified against
  // `ros2 topic list` on a live connection.
  const auto setpoint_topic =
    declare_parameter("setpoint_topic", std::string("/mavros/mavros/local"));
  const auto cmd_vel_topic =
    declare_parameter("cmd_vel_topic", std::string("/j10/cmd_vel_safe"));
  const auto vehicle_state_topic =
    declare_parameter("vehicle_state_topic", std::string("/j10/vehicle/state"));
  const auto set_mode_service =
    declare_parameter("set_mode_service", std::string("/mavros/set_mode"));
  const auto arming_service =
    declare_parameter("arming_service", std::string("/mavros/cmd/arming"));
  const auto takeoff_service =
    declare_parameter("takeoff_service", std::string("/mavros/cmd/takeoff"));

  // --- Publisher ---
  // RELIABLE because the MAVROS setpoint_raw subscription is reliable; a BEST_EFFORT
  // publisher would be silently incompatible and no setpoint would ever arrive.
  setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
    setpoint_topic, rclcpp::QoS(10).reliable());

  // --- Subscriptions ---
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = sub_group_;

  // Per the topic contract: BEST_EFFORT / KEEP_LAST(1). On a control loop a late command is
  // worse than a dropped one.
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    cmd_vel_topic, rclcpp::QoS(1).best_effort(),
    std::bind(&MavlinkBridgeNode::onCmdVel, this, std::placeholders::_1),
    sub_options);

  vehicle_state_sub_ = create_subscription<j10_interfaces::msg::VehicleState>(
    vehicle_state_topic, rclcpp::QoS(5).reliable(),
    std::bind(&MavlinkBridgeNode::onVehicleState, this, std::placeholders::_1),
    sub_options);

  // --- Services offered ---
  arm_srv_ = create_service<j10_interfaces::srv::ArmDisarm>(
    "/j10/vehicle/arm",
    std::bind(
      &MavlinkBridgeNode::handleArmDisarm, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_group_);

  takeoff_srv_ = create_service<std_srvs::srv::Trigger>(
    "/j10/vehicle/takeoff",
    std::bind(
      &MavlinkBridgeNode::handleTakeoff, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_group_);

  set_guided_srv_ = create_service<std_srvs::srv::Trigger>(
    "/j10/vehicle/set_guided",
    std::bind(
      &MavlinkBridgeNode::handleSetGuided, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_group_);

  // --- MAVROS clients ---
  set_mode_client_ = create_client<mavros_msgs::srv::SetMode>(
    set_mode_service, rmw_qos_profile_services_default, client_group_);
  arming_client_ = create_client<mavros_msgs::srv::CommandBool>(
    arming_service, rmw_qos_profile_services_default, client_group_);
  takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>(
    takeoff_service, rmw_qos_profile_services_default, client_group_);

  // --- Setpoint timer ---
  setpoint_timer_ = create_wall_timer(
    periodFromRate(params_.setpoint_rate_hz, 30.0),
    std::bind(&MavlinkBridgeNode::onSetpointTimer, this),
    timer_group_);

  RCLCPP_INFO(
    get_logger(),
    "mavlink_bridge_node up: %.1f Hz setpoints, %.0f ms command timeout, "
    "body_frame_convention=%s, type_mask=%u (velocity + yaw_rate), coordinate_frame=%u "
    "(FRAME_BODY_NED)",
    params_.setpoint_rate_hz, params_.command_timeout_sec * 1e3,
    toString(params_.body_frame_convention),
    static_cast<unsigned>(kVelocityYawRateTypeMask),
    static_cast<unsigned>(mavros_msgs::msg::PositionTarget::FRAME_BODY_NED));

  RCLCPP_INFO(
    get_logger(), "  %s -> %s | services: %s %s %s",
    cmd_vel_topic.c_str(), setpoint_topic.c_str(), set_mode_service.c_str(),
    arming_service.c_str(), takeoff_service.c_str());

  if (params_.allow_force_arm) {
    RCLCPP_WARN(
      get_logger(),
      "allow_force_arm is TRUE — preflight checks can be bypassed. This is a SITL-only "
      "setting and must be false against a real autopilot.");
  }
}

// ---------------------------------------------------------------------------------------
// Subscriptions
// ---------------------------------------------------------------------------------------

void MavlinkBridgeNode::onCmdVel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  last_cmd_ = msg->twist;
  last_cmd_time_ = now();
  cmd_ever_received_ = true;
}

void MavlinkBridgeNode::onVehicleState(const j10_interfaces::msg::VehicleState::SharedPtr msg)
{
  std::string previous_mode;
  bool was_guided_requested = guided_requested_.load();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    previous_mode = last_seen_mode_;
    last_state_ = *msg;
    last_state_time_ = now();
    state_ever_received_ = true;
    last_seen_mode_ = msg->mode;
  }

  // Mode-loss detection: we asked for GUIDED and the FC has since moved elsewhere. That is
  // either a failsafe or a pilot taking the vehicle back; either way our setpoints are no
  // longer being honoured and the operator needs to know.
  if (was_guided_requested && !previous_mode.empty() && previous_mode != msg->mode &&
    msg->mode != params_.guided_mode)
  {
    RCLCPP_ERROR(
      get_logger(),
      "OFFBOARD CONTROL LOST: flight mode changed %s -> %s. Setpoints are no longer being "
      "acted on.", previous_mode.c_str(), msg->mode.c_str());
    guided_requested_.store(false);
  }
}

// ---------------------------------------------------------------------------------------
// The setpoint stream
// ---------------------------------------------------------------------------------------

void MavlinkBridgeNode::onSetpointTimer()
{
  // ArduPilot runs its own guided takeoff controller. Streaming a velocity setpoint during
  // that window overrides the climb and the vehicle sits on the ground, so hold the stream
  // until takeoff reports complete.
  if (takeoff_in_progress_.load() && params_.suppress_setpoints_during_takeoff) {
    return;
  }

  geometry_msgs::msg::Twist command;
  bool stale = true;

  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (cmd_ever_received_) {
      const double age = (now() - last_cmd_time_).seconds();
      stale = age > params_.command_timeout_sec;
      if (!stale) {
        command = last_cmd_;
      }
    }

    // Log only on the edges, so a disarmed vehicle sitting idle does not spam the console.
    if (stale && !holding_zero_) {
      RCLCPP_WARN(
        get_logger(),
        "/j10/cmd_vel_safe stale (> %.0f ms) — holding zero velocity",
        params_.command_timeout_sec * 1e3);
    } else if (!stale && holding_zero_) {
      RCLCPP_INFO(get_logger(), "/j10/cmd_vel_safe live — following commands");
    }
    holding_zero_ = stale;
  }

  // `command` is default-constructed (all zeros) when stale — decay to hover, never repeat.

  if (sanitize(command)) {
    RCLCPP_ERROR(
      get_logger(), "non-finite component in /j10/cmd_vel_safe — scrubbed to zero");
  }

  if (clampBodyVelocity(
      command, params_.absolute_max_linear_mps, params_.absolute_max_yaw_rate_rps))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "bridge saturation limit bound — the safety filter should have caught this upstream");
  }

  mavros_msgs::msg::PositionTarget target;
  target.header.stamp = now();
  target.header.frame_id = "base_link";
  target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_NED;
  target.type_mask = kVelocityYawRateTypeMask;
  fillBodyVelocitySetpoint(command, params_.body_frame_convention, target);

  setpoint_pub_->publish(target);
  setpoints_published_.fetch_add(1);
}

// ---------------------------------------------------------------------------------------
// Service helpers
// ---------------------------------------------------------------------------------------

template<typename SrvT>
bool MavlinkBridgeNode::callService(
  const typename rclcpp::Client<SrvT>::SharedPtr & client,
  const typename SrvT::Request::SharedPtr & request,
  typename SrvT::Response::SharedPtr & response,
  std::string & error)
{
  const auto timeout = std::chrono::duration<double>(params_.service_timeout_sec);

  if (!client->wait_for_service(1s)) {
    error = std::string("MAVROS service ") + client->get_service_name() +
      " is not available — is mavros_node running and connected?";
    return false;
  }

  auto future = client->async_send_request(request);
  if (future.wait_for(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)) !=
    std::future_status::ready)
  {
    client->remove_pending_request(future);
    error = std::string("timed out waiting for ") + client->get_service_name();
    return false;
  }

  response = future.get();
  return true;
}

bool MavlinkBridgeNode::requestMode(const std::string & mode, std::string & error)
{
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->base_mode = 0;
  request->custom_mode = mode;

  mavros_msgs::srv::SetMode::Response::SharedPtr response;
  if (!callService<mavros_msgs::srv::SetMode>(set_mode_client_, request, response, error)) {
    return false;
  }
  if (!response->mode_sent) {
    error = "flight controller rejected mode " + mode;
    return false;
  }
  return true;
}

bool MavlinkBridgeNode::waitForMode(const std::string & mode, double timeout_sec)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timeout_sec));

  while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    bool fresh = false;
    const auto state = vehicleState(fresh);
    if (fresh && state.mode == mode) {
      return true;
    }
    std::this_thread::sleep_for(50ms);
  }
  return false;
}

bool MavlinkBridgeNode::waitForArmed(bool armed, double timeout_sec)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timeout_sec));

  while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    bool fresh = false;
    const auto state = vehicleState(fresh);
    if (fresh && state.armed == armed) {
      return true;
    }
    std::this_thread::sleep_for(50ms);
  }
  return false;
}

j10_interfaces::msg::VehicleState MavlinkBridgeNode::vehicleState(bool & fresh) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!state_ever_received_) {
    fresh = false;
    return j10_interfaces::msg::VehicleState();
  }
  fresh = (now() - last_state_time_).seconds() <= params_.state_timeout_sec;
  return last_state_;
}

// ---------------------------------------------------------------------------------------
// Services
// ---------------------------------------------------------------------------------------

void MavlinkBridgeNode::handleSetGuided(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::string error;
  if (!requestMode(params_.guided_mode, error)) {
    response->success = false;
    response->message = error;
    RCLCPP_ERROR(get_logger(), "set_guided failed: %s", error.c_str());
    return;
  }

  if (!waitForMode(params_.guided_mode, params_.service_timeout_sec)) {
    response->success = false;
    response->message = "mode command accepted but vehicle never reported " +
      params_.guided_mode;
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  guided_requested_.store(true);
  response->success = true;
  response->message = "vehicle is in " + params_.guided_mode;
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void MavlinkBridgeNode::handleArmDisarm(
  const std::shared_ptr<j10_interfaces::srv::ArmDisarm::Request> request,
  std::shared_ptr<j10_interfaces::srv::ArmDisarm::Response> response)
{
  bool fresh = false;
  auto state = vehicleState(fresh);

  const auto finish = [&](bool success, const std::string & message) {
      bool now_fresh = false;
      response->success = success;
      response->message = message;
      response->armed = vehicleState(now_fresh).armed;
    };

  // --- Disarm is always honoured. ---
  if (!request->arm) {
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = false;

    std::string error;
    mavros_msgs::srv::CommandBool::Response::SharedPtr res;
    if (!callService<mavros_msgs::srv::CommandBool>(arming_client_, req, res, error)) {
      finish(false, error);
      return;
    }
    if (!res->success) {
      finish(false, "flight controller refused disarm (result " + std::to_string(res->result) + ")");
      return;
    }
    waitForArmed(false, params_.service_timeout_sec);
    guided_requested_.store(false);
    finish(true, "disarmed");
    RCLCPP_INFO(get_logger(), "disarmed");
    return;
  }

  // --- Arming preconditions. ---
  const bool force = request->force;
  if (force && !params_.allow_force_arm) {
    finish(
      false,
      "force arming refused: allow_force_arm is false. This is a SITL-only escape hatch and "
      "must stay false against a real autopilot.");
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  if (!fresh) {
    finish(
      false,
      "no fresh /j10/vehicle/state — is vehicle_state_node running and MAVROS connected?");
    return;
  }
  if (!state.connected) {
    finish(false, "no MAVLink heartbeat from the flight controller");
    return;
  }

  // Per the ArmDisarm contract: arming is refused unless the bridge is already streaming
  // setpoints, so the vehicle never arms into a silent command channel.
  if (setpoints_published_.load() == 0) {
    finish(false, "bridge is not streaming setpoints yet");
    return;
  }

  if (params_.require_ekf_healthy_to_arm && !state.ekf_healthy && !force) {
    finish(
      false,
      "EKF is not reporting a usable position/velocity estimate. Indoors this usually means "
      "optical flow or the rangefinder is not fusing yet — wait, or retry with force=true "
      "in SITL.");
    return;
  }

  // --- GUIDED, then arm. ---
  std::string error;
  if (state.mode != params_.guided_mode) {
    if (!requestMode(params_.guided_mode, error)) {
      finish(false, "could not enter " + params_.guided_mode + ": " + error);
      return;
    }
    if (!waitForMode(params_.guided_mode, params_.service_timeout_sec)) {
      finish(false, "vehicle never reported " + params_.guided_mode);
      return;
    }
  }
  guided_requested_.store(true);

  auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  req->value = true;

  mavros_msgs::srv::CommandBool::Response::SharedPtr res;
  if (!callService<mavros_msgs::srv::CommandBool>(arming_client_, req, res, error)) {
    finish(false, error);
    return;
  }
  if (!res->success) {
    finish(
      false,
      "flight controller refused arming (result " + std::to_string(res->result) +
      "). Check the MAVProxy/console prearm message for the reason.");
    return;
  }

  if (!waitForArmed(true, params_.service_timeout_sec)) {
    finish(false, "arming command accepted but the vehicle never reported armed");
    return;
  }

  finish(true, "armed in " + params_.guided_mode);
  RCLCPP_INFO(get_logger(), "armed in %s", params_.guided_mode.c_str());
}

void MavlinkBridgeNode::handleTakeoff(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  bool fresh = false;
  auto state = vehicleState(fresh);

  if (!fresh) {
    response->success = false;
    response->message = "no fresh /j10/vehicle/state";
    return;
  }
  if (!state.armed) {
    response->success = false;
    response->message = "vehicle is not armed";
    return;
  }
  if (state.mode != params_.guided_mode) {
    response->success = false;
    response->message = "vehicle is in " + state.mode + ", not " + params_.guided_mode;
    return;
  }

  auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
  req->min_pitch = 0.0f;
  req->yaw = 0.0f;
  req->latitude = 0.0f;
  req->longitude = 0.0f;
  req->altitude = static_cast<float>(params_.takeoff_altitude_m);

  takeoff_in_progress_.store(true);

  std::string error;
  mavros_msgs::srv::CommandTOL::Response::SharedPtr res;
  if (!callService<mavros_msgs::srv::CommandTOL>(takeoff_client_, req, res, error)) {
    takeoff_in_progress_.store(false);
    response->success = false;
    response->message = error;
    return;
  }
  if (!res->success) {
    takeoff_in_progress_.store(false);
    response->success = false;
    response->message =
      "flight controller refused takeoff (result " + std::to_string(res->result) + ")";
    return;
  }

  RCLCPP_INFO(
    get_logger(), "takeoff to %.2f m commanded; setpoint stream held until climb completes",
    params_.takeoff_altitude_m);

  // Wait for the climb. The setpoint stream stays suppressed for this window so ArduPilot's
  // own takeoff controller is not overridden by a zero-velocity command.
  const double target = params_.takeoff_altitude_m * 0.95;
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(params_.takeoff_timeout_sec));

  bool reached = false;
  while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    bool ok = false;
    const auto s = vehicleState(ok);
    if (ok && s.pose.pose.position.z >= target) {
      reached = true;
      break;
    }
    std::this_thread::sleep_for(100ms);
  }

  // Resume streaming either way — a vehicle in the air with no setpoint stream will trip
  // ArduPilot's guided failsafe, which is a worse outcome than an incomplete climb.
  takeoff_in_progress_.store(false);

  if (!reached) {
    response->success = false;
    response->message =
      "takeoff accepted but the vehicle did not reach " +
      std::to_string(params_.takeoff_altitude_m) + " m within " +
      std::to_string(params_.takeoff_timeout_sec) + " s. Setpoint stream resumed.";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  response->success = true;
  response->message = "at altitude; setpoint stream resumed";
  RCLCPP_INFO(get_logger(), "takeoff complete — %s", response->message.c_str());
}

}  // namespace j10_mavlink

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(j10_mavlink::MavlinkBridgeNode)
