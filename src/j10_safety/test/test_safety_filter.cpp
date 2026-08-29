// Unit tests for the safety filter.
//
// No simulator, no MAVROS, no GPU, no ROS graph -- boundary rule 3 in
// docs/ARCHITECTURE.md. If these take more than a second to run, something is wrong.
//
// These tests are the evidence behind the "100% commands validated -- zero raw model
// output to the drone" requirement, so they are written adversarially: every limit is
// probed from both sides, and the failure paths are checked for "decays to zero", never
// "keeps the last command".

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "j10_safety/safety_filter.hpp"

using j10_safety::ArbitrationSource;
using j10_safety::FilterInputs;
using j10_safety::Limits;
using j10_safety::SafetyFilter;
using j10_safety::SafetyState;
using j10_safety::VehicleSnapshot;

namespace
{

/// Limits with the rate limiter effectively disabled, so a test that is probing a clamp
/// sees the clamp's output directly rather than one acceleration step toward it.
Limits permissiveAccel()
{
  Limits l;
  l.max_accel_mps2 = 1e6;
  l.max_decel_mps2 = 1e6;
  l.max_yaw_accel_rps2 = 1e6;
  return l;
}

/// A vehicle that is healthy, centred, at a comfortable altitude, facing ENU +x.
VehicleSnapshot healthyVehicle()
{
  VehicleSnapshot v;
  v.valid = true;
  v.age_sec = 0.01;
  v.x = 0.0;
  v.y = 0.0;
  v.z = 1.0;
  v.yaw = 0.0;
  v.rangefinder_valid = true;
  v.rangefinder_range = 1.0;
  v.ekf_healthy = true;
  v.armed = true;
  // An armed vehicle that is NOT in a mode which acts on offboard setpoints is a fault
  // (see the NOT_GUIDED tests), so "healthy" has to include actually being commandable.
  v.guided = true;
  v.mode = "GUIDED";
  v.battery_fraction = 0.8;
  return v;
}

/// An autonomous command that is fresh, with video up and autonomy enabled.
FilterInputs autonomousInputs(double vx, double vy = 0.0, double vz = 0.0, double wz = 0.0)
{
  FilterInputs in;
  in.autonomous.linear.x = vx;
  in.autonomous.linear.y = vy;
  in.autonomous.linear.z = vz;
  in.autonomous.angular.z = wz;
  in.autonomous_valid = true;
  in.autonomous_age_sec = 0.01;
  in.autonomy_enabled = true;
  in.video_valid = true;
  in.video_age_sec = 0.01;
  in.vehicle = healthyVehicle();
  return in;
}

bool has(const std::vector<std::string> & limits, const std::string & name)
{
  return std::find(limits.begin(), limits.end(), name) != limits.end();
}

constexpr double kDt = 1.0 / 30.0;

}  // namespace

// =======================================================================================
// Pass-through: the filter must not touch a command that is already legal.
// =======================================================================================

TEST(SafetyFilter, PassesLegalCommandUnmodified)
{
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.3);

  const auto out = filter.step(in, kDt);

  EXPECT_EQ(out.state, SafetyState::kNominal);
  EXPECT_EQ(out.source, ArbitrationSource::kAutonomous);
  EXPECT_TRUE(out.active_limits.empty());
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.3);
}

// =======================================================================================
// Arbitration ladder: ESTOP > MANUAL > OVERRIDE > AUTONOMOUS
// =======================================================================================

TEST(Arbitration, ManualWithDeadmanPreemptsAutonomous)
{
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.4);
  in.manual.linear.x = -0.2;
  in.manual_valid = true;
  in.manual_age_sec = 0.01;
  in.deadman_held = true;

  const auto out = filter.step(in, kDt);

  EXPECT_EQ(out.source, ArbitrationSource::kManual);
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, -0.2);
}

TEST(Arbitration, ManualIgnoredWithoutDeadman)
{
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.4);
  in.manual.linear.x = -0.2;
  in.manual_valid = true;
  in.manual_age_sec = 0.01;
  in.deadman_held = false;

  const auto out = filter.step(in, kDt);

  EXPECT_EQ(out.source, ArbitrationSource::kAutonomous);
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.4);
}

TEST(Arbitration, DeadmanHeldButStaleManualBrakesRatherThanFallingBackToAutonomy)
{
  // A human is holding the deadman; their input going stale must NOT silently hand
  // control back to the model.
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.4);
  in.manual_valid = true;
  in.manual_age_sec = 5.0;
  in.deadman_held = true;

  const auto out = filter.step(in, kDt);

  EXPECT_EQ(out.source, ArbitrationSource::kOverride);
  EXPECT_EQ(out.state, SafetyState::kBraking);
  EXPECT_TRUE(has(out.active_limits, "MANUAL_TIMEOUT"));
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.0);
}

// =======================================================================================
// E-stop: absolute, latching, and only an explicit reset clears it.
// =======================================================================================

TEST(Estop, ZeroesCommandImmediately)
{
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.5);
  in.estop_engaged = true;

  const auto out = filter.step(in, kDt);

  EXPECT_EQ(out.state, SafetyState::kEstop);
  EXPECT_EQ(out.source, ArbitrationSource::kEstop);
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.0);
  EXPECT_TRUE(has(out.active_limits, "ESTOP"));
}

TEST(Estop, LatchesAfterSignalClears)
{
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.5);
  in.estop_engaged = true;
  filter.step(in, kDt);

  in.estop_engaged = false;   // operator released the button
  const auto out = filter.step(in, kDt);

  EXPECT_EQ(out.state, SafetyState::kEstop) << "E-stop must latch, not follow the signal";
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.0);
  EXPECT_TRUE(filter.estopLatched());
}

TEST(Estop, ResetClearsLatch)
{
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.5);
  in.estop_engaged = true;
  filter.step(in, kDt);

  filter.reset();
  in.estop_engaged = false;
  const auto out = filter.step(in, kDt);

  EXPECT_FALSE(filter.estopLatched());
  EXPECT_EQ(out.state, SafetyState::kNominal);
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.5);
}

TEST(Estop, OutranksManualDeadman)
{
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.0);
  in.manual.linear.x = 0.5;
  in.manual_valid = true;
  in.manual_age_sec = 0.01;
  in.deadman_held = true;
  in.estop_engaged = true;

  const auto out = filter.step(in, kDt);

  EXPECT_EQ(out.source, ArbitrationSource::kEstop);
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.0);
}

// =======================================================================================
// Watchdogs: every timeout decays to zero, never to the last command.
// =======================================================================================

TEST(Watchdog, StaleAutonomousCommandDecaysToZero)
{
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.5);
  in.autonomous_age_sec = 1.0;

  const auto out = filter.step(in, kDt);

  EXPECT_EQ(out.state, SafetyState::kBraking);
  EXPECT_TRUE(has(out.active_limits, "CMD_TIMEOUT"));
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.0);
}

TEST(Watchdog, NeverRepeatsLastCommandOnInputLoss)
{
  // The regression this whole design exists to prevent.
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.5);
  ASSERT_DOUBLE_EQ(filter.step(in, kDt).commanded.linear.x, 0.5);

  in.autonomous_valid = false;
  for (int i = 0; i < 10; ++i) {
    const auto out = filter.step(in, kDt);
    EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.0) << "cycle " << i;
  }
}

TEST(Watchdog, StaleVehicleStateBrakes)
{
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.5);
  in.vehicle.age_sec = 5.0;

  const auto out = filter.step(in, kDt);

  EXPECT_EQ(out.state, SafetyState::kBraking);
  EXPECT_TRUE(has(out.active_limits, "STATE_TIMEOUT"));
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.0);
}

TEST(Watchdog, MissingVehicleStateBrakes)
{
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.5);
  in.vehicle.valid = false;

  const auto out = filter.step(in, kDt);

  EXPECT_EQ(out.state, SafetyState::kBraking);
  EXPECT_TRUE(has(out.active_limits, "STATE_INVALID"));
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.0);
}

TEST(Watchdog, UnhealthyEkfBrakes)
{
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.5);
  in.vehicle.ekf_healthy = false;

  const auto out = filter.step(in, kDt);

  EXPECT_EQ(out.state, SafetyState::kBraking);
  EXPECT_TRUE(has(out.active_limits, "EKF_UNHEALTHY"));
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.0);
}

TEST(Watchdog, VideoLossRevokesAutonomyButNotManual)
{
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.5);
  in.video_valid = false;

  auto out = filter.step(in, kDt);
  EXPECT_EQ(out.state, SafetyState::kBraking);
  EXPECT_TRUE(has(out.active_limits, "VIDEO_TIMEOUT"));
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.0);

  // A human on the deadman still flies with no video.
  in.manual.linear.x = 0.2;
  in.manual_valid = true;
  in.manual_age_sec = 0.01;
  in.deadman_held = true;

  out = filter.step(in, kDt);
  EXPECT_EQ(out.source, ArbitrationSource::kManual);
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.2);
}

TEST(Watchdog, AutonomyDisabledBrakes)
{
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.5);
  in.autonomy_enabled = false;

  const auto out = filter.step(in, kDt);

  EXPECT_EQ(out.state, SafetyState::kBraking);
  EXPECT_TRUE(has(out.active_limits, "AUTONOMY_DISABLED"));
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.0);
}

// =======================================================================================
// Velocity clamps
// =======================================================================================

TEST(Clamp, HorizontalSpeedScalesAsVectorPreservingHeading)
{
  auto limits = permissiveAccel();
  limits.max_horizontal_mps = 0.5;
  SafetyFilter filter(limits);

  // 3-4-0 has norm 5; must scale to 0.3/0.4 rather than clipping each axis to 0.5.
  auto in = autonomousInputs(3.0, 4.0);
  in.vehicle.yaw = 0.0;
  const auto out = filter.step(in, kDt);

  EXPECT_NEAR(std::hypot(out.commanded.linear.x, out.commanded.linear.y), 0.5, 1e-9);
  EXPECT_NEAR(out.commanded.linear.y / out.commanded.linear.x, 4.0 / 3.0, 1e-9);
  EXPECT_TRUE(has(out.active_limits, "SPEED_H"));
  EXPECT_EQ(out.state, SafetyState::kLimited);
}

TEST(Clamp, VerticalSpeedAndYawRate)
{
  auto limits = permissiveAccel();
  limits.max_vertical_mps = 0.3;
  limits.max_yaw_rate_rps = 0.5;
  SafetyFilter filter(limits);

  auto in = autonomousInputs(0.0, 0.0, 2.0, -3.0);
  const auto out = filter.step(in, kDt);

  EXPECT_DOUBLE_EQ(out.commanded.linear.z, 0.3);
  EXPECT_DOUBLE_EQ(out.commanded.angular.z, -0.5);
  EXPECT_TRUE(has(out.active_limits, "SPEED_V"));
  EXPECT_TRUE(has(out.active_limits, "YAW_RATE"));
}

TEST(Clamp, NonFiniteScrubbedToZero)
{
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(std::numeric_limits<double>::quiet_NaN());
  in.autonomous.linear.z = std::numeric_limits<double>::infinity();

  const auto out = filter.step(in, kDt);

  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(out.commanded.linear.z, 0.0);
  EXPECT_TRUE(has(out.active_limits, "NON_FINITE"));
}

// =======================================================================================
// Altitude envelope
// =======================================================================================

TEST(Altitude, FloorBlocksDescentAtBoundary)
{
  auto limits = permissiveAccel();
  limits.min_altitude_m = 0.5;
  limits.altitude_margin_m = 0.0;   // hard boundary, no scaling band
  limits.proximity_stop_m = 0.0;    // isolate the floor from proximity braking
  limits.proximity_slow_m = 0.0;
  SafetyFilter filter(limits);

  auto in = autonomousInputs(0.0, 0.0, -0.3);
  in.vehicle.rangefinder_range = 0.5;

  const auto out = filter.step(in, kDt);

  EXPECT_DOUBLE_EQ(out.commanded.linear.z, 0.0);
  EXPECT_TRUE(has(out.active_limits, "ALT_FLOOR"));
  EXPECT_EQ(out.state, SafetyState::kBraking);
}

TEST(Altitude, FloorAllowsClimb)
{
  auto limits = permissiveAccel();
  limits.min_altitude_m = 0.5;
  limits.altitude_margin_m = 0.0;
  SafetyFilter filter(limits);

  auto in = autonomousInputs(0.0, 0.0, 0.2);
  in.vehicle.rangefinder_range = 0.5;

  const auto out = filter.step(in, kDt);

  EXPECT_DOUBLE_EQ(out.commanded.linear.z, 0.2) << "floor must not block climbing away";
  EXPECT_TRUE(out.active_limits.empty());
}

TEST(Altitude, CeilingBlocksClimb)
{
  auto limits = permissiveAccel();
  limits.max_altitude_m = 2.0;
  limits.altitude_margin_m = 0.0;
  SafetyFilter filter(limits);

  auto in = autonomousInputs(0.0, 0.0, 0.3);
  in.vehicle.rangefinder_range = 2.0;

  const auto out = filter.step(in, kDt);

  EXPECT_DOUBLE_EQ(out.commanded.linear.z, 0.0);
  EXPECT_TRUE(has(out.active_limits, "ALT_CEILING"));
}

TEST(Altitude, MarginScalesProportionally)
{
  auto limits = permissiveAccel();
  limits.max_altitude_m = 2.0;
  limits.altitude_margin_m = 0.4;
  limits.max_vertical_mps = 1.0;
  SafetyFilter filter(limits);

  // 0.2 m from the ceiling with a 0.4 m margin -> half speed.
  auto in = autonomousInputs(0.0, 0.0, 0.4);
  in.vehicle.rangefinder_range = 1.8;

  const auto out = filter.step(in, kDt);

  EXPECT_NEAR(out.commanded.linear.z, 0.2, 1e-9);
  EXPECT_EQ(out.state, SafetyState::kLimited);
}

TEST(Altitude, PrefersRangefinderOverDriftingEkfZ)
{
  auto limits = permissiveAccel();
  limits.min_altitude_m = 0.5;
  limits.altitude_margin_m = 0.0;
  limits.proximity_stop_m = 0.0;
  limits.proximity_slow_m = 0.0;
  SafetyFilter filter(limits);

  // EKF thinks we are high, the rangefinder says we are at the floor. Trust the direct
  // measurement -- an integrated estimate drifts, and indoors that drift is what puts a
  // vehicle into the ground.
  auto in = autonomousInputs(0.0, 0.0, -0.3);
  in.vehicle.z = 5.0;
  in.vehicle.rangefinder_valid = true;
  in.vehicle.rangefinder_range = 0.5;

  const auto out = filter.step(in, kDt);

  EXPECT_DOUBLE_EQ(out.commanded.linear.z, 0.0);
  EXPECT_TRUE(has(out.active_limits, "ALT_FLOOR"));
}

TEST(Proximity, StopsDescentCloseToGround)
{
  auto limits = permissiveAccel();
  limits.proximity_stop_m = 0.25;
  limits.proximity_slow_m = 0.6;
  limits.min_altitude_m = 0.0;   // isolate proximity from the floor
  SafetyFilter filter(limits);

  auto in = autonomousInputs(0.0, 0.0, -0.3);
  in.vehicle.rangefinder_range = 0.2;

  const auto out = filter.step(in, kDt);

  EXPECT_DOUBLE_EQ(out.commanded.linear.z, 0.0);
  EXPECT_TRUE(has(out.active_limits, "PROXIMITY"));
  EXPECT_EQ(out.state, SafetyState::kBraking);
}

// =======================================================================================
// Geofence -- the command is body-FLU, the fence is ENU, so yaw matters.
// =======================================================================================

TEST(Geofence, BlocksOutwardMotionAtBoundary)
{
  auto limits = permissiveAccel();
  limits.geofence_max_x = 2.0;
  limits.geofence_margin_m = 0.0;
  SafetyFilter filter(limits);

  auto in = autonomousInputs(0.4);   // body +x, yaw 0 -> ENU +x
  in.vehicle.x = 2.0;
  in.vehicle.yaw = 0.0;

  const auto out = filter.step(in, kDt);

  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.0);
  EXPECT_TRUE(has(out.active_limits, "GEOFENCE_X"));
  EXPECT_EQ(out.state, SafetyState::kBraking);
}

TEST(Geofence, AllowsInwardMotionAtBoundary)
{
  auto limits = permissiveAccel();
  limits.geofence_max_x = 2.0;
  limits.geofence_margin_m = 0.0;
  SafetyFilter filter(limits);

  auto in = autonomousInputs(-0.4);   // flying back inside
  in.vehicle.x = 2.0;

  const auto out = filter.step(in, kDt);

  EXPECT_DOUBLE_EQ(out.commanded.linear.x, -0.4) << "must never trap the vehicle outside";
  EXPECT_TRUE(out.active_limits.empty());
}

TEST(Geofence, RespectsYawWhenRotatingBodyCommandToEnu)
{
  auto limits = permissiveAccel();
  limits.geofence_max_x = 2.0;
  limits.geofence_margin_m = 0.0;
  SafetyFilter filter(limits);

  // At the +x fence, but yawed 90 degrees: body +x now points along ENU +y, which is
  // unconstrained. A filter that ignored yaw would wrongly block this.
  auto in = autonomousInputs(0.4);
  in.vehicle.x = 2.0;
  in.vehicle.yaw = M_PI / 2.0;

  const auto out = filter.step(in, kDt);

  EXPECT_NEAR(out.commanded.linear.x, 0.4, 1e-9);
  EXPECT_TRUE(out.active_limits.empty());
}

TEST(Geofence, BlocksCorrectComponentWhenYawed)
{
  auto limits = permissiveAccel();
  limits.geofence_max_x = 2.0;
  limits.geofence_margin_m = 0.0;
  SafetyFilter filter(limits);

  // Yawed 90 degrees at the +x fence, commanding body -y == ENU +x -> must be blocked.
  auto in = autonomousInputs(0.0, -0.4);
  in.vehicle.x = 2.0;
  in.vehicle.yaw = M_PI / 2.0;

  const auto out = filter.step(in, kDt);

  double vx_enu = 0.0;
  double vy_enu = 0.0;
  j10_safety::bodyToEnu(
    out.commanded.linear.x, out.commanded.linear.y, in.vehicle.yaw, vx_enu, vy_enu);
  EXPECT_NEAR(vx_enu, 0.0, 1e-9) << "outward ENU component must be zero";
  EXPECT_TRUE(has(out.active_limits, "GEOFENCE_X"));
}

TEST(Geofence, CanBeDisabled)
{
  auto limits = permissiveAccel();
  limits.geofence_enabled = false;
  SafetyFilter filter(limits);

  auto in = autonomousInputs(0.4);
  in.vehicle.x = 50.0;

  const auto out = filter.step(in, kDt);

  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.4);
}

// =======================================================================================
// Battery failsafe
// =======================================================================================

TEST(Battery, CriticalForcesControlledDescent)
{
  auto limits = permissiveAccel();
  limits.battery_land_fraction = 0.15;
  limits.land_speed_mps = 0.3;
  limits.max_vertical_mps = 1.0;
  limits.min_altitude_m = 0.0;
  limits.proximity_stop_m = 0.0;
  limits.proximity_slow_m = 0.0;
  SafetyFilter filter(limits);

  auto in = autonomousInputs(0.5);
  in.vehicle.battery_fraction = 0.10;

  const auto out = filter.step(in, kDt);

  EXPECT_EQ(out.state, SafetyState::kLanding);
  EXPECT_TRUE(has(out.active_limits, "BATTERY_LAND"));
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.0) << "no horizontal motion while landing";
  EXPECT_NEAR(out.commanded.linear.z, -0.3, 1e-9);
}

TEST(Battery, LandingDescentIsNotBlockedByTheAltitudeFloor)
{
  // A battery landing has to be able to reach the ground.
  auto limits = permissiveAccel();
  limits.battery_land_fraction = 0.15;
  limits.land_speed_mps = 0.3;
  limits.max_vertical_mps = 1.0;
  limits.min_altitude_m = 0.5;
  limits.altitude_margin_m = 0.0;
  limits.proximity_stop_m = 0.0;
  limits.proximity_slow_m = 0.0;
  SafetyFilter filter(limits);

  auto in = autonomousInputs(0.0);
  in.vehicle.battery_fraction = 0.10;
  in.vehicle.rangefinder_range = 0.5;   // sitting on the floor limit

  const auto out = filter.step(in, kDt);

  EXPECT_LT(out.commanded.linear.z, 0.0) << "landing must still descend through the floor";
}

TEST(Battery, WarnLimitsButDoesNotLand)
{
  auto limits = permissiveAccel();
  limits.battery_warn_fraction = 0.25;
  limits.battery_land_fraction = 0.15;
  SafetyFilter filter(limits);

  auto in = autonomousInputs(0.3);
  in.vehicle.battery_fraction = 0.20;

  const auto out = filter.step(in, kDt);

  EXPECT_EQ(out.state, SafetyState::kLimited);
  EXPECT_TRUE(has(out.active_limits, "BATTERY_WARN"));
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.3) << "warning alone must not stop the flight";
}

TEST(Battery, UnknownChargeIsNotTreatedAsEmpty)
{
  // SITL reports no battery at all. That must not trigger a landing.
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.3);
  in.vehicle.battery_fraction = -1.0;

  const auto out = filter.step(in, kDt);

  EXPECT_EQ(out.state, SafetyState::kNominal);
  EXPECT_FALSE(has(out.active_limits, "BATTERY_LAND"));
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.3);
}

// =======================================================================================
// Acceleration limiting
// =======================================================================================

TEST(Accel, RampsUpGraduallyRatherThanStepping)
{
  Limits limits;
  limits.max_accel_mps2 = 0.5;
  limits.max_horizontal_mps = 10.0;
  SafetyFilter filter(limits);

  auto in = autonomousInputs(1.0);

  const auto first = filter.step(in, 0.1);
  EXPECT_NEAR(first.commanded.linear.x, 0.05, 1e-9) << "0.5 m/s^2 over 0.1 s";
  EXPECT_TRUE(has(first.active_limits, "ACCEL"));

  const auto second = filter.step(in, 0.1);
  EXPECT_NEAR(second.commanded.linear.x, 0.10, 1e-9);
}

TEST(Accel, BrakingIsNotRateLimitedIntoASlowRoll)
{
  // Deceleration gets its own, larger budget: rate-limiting a brake would defeat the
  // point of having a brake.
  Limits limits;
  limits.max_accel_mps2 = 0.5;
  limits.max_decel_mps2 = 10.0;
  limits.max_horizontal_mps = 10.0;
  SafetyFilter filter(limits);

  auto in = autonomousInputs(1.0);
  for (int i = 0; i < 100; ++i) {
    filter.step(in, 0.1);
  }
  ASSERT_NEAR(filter.step(in, 0.1).commanded.linear.x, 1.0, 1e-9);

  in.autonomous.linear.x = 0.0;
  const auto out = filter.step(in, 0.1);
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.0) << "1 m/s at 10 m/s^2 over 0.1 s -> 0";
}

TEST(Accel, EstopBypassesTheRateLimiterEntirely)
{
  Limits limits;
  limits.max_accel_mps2 = 0.5;
  limits.max_decel_mps2 = 0.5;   // deliberately slow
  limits.max_horizontal_mps = 10.0;
  SafetyFilter filter(limits);

  auto in = autonomousInputs(1.0);
  for (int i = 0; i < 100; ++i) {
    filter.step(in, 0.1);
  }

  in.estop_engaged = true;
  const auto out = filter.step(in, 0.1);

  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.0) << "E-stop is immediate, always";
}

// =======================================================================================
// Escalation: the reported state is the most severe condition of the cycle.
// =======================================================================================

TEST(Escalation, ReportsMostSevereConditionAndListsEveryActiveLimit)
{
  auto limits = permissiveAccel();
  limits.max_horizontal_mps = 0.5;
  limits.geofence_max_x = 2.0;
  limits.geofence_margin_m = 0.0;
  limits.battery_warn_fraction = 0.25;
  SafetyFilter filter(limits);

  // Over-speed (LIMITED) + at the fence (BRAKING) + low battery (LIMITED).
  auto in = autonomousInputs(5.0);
  in.vehicle.x = 2.0;
  in.vehicle.battery_fraction = 0.20;

  const auto out = filter.step(in, kDt);

  EXPECT_EQ(out.state, SafetyState::kBraking) << "most severe wins";
  EXPECT_TRUE(has(out.active_limits, "SPEED_H"));
  EXPECT_TRUE(has(out.active_limits, "GEOFENCE_X"));
  EXPECT_TRUE(has(out.active_limits, "BATTERY_WARN"));
}

TEST(Escalation, RequestedIsPreservedForPostFlightReview)
{
  auto limits = permissiveAccel();
  limits.max_horizontal_mps = 0.5;
  SafetyFilter filter(limits);

  auto in = autonomousInputs(5.0);
  const auto out = filter.step(in, kDt);

  EXPECT_DOUBLE_EQ(out.requested.linear.x, 5.0) << "what the model asked for";
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.5) << "what it actually got";
}

// =======================================================================================
// The headline property: no input, however hostile, produces an unbounded command.
// =======================================================================================

TEST(Invariant, NoInputEverExceedsTheConfiguredEnvelope)
{
  auto limits = permissiveAccel();
  limits.max_horizontal_mps = 0.5;
  limits.max_vertical_mps = 0.3;
  limits.max_yaw_rate_rps = 0.5;
  SafetyFilter filter(limits);

  const double hostile[] = {
    0.0, 1e3, -1e3, 1e9, -1e9,
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
  };

  for (double vx : hostile) {
    for (double vz : hostile) {
      for (double wz : hostile) {
        auto in = autonomousInputs(vx, 0.0, vz, wz);
        const auto out = filter.step(in, kDt);

        ASSERT_TRUE(std::isfinite(out.commanded.linear.x));
        ASSERT_TRUE(std::isfinite(out.commanded.linear.y));
        ASSERT_TRUE(std::isfinite(out.commanded.linear.z));
        ASSERT_TRUE(std::isfinite(out.commanded.angular.z));

        ASSERT_LE(
          std::hypot(out.commanded.linear.x, out.commanded.linear.y),
          limits.max_horizontal_mps + 1e-9);
        ASSERT_LE(std::abs(out.commanded.linear.z), limits.max_vertical_mps + 1e-9);
        ASSERT_LE(std::abs(out.commanded.angular.z), limits.max_yaw_rate_rps + 1e-9);
      }
    }
  }
}

// =======================================================================================
// Envelope barrier: the boundary check reads MEASURED velocity, not just the command.
//
// The margin rule alone is not a containment guarantee. It scales the *commanded* velocity
// to zero at the boundary, which describes a vehicle that is still moving outward. These
// tests pin the braking-distance form: the command is capped so the room that is left is
// enough to stop in, after subtracting the travel that happens during the loop delay.
// =======================================================================================

TEST(Barrier, NeverAllowsMoreThanTheMarginRule)
{
  // A regression guard, not a behaviour test: whatever the barrier does, it may only ever
  // tighten. If this fails, an envelope tuned against the old rule has silently loosened.
  for (double position = -1.0; position <= 1.0; position += 0.01) {
    for (double measured = -0.6; measured <= 0.6; measured += 0.1) {
      const double margin_rule =
        j10_safety::limitAgainstBounds(position, -1.0, 1.0, 0.4, 0.3);
      const double barrier = j10_safety::limitAgainstBoundsBraking(
        position, -1.0, 1.0, 0.4, 0.3, measured, 1.0, 0.3);
      EXPECT_LE(std::abs(barrier), std::abs(margin_rule) + 1e-12)
        << "position " << position << " measured " << measured;
    }
  }
}

TEST(Barrier, ReservesTheDistanceTravelledDuringTheLoopDelay)
{
  // 0.08 m from the fence while moving outward at 0.3 m/s. The vehicle covers 0.09 m
  // before any new command can take effect, so there is no room left at all.
  const double barrier = j10_safety::limitAgainstBoundsBraking(
    0.92, -1.0, 1.0, 0.4, /*commanded=*/0.3, /*measured=*/0.3,
    /*brake_accel=*/1.0, /*latency=*/0.3);
  EXPECT_DOUBLE_EQ(barrier, 0.0);

  // The margin rule alone would still have allowed outward motion here.
  EXPECT_GT(j10_safety::limitAgainstBounds(0.92, -1.0, 1.0, 0.4, 0.3), 0.0);
}

TEST(Barrier, RefusesACommandItCouldNotStopFrom)
{
  // This is the case the change exists for: a faster envelope. 0.3 m from the fence, 1.5
  // m/s commanded and measured, 1.0 m/s^2 of braking authority.
  const double margin_rule = j10_safety::limitAgainstBounds(0.7, -1.0, 1.0, 0.4, 1.5);
  const double stopping_distance = (margin_rule * margin_rule) / (2.0 * 1.0);
  EXPECT_GT(stopping_distance, 0.3)
    << "the margin rule allows a command that needs more room than exists";

  const double barrier = j10_safety::limitAgainstBoundsBraking(
    0.7, -1.0, 1.0, 0.4, 1.5, 1.5, 1.0, 0.3);
  EXPECT_DOUBLE_EQ(barrier, 0.0);
}

TEST(Barrier, InwardMotionIsNeverPenalised)
{
  // Hard against the fence, moving away from it. Nothing should be taken off this.
  EXPECT_DOUBLE_EQ(
    j10_safety::limitAgainstBoundsBraking(0.95, -1.0, 1.0, 0.4, -0.3, -0.3, 1.0, 0.3),
    -0.3);
}

TEST(Barrier, GeofenceUsesMeasuredVelocityFromTheSnapshot)
{
  Limits l = permissiveAccel();
  l.geofence_min_x = -1.0;
  l.geofence_max_x = 1.0;
  l.geofence_min_y = -1.0;
  l.geofence_max_y = 1.0;
  l.geofence_margin_m = 0.4;
  l.brake_accel_mps2 = 1.0;
  l.control_latency_sec = 0.3;
  l.max_horizontal_mps = 1.0;

  SafetyFilter filter(l);
  auto in = autonomousInputs(0.3);
  in.vehicle.x = 0.92;          // 0.08 m from the +x fence
  in.vehicle.vx = 0.3;          // and still moving at it

  const auto out = filter.step(in, kDt);
  EXPECT_NEAR(out.commanded.linear.x, 0.0, 1e-9);
  EXPECT_TRUE(has(out.active_limits, "GEOFENCE_X"));
}

TEST(Barrier, StationaryVehicleNearTheFenceKeepsTheMarginBehaviour)
{
  // With no measured motion there is nothing to reserve, so the barrier must not be
  // stricter than the margin rule -- otherwise the envelope shrinks for no reason.
  Limits l = permissiveAccel();
  l.geofence_margin_m = 0.4;
  l.brake_accel_mps2 = 1.0;
  l.control_latency_sec = 0.3;
  l.max_horizontal_mps = 1.0;
  l.geofence_min_x = -2.0;
  l.geofence_max_x = 2.0;
  l.geofence_min_y = -2.0;
  l.geofence_max_y = 2.0;

  SafetyFilter filter(l);
  auto in = autonomousInputs(0.3);
  in.vehicle.x = 1.8;           // 0.2 m in, half the margin
  in.vehicle.vx = 0.0;

  const auto out = filter.step(in, kDt);
  EXPECT_NEAR(out.commanded.linear.x, 0.3 * (0.2 / 0.4), 1e-9);
}

TEST(Barrier, RequiredMarginMatchesTheShippedEnvelope)
{
  // 0.3 m/s, 1.0 m/s^2, 300 ms: 0.045 m to brake plus 0.09 m of latency travel.
  EXPECT_NEAR(j10_safety::requiredMargin(0.3, 1.0, 0.3), 0.135, 1e-9);
  // safety.yaml ships 0.4 m, so the shipped envelope is enforceable...
  EXPECT_LT(j10_safety::requiredMargin(0.3, 1.0, 0.3), 0.4);
  // ...and would stop being enforceable if the speed limit were raised to 1.5 m/s, which
  // is exactly what the node's startup check refuses to let happen silently.
  EXPECT_GT(j10_safety::requiredMargin(1.5, 1.0, 0.3), 0.4);
}

// =======================================================================================
// Offboard authority: the filter must notice when the FC stops acting on its commands.
// =======================================================================================

TEST(Guided, ArmedButNotGuidedBrakes)
{
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.3);
  in.vehicle.guided = false;
  in.vehicle.mode = "LAND";     // a failsafe fired, or a pilot took the vehicle back

  const auto out = filter.step(in, kDt);
  EXPECT_TRUE(has(out.active_limits, "NOT_GUIDED"));
  EXPECT_EQ(out.state, SafetyState::kBraking);
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.0);
}

TEST(Guided, DisarmedVehicleIsNotAFault)
{
  // On the ground, not armed, in whatever mode the FC booted into. That is normal.
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.3);
  in.vehicle.armed = false;
  in.vehicle.guided = false;
  in.vehicle.mode = "STABILIZE";

  const auto out = filter.step(in, kDt);
  EXPECT_FALSE(has(out.active_limits, "NOT_GUIDED"));
}

TEST(Guided, ModeStringMismatchIsEnoughOnItsOwn)
{
  // guided true but the mode is not the configured one: MAVROS reports guided for more
  // than one ArduPilot mode, so the name is checked as well as the flag.
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.3);
  in.vehicle.guided = true;
  in.vehicle.mode = "AUTO";

  const auto out = filter.step(in, kDt);
  EXPECT_TRUE(has(out.active_limits, "NOT_GUIDED"));
}

TEST(Guided, CanBeDisabledForAFlightStackWithADifferentModeName)
{
  Limits l = permissiveAccel();
  l.require_guided = false;

  SafetyFilter filter(l);
  auto in = autonomousInputs(0.3);
  in.vehicle.guided = false;
  in.vehicle.mode = "OFFBOARD";

  const auto out = filter.step(in, kDt);
  EXPECT_FALSE(has(out.active_limits, "NOT_GUIDED"));
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.3);
}

// =======================================================================================
// Deadman watchdog: a held button from a dead publisher is not a held button.
// =======================================================================================

TEST(Deadman, StaleHeldDeadmanBrakes)
{
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.3);
  in.manual.linear.x = 0.2;
  in.manual_valid = true;
  in.manual_age_sec = 0.01;
  in.deadman_held = true;
  in.deadman_valid = true;
  in.deadman_age_sec = 5.0;     // j10_teleop died with the button down

  const auto out = filter.step(in, kDt);
  EXPECT_TRUE(has(out.active_limits, "DEADMAN_TIMEOUT"));
  EXPECT_EQ(out.state, SafetyState::kBraking);
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, 0.0);
}

TEST(Deadman, StaleHeldDeadmanDoesNotHandTheVehicleBackToTheModel)
{
  // The failure this guards against: a pilot's node dies, and the model quietly takes over
  // while they still believe they are flying.
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.3);
  in.manual_valid = true;
  in.manual_age_sec = 0.01;
  in.deadman_held = true;
  in.deadman_valid = true;
  in.deadman_age_sec = 5.0;

  const auto out = filter.step(in, kDt);
  EXPECT_NE(out.source, ArbitrationSource::kAutonomous);
}

TEST(Deadman, FreshHeldDeadmanStillWinsArbitration)
{
  SafetyFilter filter(permissiveAccel());
  auto in = autonomousInputs(0.3);
  in.manual.linear.x = -0.2;
  in.manual_valid = true;
  in.manual_age_sec = 0.01;
  in.deadman_held = true;
  in.deadman_valid = true;
  in.deadman_age_sec = 0.01;

  const auto out = filter.step(in, kDt);
  EXPECT_EQ(out.source, ArbitrationSource::kManual);
  EXPECT_DOUBLE_EQ(out.commanded.linear.x, -0.2);
}

// =======================================================================================
// Acceleration limiting: bad timing, and diagonals.
// =======================================================================================

TEST(Accel, NonPositiveDtHoldsInsteadOfPassingTheCommandThrough)
{
  // dt <= 0 means the clock jumped or the executor stalled. Previously this assigned the
  // target outright and reported that nothing bound -- the limiter disabled itself at
  // exactly the moment the timing could not be trusted.
  double current = 0.0;
  EXPECT_TRUE(j10_safety::rateLimit(current, 5.0, 0.5, 1.5, 0.0));
  EXPECT_DOUBLE_EQ(current, 0.0);

  current = 0.0;
  EXPECT_TRUE(j10_safety::rateLimit(current, 5.0, 0.5, 1.5, -1.0));
  EXPECT_DOUBLE_EQ(current, 0.0);
}

TEST(Accel, DiagonalRespectsTheVectorLimitNotThePerAxisOne)
{
  Limits l;
  l.max_accel_mps2 = 0.5;
  l.max_decel_mps2 = 1.5;
  l.max_horizontal_mps = 1.0;

  SafetyFilter filter(l);
  auto in = autonomousInputs(0.3, 0.3);   // 45 degrees

  const auto out = filter.step(in, kDt);
  const double magnitude = std::hypot(out.commanded.linear.x, out.commanded.linear.y);
  // Per-axis limiting would give 0.5*dt on each axis, so sqrt(2) * 0.5 * dt overall.
  EXPECT_NEAR(magnitude, 0.5 * kDt, 1e-9);
  EXPECT_TRUE(has(out.active_limits, "ACCEL"));
}

TEST(Accel, DiagonalStillReachesTheTargetEventually)
{
  Limits l;
  l.max_accel_mps2 = 0.5;
  l.max_decel_mps2 = 1.5;
  l.max_horizontal_mps = 1.0;

  SafetyFilter filter(l);
  auto in = autonomousInputs(0.3, 0.3);

  j10_safety::FilterOutput out;
  for (int i = 0; i < 200; ++i) {
    out = filter.step(in, kDt);
  }
  EXPECT_NEAR(out.commanded.linear.x, 0.3, 1e-6);
  EXPECT_NEAR(out.commanded.linear.y, 0.3, 1e-6);
}
