// Unit tests for the setpoint frame/mask helpers.
//
// These run with no ROS graph, no MAVROS and no simulator, and they exist to pin down the
// sign conventions that decide whether the vehicle flies forward or backward in Phase 1.

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "j10_mavlink/frame_conventions.hpp"

using j10_mavlink::BodyFrameConvention;
using j10_mavlink::clampBodyVelocity;
using j10_mavlink::fillBodyVelocitySetpoint;
using j10_mavlink::kVelocityYawRateTypeMask;
using j10_mavlink::parseBodyFrameConvention;
using j10_mavlink::sanitize;

namespace
{
geometry_msgs::msg::Twist makeTwist(double x, double y, double z, double yaw_rate)
{
  geometry_msgs::msg::Twist t;
  t.linear.x = x;
  t.linear.y = y;
  t.linear.z = z;
  t.angular.z = yaw_rate;
  return t;
}
}  // namespace

TEST(TypeMask, SelectsVelocityAndYawRate)
{
  using PT = mavros_msgs::msg::PositionTarget;

  // Ignore position, acceleration and absolute yaw; honour velocity and yaw_rate.
  EXPECT_EQ(kVelocityYawRateTypeMask, 1479u);

  EXPECT_TRUE(kVelocityYawRateTypeMask & PT::IGNORE_PX);
  EXPECT_TRUE(kVelocityYawRateTypeMask & PT::IGNORE_PY);
  EXPECT_TRUE(kVelocityYawRateTypeMask & PT::IGNORE_PZ);
  EXPECT_TRUE(kVelocityYawRateTypeMask & PT::IGNORE_AFX);
  EXPECT_TRUE(kVelocityYawRateTypeMask & PT::IGNORE_AFY);
  EXPECT_TRUE(kVelocityYawRateTypeMask & PT::IGNORE_AFZ);
  EXPECT_TRUE(kVelocityYawRateTypeMask & PT::IGNORE_YAW);

  EXPECT_FALSE(kVelocityYawRateTypeMask & PT::IGNORE_VX);
  EXPECT_FALSE(kVelocityYawRateTypeMask & PT::IGNORE_VY);
  EXPECT_FALSE(kVelocityYawRateTypeMask & PT::IGNORE_VZ);
  EXPECT_FALSE(kVelocityYawRateTypeMask & PT::IGNORE_YAW_RATE);
  EXPECT_FALSE(kVelocityYawRateTypeMask & PT::FORCE);
}

TEST(FrameConvention, FluIsPassThrough)
{
  // With FRAME_BODY_NED, MAVROS itself applies transform_frame_baselink_aircraft to
  // velocity and transform_frame_ned_enu to yaw_rate. Converting here too would
  // double-negate and mirror the command, so FLU must be an exact pass-through.
  mavros_msgs::msg::PositionTarget target;
  fillBodyVelocitySetpoint(makeTwist(1.0, 2.0, 3.0, 0.5), BodyFrameConvention::kFlu, target);

  EXPECT_DOUBLE_EQ(target.velocity.x, 1.0);
  EXPECT_DOUBLE_EQ(target.velocity.y, 2.0);
  EXPECT_DOUBLE_EQ(target.velocity.z, 3.0);
  EXPECT_FLOAT_EQ(target.yaw_rate, 0.5f);
}

TEST(FrameConvention, FrdInvertsYZAndYawRate)
{
  mavros_msgs::msg::PositionTarget target;
  fillBodyVelocitySetpoint(makeTwist(1.0, 2.0, 3.0, 0.5), BodyFrameConvention::kFrd, target);

  EXPECT_DOUBLE_EQ(target.velocity.x, 1.0);    // forward is forward in both
  EXPECT_DOUBLE_EQ(target.velocity.y, -2.0);   // left -> right
  EXPECT_DOUBLE_EQ(target.velocity.z, -3.0);   // up -> down
  EXPECT_FLOAT_EQ(target.yaw_rate, -0.5f);     // CCW -> CW
}

TEST(FrameConvention, IgnoredFieldsAreZeroed)
{
  mavros_msgs::msg::PositionTarget target;
  target.position.x = 42.0;
  target.acceleration_or_force.z = 9.81;
  target.yaw = 1.23f;

  fillBodyVelocitySetpoint(makeTwist(1.0, 0.0, 0.0, 0.0), BodyFrameConvention::kFlu, target);

  EXPECT_DOUBLE_EQ(target.position.x, 0.0);
  EXPECT_DOUBLE_EQ(target.acceleration_or_force.z, 0.0);
  EXPECT_FLOAT_EQ(target.yaw, 0.0f);
}

TEST(FrameConvention, ParsesKnownNamesAndRejectsOthers)
{
  BodyFrameConvention convention = BodyFrameConvention::kFrd;
  ASSERT_TRUE(parseBodyFrameConvention("flu", convention));
  EXPECT_EQ(convention, BodyFrameConvention::kFlu);

  ASSERT_TRUE(parseBodyFrameConvention("frd", convention));
  EXPECT_EQ(convention, BodyFrameConvention::kFrd);

  // An unknown name must leave the caller's value untouched.
  EXPECT_FALSE(parseBodyFrameConvention("ned", convention));
  EXPECT_EQ(convention, BodyFrameConvention::kFrd);
}

TEST(Sanitize, ScrubsNonFiniteToZero)
{
  auto twist = makeTwist(std::numeric_limits<double>::quiet_NaN(), 1.0, 0.0, 0.0);
  twist.linear.z = std::numeric_limits<double>::infinity();

  EXPECT_TRUE(sanitize(twist));
  EXPECT_DOUBLE_EQ(twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(twist.linear.y, 1.0);   // finite components survive
  EXPECT_DOUBLE_EQ(twist.linear.z, 0.0);
}

TEST(Sanitize, LeavesFiniteInputAlone)
{
  auto twist = makeTwist(0.3, -0.2, 0.1, 0.4);
  EXPECT_FALSE(sanitize(twist));
  EXPECT_DOUBLE_EQ(twist.linear.x, 0.3);
}

TEST(Clamp, ScalesLinearAsVectorPreservingDirection)
{
  // 3-4-0 has norm 5; clamping to 1.0 must scale to 0.6/0.8/0 rather than clip each axis.
  auto twist = makeTwist(3.0, 4.0, 0.0, 0.0);
  EXPECT_TRUE(clampBodyVelocity(twist, 1.0, 10.0));

  EXPECT_NEAR(twist.linear.x, 0.6, 1e-9);
  EXPECT_NEAR(twist.linear.y, 0.8, 1e-9);

  const double norm = std::hypot(twist.linear.x, twist.linear.y);
  EXPECT_NEAR(norm, 1.0, 1e-9);

  // Direction preserved: original ratio y/x = 4/3.
  EXPECT_NEAR(twist.linear.y / twist.linear.x, 4.0 / 3.0, 1e-9);
}

TEST(Clamp, LeavesInBudgetCommandUntouched)
{
  auto twist = makeTwist(0.3, 0.4, 0.0, 0.5);
  EXPECT_FALSE(clampBodyVelocity(twist, 2.0, 1.5));
  EXPECT_DOUBLE_EQ(twist.linear.x, 0.3);
  EXPECT_DOUBLE_EQ(twist.angular.z, 0.5);
}

TEST(Clamp, LimitsYawRateKeepingSign)
{
  auto twist = makeTwist(0.0, 0.0, 0.0, -3.0);
  EXPECT_TRUE(clampBodyVelocity(twist, 2.0, 1.5));
  EXPECT_DOUBLE_EQ(twist.angular.z, -1.5);
}

TEST(Clamp, NonPositiveLimitDisablesThatAxis)
{
  auto twist = makeTwist(50.0, 0.0, 0.0, 50.0);
  EXPECT_FALSE(clampBodyVelocity(twist, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(twist.linear.x, 50.0);
  EXPECT_DOUBLE_EQ(twist.angular.z, 50.0);
}
