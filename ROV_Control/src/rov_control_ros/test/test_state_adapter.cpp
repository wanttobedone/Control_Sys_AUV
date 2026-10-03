#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include "rov_control_ros/state_adapter.h"

using namespace rov_control_ros;

// 每个测试从同一份合法状态出发，仅修改所测条件，便于判断失败来源。
nav_msgs::msg::Odometry validOdometry()
{
  nav_msgs::msg::Odometry message;
  message.header.frame_id = "map";
  message.child_frame_id = "uuv00/base_link";
  message.header.stamp.sec = 10;
  message.pose.pose.position.x = 1.3;
  message.pose.pose.position.z = -0.5;
  message.pose.pose.orientation.w = std::sqrt(0.5);
  message.pose.pose.orientation.z = std::sqrt(0.5);
  message.twist.twist.linear.x = 2.0;
  message.twist.twist.angular.z = 0.3;
  return message;
}

TEST(StateAdapter, RotatesBodyVelocityButKeepsBodyAngularVelocity)
{
  StateAdapter adapter("map", "uuv00/base_link");
  const auto result = adapter.convert(validOdometry());
  ASSERT_TRUE(result.ok());
  // 绕 z 轴 +90°：机体系 (2,0,0) 应转成世界系 (0,2,0)。
  EXPECT_NEAR(result.value().v_W.x(), 0.0, 1e-12);
  EXPECT_NEAR(result.value().v_W.y(), 2.0, 1e-12);
  EXPECT_DOUBLE_EQ(result.value().omega_B.z(), 0.3);
  EXPECT_DOUBLE_EQ(result.value().p_W.x(), 1.3);
  EXPECT_EQ(result.value().stamp_ns, 10000000000LL);
}

TEST(StateAdapter, RejectsEitherWrongFrame)
{
  for (const bool changeWorld : {true, false})
  {
    StateAdapter adapter("map", "uuv00/base_link");
    auto message = validOdometry();
    if (changeWorld)
    {
      message.header.frame_id = "odom";
    }
    else
    {
      message.child_frame_id = "other/base_link";
    }
    EXPECT_FALSE(adapter.convert(message).ok());
  }
}

TEST(StateAdapter, RejectsInvalidQuaternionAndNonFiniteTwist)
{
  StateAdapter adapter("map", "uuv00/base_link");
  auto message = validOdometry();
  message.pose.pose.orientation.w = 0.0;
  message.pose.pose.orientation.z = 0.0;
  EXPECT_FALSE(adapter.convert(message).ok());
  message = validOdometry();
  message.twist.twist.linear.x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(adapter.convert(message).ok());
  message = validOdometry();
  message.pose.pose.position.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(adapter.convert(message).ok());
  message = validOdometry();
  message.twist.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(adapter.convert(message).ok());
}

TEST(StateAdapter, NormalizesOnlyRoundingError)
{
  StateAdapter adapter("map", "uuv00/base_link");
  auto message = validOdometry();
  message.pose.pose.orientation.z *= 1.0 + 1e-8;
  message.pose.pose.orientation.w *= 1.0 + 1e-8;
  const auto accepted = adapter.convert(message);
  ASSERT_TRUE(accepted.ok());
  EXPECT_NEAR(accepted.value().q_WB.norm(), 1.0, 1e-15);
  message.header.stamp.sec++;
  message.pose.pose.orientation.w *= 1.01;
  EXPECT_FALSE(adapter.convert(message).ok());
}

TEST(StateAdapter, DuplicateAndBackwardStampsRequireReset)
{
  StateAdapter adapter("map", "uuv00/base_link");
  auto message = validOdometry();
  ASSERT_TRUE(adapter.convert(message).ok());
  EXPECT_FALSE(adapter.convert(message).ok());
  message.header.stamp.sec = 9;
  EXPECT_FALSE(adapter.convert(message).ok());
  adapter.reset();
  // 新仿真从零开始允许重新接受，复位不会修改 frame 规则。
  message.header.stamp.sec = 0;
  EXPECT_TRUE(adapter.convert(message).ok());
}

TEST(StateAdapter, RejectedMessageDoesNotPoisonTimeHistory)
{
  StateAdapter adapter("map", "uuv00/base_link");
  auto message = validOdometry();
  ASSERT_TRUE(adapter.convert(message).ok());
  message.header.stamp.sec = 100;
  message.header.frame_id = "wrong";
  EXPECT_FALSE(adapter.convert(message).ok());
  message.header.frame_id = "map";
  message.header.stamp.sec = 11;
  // 拒绝的 t=100 不能使接下来的合法 t=11 被误判为倒退。
  EXPECT_TRUE(adapter.convert(message).ok());
}

TEST(StateAdapter, RejectsMalformedStamp)
{
  StateAdapter adapter("map", "uuv00/base_link");
  auto message = validOdometry();
  message.header.stamp.sec = -1;
  EXPECT_FALSE(adapter.convert(message).ok());
  message.header.stamp.sec = 1;
  message.header.stamp.nanosec = 1000000000U;
  EXPECT_FALSE(adapter.convert(message).ok());
}
