#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include "rov_control_ros/trajectory_preview_node.h"

using namespace rov_control_ros;
using namespace rov_control_core;

PreviewConfig exampleConfig()
{
  PreviewConfig config;
  config.bounds.min_W = Eigen::Vector3d(0.8, 0.5, -1.0);
  config.bounds.max_W = Eigen::Vector3d(1.8, 1.7, 0.0);
  config.limits.v_max_mps = 0.08;
  config.limits.a_max_mps2 = 0.1;
  config.limits.min_duration_s = 0.1;
  return config;
}

RobotState exampleState()
{
  RobotState state;
  state.p_W = Eigen::Vector3d(1.3, 1.0, -0.5);
  state.q_WB = Eigen::AngleAxisd(std::acos(-1.0) / 2.0, Eigen::Vector3d::UnitZ());
  return state;
}

TEST(PreviewMath, UsesCurrentPositionAndHorizontalHeading)
{
  const auto result = createTrajectory(exampleState(), exampleConfig());
  ASSERT_TRUE(result.ok());
  // 当前 yaw=90°，终点=(1.3,1.2,-0.5)，不是从原点或沿世界 x 开始。
  const auto &path = result.value().path();
  EXPECT_NEAR((path.startPosition() - exampleState().p_W).norm(), 0.0, 1e-12);
  EXPECT_NEAR(path.goalPosition().x(), 1.3, 1e-12);
  EXPECT_NEAR(path.goalPosition().y(), 1.2, 1e-12);
  EXPECT_NEAR(path.goalPosition().z(), -0.5, 1e-12);
  EXPECT_NEAR(path.length(), 0.2, 1e-12);
  // 1.875*0.2/0.08 = 4.6875，是此例起主导作用的速度限制。
  EXPECT_NEAR(result.value().duration(), 4.6875, 1e-12);
}

TEST(PreviewMath, DriftingStateIsAllowedForPreviewOnly)
{
  auto state = exampleState();
  state.v_W = Eigen::Vector3d(0.0, 0.0, 0.2);
  EXPECT_TRUE(createTrajectory(state, exampleConfig()).ok());
}

TEST(PreviewMath, RejectsVerticalHeadingAndInvalidQuaternion)
{
  auto state = exampleState();
  state.q_WB = Eigen::AngleAxisd(std::acos(-1.0) / 2.0, Eigen::Vector3d::UnitY());
  EXPECT_FALSE(createTrajectory(state, exampleConfig()).ok());
  state.q_WB.coeffs().setZero();
  EXPECT_FALSE(createTrajectory(state, exampleConfig()).ok());
}

TEST(PreviewMath, RejectsEndpointOutsideWorkspace)
{
  auto state = exampleState();
  state.p_W.y() = 1.6;
  // 起点仍在盒内，但终点 y=1.8 超出上限 1.7。
  EXPECT_FALSE(createTrajectory(state, exampleConfig()).ok());
}

TEST(PreviewMath, RejectsInvalidLength)
{
  auto config = exampleConfig();
  config.length_m = -0.2;
  EXPECT_FALSE(createTrajectory(exampleState(), config).ok());
  config.length_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(createTrajectory(exampleState(), config).ok());
}

TEST(PreviewMath, PathContainsConsistentEndpointsTimesAndFrames)
{
  const auto trajectory = createTrajectory(exampleState(), exampleConfig());
  ASSERT_TRUE(trajectory.ok());
  const rclcpp::Time epoch(100, 250000000, RCL_ROS_TIME);
  const auto result = makePathMessage(trajectory.value(), epoch, 101, "map");
  ASSERT_TRUE(result.ok());
  const auto &path = result.value();
  ASSERT_EQ(path.poses.size(), 101U);
  EXPECT_EQ(path.header.frame_id, "map");
  EXPECT_EQ(rclcpp::Time(path.header.stamp).nanoseconds(), epoch.nanoseconds());
  EXPECT_NEAR(path.poses.front().pose.position.y, 1.0, 1e-12);
  EXPECT_NEAR(path.poses.back().pose.position.y, 1.2, 1e-12);
  for (std::size_t i = 0; i < path.poses.size(); ++i)
  {
    const auto &pose = path.poses[i];
    EXPECT_EQ(pose.header.frame_id, "map");
    const double relativeTime =
      static_cast<double>(rclcpp::Time(pose.header.stamp).nanoseconds() - epoch.nanoseconds()) * 1e-9;
    const double expectedTime = 4.6875 * static_cast<double>(i) / 100.0;
    EXPECT_NEAR(relativeTime, expectedTime, 1e-9);
  }
  EXPECT_NEAR(path.poses.back().pose.orientation.z, std::sqrt(0.5), 1e-12);
}

TEST(PreviewMath, RejectsUnboundedDisplaySizeAndOverflowingTime)
{
  const auto trajectory = createTrajectory(exampleState(), exampleConfig());
  ASSERT_TRUE(trajectory.ok());
  const rclcpp::Time epoch(0, 0, RCL_ROS_TIME);
  EXPECT_FALSE(makePathMessage(trajectory.value(), epoch, 1, "map").ok());
  EXPECT_FALSE(makePathMessage(trajectory.value(), epoch, 10002, "map").ok());
  EXPECT_FALSE(makePathMessage(trajectory.value(), epoch, 101, "").ok());
  const rclcpp::Time nearEnd(std::numeric_limits<std::int32_t>::max(), 0, RCL_ROS_TIME);
  EXPECT_FALSE(makePathMessage(trajectory.value(), nearEnd, 101, "map").ok());
}
