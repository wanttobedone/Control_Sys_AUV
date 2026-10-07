#include "rov_control_ros/hippo_command.h"
#include <gtest/gtest.h>
#include <limits>

using namespace rov_control_ros;
using namespace rov_control_core;

TEST(HippoCommand, PulseUsesKnownTrapezoidTimesAndEndsAtZero)
{
  ProbeConfig config;
  config.amplitude = 0.6;
  config.hold_s = 0.3; // 此用例独立指定时间，验证梯形公式，不依赖运行配置默认值。
  // 基线1秒，上升0.1秒，因此1.05秒对应一半幅值0.3；下降中点同理。
  for (const auto &point : {std::pair<double, double>{0.0, 0.0},
                            {0.9, 0.0},
                            {1.05, 0.3},
                            {1.2, 0.6},
                            {1.45, 0.3},
                            {1.6, 0.0},
                            {100.0, 0.0}})
  {
    const auto result = samplePulse(point.first, config);
    ASSERT_TRUE(result.ok());
    EXPECT_NEAR(result.value().surge_effort, point.second, 1e-12);
    EXPECT_DOUBLE_EQ(result.value().torque_effort_B.norm(), 0.0);
  }
}

TEST(HippoCommand, EveryAxisIsIsolatedAndSupportsNegativeInput)
{
  ProbeConfig config;
  config.amplitude = -0.6;
  int axis = 0;
  for (ProbeChannel channel : {ProbeChannel::ROLL, ProbeChannel::PITCH, ProbeChannel::YAW})
  {
    config.channel = channel;
    const auto result = samplePulse(1.2, config);
    ASSERT_TRUE(result.ok());
    EXPECT_DOUBLE_EQ(result.value().surge_effort, 0.0);
    Eigen::Vector3d expected = Eigen::Vector3d::Zero();
    expected[axis++] = -0.6;
    EXPECT_TRUE(result.value().torque_effort_B.isApprox(expected, 1e-12));
  }
}

TEST(HippoCommand, InvalidConfigurationAndSamplingAreRejected)
{
  ProbeConfig config;
  config.amplitude = 0.66;
  EXPECT_FALSE(validateProbeConfig(config).ok());
  config.amplitude = 0.0;
  config.hold_s = 0.0;
  EXPECT_FALSE(validateProbeConfig(config).ok()); // 不变配置在启动入口检查一次。
  config = ProbeConfig{};
  EXPECT_FALSE(samplePulse(-0.1, config).ok());
  EXPECT_FALSE(samplePulse(std::numeric_limits<double>::quiet_NaN(), config).ok());
  config.channel = static_cast<ProbeChannel>(99);
  EXPECT_FALSE(validateProbeConfig(config).ok());
}

TEST(HippoCommand, ConfigurableTrialValuesAreNotCappedByHistoricalExperiment)
{
  ProbeConfig config;
  config.effort_limit = 0.9;
  config.amplitude = 0.8;
  config.hold_s = 15.0;
  config.max_receive_gap_s = 1.2;
  config.max_rotation_rad = 0.8;
  ASSERT_TRUE(validateProbeConfig(config).ok());
  const auto command = samplePulse(2.0, config);
  ASSERT_TRUE(command.ok());
  EXPECT_DOUBLE_EQ(command.value().surge_effort, 0.8);
  EXPECT_TRUE(makeActuatorMessages(command.value(), rclcpp::Time(1LL), "body", 0.9).ok());
  config.effort_limit = 1.1;
  EXPECT_FALSE(validateProbeConfig(config).ok()); // 归一化接口边界仍然有效。
}

TEST(HippoCommand, MessagesExplicitlyClearEveryUnusedAxis)
{
  HippoEffortCommand command;
  command.surge_effort = 0.6;
  const auto result =
      makeActuatorMessages(command, rclcpp::Time(1000000000LL), "uuv00/base_link", 0.65);
  ASSERT_TRUE(result.ok());
  const auto &pair = result.value();
  EXPECT_DOUBLE_EQ(pair.thrust.x, 0.6);
  EXPECT_DOUBLE_EQ(pair.thrust.y, 0.0);
  EXPECT_DOUBLE_EQ(pair.thrust.z, 0.0);
  EXPECT_DOUBLE_EQ(pair.torque.x, 0.0);
  EXPECT_DOUBLE_EQ(pair.torque.y, 0.0);
  EXPECT_DOUBLE_EQ(pair.torque.z, 0.0);
  EXPECT_FALSE(pair.thrust.ignore_x || pair.thrust.ignore_y || pair.thrust.ignore_z);
  EXPECT_FALSE(pair.torque.ignore_x || pair.torque.ignore_y || pair.torque.ignore_z);
  EXPECT_EQ(pair.thrust.header, pair.torque.header);
}

TEST(HippoCommand, InvalidCommandsNeverBecomeMessages)
{
  HippoEffortCommand command;
  command.torque_effort_B.y() = 0.7;
  EXPECT_FALSE(makeActuatorMessages(command, rclcpp::Time(1LL), "base", 0.65).ok());
  command = {};
  command.surge_effort = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(makeActuatorMessages(command, rclcpp::Time(1LL), "base", 0.65).ok());
  command = {};
  EXPECT_FALSE(makeActuatorMessages(command, rclcpp::Time(1LL), "", 0.65).ok());
}

TEST(HippoGuard, EquivalentQuaternionsAndSmallMotionAreAccepted)
{
  ProbeConfig config;
  RobotState start;
  start.p_W = Eigen::Vector3d(1.3, 1.0, -0.2);
  RobotState state = start;
  state.q_WB.coeffs() *= -1.0;
  state.p_W.y() += 0.02;
  EXPECT_TRUE(checkProbeState(state, &start, {}, config).ok());
  state.p_W.y() += 0.2;
  EXPECT_FALSE(checkProbeState(state, &start, {}, config).ok());
}

TEST(HippoGuard, TimeAndAttitudeFailuresAreRejected)
{
  ProbeConfig config;
  RobotState state;
  state.p_W = Eigen::Vector3d(1.3, 1.0, -0.2);
  const RobotState start = state;
  ProbeTiming timing;
  timing.clock_stall_s = 0.16;
  EXPECT_FALSE(checkProbeState(state, &start, timing, config).ok());
  timing = {};
  timing.clock_went_back = true;
  EXPECT_FALSE(checkProbeState(state, &start, timing, config).ok());
  timing = {};
  timing.state_age_s = -0.06;
  EXPECT_FALSE(checkProbeState(state, &start, timing, config).ok());
  timing = {};
  timing.ros_step_s = 0.11;
  EXPECT_FALSE(checkProbeState(state, &start, timing, config).ok());
  timing = {};
  state.q_WB = Eigen::AngleAxisd(0.20, Eigen::Vector3d::UnitZ());
  EXPECT_FALSE(checkProbeState(state, &start, timing, config).ok());
}
