#include "rov_control_core/tracking/hippo_allocator.h"
#include "rov_control_core/tracking/planar_tracker.h"
#include <gtest/gtest.h>
#include <limits>
using namespace rov_control_core;
namespace
{
RobotState resting()
{
  RobotState state;
  state.p_W = Eigen::Vector3d(1.3, 1, -0.2);
  state.q_WB = Eigen::AngleAxisd(1.5707963267948966, Eigen::Vector3d::UnitZ());
  return state;
}
} // namespace
TEST(PlanarConfig, AcceptsConfiguredLimitsAndReportsSpecificInvalidParameter)
{
  PlanarConfig config;
  config.motion.v_max_mps = 0.8;
  config.motion.a_max_mps2 = 0.6;
  config.max_force_N = 8.0;
  config.max_torque_Nm = {0.2, 0.2, 0.2};
  config.position_kp_Npm = 0.0;
  config.attitude_kp.setZero();
  config.mission_timeout_s = 0.0;
  EXPECT_TRUE(PlanarTracker::validateConfig(config).ok());
  config.motion.v_max_mps = -0.8;
  const auto invalid = PlanarTracker::validateConfig(config);
  EXPECT_FALSE(invalid.ok());
  EXPECT_NE(invalid.message.find("max_speed_mps"), std::string::npos);
  EXPECT_NE(invalid.message.find("-0.8"), std::string::npos);
}

TEST(HippoAllocation, ConfiguredMotorLimitChangesAchievableForce)
{
  for (double limit : {0.1, 0.2, 0.6, 1.0})
  {
    HippoAllocator allocator(limit);
    const auto result = allocator.allocate(100.0, {0, 0, 0});
    ASSERT_TRUE(result.ok());
    // 纯前向分配四桨等力。饱和后合力应是单桨配置边界推力的四倍。
    EXPECT_NEAR(result.value().achieved[0], 4 * HippoAllocator::forceFromThrottle(limit), 1e-10);
    for (double command : result.value().throttle)
    {
      EXPECT_LE(std::abs(command), limit + 1e-12);
    }
  }
  EXPECT_FALSE(HippoAllocator::validateMaxCommand(1.01).ok());
}

TEST(PlanarTracker, ParameterUpdatePreservesTaskAndExistingTrajectory)
{
  auto state = resting();
  PlanarConfig config;
  PlanarTracker tracker(config);
  ASSERT_TRUE(tracker.setGoal({1.3, 1.3, -0.2}, state).ok());
  for (int i = 0; i < 40; ++i)
  {
    ASSERT_TRUE(tracker.update(state, 0.02).ok());
  }
  ASSERT_EQ(tracker.phase(), TrackingPhase::TRACKING);
  const auto duration = tracker.trajectory()->duration();
  const auto elapsed = tracker.elapsed();
  config.motion.v_max_mps = 0.8;
  config.motion.a_max_mps2 = 0.6;
  tracker.setConfig(config);
  EXPECT_EQ(tracker.elapsed(), elapsed);
  EXPECT_EQ(tracker.trajectory()->duration(), duration);
  EXPECT_EQ(tracker.phase(), TrackingPhase::TRACKING);
  // core只要求正dt，上限交给实际调度它的ROS节点配置。
  EXPECT_TRUE(tracker.update(state, 0.2).ok());
  ASSERT_TRUE(tracker.setGoal({1.3, 1.4, -0.2}, state).ok());
  for (int i = 0; i < 40; ++i)
  {
    ASSERT_TRUE(tracker.update(state, 0.02).ok());
  }
  ASSERT_TRUE(tracker.trajectory());
  EXPECT_LT(tracker.trajectory()->duration(), duration);
}

TEST(HomeFrame, NinetyDegreeHeadingAndTranslationRoundTrip)
{
  const auto home = HomeFrame::fromState(resting());
  ASSERT_TRUE(home.ok());
  // 初始朝北：前0.3、左0.2应变成世界x减0.2、y加0.3，z保持。
  const auto world = home.value().toWorld({0.3, 0.2, 0});
  EXPECT_TRUE(world.isApprox(Eigen::Vector3d(1.1, 1.3, -0.2), 1e-12));
  EXPECT_TRUE(home.value().toHome(world).isApprox(Eigen::Vector3d(0.3, 0.2, 0), 1e-12));
}
TEST(HippoAllocation, IndependentLeverArmAndReactionTorqueBalance)
{
  HippoAllocator allocator;
  const auto output = allocator.allocate(0.8, {0.002, -0.01, 0.02});
  ASSERT_TRUE(output.ok());
  const auto f = output.value().forces_N;
  const double arm = 0.097 / std::sqrt(2.0);
  EXPECT_NEAR(f.sum(), 0.8, 1e-12);
  EXPECT_NEAR(0.01 * (-f[0] + f[1] - f[2] + f[3]), 0.002, 1e-12);
  EXPECT_NEAR(arm * (f[0] + f[1] - f[2] - f[3]), -0.01, 1e-12);
  EXPECT_NEAR(arm * (-f[0] + f[1] + f[2] - f[3]), 0.02, 1e-12);
  for (std::size_t i = 4; i < 8; ++i)
  {
    EXPECT_EQ(output.value().throttle[i], 0);
  }
}
TEST(HippoAllocation, InvertsDeadZoneAndScalesSaturation)
{
  EXPECT_EQ(HippoAllocator::forceFromThrottle(0.1), 0);
  EXPECT_EQ(HippoAllocator::throttleFromForce(0), 0);
  for (double f : {-1.0, -0.01, 0.01, 1.0})
  {
    EXPECT_NEAR(HippoAllocator::forceFromThrottle(HippoAllocator::throttleFromForce(f)), f, 1e-12);
  }
  HippoAllocator allocator;
  const auto output = allocator.allocate(100, {0, 0, 0});
  ASSERT_TRUE(output.ok());
  EXPECT_LT(output.value().scale, 1);
  for (double throttle : output.value().throttle)
  {
    EXPECT_LE(std::abs(throttle), 0.40000001);
  }
  EXPECT_FALSE(allocator.allocate(std::numeric_limits<double>::quiet_NaN(), {0, 0, 0}).ok());
}
TEST(PlanarTracker, BehindGoalProducesTurnInsteadOfGeometricPiStall)
{
  auto state = resting();
  PlanarTracker tracker(PlanarConfig{});
  ASSERT_TRUE(tracker.setGoal({1.3, 0.8, -0.2}, state).ok());
  TrackingOutput output;
  for (int i = 0; i < 30; ++i)
  {
    output = tracker.update(state, 0.02).value();
  }
  EXPECT_EQ(output.phase, TrackingPhase::TURNING);
  EXPECT_GT(std::abs(output.torque_B_Nm.z()), 0.02); // 180°不能因sin(pi)=0而没有转向输出
  EXPECT_EQ(output.force_x_N, 0);
}
TEST(PlanarTracker, RejectedAndRepeatedGoalsPreserveCurrentTask)
{
  auto state = resting();
  PlanarTracker tracker(PlanarConfig{});
  ASSERT_TRUE(tracker.setGoal({1.3, 1.3, -0.2}, state).ok());
  for (int i = 0; i < 60; ++i)
  {
    ASSERT_TRUE(tracker.update(state, 0.02).ok());
  }
  ASSERT_EQ(tracker.phase(), TrackingPhase::TRACKING);
  const double elapsed = tracker.elapsed();
  ASSERT_TRUE(tracker.setGoal({1.3, 1.3, -0.2}, state).ok());
  EXPECT_EQ(tracker.elapsed(), elapsed);
  EXPECT_FALSE(tracker.setGoal({50, 50, -0.2}, state).ok());
  EXPECT_EQ(tracker.phase(), TrackingPhase::TRACKING);
  EXPECT_TRUE(tracker.setGoal({1.2, 1.2, -0.2}, state).ok());
  EXPECT_EQ(tracker.phase(), TrackingPhase::BRAKING);
}
TEST(PlanarTracker, ArrivalUsesActualDistanceAndSpeedThenRemainsHolding)
{
  auto state = resting();
  PlanarTracker tracker(PlanarConfig{});
  ASSERT_TRUE(tracker.setGoal(state.p_W, state).ok());
  for (int i = 0; i < 30; ++i)
  {
    ASSERT_TRUE(tracker.update(state, 0.02).ok());
  }
  EXPECT_EQ(tracker.phase(), TrackingPhase::HOLDING);
  const auto output = tracker.update(state, 0.02);
  ASSERT_TRUE(output.ok());
  EXPECT_NEAR(output.value().force_x_N, 0, 1e-12);
  // 横向漂离容差圈必须重新接近；不能无限报告已经到点。
  state.p_W.x() += 0.06;
  EXPECT_EQ(tracker.update(state, 0.02).value().phase, TrackingPhase::BRAKING);
  tracker.reset();
  EXPECT_EQ(tracker.phase(), TrackingPhase::IDLE);
}
TEST(PlanarTracker, CrossTrackSignSteersBackTowardPath)
{
  auto state = resting();
  PlanarTracker tracker(PlanarConfig{});
  ASSERT_TRUE(tracker.setGoal({1.3, 1.3, -0.2}, state).ok());
  for (int i = 0; i < 60; ++i)
  {
    ASSERT_TRUE(tracker.update(state, 0.02).ok());
  }
  state.p_W.x() -= 0.02; // 北向路径的左侧是世界-x
  const auto output = tracker.update(state, 0.02);
  ASSERT_TRUE(output.ok());
  EXPECT_GT(output.value().cross_track_m, 0);
  EXPECT_LT(output.value().heading_error_rad, 0);
  EXPECT_FALSE(tracker.update(state, -0.02).ok());
}
