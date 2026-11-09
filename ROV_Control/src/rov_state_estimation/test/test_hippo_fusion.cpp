#include "rov_state_estimation/hippo_fusion.h"

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace rs = rov_state_estimation;
namespace
{

Eigen::Quaterniond yaw(double angle)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
}

// 解析数据源：没有读取被测实现的结果来生成观测。参考位姿和压力由指定真值计算。
// 相机延迟 80ms、5Hz；IMU 200Hz；压力 50Hz。使用类似 Unix 的大时间戳。
struct Rig
{
  explicit Rig(const rs::FusionConfig &config = rs::FusionConfig{}) : fusion(config)
  {
  }
  rs::HippoFusion fusion;
  int step{0};
  int64_t sensor_period_ns{5000000LL};
  int64_t vision_delay_ns{80000000LL};
  rs::RigidPose truth{{0.1, 0.2, 0.35}, yaw(-0.7)};
  std::optional<rs::FusionOutput> output;
  double first_ready_s{-1.0};
  struct DelayedVision
  {
    int delivery;
    int64_t stamp;
    rs::RigidPose pose;
  };
  std::deque<DelayedVision> queue;
  static constexpr int64_t epoch = 1793700000000000000LL;

  void tick(const Eigen::Vector3d &acceleration_W = Eigen::Vector3d::Zero(),
            const Eigen::Vector3d &omega_B = Eigen::Vector3d::Zero(), bool vision = true)
  {
    const int64_t stamp = epoch + static_cast<int64_t>(step) * sensor_period_ns;
    const Eigen::Vector3d force_B =
        truth.q.conjugate() * (acceleration_W + Eigen::Vector3d(0, 0, 9.81));
    const auto next = fusion.pushImu(stamp, omega_B, force_B);
    if (next)
    {
      output = next;
      if (next->ready && first_ready_s < 0.0)
      {
        first_ready_s = static_cast<double>(step * sensor_period_ns) * 1e-9;
      }
    }
    if (step % (20000000 / sensor_period_ns) == 0)
    {
      fusion.setPressure(stamp, 101325.0 + (0.35 - truth.p_m.z()) * 1000.0 * 9.81);
    }
    if (vision && step % (200000000 / sensor_period_ns) == 0)
    {
      auto observation = truth;
      // 主动交替符号：同一姿态的 q/-q 不能产生错误的大角度校正。
      if ((step / (200000000 / sensor_period_ns)) % 2 == 0)
      {
        observation.q.coeffs() *= -1.0;
      }
      queue.push_back({step + static_cast<int>(vision_delay_ns / sensor_period_ns), stamp, observation});
    }
    while (!queue.empty() && queue.front().delivery <= step)
    {
      fusion.setVision(queue.front().stamp, queue.front().pose);
      queue.pop_front();
    }
    ++step;
  }
  void settle()
  {
    for (int i = 0; i < 60000000000LL / sensor_period_ns; ++i)
    {
      tick();
    }
  }
};

TEST(FrameAdapter, RecoversKnownBodyAndRotatedLeverArm)
{
  rs::FrameAdapter frames;
  // 独立手算：水平机体 (0.1,0.2,0.4)，相机后移5cm、下移12.5cm。
  const rs::RigidPose camera{{0.05, -0.2, -0.275}, {0, 0, 0, 1}};
  const auto body = frames.cameraToBody(camera);
  EXPECT_LT((body.p_m - Eigen::Vector3d(0.1, 0.2, 0.4)).norm(), 1e-12);
  EXPECT_LT(body.q.angularDistance(Eigen::Quaterniond::Identity()), 1e-12);
  // 世界中机头转到 +Y，杆臂的 -5cm 应落在世界 -Y，不能固定减世界 X。
  const rs::RigidPose camera_turned{{0.1, -0.15, -0.275},
                                    frames.config().tag_WL.q.conjugate() * yaw(M_PI / 2) *
                                        frames.config().camera_BC.q};
  const auto turned = frames.cameraToBody(camera_turned);
  EXPECT_LT((turned.p_m - Eigen::Vector3d(0.1, 0.2, 0.4)).norm(), 1e-12);
  EXPECT_LT(turned.q.angularDistance(yaw(M_PI / 2)), 1e-12);
}

TEST(FrameAdapter, HomeKeepsTiltAndDoesNotMoveUntilReset)
{
  rs::FrameAdapter frames;
  const Eigen::Quaterniond roll(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()));
  rs::RigidPose body{{1, 2, 3}, yaw(M_PI / 2) * roll};
  ASSERT_TRUE(frames.initializeHome(body));
  EXPECT_LT(frames.toHome(body).p_m.norm(), 1e-12);
  EXPECT_LT(frames.toHome(body).q.angularDistance(roll), 1e-12);
  body.p_m.y() += 0.05;
  frames.initializeHome(body);
  EXPECT_NEAR(frames.toHome(body).p_m.x(), 0.05, 1e-12);
  frames.resetHome();
  EXPECT_FALSE(frames.home());
  ASSERT_TRUE(frames.initializeHome(body));
  EXPECT_LT(frames.toHome(body).p_m.norm(), 1e-12);
}

TEST(FrameAdapter, RejectsInvalidPoseAndVerticalHeading)
{
  rs::FrameAdapter frames;
  rs::RigidPose body;
  body.q = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));
  EXPECT_FALSE(frames.initializeHome(body));
  body.q.coeffs().setZero();
  EXPECT_THROW(frames.cameraToBody(body), std::invalid_argument);
}

TEST(HippoFusion, StationaryNonzeroPoseWithDelayedVisionAndQuaternionSignChanges)
{
  Rig rig;
  rig.settle();
  ASSERT_TRUE(rig.output);
  EXPECT_TRUE(rig.fusion.initialized())
      << rig.fusion.status() << " tilt variance=" << rig.fusion.tiltVarianceRad2();
  EXPECT_TRUE(rig.output->ready);
  EXPECT_LT((rig.output->body_WB.p_m - rig.truth.p_m).norm(), 0.03);
  EXPECT_LT(rig.output->body_WB.q.angularDistance(rig.truth.q), 0.05);
  EXPECT_LT(rig.output->velocity_W_mps.norm(), 0.03);
  EXPECT_EQ(rig.fusion.rejectedInputs(), 0u) << rig.fusion.inputError();
  RecordProperty("first_ready_simulation_s", rig.first_ready_s);
  EXPECT_GT(rig.first_ready_s, 3.0);
  EXPECT_LT(rig.first_ready_s, 60.0);
  const auto &checks = rig.fusion.diagnostics();
  EXPECT_EQ(checks.readiness_blocker, "NONE");
  EXPECT_TRUE(checks.attitude_valid && checks.vision_fresh && checks.pressure_fresh);
  // 判定按10ms步长执行：首次通过应累计0.50s，随后保留，不随运行时间无限增长。
  EXPECT_NEAR(checks.startup_consistent_s, 0.50, 1e-9);
  EXPECT_EQ(checks.check_stamp_ns, rig.output->stamp_ns);
}

TEST(HippoFusion, UserPoseGeometryIsAValidStationaryInitialization)
{
  Rig rig;
  rs::FrameAdapter frames;
  // 用户实际一帧的外参换算作为静态合成试验姿态；IMU/压力仍是解析生成，非实机融合记录。
  rig.truth = frames.cameraToBody({{-0.0122564689, -0.0920248688, -0.2321225764},
                                   {0.7014767197, 0.0566343966, 0.0423394674, -0.7091758078}});
  rig.settle();
  ASSERT_TRUE(rig.output);
  EXPECT_TRUE(rig.output->ready) << rig.fusion.status();
  EXPECT_LT((rig.output->body_WB.p_m - rig.truth.p_m).norm(), 0.04);
  EXPECT_LT(rig.output->body_WB.q.angularDistance(rig.truth.q), 0.05);
  RecordProperty("first_ready_simulation_s", rig.first_ready_s);
}

TEST(HippoFusion, Resamples250HzInputWithoutChangingIntegrationTime)
{
  Rig rig;
  rig.sensor_period_ns = 4000000LL;
  rig.settle();
  ASSERT_TRUE(rig.output && rig.output->ready);
  EXPECT_EQ((rig.output->stamp_ns - Rig::epoch) % 10000000LL, 0);
  const auto initial_q = rig.output->body_WB.q;
  for (int i = 1; i <= 250; ++i)
  {
    rig.truth.q = yaw(-0.7 + 0.2 * 0.004 * i);
    rig.tick(Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 0.2), false);
  }
  EXPECT_NEAR(initial_q.angularDistance(rig.output->body_WB.q), 0.2, 0.04);
  EXPECT_LT((rig.output->body_WB.p_m - rig.truth.p_m).norm(), 0.04);
}

TEST(HippoFusion, TiltedStationaryPoseConverges)
{
  Rig rig;
  rig.truth.q = yaw(-0.7) * Eigen::Quaterniond(Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitY())) *
                Eigen::Quaterniond(Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX()));
  rig.settle();
  ASSERT_TRUE(rig.output);
  EXPECT_TRUE(rig.output->ready) << rig.fusion.status();
  EXPECT_LT(rig.output->body_WB.q.angularDistance(rig.truth.q), 0.05);
  EXPECT_LT((rig.output->body_WB.p_m - rig.truth.p_m).norm(), 0.04);
}

TEST(HippoFusion, TracksSmoothHorizontalAndPressureHeightMotion)
{
  Rig rig;
  rig.settle();
  ASSERT_TRUE(rig.fusion.initialized());
  // 4s 半余弦，从 x=.1,z=.35 到 x=.2,z=.40；起止速度为零。
  for (int i = 0; i <= 800; ++i)
  {
    const double phase = M_PI * i / 800.0;
    const double scale = 1.0 - std::cos(phase);
    rig.truth.p_m = Eigen::Vector3d(0.1 + 0.05 * scale, 0.2, 0.35 + 0.025 * scale);
    const double acceleration = std::pow(M_PI / 4.0, 2) * std::cos(phase);
    rig.tick(Eigen::Vector3d(0.05 * acceleration, 0, 0.025 * acceleration));
  }
  for (int i = 0; i < 400; ++i)
  {
    rig.tick();
  }
  ASSERT_TRUE(rig.output);
  EXPECT_LT((rig.output->body_WB.p_m - rig.truth.p_m).norm(), 0.04);
  EXPECT_LT(rig.output->velocity_W_mps.norm(), 0.04);
}

TEST(HippoFusion, GyroPropagatesThroughOcclusionAndReportsDegraded)
{
  Rig rig;
  rig.settle();
  ASSERT_TRUE(rig.output && rig.output->ready);
  const auto initial = rig.output->body_WB.q;
  for (int i = 1; i <= 240; ++i)
  {
    rig.truth.q = yaw(-0.7 + 0.2 * 0.005 * i);
    rig.tick(Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 0.2), false);
  }
  ASSERT_TRUE(rig.output);
  EXPECT_FALSE(rig.output->ready);
  EXPECT_TRUE(rig.fusion.initialized());
  EXPECT_EQ(rig.fusion.diagnostics().readiness_blocker, "WAIT_FRESH_VISION");
  EXPECT_FALSE(rig.fusion.diagnostics().vision_fresh);
  EXPECT_NEAR(initial.angularDistance(rig.output->body_WB.q), 0.24, 0.05);
  for (int i = 0; i < 400; ++i)
  {
    rig.tick();
  }
  EXPECT_TRUE(rig.output->ready);
}

TEST(HippoFusion, RejectsBadTimesAndLatchesImuGap)
{
  rs::HippoFusion fusion;
  const Eigen::Vector3d gravity(0, 0, 9.81);
  fusion.pushImu(Rig::epoch, Eigen::Vector3d::Zero(), gravity);
  EXPECT_FALSE(fusion.setVision(Rig::epoch + 1000000000, rs::RigidPose{}));
  EXPECT_FALSE(fusion.setPressure(Rig::epoch, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(fusion.pushImu(Rig::epoch, Eigen::Vector3d::Zero(), gravity));
  EXPECT_FALSE(fusion.pushImu(Rig::epoch + 200000000, Eigen::Vector3d::Zero(), gravity));
  EXPECT_EQ(fusion.status(), "FAULT_IMU_GAP_RESTART_NODE");
  const auto rejected_at_fault = fusion.rejectedInputs();
  const auto fault = fusion.diagnostics();
  // 独立构造的源时间跳变为200ms；接收恢复与否不改变这次锁定故障。
  EXPECT_NEAR(fault.fault_imu_gap_s, 0.2, 1e-12);
  EXPECT_EQ(fault.fault_stamp_ns, Rig::epoch + 200000000);
  EXPECT_EQ(fault.fault_reason, "imu_gap_no_invented_integration");
  EXPECT_FALSE(fusion.pushImu(Rig::epoch + 210000000, Eigen::Vector3d::Zero(), gravity));
  // 模拟数十秒后仍有压力/视觉回调。不能拿它们和冻结时钟比较，制造几千次次生拒收。
  EXPECT_FALSE(fusion.setPressure(Rig::epoch + 37000000000LL, 101325.0));
  EXPECT_FALSE(fusion.setVision(Rig::epoch + 37000000000LL, rs::RigidPose{}));
  EXPECT_EQ(fusion.rejectedInputs(), rejected_at_fault);
  EXPECT_EQ(fusion.inputError(), "imu_gap_no_invented_integration");
  EXPECT_EQ(fusion.diagnostics().fault_stamp_ns, fault.fault_stamp_ns);
  EXPECT_EQ(fusion.diagnostics().last_imu_dt_s, fault.last_imu_dt_s);
}

TEST(HippoFusion, DiagnosticsStartUnavailableRatherThanZeroError)
{
  rs::HippoFusion fusion;
  const auto &checks = fusion.diagnostics();
  EXPECT_EQ(checks.check_stamp_ns, 0);
  EXPECT_EQ(checks.filter_update_count, 0u);
  EXPECT_TRUE(std::isnan(checks.position_error_m));
  EXPECT_TRUE(std::isnan(checks.angle_error_rad));
  EXPECT_EQ(checks.readiness_blocker, "WAIT_FILTER_OUTPUT");
}

TEST(HippoFusion, ReportsActualAgreementErrorsBeforeInitializationWithLongVisionDelay)
{
  rs::FusionConfig config;
  config.history_s = 0.8;
  config.vision_timeout_s = 2.0;
  config.startup_position_tolerance_m = 0.20;
  config.startup_angle_tolerance_rad = M_PI / 6.0;
  // 只为此测试延长启动等待，确保诊断发生在 initialized=false 时，而不是借 READY 取数据。
  config.startup_settle_s = 120.0;
  Rig rig(config);
  rig.vision_delay_ns = 720000000LL;
  rig.settle();
  ASSERT_TRUE(rig.output);
  ASSERT_FALSE(rig.fusion.initialized());
  const auto settled = rig.fusion.diagnostics();
  ASSERT_TRUE(settled.attitude_valid);
  EXPECT_EQ(settled.readiness_blocker, "WAIT_STARTUP_SETTLE");
  EXPECT_GT(settled.filter_update_count, 1000u);
  // 5Hz、720ms传输延迟，最新观测的年龄随两帧之间的时间在0.72～0.92s之间变化。
  EXPECT_GE(settled.vision_age_s, 0.72);
  EXPECT_LT(settled.vision_age_s, 0.93);
  EXPECT_NEAR(settled.vision_age_s,
              static_cast<double>(settled.check_stamp_ns - settled.vision_stamp_ns) * 1e-9, 1e-12);
  EXPECT_LT(settled.position_error_m, 0.04);
  EXPECT_LT(settled.angle_error_rad, 0.05);

  // 在静止估计上突然输入+40cm的视觉偏差；EKF延迟状态尚未使用它，差值应约40cm。
  const int64_t stamp = Rig::epoch + static_cast<int64_t>(rig.step) * rig.sensor_period_ns;
  auto shifted = rig.truth;
  shifted.p_m.x() += 0.40;
  ASSERT_TRUE(rig.fusion.setVision(stamp, shifted));
  const Eigen::Vector3d force_B = rig.truth.q.conjugate() * Eigen::Vector3d(0, 0, 9.81);
  ASSERT_TRUE(rig.fusion.pushImu(stamp, Eigen::Vector3d::Zero(), force_B));
  const auto position_check = rig.fusion.diagnostics();
  EXPECT_NEAR(position_check.position_error_m, 0.40, 0.04);
  EXPECT_EQ(position_check.readiness_blocker, "WAIT_POSITION_AGREEMENT");
  EXPECT_EQ(rig.fusion.status(), "WAIT_POSITION_AGREEMENT");
  EXPECT_FALSE(position_check.position_consistent);
  EXPECT_EQ(position_check.startup_consistent_s, 0.0);

  // 恢复位置，改为+1rad的航向偏差，验证位置与姿态原因能独立区分。
  auto turned = rig.truth;
  turned.q = yaw(1.0) * turned.q;
  ASSERT_TRUE(rig.fusion.setVision(stamp + 10000000, turned));
  ASSERT_TRUE(rig.fusion.pushImu(stamp + 10000000, Eigen::Vector3d::Zero(), force_B));
  const auto angle_check = rig.fusion.diagnostics();
  EXPECT_NEAR(angle_check.angle_error_rad, 1.0, 0.05);
  EXPECT_TRUE(angle_check.position_consistent);
  EXPECT_FALSE(angle_check.angle_consistent);
  EXPECT_EQ(angle_check.readiness_blocker, "WAIT_ANGLE_AGREEMENT");
  EXPECT_EQ(rig.fusion.status(), "WAIT_ANGLE_AGREEMENT");
}

TEST(HippoFusion, DoesNotInitializeWithoutTimeMatchedPressure)
{
  rs::HippoFusion fusion;
  for (int i = 0; i < 500; ++i)
  {
    const int64_t stamp = Rig::epoch + static_cast<int64_t>(i) * 10000000;
    fusion.pushImu(stamp, Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 9.81));
    if (i % 20 == 0)
    {
      fusion.setVision(stamp, rs::RigidPose{});
    }
  }
  EXPECT_FALSE(fusion.pressureAligned());
  EXPECT_FALSE(fusion.initialized());
}

} // namespace
