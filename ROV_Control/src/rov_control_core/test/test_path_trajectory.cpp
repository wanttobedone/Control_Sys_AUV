/**
 * A 批路径与轨迹的 C++ 测试，也是数学核心的调用示例。
 *
 * 阅读顺序：共用输入 → 空间检查 → 几何路径 → 状态投影 → 时长 → 轨迹采样。
 * TEST(分组, 名称) 定义一个用例；ASSERT 失败会停止本用例，EXPECT 失败会继续检查。
 * 读取 Result::value() 前用 ASSERT_TRUE(result.ok())，避免把失败结果当成有效数据。
 * EXPECT_NEAR 用绝对误差容差比较浮点数；EXPECT_DOUBLE_EQ 检查近似机器精度相等。
 *
 * 此文件不需要 ROS 节点、Gazebo 或实机。示例参数不构成真机配置。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

#include "rov_control_core/planning/line_path.h"
#include "rov_control_core/planning/quintic_trajectory.h"
#include "rov_control_core/planning/workspace_bounds.h"

namespace rov_control_core
{
namespace
{

constexpr double PI = 3.14159265358979323846;
constexpr double NAN_VALUE = std::numeric_limits<double>::quiet_NaN();
constexpr double INFINITY_VALUE = std::numeric_limits<double>::infinity();

// 共用教学场景：从 (0,0,-0.4) m 向世界 +x 走 0.2 m。
// bounds 描述允许参考点活动的范围，不是对真实池尺寸的声明。
PlanRequest exampleRequest()
{
  PlanRequest request;
  request.bounds.min_W = {-1.0, -1.0, -1.0};
  request.bounds.max_W = {1.0, 1.0, 0.0};
  request.p_start_W = {0.0, 0.0, -0.4};
  request.p_goal_W = {0.2, 0.0, -0.4};
  return request;
}

// 共用运动学限制；不含推进器能力，不能据此证明真机能够跟踪。
MotionLimits exampleLimits()
{
  MotionLimits limits;
  limits.v_max_mps = 0.08;
  limits.a_max_mps2 = 0.1;
  limits.min_duration_s = 0.1;
  return limits;
}

// 执行约束仅供首轮前向实验：起速 ≤0.01 m/s、倾斜 ≤5°、朝向误差 ≤1°。
ForwardStartConstraint exampleForwardConstraint()
{
  ForwardStartConstraint constraint;
  constraint.max_start_speed_mps = 0.01;
  constraint.max_tilt_rad = 5.0 * PI / 180.0;
  constraint.max_heading_error_rad = PI / 180.0;
  return constraint;
}

// 工作空间允许边界上的点，拒绝盒外的点与线段。
// 盒子是凸集：两端都在盒内时，整条直线也在盒内，不必离散采点。
TEST(Workspace, ConvexBoxIncludesBoundaryAndRejectsOutsideSegment)
{
  const auto bounds = exampleRequest().bounds;
  EXPECT_TRUE(validateBounds(bounds).ok());
  EXPECT_TRUE(contains(bounds, bounds.min_W));
  EXPECT_TRUE(contains(bounds, bounds.max_W));
  EXPECT_TRUE(containsSegment(bounds, bounds.min_W, bounds.max_W));
  EXPECT_FALSE(contains(bounds, Eigen::Vector3d(1.00001, 0.0, -0.4)));
  EXPECT_FALSE(containsSegment(bounds, Eigen::Vector3d(0.0, 0.0, -0.4),
                               Eigen::Vector3d(0.0, 0.0, 0.1)));
}

// 无效边界不能被当作“无限制”。
// 依次检查默认零体积、上下界颠倒、无穷大边界，以及 NaN 位置。
TEST(Workspace, InvalidOrNonFiniteBoundsFailClosed)
{
  WorkspaceBounds bounds;
  EXPECT_EQ(validateBounds(bounds).code, ErrorCode::INVALID_BOUNDS);
  bounds = exampleRequest().bounds;
  bounds.min_W.y() = bounds.max_W.y() + 1.0;
  EXPECT_FALSE(contains(bounds, Eigen::Vector3d::Zero()));
  bounds = exampleRequest().bounds;
  bounds.max_W.z() = INFINITY_VALUE;
  EXPECT_EQ(validateBounds(bounds).code, ErrorCode::NON_FINITE);
  EXPECT_FALSE(contains(exampleRequest().bounds, Eigen::Vector3d(NAN_VALUE, 0.0, 0.0)));
}

// 几何层支持三维直线，与后续执行器是否能跟踪这条线分开检查。
// 位移为 (0.3,0.4,0.2) m，因此长度应为 sqrt(0.29) m，单位方向模长为 1。
TEST(LinePath, BuildsGeneralThreeDimensionalGeometry)
{
  auto request = exampleRequest();
  request.p_goal_W = {0.3, 0.4, -0.2};
  const auto result = LinePathPlanner::plan(request);
  ASSERT_TRUE(result.ok()) << result.status().message;
  const auto &path = result.value();
  EXPECT_NEAR(path.length(), std::sqrt(0.29), 1e-14);
  EXPECT_NEAR(path.direction().norm(), 1.0, 1e-14);
  EXPECT_TRUE((path.startPosition() + path.direction() * path.length())
                  .isApprox(request.p_goal_W, 1e-13));
}

// 相同起终点与低于数值阈值的位移返回“无需运动”。
// 越界目标、NaN 目标和无效边界分别返回具体原因，不能构造有效路径。
TEST(LinePath, RejectsBadEndpointsAndReportsNoMotion)
{
  auto request = exampleRequest();
  request.p_goal_W = request.p_start_W;
  EXPECT_EQ(LinePathPlanner::plan(request).status().code, ErrorCode::NO_MOTION);
  request.p_goal_W.x() += 0.5 * MIN_PATH_LENGTH_M;
  EXPECT_EQ(LinePathPlanner::plan(request).status().code, ErrorCode::NO_MOTION);
  request.p_goal_W.x() = 2.0;
  EXPECT_EQ(LinePathPlanner::plan(request).status().code, ErrorCode::OUTSIDE_WORKSPACE);
  request.p_goal_W.x() = NAN_VALUE;
  EXPECT_EQ(LinePathPlanner::plan(request).status().code, ErrorCode::NON_FINITE);
  request = exampleRequest();
  request.bounds.max_W = request.bounds.min_W;
  EXPECT_EQ(LinePathPlanner::plan(request).status().code, ErrorCode::INVALID_BOUNDS);
}

// 检查数值溢出边界：两个端点分别有限，相减后仍可能超出 double 范围。
// 此时应报 NUMERICAL_RANGE，不能继续用无穷大长度计算单位方向。
TEST(LinePath, ReportsUnrepresentableLength)
{
  auto request = exampleRequest();
  request.bounds.min_W = Eigen::Vector3d::Constant(-1.7e308);
  request.bounds.max_W = Eigen::Vector3d::Constant(1.7e308);
  request.p_start_W = {-1.7e308, 0.0, 0.0};
  request.p_goal_W = {1.7e308, 0.0, 0.0};
  EXPECT_EQ(LinePathPlanner::plan(request).status().code, ErrorCode::NUMERICAL_RANGE);
}

// 前向约束依据机体实际朝向，而不是固定把世界 x 方向当作前方。
// yaw=0 时朝世界 +x；yaw=90° 时朝世界 +y；侧向与反向目标应拒绝。
TEST(LinePath, ForwardExecutionUsesInitialHeadingIncludingNinetyDegreeYaw)
{
  auto request = exampleRequest();
  request.forward_start = exampleForwardConstraint();
  ASSERT_TRUE(LinePathPlanner::plan(request).ok());
  request.forward_start->q_WB = Eigen::AngleAxisd(PI / 2.0, Eigen::Vector3d::UnitZ());
  EXPECT_EQ(LinePathPlanner::plan(request).status().code, ErrorCode::UNSUPPORTED_DIRECTION);
  request.p_goal_W = {0.0, 0.2, -0.4};
  ASSERT_TRUE(LinePathPlanner::plan(request).ok());
  request.p_goal_W.y() = -0.2;
  EXPECT_EQ(LinePathPlanner::plan(request).status().code, ErrorCode::UNSUPPORTED_DIRECTION);
}

// 首轮执行方案只接受近水平、接近静止起步的前向水平线。
// 分别触发：目标有高度变化、机体倾斜过大、已有速度超过起步阈值。
TEST(LinePath, ForwardExecutionRejectsVerticalMotionTiltAndMovingStart)
{
  auto request = exampleRequest();
  request.forward_start = exampleForwardConstraint();
  request.p_goal_W.z() += 0.05;
  EXPECT_EQ(LinePathPlanner::plan(request).status().code, ErrorCode::UNSUPPORTED_DIRECTION);
  request.p_goal_W.z() = -0.4;
  request.forward_start->q_WB = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY());
  EXPECT_EQ(LinePathPlanner::plan(request).status().code, ErrorCode::START_TILTED);
  request.forward_start->q_WB = Eigen::Quaterniond::Identity();
  request.forward_start->v_W.y() = 0.02;
  EXPECT_EQ(LinePathPlanner::plan(request).status().code, ErrorCode::START_MOVING);
}

// 执行约束本身也需要检查，避免把坏状态误认为“可以开始”。
// 零四元数、NaN 速度、过宽的朝向阈值应产生不同错误码。
TEST(LinePath, ForwardExecutionRejectsBadQuaternionVelocityAndLimits)
{
  auto request = exampleRequest();
  request.forward_start = exampleForwardConstraint();
  request.forward_start->q_WB.coeffs().setZero();
  EXPECT_EQ(LinePathPlanner::plan(request).status().code, ErrorCode::INVALID_QUATERNION);
  request.forward_start = exampleForwardConstraint();
  request.forward_start->v_W.x() = NAN_VALUE;
  EXPECT_EQ(LinePathPlanner::plan(request).status().code, ErrorCode::NON_FINITE);
  request.forward_start = exampleForwardConstraint();
  request.forward_start->max_heading_error_rad = PI;
  EXPECT_EQ(LinePathPlanner::plan(request).status().code, ErrorCode::INVALID_LIMITS);
}

// 将沿世界 +y 的运动投影到路径，并保留越过终点与落在起点后方的信息。
// 机器人比参考线低 0.05 m；世界 z 向上，因此 depth_offset_m 为正 0.05 m。
TEST(Progress, ProjectsWorldVelocityAndPreservesOvershootAndDepthSign)
{
  auto request = exampleRequest();
  request.p_goal_W = {0.0, 0.2, -0.4};
  const auto path = LinePathPlanner::plan(request);
  ASSERT_TRUE(path.ok());
  RobotState state;
  state.p_W = {0.03, 0.25, -0.45};
  state.v_W = {0.5, 0.1, 0.0};
  const auto result = projectProgress(path.value(), state);
  ASSERT_TRUE(result.ok());
  EXPECT_NEAR(result.value().s_m, 0.25, 1e-14);  // 终点为 0.2 m，这里要保留超出的 0.05 m
  EXPECT_NEAR(result.value().v_s_mps, 0.1, 1e-14);
  EXPECT_TRUE(result.value().cross_track_W.isApprox(Eigen::Vector3d(0.03, 0.0, -0.05)));
  EXPECT_NEAR(result.value().depth_offset_m, 0.05, 1e-14);
  state.p_W.y() = -0.1;
  EXPECT_NEAR(projectProgress(path.value(), state).value().s_m, -0.1, 1e-14);
}

// 投影前拒绝非有限状态，避免一个 NaN 速度污染控制器误差和日志。
TEST(Progress, RejectsNonFiniteState)
{
  const auto path = LinePathPlanner::plan(exampleRequest());
  ASSERT_TRUE(path.ok());
  RobotState state;
  state.v_W.z() = NAN_VALUE;
  EXPECT_EQ(projectProgress(path.value(), state).status().code, ErrorCode::NON_FINITE);
}

// 让加速度限制足够宽，使速度限制决定总时间。
// 长度 1 m，max(h')=15/8，v_max=0.25 m/s，所以 T=(15/8)/0.25=7.5 s。
TEST(Duration, VelocityBoundSelectsTime)
{
  auto limits = exampleLimits();
  limits.v_max_mps = 0.25;
  limits.a_max_mps2 = 100.0;
  const auto duration = QuinticTrajectory::chooseDuration(1.0, limits);
  ASSERT_TRUE(duration.ok());
  EXPECT_DOUBLE_EQ(duration.value(), 7.5);  // (15/8) / 0.25
}

// 分别让加速度、jerk、最短时长限制成为决定项。
// 加速度用解析峰值 10*sqrt(3)/3，jerk 用解析峰值 60，验证选择的是最大下界。
TEST(Duration, AccelerationJerkAndMinimumDurationCanEachDominate)
{
  auto limits = exampleLimits();
  limits.v_max_mps = 100.0;
  limits.a_max_mps2 = 0.2;
  auto duration = QuinticTrajectory::chooseDuration(1.0, limits);
  ASSERT_TRUE(duration.ok());
  EXPECT_NEAR(duration.value(), std::sqrt(50.0 / std::sqrt(3.0)), 1e-13);
  limits.a_max_mps2 = 100.0;
  limits.j_max_mps3 = 0.1;
  duration = QuinticTrajectory::chooseDuration(1.0, limits);
  ASSERT_TRUE(duration.ok());
  EXPECT_NEAR(duration.value(), std::cbrt(600.0), 1e-13);
  limits.min_duration_s = 100.0;
  EXPECT_DOUBLE_EQ(QuinticTrajectory::chooseDuration(1.0, limits).value(), 100.0);
}

// 各运动限制都必须是正的有限数。
// 循环验证 0、负数、NaN、无穷大，再验证极小速度上限导致的总时间溢出。
TEST(Duration, RejectsInvalidLimitsAndNumericalOverflow)
{
  for (const double invalid : {0.0, -1.0, NAN_VALUE, INFINITY_VALUE})
  {
    auto limits = exampleLimits();
    limits.v_max_mps = invalid;
    EXPECT_EQ(QuinticTrajectory::chooseDuration(0.2, limits).status().code, ErrorCode::INVALID_LIMITS);
    limits = exampleLimits();
    limits.a_max_mps2 = invalid;
    EXPECT_EQ(QuinticTrajectory::chooseDuration(0.2, limits).status().code, ErrorCode::INVALID_LIMITS);
    limits = exampleLimits();
    limits.j_max_mps3 = invalid;
    EXPECT_EQ(QuinticTrajectory::chooseDuration(0.2, limits).status().code, ErrorCode::INVALID_LIMITS);
    limits = exampleLimits();
    limits.min_duration_s = invalid;
    EXPECT_EQ(QuinticTrajectory::chooseDuration(0.2, limits).status().code, ErrorCode::INVALID_LIMITS);
    EXPECT_FALSE(QuinticTrajectory::chooseDuration(invalid, exampleLimits()).ok());
  }
  auto limits = exampleLimits();
  limits.v_max_mps = std::numeric_limits<double>::min();
  EXPECT_EQ(QuinticTrajectory::chooseDuration(10.0, limits).status().code, ErrorCode::NUMERICAL_RANGE);
}

// 最小完整调用示例：请求 → 路径 → 时间轨迹 → 5 个参考采样点。
// 期望值事先由公式独立计算；打印结果用于学习，不代表机器人实测轨迹。
TEST(Trajectory, ExamplePrintsIndependentlyCheckedQuarterPoints)
{
  // 本例的路径、边界与运动限制只用于数学验证，不是当前 ROV/水池的标定值。
  const auto path = LinePathPlanner::plan(exampleRequest());
  ASSERT_TRUE(path.ok());
  const auto created =
      QuinticTrajectory::create(path.value(), exampleLimits(), Eigen::Quaterniond::Identity());
  ASSERT_TRUE(created.ok()) << created.status().message;
  const auto &trajectory = created.value();
  ASSERT_NEAR(trajectory.duration(), 4.6875, 1e-13);

  // 独立代入可得 h(1/4)=53/512、h'(1/4)=135/128、h''(1/4)=45/8。
  // 再分别乘 L、L/T、L/T²，得到下面三组期望值。
  const double expectedPosition[] = {0.0, 0.020703125, 0.1, 0.179296875, 0.2};
  const double expectedVelocity[] = {0.0, 0.045, 0.08, 0.045, 0.0};
  const double expectedAcceleration[] = {0.0, 0.0512, 0.0, -0.0512, 0.0};
  std::cout << "教学示例：L=0.2 m，vmax=0.08 m/s，amax=0.1 m/s^2\n"
            << "t[s]          s[m]          v[m/s]        a[m/s^2]\n";
  for (int i = 0; i <= 4; ++i)
  {
    const auto result = trajectory.sample(trajectory.duration() * static_cast<double>(i) / 4.0);
    ASSERT_TRUE(result.ok());
    const auto &sample = result.value();
    EXPECT_NEAR(sample.s_m, expectedPosition[i], 1e-13);
    EXPECT_NEAR(sample.v_s_mps, expectedVelocity[i], 1e-13);
    EXPECT_NEAR(sample.a_s_mps2, expectedAcceleration[i], 1e-13);
    EXPECT_TRUE(sample.p_W.isApprox(Eigen::Vector3d(expectedPosition[i], 0.0, -0.4), 1e-13));
    std::cout << std::fixed << std::setprecision(9) << sample.t_from_start_s << "   "
              << sample.s_m << "   " << sample.v_s_mps << "   " << sample.a_s_mps2 << '\n';
  }
}

// 轨迹时段之外应保持端点，速度和加速度为零，不能继续外推五次多项式。
// t_from_start_s 保留查询时间，便于未来采样和日志；无效时间应拒绝。
TEST(Trajectory, OutsideTimeHoldsExactEndpointsWithZeroDerivatives)
{
  const auto path = LinePathPlanner::plan(exampleRequest());
  ASSERT_TRUE(path.ok());
  const auto created =
      QuinticTrajectory::create(path.value(), exampleLimits(), Eigen::Quaterniond::Identity());
  ASSERT_TRUE(created.ok());
  const auto &trajectory = created.value();
  for (const double t : {-10.0, 0.0, trajectory.duration(), trajectory.duration() + 10.0})
  {
    const auto result = trajectory.sample(t);
    ASSERT_TRUE(result.ok());
    const auto &sample = result.value();
    EXPECT_DOUBLE_EQ(sample.t_from_start_s, t);
    EXPECT_TRUE(sample.p_W == (t <= 0.0 ? path.value().startPosition() : path.value().goalPosition()));
    EXPECT_TRUE(sample.v_W.isZero(0.0));
    EXPECT_TRUE(sample.a_W.isZero(0.0));
    EXPECT_DOUBLE_EQ(sample.v_s_mps, 0.0);
    EXPECT_DOUBLE_EQ(sample.a_s_mps2, 0.0);
  }
  EXPECT_FALSE(trajectory.sample(NAN_VALUE).ok());
  EXPECT_FALSE(trajectory.sample(INFINITY_VALUE).ok());
}

// 直接检查加速度解析极值点，避免仅靠均匀采样漏掉峰值。
// 极值发生在 tau=(3±sqrt(3))/6，正负峰值分别达到 ±a_max。
TEST(Trajectory, AnalyticAccelerationExtremaRespectTheLimit)
{
  const auto path = LinePathPlanner::plan(exampleRequest());
  ASSERT_TRUE(path.ok());
  auto limits = exampleLimits();
  limits.v_max_mps = 10.0;
  limits.a_max_mps2 = 0.02;
  const auto created =
      QuinticTrajectory::create(path.value(), limits, Eigen::Quaterniond::Identity());
  ASSERT_TRUE(created.ok());
  const double early = (3.0 - std::sqrt(3.0)) / 6.0;
  const auto positivePeak = created.value().sample(early * created.value().duration());
  const auto negativePeak = created.value().sample((1.0 - early) * created.value().duration());
  ASSERT_TRUE(positivePeak.ok());
  ASSERT_TRUE(negativePeak.ok());
  EXPECT_NEAR(positivePeak.value().a_s_mps2, limits.a_max_mps2, 1e-13);
  EXPECT_NEAR(negativePeak.value().a_s_mps2, -limits.a_max_mps2, 1e-13);
}

// 改变路径长度并使用斜向三维方向，验证进度单调、参考位于盒内。
// 同时核对世界系速度/加速度模长、沿线到三维的映射，以及解析 jerk 峰值。
TEST(Trajectory, MotionLimitsHoldAcrossLengthsAndWorldDirections)
{
  for (const double length : {0.001, 0.2, 1.0})
  {
    auto request = exampleRequest();
    const Eigen::Vector3d direction = Eigen::Vector3d(1.0, 2.0, -1.0).normalized();
    request.p_goal_W = request.p_start_W + length * direction;
    const auto path = LinePathPlanner::plan(request);
    ASSERT_TRUE(path.ok());
    auto limits = exampleLimits();
    limits.j_max_mps3 = 0.05;
    const auto created =
        QuinticTrajectory::create(path.value(), limits, Eigen::Quaterniond::Identity());
    ASSERT_TRUE(created.ok());
    double lastProgress = -1.0;
    for (int i = 0; i <= 100; ++i)
    {
      const auto sample =
          created.value().sample(created.value().duration() * static_cast<double>(i) / 100.0);
      ASSERT_TRUE(sample.ok());
      EXPECT_GE(sample.value().s_m, lastProgress);
      EXPECT_LE(sample.value().s_m, length + 1e-13);
      EXPECT_LE(sample.value().v_W.norm(), limits.v_max_mps + 1e-13);
      EXPECT_LE(sample.value().a_W.norm(), limits.a_max_mps2 + 1e-13);
      EXPECT_TRUE(contains(request.bounds, sample.value().p_W));
      EXPECT_TRUE((sample.value().v_W - direction * sample.value().v_s_mps).isZero(1e-13));
      lastProgress = sample.value().s_m;
    }
    EXPECT_LE(60.0 * length / std::pow(created.value().duration(), 3), *limits.j_max_mps3 + 1e-13);
  }
}

// 用中心差分独立检查位置→速度、速度→加速度是否一致。
// epsilon 是差分时间间隔；容差允许差分截断与浮点舍入误差。
TEST(Trajectory, DerivativesMatchFiniteDifferencesOfWorldPositionAndVelocity)
{
  const auto path = LinePathPlanner::plan(exampleRequest());
  ASSERT_TRUE(path.ok());
  const auto created =
      QuinticTrajectory::create(path.value(), exampleLimits(), Eigen::Quaterniond::Identity());
  ASSERT_TRUE(created.ok());
  const auto &trajectory = created.value();
  const double epsilon = trajectory.duration() * 1e-5;
  for (const double fraction : {0.15, 0.37, 0.68, 0.9})
  {
    const double t = fraction * trajectory.duration();
    const auto before = trajectory.sample(t - epsilon);
    const auto center = trajectory.sample(t);
    const auto after = trajectory.sample(t + epsilon);
    ASSERT_TRUE(before.ok() && center.ok() && after.ok());
    const Eigen::Vector3d numericalVelocity =
        (after.value().p_W - before.value().p_W) / (2.0 * epsilon);
    const Eigen::Vector3d numericalAcceleration =
        (after.value().v_W - before.value().v_W) / (2.0 * epsilon);
    EXPECT_TRUE((numericalVelocity - center.value().v_W).isZero(1e-9));
    EXPECT_TRUE((numericalAcceleration - center.value().a_W).isZero(1e-9));
  }
}

// 只修正单位四元数的小幅舍入误差；整条轨迹保持指定姿态和零期望角速度。
// 明显错误的四元数不能通过无条件 normalize() 掩盖。
TEST(Trajectory, NormalizesOnlySmallAttitudeRoundoffAndKeepsAttitudeConstant)
{
  const auto path = LinePathPlanner::plan(exampleRequest());
  ASSERT_TRUE(path.ok());
  Eigen::Quaterniond attitude(Eigen::AngleAxisd(PI / 2.0, Eigen::Vector3d::UnitZ()));
  attitude.coeffs() *= 1.0 + 1e-7;
  const auto created =
      QuinticTrajectory::create(path.value(), exampleLimits(), attitude);
  ASSERT_TRUE(created.ok());
  for (const double t : {0.0, 1.0, 20.0})
  {
    const auto sample =
        created.value().sample(t);
    ASSERT_TRUE(sample.ok());
    EXPECT_NEAR(sample.value().q_WB_des.norm(), 1.0, 1e-14);
    EXPECT_TRUE((sample.value().q_WB_des * Eigen::Vector3d::UnitX()).isApprox(Eigen::Vector3d::UnitY()));
    EXPECT_TRUE(sample.value().omega_B_des.isZero(0.0));
  }
  for (const double invalid : {0.0, 2.0, NAN_VALUE, INFINITY_VALUE})
  {
    const Eigen::Quaterniond bad(invalid, 0.0, 0.0, 0.0);
    EXPECT_EQ(QuinticTrajectory::create(path.value(), exampleLimits(), bad).status().code,
              ErrorCode::INVALID_QUATERNION);
  }
}

// 未来参考写入调用方已经分配好的数组：地址、容量和 size 不变。
// 只写前 count 个元素；超出轨迹末端的采样保持终点静止。
TEST(Horizon, FillsPreallocatedBufferAndPadsTerminalReference)
{
  const auto path = LinePathPlanner::plan(exampleRequest());
  ASSERT_TRUE(path.ok());
  const auto created =
      QuinticTrajectory::create(path.value(), exampleLimits(), Eigen::Quaterniond::Identity());
  ASSERT_TRUE(created.ok());
  const auto &trajectory = created.value();
  std::vector<ReferenceSample> buffer(5);
  buffer[4].t_from_start_s = 1234.0;
  const auto *address = buffer.data();
  const auto capacity = buffer.capacity();
  ASSERT_TRUE(trajectory.sampleHorizon(trajectory.duration() - 0.1, 0.1, 4, buffer).ok());
  EXPECT_EQ(address, buffer.data());
  EXPECT_EQ(capacity, buffer.capacity());
  EXPECT_EQ(buffer.size(), 5U);
  EXPECT_DOUBLE_EQ(buffer[4].t_from_start_s, 1234.0);
  EXPECT_LT(buffer[0].s_m, 0.2);
  for (std::size_t i = 2; i < 4; ++i)
  {
    EXPECT_TRUE(buffer[i].p_W == path.value().goalPosition());
    EXPECT_DOUBLE_EQ(buffer[i].v_s_mps, 0.0);
  }
  for (std::size_t i = 1; i < 4; ++i)
  {
    EXPECT_NEAR(buffer[i].t_from_start_s - buffer[i - 1].t_from_start_s, 0.1, 1e-13);
  }
}

// 验证未来采样失败时，输出数组仍完整保留旧值。
// 包括容量不足、零点数、坏时间间隔、时间溢出和时间无法递增等情况。
TEST(Horizon, RejectsBadRequestsWithoutPartiallyChangingOutput)
{
  const auto path = LinePathPlanner::plan(exampleRequest());
  ASSERT_TRUE(path.ok());
  const auto created =
      QuinticTrajectory::create(path.value(), exampleLimits(), Eigen::Quaterniond::Identity());
  ASSERT_TRUE(created.ok());
  std::vector<ReferenceSample> buffer(3);
  for (auto &sample : buffer)
  {
    sample.s_m = 42.0;
    sample.t_from_start_s = 99.0;
  }
  const auto &trajectory = created.value();
  EXPECT_EQ(trajectory.sampleHorizon(0.0, 0.1, 4, buffer).code, ErrorCode::BUFFER_TOO_SMALL);
  EXPECT_EQ(trajectory.sampleHorizon(0.0, 0.1, 0, buffer).code, ErrorCode::INVALID_SAMPLING);
  for (const double step : {0.0, -0.1, NAN_VALUE, INFINITY_VALUE})
  {
    EXPECT_EQ(trajectory.sampleHorizon(0.0, step, 3, buffer).code, ErrorCode::INVALID_SAMPLING);
  }
  EXPECT_EQ(trajectory.sampleHorizon(NAN_VALUE, 0.1, 3, buffer).code, ErrorCode::INVALID_SAMPLING);
  // 每个输入都有限，但第三个时间戳 2*1e308 超出 double 范围。
  EXPECT_EQ(trajectory.sampleHorizon(0.0, 1e308, 3, buffer).code, ErrorCode::INVALID_SAMPLING);
  // 在 1e300 的量级，double 已无法分辨 0.01 s 的增量，时间戳将不再递增。
  EXPECT_EQ(trajectory.sampleHorizon(1e300, 0.01, 3, buffer).code, ErrorCode::INVALID_SAMPLING);
  for (const auto &sample : buffer)
  {
    EXPECT_DOUBLE_EQ(sample.s_m, 42.0);
    EXPECT_DOUBLE_EQ(sample.t_from_start_s, 99.0);
  }
}

}  // 匿名命名空间
}  // 命名空间 rov_control_core
