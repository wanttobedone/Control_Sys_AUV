/**
 * 五次时间轨迹实现，纯数学计算。
 *
 * 几何路径：p(s) = p0 + d*s。
 * 时间分配：tau = t/T，h(tau) = 10*tau³ - 15*tau⁴ + 6*tau⁵。
 * 沿线参考：s = L*h，v_s = (L/T)*h'，a_s = (L/T²)*h''。
 *
 * 调用顺序：
 *   create() → chooseDuration() → 保存路径与 T；
 *   控制循环每次调用 sample(t)，或由 sampleHorizon() 生成未来参考。
 *
 * 本文件的速度/加速度都是参考值，不表示机器人已经实际达到这些值。
 */

#include "rov_control_core/planning/quintic_trajectory.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rov_control_core
{
namespace
{

// 正数与有限性检查，供速度/加速度限制、总时间和采样间隔共用。
bool isPositiveFinite(double value)
{
  return std::isfinite(value) && value > 0.0;
}

/**
 * 计算已经走过的距离比例 h(tau)，输入 tau 在 [0,1] 内。
 *
 * h(0)=0，h(1)=1；两端一阶、二阶导数都为零。
 * 后半段用 h(tau)=1-h(1-tau) 计算，减少接近终点时大数相减的精度损失。
 * 这是同一个五次多项式的等价写法，不是另一条轨迹。
 */
double positionFraction(double tau)
{
  bool firstHalf = tau <= 0.5;
  double x = tau;
  if (!firstHalf)
  {
    x = 1.0 - tau;
  }

  // 展开后为 10*x³ - 15*x⁴ + 6*x⁵；以下写法减少重复乘法。
  double fraction = x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
  if (!firstHalf)
  {
    fraction = 1.0 - fraction;
  }

  return fraction;
}

}  // 匿名命名空间

QuinticTrajectory::QuinticTrajectory(const LinePath &path,
                                     double duration,
                                     const Eigen::Quaterniond &attitude)
    : path_(path), duration_(duration), attitude_(attitude),
      velocityScale_(path.length() / duration),
      accelerationScale_(velocityScale_ / duration)
{
}

double QuinticTrajectory::duration() const
{
  return duration_;
}

const LinePath &QuinticTrajectory::path() const
{
  return path_;
}

Result<double> QuinticTrajectory::chooseDuration(double length, const MotionLimits &limits)
{
  // 1. 长度和限制值必须有效。零长度目标应该已在 plan() 中返回“无需运动”。
  if (!isPositiveFinite(length))
  {
    return Result<double>::failure(ErrorCode::INVALID_ARGUMENT, "路径长度必须为正的有限数值");
  }
  if (!isPositiveFinite(limits.v_max_mps) ||
      !isPositiveFinite(limits.a_max_mps2) ||
      !isPositiveFinite(limits.min_duration_s) ||
      (limits.j_max_mps3 && !isPositiveFinite(*limits.j_max_mps3)))
  {
    return Result<double>::failure(ErrorCode::INVALID_LIMITS, "运动限制必须为正的有限数值");
  }

  // 2. 根据解析导数峰值，分别计算满足每项约束所需的最短时间。
  //
  // max(h')   = 15/8          → T >= (15/8)*L/v_max
  // max|h''|  = 10*sqrt(3)/3  → T >= sqrt((10*sqrt(3)/3)*L/a_max)
  // max|h'''| = 60            → T >= cbrt(60*L/j_max)，仅启用 jerk 限制时使用
  //
  // long double 用于中间计算，减少先算 L/v 时就溢出的情况。
  // 常数后缀 L 表示 long double 类型，不是公式中的路径长度 L。
  long double preciseLength = length;
  long double velocityTime = 1.875L * preciseLength / limits.v_max_mps;
  long double accelerationTime =
      std::sqrt((10.0L * std::sqrt(3.0L) / 3.0L) * preciseLength / limits.a_max_mps2);

  // 3. 要同时满足所有下界，所以取最大的时间。
  long double requiredTime = std::max(
      {static_cast<long double>(limits.min_duration_s), velocityTime, accelerationTime});
  if (limits.j_max_mps3)
  {
    long double jerkTime = std::cbrt(60.0L * preciseLength / *limits.j_max_mps3);
    requiredTime = std::max(requiredTime, jerkTime);
  }

  // 4. 公共接口使用 double，先确认最终结果可以转换为 double。
  if (!std::isfinite(requiredTime) || requiredTime > std::numeric_limits<double>::max())
  {
    return Result<double>::failure(ErrorCode::NUMERICAL_RANGE, "轨迹总时间无法可靠表示");
  }

  double duration = static_cast<double>(requiredTime);
  if (static_cast<long double>(duration) < requiredTime)
  {
    // 若转换时向下舍入，向上挪到下一个可表示的数，避免低于刚算出的时间下界。
    duration = std::nextafter(duration, std::numeric_limits<double>::infinity());
  }
  if (!isPositiveFinite(duration))
  {
    return Result<double>::failure(ErrorCode::NUMERICAL_RANGE, "可表示的轨迹总时间不合法");
  }

  return Result<double>::success(duration);
}

Result<QuinticTrajectory> QuinticTrajectory::create(const LinePath &path,
                                                    const MotionLimits &limits,
                                                    const Eigen::Quaterniond &attitude)
{
  // 1. 检查参考姿态。明显无效的四元数不能仅靠 normalize() 掩盖。
  double quaternionNorm = attitude.coeffs().stableNorm();
  if (!attitude.coeffs().allFinite() || !std::isfinite(quaternionNorm) ||
      std::abs(quaternionNorm - 1.0) > QUATERNION_NORM_TOLERANCE)
  {
    return Result<QuinticTrajectory>::failure(
        ErrorCode::INVALID_QUATERNION, "参考姿态必须为接近单位长度的四元数");
  }

  // 2. 根据路径长度和运动限制选择总时间。错误继续原样交给调用者。
  Result<double> durationResult = chooseDuration(path.length(), limits);
  if (!durationResult.ok())
  {
    return Result<QuinticTrajectory>::failure(
        durationResult.status().code, durationResult.status().message);
  }

  // 3. 保存路径、时间、归一化后的姿态，并预先算好 L/T 与 L/T²。
  // 固定尺度只计算一次，每次 sample() 只需代入 tau 计算多项式。
  QuinticTrajectory trajectory(path, durationResult.value(), attitude.normalized());

  // 4. 防止极端输入让导数尺度溢出；轨迹对象有效不只要求 T 有限。
  double peakVelocity = 1.875 * trajectory.velocityScale_;
  double peakAcceleration = (10.0 * std::sqrt(3.0) / 3.0) * trajectory.accelerationScale_;
  if (!std::isfinite(peakVelocity) || !std::isfinite(peakAcceleration))
  {
    return Result<QuinticTrajectory>::failure(
        ErrorCode::NUMERICAL_RANGE, "轨迹导数无法可靠表示");
  }

  return Result<QuinticTrajectory>::success(trajectory);
}

Result<ReferenceSample> QuinticTrajectory::sample(double time) const
{
  // 1. 输入是轨迹开始后的相对时间 [s]，不是 ROS 的绝对时间戳。
  if (!std::isfinite(time))
  {
    return Result<ReferenceSample>::failure(ErrorCode::INVALID_SAMPLING, "采样时间必须为有限数值");
  }

  ReferenceSample reference;
  reference.t_from_start_s = time;
  reference.q_WB_des = attitude_;

  // 2. 轨迹开始前保持起点；结构体默认速度、加速度和期望角速度均为零。
  if (time <= 0.0)
  {
    reference.p_W = path_.startPosition();
  }
  else if (time >= duration_)
  {
    // 到达末端后保持原始终点，而不是继续延长多项式。
    // 未来 MPC 时域超过 T 时，也得到“停在终点”的参考。
    reference.s_m = path_.length();
    reference.p_W = path_.goalPosition();
  }
  else
  {
    // 3. 把实际时间归一化到 [0,1]，方便同一个多项式适应不同总时间。
    double tau = time / duration_;
    double oneMinusTau = 1.0 - tau;

    // 4. 五次多项式及其对 tau 的导数。
    // h'  = 30*tau²*(1-tau)²
    // h'' = 60*tau*(1-tau)*(1-2*tau)
    double h = positionFraction(tau);
    double hPrime = 30.0 * tau * tau * oneMinusTau * oneMinusTau;
    double hSecond = 60.0 * tau * oneMinusTau * (1.0 - 2.0 * tau);

    // 5. 换成对实际时间 t 的导数。每对 t 求导一次，就多乘一个 1/T。
    reference.s_m = path_.length() * h;
    reference.v_s_mps = velocityScale_ * hPrime;
    reference.a_s_mps2 = accelerationScale_ * hSecond;

    // 6. 将沿线标量参考映射回世界系三维向量。
    // 直线路径的单位方向 d 不随时间变化，所以这里没有额外曲率项。
    reference.p_W = path_.startPosition() + path_.direction() * reference.s_m;
    reference.v_W = path_.direction() * reference.v_s_mps;
    reference.a_W = path_.direction() * reference.a_s_mps2;
  }

  // 7. 输出前检查数值，避免把溢出的参考交给后续控制器。
  if (!reference.p_W.allFinite() || !reference.v_W.allFinite() || !reference.a_W.allFinite())
  {
    return Result<ReferenceSample>::failure(ErrorCode::NUMERICAL_RANGE, "参考采样计算溢出");
  }

  return Result<ReferenceSample>::success(reference);
}

Status QuinticTrajectory::sampleHorizon(double time,
                                        double timeStep,
                                        std::size_t count,
                                        std::vector<ReferenceSample> &out) const
{
  // 1. 调用者提前设置数组 size，控制循环中不反复申请和释放内存。
  if (!std::isfinite(time) || !isPositiveFinite(timeStep) || count == 0)
  {
    return {ErrorCode::INVALID_SAMPLING, "未来采样需要有限起始时间、正间隔和非零点数"};
  }
  if (out.size() < count)
  {
    return {ErrorCode::BUFFER_TOO_SMALL, "请先为未来参考数组设置足够的 size"};
  }

  // 2. 第一遍只检查，不写 out，保证中途失败不会留下半段新、半段旧参考。
  // fma(i, dt, t) 计算 t+i*dt，并减少一次中间舍入。
  double previousTime = time;
  for (std::size_t i = 0; i < count; ++i)
  {
    double sampleTime = std::fma(static_cast<double>(i), timeStep, time);
    if (!std::isfinite(sampleTime) || (i > 0 && sampleTime <= previousTime))
    {
      // 绝对数值过大时，t+dt 可能仍等于 t；这种时序不能交给 MPC 使用。
      return {ErrorCode::INVALID_SAMPLING, "未来采样时间溢出或无法以当前精度递增"};
    }

    Result<ReferenceSample> sampleResult = sample(sampleTime);
    if (!sampleResult.ok())
    {
      return sampleResult.status();
    }
    previousTime = sampleTime;
  }

  // 3. 全部通过后再写入前 count 个元素，数组后面的元素保持原值。
  // 两遍计算的代价是重复求值简单多项式，不需要额外的临时数组。
  for (std::size_t i = 0; i < count; ++i)
  {
    double sampleTime = std::fma(static_cast<double>(i), timeStep, time);
    Result<ReferenceSample> sampleResult = sample(sampleTime);
    out[i] = sampleResult.value();
  }

  return {};
}

}  // 命名空间 rov_control_core
