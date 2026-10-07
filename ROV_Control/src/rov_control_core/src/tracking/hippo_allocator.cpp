#include "rov_control_core/tracking/hippo_allocator.h"
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rov_control_core
{
HippoAllocator::HippoAllocator(double maxCommand)
{
  const auto status = validateMaxCommand(maxCommand);
  if (!status.ok())
  {
    throw std::invalid_argument(status.message);
  }
  setMaxCommand(maxCommand);
  const double arm = 0.097 / std::sqrt(2.0);
  // 每列对应一个推进器：Fx=f，My=z*f，Mz=-y*f；Mx含桨反作用力矩。
  // physical f 的正方向均沿 body+x；电机转向符号在最后一步映射。
  matrix_ << 1, 1, 1, 1, -0.01, 0.01, -0.01, 0.01, arm, arm, -arm, -arm, -arm, arm, arm, -arm;
  inverse_ = matrix_.inverse(); // 固定满秩4×4矩阵；正式测试独立核对力臂和可逆性
}
Status HippoAllocator::validateMaxCommand(double maxCommand)
{
  // 1 是归一化执行器接口边界，0.4 只是默认配置；死区内的限幅意味着无可用推力。
  if (!std::isfinite(maxCommand) || maxCommand <= 0 || maxCommand > 1)
  {
    return {ErrorCode::INVALID_LIMITS,
            "max_thruster_command=" + std::to_string(maxCommand) + " 必须位于 (0,1]"};
  }
  return {};
}
void HippoAllocator::setMaxCommand(double maxCommand)
{
  maxForce_N_ = forceFromThrottle(maxCommand);
}
double HippoAllocator::forceFromThrottle(double throttle)
{
  const double n = 50.0 * std::abs(throttle); // 3000 rpm = 50 rev/s
  const double force = std::max(0.0, 0.003313 * n * n + 0.1086 * n - 0.8576);
  return std::copysign(force, throttle);
}
double HippoAllocator::throttleFromForce(double force_N)
{
  if (std::abs(force_N) < 1e-6)
  {
    return 0.0;
  }
  // 解 a*n²+b*n+c=|f| 的正根。f趋于0时命令趋近死区边界，是该模型的固有映射。
  const double n =
      (-0.1086 + std::sqrt(0.1086 * 0.1086 + 4.0 * 0.003313 * (std::abs(force_N) + 0.8576))) /
      (2.0 * 0.003313);
  return std::copysign(n / 50.0, force_N);
}
Result<AllocationOutput> HippoAllocator::allocate(double force_x_N,
                                                  const Eigen::Vector3d &torque_B_Nm) const
{
  if (!std::isfinite(force_x_N) || !torque_B_Nm.allFinite())
  {
    return Result<AllocationOutput>::failure(ErrorCode::NON_FINITE, "物理控制请求含非有限数");
  }
  Eigen::Vector4d requested;
  requested << force_x_N, torque_B_Nm;
  AllocationOutput output;
  output.forces_N = inverse_ * requested;
  if (!output.forces_N.allFinite())
  {
    return Result<AllocationOutput>::failure(ErrorCode::NUMERICAL_RANGE, "分配数值溢出");
  }
  const double peak = output.forces_N.cwiseAbs().maxCoeff();
  if (peak > maxForce_N_)
  {
    output.scale = maxForce_N_ / peak;
    output.forces_N *= output.scale;
  }
  const std::array<double, 4> sign{-1, 1, -1, 1};
  for (std::size_t i = 0; i < 4; ++i)
  {
    output.throttle[i] = sign[i] * throttleFromForce(output.forces_N[static_cast<Eigen::Index>(i)]);
  }
  output.achieved = matrix_ * output.forces_N;
  return Result<AllocationOutput>::success(output);
}
} // namespace rov_control_core
