/**
 * 原版 Hippo 四推进器的物理分配与静态推力曲线逆映射。
 * 此文件是仿真专用适配：几何、桨旋向和曲线来自固定的 USTROV 模型。
 * 不可把这些数值当成用户真实四推进器 ROV 的标定。
 */
#ifndef ROV_CONTROL_CORE_HIPPO_ALLOCATOR_H_
#define ROV_CONTROL_CORE_HIPPO_ALLOCATOR_H_
#include "rov_control_core/types.h"
#include <array>

namespace rov_control_core
{
struct AllocationOutput
{
  std::array<double, 8> throttle{}; // ROS 既有消息8槽位；只有前4路，后4路必须零
  Eigen::Vector4d forces_N = Eigen::Vector4d::Zero();
  Eigen::Vector4d achieved = Eigen::Vector4d::Zero(); // Fx[N], Mx/My/Mz[N·m]
  double scale = 1.0; // 同比例缩小所有推力，保持请求 wrench 方向，不逐电机硬裁剪
};
class HippoAllocator
{
public:
  explicit HippoAllocator(double maxCommand = 0.4);
  // 参数入口校验后调用；限幅集中在分配器，发送节点不重复另一套试验阈值。
  void setMaxCommand(double maxCommand);
  static Status validateMaxCommand(double maxCommand);
  // 输入物理前向力与三个力矩；返回电机命令、实际分配值和限幅比例。
  // 不保存上一周期、不发布消息；零请求返回精确零，不靠怠速抵消死区。
  Result<AllocationOutput> allocate(double force_x_N, const Eigen::Vector3d &torque_B_Nm) const;
  const Eigen::Matrix4d &matrix() const
  {
    return matrix_;
  }
  static double forceFromThrottle(double throttle); // 单台沿其正推力方向，供公式验证
  static double throttleFromForce(double force_N);  // 带符号曲线逆映射
private:
  Eigen::Matrix4d matrix_;
  Eigen::Matrix4d inverse_;
  double maxForce_N_ = 0.0; // 由电机指令上限经正向曲线得到的单桨推力上限
};
} // namespace rov_control_core
#endif
