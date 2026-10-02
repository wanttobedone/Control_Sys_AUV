/**
 * 五次时间轨迹：为一条直线路径分配运动时间。
 *
 * 使用顺序：plan() 得到路径 → create() 得到轨迹 → sample(t) 查询参考。
 * MPC 后续可调用 sampleHorizon()，一次读取未来多个时刻的参考。
 *
 * 运动假设：静止起步、静止结束，起终点加速度为零，参考姿态保持不变。
 * 本类只生成参考，不根据真实机器人误差修改轨迹，也不发布控制命令。
 */

#ifndef ROV_CONTROL_CORE_QUINTIC_TRAJECTORY_H_
#define ROV_CONTROL_CORE_QUINTIC_TRAJECTORY_H_

#include <cstddef>
#include <vector>

#include "rov_control_core/planning/line_path.h"

namespace rov_control_core
{

class QuinticTrajectory
{
public:
  // 输入：有效路径、运动学限制、固定参考姿态 q_WB。
  // 输出：保存了路径和总时间的轨迹对象，或明确的错误原因。
  static Result<QuinticTrajectory> create(const LinePath &path,
                                          const MotionLimits &limits,
                                          const Eigen::Quaterniond &attitude);

  // 输入：长度 [m]、速度/加速度/可选 jerk 限制。
  // 输出：满足这些参考运动学限制的总时间 T [s]，尚不考虑推力能力。
  static Result<double> chooseDuration(double length, const MotionLimits &limits);

  // 输入：从轨迹起点开始计时的 t [s]。
  // 输出：同一时刻的位置、速度、加速度和姿态参考。
  // t<=0 保持起点，t>=T 保持终点；两侧保持段的速度/加速度均为零。
  Result<ReferenceSample> sample(double time) const;

  // 输入：起始时间 [s]、采样间隔 [s]、点数和预先设置好 size 的数组。
  // 输出：out[0..count-1] 中的未来参考；超出轨迹末端时填终点静止参考。
  // 不 resize、不申请临时数组；失败时整个 out 保持原值。
  Status sampleHorizon(double time,
                       double timeStep,
                       std::size_t count,
                       std::vector<ReferenceSample> &out) const;

  double duration() const;      // 查询总时间 T [s]
  const LinePath &path() const;  // 查询几何路径，不允许修改内部路径

private:
  // 只有 create() 完成参数检查后才会调用构造函数。
  QuinticTrajectory(const LinePath &path, double duration, const Eigen::Quaterniond &attitude);

  LinePath path_;                // 已确定的几何路径
  double duration_;              // 运动总时间 T [s]
  Eigen::Quaterniond attitude_;  // 固定参考姿态 q_WB
  double velocityScale_;         // L/T [m/s]，采样时再乘 h'(tau)
  double accelerationScale_;     // L/T² [m/s²]，采样时再乘 h''(tau)
};

}  // 命名空间 rov_control_core

#endif
