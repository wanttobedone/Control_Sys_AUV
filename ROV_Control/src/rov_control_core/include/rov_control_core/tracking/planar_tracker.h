/**
 * Hippo 水平到点控制的纯 C++ 核心。
 * 输入世界系 RobotState、目标点和 ROS 时间步；输出物理 Fx/Mx/My/Mz。
 * 无 ROS、无文件读写。深度只提供诊断，绝不生成独立 Fy/Fz 控制量。
 * 复用已有 LinePathPlanner、QuinticTrajectory、projectProgress。
 */
#ifndef ROV_CONTROL_CORE_PLANAR_TRACKER_H_
#define ROV_CONTROL_CORE_PLANAR_TRACKER_H_

#include "rov_control_core/planning/quintic_trajectory.h"
#include <optional>

namespace rov_control_core
{
// 上电局部坐标：平移取首次有效状态，旋转只取初始 yaw，z 始终竖直向上。
// 在节点生命周期中固定；机器人转向后不能重新用当前姿态转换旧目标。
struct HomeFrame
{
  Eigen::Vector3d origin_W = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_WH = Eigen::Quaterniond::Identity();
  // 输入首次有效世界状态；输出原点和仅含初始yaw的旋转，机头近竖直则拒绝。
  static Result<HomeFrame> fromState(const RobotState &state);
  // 点坐标转换包含平移；这些接口不能用于速度向量（速度只旋转）。单位均为米。
  Eigen::Vector3d toWorld(const Eigen::Vector3d &point_H) const;
  Eigen::Vector3d toHome(const Eigen::Vector3d &point_W) const;
};

enum class TrackingPhase
{
  IDLE,
  BRAKING,
  TURNING,
  TRACKING,
  HOLDING
};
const char *trackingPhaseName(TrackingPhase phase);
double wrapAngle(double angle); // 返回 [-pi,pi] 中等价角，处理跨 ±pi 的航向误差

struct PlanarConfig
{
  WorkspaceBounds bounds{Eigen::Vector3d(0.25, 0.25, -1.3), Eigen::Vector3d(1.8, 3.7, 0.1)};
  MotionLimits motion{0.08, 0.06, std::nullopt, 1.0};
  double mass_kg = 2.31;         // 当前 Hippo 名义质量，前馈近似，不称为辨识结果
  double surge_drag_Nspm = 5.39; // 上游水动力是线性阻尼，单位 N/(m/s)
  double position_kp_Npm = 4.0;
  double velocity_kd_Nspm = 4.5;
  double lookahead_m = 0.20;                      // 横向偏差通过航向纠正，不能直接请求横移推力
  Eigen::Vector3d attitude_kp{0.035, 0.05, 0.10}; // N·m/rad，roll/pitch/yaw
  Eigen::Vector3d attitude_kd{0.028, 0.06, 0.08}; // N·m/(rad/s)
  double max_force_N = 1.5;
  Eigen::Vector3d max_torque_Nm{0.006, 0.025, 0.025};
  double arrival_radius_m = 0.025;
  double arrival_speed_mps = 0.015;
  double arrival_dwell_s = 0.75;
  double reacquire_radius_m = 0.05;
  double mission_timeout_s = 90.0;
};

struct TrackingOutput
{
  double force_x_N = 0.0;
  Eigen::Vector3d torque_B_Nm = Eigen::Vector3d::Zero();
  ReferenceSample reference;
  double heading_error_rad = 0.0;
  double distance_xy_m = 0.0;
  double cross_track_m = 0.0;
  bool limited = false;
  TrackingPhase phase = TrackingPhase::IDLE;
};

class PlanarTracker
{
public:
  explicit PlanarTracker(const PlanarConfig &config);
  // 更新已校验的参数，保留目标、阶段和计时。增益/限幅下个周期生效；
  // 已生成的轨迹保存自己的时间律，运动限制在下一次 planFrom 时使用。
  void setConfig(const PlanarConfig &config);
  // 只改目标/阶段，不直接发送。移动中换目标先制动，再重新转向规划。
  // 同一个目标重复发布不重置进度；错误目标保持原任务不变。
  Status setGoal(const Eigen::Vector3d &goal_W, const RobotState &state);
  // 每个有效控制周期调用。输入最新世界状态、ROS时间步(s)，输出Fx[N]/力矩[N·m]。
  // 内部推进制动/转向/跟踪/保持阶段及计时；非法dt、计算失败或任务超时返回错误。
  Result<TrackingOutput> update(const RobotState &state, double dt_s);
  void reset(); // 清任务回 IDLE，失效恢复由 ROS 节点显式管理
  TrackingPhase phase() const
  {
    return phase_;
  }
  const Eigen::Vector3d &goal() const
  {
    return goal_W_;
  }
  const std::optional<QuinticTrajectory> &trajectory() const
  {
    return trajectory_;
  }
  double elapsed() const
  {
    return elapsed_s_;
  }
  static Status validateConfig(const PlanarConfig &config);

private:
  Status planFrom(const RobotState &state); // 从当前近静止状态创建水平直线与五次时间律
  PlanarConfig config_;
  TrackingPhase phase_ = TrackingPhase::IDLE;
  Eigen::Vector3d goal_W_ = Eigen::Vector3d::Zero();
  std::optional<QuinticTrajectory> trajectory_;
  double heading_rad_ = 0.0; // 当前直线航向；BRAKING期间先保存制动开始时的航向
  double elapsed_s_ = 0.0;   // 当前五次轨迹参考进度，大航向偏差时暂停推进
  double mission_s_ = 0.0;   // 本次接近任务累计时间；HOLDING不触发任务超时
  double stable_s_ = 0.0;    // 各阶段进入条件连续满足的时间，越界即清零
};
} // namespace rov_control_core
#endif
