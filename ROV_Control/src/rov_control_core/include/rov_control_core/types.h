/**
 * 路径规划与轨迹生成使用的数据结构。
 *
 * 数据流：PlanRequest → LinePath → QuinticTrajectory → ReferenceSample。
 * 反馈侧：RobotState + LinePath → ProgressState。
 *
 * 坐标约定：W 为世界系 map，B 为机体系 base_link。
 * q_WB 把机体系向量旋转到世界系：v_W = q_WB * v_B。
 * 长度用 m，时间用 s，角度用 rad；世界系 z 向上，深度通常向下为正。
 * 字段名中的 W/B 和单位后缀用于明确物理含义，避免混用坐标或单位。
 *
 * 本层不订阅 ROS、不读取时钟；坐标转换和时间对齐由后续适配层负责。
 */

#ifndef ROV_CONTROL_CORE_TYPES_H_
#define ROV_CONTROL_CORE_TYPES_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <optional>

#include "rov_control_core/result.h"

namespace rov_control_core
{

// 一次状态快照。这里保存实际状态，与后面的期望状态 ReferenceSample 区分。
struct RobotState
{
  // 状态时间戳 [ns]，时钟来源由适配层统一
  std::int64_t stamp_ns = 0;
  // 实际世界系位置 [m]
  Eigen::Vector3d p_W = Eigen::Vector3d::Zero();
  // 实际机体系到世界系的旋转
  Eigen::Quaterniond q_WB = Eigen::Quaterniond::Identity();
  // 实际世界系线速度 [m/s]，不是机体系速度
  Eigen::Vector3d v_W = Eigen::Vector3d::Zero();
  // 实际机体系角速度 [rad/s]
  Eigen::Vector3d omega_B = Eigen::Vector3d::Zero();
};

// 机器人中心允许进入的轴对齐盒子。
// 传入前已经扣除机体尺寸、挂载和余量，不能直接填入水池外轮廓。
struct WorkspaceBounds
{
  // 各轴允许的最小坐标 [m]
  Eigen::Vector3d min_W = Eigen::Vector3d::Zero();
  // 各轴允许的最大坐标 [m]
  Eigen::Vector3d max_W = Eigen::Vector3d::Zero();
  // 默认 min=max，属于非法边界，要求调用者先明确设置工作空间。
};

// 约束的是参考轨迹的运动学量，尚不包含推进器推力或机器人动力学约束。
struct MotionLimits
{
  // 最大沿线速度 [m/s]，必须大于零
  double v_max_mps = 0.0;
  // 最大沿线加速度绝对值 [m/s²]，必须大于零
  double a_max_mps2 = 0.0;
  // 可选 jerk 限制 [m/s³]；不设置则不参与时间计算
  std::optional<double> j_max_mps3;
  // 最短运动时间 [s]，必须大于零
  double min_duration_s = 0.0;
};

// 首轮 Hippo 前向直线实验的启动条件。
// 纯几何规划可以不启用；后续执行节点必须根据当前状态与配置填入。
struct ForwardStartConstraint
{
  // 起始姿态
  Eigen::Quaterniond q_WB = Eigen::Quaterniond::Identity();
  // 起始世界系速度 [m/s]
  Eigen::Vector3d v_W = Eigen::Vector3d::Zero();
  // 允许的起始速度大小 [m/s]
  double max_start_speed_mps = 0.0;
  // 机体 z 轴偏离世界系 z 轴的最大角度 [rad]
  double max_tilt_rad = 0.0;
  // 路径与当前水平朝向的最大夹角 [rad]
  double max_heading_error_rad = 0.0;
};

// 调用 plan() 时一次性提交的规划请求，不包含 ROS 消息或话题名。
struct PlanRequest
{
  // 路径起点 [m]
  Eigen::Vector3d p_start_W = Eigen::Vector3d::Zero();
  // 路径终点 [m]
  Eigen::Vector3d p_goal_W = Eigen::Vector3d::Zero();
  // 已留余量的工作空间
  WorkspaceBounds bounds;
  // 是否检查首轮前向执行条件
  std::optional<ForwardStartConstraint> forward_start;
};

// sample(t) 返回的单个参考点。位置、速度和加速度对应同一个时刻。
struct ReferenceSample
{
  // 调用者请求的相对时间 [s]，超出 [0,T] 也保留该时间
  double t_from_start_s = 0.0;
  // 从起点算起的参考沿线距离 [m]
  double s_m = 0.0;
  // 参考沿线速度 ds/dt [m/s]
  double v_s_mps = 0.0;
  // 参考沿线加速度 d²s/dt² [m/s²]
  double a_s_mps2 = 0.0;
  // 世界系参考位置 p0 + d*s [m]
  Eigen::Vector3d p_W = Eigen::Vector3d::Zero();
  // 世界系参考速度 d*v_s [m/s]
  Eigen::Vector3d v_W = Eigen::Vector3d::Zero();
  // 世界系参考加速度 d*a_s [m/s²]
  Eigen::Vector3d a_W = Eigen::Vector3d::Zero();
  // 参考姿态，首版沿轨迹保持不变
  Eigen::Quaterniond q_WB_des = Eigen::Quaterniond::Identity();
  // 参考机体系角速度，首版为零
  Eigen::Vector3d omega_B_des = Eigen::Vector3d::Zero();
};

// projectProgress() 的输出，描述实际状态相对于几何直线的位置。
struct ProgressState
{
  // 实际沿线距离 [m]，不裁剪，负值和超过终点的值都保留
  double s_m = 0.0;
  // 实际速度沿路径方向的投影 [m/s]
  double v_s_mps = 0.0;
  // 与路径垂直的三维位置残差 [m]
  Eigen::Vector3d cross_track_W = Eigen::Vector3d::Zero();
  // 相对路径投影点的深度偏差 [m]，向下为正，不是绝对深度
  double depth_offset_m = 0.0;
};

// 下面是浮点计算容差，不是硬件限位，也不是实验误差的验收标准。
// 小于等于此长度视为无需运动
constexpr double MIN_PATH_LENGTH_M = 1e-9;
// 只容许这么小的单位四元数模长误差
constexpr double QUATERNION_NORM_TOLERANCE = 1e-6;

}  // 命名空间 rov_control_core

#endif
