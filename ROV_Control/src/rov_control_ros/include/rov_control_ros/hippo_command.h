/**
 * Hippo 有限输入试验的计算与消息边界。
 *
 * samplePulse 产生无量纲 effort，makeActuatorMessages 转成已有 mixer 消息。
 * 这里不发布消息、不读取时钟、不改变状态机；调用方提供全部时间和状态。
 * 仅用于当前 Hippo 仿真，不能把 effort 当成牛顿或真实 ROV 的油门标定。
 */
#ifndef ROV_CONTROL_ROS_HIPPO_COMMAND_H_
#define ROV_CONTROL_ROS_HIPPO_COMMAND_H_

#include "rov_control_core/types.h"
#include <rclcpp/time.hpp>
#include <string>
#include <ustrov_control_msgs/msg/actuator_setpoint.hpp>

namespace rov_control_ros
{
enum class ProbeChannel
{
  SURGE,
  ROLL,
  PITCH,
  YAW
};

struct HippoEffortCommand
{
  // 机体 x 方向的命令域输入；正负方向须由实际模型响应核验。
  double surge_effort = 0.0;
  // 顺序为绕机体 x/y/z 的 roll/pitch/yaw 命令；单位均为无量纲，不是 N·m。
  Eigen::Vector3d torque_effort_B = Eigen::Vector3d::Zero();
};

struct ProbeConfig
{
  std::string worldFrame = "map";
  std::string bodyFrame = "uuv00/base_link";
  ProbeChannel channel = ProbeChannel::SURGE;
  // 默认全零试验：启动后仍需显式调用 start；非零幅值在 launch 中明确指定。
  double amplitude = 0.0;
  double effort_limit = 0.65;
  // 下列五段使用 ROS 秒数，拼成一次有限梯形脉冲和前后零输入观察段。
  double baseline_s = 1.0;
  double rise_s = 0.1;
  // 实测 0.3 s 在 pitch/yaw 触发角速度保护，缩短保持段而不放宽保护阈值。
  double hold_s = 0.1;
  double fall_s = 0.1;
  double observe_s = 1.0;
  double timer_period_s = 0.02;     // wall timer 名义周期，不等于实测实时性保证
  double max_state_age_s = 0.10;    // ROS now 与 odometry 时间戳允许的最大差
  double max_receive_gap_s = 0.15;  // steady 时钟计算的接收/仿真时钟停顿上限
  double future_tolerance_s = 0.05; // 消息领先当前 ROS 时间时允许的同步容差
  double max_ros_step_s = 0.10;     // 两次定时回调观察到的最大正 ROS 时间增量
  double zero_repeat_s = 0.50;      // 故障或结束后用 steady 时间限制零命令补发
  double max_start_speed_mps = 0.03;
  double max_start_rate_radps = 0.10;
  double max_start_tilt_rad = 0.0872664626; // 5°，接口统一使用 rad
  double max_displacement_m = 0.10;
  double max_rotation_rad = 0.1745329252; // 10°，相对试验起始姿态
  double max_speed_mps = 0.20;
  double max_rate_radps = 0.50;
  rov_control_core::WorkspaceBounds bounds{Eigen::Vector3d(0.8, 0.5, -1.0),
                                           Eigen::Vector3d(1.8, 1.7, 0.0)};
};

struct ProbeTiming
{
  double ros_step_s = 0.0;    // 本次 ROS now - 上次回调 ROS now
  double state_age_s = 0.0;   // ROS now - 最新 odometry stamp，可在容差内为负
  double receive_gap_s = 0.0; // steady now - 最近一次有效 odometry 接收时刻
  double clock_stall_s = 0.0; // steady now - 最近一次观察到 ROS 时间前进的时刻
  bool clock_went_back = false;
};

struct ActuatorMessagePair
{
  ustrov_control_msgs::msg::ActuatorSetpoint thrust;
  ustrov_control_msgs::msg::ActuatorSetpoint torque;
};

// 纯校验：合法时返回成功 Status，非法时给原因；不裁剪用户配置。
rov_control_core::Status validateProbeConfig(const ProbeConfig &config);
// 返回五段 ROS 时长之和；调用方须先用 validateProbeConfig 保证配置合法。
double probeDuration(const ProbeConfig &config);
// 将通道枚举转为诊断中的固定名称；非法枚举返回 invalid。
const char *probeChannelName(ProbeChannel channel);
// 输入相对 start 的 ROS 秒数，输出唯一指定通道上的梯形脉冲，其余分量全零。
// config须在启动入口经validateProbeConfig校验；采样期间不修改。
// 内部无历史状态；结束后的任意时间均返回全零，负时间/非有限值返回失败。
rov_control_core::Result<HippoEffortCommand> samplePulse(double elapsed, const ProbeConfig &config);
// 发送边界：检查命令、时间戳、frame 和幅值，将所有轴/ignore 标志显式赋值。
// 输出只是两条消息的数据对象；调用者负责 publish，异常输入不生成可用消息。
rov_control_core::Result<ActuatorMessagePair>
makeActuatorMessages(const HippoEffortCommand &command, const rclcpp::Time &stamp,
                     const std::string &bodyFrame, double limit);
// 输入统一世界系状态、可选初态、时序差和已校验配置；返回是否允许继续。
// startState=nullptr 时只做当前状态/时序检查；有初态时还检查本次运动范围。
// 最短相对转角使用 2*acos(|q0·q|)，正确处理四元数 q 与 -q 的等价性。
// 该函数不读取时钟、不修改状态机；节点根据失败原因决定归零和故障锁存。
rov_control_core::Status checkProbeState(const rov_control_core::RobotState &state,
                                         const rov_control_core::RobotState *startState,
                                         const ProbeTiming &timing, const ProbeConfig &config);
} // namespace rov_control_ros
#endif
