#include "rov_control_ros/hippo_command.h"
#include "rov_control_core/planning/workspace_bounds.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace rov_control_ros
{
using namespace rov_control_core;

double probeDuration(const ProbeConfig &config)
{
  return config.baseline_s + config.rise_s + config.hold_s + config.fall_s + config.observe_s;
}

const char *probeChannelName(ProbeChannel channel)
{
  switch (channel)
  {
  case ProbeChannel::SURGE:
    return "surge";
  case ProbeChannel::ROLL:
    return "roll";
  case ProbeChannel::PITCH:
    return "pitch";
  case ProbeChannel::YAW:
    return "yaw";
  }
  return "invalid";
}

Status validateProbeConfig(const ProbeConfig &config)
{
  if (std::string(probeChannelName(config.channel)) == "invalid" || config.worldFrame.empty() ||
      config.bodyFrame.empty())
  {
    return {ErrorCode::INVALID_ARGUMENT, "通道或坐标名非法"};
  }
  // 有限幅值不是合法配置的充分条件：时间、空间和检查阈值也必须明确。
  for (double value :
       {config.effort_limit, config.baseline_s, config.rise_s, config.hold_s, config.fall_s,
        config.observe_s, config.timer_period_s, config.max_state_age_s, config.max_receive_gap_s,
        config.future_tolerance_s, config.max_ros_step_s, config.zero_repeat_s,
        config.max_start_speed_mps, config.max_start_rate_radps, config.max_start_tilt_rad,
        config.max_displacement_m, config.max_rotation_rad, config.max_speed_mps,
        config.max_rate_radps})
  {
    if (!std::isfinite(value) || value <= 0.0)
    {
      return {ErrorCode::INVALID_LIMITS, "试验时间和限制必须为有限正数"};
    }
  }
  if (!std::isfinite(config.amplitude) || std::abs(config.amplitude) > config.effort_limit)
  {
    return {ErrorCode::INVALID_LIMITS,
            "amplitude=" + std::to_string(config.amplitude) +
                " 的绝对值必须不超过 effort_limit=" + std::to_string(config.effort_limit)};
  }
  // 保留归一化消息接口的范围；0.65、10秒等试验选值只留在默认配置中。
  if (config.effort_limit > 1.0)
  {
    return {ErrorCode::INVALID_LIMITS, "effort_limit 必须位于(0,1]，对应归一化输入接口"};
  }
  if (!std::isfinite(probeDuration(config)))
  {
    return {ErrorCode::INVALID_LIMITS, "脉冲总时长溢出"};
  }
  return validateBounds(config.bounds);
}

Result<HippoEffortCommand> samplePulse(double elapsed, const ProbeConfig &config)
{
  // config在启动入口校验且本次试验期间不变，无需每20ms重验同一份配置。
  if (!std::isfinite(elapsed) || elapsed < 0.0)
  {
    return Result<HippoEffortCommand>::failure(ErrorCode::INVALID_SAMPLING,
                                               "脉冲时间必须为非负有限秒数");
  }
  // 全零基线和末端观察都不产生输入。这里逐段计算比例，便于对应时间图。
  const double pulseTime = elapsed - config.baseline_s;
  double fraction = 0.0;
  if (pulseTime >= 0.0 && pulseTime < config.rise_s)
  {
    fraction = pulseTime / config.rise_s;
  }
  else if (pulseTime >= config.rise_s && pulseTime < config.rise_s + config.hold_s)
  {
    fraction = 1.0;
  }
  else if (pulseTime >= config.rise_s + config.hold_s &&
           pulseTime < config.rise_s + config.hold_s + config.fall_s)
  {
    fraction = 1.0 - (pulseTime - config.rise_s - config.hold_s) / config.fall_s;
  }
  const double effort = config.amplitude * std::clamp(fraction, 0.0, 1.0);
  HippoEffortCommand command;
  switch (config.channel)
  {
  case ProbeChannel::SURGE:
    command.surge_effort = effort;
    break;
  case ProbeChannel::ROLL:
    command.torque_effort_B.x() = effort;
    break;
  case ProbeChannel::PITCH:
    command.torque_effort_B.y() = effort;
    break;
  case ProbeChannel::YAW:
    command.torque_effort_B.z() = effort;
    break;
  }
  return Result<HippoEffortCommand>::success(command);
}

Result<ActuatorMessagePair> makeActuatorMessages(const HippoEffortCommand &command,
                                                 const rclcpp::Time &stamp,
                                                 const std::string &bodyFrame, double limit)
{
  if (!std::isfinite(limit) || limit <= 0.0 || limit > 1.0 || bodyFrame.empty() ||
      stamp.nanoseconds() < 0 ||
      stamp.nanoseconds() / 1000000000LL > std::numeric_limits<std::int32_t>::max() ||
      !std::isfinite(command.surge_effort) || !command.torque_effort_B.allFinite() ||
      std::abs(command.surge_effort) > limit ||
      command.torque_effort_B.cwiseAbs().maxCoeff() > limit)
  {
    return Result<ActuatorMessagePair>::failure(ErrorCode::INVALID_ARGUMENT,
                                                "发送命令、时间、坐标或幅值非法");
  }
  ActuatorMessagePair messages;
  messages.thrust.header.stamp = stamp;
  messages.thrust.header.frame_id = bodyFrame;
  messages.torque.header = messages.thrust.header;
  // 必须清零未使用的通道；ignore=true 会让 mixer 保留上一份值。
  messages.thrust.ignore_x = messages.thrust.ignore_y = messages.thrust.ignore_z = false;
  messages.torque.ignore_x = messages.torque.ignore_y = messages.torque.ignore_z = false;
  messages.thrust.x = command.surge_effort;
  messages.thrust.y = 0.0;
  messages.thrust.z = 0.0;
  messages.torque.x = command.torque_effort_B.x();
  messages.torque.y = command.torque_effort_B.y();
  messages.torque.z = command.torque_effort_B.z();
  return Result<ActuatorMessagePair>::success(messages);
}

Status checkProbeState(const RobotState &state, const RobotState *startState,
                       const ProbeTiming &timing, const ProbeConfig &config)
{
  if (!state.p_W.allFinite() || !state.v_W.allFinite() || !state.omega_B.allFinite() ||
      !state.q_WB.coeffs().allFinite() ||
      std::abs(state.q_WB.norm() - 1.0) > QUATERNION_NORM_TOLERANCE)
  {
    return {ErrorCode::NON_FINITE, "试验状态数值或姿态非法"};
  }
  for (double value :
       {timing.ros_step_s, timing.state_age_s, timing.receive_gap_s, timing.clock_stall_s})
  {
    if (!std::isfinite(value))
    {
      return {ErrorCode::INVALID_ARGUMENT, "时序检查量不是有限数值"};
    }
  }
  if (timing.clock_went_back || timing.ros_step_s < 0.0)
  {
    return {ErrorCode::INVALID_ARGUMENT, "ROS 时间倒退，需要显式复位"};
  }
  if (timing.ros_step_s > config.max_ros_step_s)
  {
    return {ErrorCode::INVALID_ARGUMENT, "ROS 时间单次增量超限"};
  }
  if (timing.state_age_s > config.max_state_age_s ||
      timing.state_age_s < -config.future_tolerance_s || timing.receive_gap_s < 0.0 ||
      timing.clock_stall_s < 0.0 || timing.receive_gap_s > config.max_receive_gap_s ||
      timing.clock_stall_s > config.max_receive_gap_s)
  {
    return {ErrorCode::INVALID_ARGUMENT, "状态过期、来自未来或仿真时钟停止"};
  }
  if (!contains(config.bounds, state.p_W))
  {
    return {ErrorCode::OUTSIDE_WORKSPACE, "机器人离开试验活动盒"};
  }
  if (state.v_W.norm() > config.max_speed_mps || state.omega_B.norm() > config.max_rate_radps)
  {
    return {ErrorCode::INVALID_LIMITS, "试验线速度或角速度超限"};
  }
  if (startState != nullptr)
  {
    if (!startState->p_W.allFinite() || !startState->q_WB.coeffs().allFinite() ||
        std::abs(startState->q_WB.norm() - 1.0) > QUATERNION_NORM_TOLERANCE)
    {
      return {ErrorCode::INVALID_ARGUMENT, "试验起始状态非法"};
    }
    // q 和 -q 表示相同旋转。取 |dot| 得最短旋转角，避免欧拉角跨 ±pi 跳变。
    const double cosineHalf =
        std::clamp(std::abs(startState->q_WB.normalized().dot(state.q_WB.normalized())), 0.0, 1.0);
    const double rotation = 2.0 * std::acos(cosineHalf);
    if ((state.p_W - startState->p_W).norm() > config.max_displacement_m ||
        rotation > config.max_rotation_rad)
    {
      return {ErrorCode::INVALID_LIMITS, "相对起始点的位移或转角超限"};
    }
  }
  return {};
}
} // namespace rov_control_ros
