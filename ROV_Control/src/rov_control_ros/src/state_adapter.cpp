#include "rov_control_ros/state_adapter.h"
#include <cmath>

namespace rov_control_ros
{
using rov_control_core::ErrorCode;
using rov_control_core::Result;
using rov_control_core::RobotState;

StateAdapter::StateAdapter(const std::string &worldFrame, const std::string &bodyFrame)
    : worldFrame_(worldFrame), bodyFrame_(bodyFrame)
{
}

Result<RobotState> StateAdapter::convert(const nav_msgs::msg::Odometry &message)
{
  // 先检查语义，防止数值合法但坐标含义不同的消息进入核心。
  if (message.header.frame_id != worldFrame_ || message.child_frame_id != bodyFrame_)
  {
    return Result<RobotState>::failure(ErrorCode::INVALID_ARGUMENT, "Odometry 坐标名不匹配");
  }
  const auto &stamp = message.header.stamp;
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U)
  {
    return Result<RobotState>::failure(ErrorCode::INVALID_ARGUMENT, "消息时间戳格式非法");
  }
  const std::int64_t stampNs = static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
                               static_cast<std::int64_t>(stamp.nanosec);
  if (lastStampNs_ && stampNs <= *lastStampNs_)
  {
    return Result<RobotState>::failure(ErrorCode::INVALID_ARGUMENT, "消息时间重复或乱序");
  }

  RobotState state;
  state.stamp_ns = stampNs;
  const auto &pose = message.pose.pose;
  const auto &twist = message.twist.twist;
  state.p_W = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  // Eigen 构造顺序是 w,x,y,z；ROS 消息字段排列是 x,y,z,w，不能直接复制数组。
  state.q_WB = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                  pose.orientation.z);
  const Eigen::Vector3d velocityB(twist.linear.x, twist.linear.y, twist.linear.z);
  state.omega_B = Eigen::Vector3d(twist.angular.x, twist.angular.y, twist.angular.z);
  if (!state.p_W.allFinite() || !state.q_WB.coeffs().allFinite() || !velocityB.allFinite() ||
      !state.omega_B.allFinite())
  {
    return Result<RobotState>::failure(ErrorCode::NON_FINITE, "状态包含 NaN 或 Inf");
  }
  const double norm = state.q_WB.coeffs().stableNorm();
  if (std::abs(norm - 1.0) > rov_control_core::QUATERNION_NORM_TOLERANCE)
  {
    return Result<RobotState>::failure(ErrorCode::INVALID_QUATERNION, "状态四元数不是单位四元数");
  }
  // 仅修正舍入级模长误差；不能把任意坏四元数归一化后当有效测量。
  state.q_WB.normalize();
  state.v_W = state.q_WB * velocityB;
  if (!state.v_W.allFinite())
  {
    return Result<RobotState>::failure(ErrorCode::NUMERICAL_RANGE, "速度旋转数值溢出");
  }
  lastStampNs_ = stampNs;
  return Result<RobotState>::success(state);
}

void StateAdapter::reset()
{
  lastStampNs_.reset();
}
} // namespace rov_control_ros
