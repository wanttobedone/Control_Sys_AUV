/**
 * ROS 消息与数学核心之间的状态适配器，不拥有订阅器。
 * 调用链：节点 onOdometry → convert → RobotState → 轨迹/进度计算。
 * Odometry 的 pose 属于 header.frame_id，twist 属于 child_frame_id。
 * 这里严格要求 map / vehicle/base_link，不通过改名字代替 TF 变换。
 */
#ifndef ROV_CONTROL_ROS_STATE_ADAPTER_H_
#define ROV_CONTROL_ROS_STATE_ADAPTER_H_

#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <string>
#include "rov_control_core/types.h"

namespace rov_control_ros
{
class StateAdapter
{
public:
  // 输入：期望的世界系、机体系名字；时间历史初始为空。
  StateAdapter(const std::string &worldFrame, const std::string &bodyFrame);

  // 成功才更新上一有效时间戳。重复/倒退、非法数值、未知坐标均返回失败。
  // 不使用 covariance 作融合或精度判断，本批只做接口转换。
  rov_control_core::Result<rov_control_core::RobotState> convert(
    const nav_msgs::msg::Odometry &message);

  // 显式故障复位后允许一个新的时间段；坐标约定保持不变。
  void reset();

private:
  std::string worldFrame_;
  std::string bodyFrame_;
  std::optional<std::int64_t> lastStampNs_;
};
}
#endif
