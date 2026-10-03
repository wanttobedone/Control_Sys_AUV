/**
 * B 批只读轨迹预览节点：状态 → 轨迹 → RViz/诊断。
 * 本头文件中的两个纯函数允许单元测试直接检验坐标、路径和时间转换。
 * 所有回调由单线程 executor 串行调用，不支持多线程 executor 并发访问。
 * 没有推力、力矩、推进器命令发布器，PREVIEW 不是控制使能。
 */
#ifndef ROV_CONTROL_ROS_TRAJECTORY_PREVIEW_NODE_H_
#define ROV_CONTROL_ROS_TRAJECTORY_PREVIEW_NODE_H_

#include <chrono>
#include <memory>
#include <optional>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "rov_control_core/planning/quintic_trajectory.h"
#include "rov_control_ros/state_adapter.h"

namespace rov_control_ros
{
struct PreviewConfig
{
  std::string worldFrame = "map";
  std::string bodyFrame = "uuv00/base_link";
  double length_m = 0.2;
  rov_control_core::MotionLimits limits;
  rov_control_core::WorkspaceBounds bounds;
  std::size_t pathPointCount = 101;
  double timer_period_s = 0.02;       // wall timer 检查周期，不决定参考的时间进度
  double max_state_age_s = 0.25;      // now_ros - 消息时间戳的最大值
  double future_tolerance_s = 0.05;   // /clock 和状态在不同话题到达的容差
  double max_receive_gap_s = 2.0;     // steady clock 上允许无有效消息的最长时间
};

enum class PreviewStatus
{
  WAIT_STATE,
  READY,
  PREVIEW,
  FINISHED,
  FAULT
};

// 输入：当前世界系状态、配置。输出：沿当前水平朝向的固定轨迹。
// 不检查起速/倾角的执行可行性，本函数只用于显示；C 批控制必须另加门控。
rov_control_core::Result<rov_control_core::QuinticTrajectory> createTrajectory(
  const rov_control_core::RobotState &state, const PreviewConfig &config);

// 输入：轨迹、ROS 起始时间、显示点数 [2,10001]、世界系名。
// 输出：含首尾的 Path；每个 pose 的 stamp=epoch+该点相对时间。
// 先检查 ROS 消息 sec 字段范围，避免长轨迹把时间戳截断溢出。
rov_control_core::Result<nav_msgs::msg::Path> makePathMessage(
  const rov_control_core::QuinticTrajectory &trajectory,
  const rclcpp::Time &epoch, std::size_t count, const std::string &frame);

class TrajectoryPreviewNode : public rclcpp::Node
{
public:
  explicit TrajectoryPreviewNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  using Trigger = std_srvs::srv::Trigger;
  using SteadyClock = std::chrono::steady_clock;

  // 启动时声明/读取参数并检查，非法配置抛异常使 main 返回非零。
  PreviewConfig loadConfig();
  // 消息 → 适配器 → 最新状态/steady 接收时刻；坏消息锁存 FAULT。
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr message);
  // 只读查询，两种时钟分别检查消息年龄和接收间隔，不修改状态。
  rov_control_core::Status isStateFresh(const rclcpp::Time &rosNow,
                                       SteadyClock::time_point steadyNow) const;
  // Trigger 请求为空；响应 success/message 表示生成预览成功或具体拒绝原因。
  void onStart(const Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response);
  // 清轨迹、保留有效状态；FAULT 时拒绝，不能绕过显式复位。
  void onStop(const Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response);
  // 清时间历史、快照和轨迹，进入 WAIT_STATE，不自动 start。
  void onResetFault(const Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response);
  // wall tick → 时效检查 → 按 ROS 时间采样 → 发布参考/诊断。
  void onTimer();
  // ReferenceSample → PoseStamped；速度/加速度另由诊断输出。
  void publishReference(const rov_control_core::ReferenceSample &sample, const rclcpp::Time &stamp);
  // 输出当前阶段、原因、消息年龄、实际/参考进度；同时更新下次诊断时间。
  void publishDiagnostics();
  // 清活动轨迹并发布空 Path；是否清最新状态由调用方决定。
  void clearRun(PreviewStatus nextStatus, const std::string &reason);

  PreviewConfig config_;
  std::unique_ptr<StateAdapter> adapter_;
  std::optional<rov_control_core::RobotState> latestState_;
  std::optional<SteadyClock::time_point> lastReceiveSteady_;
  std::optional<rov_control_core::QuinticTrajectory> trajectory_;
  std::optional<rclcpp::Time> t0_;
  std::optional<rclcpp::Time> lastRosTime_;
  std::optional<rov_control_core::ReferenceSample> lastReference_;
  SteadyClock::time_point nextDiagnosticSteady_;
  PreviewStatus status_ = PreviewStatus::WAIT_STATE;
  std::string reason_ = "等待有效 Odometry";
  PreviewStatus lastLoggedStatus_ = PreviewStatus::WAIT_STATE;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr referencePub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticPub_;
  rclcpp::Service<Trigger>::SharedPtr startService_;
  rclcpp::Service<Trigger>::SharedPtr stopService_;
  rclcpp::Service<Trigger>::SharedPtr resetService_;
  rclcpp::TimerBase::SharedPtr timer_;
  // 参数在启动时冻结。回调句柄必须保存，否则回调会被注销。
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameterGuard_;
};
}
#endif
