/**
 * C1 单通道执行试验节点：由 start 服务触发一次有限脉冲。
 *
 * odometry → StateAdapter → 状态检查；ROS 时间 → samplePulse → 两类 setpoint。
 * wall timer 在仿真暂停时仍能终止输出。所有业务回调使用同一个单线程 executor。
 * 节点不接 PX4、不 arm、不做 PD/MPC；mixer 输出仅是命令观测，不是推力反馈。
 */
#ifndef ROV_CONTROL_ROS_ACTUATION_PROBE_NODE_H_
#define ROV_CONTROL_ROS_ACTUATION_PROBE_NODE_H_

#include "rov_control_ros/hippo_command.h"
#include "rov_control_ros/state_adapter.h"
#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ustrov_control_msgs/msg/actuator_controls.hpp>

namespace rov_control_ros
{
enum class ProbeStatus
{
  WAIT_STATE,
  READY,
  RUNNING,
  FINISHED,
  FAULT
};

class ActuationProbeNode : public rclcpp::Node
{
public:
  explicit ActuationProbeNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  // main 在 ROS context 仍有效时调用，并继续 spin 一个有界时间以补发零命令。
  void prepareShutdown();

private:
  using SteadyClock = std::chrono::steady_clock;
  using Trigger = std_srvs::srv::Trigger;
  using RobotState = rov_control_core::RobotState;
  using Status = rov_control_core::Status;
  using MotorMessage = ustrov_control_msgs::msg::ActuatorControls;
  using Setpoint = ustrov_control_msgs::msg::ActuatorSetpoint;

  // 构造期间读取 YAML/launch 参数，返回已校验配置；错误抛出异常，节点不进入运行。
  ProbeConfig loadConfig();
  // 查询本命名空间的命令发布端点和 mixer，返回检查结果；不改变 ROS 图。
  // onStart 立即检查，onTimer 每 100 ms 检查；DDS 图发现有延迟，不能代替互斥锁。
  Status checkCommandSources() const;
  // 输入当前 ROS/steady 时间，结合成员里的上次时间和消息时间，生成各项时序差。
  // 此函数只计算，不推进 lastRosTime_；onTimer 末尾才更新该历史值。
  ProbeTiming makeTiming(const rclcpp::Time &rosNow, SteadyClock::time_point steadyNow) const;
  // 校验最新状态；starting=true 时额外要求初始速度、角速度和倾斜足够小。
  // RUNNING 时传入 startState_ 检查相对运动，等待状态时不检查相对位移。
  Status checkCurrentState(const ProbeTiming &timing, bool starting) const;
  // ROS 订阅回调：检查 frame/四元数/时间并转换 v_B→v_W，成功才更新状态快照。
  // 非法消息使试验进入 FAULT；FAULT 期间不靠下一条正常消息自动解除故障。
  void onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr message);
  // 缓存上游实际发布的电机命令，记录 steady 接收时间并确认全零基线。
  // 检查数值、时间戳、当前单通道混控范围；它不计算或测量推进器推力。
  void onThrusterCommand(MotorMessage::ConstSharedPtr message);
  // Trigger 请求无业务字段；成功返回 success=true 并保存 startState_/t0_/runId_。
  // 只启动一次配置好的试验，重复 start 不打断当前运行，也不绕过 FAULT。
  void onStart(Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response);
  // 主动停止：调用 stopOutput 发送有限全零序列；已经 FAULT 时保持故障锁存。
  void onStop(Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response);
  // 归零结束后清除旧状态和时间历史，重置适配器，再等待新状态；不自动开始脉冲。
  void onResetFault(Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response);
  // wall timer 主流程：读取两种时间→检查状态/来源→按 ROS 时间采样→发送→诊断。
  // /clock 停止时仍会被 executor 调用，以 steady 时间检测超时和补发零命令。
  void onTimer();
  // 将无量纲命令封装成两类 setpoint 并 publish，成功后更新 lastCommand_/sentPairs_。
  // 返回成功只说明本地发布调用成功；两条消息不原子送达，不构成执行器确认。
  Status publishCommand(const HippoEffortCommand &command, const rclcpp::Time &stamp);
  // 统一退出入口：保存阶段/原因；本节点启动过试验才立即归零，并设置补发截止时间。
  // onTimer 在截止后停止发送，重复 stop 不延长该窗口。
  void stopOutput(ProbeStatus nextStatus, const std::string &reason);
  // 将配置、状态机、时效、已发送命令和 mixer 缓存写入 diagnostics；不写磁盘。
  // rosbag 是外部记录者；阶段变化即时发布，其余由 onTimer 限制在约 10 Hz。
  void publishDiagnostics();

  // 固定配置与状态适配器；全部业务回调串行执行，无需跨线程共享状态锁。
  ProbeConfig config_;
  std::unique_ptr<StateAdapter> adapter_;
  std::optional<RobotState> latestState_;
  std::optional<RobotState> startState_; // 每次 start 冻结，用于相对位移和转角检查
  std::optional<SteadyClock::time_point> lastReceiveSteady_;
  std::optional<rclcpp::Time> lastRosTime_;
  std::optional<rclcpp::Time> t0_; // 本次试验起点，只使用 ROS 仿真时间推进脉冲
  SteadyClock::time_point lastClockAdvanceSteady_;
  SteadyClock::time_point nextGraphCheckSteady_;
  SteadyClock::time_point nextDiagnosticSteady_;
  std::optional<SteadyClock::time_point> zeroUntilSteady_; // 有值表示仍在有限补发零命令
  bool clockAdvanced_ = false;
  bool outputOwned_ = false; // 只有通过 start 的节点才能进入自己的归零发送流程
  bool baselineConfirmed_ = false;
  bool shuttingDown_ = false;
  std::optional<MotorMessage> lastMotors_;
  std::optional<SteadyClock::time_point> lastMotorReceiveSteady_;
  HippoEffortCommand lastCommand_;
  Status graphStatus_;
  ProbeStatus status_ = ProbeStatus::WAIT_STATE;
  std::optional<ProbeStatus> lastLoggedStatus_;
  std::string reason_ = "等待有效状态与仿真时钟";
  std::uint64_t runId_ = 0;
  std::uint64_t sentPairs_ = 0;
  double maxRosStepDuringRun_s_ = 0.0; // 保存本次实测最大时间步，故障后仍可查询

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub_;
  rclcpp::Subscription<MotorMessage>::SharedPtr motorsSub_;
  rclcpp::Publisher<Setpoint>::SharedPtr thrustPub_;
  rclcpp::Publisher<Setpoint>::SharedPtr torquePub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticPub_;
  rclcpp::Service<Trigger>::SharedPtr startService_;
  rclcpp::Service<Trigger>::SharedPtr stopService_;
  rclcpp::Service<Trigger>::SharedPtr resetService_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameterGuard_;
};
} // namespace rov_control_ros
#endif
