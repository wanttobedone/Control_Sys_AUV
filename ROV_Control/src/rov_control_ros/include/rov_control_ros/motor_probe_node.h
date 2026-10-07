/**
 * 真机接入前的逐电机开环试验：预设 u(t)，复用 USTROV 的解锁/上锁桥。
 *
 * 不订阅 Odometry，不计算位置误差，不做推力分配；channel 是输入数组下标。
 * VehicleStatus 和 ActuatorMotors 只用于运行许可与通信检查，不是运动反馈。
 * 所有回调由单线程 executor 串行调用；计时使用 steady_clock，消息用 ROS 时间戳。
 */
#ifndef ROV_CONTROL_ROS_MOTOR_PROBE_NODE_H_
#define ROV_CONTROL_ROS_MOTOR_PROBE_NODE_H_

#include <array>
#include <chrono>
#include <cstdint>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <optional>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <ustrov_control_msgs/msg/actuator_controls.hpp>

namespace rov_control_ros
{
// 配置只在启动时加载；试验改变通道/幅值时先结束并重新启动节点。
// 此处给出的幅值是无量纲归一化输入，不等于牛顿、转速或 PWM 微秒。
struct MotorProbeConfig
{
  int channel = 0;        // ActuatorControls.control[0..7]
  double amplitude = 0.0; // [-1,1]；缺省全零，允许零幅值完整验链
  double ramp_s = 0.5;    // 每侧斜坡时间，必须 > 0
  double hold_s = 0.5;    // 平台时间，允许为 0
  double output_rate_hz = 50.0;
  double arm_timeout_s = 25.0;          // 覆盖桥预流及两段模式/解锁等待
  double bridge_output_timeout_s = 0.3; // ACTIVE 期间桥输出流的接收超时
};

enum class MotorProbePhase
{
  IDLE,
  ARMING,
  RUNNING,
  STOPPING,
  FINISHED,
  FAULT
};

class MotorProbeNode : public rclcpp::Node
{
public:
  explicit MotorProbeNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  // 纯计算：调用者先校验配置。e=0/T 时输出零，T=2*ramp_s+hold_s。
  // 所有未选通道始终为零；单独公开此函数用于检验波形边界和正反输入。
  static std::array<double, 8> makeCommand(const MotorProbeConfig &config, double elapsed_s);

  // main 在 ROS context 有效时调用，随后短时继续 spin，争取完成 stop 握手。
  // SIGKILL/掉电无法进入这里，依赖原桥命令超时和 PX4 自身的失联处置。
  void prepareShutdown();
  bool stopConfirmed() const;

private:
  using Clock = std::chrono::steady_clock;
  using Trigger = std_srvs::srv::Trigger;
  using SetBool = std_srvs::srv::SetBool;
  using Px4Status = px4_msgs::msg::VehicleStatus;
  using Px4Motors = px4_msgs::msg::ActuatorMotors;
  using Motors = ustrov_control_msgs::msg::ActuatorControls;

  // 声明只读参数并一次性验证范围/时长；错误时不创建执行器发布者。
  void loadConfig();
  // DDS 图只能提供发现结果，不能充当跨进程互斥锁。启动即查，运行约 5 Hz 查。
  bool checkCommandSources();
  bool isArmed() const;
  bool isOffboard() const;
  bool hasFreshBridgeOutput(Clock::time_point now) const;

  // 只缓存快照。VehicleStatus 可能低频，不将它当 50 Hz 心跳。
  void onPx4Status(Px4Status::ConstSharedPtr message);
  // 观察实际桥输出，记录到达时刻及全 NaN 上锁帧；不解释为转速/推力测量。
  void onBridgeOutput(Px4Motors::ConstSharedPtr message);
  void onStart(Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response);
  void onStop(Trigger::Request::SharedPtr request, Trigger::Response::SharedPtr response);
  // 异步回调携带试验编号，迟到的旧试验响应不能推进新试验。
  void onArmReply(std::uint64_t runId, rclcpp::Client<SetBool>::SharedFuture future);
  void onTimer();

  // 所有退出共用此路径：立即零输入，取消未处理的本地 arm future，请求桥 stop。
  // stop 服务接受、桥的 NaN 输出、PX4 已上锁三者齐全，才结束 STOPPING。
  void beginStop(const std::string &reason, bool fault);
  void requestBridgeStop();
  void publishCommand(const std::array<double, 8> &command);
  void publishDiagnostics();
  void setPhase(MotorProbePhase phase, const std::string &reason);
  static const char *phaseName(MotorProbePhase phase);

  MotorProbeConfig config_;
  MotorProbePhase phase_ = MotorProbePhase::IDLE;
  std::string reason_ = "等待 PX4 状态及显式 start；当前仅发送零命令";
  std::string commandTopic_;
  std::string motorsTopic_;
  Px4Status px4Status_{};
  bool statusReceived_ = false;
  bool outputAllNan_ = false;
  bool armAccepted_ = false;
  bool stopAccepted_ = false;
  bool faultAfterStop_ = false;
  bool shuttingDown_ = false;
  bool graphOk_ = false;
  bool commandSourceUnique_ = false;
  std::string stopReason_; // 保留首次终止原因，完成上锁后仍可从 diagnostics 读取。
  std::uint64_t runId_ = 0;
  std::uint64_t outputSequence_ = 0;
  std::uint64_t outputSequenceAtStop_ = 0;
  std::optional<int64_t> armRequest_;
  std::optional<int64_t> stopRequest_;
  std::optional<Clock::time_point> lastOutputTime_;
  std::optional<Clock::time_point> lastStatusTime_;
  Clock::time_point phaseStart_;
  Clock::time_point lastTick_;
  Clock::time_point nextGraphCheck_;
  Clock::time_point nextStopRequest_;
  std::array<double, 8> lastCommand_{};
  double elapsed_s_ = 0.0;
  double timerGap_s_ = 0.0;

  rclcpp::Publisher<Motors>::SharedPtr commandPub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticPub_;
  rclcpp::Subscription<Px4Status>::SharedPtr statusSub_;
  rclcpp::Subscription<Px4Motors>::SharedPtr outputSub_;
  rclcpp::Client<SetBool>::SharedPtr armClient_;
  rclcpp::Client<Trigger>::SharedPtr stopClient_;
  rclcpp::Service<Trigger>::SharedPtr startService_;
  rclcpp::Service<Trigger>::SharedPtr stopService_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace rov_control_ros
#endif
