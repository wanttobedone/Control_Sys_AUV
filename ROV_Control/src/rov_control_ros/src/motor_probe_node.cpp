#include "rov_control_ros/motor_probe_node.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <stdexcept>

namespace rov_control_ros
{
using namespace std::chrono_literals;

MotorProbeNode::MotorProbeNode(const rclcpp::NodeOptions &options) : Node("motor_probe", options)
{
  loadConfig();
  auto qos = rclcpp::SensorDataQoS().keep_last(1);
  commandPub_ = create_publisher<Motors>(commandTopic_, qos);
  diagnosticPub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", 10);
  statusSub_ = create_subscription<Px4Status>(
      get_parameter("vehicle_status_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&MotorProbeNode::onPx4Status, this, std::placeholders::_1));
  outputSub_ = create_subscription<Px4Motors>(
      motorsTopic_, qos, std::bind(&MotorProbeNode::onBridgeOutput, this, std::placeholders::_1));
  armClient_ = create_client<SetBool>(get_parameter("bridge_arm_service").as_string());
  stopClient_ = create_client<Trigger>(get_parameter("bridge_stop_service").as_string());
  startService_ =
      create_service<Trigger>("~/start", std::bind(&MotorProbeNode::onStart, this,
                                                   std::placeholders::_1, std::placeholders::_2));
  stopService_ =
      create_service<Trigger>("~/stop", std::bind(&MotorProbeNode::onStop, this,
                                                  std::placeholders::_1, std::placeholders::_2));
  phaseStart_ = lastTick_ = nextGraphCheck_ = nextStopRequest_ = Clock::now();
  const auto period = std::chrono::duration<double>(1.0 / config_.output_rate_hz);
  timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                             std::bind(&MotorProbeNode::onTimer, this));
  RCLCPP_INFO(get_logger(), "逐电机开环试验：输入通道 %d，幅值 %.3f，总脉冲 %.2f s；等待 start",
              config_.channel, config_.amplitude, 2.0 * config_.ramp_s + config_.hold_s);
}

void MotorProbeNode::loadConfig()
{
  // 试验参数按启动快照使用，避免一次脉冲中途改变含义。改 YAML 后需重启。
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  // ROS整数参数实际为int64，先检查再转换，防止超大整数截断后误选另一电机。
  const auto channel = declare_parameter<int64_t>("channel", 0, descriptor);
  if (channel < 0 || channel >= 8)
  {
    throw std::invalid_argument("channel 必须是 ActuatorControls 的数组下标 0..7");
  }
  config_.channel = static_cast<int>(channel);
  config_.amplitude = declare_parameter<double>("amplitude", 0.0, descriptor);
  config_.ramp_s = declare_parameter<double>("ramp_s", 0.5, descriptor);
  config_.hold_s = declare_parameter<double>("hold_s", 0.5, descriptor);
  config_.output_rate_hz = declare_parameter<double>("output_rate_hz", 50.0, descriptor);
  config_.arm_timeout_s = declare_parameter<double>("arm_timeout_s", 25.0, descriptor);
  config_.bridge_output_timeout_s =
      declare_parameter<double>("bridge_output_timeout_s", 0.3, descriptor);
  commandTopic_ = declare_parameter<std::string>("command_topic", "thruster_command", descriptor);
  motorsTopic_ = declare_parameter<std::string>("actuator_motors_topic", "/fmu/in/actuator_motors",
                                                descriptor);
  declare_parameter<std::string>("vehicle_status_topic", "/fmu/out/vehicle_status_v1", descriptor);
  declare_parameter<std::string>("bridge_arm_service", "px4_actuator_bridge/arm", descriptor);
  declare_parameter<std::string>("bridge_stop_service", "px4_actuator_bridge/stop", descriptor);
  if (get_parameter("use_sim_time").as_bool())
  {
    throw std::invalid_argument("motor_probe 使用真实时间，use_sim_time 必须为 false");
  }
  if (!std::isfinite(config_.amplitude) || std::abs(config_.amplitude) > 1.0)
  {
    throw std::invalid_argument("amplitude 必须是 [-1,1] 内的有限归一化数值");
  }
  const auto positive = [](double value) { return std::isfinite(value) && value > 0.0; };
  if (!positive(config_.ramp_s) || !std::isfinite(config_.hold_s) || config_.hold_s < 0.0 ||
      !std::isfinite(2.0 * config_.ramp_s + config_.hold_s))
  {
    throw std::invalid_argument("ramp_s 必须 > 0，hold_s 必须 >= 0，总时长必须有限");
  }
  if (!positive(config_.output_rate_hz) || config_.output_rate_hz > 400.0 ||
      !positive(config_.arm_timeout_s) || !positive(config_.bridge_output_timeout_s) ||
      1.0 / config_.output_rate_hz >= config_.bridge_output_timeout_s)
  {
    throw std::invalid_argument("频率必须在 (0,400] Hz，超时为正且输出周期小于桥输出超时");
  }
}

std::array<double, 8> MotorProbeNode::makeCommand(const MotorProbeConfig &config, double elapsed_s)
{
  std::array<double, 8> command{};
  const double total_s = 2.0 * config.ramp_s + config.hold_s;
  if (elapsed_s <= 0.0 || elapsed_s >= total_s)
  {
    return command;
  }
  // 梯形脉冲：幅值为负时自动得到反向试验，不另外改变电机映射符号。
  double ratio = 1.0;
  if (elapsed_s < config.ramp_s)
  {
    ratio = elapsed_s / config.ramp_s;
  }
  else if (elapsed_s > config.ramp_s + config.hold_s)
  {
    ratio = (total_s - elapsed_s) / config.ramp_s;
  }
  command[static_cast<std::size_t>(config.channel)] = config.amplitude * ratio;
  return command;
}

bool MotorProbeNode::checkCommandSources()
{
  // command 只能有本节点；PX4 电机输入只能有一个 bridge。
  // 日志/绘图等订阅者数量不受限制；订阅者存在不能证明电机实际执行。
  commandSourceUnique_ = count_publishers(commandTopic_) == 1;
  return commandSourceUnique_ && count_publishers(motorsTopic_) == 1 &&
         commandPub_->get_subscription_count() > 0;
}

bool MotorProbeNode::isArmed() const
{
  return statusReceived_ && px4Status_.arming_state == Px4Status::ARMING_STATE_ARMED;
}

bool MotorProbeNode::isOffboard() const
{
  return statusReceived_ && px4Status_.nav_state == Px4Status::NAVIGATION_STATE_OFFBOARD;
}

bool MotorProbeNode::hasFreshBridgeOutput(Clock::time_point time) const
{
  return lastOutputTime_ && std::chrono::duration<double>(time - *lastOutputTime_).count() <
                                config_.bridge_output_timeout_s;
}

void MotorProbeNode::onPx4Status(Px4Status::ConstSharedPtr message)
{
  px4Status_ = *message;
  statusReceived_ = true;
  lastStatusTime_ = Clock::now();
}

void MotorProbeNode::onBridgeOutput(Px4Motors::ConstSharedPtr message)
{
  outputAllNan_ = std::all_of(message->control.begin(), message->control.end(),
                              [](float value) { return std::isnan(value); });
  lastOutputTime_ = Clock::now();
  ++outputSequence_;
}

void MotorProbeNode::onStart(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  if (shuttingDown_ || (phase_ != MotorProbePhase::IDLE && phase_ != MotorProbePhase::FINISHED))
  {
    response->message = "当前阶段不允许 start；FAULT 需确认上锁并重启节点";
    return;
  }
  graphOk_ = checkCommandSources();
  if (!graphOk_ || !statusReceived_ || isArmed() || px4Status_.failsafe ||
      !armClient_->service_is_ready() || !stopClient_->service_is_ready())
  {
    response->message = "需要唯一命令源/桥、PX4 已上锁且无 failsafe、arm/stop 服务可用";
    return;
  }
  // 上游桥以消息接收时间判定新鲜度；先发零，再异步请求解锁，等待时持续发零。
  publishCommand({});
  ++runId_;
  armAccepted_ = false;
  stopAccepted_ = false;
  faultAfterStop_ = false;
  stopReason_.clear();
  elapsed_s_ = 0.0;
  setPhase(MotorProbePhase::ARMING, "解锁流程开始；等待 PX4 Armed/Offboard 与桥输出");
  auto request = std::make_shared<SetBool::Request>();
  request->data = true;
  const auto run = runId_;
  auto pending = armClient_->async_send_request(
      request,
      [this, run](rclcpp::Client<SetBool>::SharedFuture future) { onArmReply(run, future); });
  armRequest_ = pending.request_id;
  response->success = true;
  response->message = "已发起试验 " + std::to_string(runId_) + "，尚未开始非零脉冲";
}

void MotorProbeNode::onArmReply(std::uint64_t runId, rclcpp::Client<SetBool>::SharedFuture future)
{
  if (runId != runId_ || phase_ != MotorProbePhase::ARMING)
  {
    return;
  }
  armRequest_.reset();
  const auto response = future.get();
  if (!response->success)
  {
    beginStop("bridge 拒绝解锁：" + response->message, true);
    return;
  }
  armAccepted_ = true; // 仅接受服务；不能在这里就开始计脉冲时间。
}

void MotorProbeNode::onStop(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  beginStop("用户请求停止", false);
  response->success = true;
  response->message = "已归零并请求 bridge stop；以 FINISHED/FAULT 和 PX4 上锁确认结果";
}

void MotorProbeNode::beginStop(const std::string &reason, bool fault)
{
  faultAfterStop_ = faultAfterStop_ || fault || phase_ == MotorProbePhase::FAULT;
  if (phase_ == MotorProbePhase::STOPPING)
  {
    return;
  }
  if (armRequest_)
  {
    // 清理未完成的客户端请求，避免未来超时请求无限累积；服务器仍可能收到旧请求。
    armClient_->remove_pending_request(*armRequest_);
    armRequest_.reset();
  }
  outputSequenceAtStop_ = outputSequence_;
  stopReason_ = reason;
  stopAccepted_ = false;
  setPhase(MotorProbePhase::STOPPING, reason);
  // 出现其他命令源时不与它反复竞争发送，直接通过桥 stop 停止执行。
  if (commandSourceUnique_)
  {
    publishCommand({});
  }
  requestBridgeStop();
}

void MotorProbeNode::requestBridgeStop()
{
  nextStopRequest_ = Clock::now() + 1s;
  if (stopRequest_)
  {
    stopClient_->remove_pending_request(*stopRequest_);
    stopRequest_.reset();
  }
  if (!stopClient_->service_is_ready())
  {
    return; // 保持 STOPPING，每秒重试；没有确认时不能宣布完成。
  }
  const auto run = runId_;
  auto pending =
      stopClient_->async_send_request(std::make_shared<Trigger::Request>(),
                                      [this, run](rclcpp::Client<Trigger>::SharedFuture future)
                                      {
                                        if (run == runId_ && phase_ == MotorProbePhase::STOPPING)
                                        {
                                          stopAccepted_ = future.get()->success;
                                          stopRequest_.reset();
                                        }
                                      });
  stopRequest_ = pending.request_id;
}

void MotorProbeNode::onTimer()
{
  const auto time = Clock::now();
  timerGap_s_ = std::chrono::duration<double>(time - lastTick_).count();
  lastTick_ = time;
  const bool report = time >= nextGraphCheck_;
  if (report)
  {
    graphOk_ = checkCommandSources();
    nextGraphCheck_ = time + 200ms;
  }
  if (phase_ == MotorProbePhase::ARMING || phase_ == MotorProbePhase::RUNNING)
  {
    if (!graphOk_ || !statusReceived_ || px4Status_.failsafe ||
        timerGap_s_ >= config_.bridge_output_timeout_s)
    {
      beginStop("命令源冲突、PX4 failsafe 或节点调度间隔超限", true);
    }
  }
  if (phase_ == MotorProbePhase::ARMING)
  {
    if (std::chrono::duration<double>(time - phaseStart_).count() >= config_.arm_timeout_s)
    {
      beginStop("等待解锁完成超时", true);
    }
    else if (armAccepted_ && isArmed() && isOffboard() && hasFreshBridgeOutput(time) &&
             !outputAllNan_)
    {
      // bridge 内部可能还差一个 20ms tick 才从 WAITING_FOR_ARMED 到 ACTIVE；
      // 此处先发送波形的 t=0 零帧。两者状态不同，不声称读取了桥的内部枚举。
      setPhase(MotorProbePhase::RUNNING, "PX4 已 Armed/Offboard，开始有限单电机脉冲");
    }
  }
  std::array<double, 8> command{};
  if (phase_ == MotorProbePhase::RUNNING)
  {
    if (!isArmed() || !isOffboard() || !hasFreshBridgeOutput(time) || outputAllNan_)
    {
      beginStop("运行期间 PX4 退出控制模式或 bridge 输出中断", true);
    }
    else
    {
      elapsed_s_ = std::max(0.0, std::chrono::duration<double>(time - phaseStart_).count());
      if (elapsed_s_ >= 2.0 * config_.ramp_s + config_.hold_s)
      {
        beginStop("有限脉冲结束，等待上锁确认", false);
      }
      else
      {
        command = makeCommand(config_, elapsed_s_);
      }
    }
  }
  if (phase_ == MotorProbePhase::STOPPING)
  {
    if (stopAccepted_ && statusReceived_ && !isArmed() && outputAllNan_ &&
        outputSequence_ > outputSequenceAtStop_)
    {
      setPhase(faultAfterStop_ ? MotorProbePhase::FAULT : MotorProbePhase::FINISHED,
               faultAfterStop_ ? "异常试验已确认上锁；检查原因并重启后再试" : "试验已确认上锁");
    }
    else if (time >= nextStopRequest_)
    {
      requestBridgeStop();
    }
  }
  // 防止取消 start 后迟到的解锁请求重新生效；不会自动开始第二次脉冲。
  if ((phase_ == MotorProbePhase::FINISHED || phase_ == MotorProbePhase::FAULT) && isArmed())
  {
    beginStop("结束后检测到迟到/外部解锁，重新请求上锁", true);
  }
  if (commandSourceUnique_)
  {
    publishCommand(command); // 非 RUNNING 阶段 command 保持全零。
  }
  if (report)
  {
    publishDiagnostics();
  }
}

void MotorProbeNode::publishCommand(const std::array<double, 8> &command)
{
  Motors message;
  message.header.stamp = now();
  message.control = command;
  commandPub_->publish(message);
  lastCommand_ = command;
}

void MotorProbeNode::setPhase(MotorProbePhase phase, const std::string &reason)
{
  phase_ = phase;
  reason_ = reason;
  phaseStart_ = Clock::now();
  RCLCPP_INFO(get_logger(), "%s：%s", phaseName(phase_), reason.c_str());
  publishDiagnostics();
}

const char *MotorProbeNode::phaseName(MotorProbePhase phase)
{
  switch (phase)
  {
  case MotorProbePhase::IDLE:
    return "IDLE";
  case MotorProbePhase::ARMING:
    return "ARMING";
  case MotorProbePhase::RUNNING:
    return "RUNNING";
  case MotorProbePhase::STOPPING:
    return "STOPPING";
  case MotorProbePhase::FINISHED:
    return "FINISHED";
  case MotorProbePhase::FAULT:
    return "FAULT";
  }
  return "UNKNOWN";
}

void MotorProbeNode::publishDiagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_fully_qualified_name();
  status.hardware_id = "PX4 motor command path (not motor feedback)";
  status.level = faultAfterStop_ ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                                 : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = reason_;
  const auto add = [&status](const std::string &key, const std::string &value)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(item);
  };
  const auto time = Clock::now();
  add("phase", phaseName(phase_));
  add("stop_reason", stopReason_);
  add("run_id", std::to_string(runId_));
  add("channel", std::to_string(config_.channel));
  add("amplitude", std::to_string(config_.amplitude));
  add("command", std::to_string(lastCommand_[static_cast<std::size_t>(config_.channel)]));
  add("elapsed_s", std::to_string(elapsed_s_));
  add("timer_gap_s", std::to_string(timerGap_s_));
  add("graph_ok", graphOk_ ? "true" : "false");
  add("arming_state", std::to_string(px4Status_.arming_state));
  add("nav_state", std::to_string(px4Status_.nav_state));
  add("px4_failsafe", px4Status_.failsafe ? "true" : "false");
  add("status_received", statusReceived_ ? "true" : "false");
  add("status_age_s_informational",
      lastStatusTime_
          ? std::to_string(std::chrono::duration<double>(time - *lastStatusTime_).count())
          : "unknown");
  add("bridge_output_age_s",
      lastOutputTime_
          ? std::to_string(std::chrono::duration<double>(time - *lastOutputTime_).count())
          : "unknown");
  add("stop_accepted", stopAccepted_ ? "true" : "false");
  message.status.push_back(status);
  diagnosticPub_->publish(message);
}

void MotorProbeNode::prepareShutdown()
{
  shuttingDown_ = true;
  if (phase_ == MotorProbePhase::ARMING || phase_ == MotorProbePhase::RUNNING)
  {
    beginStop("节点退出", false);
  }
}

bool MotorProbeNode::stopConfirmed() const
{
  return phase_ == MotorProbePhase::IDLE || phase_ == MotorProbePhase::FINISHED ||
         phase_ == MotorProbePhase::FAULT;
}
} // namespace rov_control_ros
