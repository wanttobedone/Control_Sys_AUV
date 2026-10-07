#include "rov_control_ros/actuation_probe_node.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

namespace rov_control_ros
{
using namespace rov_control_core;

ActuationProbeNode::ActuationProbeNode(const rclcpp::NodeOptions &options)
    : Node("actuation_probe", options), lastClockAdvanceSteady_(SteadyClock::now()),
      nextGraphCheckSteady_(SteadyClock::now()), nextDiagnosticSteady_(SteadyClock::now())
{
  config_ = loadConfig();
  adapter_ = std::make_unique<StateAdapter>(config_.worldFrame, config_.bodyFrame);
  using std::placeholders::_1;
  using std::placeholders::_2;
  // 状态只保留最新一份。setpoint 使用可靠、非持久 QoS，避免后加入订阅者重放旧脉冲。
  odometrySub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ActuationProbeNode::onOdometry, this, _1));
  motorsSub_ = create_subscription<MotorMessage>(
      "thruster_command", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ActuationProbeNode::onThrusterCommand, this, _1));
  thrustPub_ = create_publisher<Setpoint>("thrust_setpoint", rclcpp::QoS(1));
  torquePub_ = create_publisher<Setpoint>("torque_setpoint", rclcpp::QoS(1));
  diagnosticPub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", 10);
  startService_ =
      create_service<Trigger>("~/start", std::bind(&ActuationProbeNode::onStart, this, _1, _2));
  stopService_ =
      create_service<Trigger>("~/stop", std::bind(&ActuationProbeNode::onStop, this, _1, _2));
  resetService_ = create_service<Trigger>(
      "~/reset_fault", std::bind(&ActuationProbeNode::onResetFault, this, _1, _2));
  timer_ = create_wall_timer(std::chrono::duration<double>(config_.timer_period_s),
                             std::bind(&ActuationProbeNode::onTimer, this));
  parameterGuard_ = add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> &)
      {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "输入试验参数在启动时固定，修改后请重启节点";
        return result;
      });
  graphStatus_ = {ErrorCode::INVALID_ARGUMENT, "等待命令图发现"};
  RCLCPP_INFO(get_logger(), "C1 输入试验已启动：%s，幅值 %.3f；等待显式 start",
              probeChannelName(config_.channel), config_.amplitude);
}

ProbeConfig ActuationProbeNode::loadConfig()
{
  // 本节点仅接仿真。use_sim_time 只是必要条件，不能据此认证任意 ROS 图没有硬件。
  if (!get_parameter("use_sim_time").as_bool())
  {
    throw std::invalid_argument("C1 只用于 Hippo 仿真，必须设置 use_sim_time=true");
  }
  ProbeConfig config;
  config.worldFrame = declare_parameter<std::string>("world_frame", config.worldFrame);
  config.bodyFrame = declare_parameter<std::string>("body_frame", config.bodyFrame);
  const auto channel = declare_parameter<std::string>("channel", "surge");
  if (channel == "surge")
  {
    config.channel = ProbeChannel::SURGE;
  }
  else if (channel == "roll")
  {
    config.channel = ProbeChannel::ROLL;
  }
  else if (channel == "pitch")
  {
    config.channel = ProbeChannel::PITCH;
  }
  else if (channel == "yaw")
  {
    config.channel = ProbeChannel::YAW;
  }
  else
  {
    throw std::invalid_argument("channel 只能为 surge/roll/pitch/yaw");
  }

  // 每个参数逐项列出，方便从 YAML 追踪到其单位与检查位置。
  config.amplitude = declare_parameter<double>("amplitude", config.amplitude);
  config.effort_limit = declare_parameter<double>("effort_limit", config.effort_limit);
  config.baseline_s = declare_parameter<double>("baseline_s", config.baseline_s);
  config.rise_s = declare_parameter<double>("rise_s", config.rise_s);
  config.hold_s = declare_parameter<double>("hold_s", config.hold_s);
  config.fall_s = declare_parameter<double>("fall_s", config.fall_s);
  config.observe_s = declare_parameter<double>("observe_s", config.observe_s);
  config.timer_period_s = declare_parameter<double>("timer_period_s", config.timer_period_s);
  config.max_state_age_s = declare_parameter<double>("max_state_age_s", config.max_state_age_s);
  config.max_receive_gap_s =
      declare_parameter<double>("max_receive_gap_s", config.max_receive_gap_s);
  config.future_tolerance_s =
      declare_parameter<double>("future_tolerance_s", config.future_tolerance_s);
  config.max_ros_step_s = declare_parameter<double>("max_ros_step_s", config.max_ros_step_s);
  config.zero_repeat_s = declare_parameter<double>("zero_repeat_s", config.zero_repeat_s);
  config.max_start_speed_mps =
      declare_parameter<double>("max_start_speed_mps", config.max_start_speed_mps);
  config.max_start_rate_radps =
      declare_parameter<double>("max_start_rate_radps", config.max_start_rate_radps);
  config.max_start_tilt_rad =
      declare_parameter<double>("max_start_tilt_rad", config.max_start_tilt_rad);
  config.max_displacement_m =
      declare_parameter<double>("max_displacement_m", config.max_displacement_m);
  config.max_rotation_rad = declare_parameter<double>("max_rotation_rad", config.max_rotation_rad);
  config.max_speed_mps = declare_parameter<double>("max_speed_mps", config.max_speed_mps);
  config.max_rate_radps = declare_parameter<double>("max_rate_radps", config.max_rate_radps);
  const auto minimum = declare_parameter<std::vector<double>>("workspace_min_W", {0.8, 0.5, -1.0});
  const auto maximum = declare_parameter<std::vector<double>>("workspace_max_W", {1.8, 1.7, 0.0});
  if (minimum.size() != 3 || maximum.size() != 3)
  {
    throw std::invalid_argument("活动盒的 min/max 必须各有三个数");
  }
  for (std::size_t i = 0; i < 3; ++i)
  {
    config.bounds.min_W[static_cast<Eigen::Index>(i)] = minimum[i];
    config.bounds.max_W[static_cast<Eigen::Index>(i)] = maximum[i];
  }
  const auto valid = validateProbeConfig(config);
  if (!valid.ok())
  {
    throw std::invalid_argument(valid.message);
  }
  return config;
}

Status ActuationProbeNode::checkCommandSources() const
{
  // 用发布端点数量检测重复节点；仅比较名字会漏掉两个同名进程。
  for (const auto *publisher : {thrustPub_.get(), torquePub_.get()})
  {
    const auto endpoints = get_publishers_info_by_topic(publisher->get_topic_name());
    if (endpoints.size() > 1)
    {
      return {ErrorCode::INVALID_ARGUMENT, "setpoint 发布者不是本节点唯一来源"};
    }
  }
  const auto mixers = get_publishers_info_by_topic(motorsSub_->get_topic_name());
  if (mixers.size() > 1)
  {
    return {ErrorCode::INVALID_ARGUMENT, "thruster_command 存在多个发布者"};
  }
  return {};
}

ProbeTiming ActuationProbeNode::makeTiming(const rclcpp::Time &rosNow,
                                           SteadyClock::time_point steadyNow) const
{
  ProbeTiming timing;
  if (lastRosTime_)
  {
    timing.ros_step_s = (rosNow - *lastRosTime_).seconds();
    timing.clock_went_back = rosNow < *lastRosTime_;
  }
  if (latestState_)
  {
    timing.state_age_s = static_cast<double>(rosNow.nanoseconds() - latestState_->stamp_ns) * 1e-9;
  }
  if (lastReceiveSteady_)
  {
    timing.receive_gap_s = std::chrono::duration<double>(steadyNow - *lastReceiveSteady_).count();
  }
  // 当前已经看到 ROS 时间前进时，时钟停顿时间应为零，而不是上一 tick 的年龄。
  if (timing.ros_step_s <= 0.0)
  {
    timing.clock_stall_s =
        std::chrono::duration<double>(steadyNow - lastClockAdvanceSteady_).count();
  }
  return timing;
}

Status ActuationProbeNode::checkCurrentState(const ProbeTiming &timing, bool starting) const
{
  if (!latestState_ || !lastReceiveSteady_ || !clockAdvanced_)
  {
    return {ErrorCode::INVALID_ARGUMENT, "等待有效状态和前进的仿真时钟"};
  }
  const RobotState *initial =
      status_ == ProbeStatus::RUNNING && startState_ ? &*startState_ : nullptr;
  const auto checked = checkProbeState(*latestState_, initial, timing, config_);
  if (!checked.ok())
  {
    return checked;
  }
  if (starting)
  {
    const Eigen::Vector3d bodyZ = latestState_->q_WB * Eigen::Vector3d::UnitZ();
    const double tilt = std::acos(std::clamp(bodyZ.z(), -1.0, 1.0));
    if (latestState_->v_W.norm() > config_.max_start_speed_mps ||
        latestState_->omega_B.norm() > config_.max_start_rate_radps ||
        tilt > config_.max_start_tilt_rad)
    {
      return {ErrorCode::START_MOVING, "起始速度、角速度或倾斜过大，请等稳定后再 start"};
    }
  }
  return {};
}

void ActuationProbeNode::onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  if (status_ == ProbeStatus::FAULT)
  {
    return;
  }
  const auto converted = adapter_->convert(*message);
  if (!converted.ok())
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "丢弃Odometry：%s",
                         converted.status().message.c_str());
    return;
  }
  latestState_ = converted.value();
  lastReceiveSteady_ = SteadyClock::now();
}

void ActuationProbeNode::onThrusterCommand(MotorMessage::ConstSharedPtr message)
{
  bool finite = message->header.stamp.sec >= 0 && message->header.stamp.nanosec < 1000000000U;
  double peak = 0.0;
  for (double value : message->control)
  {
    finite = finite && std::isfinite(value) && std::abs(value) <= 1.0 + 1e-9;
    peak = std::max(peak, std::abs(value));
  }
  if (!finite)
  {
    if (status_ == ProbeStatus::RUNNING)
    {
      stopOutput(ProbeStatus::FAULT, "mixer 输出数值或时间戳非法");
    }
    return;
  }
  lastMotors_ = *message;
  lastMotorReceiveSteady_ = SteadyClock::now();
  // 当前模型单轴系数为 0.25。基线确认后出现更大的电机值或后四路非零，
  // 说明运行配置/命令组合与 C1 假设不符；这不能被当作推力测量来继续试验。
  if (status_ == ProbeStatus::RUNNING && baselineConfirmed_)
  {
    bool unexpected = peak > 0.25 * config_.effort_limit + 1e-8;
    for (std::size_t i = 4; i < message->control.size(); ++i)
    {
      unexpected = unexpected || std::abs(message->control[i]) > 1e-9;
    }
    if (unexpected)
    {
      stopOutput(ProbeStatus::FAULT, "mixer 输出与当前 Hippo 单通道范围不符");
      return;
    }
  }
  if (status_ == ProbeStatus::RUNNING && t0_ && (now() - *t0_).seconds() < config_.baseline_s &&
      rclcpp::Time(message->header.stamp, RCL_ROS_TIME) >= *t0_ && peak < 1e-9)
  {
    baselineConfirmed_ = true;
  }
}

void ActuationProbeNode::onStart(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  if (shuttingDown_ || zeroUntilSteady_ ||
      (status_ != ProbeStatus::READY && status_ != ProbeStatus::FINISHED))
  {
    response->message = "仅 READY/FINISHED 且归零结束后可 start；FAULT 需显式复位";
    return;
  }
  const auto rosNow = now();
  const auto checked = checkCurrentState(makeTiming(rosNow, SteadyClock::now()), true);
  graphStatus_ = checkCommandSources();
  if (!checked.ok() || !graphStatus_.ok())
  {
    response->message = !checked.ok() ? checked.message : graphStatus_.message;
    return;
  }
  startState_ = latestState_;
  t0_ = rosNow;
  baselineConfirmed_ = false;
  outputOwned_ = true;
  ++runId_;
  maxRosStepDuringRun_s_ = 0.0;
  status_ = ProbeStatus::RUNNING;
  reason_ = "有限输入试验运行中，先确认全零基线";
  const auto sent = publishCommand({}, rosNow);
  if (!sent.ok())
  {
    stopOutput(ProbeStatus::FAULT, sent.message);
    response->message = sent.message;
    return;
  }
  publishDiagnostics();
  response->success = true;
  response->message = "试验 " + std::to_string(runId_) + " 开始，通道 " +
                      probeChannelName(config_.channel) + "，幅值 " +
                      std::to_string(config_.amplitude);
}

void ActuationProbeNode::onStop(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  // stop 不解除 FAULT；重复 stop 也不无限延长已启动的归零补发。
  const auto next = status_ == ProbeStatus::FAULT ? ProbeStatus::FAULT : ProbeStatus::WAIT_STATE;
  stopOutput(next, status_ == ProbeStatus::FAULT ? reason_ : "用户停止试验，已请求归零");
  response->success = true;
  response->message = reason_;
}

void ActuationProbeNode::onResetFault(Trigger::Request::SharedPtr,
                                      Trigger::Response::SharedPtr response)
{
  if (zeroUntilSteady_ || status_ == ProbeStatus::RUNNING || shuttingDown_)
  {
    response->message = "请先 stop 并等待有限零命令补发结束";
    return;
  }
  latestState_.reset();
  startState_.reset();
  lastReceiveSteady_.reset();
  lastRosTime_.reset();
  t0_.reset();
  lastMotors_.reset();
  lastMotorReceiveSteady_.reset();
  adapter_->reset();
  clockAdvanced_ = false;
  baselineConfirmed_ = false;
  lastClockAdvanceSteady_ = SteadyClock::now();
  status_ = ProbeStatus::WAIT_STATE;
  reason_ = "已复位，等待新的有效状态；不会自动 start";
  response->success = true;
  response->message = reason_;
  publishDiagnostics();
}

void ActuationProbeNode::onTimer()
{
  const auto rosNow = now();
  const auto steadyNow = SteadyClock::now();
  const auto timing = makeTiming(rosNow, steadyNow);
  if (timing.ros_step_s > 0.0)
  {
    clockAdvanced_ = true;
    lastClockAdvanceSteady_ = steadyNow;
  }
  // 图发现不是瞬时互斥；每 100 ms 检查一次，避免每个周期扫描完整 ROS 图。
  if (steadyNow >= nextGraphCheckSteady_)
  {
    graphStatus_ = checkCommandSources();
    nextGraphCheckSteady_ = steadyNow + std::chrono::milliseconds(100);
  }

  if (status_ != ProbeStatus::FAULT && timing.clock_went_back)
  {
    stopOutput(ProbeStatus::FAULT, "ROS 时间倒退，需要显式复位");
  }
  if (status_ == ProbeStatus::RUNNING)
  {
    maxRosStepDuringRun_s_ = std::max(maxRosStepDuringRun_s_, timing.ros_step_s);
    const auto checked = checkCurrentState(timing, false);
    if (!checked.ok() || !graphStatus_.ok())
    {
      stopOutput(ProbeStatus::FAULT, !checked.ok() ? checked.message : graphStatus_.message);
    }
    else
    {
      const double elapsed = (rosNow - *t0_).seconds();
      // 只有基线已确认、mixer 输出仍有更新，才允许跨入非零阶段。
      const bool motorFresh =
          lastMotorReceiveSteady_ &&
          std::chrono::duration<double>(steadyNow - *lastMotorReceiveSteady_).count() <=
              config_.max_receive_gap_s;
      if (elapsed >= config_.baseline_s && (!baselineConfirmed_ || !motorFresh))
      {
        stopOutput(ProbeStatus::FAULT, "未确认 mixer 零基线或输出接收中断");
      }
      else if (elapsed >= probeDuration(config_))
      {
        stopOutput(ProbeStatus::FINISHED, "有限输入试验完成，已请求归零；不表示位置静止");
      }
      else if (timing.ros_step_s > 0.0)
      {
        const auto sampled = samplePulse(elapsed, config_);
        if (!sampled.ok())
        {
          stopOutput(ProbeStatus::FAULT, sampled.status().message);
        }
        else
        {
          const auto sent = publishCommand(sampled.value(), rosNow);
          if (!sent.ok())
          {
            stopOutput(ProbeStatus::FAULT, sent.message);
          }
        }
      }
    }
  }
  else if (!zeroUntilSteady_ && !shuttingDown_ && status_ != ProbeStatus::FAULT)
  {
    const auto checked = checkCurrentState(timing, false);
    if (status_ != ProbeStatus::FINISHED)
    {
      status_ = checked.ok() && graphStatus_.ok() ? ProbeStatus::READY : ProbeStatus::WAIT_STATE;
      reason_ = !checked.ok() ? checked.message
                              : (!graphStatus_.ok() ? graphStatus_.message
                                                    : "状态和命令图就绪，可显式 start");
    }
  }
  lastRosTime_ = rosNow;

  if (zeroUntilSteady_)
  {
    // 即使 /clock 暂停也发送零；0.5 s 后停止，不永久维持发送来掩盖下游超时。
    if (steadyNow < *zeroUntilSteady_)
    {
      const auto sent = publishCommand({}, rosNow);
      if (!sent.ok())
      {
        status_ = ProbeStatus::FAULT;
        reason_ = "补发零命令失败，不能确认下游已归零";
      }
    }
    else
    {
      zeroUntilSteady_.reset();
      outputOwned_ = false;
      publishDiagnostics();
    }
  }
  if (steadyNow >= nextDiagnosticSteady_ || lastLoggedStatus_ != status_)
  {
    publishDiagnostics();
  }
}

ActuationProbeNode::Status ActuationProbeNode::publishCommand(const HippoEffortCommand &command,
                                                              const rclcpp::Time &stamp)
{
  const auto messages =
      makeActuatorMessages(command, stamp, config_.bodyFrame, config_.effort_limit);
  if (!messages.ok())
  {
    return messages.status();
  }
  try
  {
    thrustPub_->publish(messages.value().thrust);
    torquePub_->publish(messages.value().torque);
    lastCommand_ = command;
    ++sentPairs_;
  }
  catch (const std::exception &)
  {
    return {ErrorCode::INVALID_ARGUMENT, "ROS 命令发布失败"};
  }
  return {};
}

void ActuationProbeNode::stopOutput(ProbeStatus nextStatus, const std::string &reason)
{
  status_ = nextStatus;
  reason_ = reason;
  if (outputOwned_)
  {
    if (!zeroUntilSteady_)
    {
      zeroUntilSteady_ =
          SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
                                   std::chrono::duration<double>(config_.zero_repeat_s));
    }
    const auto sent = publishCommand({}, now());
    if (!sent.ok())
    {
      status_ = ProbeStatus::FAULT;
      reason_ = "请求归零时发布失败；不能确认下游已收到零命令";
    }
  }
  publishDiagnostics();
}

void ActuationProbeNode::prepareShutdown()
{
  shuttingDown_ = true;
  stopOutput(status_ == ProbeStatus::FAULT ? ProbeStatus::FAULT : ProbeStatus::WAIT_STATE,
             "节点正常退出，执行有限归零补发");
}

void ActuationProbeNode::publishDiagnostics()
{
  const char *phase = "WAIT_STATE";
  switch (status_)
  {
  case ProbeStatus::WAIT_STATE:
    phase = "WAIT_STATE";
    break;
  case ProbeStatus::READY:
    phase = "READY";
    break;
  case ProbeStatus::RUNNING:
    phase = "RUNNING";
    break;
  case ProbeStatus::FINISHED:
    phase = "FINISHED";
    break;
  case ProbeStatus::FAULT:
    phase = "FAULT";
    break;
  }
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus entry;
  entry.name = std::string(get_fully_qualified_name()) + "/input_probe";
  entry.hardware_id = "hippo_simulation_only";
  entry.message = reason_;
  entry.level = status_ == ProbeStatus::FAULT ? 2 : (status_ == ProbeStatus::WAIT_STATE ? 1 : 0);
  const auto add = [&entry](const std::string &key, const std::string &value)
  {
    diagnostic_msgs::msg::KeyValue field;
    field.key = key;
    field.value = value;
    entry.values.push_back(field);
  };
  add("state", phase);
  add("run_id", std::to_string(runId_));
  add("channel", probeChannelName(config_.channel));
  add("amplitude", std::to_string(config_.amplitude));
  add("duration_s", std::to_string(probeDuration(config_)));
  add("sent_pairs", std::to_string(sentPairs_));
  add("observed_max_ros_step_s", std::to_string(maxRosStepDuringRun_s_));
  add("zero_repeat_active", zeroUntilSteady_ ? "true" : "false");
  add("baseline_confirmed", baselineConfirmed_ ? "true" : "false");
  add("sent_surge_effort", std::to_string(lastCommand_.surge_effort));
  for (Eigen::Index i = 0; i < 3; ++i)
  {
    add("sent_torque_effort_" + std::to_string(i), std::to_string(lastCommand_.torque_effort_B[i]));
  }
  if (t0_)
  {
    add("elapsed_ros_s", std::to_string((now() - *t0_).seconds()));
  }
  if (latestState_)
  {
    const auto timing = makeTiming(now(), SteadyClock::now());
    add("state_ros_age_s", std::to_string(timing.state_age_s));
    add("state_receive_gap_s", std::to_string(timing.receive_gap_s));
    if (startState_)
    {
      const Eigen::Vector3d direction = startState_->q_WB * Eigen::Vector3d::UnitX();
      add("delta_along_initial_heading_m",
          std::to_string(direction.dot(latestState_->p_W - startState_->p_W)));
      add("speed_along_initial_heading_mps", std::to_string(direction.dot(latestState_->v_W)));
    }
    for (Eigen::Index i = 0; i < 3; ++i)
    {
      add("omega_B_" + std::to_string(i) + "_radps", std::to_string(latestState_->omega_B[i]));
    }
  }
  if (lastMotors_)
  {
    for (std::size_t i = 0; i < lastMotors_->control.size(); ++i)
    {
      add("mixer_output_" + std::to_string(i), std::to_string(lastMotors_->control[i]));
    }
  }
  message.status.push_back(entry);
  diagnosticPub_->publish(message);
  nextDiagnosticSteady_ = SteadyClock::now() + std::chrono::milliseconds(100);
  if (lastLoggedStatus_ != status_)
  {
    RCLCPP_INFO(get_logger(), "%s：%s", phase, reason_.c_str());
    lastLoggedStatus_ = status_;
  }
}
} // namespace rov_control_ros
