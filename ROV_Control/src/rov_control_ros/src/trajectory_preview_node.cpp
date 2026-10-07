#include "rov_control_ros/trajectory_preview_node.h"

#include "rov_control_core/planning/workspace_bounds.h"
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>

namespace rov_control_ros
{
using namespace rov_control_core;

Result<QuinticTrajectory> createTrajectory(const RobotState &state, const PreviewConfig &config)
{
  // 纯函数也检查四元数，使直接调用的测试/后续工具不能跳过 StateAdapter。
  const double quaternionNorm = state.q_WB.coeffs().stableNorm();
  if (!state.q_WB.coeffs().allFinite() || !std::isfinite(quaternionNorm) ||
      std::abs(quaternionNorm - 1.0) > QUATERNION_NORM_TOLERANCE)
  {
    return Result<QuinticTrajectory>::failure(ErrorCode::INVALID_QUATERNION, "规划起始姿态非法");
  }
  if (!std::isfinite(config.length_m) || config.length_m <= MIN_PATH_LENGTH_M)
  {
    return Result<QuinticTrajectory>::failure(ErrorCode::INVALID_ARGUMENT, "预览长度必须为正");
  }

  // 1. 机头 +x 转到世界系；2. 去掉 z；3. 归一化。
  // yaw=90° 时 d=(0,1,0)，所以目标应沿世界 +y，而不是固定沿世界 +x。
  const Eigen::Quaterniond attitude = state.q_WB.normalized();
  Eigen::Vector3d direction = attitude * Eigen::Vector3d::UnitX();
  direction.z() = 0.0;
  const double horizontalNorm = direction.norm();
  if (horizontalNorm <= 1e-6)
  {
    return Result<QuinticTrajectory>::failure(ErrorCode::UNSUPPORTED_DIRECTION,
                                              "机头接近竖直，无法定义水平预览方向");
  }
  direction /= horizontalNorm;

  PlanRequest request;
  request.p_start_W = state.p_W;
  request.p_goal_W = state.p_W + config.length_m * direction;
  request.bounds = config.bounds;
  // forward_start 不设置：允许被动漂移时查看参考，不表示当前状态可以执行。
  const auto path = LinePathPlanner::plan(request);
  if (!path.ok())
  {
    return Result<QuinticTrajectory>::failure(path.status().code, path.status().message);
  }
  return QuinticTrajectory::create(path.value(), config.limits, attitude);
}

Result<nav_msgs::msg::Path> makePathMessage(const QuinticTrajectory &trajectory,
                                            const rclcpp::Time &epoch, std::size_t count,
                                            const std::string &frame)
{
  if (count < 2 || count > 10001 || frame.empty() || epoch.nanoseconds() < 0)
  {
    return Result<nav_msgs::msg::Path>::failure(ErrorCode::INVALID_ARGUMENT,
                                                "Path 点数、坐标名或起始时间非法");
  }
  // builtin_interfaces/Time 的 sec 是 int32。先用 long double 检查加法范围，
  // 再取整到 ns；不要先转换巨大 duration 后才检查，此时已经可能溢出。
  constexpr std::int64_t maxStampNs =
      static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()) * 1000000000LL +
      999999999LL;
  const long double durationNs = static_cast<long double>(trajectory.duration()) * 1e9L;
  const long double endNs = static_cast<long double>(epoch.nanoseconds()) + durationNs;
  if (!std::isfinite(durationNs) || endNs > static_cast<long double>(maxStampNs) - 0.5L)
  {
    return Result<nav_msgs::msg::Path>::failure(ErrorCode::NUMERICAL_RANGE,
                                                "轨迹超出 ROS 消息时间戳范围");
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  path.header.stamp = epoch;
  path.poses.reserve(count);
  for (std::size_t i = 0; i < count; ++i)
  {
    const double fraction = static_cast<double>(i) / static_cast<double>(count - 1);
    const double time = trajectory.duration() * fraction;
    const auto result = trajectory.sample(time);
    if (!result.ok())
    {
      return Result<nav_msgs::msg::Path>::failure(result.status().code, result.status().message);
    }
    const auto &sample = result.value();
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame;
    const auto offsetNs =
        static_cast<std::int64_t>(std::llround(static_cast<long double>(time) * 1e9L));
    pose.header.stamp = rclcpp::Time(epoch.nanoseconds() + offsetNs, epoch.get_clock_type());
    pose.pose.position.x = sample.p_W.x();
    pose.pose.position.y = sample.p_W.y();
    pose.pose.position.z = sample.p_W.z();
    pose.pose.orientation.w = sample.q_WB_des.w();
    pose.pose.orientation.x = sample.q_WB_des.x();
    pose.pose.orientation.y = sample.q_WB_des.y();
    pose.pose.orientation.z = sample.q_WB_des.z();
    path.poses.push_back(pose);
  }
  return Result<nav_msgs::msg::Path>::success(path);
}

TrajectoryPreviewNode::TrajectoryPreviewNode(const rclcpp::NodeOptions &options)
    : Node("trajectory_preview", options), nextDiagnosticSteady_(SteadyClock::now())
{
  config_ = loadConfig();
  adapter_ = std::make_unique<StateAdapter>(config_.worldFrame, config_.bodyFrame);
  using std::placeholders::_1;
  using std::placeholders::_2;

  // 订阅使用相对名称，随 vehicle namespace 隔离；~ 表示本节点私有名称。
  // best-effort 请求可接可靠或 best-effort 状态发布者，队列只保存最近一份。
  odometrySub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&TrajectoryPreviewNode::onOdometry, this, _1));
  pathPub_ = create_publisher<nav_msgs::msg::Path>("~/planned_path",
                                                   rclcpp::QoS(1).reliable().transient_local());
  referencePub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("~/reference_pose", rclcpp::QoS(1));
  diagnosticPub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", rclcpp::QoS(10));
  startService_ =
      create_service<Trigger>("~/start", std::bind(&TrajectoryPreviewNode::onStart, this, _1, _2));
  stopService_ =
      create_service<Trigger>("~/stop", std::bind(&TrajectoryPreviewNode::onStop, this, _1, _2));
  resetService_ = create_service<Trigger>(
      "~/reset_fault", std::bind(&TrajectoryPreviewNode::onResetFault, this, _1, _2));
  timer_ = create_wall_timer(std::chrono::duration<double>(config_.timer_period_s),
                             std::bind(&TrajectoryPreviewNode::onTimer, this));

  // 所有参数已经读取；拒绝运行中修改，避免参数服务器显示值与实际 config 不一致。
  parameterGuard_ = add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> &)
      {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "本批参数仅在启动时加载，请修改 YAML 后重启节点";
        return result;
      });
  RCLCPP_INFO(get_logger(), "只读预览节点已启动：等待状态，start 服务生成参考");
}

PreviewConfig TrajectoryPreviewNode::loadConfig()
{
  PreviewConfig config;
  config.worldFrame = declare_parameter<std::string>("world_frame", "map");
  config.bodyFrame = declare_parameter<std::string>("body_frame", "uuv00/base_link");
  config.length_m = declare_parameter<double>("length_m", 0.2);
  config.limits.v_max_mps = declare_parameter<double>("v_max_mps", 0.08);
  config.limits.a_max_mps2 = declare_parameter<double>("a_max_mps2", 0.1);
  config.limits.min_duration_s = declare_parameter<double>("min_duration_s", 0.1);
  config.timer_period_s = declare_parameter<double>("timer_period_s", 0.02);
  config.max_state_age_s = declare_parameter<double>("max_state_age_s", 0.25);
  config.future_tolerance_s = declare_parameter<double>("future_tolerance_s", 0.05);
  config.max_receive_gap_s = declare_parameter<double>("max_receive_gap_s", 2.0);
  const auto count = declare_parameter<std::int64_t>("path_point_count", 101);
  const auto minimum = declare_parameter<std::vector<double>>("workspace_min_W", {0.8, 0.5, -1.0});
  const auto maximum = declare_parameter<std::vector<double>>("workspace_max_W", {1.8, 1.7, 0.0});

  if (config.worldFrame.empty() || config.bodyFrame.empty() || minimum.size() != 3 ||
      maximum.size() != 3 || count < 2 || count > 10001)
  {
    throw std::invalid_argument("frame、三维空间数组或显示点数非法");
  }
  config.pathPointCount = static_cast<std::size_t>(count);
  for (std::size_t i = 0; i < 3; ++i)
  {
    config.bounds.min_W[static_cast<Eigen::Index>(i)] = minimum[i];
    config.bounds.max_W[static_cast<Eigen::Index>(i)] = maximum[i];
  }
  const auto boundsStatus = validateBounds(config.bounds);
  const auto duration = QuinticTrajectory::chooseDuration(config.length_m, config.limits);
  if (!boundsStatus.ok() || !duration.ok() || config.length_m <= MIN_PATH_LENGTH_M)
  {
    throw std::invalid_argument("工作空间、预览长度或运动学限制非法");
  }
  // 具体试验频率属于配置，ROS负责检查duration是否能表示。
  if (!std::isfinite(config.timer_period_s) || config.timer_period_s <= 0.0)
  {
    throw std::invalid_argument("timer_period_s 必须为有限正秒数");
  }
  for (const double value :
       {config.max_state_age_s, config.future_tolerance_s, config.max_receive_gap_s})
  {
    if (!std::isfinite(value) || value <= 0.0)
    {
      throw std::invalid_argument("消息时效阈值必须是有限正数");
    }
  }
  return config;
}

void TrajectoryPreviewNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  // 故障锁存：新状态到达不自动恢复。用户 reset_fault 后再接收新的时间段。
  if (status_ == PreviewStatus::FAULT)
  {
    return;
  }
  const auto result = adapter_->convert(*message);
  if (!result.ok())
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "丢弃Odometry：%s",
                         result.status().message.c_str());
    return;
  }
  latestState_ = result.value();
  lastReceiveSteady_ = SteadyClock::now();
  // READY 由 timer 在核验时间后给出，收到一条消息不等于它仍新鲜。
}

Status TrajectoryPreviewNode::isStateFresh(const rclcpp::Time &rosNow,
                                           SteadyClock::time_point steadyNow) const
{
  if (!latestState_ || !lastReceiveSteady_)
  {
    return {ErrorCode::INVALID_ARGUMENT, "尚无有效状态"};
  }
  if (lastRosTime_ && rosNow < *lastRosTime_)
  {
    return {ErrorCode::INVALID_ARGUMENT, "ROS 时间倒退，请复位"};
  }
  const double rosAge = static_cast<double>(rosNow.nanoseconds() - latestState_->stamp_ns) * 1e-9;
  const double receiveAge = std::chrono::duration<double>(steadyNow - *lastReceiveSteady_).count();
  if (rosAge < -config_.future_tolerance_s)
  {
    return {ErrorCode::INVALID_ARGUMENT, "状态时间明显晚于 ROS 时钟"};
  }
  if (rosAge > config_.max_state_age_s)
  {
    return {ErrorCode::INVALID_ARGUMENT, "状态的 ROS 时间年龄超限"};
  }
  if (receiveAge > config_.max_receive_gap_s)
  {
    return {ErrorCode::INVALID_ARGUMENT, "有效状态接收中断或仿真暂停过久"};
  }
  return {};
}

void TrajectoryPreviewNode::onStart(const Trigger::Request::SharedPtr,
                                    Trigger::Response::SharedPtr response)
{
  if (status_ != PreviewStatus::READY && status_ != PreviewStatus::FINISHED)
  {
    response->success = false;
    response->message = "仅 READY/FINISHED 可开始；FAULT 先 reset_fault，运行中可先 stop";
    return;
  }
  const auto rosNow = now();
  const auto freshness = isStateFresh(rosNow, SteadyClock::now());
  if (!freshness.ok())
  {
    clearRun(PreviewStatus::FAULT, freshness.message);
    response->message = freshness.message;
    return;
  }
  const auto result = createTrajectory(*latestState_, config_);
  if (!result.ok())
  {
    // 规划被拒绝不破坏已经存在的显示；用户可调整场景后再尝试。
    response->message = result.status().message;
    return;
  }
  const auto path =
      makePathMessage(result.value(), rosNow, config_.pathPointCount, config_.worldFrame);
  if (!path.ok())
  {
    response->message = path.status().message;
    return;
  }
  // 所有步骤成功后一次性提交。以后机器人漂移不会移动这个轨迹的起点。
  trajectory_ = result.value();
  t0_ = rosNow;
  lastReference_.reset();
  status_ = PreviewStatus::PREVIEW;
  reason_ = "参考预览中；未输出控制命令";
  pathPub_->publish(path.value());
  publishDiagnostics();
  response->success = true;
  response->message = "已生成预览；总时间 " + std::to_string(trajectory_->duration()) + " s";
}

void TrajectoryPreviewNode::onStop(const Trigger::Request::SharedPtr,
                                   Trigger::Response::SharedPtr response)
{
  // stop 不能绕过故障锁存；这与 reset_fault 的职责分开。
  if (status_ == PreviewStatus::FAULT)
  {
    response->message = "当前 FAULT，请使用 reset_fault";
    return;
  }
  const auto fresh = isStateFresh(now(), SteadyClock::now());
  const auto next = fresh.ok() ? PreviewStatus::READY : PreviewStatus::WAIT_STATE;
  if (!fresh.ok())
  {
    // stop 返回“等待”时丢弃过期快照，否则下一次 timer 仍会检查旧消息。
    // 保留适配器的单调时间历史；只有 reset_fault 才允许换一个时间段。
    latestState_.reset();
    lastReceiveSteady_.reset();
  }
  clearRun(next, "已停止参考预览");
  response->success = true;
  response->message = reason_;
}

void TrajectoryPreviewNode::onResetFault(const Trigger::Request::SharedPtr,
                                         Trigger::Response::SharedPtr response)
{
  // 也允许主动放弃当前预览并重新等状态，绝不在 reset 后自动 start。
  latestState_.reset();
  lastReceiveSteady_.reset();
  lastRosTime_.reset();
  adapter_->reset();
  clearRun(PreviewStatus::WAIT_STATE, "已复位，等待新的有效状态");
  response->success = true;
  response->message = reason_;
}

void TrajectoryPreviewNode::onTimer()
{
  const auto rosNow = now();
  const auto steadyNow = SteadyClock::now();
  if (status_ != PreviewStatus::FAULT)
  {
    if (lastRosTime_ && rosNow < *lastRosTime_)
    {
      clearRun(PreviewStatus::FAULT, "ROS 时间倒退，请复位");
    }
    else if (latestState_)
    {
      const auto fresh = isStateFresh(rosNow, steadyNow);
      if (!fresh.ok())
      {
        clearRun(PreviewStatus::FAULT, fresh.message);
      }
      else if (status_ == PreviewStatus::WAIT_STATE)
      {
        status_ = PreviewStatus::READY;
        reason_ = "状态有效，可调用 start 预览";
        publishDiagnostics();
      }
    }
  }
  lastRosTime_ = rosNow;

  if (status_ == PreviewStatus::PREVIEW && trajectory_ && t0_)
  {
    // Wall timer 在暂停时仍会触发；这里的 elapsed 只由 ROS 时间决定。
    // 到达终点后发布一次终点，FINISHED 保留路径/最后参考供诊断和 RViz 查看。
    const double elapsed = (rosNow - *t0_).seconds();
    const auto result = trajectory_->sample(elapsed);
    if (!result.ok())
    {
      clearRun(PreviewStatus::FAULT, result.status().message);
    }
    else
    {
      lastReference_ = result.value();
      publishReference(*lastReference_, rosNow);
      if (elapsed >= trajectory_->duration())
      {
        status_ = PreviewStatus::FINISHED;
        reason_ = "参考播放完成；不表示机器人到达目标";
        publishDiagnostics();
      }
    }
  }
  if (steadyNow >= nextDiagnosticSteady_)
  {
    publishDiagnostics();
  }
}

void TrajectoryPreviewNode::publishReference(const ReferenceSample &sample,
                                             const rclcpp::Time &stamp)
{
  geometry_msgs::msg::PoseStamped message;
  message.header.frame_id = config_.worldFrame;
  message.header.stamp = stamp;
  message.pose.position.x = sample.p_W.x();
  message.pose.position.y = sample.p_W.y();
  message.pose.position.z = sample.p_W.z();
  message.pose.orientation.w = sample.q_WB_des.w();
  message.pose.orientation.x = sample.q_WB_des.x();
  message.pose.orientation.y = sample.q_WB_des.y();
  message.pose.orientation.z = sample.q_WB_des.z();
  referencePub_->publish(message);
}

void TrajectoryPreviewNode::publishDiagnostics()
{
  // 1 Hz 常态输出 + 状态变化立即输出。诊断携带参考速度/加速度，Pose 本身没有它们。
  const auto rosNow = now();
  const auto steadyNow = SteadyClock::now();
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = rosNow;
  diagnostic_msgs::msg::DiagnosticStatus diagnostic;
  diagnostic.name = std::string(get_fully_qualified_name()) + "/preview";
  diagnostic.hardware_id = "simulation_reference_only";
  diagnostic.message = reason_;
  diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  if (status_ == PreviewStatus::WAIT_STATE)
  {
    diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }
  if (status_ == PreviewStatus::FAULT)
  {
    diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  std::string stateName;
  switch (status_)
  {
  case PreviewStatus::WAIT_STATE:
    stateName = "WAIT_STATE";
    break;
  case PreviewStatus::READY:
    stateName = "READY";
    break;
  case PreviewStatus::PREVIEW:
    stateName = "PREVIEW";
    break;
  case PreviewStatus::FINISHED:
    stateName = "FINISHED";
    break;
  case PreviewStatus::FAULT:
    stateName = "FAULT";
    break;
  }
  // 局部添加函数只用于消息填充，无隐藏状态机或控制计算。
  const auto add = [&diagnostic](const std::string &key, const std::string &value)
  {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = key;
    entry.value = value;
    diagnostic.values.push_back(entry);
  };
  add("state", stateName);
  add("control_output", "none");
  if (latestState_ && lastReceiveSteady_)
  {
    const double age = static_cast<double>(rosNow.nanoseconds() - latestState_->stamp_ns) * 1e-9;
    add("state_ros_age_s", std::to_string(age));
    add("state_receive_age_s",
        std::to_string(std::chrono::duration<double>(steadyNow - *lastReceiveSteady_).count()));
  }
  if (trajectory_ && t0_)
  {
    add("duration_s", std::to_string(trajectory_->duration()));
    add("elapsed_ros_s", std::to_string((rosNow - *t0_).seconds()));
    if (latestState_)
    {
      const auto progress = projectProgress(trajectory_->path(), *latestState_);
      if (progress.ok())
      {
        add("actual_s_m", std::to_string(progress.value().s_m));
        add("actual_v_s_mps", std::to_string(progress.value().v_s_mps));
        add("cross_track_m", std::to_string(progress.value().cross_track_W.norm()));
        add("depth_offset_m", std::to_string(progress.value().depth_offset_m));
      }
    }
  }
  if (lastReference_)
  {
    add("reference_s_m", std::to_string(lastReference_->s_m));
    add("reference_v_s_mps", std::to_string(lastReference_->v_s_mps));
    add("reference_a_s_mps2", std::to_string(lastReference_->a_s_mps2));
  }
  array.status.push_back(diagnostic);
  diagnosticPub_->publish(array);
  nextDiagnosticSteady_ = steadyNow + std::chrono::seconds(1);
  if (lastLoggedStatus_ != status_)
  {
    RCLCPP_INFO(get_logger(), "%s：%s", stateName.c_str(), reason_.c_str());
    lastLoggedStatus_ = status_;
  }
}

void TrajectoryPreviewNode::clearRun(PreviewStatus nextStatus, const std::string &reason)
{
  trajectory_.reset();
  t0_.reset();
  lastReference_.reset();
  status_ = nextStatus;
  reason_ = reason;
  // 撤销 transient-local 中的旧路径。Pose 显示可能保留上一箭头，
  // 是否仍有效应以 diagnostics 为准，不能以 RViz 上有箭头判断节点在运行。
  nav_msgs::msg::Path emptyPath;
  emptyPath.header.frame_id = config_.worldFrame;
  emptyPath.header.stamp = now();
  pathPub_->publish(emptyPath);
  publishDiagnostics();
}
} // namespace rov_control_ros
