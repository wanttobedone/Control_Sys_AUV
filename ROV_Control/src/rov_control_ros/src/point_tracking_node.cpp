#include "rov_control_ros/point_tracking_node.h"
#include "rov_control_core/planning/workspace_bounds.h"
#include "rov_control_ros/trajectory_preview_node.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <tuple>

namespace rov_control_ros
{
using namespace rov_control_core;
namespace
{
geometry_msgs::msg::Pose toPose(const Eigen::Vector3d &p, const Eigen::Quaterniond &q)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = p.x();
  pose.position.y = p.y();
  pose.position.z = p.z();
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  return pose;
}
} // namespace
PointTrackingNode::PointTrackingNode(const rclcpp::NodeOptions &options)
    : Node("point_tracking", options)
{
  // 先建立纯计算对象，再连接ROS。构造阶段不发电机命令，也不自动接收旧任务。
  config_ = loadConfig();
  tracker_ = std::make_unique<PlanarTracker>(config_);
  allocator_.setMaxCommand(limits_.max_thruster_command);
  adapter_ = std::make_unique<StateAdapter>(worldFrame_, bodyFrame_);
  using std::placeholders::_1;
  using std::placeholders::_2;
  odomSub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&PointTrackingNode::onOdometry, this, _1));
  // 目标是事件，不使用transient-local，防止节点重启重放旧任务。
  goalSub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "goal", rclcpp::QoS(1), std::bind(&PointTrackingNode::onGoal, this, _1));
  motorsPub_ = create_publisher<ustrov_control_msgs::msg::ActuatorControls>("thruster_command", 1);
  diagnosticPub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", 10);
  pathPub_ =
      create_publisher<nav_msgs::msg::Path>("~/planned_path", rclcpp::QoS(1).transient_local());
  actualPub_ =
      create_publisher<nav_msgs::msg::Path>("~/actual_path", rclcpp::QoS(1).transient_local());
  localOdomPub_ =
      create_publisher<nav_msgs::msg::Odometry>("odometry_home", rclcpp::SensorDataQoS());
  referencePub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/reference_pose", 1);
  homePub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("~/home", rclcpp::QoS(1).transient_local());
  goalPub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/accepted_goal",
                                                               rclcpp::QoS(1).transient_local());
  homeTf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
  stopService_ =
      create_service<Trigger>("~/stop", std::bind(&PointTrackingNode::onStop, this, _1, _2));
  resetService_ = create_service<Trigger>("~/reset_fault",
                                          std::bind(&PointTrackingNode::onReset, this, _1, _2));
  timer_ = create_wall_timer(std::chrono::milliseconds(20),
                             std::bind(&PointTrackingNode::onTimer, this));
  parameterValidation_ =
      add_on_set_parameters_callback(std::bind(&PointTrackingNode::onParameters, this, _1));
  parameterApplication_ =
      add_post_set_parameters_callback(std::bind(&PointTrackingNode::applyParameters, this, _1));
  logConfig();
  RCLCPP_INFO(get_logger(),
              "Hippo XY 到点节点：等待 home 和有效状态，向 goal 发布 PointStamped 开始");
}

void PointTrackingNode::publishHome()
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = worldFrame_;
  pose.header.stamp = now();
  pose.pose = toPose(home_->origin_W, home_->q_WH);
  homePub_->publish(pose);
  geometry_msgs::msg::TransformStamped tf;
  tf.header = pose.header;
  tf.child_frame_id = homeFrame_;
  tf.transform.translation.x = home_->origin_W.x();
  tf.transform.translation.y = home_->origin_W.y();
  tf.transform.translation.z = home_->origin_W.z();
  tf.transform.rotation = pose.pose.orientation;
  homeTf_->sendTransform(tf);
  RCLCPP_INFO(get_logger(), "home 已固定：世界位置 [%.3f, %.3f, %.3f]，目标 frame=%s",
              home_->origin_W.x(), home_->origin_W.y(), home_->origin_W.z(), homeFrame_.c_str());
}

void PointTrackingNode::onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  if (fault_)
  {
    return;
  }
  const auto converted = adapter_->convert(*message);
  if (!converted.ok())
  {
    // 输入边界拒绝一帧坏消息即可；只有持续缺少有效反馈才由checkState超时停机。
    // 不刷新received_，否则持续坏帧会伪装成健康的数据流。
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "丢弃Odometry：%s",
                         converted.status().message.c_str());
    return;
  }
  state_ = converted.value();
  received_ = Steady::now();
  if (!home_)
  {
    const auto home = HomeFrame::fromState(*state_);
    if (!home.ok())
    {
      stopOutput(home.status().message, true);
      return;
    }
    home_ = home.value();
    publishHome();
  }
  // 同一实际状态另发home表达；twist仍属于body，所以不额外旋转原始twist字段。
  auto local = *message;
  local.header.frame_id = homeFrame_;
  local.pose.pose = toPose(home_->toHome(state_->p_W), home_->q_WH.conjugate() * state_->q_WB);
  // 当前源为真值零协方差；真机非零协方差需完整变换，不能直接复用此显示适配。
  localOdomPub_->publish(local);
}

PointTrackingNode::Status PointTrackingNode::checkGraph() const
{
  const auto sources = get_publishers_info_by_topic(motorsPub_->get_topic_name());
  if (sources.size() > 1)
  {
    return {ErrorCode::INVALID_ARGUMENT,
            "thruster_command 存在其他发布者；请关闭原 mixer/控制节点"};
  }
  return {};
}
PointTrackingNode::Status PointTrackingNode::checkState(const rclcpp::Time &rosNow,
                                                        Steady::time_point wallNow) const
{
  if (!state_ || !home_ || !received_ || !clockSeen_)
  {
    return {ErrorCode::INVALID_ARGUMENT, "等待有效状态和前进的仿真时间"};
  }
  const double age = static_cast<double>(rosNow.nanoseconds() - state_->stamp_ns) * 1e-9;
  const double gap = std::chrono::duration<double>(wallNow - *received_).count();
  const double clockGap = std::chrono::duration<double>(wallNow - clockAdvanced_).count();
  if (age > limits_.max_state_age_s || age < -limits_.max_state_age_s ||
      gap > limits_.max_receive_gap_s || clockGap > limits_.max_receive_gap_s)
  {
    return {ErrorCode::INVALID_ARGUMENT,
            "有效状态/时钟超时：age=" + std::to_string(age) + " s, receive_gap=" +
                std::to_string(gap) + " s, clock_gap=" + std::to_string(clockGap) +
                " s；阈值 age=" + std::to_string(limits_.max_state_age_s) +
                " s, gap=" + std::to_string(limits_.max_receive_gap_s) + " s"};
  }
  if (!contains(config_.bounds, state_->p_W))
  {
    return {ErrorCode::OUTSIDE_WORKSPACE, "实际位置离开活动盒"};
  }
  const Eigen::Vector3d z = state_->q_WB * Eigen::Vector3d::UnitZ();
  const double tilt = std::acos(std::clamp(z.z(), -1.0, 1.0));
  // 0表示关闭对应运动监测；无隐藏速度、倾斜或深度的第二套数值。
  for (const auto &[name, measured, limit] :
       {std::tuple<const char *, double, double>{"max_actual_speed_mps", state_->v_W.norm(),
                                                 limits_.max_actual_speed_mps},
        {"max_tilt_rad", tilt, limits_.max_tilt_rad},
        {"max_angular_speed_radps", state_->omega_B.norm(), limits_.max_angular_speed_radps},
        {"max_depth_drift_m", std::abs(state_->p_W.z() - home_->origin_W.z()),
         limits_.max_depth_drift_m}})
  {
    if (limit > 0 && measured > limit)
    {
      return {ErrorCode::INVALID_LIMITS, std::string(name) +
                                             " 超限：实际=" + std::to_string(measured) +
                                             "，配置=" + std::to_string(limit)};
    }
  }
  return {};
}

void PointTrackingNode::onGoal(geometry_msgs::msg::PointStamped::ConstSharedPtr message)
{
  const auto reject = [this](const std::string &reason)
  {
    ++rejectedGoals_;
    reason_ = "拒绝目标：" + reason;
    RCLCPP_WARN(get_logger(), "%s", reason_.c_str());
    publishDiagnostics();
  };
  graphStatus_ = checkGraph();
  const auto checked = checkState(now(), Steady::now());
  if (shuttingDown_ || fault_ || zeroUntil_ || !checked.ok() || !graphStatus_.ok())
  {
    reject(fault_
               ? "故障锁存，需要 reset_fault"
               : (!checked.ok() ? checked.message
                                : (!graphStatus_.ok() ? graphStatus_.message : "正在归零或退出")));
    return;
  }
  const Eigen::Vector3d goal_H(message->point.x, message->point.y, message->point.z);
  if (message->header.frame_id != homeFrame_ || !goal_H.allFinite() || std::abs(goal_H.z()) > 1e-6)
  {
    reject("必须使用本实例 home frame，x/y为有限米数，z必须为0（仅水平目标）");
    return;
  }
  // goal是保持有效直到到达/替换/停止的任务，不是需要连续刷新的速度指令。
  // 因此header.stamp不参与过期判断；反馈时效由checkState独立负责。
  const Eigen::Vector3d goal_W = home_->toWorld(goal_H);
  const auto previous = tracker_->goal();
  const bool repeated =
      tracker_->phase() != TrackingPhase::IDLE && (goal_W - previous).head<2>().norm() < 1e-6;
  const auto accepted = tracker_->setGoal(goal_W, *state_);
  if (!accepted.ok())
  {
    reject(accepted.message);
    return;
  }
  if (repeated)
  {
    return;
  }
  ++goalId_;
  owned_ = true;
  pathPublished_ = false;
  maxError_m_ = errorSquares_ = maxStep_s_ = 0;
  errorSamples_ = 0;
  actualPath_.poses.clear();
  nav_msgs::msg::Path empty;
  empty.header.frame_id = worldFrame_;
  empty.header.stamp = now();
  pathPub_->publish(empty);
  geometry_msgs::msg::PoseStamped goal;
  goal.header = empty.header;
  goal.pose = toPose(tracker_->goal(), home_->q_WH);
  goalPub_->publish(goal);
  reason_ = "已接收水平目标，先制动和转向";
  RCLCPP_INFO(get_logger(), "目标 %lu：home [%.3f, %.3f] → world [%.3f, %.3f]",
              static_cast<unsigned long>(goalId_), goal_H.x(), goal_H.y(), goal_W.x(), goal_W.y());
  publishDiagnostics();
}

void PointTrackingNode::sendMotors(const std::array<double, 8> &command)
{
  ustrov_control_msgs::msg::ActuatorControls message;
  message.header.stamp = now();
  message.header.frame_id = bodyFrame_;
  for (std::size_t i = 0; i < command.size(); ++i)
  {
    if (!std::isfinite(command[i]) || std::abs(command[i]) > 1.0 + 1e-9)
    {
      throw std::runtime_error("thruster_command[" + std::to_string(i) + "]=" +
                               std::to_string(command[i]) + " 超出归一化[-1,1]接口或不是有限数");
    }
    message.control[i] = std::clamp(command[i], -1.0, 1.0);
  }
  motorsPub_->publish(message); // 一条8槽消息；bridge随后向各Gazebo推进器转发
}
void PointTrackingNode::stopOutput(const std::string &reason, bool fault)
{
  fault_ = fault_ || fault;
  reason_ = reason;
  tracker_->reset();
  // 停止后诊断中的请求力/力矩也归零，避免把上一控制周期误读为仍在发送。
  output_ = {};
  if (owned_)
  {
    if (!zeroUntil_)
    {
      zeroUntil_ = Steady::now() + std::chrono::milliseconds(500);
    }
    sendMotors({});
    allocation_ = {};
  }
  publishDiagnostics();
}
void PointTrackingNode::onStop(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  stopOutput(fault_ ? reason_ : "用户停止，归零后等待新目标", false);
  response->success = true;
  response->message = reason_;
}
void PointTrackingNode::onReset(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
{
  if (owned_ || zeroUntil_ || shuttingDown_)
  {
    response->message = "先stop并等待归零结束";
    return;
  }
  fault_ = false;
  state_.reset();
  received_.reset();
  lastRos_.reset();
  clockSeen_ = false;
  clockAdvanced_ = Steady::now();
  adapter_->reset();
  tracker_->reset();
  reason_ = "故障已复位；home保持不变，等待状态和新目标";
  response->success = true;
  response->message = reason_;
  publishDiagnostics();
}
void PointTrackingNode::prepareShutdown()
{
  shuttingDown_ = true;
  stopOutput(fault_ ? reason_ : "正常退出，有限补发零命令", false);
}

void PointTrackingNode::onTimer()
{
  const auto wall = Steady::now();
  const auto ros = now();
  lastStep_s_ = lastRos_ ? (ros - *lastRos_).seconds() : 0;
  if (lastStep_s_ > 0)
  {
    clockSeen_ = true;
    clockAdvanced_ = wall;
  }
  if (wall >= nextGraph_)
  {
    graphStatus_ = checkGraph();
    nextGraph_ = wall + std::chrono::seconds(1);
  }
  if (lastStep_s_ < 0 && !fault_)
  {
    stopOutput("ROS时间倒退；世界重置后建议重启完整仿真", true);
  }
  if (owned_ && !zeroUntil_ && !fault_ && !shuttingDown_)
  {
    maxStep_s_ = std::max(maxStep_s_, lastStep_s_);
    const auto checked = checkState(ros, wall);
    if (!checked.ok() || !graphStatus_.ok() || lastStep_s_ > limits_.max_ros_step_s)
    {
      stopOutput(!checked.ok() ? checked.message
                               : (!graphStatus_.ok() ? graphStatus_.message
                                                     : "ROS dt=" + std::to_string(lastStep_s_) +
                                                           " s 超过 max_ros_step_s=" +
                                                           std::to_string(limits_.max_ros_step_s)),
                 true);
    }
    else if (lastStep_s_ > 0)
    {
      const auto begin = Steady::now();
      const auto previousPhase = tracker_->phase();
      const auto output = tracker_->update(*state_, lastStep_s_);
      if (!output.ok())
      {
        stopOutput(output.status().message, true);
      }
      else
      {
        output_ = output.value();
        // 保持中漂移超圈也会再次规划；新的TRACKING段必须刷新RViz完整路径。
        if (output_.phase == TrackingPhase::TRACKING && previousPhase != TrackingPhase::TRACKING)
        {
          pathPublished_ = false;
        }
        const auto allocation = allocator_.allocate(output_.force_x_N, output_.torque_B_Nm);
        if (!allocation.ok())
        {
          stopOutput(allocation.status().message, true);
        }
        else
        {
          allocation_ = allocation.value();
          sendMotors(allocation_.throttle);
          if (output_.phase == TrackingPhase::TRACKING)
          {
            const double error = (output_.reference.p_W - state_->p_W).head<2>().norm();
            maxError_m_ = std::max(maxError_m_, error);
            errorSquares_ += error * error;
            ++errorSamples_;
          }
          reason_ = output_.phase == TrackingPhase::HOLDING ? "已到达水平容差，继续保持；深度仅监测"
                                                            : "水平到点控制运行中";
        }
      }
      compute_ms_ = std::chrono::duration<double, std::milli>(Steady::now() - begin).count();
    }
  }
  lastRos_ = ros;
  if (zeroUntil_)
  {
    if (wall < *zeroUntil_)
    {
      sendMotors({});
    }
    else
    {
      zeroUntil_.reset();
      owned_ = false;
    }
  }
  if (wall >= nextDisplay_)
  {
    if (state_ && home_)
    {
      publishDisplay(output_);
    }
    publishDiagnostics();
    nextDisplay_ = wall + std::chrono::milliseconds(100);
  }
}

void PointTrackingNode::publishDisplay(const TrackingOutput &output)
{
  geometry_msgs::msg::PoseStamped actual;
  actual.header.frame_id = worldFrame_;
  actual.header.stamp = now();
  actual.pose = toPose(state_->p_W, state_->q_WB);
  actualPath_.header = actual.header;
  if (owned_ && !zeroUntil_)
  {
    // 最多保留2000点（约200秒），防止长时间保持导致显示消息无限膨胀。
    if (actualPath_.poses.size() >= 2000)
    {
      actualPath_.poses.erase(actualPath_.poses.begin(), actualPath_.poses.begin() + 500);
    }
    actualPath_.poses.push_back(actual);
    actualPub_->publish(actualPath_);
    auto reference = actual;
    reference.pose = toPose(output.reference.p_W, output.reference.q_WB_des);
    referencePub_->publish(reference);
    if (tracker_->trajectory() && !pathPublished_)
    {
      const auto path = makePathMessage(*tracker_->trajectory(), now(), 101, worldFrame_);
      if (path.ok())
      {
        pathPub_->publish(path.value());
        pathPublished_ = true;
      }
    }
  }
}
void PointTrackingNode::publishDiagnostics()
{
  std::string phase;
  if (fault_)
  {
    phase = "FAULT";
  }
  else if (shuttingDown_)
  {
    phase = "STOPPING";
  }
  else if (owned_ && !zeroUntil_)
  {
    phase = trackingPhaseName(tracker_->phase());
  }
  else if (zeroUntil_)
  {
    phase = "STOPPING";
  }
  else
  {
    const auto state = checkState(now(), Steady::now());
    phase = state.ok() && graphStatus_.ok() ? "READY" : "WAIT_STATE";
    if (!state.ok())
    {
      reason_ = state.message;
    }
    else if (!graphStatus_.ok())
    {
      reason_ = graphStatus_.message;
    }
    else if (lastPhase_ == "WAIT_STATE" || reason_ == "等待有效状态")
    {
      reason_ = "状态与命令来源就绪，可向 goal 发布水平目标";
    }
  }
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus entry;
  entry.name = std::string(get_fully_qualified_name()) + "/horizontal_tracking";
  entry.hardware_id = "hippo_simulation_only";
  entry.level = fault_ ? 2 : (phase == "WAIT_STATE" ? 1 : 0);
  entry.message = reason_;
  const auto add = [&entry](const std::string &key, const std::string &value)
  {
    diagnostic_msgs::msg::KeyValue field;
    field.key = key;
    field.value = value;
    entry.values.push_back(field);
  };
  add("state", phase);
  add("goal_id", std::to_string(goalId_));
  add("rejected_goals", std::to_string(rejectedGoals_));
  add("home_frame", homeFrame_);
  add("distance_xy_m", std::to_string(output_.distance_xy_m));
  add("cross_track_m", std::to_string(output_.cross_track_m));
  add("heading_error_rad", std::to_string(output_.heading_error_rad));
  add("force_x_N", std::to_string(output_.force_x_N));
  add("torque_x_Nm", std::to_string(output_.torque_B_Nm.x()));
  add("torque_y_Nm", std::to_string(output_.torque_B_Nm.y()));
  add("torque_z_Nm", std::to_string(output_.torque_B_Nm.z()));
  add("allocation_scale", std::to_string(allocation_.scale));
  add("max_speed_mps", std::to_string(config_.motion.v_max_mps));
  add("max_actual_speed_mps", std::to_string(limits_.max_actual_speed_mps));
  add("max_thruster_command", std::to_string(limits_.max_thruster_command));
  add("limited", output_.limited ? "true" : "false");
  add("elapsed_reference_s", std::to_string(tracker_->elapsed()));
  add("ros_step_s", std::to_string(lastStep_s_));
  add("max_ros_step_s", std::to_string(limits_.max_ros_step_s));
  add("observed_max_ros_step_s", std::to_string(maxStep_s_));
  add("compute_ms", std::to_string(compute_ms_));
  add("max_tracking_error_m", std::to_string(maxError_m_));
  add("rms_tracking_error_m",
      std::to_string(errorSamples_ ? std::sqrt(errorSquares_ / static_cast<double>(errorSamples_))
                                   : 0));
  if (state_ && home_)
  {
    const auto local = home_->toHome(state_->p_W);
    add("actual_home_x_m", std::to_string(local.x()));
    add("actual_home_y_m", std::to_string(local.y()));
    add("depth_drift_m", std::to_string(state_->p_W.z() - home_->origin_W.z()));
    add("speed_xy_mps", std::to_string(state_->v_W.head<2>().norm()));
    add("state_age_s",
        std::to_string(static_cast<double>(now().nanoseconds() - state_->stamp_ns) * 1e-9));
  }
  for (std::size_t i = 0; i < 4; ++i)
  {
    add("motor_" + std::to_string(i), std::to_string(allocation_.throttle[i]));
  }
  message.status.push_back(entry);
  diagnosticPub_->publish(message);
  if (phase != lastPhase_)
  {
    RCLCPP_INFO(get_logger(), "%s：%s", phase.c_str(), reason_.c_str());
    lastPhase_ = phase;
  }
}
} // namespace rov_control_ros
