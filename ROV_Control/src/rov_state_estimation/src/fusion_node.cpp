#include "rov_state_estimation/fusion_node.h"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <stdexcept>

namespace rov_state_estimation
{
namespace
{

geometry_msgs::msg::Pose toMessage(const RigidPose &pose)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.p_m.x();
  msg.position.y = pose.p_m.y();
  msg.position.z = pose.p_m.z();
  msg.orientation.x = pose.q.x();
  msg.orientation.y = pose.q.y();
  msg.orientation.z = pose.q.z();
  msg.orientation.w = pose.q.w();
  return msg;
}

geometry_msgs::msg::TransformStamped toTransform(const RigidPose &pose, const std::string &parent,
                                                 const std::string &child,
                                                 const builtin_interfaces::msg::Time &stamp)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.frame_id = parent;
  msg.header.stamp = stamp;
  msg.child_frame_id = child;
  msg.transform.translation.x = pose.p_m.x();
  msg.transform.translation.y = pose.p_m.y();
  msg.transform.translation.z = pose.p_m.z();
  msg.transform.rotation = toMessage(pose).orientation;
  return msg;
}

} // namespace

FusionNode::FusionNode(const rclcpp::NodeOptions &options) : Node("hippo_fusion", options)
{
  loadConfig();
  frames_ = std::make_unique<FrameAdapter>(frame_config_);
  fusion_ = std::make_unique<HippoFusion>(fusion_config_);
  visual_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("vision/body_pose", 10);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("state/pose", 10);
  home_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("state/pose_home", 10);
  velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("state/velocity_body", 10);
  diagnostic_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("state/diagnostics", 10);
  tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
  publishStaticTag();
  vision_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "vision/camera_pose", 10, std::bind(&FusionNode::onCameraPose, this, std::placeholders::_1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS().keep_last(100),
      std::bind(&FusionNode::onImu, this, std::placeholders::_1));
  pressure_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(
      "pressure", rclcpp::SensorDataQoS().keep_last(20),
      std::bind(&FusionNode::onPressure, this, std::placeholders::_1));
  reset_service_ = create_service<std_srvs::srv::Trigger>(
      "state/reset_home",
      std::bind(&FusionNode::onResetHome, this, std::placeholders::_1, std::placeholders::_2));
  // 健康定时器只发诊断；IMU 中断时绝不靠它继续发布带新时间戳的旧状态。
  health_timer_ = create_wall_timer(std::chrono::milliseconds(250),
                                    std::bind(&FusionNode::publishDiagnostics, this));
  RCLCPP_INFO(get_logger(), "只读 Hippo EKF：等待 IMU、压力和 %s；初始化时保持 ROV 静稳",
              tag_frame_.c_str());
}

void FusionNode::loadConfig()
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  prefix_ = declare_parameter<std::string>("frame_prefix", "uuv00", descriptor);
  tag_frame_ = declare_parameter<std::string>("tag_frame", prefix_ + "/tag_16", descriptor);
  imu_frame_ = declare_parameter<std::string>("imu_frame", "base_link", descriptor);
  pressure_frame_ = declare_parameter<std::string>("pressure_frame", "base_link", descriptor);
  if (prefix_.empty() || prefix_.front() == '/' || tag_frame_.empty() || imu_frame_.empty() ||
      pressure_frame_.empty())
  {
    throw std::invalid_argument("frame_prefix 必须非空且无开头 /，输入 frame 必须非空");
  }
  world_frame_ = prefix_ + "/vision_world";
  home_frame_ = prefix_ + "/home";
  body_frame_ = prefix_ + "/fused_body";
  const auto read_pose = [this, &descriptor](const std::string &translation_name,
                                             const std::string &rotation_name, RigidPose &pose)
  {
    const std::vector<double> p_default{pose.p_m.x(), pose.p_m.y(), pose.p_m.z()};
    const std::vector<double> q_default{pose.q.x(), pose.q.y(), pose.q.z(), pose.q.w()};
    const auto p = declare_parameter<std::vector<double>>(translation_name, p_default, descriptor);
    const auto q = declare_parameter<std::vector<double>>(rotation_name, q_default, descriptor);
    if (p.size() != 3 || q.size() != 4)
    {
      throw std::invalid_argument(translation_name + " 需 3 个数，" + rotation_name +
                                  " 需 xyzw 4 个数");
    }
    pose.p_m = Eigen::Vector3d(p[0], p[1], p[2]);
    pose.q = Eigen::Quaterniond(q[3], q[0], q[1], q[2]);
  };
  read_pose("camera_translation_B_m", "camera_rotation_BC_xyzw", frame_config_.camera_BC);
  read_pose("tag_translation_W_m", "tag_rotation_WL_xyzw", frame_config_.tag_WL);
  const auto read_number = [this, &descriptor](const std::string &name, double &value)
  { value = declare_parameter<double>(name, value, descriptor); };
  read_number("water_density_kg_m3", fusion_config_.water_density_kg_m3);
  read_number("pressure_height_std_m", fusion_config_.pressure_height_std_m);
  read_number("vision_position_std_m", fusion_config_.vision_position_std_m);
  read_number("history_s", fusion_config_.history_s);
  read_number("pair_tolerance_s", fusion_config_.pair_tolerance_s);
  read_number("imu_max_gap_s", fusion_config_.imu_max_gap_s);
  read_number("vision_timeout_s", fusion_config_.vision_timeout_s);
  read_number("pressure_timeout_s", fusion_config_.pressure_timeout_s);
  read_number("startup_settle_s", fusion_config_.startup_settle_s);
  read_number("startup_position_tolerance_m", fusion_config_.startup_position_tolerance_m);
  read_number("startup_angle_tolerance_rad", fusion_config_.startup_angle_tolerance_rad);
}

void FusionNode::onCameraPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (msg->header.frame_id != tag_frame_)
  {
    ++frame_error_count_;
    last_frame_error_ = "vision_frame_expected_" + tag_frame_;
    return;
  }
  const auto &p = msg->pose.position;
  const auto &q = msg->pose.orientation;
  const RigidPose camera_LC{{p.x, p.y, p.z}, {q.w, q.x, q.y, q.z}};
  if (!FrameAdapter::isValid(camera_LC))
  {
    ++frame_error_count_;
    last_frame_error_ = "invalid_camera_pose";
    return;
  }
  const RigidPose body_WB = frames_->cameraToBody(camera_LC);
  const int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  if (!fusion_->setVision(stamp_ns, body_WB))
  {
    return;
  }
  geometry_msgs::msg::PoseStamped observation;
  observation.header = msg->header;
  observation.header.frame_id = world_frame_;
  observation.pose = toMessage(body_WB);
  visual_pub_->publish(observation);
}

void FusionNode::onPressure(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
{
  if (msg->header.frame_id != pressure_frame_)
  {
    ++frame_error_count_;
    last_frame_error_ = "pressure_frame_expected_" + pressure_frame_;
    return;
  }
  fusion_->setPressure(rclcpp::Time(msg->header.stamp).nanoseconds(), msg->fluid_pressure);
}

void FusionNode::onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (msg->header.frame_id != imu_frame_)
  {
    ++frame_error_count_;
    last_frame_error_ = "imu_frame_expected_" + imu_frame_;
    return;
  }
  const int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  const auto &w = msg->angular_velocity;
  const auto &a = msg->linear_acceleration;
  const Eigen::Vector3d omega_B(w.x, w.y, w.z);
  const Eigen::Vector3d specific_force_B(a.x, a.y, a.z);
  if (stamp_ns > last_imu_stamp_ns_ && omega_B.allFinite() && specific_force_B.allFinite())
  {
    received_imu_ = true;
    last_imu_receive_ = std::chrono::steady_clock::now();
    last_imu_stamp_ns_ = stamp_ns;
  }
  // 不读取 msg.orientation：USTROV SensorCombined 桥明确将其标为未知。
  const auto output = fusion_->pushImu(stamp_ns, omega_B, specific_force_B);
  if (output && fusion_->initialized())
  {
    last_ready_ = output->ready;
    publishState(*output);
  }
}

void FusionNode::publishStaticTag()
{
  static_tf_->sendTransform(toTransform(frame_config_.tag_WL, world_frame_, tag_frame_, now()));
}

void FusionNode::publishState(const FusionOutput &output)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = rclcpp::Time(output.stamp_ns);
  pose.header.frame_id = world_frame_;
  pose.pose = toMessage(output.body_WB);
  pose_pub_->publish(pose);
  ++output_count_;

  geometry_msgs::msg::TwistStamped velocity;
  velocity.header.stamp = pose.header.stamp;
  velocity.header.frame_id = body_frame_;
  // TwistStamped 的 linear 和 angular 必须在同一坐标系，这里统一为 FLU 机体系。
  const Eigen::Vector3d v_B = output.body_WB.q.conjugate() * output.velocity_W_mps;
  velocity.twist.linear.x = v_B.x();
  velocity.twist.linear.y = v_B.y();
  velocity.twist.linear.z = v_B.z();
  velocity.twist.angular.x = output.angular_velocity_B_radps.x();
  velocity.twist.angular.y = output.angular_velocity_B_radps.y();
  velocity.twist.angular.z = output.angular_velocity_B_radps.z();
  velocity_pub_->publish(velocity);

  if (!frames_->home() && output.ready && fusion_->visionStamp() > home_reset_vision_stamp_ns_)
  {
    if (frames_->initializeHome(output.body_WB))
    {
      RCLCPP_INFO(get_logger(), "home 已建立：融合机体位置为原点，初始水平机头为 +X，Z 向上");
    }
  }
  if (frames_->home())
  {
    const RigidPose body_HB = frames_->toHome(output.body_WB);
    pose.header.frame_id = home_frame_;
    pose.pose = toMessage(body_HB);
    home_pub_->publish(pose);
    tf_->sendTransform(toTransform(*frames_->home(), world_frame_, home_frame_, pose.header.stamp));
    tf_->sendTransform(toTransform(body_HB, home_frame_, body_frame_, pose.header.stamp));
  }
}

void FusionNode::onResetHome(const std_srvs::srv::Trigger::Request::SharedPtr,
                             std_srvs::srv::Trigger::Response::SharedPtr response)
{
  // 只清 home；不丢弃 EKF 历史、不重新标压力。等待请求之后的新视觉与新融合输出。
  home_reset_vision_stamp_ns_ = fusion_->visionStamp();
  frames_->resetHome();
  response->success = true;
  response->message = "已清除 home；等待新的有效视觉和就绪融合状态重建";
}

void FusionNode::publishDiagnostics()
{
  const double imu_receive_age_s =
      received_imu_
          ? std::chrono::duration<double>(std::chrono::steady_clock::now() - last_imu_receive_)
                .count()
          : -1.0;
  std::string state = fusion_->status();
  const bool imu_stale = !received_imu_ || imu_receive_age_s > fusion_config_.imu_max_gap_s;
  if (imu_stale && state.rfind("FAULT", 0) != 0)
  {
    state = received_imu_ ? "IMU_RECEIVE_STALE" : "WAIT_IMU";
  }
  const bool ready = last_ready_ && !imu_stale && state == "READY";
  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_fully_qualified_name();
  status.hardware_id = "USTROV_EKF_read_only";
  status.level = ready ? 0 : (state.rfind("FAULT", 0) == 0 ? 2 : 1);
  status.message = state;
  const auto add = [&status](const std::string &key, const std::string &value)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(item);
  };
  add("ready", ready ? "true" : "false");
  add("home_ready", frames_->home() ? "true" : "false");
  add("pressure_aligned", fusion_->pressureAligned() ? "true" : "false");
  add("tilt_variance_rad2", std::to_string(fusion_->tiltVarianceRad2()));
  add("imu_receive_age_s", std::to_string(imu_receive_age_s));
  add("imu_source_stamp_ns", std::to_string(last_imu_stamp_ns_));
  add("vision_source_stamp_ns", std::to_string(fusion_->visionStamp()));
  add("pressure_source_stamp_ns", std::to_string(fusion_->pressureStamp()));
  add("rejected_inputs", std::to_string(fusion_->rejectedInputs()));
  add("last_input_error", fusion_->inputError());
  add("frame_errors", std::to_string(frame_error_count_));
  add("last_frame_error", last_frame_error_);
  add("output_count", std::to_string(output_count_));
  add("covariance", "upstream_not_exported_pose_twist_only");
  msg.status.push_back(status);
  diagnostic_pub_->publish(msg);
  if (state != last_reported_status_)
  {
    RCLCPP_INFO(get_logger(), "%s", state.c_str());
    last_reported_status_ = state;
  }
}

} // namespace rov_state_estimation
