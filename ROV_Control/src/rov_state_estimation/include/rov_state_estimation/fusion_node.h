#pragma once

#include "rov_state_estimation/hippo_fusion.h"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>

namespace rov_state_estimation
{

// 一个只读 ROS 进程。订阅、服务、健康定时器由单线程 executor 串行调度。
// 不发布控制/执行器命令；不把缺失的 EKF 协方差填零冒充 Odometry。
class FusionNode : public rclcpp::Node
{
public:
  explicit FusionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void loadConfig();
  void onCameraPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onPressure(const sensor_msgs::msg::FluidPressure::SharedPtr msg);
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void publishState(const FusionOutput &output);
  void publishDiagnostics();
  void publishStaticTag();
  void onResetHome(const std_srvs::srv::Trigger::Request::SharedPtr request,
                   std_srvs::srv::Trigger::Response::SharedPtr response);

  FrameConfig frame_config_;
  FusionConfig fusion_config_;
  std::unique_ptr<FrameAdapter> frames_;
  std::unique_ptr<HippoFusion> fusion_;
  std::string prefix_;
  std::string tag_frame_;
  std::string imu_frame_;
  std::string pressure_frame_;
  std::string world_frame_;
  std::string home_frame_;
  std::string body_frame_;
  std::string last_frame_error_;
  std::string last_reported_status_;
  int64_t last_imu_stamp_ns_{0};
  int64_t home_reset_vision_stamp_ns_{0};
  uint64_t output_count_{0};
  uint64_t frame_error_count_{0};
  bool received_imu_{false};
  bool last_ready_{false};
  std::chrono::steady_clock::time_point last_imu_receive_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr visual_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr home_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr health_timer_;
};

} // namespace rov_state_estimation
