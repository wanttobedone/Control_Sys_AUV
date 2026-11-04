#pragma once

#include "rov_localization/apriltag_detector.h"
#include "rov_localization/tag_pose_estimator.h"
#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace rov_localization
{
// 只读ROS接线层：接收原始JPEG图像，复用已有检测/PnP，输出标签系中的相机位姿。
// 同一个节点可以放在lab或Pi。它不打开USB、不控制电机、不生成机体速度或home。
// 单线程executor串行调用onImage/onHealthTimer；检测库内部并行在detect返回前结束。
class TagPoseNode : public rclcpp::Node
{
public:
  // 读取启动参数并创建ROS通信端点；配置不完整时抛异常退出，不打开相机或飞控。
  explicit TagPoseNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  using SteadyClock = std::chrono::steady_clock;
  using ImageMessage = sensor_msgs::msg::CompressedImage;

  // 启动只读配置；真实K/D只有camera_file一处来源，不采用驱动未标定的CameraInfo。
  void loadConfig();
  // 一条新图像触发一次求解。坏帧仅更新原因，不将上一帧位姿补成新观测。
  // 本回调拥有计数、源时间、接收时间、耗时和valid/reason的更新权。
  void onImage(ImageMessage::ConstSharedPtr message);
  // 调用前pose.valid必须为true。输入是T_LC，输出PoseStamped和独立显示TF；
  // 位置单位米、旋转为单位四元数，时间保留source_header，不缓存历史位姿。
  void publishPose(const TagPoseEstimate &pose, const std_msgs::msg::Header &source_header);
  // 输入原始BGR图/所有检测/目标结果，输出同一源时间的叠加图。无人订阅时跳过画图。
  void publishDebugImage(const cv::Mat &image, const std::vector<TagDetection> &detections,
                         const TagPoseEstimate &pose, const std_msgs::msg::Header &source_header);
  // 读取当前计数、原因、耗时，发布DiagnosticArray。诊断stamp是本节点当前时间，
  // 和继承自相机驱动的pose时间不同；不把本机接收年龄声称为曝光到输出的延迟。
  void publishDiagnostics();
  // 看门狗只发布健康状态，不能产生位姿；断图时间使用本机单调时钟。
  void onHealthTimer();

  AprilTagDetector detector_;
  std::unique_ptr<TagPoseEstimator> estimator_;
  std::string inputFrame_;
  std::string tagFrame_;
  std::string displayCameraFrame_;
  double imageTimeoutS_{2.0};
  bool publishDebug_{true};

  // 最近一次源时间用于拒绝驱动重发/乱序；重启节点后清空，换ID也要求重启。
  std::int64_t lastStampNs_{-1};
  std::optional<SteadyClock::time_point> lastFreshReceive_;
  std::optional<SteadyClock::time_point> lastValidReceive_;
  std::uint64_t receivedFrames_{0};
  std::uint64_t processedFrames_{0};
  std::uint64_t validFrames_{0};
  bool valid_{false};
  std::string reason_{"waiting_image"};
  double rmsPx_{std::numeric_limits<double>::quiet_NaN()};
  double decodeMs_{0.0};
  double detectMs_{0.0};
  double solveMs_{0.0};

  rclcpp::Subscription<ImageMessage>::SharedPtr imageSub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugPub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticsPub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfPub_;
  rclcpp::TimerBase::SharedPtr healthTimer_;
};
} // namespace rov_localization
