#include "rov_localization/tag_pose_node.h"
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <stdexcept>
#include <tf2/LinearMath/Matrix3x3.hpp>

namespace rov_localization
{
TagPoseNode::TagPoseNode(const rclcpp::NodeOptions &options) : Node("tag_pose", options)
{
  loadConfig();
  posePub_ = create_publisher<geometry_msgs::msg::PoseStamped>("vision/camera_pose", 1);
  debugPub_ = create_publisher<sensor_msgs::msg::Image>("vision/debug_image",
                                                        rclcpp::SensorDataQoS().keep_last(1));
  diagnosticsPub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("vision/diagnostics", 1);
  tfPub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  imageSub_ = create_subscription<ImageMessage>(
      "vertical_camera/image_raw/compressed", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&TagPoseNode::onImage, this, std::placeholders::_1));
  healthTimer_ = create_wall_timer(std::chrono::milliseconds(250),
                                   std::bind(&TagPoseNode::onHealthTimer, this));
  RCLCPP_INFO(get_logger(), "只读视觉节点：ID=%d，输入=%s，位姿属于%s；等待JPEG图像",
              estimator_->options().target_id, inputFrame_.c_str(), tagFrame_.c_str());
}

void TagPoseNode::loadConfig()
{
  rcl_interfaces::msg::ParameterDescriptor readOnly;
  readOnly.read_only = true;
  readOnly.description = "启动配置；修改文件后重启节点，避免显示值与求解器模型不一致";
  const auto cameraFile = declare_parameter<std::string>("camera_file", "", readOnly);
  const auto optionsFile = declare_parameter<std::string>("options_file", "", readOnly);
  const auto prefix = declare_parameter<std::string>("frame_prefix", "uuv00", readOnly);
  imageTimeoutS_ = declare_parameter<double>("image_timeout_s", 2.0, readOnly);
  publishDebug_ = declare_parameter<bool>("publish_debug_image", true, readOnly);
  if (cameraFile.empty() || optionsFile.empty())
  {
    throw std::invalid_argument("camera_file和options_file必须指向实际XML/YAML文件");
  }
  if (prefix.empty() || prefix.front() == '/' || prefix.back() == '/' ||
      prefix.find_first_of(" \t\r\n") != std::string::npos)
  {
    throw std::invalid_argument("frame_prefix须非空，不能含空白或以/开头、结尾");
  }
  if (!std::isfinite(imageTimeoutS_) || imageTimeoutS_ <= 0.0)
  {
    throw std::invalid_argument("image_timeout_s必须为有限正数，单位秒");
  }
  const CameraModel camera = loadCameraModel(cameraFile);
  const TagPoseOptions options = loadTagPoseOptions(optionsFile);
  estimator_ = std::make_unique<TagPoseEstimator>(camera, options);
  detector_.configure(options.detector_threads, options.max_hamming);
  inputFrame_ = prefix + "/camera_optical";
  tagFrame_ = prefix + "/tag_" + std::to_string(options.target_id);
  // 这是RViz独立显示帧，不给已有base_link→camera_optical静态TF再加第二个父帧。
  displayCameraFrame_ = prefix + "/vision_camera_optical";
}

void TagPoseNode::onImage(ImageMessage::ConstSharedPtr message)
{
  ++receivedFrames_;
  valid_ = false;
  rmsPx_ = std::numeric_limits<double>::quiet_NaN();
  decodeMs_ = 0.0;
  detectMs_ = 0.0;
  solveMs_ = 0.0;
  reason_ = "ok";
  const auto received = SteadyClock::now();
  const auto &stamp = message->header.stamp;
  const std::int64_t stampNs = static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
                               static_cast<std::int64_t>(stamp.nanosec);
  // 接口语义只在入口核验：不能把无标记/错相机/重复的图像当新的测量。
  if (message->header.frame_id != inputFrame_)
  {
    reason_ = "input_frame_mismatch";
  }
  else if (stamp.sec < 0 || stamp.nanosec >= 1000000000U || stampNs <= 0)
  {
    reason_ = "invalid_stamp";
  }
  else if (stampNs <= lastStampNs_)
  {
    reason_ = "non_increasing_stamp";
  }
  else if (message->format.find("jpeg") == std::string::npos || message->data.empty())
  {
    reason_ = "expected_nonempty_jpeg";
  }
  if (reason_ != "ok")
  {
    publishDiagnostics();
    return;
  }
  lastStampNs_ = stampNs;
  lastFreshReceive_ = received;

  try
  {
    // JPEG是原始相机图的压缩表示。这里只解码，不rectify；求解仍使用原K和原D。
    const cv::Mat image = cv::imdecode(message->data, cv::IMREAD_COLOR);
    const auto decoded = SteadyClock::now();
    decodeMs_ = std::chrono::duration<double, std::milli>(decoded - received).count();
    if (image.empty())
    {
      reason_ = "jpeg_decode_failed";
      publishDiagnostics();
      return;
    }
    if (image.size() != estimator_->camera().image_size)
    {
      reason_ = "image_size_mismatch";
      publishDiagnostics();
      return;
    }
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    const auto detections = detector_.detect(gray);
    const auto detected = SteadyClock::now();
    // detect_ms含灰度转换及AprilTag检测；solve_ms仅含已有PnP和质量判断。
    detectMs_ = std::chrono::duration<double, std::milli>(detected - decoded).count();
    const auto pose = estimator_->estimate(detections);
    solveMs_ = std::chrono::duration<double, std::milli>(SteadyClock::now() - detected).count();
    ++processedFrames_;
    valid_ = pose.valid;
    reason_ = pose.reason;
    rmsPx_ = pose.reprojection_error_px;
    if (pose.valid)
    {
      ++validFrames_;
      lastValidReceive_ = received;
      publishPose(pose, message->header);
    }
    publishDebugImage(image, detections, pose, message->header);
  }
  catch (const cv::Exception &error)
  {
    valid_ = false;
    reason_ = "opencv_error";
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "图像处理失败：%s", error.what());
  }
  publishDiagnostics();
}

void TagPoseNode::publishPose(const TagPoseEstimate &pose,
                              const std_msgs::msg::Header &source_header)
{
  geometry_msgs::msg::PoseStamped message;
  message.header = source_header;
  message.header.frame_id = tagFrame_;
  message.pose.position.x = pose.p_LC_m[0];
  message.pose.position.y = pose.p_LC_m[1];
  message.pose.position.z = pose.p_LC_m[2];
  // R_LC把相机光学向量旋转到标签局部系。矩阵直接转四元数，避免经显示用欧拉角绕行。
  const auto &rotation = pose.R_LC;
  const tf2::Matrix3x3 matrix(rotation(0, 0), rotation(0, 1), rotation(0, 2), rotation(1, 0),
                              rotation(1, 1), rotation(1, 2), rotation(2, 0), rotation(2, 1),
                              rotation(2, 2));
  tf2::Quaternion quaternion;
  matrix.getRotation(quaternion);
  quaternion.normalize();
  message.pose.orientation.x = quaternion.x();
  message.pose.orientation.y = quaternion.y();
  message.pose.orientation.z = quaternion.z();
  message.pose.orientation.w = quaternion.w();
  posePub_->publish(message);

  geometry_msgs::msg::TransformStamped transform;
  transform.header = message.header;
  transform.child_frame_id = displayCameraFrame_;
  transform.transform.translation.x = message.pose.position.x;
  transform.transform.translation.y = message.pose.position.y;
  transform.transform.translation.z = message.pose.position.z;
  transform.transform.rotation = message.pose.orientation;
  tfPub_->sendTransform(transform);
}

void TagPoseNode::publishDebugImage(const cv::Mat &image,
                                    const std::vector<TagDetection> &detections,
                                    const TagPoseEstimate &pose,
                                    const std_msgs::msg::Header &source_header)
{
  // 图像只为调试服务。Pi本地部署可关闭；没有订阅者时不复制/画图。
  if (!publishDebug_ || debugPub_->get_subscription_count() == 0)
  {
    return;
  }
  cv::Mat overlay = image.clone();
  for (const auto &detection : detections)
  {
    const bool target = detection.id == estimator_->options().target_id;
    const cv::Scalar color = target ? cv::Scalar(0, 200, 0) : cv::Scalar(180, 180, 180);
    for (std::size_t i = 0; i < 4; ++i)
    {
      const auto &point = detection.corners_px[i];
      cv::line(overlay, point, detection.corners_px[(i + 1) % 4], color, 2);
      cv::putText(overlay, "p" + std::to_string(i), point, cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }
    cv::putText(overlay, "ID=" + std::to_string(detection.id),
                detection.corners_px[0] + cv::Point2f(0.0F, 18.0F), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                color, 2);
  }
  const cv::Scalar statusColor = pose.valid ? cv::Scalar(0, 190, 0) : cv::Scalar(0, 0, 220);
  cv::putText(overlay,
              "target=" + std::to_string(estimator_->options().target_id) + " " + pose.reason,
              cv::Point(12, 25), cv::FONT_HERSHEY_SIMPLEX, 0.65, statusColor, 2);
  sensor_msgs::msg::Image message;
  message.header = source_header;
  message.height = static_cast<std::uint32_t>(overlay.rows);
  message.width = static_cast<std::uint32_t>(overlay.cols);
  message.encoding = "bgr8";
  message.is_bigendian = false;
  message.step = message.width * 3U;
  // clone得到连续BGR内存；直接填标准Image，避免额外引入一层图像转换依赖。
  message.data.assign(overlay.datastart, overlay.dataend);
  debugPub_->publish(message);
}

void TagPoseNode::onHealthTimer()
{
  if (lastFreshReceive_ &&
      std::chrono::duration<double>(SteadyClock::now() - *lastFreshReceive_).count() >
          imageTimeoutS_)
  {
    valid_ = false;
    reason_ = "image_timeout";
  }
  publishDiagnostics();
}

void TagPoseNode::publishDiagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray message;
  // 诊断的时间是本节点当前时间；位姿的时间来自相机，二者不能混为同一概念。
  message.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = std::string(get_namespace()) + "/tag_pose";
  status.hardware_id = inputFrame_;
  status.level = valid_ ? diagnostic_msgs::msg::DiagnosticStatus::OK
                        : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = reason_;
  const auto add = [&status](const std::string &key, const std::string &value)
  {
    diagnostic_msgs::msg::KeyValue field;
    field.key = key;
    field.value = value;
    status.values.push_back(field);
  };
  add("valid", valid_ ? "true" : "false");
  add("target_id", std::to_string(estimator_->options().target_id));
  add("pose_frame", tagFrame_);
  add("camera_status", estimator_->camera().calibration_status);
  add("received_frames", std::to_string(receivedFrames_));
  add("processed_frames", std::to_string(processedFrames_));
  add("valid_frames", std::to_string(validFrames_));
  add("source_stamp_ns", std::to_string(lastStampNs_));
  add("rms_px", std::to_string(rmsPx_));
  add("decode_ms", std::to_string(decodeMs_));
  add("detect_ms_including_grayscale", std::to_string(detectMs_));
  add("solve_ms", std::to_string(solveMs_));
  const auto current = SteadyClock::now();
  add("since_fresh_receive_s",
      lastFreshReceive_
          ? std::to_string(std::chrono::duration<double>(current - *lastFreshReceive_).count())
          : "unavailable");
  add("since_valid_receive_s",
      lastValidReceive_
          ? std::to_string(std::chrono::duration<double>(current - *lastValidReceive_).count())
          : "unavailable");
  add("timing_note",
      "receive ages are local; source stamp is driver time, not verified exposure time");
  message.status.push_back(status);
  diagnosticsPub_->publish(message);
}
} // namespace rov_localization
