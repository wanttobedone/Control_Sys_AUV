// 正式ROS接口回归：合成JPEG走真实解码/检测/PnP和DDS，不访问USB、Pi或执行器。
#include "rov_localization/tag_pose_node.h"
#include <filesystem>
#include <functional>
#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <thread>
extern "C"
{
#include <apriltag.h>
#include <tag36h11.h>
}

namespace rl = rov_localization;
namespace fs = std::filesystem;

class TagPoseRos : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
    prefix_ = "tag_ros_test_" + std::to_string(suffix);
    directory_ = fs::temp_directory_path() / ("rov_tag_pose_ros_" + std::to_string(suffix));
    ASSERT_TRUE(fs::create_directory(directory_));
    camera_.image_size = cv::Size(1280, 720);
    camera_.camera_matrix = (cv::Mat_<double>(3, 3) << 700, 0, 640, 0, 690, 360, 0, 0, 1);
    camera_.dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
    camera_.calibration_status = "synthetic_test_only";
    cv::FileStorage file((directory_ / "camera.xml").string(), cv::FileStorage::WRITE);
    file << "camera_model" << "pinhole" << "distortion_model" << "plumb_bob" << "status"
         << camera_.calibration_status << "image_width" << 1280 << "image_height" << 720
         << "camera_matrix" << camera_.camera_matrix << "dist_coeffs" << camera_.dist_coeffs;
    file.release();
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/" + prefix_});
    options.parameter_overrides({{"camera_file", (directory_ / "camera.xml").string()},
                                 {"options_file", std::string(ROV_TAG_POSE_CONFIG)},
                                 {"frame_prefix", prefix_},
                                 {"image_timeout_s", 1.0}});
    node_ = std::make_shared<rl::TagPoseNode>(options);
    probe_ = std::make_shared<rclcpp::Node>("probe", "/" + prefix_);
    imagePub_ = probe_->create_publisher<sensor_msgs::msg::CompressedImage>(
        "vertical_camera/image_raw/compressed", rclcpp::SensorDataQoS());
    poseSub_ = probe_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "vision/camera_pose", 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message)
        {
          pose_ = *message;
          ++poseCount_;
        });
    diagnosticsSub_ = probe_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "vision/diagnostics", 10,
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message)
        {
          status_ = message->status.at(0);
          ++diagnosticsCount_;
        });
    debugSub_ = probe_->create_subscription<sensor_msgs::msg::Image>(
        "vision/debug_image", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr message) { debug_ = *message; });
    tfSub_ = probe_->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", 10,
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message)
        {
          for (const auto &transform : message->transforms)
          {
            if (transform.child_frame_id == prefix_ + "/vision_camera_optical")
            {
              transform_ = transform;
              ++tfCount_;
            }
          }
        });
    executor_.add_node(node_);
    executor_.add_node(probe_);
    ASSERT_TRUE(pumpUntil(
        [this]
        {
          return imagePub_->get_subscription_count() > 0 && poseSub_->get_publisher_count() > 0 &&
                 debugSub_->get_publisher_count() > 0 && tfSub_->get_publisher_count() > 0 &&
                 diagnosticsSub_->get_publisher_count() > 0;
        }));
  }

  void TearDown() override
  {
    if (node_)
    {
      executor_.remove_node(node_);
    }
    if (probe_)
    {
      executor_.remove_node(probe_);
    }
    node_.reset();
    probe_.reset();
    // 只清理由本夹具生成的目录。正式测试源码和用户标定/录像不在此路径。
    if (directory_.parent_path() == fs::temp_directory_path() &&
        directory_.filename().string().rfind("rov_tag_pose_ros_", 0) == 0)
    {
      fs::remove_all(directory_);
    }
  }

  bool pumpUntil(const std::function<bool()> &condition, double timeout_s = 4.0)
  {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() <
           timeout_s)
    {
      executor_.spin_some();
      if (condition())
      {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
  }

  sensor_msgs::msg::CompressedImage message(const cv::Mat &image, int second)
  {
    sensor_msgs::msg::CompressedImage result;
    result.header.frame_id = prefix_ + "/camera_optical";
    result.header.stamp.sec = second;
    result.header.stamp.nanosec = 123456789U;
    result.format = "jpeg";
    cv::imencode(".jpg", image, result.data, {cv::IMWRITE_JPEG_QUALITY, 95});
    return result;
  }

  cv::Mat tagImage()
  {
    // 真实编码图案先检测一次取得原生p序，然后按独立给定T_CL投影。
    // 此步骤只决定图案朝向，期望位姿由下面的rvec_/tvec_解析计算，不来自待测节点。
    apriltag_family_t *family = tag36h11_create();
    image_u8_t *raw = apriltag_to_image(family, 16);
    const cv::Mat small(raw->height, raw->width, CV_8UC1, raw->buf,
                        static_cast<std::size_t>(raw->stride));
    cv::Mat marker;
    cv::resize(small, marker, cv::Size(), 30.0, 30.0, cv::INTER_NEAREST);
    image_u8_destroy(raw);
    tag36h11_destroy(family);
    cv::copyMakeBorder(marker, marker, 80, 80, 80, 80, cv::BORDER_CONSTANT, cv::Scalar(255));
    rl::AprilTagDetector detector;
    detector.configure(1, 0);
    const auto detected = detector.detect(marker);
    if (detected.size() != 1)
    {
      throw std::runtime_error("测试图案未检出唯一ID");
    }
    const rl::TagPoseEstimator geometry(camera_, rl::TagPoseOptions{});
    std::vector<cv::Point2d> projected;
    cv::projectPoints(geometry.objectPoints(), rvec_, tvec_, camera_.camera_matrix,
                      camera_.dist_coeffs, projected);
    // projectPoints的输出保持与double物点同精度，再为warpPerspective转float。
    std::vector<cv::Point2f> destination;
    for (const auto &point : projected)
    {
      destination.emplace_back(point);
    }
    const std::vector<cv::Point2f> source(detected[0].corners_px.begin(),
                                          detected[0].corners_px.end());
    cv::Mat image;
    cv::warpPerspective(marker, image, cv::getPerspectiveTransform(source, destination),
                        camera_.image_size, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255));
    return image;
  }

  void expectRejected(const sensor_msgs::msg::CompressedImage &image, const std::string &reason)
  {
    const auto count = poseCount_;
    const auto tfCount = tfCount_;
    const auto oldDiagnostics = diagnosticsCount_;
    imagePub_->publish(image);
    ASSERT_TRUE(
        pumpUntil([&] { return diagnosticsCount_ > oldDiagnostics && status_.message == reason; }))
        << reason;
    EXPECT_EQ(poseCount_, count);
    EXPECT_EQ(tfCount_, tfCount);
  }

  fs::path directory_;
  std::string prefix_;
  rl::CameraModel camera_;
  const cv::Vec3d rvec_{0.4, -0.25, 0.1};
  const cv::Vec3d tvec_{0.0, 0.0, 0.30};
  std::shared_ptr<rl::TagPoseNode> node_;
  rclcpp::Node::SharedPtr probe_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr imagePub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticsSub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debugSub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tfSub_;
  geometry_msgs::msg::PoseStamped pose_;
  geometry_msgs::msg::TransformStamped transform_;
  sensor_msgs::msg::Image debug_;
  diagnostic_msgs::msg::DiagnosticStatus status_;
  std::size_t poseCount_{0};
  std::size_t tfCount_{0};
  std::size_t diagnosticsCount_{0};
};

TEST_F(TagPoseRos, JpegProducesCameraPoseTfAndDebugWithOriginalTimestamp)
{
  imagePub_->publish(message(tagImage(), 100));
  ASSERT_TRUE(
      pumpUntil([this] { return poseCount_ == 1 && tfCount_ == 1 && !debug_.data.empty(); }));
  EXPECT_EQ(pose_.header.frame_id, prefix_ + "/tag_16");
  EXPECT_EQ(pose_.header.stamp.sec, 100);
  EXPECT_EQ(pose_.header.stamp.nanosec, 123456789U);
  cv::Matx33d rotation;
  cv::Rodrigues(rvec_, rotation);
  const cv::Vec3d expected = -(rotation.t() * tvec_);
  // 渲染/角点亚像素误差的测试容差为3mm；不作为水下实机精度声明。
  const auto &position = pose_.pose.position;
  EXPECT_LT(cv::norm(cv::Vec3d(position.x, position.y, position.z) - expected), 0.003);
  const auto &q = pose_.pose.orientation;
  EXPECT_NEAR(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 1.0, 1e-10);
  const tf2::Matrix3x3 actualRotation(tf2::Quaternion(q.x, q.y, q.z, q.w));
  for (int row = 0; row < 3; ++row)
  {
    for (int column = 0; column < 3; ++column)
    {
      // 要验证的是R_LC=R_CL转置，不只是四元数模长正确。
      EXPECT_NEAR(actualRotation[row][column], rotation(column, row), 0.02);
    }
  }
  EXPECT_EQ(transform_.header, pose_.header);
  EXPECT_EQ(transform_.transform.rotation, pose_.pose.orientation);
  EXPECT_DOUBLE_EQ(transform_.transform.translation.x, position.x);
  EXPECT_EQ(debug_.header.stamp, pose_.header.stamp);
  EXPECT_EQ(debug_.header.frame_id, prefix_ + "/camera_optical");
  EXPECT_EQ(debug_.encoding, "bgr8");
  EXPECT_EQ(debug_.data.size(), 1280U * 720U * 3U);
}

TEST_F(TagPoseRos, RejectsMissingTagsWrongSizeBadJpegAndWrongFrameWithoutOldPose)
{
  imagePub_->publish(message(tagImage(), 100));
  ASSERT_TRUE(pumpUntil([this] { return poseCount_ == 1 && tfCount_ == 1; }));
  expectRejected(message(cv::Mat(720, 1280, CV_8UC1, cv::Scalar(255)), 101), "target_not_found");
  expectRejected(message(cv::Mat(480, 640, CV_8UC1, cv::Scalar(255)), 102), "image_size_mismatch");
  auto bad = message(tagImage(), 103);
  bad.data = {1, 2, 3, 4};
  expectRejected(bad, "jpeg_decode_failed");
  bad = message(tagImage(), 104);
  bad.header.frame_id = "wrong_camera";
  expectRejected(bad, "input_frame_mismatch");
  bad = message(tagImage(), 105);
  bad.format = "png";
  expectRejected(bad, "expected_nonempty_jpeg");
}

TEST_F(TagPoseRos, RejectsRepeatedOrBackwardSourceTimeAndRecoversOnNewFrame)
{
  const auto first = message(tagImage(), 100);
  imagePub_->publish(first);
  ASSERT_TRUE(pumpUntil([this] { return poseCount_ == 1 && tfCount_ == 1; }));
  expectRejected(first, "non_increasing_stamp");
  auto old = first;
  old.header.stamp.sec = 99;
  expectRejected(old, "non_increasing_stamp");
  old.header.stamp.sec = 0;
  old.header.stamp.nanosec = 0;
  expectRejected(old, "invalid_stamp");
  auto next = first;
  next.header.stamp.sec = 101;
  imagePub_->publish(next);
  ASSERT_TRUE(pumpUntil([this] { return poseCount_ == 2; }));
  EXPECT_EQ(pose_.header.stamp.sec, 101);
}

TEST_F(TagPoseRos, NoImageTimeoutPublishesStatusOnlyAndNewImageResumes)
{
  imagePub_->publish(message(tagImage(), 100));
  ASSERT_TRUE(pumpUntil([this] { return poseCount_ == 1 && tfCount_ == 1; }));
  ASSERT_TRUE(pumpUntil([this] { return status_.message == "image_timeout"; }, 2.5));
  EXPECT_EQ(poseCount_, 1U);
  EXPECT_EQ(tfCount_, 1U);
  imagePub_->publish(message(tagImage(), 101));
  ASSERT_TRUE(pumpUntil([this] { return poseCount_ == 2 && status_.message == "ok"; }));
}

TEST_F(TagPoseRos, ConfigurationCannotSilentlyChangeWhileRunning)
{
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("frame_prefix", "other")).successful);
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("image_timeout_s", -1.0)).successful);
  rclcpp::NodeOptions missing;
  EXPECT_THROW(std::make_shared<rl::TagPoseNode>(missing), std::invalid_argument);
}
