#include "rov_localization/apriltag_detector.h"
#include "rov_localization/tag_pose_estimator.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iterator>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
extern "C"
{
#include <apriltag.h>
#include <tag36h11.h>
}

namespace rl = rov_localization;
namespace fs = std::filesystem;

namespace
{
rl::CameraModel cameraModel()
{
  rl::CameraModel camera;
  camera.image_size = cv::Size(1280, 720);
  camera.camera_matrix =
      (cv::Mat_<double>(3, 3) << 700.0, 0.0, 640.0, 0.0, 690.0, 360.0, 0.0, 0.0, 1.0);
  camera.dist_coeffs = (cv::Mat_<double>(1, 5) << -0.06, 0.015, 0.001, -0.0005, 0.0);
  camera.calibration_status = "candidate_requires_metric_validation";
  return camera;
}

// 用真正已知的T_CL投影角点。几何真值不依赖待测求解器的输出。
rl::TagDetection projectTag(const rl::TagPoseEstimator &estimator, const cv::Vec3d &rvec,
                            const cv::Vec3d &tvec)
{
  std::vector<cv::Point2d> pixels;
  cv::projectPoints(estimator.objectPoints(), rvec, tvec, estimator.camera().camera_matrix,
                    estimator.camera().dist_coeffs, pixels);
  rl::TagDetection detection;
  detection.id = estimator.options().target_id;
  for (std::size_t i = 0; i < 4; ++i)
  {
    detection.corners_px[i] = cv::Point2f(pixels[i]);
  }
  return detection;
}

std::string fileText(const fs::path &path)
{
  std::ifstream stream(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(stream), {}};
}

class TemporaryTagPose : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
    directory_ = fs::temp_directory_path() / ("rov_tag_pose_test_" + std::to_string(suffix));
    ASSERT_TRUE(fs::create_directory(directory_));
  }

  void TearDown() override
  {
    // 路径由本夹具创建，核绝对父目录和固定前缀后才递归清理。
    if (directory_.parent_path() == fs::temp_directory_path() &&
        directory_.filename().string().rfind("rov_tag_pose_test_", 0) == 0)
    {
      fs::remove_all(directory_);
    }
  }

  void writeCamera(const rl::CameraModel &camera)
  {
    cv::FileStorage storage((directory_ / "camera.xml").string(), cv::FileStorage::WRITE);
    storage << "status" << camera.calibration_status << "camera_model" << "pinhole"
            << "distortion_model" << "plumb_bob";
    storage << "image_width" << camera.image_size.width << "image_height"
            << camera.image_size.height;
    storage << "camera_matrix" << camera.camera_matrix << "dist_coeffs" << camera.dist_coeffs;
  }

  // 只运行本次构建的CLI和本夹具创建的文件。无设备访问、ROS或真实电机动作。
  int runCli(const fs::path &video, const std::string &result_name, const std::string &extra = "",
             bool display = false) const
  {
    const std::string command =
        std::string("QT_QPA_PLATFORM=offscreen \"") + ROV_TAG_POSE_EXECUTABLE + "\" --video=\"" +
        video.string() + "\" --camera=\"" + (directory_ / "camera.xml").string() +
        "\" --config=\"" + ROV_TAG_POSE_CONFIG + "\" --output=\"" +
        (directory_ / result_name).string() + "\" --display=" + (display ? "true " : "false ") +
        extra + " >\"" + (directory_ / "console.log").string() + "\" 2>&1";
    return std::system(command.c_str());
  }

  fs::path directory_;
};
} // namespace

TEST(TagPose, RecoversCameraPoseByInvertingNontrivialTagTransform)
{
  const rl::TagPoseEstimator estimator(cameraModel(), rl::TagPoseOptions{});
  const cv::Vec3d rvec(0.45, -0.30, 0.20);
  const cv::Vec3d tvec(0.015, -0.010, 0.34);
  const auto result = estimator.estimate({projectTag(estimator, rvec, tvec)});
  ASSERT_TRUE(result.valid) << result.reason;
  cv::Matx33d expected_R_CL;
  cv::Rodrigues(rvec, expected_R_CL);
  const cv::Vec3d expected_position = -(expected_R_CL.t() * tvec);
  // float检测角点造成微小数值误差；1e-5m容差服务合成几何，不是真机精度承诺。
  EXPECT_LT(cv::norm(result.p_LC_m - expected_position), 1e-5);
  EXPECT_LT(cv::norm(cv::Mat(result.R_LC - expected_R_CL.t())), 1e-4);
  EXPECT_NEAR(result.height_above_plane_m, std::abs(expected_position[2]), 1e-5);
  EXPECT_LT(result.reprojection_error_px, 0.001);
  const cv::Vec3d angles = result.rpy_LC_deg * (3.14159265358979323846 / 180.0);
  cv::Matx33d rotation_x;
  cv::Matx33d rotation_y;
  cv::Matx33d rotation_z;
  cv::Rodrigues(cv::Vec3d(angles[0], 0.0, 0.0), rotation_x);
  cv::Rodrigues(cv::Vec3d(0.0, angles[1], 0.0), rotation_y);
  cv::Rodrigues(cv::Vec3d(0.0, 0.0, angles[2]), rotation_z);
  // 显示角必须按Rz*Ry*Rx还原R_LC，不能把Rodrigues三分量直接当欧拉角。
  EXPECT_LT(cv::norm(cv::Mat(rotation_z * rotation_y * rotation_x - result.R_LC)), 1e-8);
}

TEST(TagPose, HandlesFrontalViewAndReportedWetDistortionWithoutChangingIntrinsics)
{
  auto camera = cameraModel();
  // 用用户报告K/D检验强畸变数值路径；投影仍为合成，不是真实湿光路精度证据。
  camera.camera_matrix = (cv::Mat_<double>(3, 3) << 675.434436764, 0.0, 677.439238466, 0.0,
                          671.130192323, 349.137554818, 0.0, 0.0, 1.0);
  camera.dist_coeffs = (cv::Mat_<double>(1, 5) << 0.746329309562, -0.205327268155, -0.018293589416,
                        0.066863663112, -0.015357379302);
  const rl::TagPoseEstimator estimator(camera, rl::TagPoseOptions{});
  for (const auto &rvec : {cv::Vec3d(0.0, 0.0, 0.0), cv::Vec3d(0.45, -0.30, 0.20)})
  {
    const cv::Vec3d tvec(0.10, 0.025, 0.32);
    const auto result = estimator.estimate({projectTag(estimator, rvec, tvec)});
    ASSERT_TRUE(result.valid) << result.reason;
    cv::Matx33d rotation;
    cv::Rodrigues(rvec, rotation);
    EXPECT_LT(cv::norm(result.p_LC_m + rotation.t() * tvec), 1e-4);
    EXPECT_LT(result.reprojection_error_px, 0.001);
  }
  EXPECT_EQ(cv::norm(camera.camera_matrix - estimator.camera().camera_matrix), 0.0);
  EXPECT_EQ(cv::norm(camera.dist_coeffs - estimator.camera().dist_coeffs), 0.0);
}

TEST(TagPose, RecoversKnownOneHundredMillimeterCameraDisplacement)
{
  const rl::TagPoseEstimator estimator(cameraModel(), rl::TagPoseOptions{});
  const cv::Vec3d rvec(0.32, -0.18, 0.09);
  cv::Matx33d rotation;
  cv::Rodrigues(rvec, rotation);
  const cv::Vec3d first_position(-0.03, -0.08, -0.42);
  const cv::Vec3d second_position = first_position + cv::Vec3d(0.10, 0.0, 0.0);
  const auto first =
      estimator.estimate({projectTag(estimator, rvec, -(rotation * first_position))});
  const auto second =
      estimator.estimate({projectTag(estimator, rvec, -(rotation * second_position))});
  ASSERT_TRUE(first.valid) << first.reason;
  ASSERT_TRUE(second.valid) << second.reason;
  const auto displacement = second.p_LC_m - first.p_LC_m;
  EXPECT_NEAR(displacement[0], 0.100, 1e-5);
  EXPECT_NEAR(displacement[1], 0.0, 1e-5);
  EXPECT_NEAR(displacement[2], 0.0, 1e-5);
}

TEST(TagPose, DoesNotReusePoseWhenTargetDisappearsAndRejectsDuplicateId)
{
  const rl::TagPoseEstimator estimator(cameraModel(), rl::TagPoseOptions{});
  const auto detection = projectTag(estimator, cv::Vec3d(0.4, 0.2, 0.1), cv::Vec3d(0, 0, 0.35));
  ASSERT_TRUE(estimator.estimate({detection}).valid);
  const auto missing = estimator.estimate({});
  EXPECT_FALSE(missing.has_pose);
  EXPECT_FALSE(missing.valid);
  EXPECT_EQ(missing.reason, "target_not_found");
  const auto duplicate = estimator.estimate({detection, detection});
  EXPECT_FALSE(duplicate.has_pose);
  EXPECT_EQ(duplicate.reason, "duplicate_target_id");
}

TEST(TagPose, RejectsInvalidCornersHammingAndLargePixelResidual)
{
  auto options = rl::TagPoseOptions{};
  options.max_reprojection_error_px = 0.001;
  const rl::TagPoseEstimator estimator(cameraModel(), options);
  auto detection = projectTag(estimator, cv::Vec3d(0.4, 0.2, 0.1), cv::Vec3d(0, 0, 0.35));
  detection.hamming = 1;
  EXPECT_EQ(estimator.estimate({detection}).reason, "hamming_rejected");
  detection.hamming = 0;
  const auto original = detection.corners_px[0];
  detection.corners_px[0].x = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(estimator.estimate({detection}).reason, "clipped_or_invalid_corners");
  detection.corners_px[0] = original;
  detection.corners_px[0].x += 3.0F;
  const auto result = estimator.estimate({detection});
  EXPECT_TRUE(result.has_pose);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "reprojection_error");
}

TEST(TagPose, ReportsDistinctPlanarSolutionsWithSimilarResidualsAsAmbiguous)
{
  auto options = rl::TagPoseOptions{};
  // 构造宽松的相似误差选值，验证存在两个不同局部解时走歧义分支。
  // 不把该测试选值带到实际配置，也不依赖一次随机噪声采样。
  options.ambiguity_error_gap_px = 100.0;
  options.ambiguity_rotation_deg = 0.1;
  options.max_reprojection_error_px = 10.0;
  const rl::TagPoseEstimator estimator(cameraModel(), options);
  bool saw_ambiguous = false;
  for (int i = 0; i < 20; ++i)
  {
    const double phase = static_cast<double>(i);
    auto detection = projectTag(estimator, cv::Vec3d(0.12, -0.08, 0.0), cv::Vec3d(0.01, 0.0, 0.7));
    detection.corners_px[0].x += static_cast<float>(0.35 * std::sin(phase));
    detection.corners_px[1].y += static_cast<float>(0.40 * std::cos(phase));
    const auto result = estimator.estimate({detection});
    if (result.ambiguous)
    {
      EXPECT_TRUE(result.has_pose);
      EXPECT_FALSE(result.valid);
      EXPECT_EQ(result.reason, "ambiguous_pose");
      saw_ambiguous = true;
    }
  }
  EXPECT_TRUE(saw_ambiguous);
}

TEST_F(TemporaryTagPose, LoadsExportedCameraAndRejectsUnknownModelOrInvalidOptions)
{
  writeCamera(cameraModel());
  const auto loaded = rl::loadCameraModel((directory_ / "camera.xml").string());
  EXPECT_EQ(loaded.image_size, cv::Size(1280, 720));
  EXPECT_DOUBLE_EQ(loaded.camera_matrix.at<double>(0, 0), 700.0);
  EXPECT_EQ(rl::loadTagPoseOptions(ROV_TAG_POSE_CONFIG).target_id, 16);
  cv::FileStorage invalid((directory_ / "wrong.xml").string(), cv::FileStorage::WRITE);
  invalid << "camera_model" << "fisheye" << "distortion_model" << "equidistant";
  invalid.release();
  EXPECT_THROW(rl::loadCameraModel((directory_ / "wrong.xml").string()), std::invalid_argument);
  auto options = rl::TagPoseOptions{};
  options.tag_width_m = 0.0;
  EXPECT_THROW(rl::TagPoseEstimator estimator(loaded, options), std::invalid_argument);
}

TEST_F(TemporaryTagPose, ProcessesVideoLogsMissingFramesAndPreservesInputAndOldResults)
{
  auto camera = cameraModel();
  camera.dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
  writeCamera(camera);
  const rl::TagPoseEstimator estimator(camera, rl::TagPoseOptions{});
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
  const auto reference = detector.detect(marker);
  ASSERT_EQ(reference.size(), 1U);
  const auto projected = projectTag(estimator, cv::Vec3d(0.4, -0.25, 0.1), cv::Vec3d(0, 0, 0.30));
  const std::vector<cv::Point2f> source(reference[0].corners_px.begin(),
                                        reference[0].corners_px.end());
  const std::vector<cv::Point2f> destination(projected.corners_px.begin(),
                                             projected.corners_px.end());
  cv::Mat image;
  cv::warpPerspective(marker, image, cv::getPerspectiveTransform(source, destination),
                      camera.image_size, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255));
  cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
  const fs::path video = directory_ / "tag and missing.avi";
  cv::VideoWriter writer(video.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 5.0,
                         camera.image_size);
  ASSERT_TRUE(writer.isOpened());
  for (int i = 0; i < 4; ++i)
  {
    writer.write(image);
  }
  writer.write(cv::Mat(720, 1280, CV_8UC3, cv::Scalar(255, 255, 255)));
  writer.write(cv::Mat(720, 1280, CV_8UC3, cv::Scalar(255, 255, 255)));
  writer.release();
  const auto original = fileText(video);
  ASSERT_EQ(runCli(video, "result with spaces"), 0) << fileText(directory_ / "console.log");
  cv::FileStorage info((directory_ / "result with spaces/run_info.yaml").string(),
                       cv::FileStorage::READ);
  EXPECT_EQ(static_cast<double>(info["frames"]), 6.0);
  EXPECT_EQ(static_cast<double>(info["valid_frames"]), 4.0);
  const auto csv = fileText(directory_ / "result with spaces/pose.csv");
  // 最后两帧明确丢检且位置nan；它们不能复用前四帧位置。
  EXPECT_NE(csv.find("4,0.8,16,0,0,0,target_not_found,nan,nan,nan,nan"), std::string::npos);
  EXPECT_EQ(fileText(video), original);
  EXPECT_NE(runCli(video, "result with spaces"), 0);
  EXPECT_EQ(fileText(directory_ / "result with spaces/pose.csv"), csv);
  EXPECT_NE(runCli(video, "wrong_id", "--tag-id=15"), 0);
  cv::FileStorage missing((directory_ / "wrong_id/run_info.yaml").string(), cv::FileStorage::READ);
  EXPECT_EQ(static_cast<double>(missing["valid_frames"]), 0.0);
  EXPECT_EQ(static_cast<int>(missing["target_id"]), 15);
  // offscreen仅验证显示/画轴/waitKey代码，不代表实机桌面或WSLg显示验收。
  ASSERT_EQ(runCli(video, "display_smoke", "--max-frames=2", true), 0)
      << fileText(directory_ / "console.log");
  cv::FileStorage displayed((directory_ / "display_smoke/run_info.yaml").string(),
                            cv::FileStorage::READ);
  EXPECT_EQ(static_cast<double>(displayed["frames"]), 2.0);
  EXPECT_EQ(static_cast<std::string>(displayed["stop_reason"]), "max_frames");
}

TEST_F(TemporaryTagPose, RejectsVideoResolutionMismatchBeforeWritingResults)
{
  writeCamera(cameraModel());
  const fs::path video = directory_ / "wrong_size.avi";
  cv::VideoWriter writer(video.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 5.0,
                         cv::Size(640, 480));
  ASSERT_TRUE(writer.isOpened());
  writer.write(cv::Mat(480, 640, CV_8UC3, cv::Scalar(255, 255, 255)));
  writer.release();
  EXPECT_NE(runCli(video, "wrong_size"), 0);
  EXPECT_FALSE(fs::exists(directory_ / "wrong_size"));
  EXPECT_NE(fileText(directory_ / "console.log").find("尺寸与相机XML不一致"), std::string::npos);
}
