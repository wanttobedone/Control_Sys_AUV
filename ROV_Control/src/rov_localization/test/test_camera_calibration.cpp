#include "rov_localization/apriltag_detector.h"
#include "rov_localization/camera_calibrator.h"

#include <chrono>
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
rl::CalibrationOptions options()
{
  return rl::loadOptions(ROV_CALIBRATION_CONFIG);
}

cv::Mat knownCamera()
{
  return (cv::Mat_<double>(3, 3) << 700.0, 0.0, 640.0, 0.0, 690.0, 360.0, 0.0, 0.0, 1.0);
}

// 不同时刻覆盖不同倾角、中心和距离。参数是真实已知的测试输入，不是相机标定值。
void testPose(int index, cv::Vec3d &rvec, cv::Vec3d &tvec)
{
  const double phase = static_cast<double>(index);
  rvec = cv::Vec3d(0.45 * std::sin(phase * 0.73), 0.40 * std::cos(phase * 0.91),
                   0.15 * std::sin(phase * 0.33));
  tvec = cv::Vec3d(0.17 * std::sin(phase * 0.43), 0.09 * std::cos(phase * 0.57),
                   0.36 + 0.07 * std::sin(phase * 0.21));
}

rl::TagDetection projectTag(const rl::CameraCalibrator &calibrator, const cv::Mat &distortion,
                            int index)
{
  cv::Vec3d rvec;
  cv::Vec3d tvec;
  testPose(index, rvec, tvec);
  std::vector<cv::Point2f> pixels;
  cv::projectPoints(calibrator.makeObjectPoints(), rvec, tvec, knownCamera(), distortion, pixels);
  rl::TagDetection detection;
  detection.id = 7;
  std::copy(pixels.begin(), pixels.end(), detection.corners_px.begin());
  return detection;
}

// 对应点测试仅校验几何求解，灰度纹理用于通过清晰度检查，不声称检测真实水下图像。
std::vector<rl::CalibrationView> syntheticViews(const rl::CameraCalibrator &calibrator,
                                                const cv::Mat &distortion)
{
  cv::Mat texture(720, 1280, CV_8UC1);
  cv::RNG rng(42);
  rng.fill(texture, cv::RNG::UNIFORM, 0, 256);
  std::vector<rl::CalibrationView> views;
  for (int i = 0; i < 90; ++i)
  {
    rl::CalibrationView view;
    std::string reason;
    const auto detection = projectTag(calibrator, distortion, i);
    if (calibrator.describeView(texture, detection, view, reason))
    {
      view.frame_index = i * 60;
      view.video_time_s = i * 2.0;
      views.push_back(view);
    }
  }
  return views;
}

cv::Mat renderedTag()
{
  apriltag_family_t *family = tag36h11_create();
  image_u8_t *raw = apriltag_to_image(family, 7);
  const cv::Mat small(raw->height, raw->width, CV_8UC1, raw->buf,
                      static_cast<std::size_t>(raw->stride));
  cv::Mat enlarged;
  cv::resize(small, enlarged, cv::Size(), 20.0, 20.0, cv::INTER_NEAREST);
  image_u8_destroy(raw);
  tag36h11_destroy(family);
  cv::Mat with_margin;
  cv::copyMakeBorder(enlarged, with_margin, 80, 80, 80, 80, cv::BORDER_CONSTANT, cv::Scalar(255));
  return with_margin;
}

class TemporaryCalibration : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
    directory_ = fs::temp_directory_path() / ("rov_calibration_test_" + std::to_string(suffix));
    ASSERT_TRUE(fs::create_directory(directory_));
  }

  void TearDown() override
  {
    // 只清理由本用例新建且名称/父目录都核对过的路径，不删除用户录像或结果。
    if (directory_.parent_path() == fs::temp_directory_path() &&
        directory_.filename().string().rfind("rov_calibration_test_", 0) == 0)
    {
      fs::remove_all(directory_);
    }
  }
  fs::path directory_;
};
} // namespace

TEST(CameraCalibration, UsesMeasuredBlackSquareAndCanonicalCornerOrder)
{
  const rl::CameraCalibrator calibrator(options());
  const auto points = calibrator.makeObjectPoints();
  ASSERT_EQ(points.size(), 4U);
  // 80mm黑框的一半是0.040m，不是100mm成品白边的一半；p序来自官方H定义。
  EXPECT_FLOAT_EQ(points[0].x, -0.040F);
  EXPECT_FLOAT_EQ(points[0].y, 0.040F);
  EXPECT_FLOAT_EQ(points[1].x, 0.040F);
  EXPECT_FLOAT_EQ(points[2].y, -0.040F);
  EXPECT_FLOAT_EQ(points[3].x, -0.040F);
}

TEST(CameraCalibration, RejectsUnknownModelAndInvalidConfiguration)
{
  auto config = options();
  config.camera_model = "fisheye";
  EXPECT_THROW(rl::CameraCalibrator calibrator(config), std::runtime_error);
  config = options();
  config.tag_width_m = 0.0;
  EXPECT_THROW(rl::CameraCalibrator calibrator(config), std::runtime_error);
}

TEST(CameraCalibration, KeepsIdAndCanonicalCornersUnderFourImageRotations)
{
  rl::AprilTagDetector detector;
  detector.configure(1, 0);
  cv::Mat image = renderedTag();
  const auto reference = detector.detect(image);
  ASSERT_EQ(reference.size(), 1U);
  ASSERT_EQ(reference[0].id, 7);
  auto expected = reference[0].corners_px;
  for (int rotation = 0; rotation < 4; ++rotation)
  {
    const auto detected = detector.detect(image);
    ASSERT_EQ(detected.size(), 1U);
    EXPECT_EQ(detected[0].id, 7);
    for (std::size_t corner = 0; corner < 4; ++corner)
    {
      // 90度顺时针像素变换：(x,y)→(H-1-y,x)。允许2px检测细化误差。
      EXPECT_LT(cv::norm(expected[corner] - detected[0].corners_px[corner]), 2.0);
      expected[corner] =
          cv::Point2f(static_cast<float>(image.rows - 1) - expected[corner].y, expected[corner].x);
    }
    cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);
  }
}

TEST(CameraCalibration, RejectsClippedCornersAndBlur)
{
  const rl::CameraCalibrator calibrator(options());
  cv::Mat gray(720, 1280, CV_8UC1, cv::Scalar(128));
  auto detection = projectTag(calibrator, cv::Mat::zeros(5, 1, CV_64F), 2);
  rl::CalibrationView view;
  std::string reason;
  EXPECT_FALSE(calibrator.describeView(gray, detection, view, reason));
  EXPECT_EQ(reason, "blur");
  detection.corners_px[0].x = -1.0F;
  EXPECT_FALSE(calibrator.describeView(gray, detection, view, reason));
  EXPECT_EQ(reason, "clipped_or_invalid_corners");
}

TEST(CameraCalibration, DoesNotTreatRepeatedStaticFramesAsManyViews)
{
  const rl::CameraCalibrator calibrator(options());
  const auto varied = syntheticViews(calibrator, cv::Mat::zeros(5, 1, CV_64F));
  ASSERT_FALSE(varied.empty());
  std::vector<rl::CalibrationView> repeated(100, varied.front());
  for (std::size_t i = 0; i < repeated.size(); ++i)
  {
    repeated[i].frame_index = static_cast<std::int64_t>(i * 60);
    repeated[i].video_time_s = static_cast<double>(i) * 2.0;
  }
  rl::CalibrationResult result;
  result.image_size = cv::Size(1280, 720);
  EXPECT_THROW(calibrator.selectViews(repeated, result), std::runtime_error);
}

TEST(CameraCalibration, RecoversKnownCameraFromVariedPlanarProjections)
{
  const rl::CameraCalibrator calibrator(options());
  const cv::Mat distortion = (cv::Mat_<double>(5, 1) << -0.06, 0.015, 0.001, -0.0005, 0.0);
  const auto views = syntheticViews(calibrator, distortion);
  rl::CalibrationResult result;
  result.image_size = cv::Size(1280, 720);
  calibrator.selectViews(views, result);
  calibrator.solveIntrinsics(result);
  calibrator.evaluateResult(result);
  // 无噪声float投影，1px内参容差大于浮点数值误差，不代表真机允许1px定位误差。
  EXPECT_NEAR(result.camera_matrix.at<double>(0, 0), 700.0, 1.0);
  EXPECT_NEAR(result.camera_matrix.at<double>(1, 1), 690.0, 1.0);
  EXPECT_NEAR(result.camera_matrix.at<double>(0, 2), 640.0, 1.0);
  EXPECT_NEAR(result.camera_matrix.at<double>(1, 2), 360.0, 1.0);
  EXPECT_LT(result.rms_px, 0.01);
  EXPECT_LT(result.holdout_rms_px, 0.01);
}

TEST_F(TemporaryCalibration, EmptyDetectionVideoReportsFailureWithoutFakeParameters)
{
  const auto input = directory_ / "blank.avi";
  cv::VideoWriter writer(input.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 5.0,
                         cv::Size(640, 480));
  ASSERT_TRUE(writer.isOpened());
  for (int frame = 0; frame < 15; ++frame)
  {
    writer.write(cv::Mat(480, 640, CV_8UC3, cv::Scalar(255, 255, 255)));
  }
  writer.release();
  rl::CameraCalibrator calibrator(options());
  const auto output = directory_ / "failed";
  const auto result = calibrator.run(input.string(), output.string());
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(fs::exists(output / "calibration_report.yaml"));
  EXPECT_FALSE(fs::exists(output / "camera_wet.xml"));
  EXPECT_THROW(calibrator.run(input.string(), output.string()), std::runtime_error);
}

TEST_F(TemporaryCalibration, MetadataFrameCountDifferenceDoesNotAbortSampleSelection)
{
  const auto input = directory_ / "count_difference.avi";
  cv::VideoWriter writer(input.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 5.0,
                         cv::Size(640, 480));
  ASSERT_TRUE(writer.isOpened());
  constexpr int decoded_count = 15;
  constexpr int reported_count = 19;
  for (int i = 0; i < decoded_count; ++i)
  {
    writer.write(cv::Mat(480, 640, CV_8UC3, cv::Scalar(255, 255, 255)));
  }
  writer.release();

  // 仅修改本用例临时AVI的元数据，制造“实际15帧、报告19帧”的可重复输入。
  // AVI avih.dwTotalFrames位于chunk数据偏移16；strh.dwLength位于数据偏移32。
  // 每个chunk头为8字节，两个字段均是uint32小端；实际MJPEG图像不增删。
  std::ifstream original(input, std::ios::binary);
  std::string bytes{std::istreambuf_iterator<char>(original), {}};
  original.close();
  const auto main_header = bytes.find("avih");
  const auto stream_header = bytes.find("strh");
  ASSERT_NE(main_header, std::string::npos);
  ASSERT_NE(stream_header, std::string::npos);
  for (const auto offset : {main_header + 24, stream_header + 40})
  {
    ASSERT_LT(offset + 3, bytes.size());
    for (unsigned int byte = 0; byte < 4; ++byte)
    {
      bytes[offset + byte] = static_cast<char>((reported_count >> (8U * byte)) & 0xff);
    }
  }
  std::ofstream modified(input, std::ios::binary | std::ios::trunc);
  modified.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  modified.close();
  ASSERT_FALSE(modified.fail());
  cv::VideoCapture metadata(input.string(), cv::CAP_FFMPEG);
  ASSERT_TRUE(metadata.isOpened());
  ASSERT_EQ(metadata.get(cv::CAP_PROP_FRAME_COUNT), reported_count);
  metadata.release();

  rl::CameraCalibrator calibrator(options());
  const auto output = directory_ / "count_report";
  const auto result = calibrator.run(input.string(), output.string());
  // 白图本来无标签，应在选样阶段失败；不能再错误归因为帧数差异。
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("没有单个ID"), std::string::npos);
  cv::FileStorage report((output / "calibration_report.yaml").string(), cv::FileStorage::READ);
  EXPECT_EQ(static_cast<double>(report["decoded_frames"]), decoded_count);
  EXPECT_EQ(static_cast<double>(report["frame_count_difference"]), -4.0);
  EXPECT_FALSE(static_cast<std::string>(report["frame_count_warning"]).empty());
  EXPECT_FALSE(fs::exists(output / "camera_wet.xml"));
}

TEST_F(TemporaryCalibration, ProcessesRenderedTagVideoAndPreservesOriginalAndExistingResults)
{
  auto config = options();
  // 合成录像用于软件通路，减少抽样重复过滤，所有几何/误差检查仍使用原选值。
  config.sample_period_s = 0.2;
  const rl::CameraCalibrator geometry(config);
  rl::AprilTagDetector detector;
  detector.configure(1, 0);
  const cv::Mat marker = renderedTag();
  const auto reference = detector.detect(marker);
  ASSERT_EQ(reference.size(), 1U);
  const std::vector<cv::Point2f> source(reference[0].corners_px.begin(),
                                        reference[0].corners_px.end());
  const auto input = directory_ / "tag_views.avi";
  // 1fps使各帧属于不同时间组；实际程序仍按文件fps，不假定用户录像都是30fps。
  cv::VideoWriter writer(input.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 1.0,
                         cv::Size(1280, 720));
  ASSERT_TRUE(writer.isOpened());
  for (int i = 0; i < 90; ++i)
  {
    const auto detection = projectTag(geometry, cv::Mat::zeros(5, 1, CV_64F), i);
    const std::vector<cv::Point2f> destination(detection.corners_px.begin(),
                                               detection.corners_px.end());
    const cv::Mat homography = cv::getPerspectiveTransform(source, destination);
    cv::Mat gray;
    cv::warpPerspective(marker, gray, homography, cv::Size(1280, 720), cv::INTER_LINEAR,
                        cv::BORDER_CONSTANT, cv::Scalar(255));
    cv::Mat color;
    cv::cvtColor(gray, color, cv::COLOR_GRAY2BGR);
    writer.write(color);
  }
  writer.release();
  std::ifstream before_stream(input, std::ios::binary);
  const std::string before{std::istreambuf_iterator<char>(before_stream), {}};
  rl::CameraCalibrator calibrator(config);
  const auto output = directory_ / "results";
  const auto result = calibrator.run(input.string(), output.string());
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.target_id, 7);
  EXPECT_TRUE(fs::exists(output / "camera_wet.xml"));
  EXPECT_TRUE(fs::exists(output / "selected_views.csv"));
  cv::FileStorage parameters((output / "camera_wet.xml").string(), cv::FileStorage::READ);
  EXPECT_EQ(static_cast<int>(parameters["image_width"]), 1280);
  EXPECT_EQ(static_cast<std::string>(parameters["status"]), "candidate_requires_metric_validation");
  // 检测/图像重采样引入亚像素误差，允许10px焦距偏差；这是软件合成测试容差。
  EXPECT_NEAR(result.camera_matrix.at<double>(0, 0), 700.0, 10.0);
  EXPECT_THROW(calibrator.run(input.string(), output.string()), std::runtime_error);
  std::ifstream after_stream(input, std::ios::binary);
  const std::string after{std::istreambuf_iterator<char>(after_stream), {}};
  EXPECT_EQ(before, after);
}
