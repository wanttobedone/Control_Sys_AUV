#pragma once

#include "rov_localization/vision_types.h"

// 原生类型仅在此声明，调用者不需要管理C库资源。
struct apriltag_detector;
struct apriltag_family;

namespace rov_localization
{
// 只封装原生检测，不重写解码/优化。离线工具直接使用原始灰度图，不预先去畸变。
class AprilTagDetector
{
public:
  AprilTagDetector() = default;
  ~AprilTagDetector();
  AprilTagDetector(const AprilTagDetector &) = delete;
  AprilTagDetector &operator=(const AprilTagDetector &) = delete;

  void configure(int threads, int max_hamming);
  std::vector<TagDetection> detect(const cv::Mat &gray);

private:
  void release();
  apriltag_detector *detector_{nullptr};
  apriltag_family *family_{nullptr};
};
} // namespace rov_localization
