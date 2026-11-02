#pragma once

#include "rov_localization/vision_types.h"

namespace rov_localization
{
// 读取完整的实验选值；缺字段/错模型直接报具体原因，不静默使用另一套上限。
CalibrationOptions loadOptions(const std::string &path);
void validateOptions(const CalibrationOptions &options);

// 单次离线任务。run拥有全部样本和结果，按扫描→选ID→选帧→求解→检查→保存调用。
// 纯C++接口不依赖ROS，也不发布任何执行器或位姿消息。
class CameraCalibrator
{
public:
  explicit CameraCalibrator(const CalibrationOptions &options);
  CalibrationResult run(const std::string &video_path, const std::string &output_dir);

  // 几何/求解接口公开供正式测试及未来已知目标输入复用，不含视频或硬件依赖。
  std::vector<cv::Point3f> makeObjectPoints() const;
  bool describeView(const cv::Mat &gray, const TagDetection &detection, CalibrationView &view,
                    std::string &reason) const;
  void selectViews(const std::vector<CalibrationView> &candidates, CalibrationResult &result) const;
  void solveIntrinsics(CalibrationResult &result) const;
  void evaluateResult(CalibrationResult &result) const;

private:
  void scanVideo(const std::string &path, CalibrationResult &result);
  int selectTargetId(const cv::Size &image_size) const;
  void saveResult(const std::string &video_path, const std::string &output_dir,
                  const CalibrationResult &result) const;
  double viewDistance(const CalibrationView &first, const CalibrationView &second) const;
  double checkFocalStability(const CalibrationResult &result) const;

  CalibrationOptions options_;
  ScanStatistics statistics_;
  std::map<int, std::vector<CalibrationView>> candidates_;
};
} // namespace rov_localization
